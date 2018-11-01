#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include "../staging/android/timed_output.h"
#include <linux/err.h>
#include <linux/of_gpio.h>

#define DEBUG

struct vibrator_drvdata {
	struct platform_device *pdev;
	struct hrtimer timer;
	struct timed_output_dev dev;
	struct work_struct work;
	struct mutex lock;
	struct pwm_device *pwm;
	int max_timeout;
	int timeout;
	bool running;
	bool vibrate;

	int vib_gpio;
};

#define	VIBRATOR_PWR_ON		0x1
#define VIBRATOR_PWR_OFF	0x0
#define VIBRATOR_MIN_TIMEOUT	70

static void vibrator_set(struct vibrator_drvdata *dev_dat, int enable)
{
	if (enable) {
#ifdef DEBUG
		pr_err("[VIB]: %s: enable\n", __func__);
#endif
		gpio_set_value(dev_dat->vib_gpio, 1);
	} else {
#ifdef DEBUG
		pr_err("[VIB]: %s: disable\n", __func__);
#endif
		gpio_set_value(dev_dat->vib_gpio, 0);
	}
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *_timer)
{
	struct vibrator_drvdata *dev_dat =
		container_of(_timer, struct vibrator_drvdata, timer);

	dev_dat->timeout = 0;

	schedule_work(&dev_dat->work);
	return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *_work)
{
	struct vibrator_drvdata *dev_dat =
		container_of(_work, struct vibrator_drvdata, work);

	if (0 == dev_dat->timeout) {
		if (!dev_dat->running)
			return ;

		dev_dat->running = false;
		vibrator_set(dev_dat, false);
	} else {
		if (dev_dat->running)
			return ;

		dev_dat->running = true;
		vibrator_set(dev_dat, true);
	}
}

static int vibrator_get_time(struct timed_output_dev *_dev)
{
	struct vibrator_drvdata *dev_dat =
		container_of(_dev, struct vibrator_drvdata, dev);

	if (hrtimer_active(&dev_dat->timer)) {
		ktime_t r = hrtimer_get_remaining(&dev_dat->timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void vibrator_enable(struct timed_output_dev *_dev, int timeout)
{
	struct vibrator_drvdata *dev_dat =
		container_of(_dev, struct vibrator_drvdata, dev);

#ifdef DEBUG
	printk(KERN_WARNING "[VIB] time = %dms\n", timeout);
#endif

	mutex_lock(&dev_dat->lock);
	cancel_work_sync(&dev_dat->work);
	hrtimer_cancel(&dev_dat->timer);
	dev_dat->timeout = timeout;
	schedule_work(&dev_dat->work);
	if (timeout > 0) {
		if (timeout > dev_dat->max_timeout)
			timeout = dev_dat->max_timeout;

		if (timeout < VIBRATOR_MIN_TIMEOUT)
			timeout = VIBRATOR_MIN_TIMEOUT;

		hrtimer_start(&dev_dat->timer,
			ns_to_ktime((u64)timeout * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
	}
	mutex_unlock(&dev_dat->lock);
}

#ifdef CONFIG_OF
static int vibrator_parse_dt(struct device *dev,
							struct vibrator_drvdata *dev_dat)
{
	struct device_node *np = dev->of_node;
	int rc = 0;

	rc = of_property_read_u32(np, "max-timeout", &dev_dat->max_timeout);
	if (rc) {
		dev_err(dev, "Unable to read max timeout\n");
		return rc;
	}

	dev_dat->vib_gpio = of_get_gpio(np, 0);
	rc = gpio_request(dev_dat->vib_gpio, "vibrator-out");
	if(rc) {
		dev_err(dev, "Failed to request vib_gpio\n");
		return rc;
	}

	return 0;
}
#endif

static int vibrator_probe(struct platform_device *pdev)
{
	struct vibrator_drvdata *dev_dat;
	int ret;

	printk(KERN_WARNING "%s: %s registering\n", __func__, pdev->name);

	dev_dat = kzalloc(sizeof(*dev_dat), GFP_KERNEL);
	if (!dev_dat)
		return -ENOMEM;

	ret = vibrator_parse_dt(&pdev->dev, dev_dat);
	if (ret) {
		dev_err(&pdev->dev, "Parsing DT failed(%d)", ret);
		return ret;
	}

	dev_dat->pdev = pdev;
	dev_dat->vibrate = false;

	mutex_init(&dev_dat->lock);

	hrtimer_init(&dev_dat->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev_dat->timer.function = vibrator_timer_func;

	/* register with timed output class */
	dev_dat->dev.name = "vibrator";
	dev_dat->dev.get_time = vibrator_get_time;
	dev_dat->dev.enable = vibrator_enable;
	ret = timed_output_dev_register(&dev_dat->dev);
	if (ret < 0) {
		pr_err("timed output register failed %d\n", ret);
		goto setup_fail;
	}

	INIT_WORK(&dev_dat->work, vibrator_work);

	pr_debug("%s registered\n", pdev->name);
	return 0;

setup_fail:
	mutex_destroy(&dev_dat->lock);
	kfree(dev_dat);
	return ret;
}

static int vibrator_remove(struct platform_device *pdev)
{
	struct vibrator_drvdata *dev_dat = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	timed_output_dev_unregister(&dev_dat->dev);
	hrtimer_cancel(&dev_dat->timer);
	mutex_destroy(&dev_dat->lock);

	kfree(dev_dat);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vibrator_match[] = {
	{ .compatible = "h532b,vibrator", },
	{ },
};
#endif

static struct platform_driver vibrator_driver = {
	.probe	= vibrator_probe,
	.remove = vibrator_remove,
	.driver = {
		.name	= "vibrator",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(vibrator_match),
#endif
	},
};

static int __init vibrator_init(void)
{
	printk(KERN_WARNING "%s: %s\n", __func__, vibrator_driver.driver.name);
	return platform_driver_register(&vibrator_driver);
}

static void __exit vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}

module_init(vibrator_init);
module_exit(vibrator_exit);

MODULE_ALIAS("platform:h532b-vib");
MODULE_DESCRIPTION("h532b Vibrator Driver");
MODULE_LICENSE("GPL");
