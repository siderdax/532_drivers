#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include "../staging/android/timed_output.h"
#include <linux/cdev.h>

#if 0
#define DBG_PRINTK(fmt, args...) printk("\t\tBUZZER %s: " fmt "\r\n", __func__, ##args)
#else
#define DBG_PRINTK(ARGS...)		do {} while(0);
#endif

/* I/O Control Commands */
#define H532B_BUZZ_OUT		_IOW('C', 0x01, buzz_data)
// #define H532B_BUZZ_DATA		_IOW('C', 0x02, buzz_data)

/* PWM en/disable */
#define ENABLE 			1
#define DISABLE 		0

/* Character device */
#define DEVICE_NAME "h532bbuzzer"
#define CLASS_NAME	"ch532bbuzzer"

unsigned int buzzer_major = 0;
static struct class *buzzer_class;

/* Defines for ioctl */

typedef struct
{
	unsigned int period;
	unsigned int duty;
	unsigned int duration;
}__attribute__((packed))buzz_data;

// Global Vars
unsigned int gPWM_period = 1000000;
unsigned int gPWM_duty = 50;
unsigned int gBuzz_duration = 20;
struct work_struct *gWork;
// --------

struct buzzer_pwm_data
{
	struct platform_device *pdev;
	struct pwm_device *pwm;
	struct work_struct work;
	int pwm_channel;
};

// static int buzzer_get_time()
// {
// 	if (hrtimer_active(&ddata->timer)) {
// 		ktime_t r = hrtimer_get_remaining(&ddata->timer);
// 		struct timeval t = ktime_to_timeval(r);
// 		return t.tv_sec * 1000 + t.tv_usec / 1000;
// 	} else
// 		return 0;
// }

static int buzz_out(struct buzzer_pwm_data *bpdata, int en)
{
	int ret = 0;
	u32 duty_ns = 0;

	DBG_PRINTK("Buzzing / %d", en);

	switch(en) {
		case DISABLE:
			pwm_disable(bpdata->pwm);
			break;
		case ENABLE:
			duty_ns = (u32)(gPWM_period * gPWM_duty / 100);
			ret = pwm_config(bpdata->pwm, duty_ns, gPWM_period);
			if(!ret)
				ret = pwm_enable(bpdata->pwm);
			break;
	}

	DBG_PRINTK("return = %d", ret);
	return ret;
}

static void buzzer_work(struct work_struct *_work)
{
	struct buzzer_pwm_data *bpdata = container_of(_work, struct buzzer_pwm_data, 
		work);

	switch(gBuzz_duration) {
		case 0:
			buzz_out(bpdata, DISABLE);
			break;
		case 1:
			buzz_out(bpdata, ENABLE);
			break;
		default:
			buzz_out(bpdata, ENABLE);
			mdelay(gBuzz_duration);
			buzz_out(bpdata, DISABLE);
	}
}

static long buzzer_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret=0;
	buzz_data buzzdata;

	DBG_PRINTK("called");
	
	switch (cmd) {
	case H532B_BUZZ_OUT:
		ret = copy_from_user( (void *)&buzzdata, (void *)arg, sizeof(buzz_data));
		gPWM_period = buzzdata.period;
		gPWM_duty = buzzdata.duty;
		gBuzz_duration = buzzdata.duration;
		schedule_work(gWork);
		break;
	}
	
	return ret;
}

/* Character device file operations */
static const struct file_operations buzzer_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= buzzer_ioctl,
	.llseek		= no_llseek,
};

int buzzer_of_dts(struct buzzer_pwm_data *bpdata)
{
	struct device_node *node = bpdata->pdev->dev.of_node;

	// Get data PWM Channel
	if (of_property_read_u32(node, "pwm_channel", &bpdata->pwm_channel)) {
		DBG_PRINTK("failed to get PWM channel");
		return -1;
	}

	return 0;
}

static int h532b_buzzer_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct buzzer_pwm_data *bpdata;

	DBG_PRINTK("_");

	bpdata = kzalloc(sizeof(struct buzzer_pwm_data), GFP_KERNEL);
	if(!bpdata)
		ret = -ENOMEM;

	bpdata->pdev = pdev;

	ret = buzzer_of_dts(bpdata);
	if(ret != 0)
		goto err;

	DBG_PRINTK("pwm_channel = %d, %s", bpdata->pwm_channel, pdev->name);
	bpdata->pwm = pwm_request(bpdata->pwm_channel, pdev->name);
	if (IS_ERR(bpdata->pwm)) {
		pr_err("pwm request failed\n");
		ret = PTR_ERR(bpdata->pwm);
		goto err;
	}

	INIT_WORK(&bpdata->work, buzzer_work);
	gWork = &bpdata->work;

	/* BEEEEEEPP!!! */
	schedule_work(&bpdata->work);

	return 0;
err:
	kfree(bpdata);
	return ret;
}

static int h532b_buzzer_remove(struct platform_device *pdev)
{
	struct buzzer_pwm_data *bpdata = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	if(bpdata != NULL) {
		pwm_free(bpdata->pwm);
		kfree(bpdata);
	}
	return 0;
}

static const struct of_device_id of_h532b_buzzer_match[] = {
	{ .compatible = "h532b-buzzer", },
	{},
};
MODULE_DEVICE_TABLE(of, of_h532b_buzzer_match);

static struct platform_driver led_pwm_driver = {
	.probe		= h532b_buzzer_probe,
	.remove		= h532b_buzzer_remove,
	.driver		= {
		.name	= "h532b_buzzer",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_h532b_buzzer_match),
	},
};

module_platform_driver(led_pwm_driver);

static int __init buzzer_init(void)
{
	struct device *dev = NULL;
	int ret = 0;
		
	DBG_PRINTK("++");

	/* init CHDEV */
	buzzer_major = register_chrdev(0, DEVICE_NAME, &buzzer_fops);
	if (buzzer_major < 0){
		DBG_PRINTK("buzzer failed to register a major number");
		return buzzer_major;
	}
	DBG_PRINTK("buzzer : registered correctly with major number %d",
		buzzer_major);

	buzzer_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(buzzer_class)) {
		DBG_PRINTK("Failed to create the device -- buzzer");
		unregister_chrdev(buzzer_major, DEVICE_NAME);
		ret = PTR_ERR(buzzer_class);
		goto err;
	}
	DBG_PRINTK("buzzer: device class created correctly");
	
	dev = device_create(buzzer_class, NULL, MKDEV(buzzer_major, 0), NULL,
		DEVICE_NAME);
	if(!dev) {
		DBG_PRINTK("h532blcd_init: device_create fail");
		goto err;		
	}
	DBG_PRINTK("buzzer: device class created correctly");
	/* init CHDEV END*/

	DBG_PRINTK("-- ret:%d", ret);
	return 0;
err:
	DBG_PRINTK("error:0x%X", ret);
	return ret;	
}

static void __exit buzzer_exit(void)
{
	device_destroy(buzzer_class, MKDEV(buzzer_major, 0));
  	class_unregister(buzzer_class);
	class_destroy(buzzer_class);
	unregister_chrdev(buzzer_major, DEVICE_NAME);
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_AUTHOR("YIKIM");
MODULE_DESCRIPTION("H532B buzzer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:h532b_buzzer");
