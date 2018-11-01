#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/pwm.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>


/* Global vars for initialization of Character device */
unsigned int irda_major = 0;
static struct class *irda_class;
#define DEVICE_NAME "h_irda"
#define CLASS_NAME	"h_irda_class"

#define USE_RX
#define DATA_CAPTURE_LENGTH 	128

#define C_PERIOD			26		/* Carrier period */
#define ONE_NS 				1000000000
#define HALF_NS 			500000000

#ifdef USE_RX
static DEFINE_MUTEX(irda_rx_mutex);
struct irda_rx_data {
	int 		irda_rx;
	int 		data[DATA_CAPTURE_LENGTH];
	int 		data_idx;

	struct 		hrtimer timer;
	ktime_t 	period;
};
#endif

#define MAX_PATTERN_LEN		128
struct pattern_data {
    int freq;
    int pattern[MAX_PATTERN_LEN];
};

static DEFINE_MUTEX(irda_tx_mutex);
struct irda_tx_data {
	int 			irda_tx;

	struct pattern_data pt_dat;
	int 				ptd_cnt;

	u32 				pwm_ch;
	struct pwm_device	*pwm;
};

/* Global vars */
struct irda_tx_data *gTx_data;

long gCount, gTod;
int gCntidx;
struct timespec ts;

/* 
 * Receiving data 
 */
#ifdef USE_RX
static enum hrtimer_restart irda_rx_hrtimer(struct hrtimer *timer)
{
	struct irda_rx_data *rx_data;
	int i;
	rx_data = container_of(timer, struct irda_rx_data, timer);

	mutex_lock(&irda_rx_mutex);
	rx_data->data[rx_data->data_idx]++;
	if(rx_data->data[rx_data->data_idx] > 2000 && rx_data->data_idx != 0) {
		printk("IRDA : Stop capturing\r\n");
		printk("data = { ");
		for(i = 0 ; i < DATA_CAPTURE_LENGTH; i++) {
			if(rx_data->data[i] == 0) break;
			printk("%d, ", rx_data->data[i]);
		}
		printk(" };\n");
		return HRTIMER_NORESTART;
	}
	mutex_unlock(&irda_rx_mutex);

	hrtimer_forward_now(&rx_data->timer, rx_data->period);
	return HRTIMER_RESTART;
}

static irqreturn_t irdarx_irq(int irq, void *dev_id)
{
	// struct irda_rx_data *rx_data = (struct irda_rx_data *)dev_id;
	long Tod;

	__getnstimeofday(&ts);
	Tod = ts.tv_nsec;
	gCount = Tod - gTod;
	printk("gCount = %ld\n", gCount);
	__getnstimeofday(&ts);
	gTod = ts.tv_nsec;

	// if(!hrtimer_active(&rx_data->timer))
	// 	return IRQ_HANDLED;
	// else
	// 	rx_data->data_idx = (rx_data->data_idx + 1) % DATA_CAPTURE_LENGTH;

	return IRQ_HANDLED;
}

static irqreturn_t irdakey1_irq(int irq, void *dev_id)
{
	// struct irda_rx_data *rx_data = (struct irda_rx_data *)dev_id;

	/* Saving Data */
	// if(!hrtimer_active(&rx_data->timer)){
	// 	printk("IRDA : Capturing IR data\r\n");
	// 	rx_data->data_idx = 0;
	// 	memset(rx_data->data, 0, sizeof(rx_data->data));
	// 	hrtimer_start(&rx_data->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	// }
	// else {
	// 	printk("IRDA : Already started\r\n");
	// }

	if(gTx_data != NULL) {
		pwm_config(gTx_data->pwm, (/*HALF*/ONE_NS / 38000), ONE_NS / 38000);
	}

	return IRQ_HANDLED;
}

static irqreturn_t irdakey2_irq(int irq, void *dev_id)
{
	// struct irda_rx_data *rx_data = (struct irda_rx_data *)dev_id;
	// int i;

	// printk("data = { ");
	// for(i = 0 ; i < DATA_CAPTURE_LENGTH; i++) {
	// 	if(rx_data->data[i] == 0) break;
	// 	printk("%d, ", rx_data->data[i]);
	// }
	// printk(" };\n");

	printk("gCount = %ld\n", gCount);

	return IRQ_HANDLED;
}
#endif

/*
 * Use this function to send data
 * irda_tx_data.data : the value of signal data
 */
void sendTxData(struct irda_tx_data *tx_data) {
	int p_idx = 0;
	unsigned long flags;

	local_irq_save(flags);
	while(p_idx < tx_data->ptd_cnt) {
		/* Career frequency */
		if(tx_data->pt_dat.pattern[p_idx] < 0) break;
		pwm_config(tx_data->pwm,
			(HALF_NS / tx_data->pt_dat.freq) * ((p_idx + 1) % 2),
			ONE_NS / tx_data->pt_dat.freq);
		udelay(tx_data->pt_dat.pattern[p_idx]);
		p_idx++;
	}
	pwm_config(tx_data->pwm, 0,	ONE_NS / tx_data->pt_dat.freq);
	local_irq_restore(flags);
}

static ssize_t irda_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	mutex_lock(&irda_tx_mutex);
	if (copy_from_user(&gTx_data->pt_dat, buffer, count))
		return -EFAULT;
	gTx_data->ptd_cnt = (count / sizeof(int)) - 1;
	sendTxData(gTx_data);
	mutex_unlock(&irda_tx_mutex);
	
	return count;
}

/* Character device file operations */
static const struct file_operations irda_fops = {
	.owner		= THIS_MODULE,
	.write		= irda_write,
	.llseek		= no_llseek,
};

static int h532b_irda_probe(struct platform_device *pdev)
{
	struct device *dev = NULL;
	int ret;

#ifdef USE_RX
	irq_handler_t handler;
	struct irda_rx_data *rx_data;
#endif
	struct irda_tx_data *tx_data;

	printk("[[ %s ]]\r\n", __func__);

#ifdef USE_RX
	rx_data = devm_kzalloc(&pdev->dev, sizeof(*rx_data), GFP_KERNEL);
	if (!rx_data) return -ENOMEM;

	rx_data->irda_rx = of_get_gpio(pdev->dev.of_node, 0);
	ret = gpio_request(rx_data->irda_rx, "h532b_irda_rx");
	if(ret < 0)
		printk("failed to request irda_rx gpio");

	handler = &irdarx_irq;
	ret = request_irq(gpio_to_irq(rx_data->irda_rx), handler,
									IRQF_TRIGGER_RISING,
			  						"hims-irda", rx_data);
	if (ret) {
		printk("unable to request IRQ%d\n", gpio_to_irq(rx_data->irda_rx));
		goto fail_err;
	}

	hrtimer_init(&rx_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rx_data->period = ktime_set(0, 10000);
	rx_data->timer.function = irda_rx_hrtimer;
	/* Starting read timer */
	// hrtimer_start(&rx_data->timer, rx_data->period, HRTIMER_MODE_REL);

	/* GPIO KEYS for testing */
	handler = &irdakey1_irq;
	ret = request_irq(gpio_to_irq(of_get_gpio(pdev->dev.of_node, 2)), handler,
									IRQF_TRIGGER_FALLING,
			  						"hims-irda-key1", rx_data);
	if (ret) {
		printk("unable to request IRQ%d\n", gpio_to_irq(rx_data->irda_rx));
		goto fail_err;
	}

	handler = &irdakey2_irq;
	ret = request_irq(gpio_to_irq(of_get_gpio(pdev->dev.of_node, 3)), handler,
									IRQF_TRIGGER_FALLING,
			  						"hims-irda-key2", rx_data);
	if (ret) {
		printk("unable to request IRQ%d\n", gpio_to_irq(rx_data->irda_rx));
		goto fail_err;
	}
#endif

	/* Init TX data*/
	tx_data = devm_kzalloc(&pdev->dev, sizeof(*tx_data), GFP_KERNEL);
	if (!tx_data) return -ENOMEM;

	tx_data->irda_tx = of_get_gpio(pdev->dev.of_node, 1);
	ret = gpio_request(tx_data->irda_tx, "h532b_irda_tx");
	if(ret < 0)
		printk("failed to request irda_tx gpio");

	/* PWM */
	ret = of_property_read_u32(pdev->dev.of_node, "pwm-ch", &tx_data->pwm_ch);
	if (ret && (ret != -EINVAL)) {
		printk("Unable to read pwm channel\n");
		return ret;
	}
	tx_data->pwm = pwm_request((int)tx_data->pwm_ch, pdev->name);
	ret = pwm_config(tx_data->pwm, 0, C_PERIOD * 1000);
	if (ret) {
		printk("%s: pwm_config fail\n", __func__);
	}
	pwm_enable(tx_data->pwm);

	/* Init global structure */
	gTx_data = tx_data;


	/* init character device */
	irda_major = register_chrdev(0, DEVICE_NAME, &irda_fops);
	if (irda_major < 0){
		printk("irda failed to register a major number");
		ret = irda_major;
		goto fail_err;
	}
	printk("irda : registered correctly with major number %d",
		irda_major);

	irda_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(irda_class)) {
		printk("Failed to create the device -- irda");
		unregister_chrdev(irda_major, DEVICE_NAME);
		ret = PTR_ERR(irda_class);
		goto fail_err;
	}
	printk("irda: device class created correctly");
	
	dev = device_create(irda_class, NULL, MKDEV(irda_major, 0), NULL,
		DEVICE_NAME);
	if(!dev) {
		printk("init: device_create fail");
		goto fail_err;		
	}
	printk("irda: device class created correctly");

	platform_set_drvdata(pdev, tx_data);

	return 0;
fail_err:
	pwm_disable(tx_data->pwm);
	pwm_free(tx_data->pwm);
	return ret;
}

static int h532b_irda_remove(struct platform_device *pdev)
{
	struct irda_tx_data *tx_data = platform_get_drvdata(pdev);
	printk("[[%s]]\r\n", __func__);
	platform_set_drvdata(pdev, NULL);
	pwm_disable(tx_data->pwm);
	pwm_free(tx_data->pwm);

	device_destroy(irda_class, MKDEV(irda_major, 0));
  	class_unregister(irda_class);
	class_destroy(irda_class);
	unregister_chrdev(irda_major, DEVICE_NAME);
	return 0;
}

static const struct of_device_id of_h532b_irda_match[] = {
	{ .compatible = "h532b-irda", },
	{},
};
MODULE_DEVICE_TABLE(of, of_h532b_irda_match);

static struct platform_driver brl_irda_driver = {
	.probe		= h532b_irda_probe,
	.remove		= h532b_irda_remove,
	.driver		= {
		.name	= "h532b_irda",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_h532b_irda_match),
	},
};

module_platform_driver(brl_irda_driver);

MODULE_AUTHOR("YIKIM");
MODULE_DESCRIPTION("H532B GPIO irda driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:h532b_irda");
