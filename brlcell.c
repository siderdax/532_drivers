#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <plat/gpio-cfg.h>

#include "brlcell.h"

/* Global vars for initialization of Character device */
unsigned int brlcell_major = 0;
static struct class *brlcell_class;

static DEFINE_MUTEX(ghmutex_cell);
static DEFINE_MUTEX(ghHW_cell);

struct hims_brlcell *ghims_brlcell;

#define CLOCK_LOW		gpio_set_value(ghims_brlcell->gpio_clk, 0);
#define CLOCK_HIGH		gpio_set_value(ghims_brlcell->gpio_clk, 1);
#define DATA_LOW		gpio_set_value(ghims_brlcell->gpio_data, 0);
#define DATA_HIGH		gpio_set_value(ghims_brlcell->gpio_data, 1);
#define STROBE_LOW		gpio_set_value(ghims_brlcell->gpio_strb, 0);
#define STROBE_HIGH		gpio_set_value(ghims_brlcell->gpio_strb, 1);

extern long start_key_t, end_key_t;

static inline long myClock(void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void initCell(void)
{
	int k = 0; int p = 0;
	unsigned char asdf;
	
	STROBE_LOW;
	
     for( p = 0; p < BRAILLE_SIZE; p++)
	 {
		asdf = 0x55;
        for(k = 0; k < 8; k++)
		{
			if(asdf & 0x80) {
				DATA_HIGH;                      
			} else {
				DATA_LOW;
			}
			CLOCK_HIGH;
			udelay(5);
			CLOCK_LOW;
			udelay(5);
			asdf <<= 1;
		}
	}
	udelay(5);
	STROBE_HIGH;
	udelay(5);
    STROBE_LOW;
}

void setCellData(unsigned char *cellData)
{
	unsigned char	tmp=0;
	unsigned char	*pTemp=NULL;
	int		i, j;
	DBG_PRINTK("#2 cellData[0] : %d, [1] : %d, [2] : %d",
		cellData[0], cellData[1], cellData[2]);
	
	STROBE_LOW;
	
	pTemp = &cellData[BRAILLE_SIZE-1];
	for (i=0; i<BRAILLE_SIZE; i++)
	{
		tmp = *pTemp--;
		for (j = 0; j < 8; j++) {
			/* send each bit from the MSB */
			if( (j%2) == 0 ) {
				if ( !((tmp<<j) & 0x80) )	{
					DATA_LOW;
				} else {
					DATA_HIGH;
				}
			} else {
				if ( !((tmp<<j) & 0x80) )	{
					DATA_HIGH;
				} else {
					DATA_LOW;
				}
			}
			// udelay(5);
			CLOCK_HIGH;
			// udelay(5);
			CLOCK_LOW;			
			// udelay(5);
		}
	}
	// udelay(1);

	STROBE_HIGH;
	udelay(1);
 	STROBE_LOW;
}

static long brlcell_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret=0;
	cell_data celldata;

	DBG_PRINTK("called");
	
	switch (cmd) {
	case BRLCELL_SET_DATA:
		ret = copy_from_user( (void *)&celldata, (void *)arg, sizeof(cell_data));
		setCellData((unsigned char *)celldata.data);
		break;
	}
	
	return ret;
}

static int brlcell_open(struct inode *inode, struct file *file)
{
	DBG_PRINTK("called, %lu", BRLCELL_SET_DATA);
	return 0;
}

static int brlcell_release(struct inode *inode, struct file *file)
{
	DBG_PRINTK("called, %s", __func__);
	return 0;
}

static int brlcell_probe(struct platform_device *pdev) {
	struct device_node *node;

	DBG_PRINTK("called");

	node = pdev->dev.of_node;
	if (!node) {
		DBG_PRINTK("node null error");
		return -ENODEV;
	}

	ghims_brlcell->gpio_clk  = of_get_gpio(node, 0);
	ghims_brlcell->gpio_data = of_get_gpio(node, 1);
	ghims_brlcell->gpio_strb = of_get_gpio(node, 2);

	DBG_PRINTK("count = %d", of_gpio_count(node));

	// if (gpio_request(gpio[0], "BRLCELL_GPIO - CLOCK")) {
	// 	DBG_PRINTK("fail to request gpio(BRLCELL_GPIO - CLOCK)");
	// 	return -ENODEV;
	// }

	initCell();
	// Set cell thread data for test
	{
		unsigned char bCellData[BRAILLE_SIZE] = {0x20, 0x20, 0x4c, 0x84, 0x8a,
			0x86, 0x00, 0x20, 0x42, 0xc6, 0xec, 0xca, 0x20, 0xca, 0x8, 0xe, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		setCellData(bCellData);
	}

	return 0;
}

static const struct of_device_id brlcell_of_match[] = {
	{
		.compatible = "hims,h532b-cell-gpio",
	},
	{},
};
MODULE_DEVICE_TABLE(of, brlcell_of_match);

/* Character device file operations */
static const struct file_operations brlcell_fops = {
	.owner		= THIS_MODULE,
	.open		= brlcell_open,
	.release	= brlcell_release,
	.unlocked_ioctl	= brlcell_ioctl,
	.llseek		= no_llseek,
};

static struct platform_driver brlcell_platform_driver = {
	.probe = brlcell_probe,
	.remove = NULL,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = brlcell_of_match,
	}
}; 
module_platform_driver(brlcell_platform_driver);

static int __init brlcell_init(void)
{
	struct device *dev = NULL;
	int ret = 0;
		
	DBG_PRINTK("++");

	/* init CHDEV */
	brlcell_major = register_chrdev(0, DEVICE_NAME, &brlcell_fops);
	if (brlcell_major < 0){
		DBG_PRINTK("brlcell failed to register a major number");
		return brlcell_major;
	}
	DBG_PRINTK("brlcell : registered correctly with major number %d",
		brlcell_major);

	brlcell_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(brlcell_class)) {
		DBG_PRINTK("Failed to create the device -- brlcell");
		unregister_chrdev(brlcell_major, DEVICE_NAME);
		ret = PTR_ERR(brlcell_class);
		goto err;
	}
	DBG_PRINTK("brlcell: device class created correctly");
	
	dev = device_create(brlcell_class, NULL, MKDEV(brlcell_major, 0), NULL,
		DEVICE_NAME);
	if(!dev) {
		DBG_PRINTK("init: device_create fail");
		goto err;		
	}
	DBG_PRINTK("brlcell: device class created correctly");
	/* init CHDEV END*/

	/* Cell device driver struct */
	ghims_brlcell = kzalloc(sizeof(*ghims_brlcell), GFP_KERNEL);
	if(!ghims_brlcell) {
		ret = -ENOMEM;
		DBG_PRINTK("brlcell_init: kzalloc fail \r\n");
		goto err_free_mem;
	}

	/* Thread loop waitqueue */
	init_waitqueue_head(&ghims_brlcell->wait);
	ghims_brlcell->stopped = false;

	DBG_PRINTK("-- ret:%d", ret);
	return 0;
err_free_mem:
	kfree(ghims_brlcell);
err:
	DBG_PRINTK("error:0x%X", ret);
	return ret;	
}

static void __exit brlcell_exit(void)
{
	device_destroy(brlcell_class, MKDEV(brlcell_major, 0));
  	class_unregister(brlcell_class);
	class_destroy(brlcell_class);
	unregister_chrdev(brlcell_major, DEVICE_NAME);
}

module_init(brlcell_init);
module_exit(brlcell_exit);


MODULE_AUTHOR("Paul Gortmaker");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(BRLCELL_MINOR);