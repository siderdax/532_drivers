#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>

#include <asm/io.h>

static ssize_t set_rf_en(struct device *dev, struct device_attribute *attr,
			    		const char *buf, size_t size)
{
	int rf_gpio;
	unsigned long value;

	value = simple_strtoul(buf, NULL, 0);
	rf_gpio = of_get_named_gpio(dev->of_node, "rf_en", 0);

	if(rf_gpio > 0)
		gpio_set_value(rf_gpio, (int)value);
	else
		printk("%s : gpio error\r\n", __func__);

	return size;
}

static ssize_t show_rf_en(struct device *dev,struct device_attribute *attr,
						char *buf)
{
	int rf_gpio, value = -1;

	rf_gpio = of_get_named_gpio(dev->of_node, "rf_en", 0);
	if(rf_gpio > 0)
		value = gpio_get_value(rf_gpio);
	else
		printk("%s : gpio error\r\n", __func__);

	return sprintf(buf, "%d\n", value);
}

static DEVICE_ATTR(rf_enable, S_IWUGO|S_IRUGO, show_rf_en, set_rf_en);

static ssize_t set_uart3(struct device *dev, struct device_attribute *attr,
			    		const char *buf, size_t size)
{
	void __iomem *uart_sdata_addr;
	u32	uart_sdata;

	uart_sdata_addr = ioremap(0x14C40000, 1000);
	uart_sdata = simple_strtoul(buf, NULL, 0);;
	__raw_writel(uart_sdata, uart_sdata_addr);

	return size;
}

static ssize_t show_uart3(struct device *dev,struct device_attribute *attr,
						char *buf)
{
	void __iomem *uart_sdata_addr;

	uart_sdata_addr = ioremap(0x14C40000, 1000);

	return sprintf(buf, "uart settings : 0x%x\n", __raw_readl(uart_sdata_addr));
}

static DEVICE_ATTR(uart3_settings, S_IWUGO|S_IRUGO, show_uart3, set_uart3);

static struct attribute *gpioen_attributes[] = {
	&dev_attr_rf_enable.attr,
	&dev_attr_uart3_settings.attr,
	NULL
};

static struct attribute_group gpioen_attr_group = {
	.attrs			= gpioen_attributes,
	.name			= "gpio_enable_controller",
};

static int h532b_gpio_en_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("[[%s]]\r\n", __func__);

	ret = sysfs_create_group(&pdev->dev.kobj, &gpioen_attr_group);
	if(ret < 0)
		printk("%s : Can't create sysfs\r\n", __func__);

	return ret;
}

static int h532b_gpio_en_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gpioen_attr_group);
	return 0;
}

static const struct of_device_id of_h532b_gpio_en_match[] = {
	{ .compatible = "h532b-gpio_en", },
	{},
};
MODULE_DEVICE_TABLE(of, of_h532b_gpio_en_match);

static struct platform_driver led_pwm_driver = {
	.probe		= h532b_gpio_en_probe,
	.remove		= h532b_gpio_en_remove,
	.driver		= {
		.name	= "h532b_gpio_en",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_h532b_gpio_en_match),
	},
};

module_platform_driver(led_pwm_driver);

MODULE_AUTHOR("YIKIM");
MODULE_DESCRIPTION("H532B GPIO enable control driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:h532b_gpio_en");
