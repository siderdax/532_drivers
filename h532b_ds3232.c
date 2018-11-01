#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/rtc.h>

#define DS3232_REG_SECONDS	0x00
#define DS3232_REG_MINUTES	0x01
#define DS3232_REG_HOURS	0x02
#define DS3232_REG_AMPM		0x02
#define DS3232_REG_DAY		0x03
#define DS3232_REG_DATE		0x04
#define DS3232_REG_MONTH	0x05
#define DS3232_REG_CENTURY	0x05
#define DS3232_REG_YEAR		0x06
#define DS3232_REG_ALARM1         0x07	/* Alarm 1 BASE */
#define DS3232_REG_ALARM2         0x0B	/* Alarm 2 BASE */
#define DS3232_REG_CR		0x0E	/* Control register */
#	define DS3232_REG_CR_nEOSC        0x80
#       define DS3232_REG_CR_INTCN        0x04
#       define DS3232_REG_CR_A2IE        0x02
#       define DS3232_REG_CR_A1IE        0x01

#define DS3232_REG_SR	0x0F	/* control/status register */
#	define DS3232_REG_SR_OSF   0x80
#       define DS3232_REG_SR_BSY   0x04
#       define DS3232_REG_SR_A2F   0x02
#       define DS3232_REG_SR_A1F   0x01

struct ds3232 {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct work_struct work;

	/* The mutex protects alarm operations, and prevents a race
	 * between the enable_irq() in the workqueue and the free_irq()
	 * in the remove function.
	 */
	struct mutex mutex;
	int exiting;
};

struct rtc_time *custom_tm;
static struct i2c_driver ds3232_driver;

// For rtc-sec.c
struct device *ds3232_dev;
// Came from rtc-sec.c
extern struct sec_pmic_dev *gSEC_PMIC_iodev;

static void rtc_power(int onoff)
{
	struct regulator *regulator_19 = NULL;
	struct regulator *regulator_20 = NULL;
	int ret = 0;

	regulator_19 = regulator_get(NULL, "vdd_ldo19");
	if(regulator_19 == NULL) {
 		pr_err("LDO19 get error\n");
 		return;
 	} else {
		pr_info("LDO19 turn on\n");
	}

	regulator_20 = regulator_get(NULL, "vdd_ldo20");
	if(regulator_20 == NULL) {
 		pr_err("LDO20 get error\n");
 		return;
 	} else {
		pr_info("LDO20 turn on\n");
	}


	if(onoff == 1) {
		ret = regulator_enable(regulator_19);
		if(ret == 0)
			ret = regulator_enable(regulator_20);
	}
	else {
		ret = regulator_disable(regulator_19);
		if(ret == 0)
			ret = regulator_disable(regulator_20);
	}

	if (ret < 0) {
		pr_err("%s: regulator %sable fail %d\n", __func__,
				onoff ? "en" : "dis", ret);

		goto regs_fail;
	}

regs_fail:
	regulator_put(regulator_19);
	regulator_put(regulator_20);

	return;
}

static int tm_to_data(struct rtc_time *tm, u8 *data)
{
	data[RTC_SEC] = tm->tm_sec;
	data[RTC_MIN] = tm->tm_min;

	if (tm->tm_hour >= 12)
		data[RTC_HOUR] = tm->tm_hour | HOUR_PM_MASK;
	else
		data[RTC_HOUR] = tm->tm_hour;

	data[RTC_WEEKDAY] = BIT(tm->tm_wday);
	data[RTC_DATE] = tm->tm_mday;
	data[RTC_MONTH] = tm->tm_mon + 1;
	data[RTC_YEAR] = tm->tm_year > 100 ? (tm->tm_year - 100) : 0 ;

	if (tm->tm_year < 100) {
		pr_warn("%s: Cannot handle the year %d\n", __func__,
				1900 + tm->tm_year);
		return -EINVAL;
	}
	return 0;
}

static int ds3232_check_rtc_status(struct i2c_client *client)
{
	int ret = 0;
	int control, stat;

	stat = i2c_smbus_read_byte_data(client, DS3232_REG_SR);
	if (stat < 0)
		return stat;

	if (stat & DS3232_REG_SR_OSF)
		dev_warn(&client->dev,
				"oscillator discontinuity flagged, "
				"time unreliable\n");

	stat &= ~(DS3232_REG_SR_OSF | DS3232_REG_SR_A1F | DS3232_REG_SR_A2F);

	ret = i2c_smbus_write_byte_data(client, DS3232_REG_SR, stat);
	if (ret < 0)
		return ret;

	/* If the alarm is pending, clear it before requesting
	 * the interrupt, so an interrupt event isn't reported
	 * before everything is initialized.
	 */

	control = i2c_smbus_read_byte_data(client, DS3232_REG_CR);
	if (control < 0)
		return control;

	control &= ~(DS3232_REG_CR_A1IE | DS3232_REG_CR_A2IE);
	control |= DS3232_REG_CR_INTCN;

	return i2c_smbus_write_byte_data(client, DS3232_REG_CR, control);
}

static int ds3232_read_time(struct device *dev, struct rtc_time *time)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	u8 buf[7];
	unsigned int year, month, day, hour, minute, second;
	unsigned int week, twelve_hr, am_pm;
	unsigned int century, add_century = 0;

	ret = i2c_smbus_read_i2c_block_data(client, DS3232_REG_SECONDS, 7, buf);

	if (ret < 0)
		return ret;
	if (ret < 7)
		return -EIO;

	second = buf[0];
	minute = buf[1];
	hour = buf[2];
	week = buf[3];
	day = buf[4];
	month = buf[5];
	year = buf[6];

	/* Extract additional information for AM/PM and century */

	twelve_hr = hour & 0x40;
	am_pm = hour & 0x20;
	century = month & 0x80;

	/* Write to rtc_time structure */

	time->tm_sec = bcd2bin(second);
	time->tm_min = bcd2bin(minute);
	if (twelve_hr) {
		/* Convert to 24 hr */
		if (am_pm)
			time->tm_hour = bcd2bin(hour & 0x1F) + 12;
		else
			time->tm_hour = bcd2bin(hour & 0x1F);
	} else {
		time->tm_hour = bcd2bin(hour);
	}

	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	time->tm_wday = bcd2bin(week) - 1;
	time->tm_mday = bcd2bin(day);
	/* linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	time->tm_mon = bcd2bin(month & 0x7F) - 1;
	if (century)
		add_century = 100;

	time->tm_year = bcd2bin(year) + add_century;

	printk("\t\t%s : \r\n", __func__);
	printk("\t\t\t%d-%d-%d\r\n", time->tm_year, time->tm_mon, time->tm_mday);
	printk("\t\t\t%d, %d, %d\r\n", time->tm_yday, time->tm_wday, time->tm_isdst);
	printk("\t\t\t%d:%d:%d\r\n", time->tm_hour, time->tm_min, time->tm_sec);

	return rtc_valid_tm(time);
}

static int ds3232_set_time(struct device *dev, struct rtc_time *time)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 buf[7];

	/* Extract time from rtc_time and load into ds3232*/

	buf[0] = bin2bcd(time->tm_sec);
	buf[1] = bin2bcd(time->tm_min);
	buf[2] = bin2bcd(time->tm_hour);
	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	buf[3] = bin2bcd(time->tm_wday + 1);
	buf[4] = bin2bcd(time->tm_mday); /* Date */
	/* linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	buf[5] = bin2bcd(time->tm_mon + 1);
	if (time->tm_year >= 100) {
		buf[5] |= 0x80;
		buf[6] = bin2bcd(time->tm_year - 100);
	} else {
		buf[6] = bin2bcd(time->tm_year);
	}

	return i2c_smbus_write_i2c_block_data(client,
					      DS3232_REG_SECONDS, 7, buf);
}

/* Exist for rtc-sec.c */
int h532b_ds3232_set_time(struct rtc_time *time)
{
	pr_info("%s called\r\n", __func__);
	if(ds3232_dev == NULL) {
		pr_warn("%s : Warning : device is not initialized yet, skipping\r\n", __func__);
		return 0;
	}
	return ds3232_set_time(ds3232_dev, time);
}


/*
 * DS3232 has two alarm, we only use alarm1
 * According to linux specification, only support one-shot alarm
 * no periodic alarm mode
 */
static int ds3232_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds3232 *ds3232 = i2c_get_clientdata(client);
	int control, stat;
	int ret;
	u8 buf[4];

	mutex_lock(&ds3232->mutex);

	ret = i2c_smbus_read_byte_data(client, DS3232_REG_SR);
	if (ret < 0)
		goto out;
	stat = ret;
	ret = i2c_smbus_read_byte_data(client, DS3232_REG_CR);
	if (ret < 0)
		goto out;
	control = ret;
	ret = i2c_smbus_read_i2c_block_data(client, DS3232_REG_ALARM1, 4, buf);
	if (ret < 0)
		goto out;

	alarm->time.tm_sec = bcd2bin(buf[0] & 0x7F);
	alarm->time.tm_min = bcd2bin(buf[1] & 0x7F);
	alarm->time.tm_hour = bcd2bin(buf[2] & 0x7F);
	alarm->time.tm_mday = bcd2bin(buf[3] & 0x7F);

	alarm->time.tm_mon = -1;
	alarm->time.tm_year = -1;
	alarm->time.tm_wday = -1;
	alarm->time.tm_yday = -1;
	alarm->time.tm_isdst = -1;

	alarm->enabled = !!(control & DS3232_REG_CR_A1IE);
	alarm->pending = !!(stat & DS3232_REG_SR_A1F);

	ret = 0;
out:
	mutex_unlock(&ds3232->mutex);
	return ret;
}

/*
 * linux rtc-module does not support wday alarm
 * and only 24h time mode supported indeed
 */
static int ds3232_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds3232 *ds3232 = i2c_get_clientdata(client);
	int control, stat;
	int ret;
	u8 buf[4];

	if (client->irq <= 0)
		return -EINVAL;

	mutex_lock(&ds3232->mutex);

	buf[0] = bin2bcd(alarm->time.tm_sec);
	buf[1] = bin2bcd(alarm->time.tm_min);
	buf[2] = bin2bcd(alarm->time.tm_hour);
	buf[3] = bin2bcd(alarm->time.tm_mday);

	/* clear alarm interrupt enable bit */
	ret = i2c_smbus_read_byte_data(client, DS3232_REG_CR);
	if (ret < 0)
		goto out;
	control = ret;
	control &= ~(DS3232_REG_CR_A1IE | DS3232_REG_CR_A2IE);
	ret = i2c_smbus_write_byte_data(client, DS3232_REG_CR, control);
	if (ret < 0)
		goto out;

	/* clear any pending alarm flag */
	ret = i2c_smbus_read_byte_data(client, DS3232_REG_SR);
	if (ret < 0)
		goto out;
	stat = ret;
	stat &= ~(DS3232_REG_SR_A1F | DS3232_REG_SR_A2F);
	ret = i2c_smbus_write_byte_data(client, DS3232_REG_SR, stat);
	if (ret < 0)
		goto out;

	ret = i2c_smbus_write_i2c_block_data(client, DS3232_REG_ALARM1, 4, buf);

	if (alarm->enabled) {
		control |= DS3232_REG_CR_A1IE;
		ret = i2c_smbus_write_byte_data(client, DS3232_REG_CR, control);
	}
out:
	mutex_unlock(&ds3232->mutex);
	return ret;
}

static void ds3232_update_alarm(struct i2c_client *client)
{
	struct ds3232 *ds3232 = i2c_get_clientdata(client);
	int control;
	int ret;
	u8 buf[4];

	mutex_lock(&ds3232->mutex);

	ret = i2c_smbus_read_i2c_block_data(client, DS3232_REG_ALARM1, 4, buf);
	if (ret < 0)
		goto unlock;

	buf[0] = bcd2bin(buf[0]) < 0 || (ds3232->rtc->irq_data & RTC_UF) ?
								0x80 : buf[0];
	buf[1] = bcd2bin(buf[1]) < 0 || (ds3232->rtc->irq_data & RTC_UF) ?
								0x80 : buf[1];
	buf[2] = bcd2bin(buf[2]) < 0 || (ds3232->rtc->irq_data & RTC_UF) ?
								0x80 : buf[2];
	buf[3] = bcd2bin(buf[3]) < 0 || (ds3232->rtc->irq_data & RTC_UF) ?
								0x80 : buf[3];

	ret = i2c_smbus_write_i2c_block_data(client, DS3232_REG_ALARM1, 4, buf);
	if (ret < 0)
		goto unlock;

	control = i2c_smbus_read_byte_data(client, DS3232_REG_CR);
	if (control < 0)
		goto unlock;

	if (ds3232->rtc->irq_data & (RTC_AF | RTC_UF))
		/* enable alarm1 interrupt */
		control |= DS3232_REG_CR_A1IE;
	else
		/* disable alarm1 interrupt */
		control &= ~(DS3232_REG_CR_A1IE);
	i2c_smbus_write_byte_data(client, DS3232_REG_CR, control);

unlock:
	mutex_unlock(&ds3232->mutex);
}

static int ds3232_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds3232 *ds3232 = i2c_get_clientdata(client);

	if (client->irq <= 0)
		return -EINVAL;

	if (enabled)
		ds3232->rtc->irq_data |= RTC_AF;
	else
		ds3232->rtc->irq_data &= ~RTC_AF;

	ds3232_update_alarm(client);
	return 0;
}

// static irqreturn_t ds3232_irq(int irq, void *dev_id)
// {
// 	struct i2c_client *client = dev_id;
// 	struct ds3232 *ds3232 = i2c_get_clientdata(client);

// 	disable_irq_nosync(irq);
// 	schedule_work(&ds3232->work);
// 	return IRQ_HANDLED;
// }

static void ds3232_work(struct work_struct *work)
{
	struct ds3232 *ds3232 = container_of(work, struct ds3232, work);
	struct i2c_client *client = ds3232->client;
	int stat, control;

	mutex_lock(&ds3232->mutex);

	stat = i2c_smbus_read_byte_data(client, DS3232_REG_SR);
	if (stat < 0)
		goto unlock;

	if (stat & DS3232_REG_SR_A1F) {
		control = i2c_smbus_read_byte_data(client, DS3232_REG_CR);
		if (control < 0)
			goto out;
		/* disable alarm1 interrupt */
		control &= ~(DS3232_REG_CR_A1IE);
		i2c_smbus_write_byte_data(client, DS3232_REG_CR, control);

		/* clear the alarm pend flag */
		stat &= ~DS3232_REG_SR_A1F;
		i2c_smbus_write_byte_data(client, DS3232_REG_SR, stat);

		// rtc_update_irq(ds3232->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	if (!ds3232->exiting)
		enable_irq(client->irq);
unlock:
	mutex_unlock(&ds3232->mutex);
}

static const struct rtc_class_ops ds3232_rtc_ops = {
	.read_time = ds3232_read_time,
	.set_time = ds3232_set_time,
	.read_alarm = ds3232_read_alarm,
	.set_alarm = ds3232_set_alarm,
	.alarm_irq_enable = ds3232_alarm_irq_enable,
};

/* Function for S2MP15X pmic RTC */
int sec_time_write(struct sec_pmic_dev *iodev, struct rtc_time *rt)
{
	int ret;
	u8 tm_data[NR_RTC_CNT_REGS], update_reg;
	u32 update_val;

	ret = tm_to_data(rt, tm_data);
	if(!ret) {
		ret = sec_rtc_bulk_write(iodev, S2M_RTC_SEC, NR_RTC_CNT_REGS,
			tm_data);
	}

	ret = sec_rtc_read(iodev, S2M_RTC_UPDATE, &update_val);
	if (ret < 0) {
		pr_err("%s: fail to read update reg(%d)\n",	__func__, ret);
		return ret;
	}

	update_reg = update_val & ~(RTC_WUDR_MASK_REV | RTC_FREEZE_MASK |
										RTC_RUDR_MASK | RTC_AUDR_MASK_REV);

	ret = sec_rtc_write(iodev, S2M_RTC_UPDATE, update_reg | RTC_WUDR_MASK_REV);
	if (ret < 0)
		pr_err("%s: fail to write update reg(%d,%u)\n", __func__, ret,
												update_reg | RTC_WUDR_MASK_REV);

	return ret;
}

static int ds3232_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ds3232 *ds3232;
	int ret;

	ds3232_dev = &client->dev;

	/* Power on RTC */
	rtc_power(1);

	ds3232 = devm_kzalloc(&client->dev, sizeof(struct ds3232), GFP_KERNEL);
	if (!ds3232)
		return -ENOMEM;

	ds3232->client = client;
	i2c_set_clientdata(client, ds3232);

	INIT_WORK(&ds3232->work, ds3232_work);
	mutex_init(&ds3232->mutex);

	ret = ds3232_check_rtc_status(client);
	if (ret)
		return ret;

	/* 	This RTC is not main device, only supporting primary RTC. 
	 *	So disabled interrupt & rtc driver
	 */

	// ds3232->rtc = devm_rtc_device_register(&client->dev, client->name,
	// 				  &ds3232_rtc_ops, THIS_MODULE);
	// if (IS_ERR(ds3232->rtc)) {
	// 	dev_err(&client->dev, "unable to register the class device\n");
	// 	return PTR_ERR(ds3232->rtc);
	// }

	// if (client->irq >= 0) {
	// 	ret = devm_request_irq(&client->dev, client->irq, ds3232_irq, 0,
	// 			 "ds3232", client);
	// 	if (ret) {
	// 		dev_err(&client->dev, "unable to request IRQ\n");
	// 		return ret;
	// 	}
	// }

	custom_tm = kzalloc(sizeof(*custom_tm), GFP_KERNEL);

	printk("CHECKING TIME INFORMATION\r\n");
	ds3232_read_time(&client->dev, custom_tm);

	/* Write time data to primary RTC */
	sec_time_write(gSEC_PMIC_iodev, custom_tm);

	return 0;
}

static int ds3232_resume(struct device *dev)
{
	/* Write time data to primary RTC */
	sec_time_write(gSEC_PMIC_iodev, custom_tm);

	return 0;
}

static int ds3232_remove(struct i2c_client *client)
{
	struct ds3232 *ds3232 = i2c_get_clientdata(client);

	if (client->irq >= 0) {
		mutex_lock(&ds3232->mutex);
		ds3232->exiting = 1;
		mutex_unlock(&ds3232->mutex);

		devm_free_irq(&client->dev, client->irq, client);
		cancel_work_sync(&ds3232->work);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ds3232_of_match[] = {
	{ .compatible = "maxim,ds3232", },
	{ .compatible = "h532b,rtc", },
	{ }
};
MODULE_DEVICE_TABLE(of, max98095_of_match);
#endif

static const struct i2c_device_id ds3232_id[] = {
	{ "ds3232", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds3232_id);

static const struct dev_pm_ops ds3232_pm_ops = {
	.resume = ds3232_resume,
};

static struct i2c_driver ds3232_driver = {
	.driver = {
		.name = "rtc-ds3232",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ds3232_of_match,
#endif
		.pm = &ds3232_pm_ops,
	},
	.probe = ds3232_probe,
	.remove = ds3232_remove,
	.id_table = ds3232_id,
};

module_i2c_driver(ds3232_driver);

MODULE_AUTHOR("YIKIM");
MODULE_DESCRIPTION("H532B RTC Driver");
MODULE_LICENSE("GPL");
