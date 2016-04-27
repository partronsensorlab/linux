/*
* drivers/sensors/dts201a.c
*
* Partron DTS201A Thermopile Sensor module driver
*
* Copyright (C) 2013 Partron Co., Ltd. - Sensor Lab.
* partron (partron@partron.co.kr)
*
* Both authors are willing to be considered the contact and update points for
* the driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 1.0.0 2013/Aug/08:
	first release

******************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include "dts201a.h"
//#include <linux/sensors.h>
//#include <linux/sensor/sensors_core.h>

#undef DTS201A_DEBUG
//#define	DTS201A_DEBUG
#define	DTS201A_TEST

struct dts201a_data {
	struct i2c_client *client;
	struct mutex lock;
	struct workqueue_struct *thermopile_wq;
	struct work_struct work_thermopile;
	struct input_dev *input_dev;
	struct device *dev;
	struct outputdata out;
	struct hrtimer timer;
	ktime_t poll_delay;
	int hw_initialized;
	atomic_t enabled;
};

static int dts201a_i2c_read(struct dts201a_data *ther,
				  u8 *buf, u8 len)
{
	struct i2c_msg msg[1];
	int err = 0;
	int retry = 3;

	if ((ther->client == NULL) || (!ther->client->adapter))
		return -ENODEV;

//	msg->addr = ther->client->addr;
	msg->addr = DTS201A_SADDR;
	msg->flags = DTS_I2C_READ;
	msg->len = len;
	msg->buf = buf;

	while (retry--) {
		err = i2c_transfer(ther->client->adapter, msg, 1);
		if (err >= 0)
			return err;
	}
	pr_err("%s: i2c read failed at addr 0x%x: %d\n", __func__, msg->addr, err);

	return err;
}

static int dts201a_i2c_write(struct dts201a_data *ther,
				u8 cmd, u16 bm_config)
{
	struct i2c_msg msg[1];
	int err = 0;
	int retry = 3;
	u8 wdata[3]={0, };

	if ((ther->client == NULL) || (!ther->client->adapter))
		return -ENODEV;

	wdata[0] = cmd;
	wdata[1] = (u8)(bm_config>>8);
	wdata[2] = (u8)(bm_config);

//	msg->addr = ther->client->addr;
	msg->addr = DTS201A_SADDR;
	msg->flags = DTS_I2C_WRITE;
	msg->len = 3;
	msg->buf = wdata;

	while (retry--) {
		err = i2c_transfer(ther->client->adapter, msg, 1);
		if (err >= 0)
			return err;
	}
	pr_err("%s, i2c transfer error(%d)\n", __func__, err);

	return err;
}

static int dts201a_hw_init(struct dts201a_data *ther)
{
	int err = 0;
	u8 rbuf[5]={0, };
	u16 custom_id[2]={0, };

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %s\n", __func__, DTS_DEV_NAME);
#endif

	err = dts201a_i2c_write(ther, DTS_CUST_ID0, 0x0);
	if (err < 0) goto error_firstread;
	msleep(20);
	err = dts201a_i2c_read(ther, rbuf, 5);
	if (err < 0) goto error_firstread;
	custom_id[0] = ( (rbuf[1]<<8) | rbuf[2] );

	err = dts201a_i2c_write(ther, DTS_CUST_ID1, 0x0);
	if (err < 0) goto error_firstread;
	msleep(20);
	err = dts201a_i2c_read(ther, rbuf, 5);
	if (err < 0) goto error_firstread;
	custom_id[1] = ( (rbuf[1]<<8) | rbuf[2] );

	if (custom_id[1] != DTS201A_ID1 && custom_id[0] != DTS201A_ID0) {
		err = -ENODEV;
		goto error_unknown_device;
	}

	ther->hw_initialized = 1;

	return 0;

error_firstread:
	dev_warn(&ther->client->dev, "Error reading CUST_ID_DTS201A : is device "
		"available/working?\n");
	goto err_resume_state;
error_unknown_device:
	dev_err(&ther->client->dev, "device unknown. Expected: 0x%04x_%04x,"
		" custom_id: 0x%04x_%04x\n", DTS201A_ID1, 
		DTS201A_ID0, custom_id[1], custom_id[0]);
err_resume_state:
	ther->hw_initialized = 0;
	dev_err(&ther->client->dev, "hw init error : status = 0x%02x: err = %d\n", rbuf[0], err);

	return err;
}

static int dts201a_get_thermtemp_data(struct dts201a_data *ther)
{
	int err = 0, i = 0;
	u8 rbuf[7] = {0,};
	u32 thermopile = 0;
	u32 temperature = 0;
	u8 status = 0;

#if 0
	err = dts201a_i2c_write(ther, DTS_SM_AZSM, 0x0);
	if (err < 0) goto i2c_error;
	msleep(DTS_CONVERSION_TIME);
	err = dts201a_i2c_read(ther, rbuf, 4);
	if (err < 0) goto i2c_error;
	thermopile = (s32) ((((s8) rbuf[1]) << 16) | (rbuf[2] << 8) | rbuf[3]);

	err = dts201a_i2c_write(ther, DTS_TM_AZTM, 0x0);
	if (err < 0) goto i2c_error;
	msleep(DTS_CONVERSION_TIME);
	err = dts201a_i2c_read(ther, rbuf, 4);
	if (err < 0) goto i2c_error;
	temperature = (s32) ((((s8) rbuf[1]) << 16) | (rbuf[2] << 8) | rbuf[3]);

	status = rbuf[0];
#else
	err = dts201a_i2c_write(ther, DTS_MEASURE4, 0x0);
	if (err < 0) goto i2c_error;
	msleep(DTS_CONVERSION_TIME);
	err = dts201a_i2c_read(ther, rbuf, 7);
	if (err < 0) goto i2c_error;

	status = rbuf[0];
	thermopile = ((rbuf[1]) << 16) | (rbuf[2] << 8) | rbuf[3];
	temperature = ((rbuf[4]) << 16) | (rbuf[5] << 8) | rbuf[6];

	//average
	for(i=0; i<3; i++) {
		err = dts201a_i2c_write(ther, DTS_MEASURE4, 0x0);
		if (err < 0) goto i2c_error;
		msleep(DTS_CONVERSION_TIME);
		err = dts201a_i2c_read(ther, rbuf, 7);
		if (err < 0) goto i2c_error;

		status = rbuf[0];
		thermopile = (thermopile + ((rbuf[1] << 16) | (rbuf[2] << 8) | rbuf[3]))/2;
		temperature = (temperature + ((rbuf[4] << 16) | (rbuf[5] << 8) | rbuf[6]))/2;
	}
#endif

#ifdef DTS201A_DEBUG
//	dev_dbg(&ther->client->dev, "thermopile = 0x%06x"
//			"Temperature = 0x%06x, status = 0x%02x\n",
//					thermopile, temperature, status);
	pr_err("%s : rbuf[1][2][3] = 0x%02x_%02x_%02x = 0x%06x\n", __func__, rbuf[1], rbuf[2], rbuf[3], thermopile);
	pr_err("%s : rbuf[4][5][6] = 0x%02x_%02x_%02x = 0x%06x\n", __func__, rbuf[4], rbuf[5], rbuf[6], temperature);
#endif

	ther->out.therm = thermopile;
	ther->out.temp = temperature;
	ther->out.status = status;

	return err;

i2c_error :
	ther->out.therm = 0;
	ther->out.temp = 0;
	ther->out.status = 0;
	
	return err;
}

static int dts201a_enable(struct dts201a_data *ther)
{
	int err = 0;

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %d\n", __func__, 0);
#endif

	if (!atomic_cmpxchg(&ther->enabled, 0, 1))
		hrtimer_start(&ther->timer, ther->poll_delay, HRTIMER_MODE_REL);

	return err;
}

static int dts201a_disable(struct dts201a_data *ther)
{
	int err = 0;

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %d\n", __func__, 0);
#endif

	if (atomic_cmpxchg(&ther->enabled, 1, 0)) {
		hrtimer_cancel(&ther->timer);
		cancel_work_sync(&ther->work_thermopile);
	}

	return err;
}

static ssize_t dts201a_poll_delay_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s poll_delay = %lld\n", __func__, ktime_to_ns(ther->poll_delay));
#endif

	return sprintf(buf, "%lld\n", ktime_to_ns(ther->poll_delay));
}

static ssize_t dts201a_poll_delay_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);
	unsigned long new_delay = 0;
	int err;

	//err = strict_strtoul(buf, 10, &new_delay);
	if (err < 0)
		return err;

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s new_delay = %ld\n", __func__, new_delay);
#endif

	if(new_delay < DTS_DELAY_MINIMUM)
		new_delay = DTS_DELAY_MINIMUM;

	if (new_delay != ktime_to_ns(ther->poll_delay))
		ther->poll_delay = ns_to_ktime(new_delay);

	mutex_lock(&ther->lock);
	ther->poll_delay = ns_to_ktime(new_delay);
	mutex_unlock(&ther->lock);

	return size;
}

static ssize_t dts201a_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s ther->enabled = %d\n", __func__, 
		atomic_read(&ther->enabled));
#endif

	return sprintf(buf, "%d\n", atomic_read(&ther->enabled));
}

static ssize_t dts201a_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	
#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s new_value = 0x%d\n", __func__, new_value);
#endif

	if (new_value)
		dts201a_enable(ther);
	else
		dts201a_disable(ther);

	return size;
}


static DEVICE_ATTR(poll_delay, 0664,
		dts201a_poll_delay_show, dts201a_poll_delay_store);
static DEVICE_ATTR(enable, 0664,
		dts201a_enable_show, dts201a_enable_store);

static struct attribute *thermopile_sysfs_attrs[] = {
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group thermopile_attribute_group = {
	.attrs = thermopile_sysfs_attrs,
};

static ssize_t dts201a_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s VENDOR = %s\n", __func__, DTS_VENDOR);
#endif

	return sprintf(buf, "%s\n", DTS_VENDOR);
}

static ssize_t dts201a_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s CHIP_ID = %s\n", __func__, DTS_CHIP_ID);
#endif

	return sprintf(buf, "%s\n", DTS_CHIP_ID);
}

#ifdef DTS201A_TEST
void Hex2AsciiValue(u8 inHex, u8 *outAscii)
{
	u8 msb, lsb;
	
	msb = (inHex>>4) & 0x0f;
	lsb = inHex & 0x0f;

	if( ( 0x00 <= msb ) && ( msb <= 0x09 ) )		msb += 0x30;
	else if( ( 0x0A <= msb ) && ( msb <= 0x0F ) )	msb += 0x37;

	if( ( 0x00 <= lsb ) && ( lsb <= 0x09 ) )		lsb += 0x30;
	else if( ( 0x0A <= lsb ) && ( lsb <= 0x0F ) )	lsb += 0x37;

	*(outAscii+0) = msb;
	*(outAscii+1) = lsb;
	*(outAscii+2) = ',';	
}

static int coefficient_save_file(struct dts201a_data *ther)
{
	struct file *offset_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;
	u8 rbuf[3]={0, };
	u8 rdata[42] = {0,};
	int i, buf_size = 3;

	for(i=0x1a; i<=0x2e; i++)
	{
		err = dts201a_i2c_write(ther, i, 0x0);
		if (err < 0) {
			pr_err("%s: i2c write error\n", __func__);
			return err;
		}
		msleep(12);
		err = dts201a_i2c_read(ther, rbuf, 3);
		if (err < 0) {
			pr_err("%s: i2c read error\n", __func__);
			return err;
		}

		rdata[(i-0x1a)+(i-0x1a)]=rbuf[1];	
		rdata[((i-0x1a)+(i-0x1a))+1]=rbuf[2];
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	offset_filp = filp_open(COEFFICIENT_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(offset_filp)) {
		pr_err("%s: Can't open /efs/FactoryApp/coefficient file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(offset_filp);
		return err;
	}

	for(i=0; i<42; i++)
	{
		Hex2AsciiValue(rdata[i], rbuf);

//		pr_info("\t[PARTRON] %s rdata[%d]=0x%02x(%02x_%02x_%02x)\n", __func__,i,rdata[i], rbuf[0], rbuf[1], rbuf[2]);

		err = offset_filp->f_op->write(offset_filp,
			rbuf, buf_size*sizeof(u8), &offset_filp->f_pos);
		if (err < 0) {
			pr_err("%s: Can't write the offset data to file\n", __func__);
			err = -EIO;
		}
	}

	filp_close(offset_filp, current->files);
	set_fs(old_fs);

	msleep(150);	/* delay for clearing */

	return err;
}

static ssize_t coefficient_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);
	int err = 0;
	u8 rbuf[3]={0, };
	u8 rdata[42] = {0,};
	int i;

	for(i=0x1a; i<=0x2e; i++)
	{
		err = dts201a_i2c_write(ther, i, 0x0);
		if (err < 0) {
			pr_err("%s: i2c write error\n", __func__);
			return err;
		}
		msleep(12);
		err = dts201a_i2c_read(ther, rbuf, 3);
		if (err < 0) {
			pr_err("%s: i2c read error\n", __func__);
			return err;
		}

		rdata[(i-0x1a)+(i-0x1a)]=rbuf[1];	
		rdata[((i-0x1a)+(i-0x1a))+1]=rbuf[2];
	}

	for(i=0; i<42; i++)
		pr_info("\t[PARTRON] %s rdata[%d]=0x%02x\n", __func__,i,rdata[i]);

	return sprintf(buf, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
		rdata[0],rdata[1],rdata[2],rdata[3],rdata[4],rdata[5],rdata[6],rdata[7],rdata[8],rdata[9]);
}

static ssize_t coefficient_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);
	int err = 0;

	if (sysfs_streq(buf, "1"))
	{
		err = coefficient_save_file(ther);
		if (err < 0) {
			pr_err("%s: coefficient_save_file failed\n", __func__);
			return err;
		}
	}

	return size;
}
#endif

static DEVICE_ATTR(vendor, 0644, dts201a_vendor_show, NULL);
static DEVICE_ATTR(name, 0644, dts201a_name_show, NULL);
#ifdef DTS201A_TEST
static DEVICE_ATTR(coefficient_reg, 0664, coefficient_reg_show, coefficient_reg_store);
#endif


static enum hrtimer_restart dts201a_timer_func(struct hrtimer *timer)
{
	struct dts201a_data *ther
		= container_of(timer, struct dts201a_data, timer);
	queue_work(ther->thermopile_wq, &ther->work_thermopile);
	hrtimer_forward_now(&ther->timer, ther->poll_delay);

	return HRTIMER_RESTART;
}

static void dts201a_work_func(struct work_struct *work)
{
	int err = 0;
	struct dts201a_data *ther = container_of(work, struct dts201a_data, 
						work_thermopile);

	mutex_lock(&ther->lock);
	err = dts201a_get_thermtemp_data(ther);
	mutex_unlock(&ther->lock);
	if (err < 0)
		dev_err(&ther->client->dev, "get_thermopile_data failed\n");

	input_report_rel(ther->input_dev, REL_X, ther->out.therm);
	input_report_rel(ther->input_dev, REL_Y, ther->out.temp);
	input_report_rel(ther->input_dev, REL_Z, ther->out.status);
	input_sync(ther->input_dev);

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s therm = 0x%06x, temp = 0x%06x\n", 
		__func__, ther->out.therm, ther->out.temp);
#endif

	return;
}

static int dts201a_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct dts201a_data *ther;
	struct dts_platform_data *pdata;
	int err = 0;

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s DEV_NAME = %s\n", __func__, DTS_DEV_NAME);
#endif

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODATA;
		goto err_exit_platform_data_null;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err_exit_check_functionality_failed;
	}

	ther = kzalloc(sizeof(struct dts201a_data), GFP_KERNEL);
	if (ther == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto err_exit_alloc_data_failed;
	}

	pdata = client->dev.platform_data;
	ther->client = client;
	i2c_set_clientdata(client, ther);
	
	mutex_init(&ther->lock);
	mutex_lock(&ther->lock);

	err = dts201a_hw_init(ther);
	if (err < 0)
		goto err_mutexunlockfreedata;

	hrtimer_init(&ther->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ther->poll_delay = ns_to_ktime(DTS_DELAY_DEFAULT * NSEC_PER_MSEC);
	ther->timer.function = dts201a_timer_func;

	ther->thermopile_wq = create_singlethread_workqueue("dts201a_thermopile_wq");
	if (!ther->thermopile_wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}

	INIT_WORK(&ther->work_thermopile, dts201a_work_func);

	ther->input_dev = input_allocate_device();
	if (!ther->input_dev) {
		err = -ENOMEM;
		dev_err(&ther->client->dev, "input device allocate failed\n");
		goto err_input_allocate_device;
	}

	ther->input_dev->name = "thermopile_sensor";
	input_set_drvdata(ther->input_dev, ther);
	input_set_capability(ther->input_dev, EV_REL, REL_X);
	input_set_capability(ther->input_dev, EV_REL, REL_Y);
	input_set_capability(ther->input_dev, EV_REL, REL_Z);

	err = input_register_device(ther->input_dev);
	if (err < 0) {
		dev_err(&ther->client->dev,
			"unable to register input polled device %s\n",
			ther->input_dev->name);
		goto err_input_register_device;
	}

	err = sysfs_create_group(&ther->input_dev->dev.kobj,
				&thermopile_attribute_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group;
	}
/*
	ther->dev = sensors_classdev_register("thermopile_sensor");
	if (IS_ERR(ther->dev)) {
		err = PTR_ERR(ther->dev);
		pr_err("%s: device_create failed[%d]\n", __func__, err);
		goto err_device_create;
	}
*/
	err = device_create_file(ther->dev, &dev_attr_vendor);
	if (err < 0) {
		pr_err("%s: device_create failed(vendor) failed\n",
			__func__);
		goto err_device_create_file2;
	}

	err = device_create_file(ther->dev, &dev_attr_name);
	if (err < 0) {
		pr_err("%s: device_create failed(name) failed\n",
			__func__);
		goto err_device_create_file3;
	}

#ifdef DTS201A_TEST
	if (device_create_file(ther->dev, 	&dev_attr_coefficient_reg) < 0)
	{
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_coefficient_reg.attr.name);
		goto err_device_create_file4;
	}
#endif

	dev_set_drvdata(ther->dev, ther);

	mutex_unlock(&ther->lock);

	return 0;

#ifdef DTS201A_TEST
err_device_create_file4:
	device_remove_file(ther->dev, &dev_attr_coefficient_reg);
#endif	
err_device_create_file3:
	device_remove_file(ther->dev, &dev_attr_name);	
err_device_create_file2:
	device_remove_file(ther->dev, &dev_attr_vendor);
/*
err_device_create:
	sensors_classdev_unregister(ther->dev);
	sysfs_remove_group(&ther->input_dev->dev.kobj,
					&thermopile_attribute_group);
*/
err_sysfs_create_group:
	input_unregister_device(ther->input_dev);
err_input_register_device:
	input_free_device(ther->input_dev);
err_input_allocate_device:
err_create_workqueue:
	destroy_workqueue(ther->thermopile_wq);
err_mutexunlockfreedata:
	mutex_unlock(&ther->lock);
	kfree(ther);
err_exit_alloc_data_failed:
err_exit_check_functionality_failed:
err_exit_platform_data_null:
	pr_err("%s: Driver Init failed\n", DTS_DEV_NAME);
	return err;
}

static int dts201a_remove(struct i2c_client *client)
{
	struct dts201a_data *ther = i2c_get_clientdata(client);

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %d\n", __func__, 0);
#endif

#ifdef DTS201A_TEST
	device_remove_file(ther->dev, &dev_attr_coefficient_reg);
#endif

	device_remove_file(ther->dev, &dev_attr_name);
	device_remove_file(ther->dev, &dev_attr_vendor);
	//sensors_classdev_unregister(ther->dev);
	sysfs_remove_group(&ther->input_dev->dev.kobj,
				&thermopile_attribute_group);
	input_unregister_device(ther->input_dev);
	dts201a_disable(ther);
	flush_workqueue(ther->thermopile_wq);
	destroy_workqueue(ther->thermopile_wq);
	input_free_device(ther->input_dev);
	mutex_destroy(&ther->lock);
	kfree(ther);

	return 0;
}

static int dts201a_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dts201a_data *ther = i2c_get_clientdata(client);
	int ret = 0;

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %d\n", __func__, 0);
#endif

	if (!atomic_read(&ther->enabled)) {
		ret = dts201a_enable(ther);
		if (ret < 0)
			pr_err("%s: could not enable\n", __func__);
	}

	return ret;
}

static int dts201a_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dts201a_data *ther = i2c_get_clientdata(client);
	int ret = 0;

#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %d\n", __func__, 0);
#endif

	if (atomic_read(&ther->enabled)) {
		ret = dts201a_disable(ther);
		if (ret < 0)
			pr_err("%s: could not disable\n", __func__);
	}
	
	return ret;
}

static const struct i2c_device_id dts201a_id[] = { 
	{ DTS_DEV_NAME, 0}, 
	{ },
};

MODULE_DEVICE_TABLE(i2c, dts201a_id);

static const struct dev_pm_ops dts201a_pm_ops = {
	.suspend = dts201a_suspend,
	.resume = dts201a_resume
};

static struct i2c_driver dts201a_driver = {
	.driver = {
			.name = DTS_DEV_NAME,
			.owner = THIS_MODULE,
			.pm = &dts201a_pm_ops
	},
	.probe = dts201a_probe,
	.remove = dts201a_remove,
	.id_table = dts201a_id,
};

static int __init dts201a_init(void)
{
#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %s\n", __func__, DTS_DEV_NAME);
#endif

	return i2c_add_driver(&dts201a_driver);
}

static void __exit dts201a_exit(void)
{
#ifdef dts201A_DEBUG
	pr_info("\t[PARTRON] %s = %s\n", __func__, DTS_DEV_NAME);
#endif

	i2c_del_driver(&dts201a_driver);
	return;
}

module_init(dts201a_init);
module_exit(dts201a_exit);

MODULE_DESCRIPTION("PARTRON dts201a thermopile sensor sysfs driver");
MODULE_AUTHOR("PARTRON");
MODULE_LICENSE("GPL");
