/*
 * drivers/misc/mediatek/sensors-1.0/humidity/aht10/aht10.c
 *
 * aht10 temerature driver
 *
 * Copyright 2020, jimmy <lijiaming@knowin.com>
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/async.h>
#include <linux/ioport.h>
#include <linux/wakelock.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <linux/irq.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include "humidity.h"
#include "cust_hmdy.h"


/////////////////////////////////////////////////

#define AHT_TAG                  "[AHT10] "
#define AHT_FUN(f)               //printk(KERN_INFO AHT_TAG"%s\n", __FUNCTION__)
#define AHT_ERR(fmt, args...)    printk(KERN_ERR  AHT_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define AHT_INFO(fmt, args...)   //printk(KERN_ERR AHT_TAG fmt, ##args)
#define AHT_WARN(fmt, args...)   printk(KERN_WARNING AHT_TAG fmt, ##args)

/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AHT_DRV_NAME            "aht10"
#define AHT_DEV_BASENAME        "aht10"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AHT_IOC_MAGIC              'A'
#define AHT_IOC_INIT              _IOW(AHT_IOC_MAGIC, 1, int)   /* set init */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define AHT10_INIT_CMD          0xE1 // initialize cmd for measuring
#define AHT10_MEASURE_CMD       0xAC // tigger measure cmd
#define AHT10_NORMAL_CMD        0xA8 // normal cmd
#define AHT10_RESET_CMD         0xBA // soft reset cmd

#define AHT10_BUFSIZE           128

static int aht10_local_init(void);
static int aht10_local_uninit(void);

static struct hmdy_init_info aht10_init_info = {
		.name = "aht10",
		.init = aht10_local_init,
		.uninit = aht10_local_uninit,
	
};

struct aht10_dev_data {

    struct wake_lock work_wake_lock;
    struct i2c_client *client;
	struct hmdy_hw *hw;
    struct miscdevice miscdev;
    //struct input_dev *input;

    atomic_t opened;
    struct work_struct init_work;
    atomic_t is_init;

    int cur_temp;
    int cur_humi;
};

static int	aht10_init_flag = -1;	// 0<==>OK -1 <==> fail
static struct aht10_dev_data *aht10_data;
struct hmdy_hw hmdy_cust;
static struct hmdy_hw *hw = &hmdy_cust;


static int aht10_reg_read(const struct i2c_client *client,
                          int len, u8 *val)
{
    int ret;
    struct i2c_msg msg[1];
	AHT_FUN();

    msg[0].addr   = client->addr;
    msg[0].flags  = client->flags & I2C_M_TEN;
    msg[0].flags |= I2C_M_RD;
    msg[0].len    = len;
    msg[0].buf    = val;

    ret = i2c_transfer(client->adapter, msg, 1);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c read error\n", __func__);
    return ret;
}


static int aht10_cmd_write(const struct i2c_client *client,
                           u8 cmd)
{
    int ret;
    struct i2c_msg msg[1];

    msg[0].addr   = client->addr;
    msg[0].flags  = client->flags & I2C_M_TEN;
    msg[0].len    = 1;
    msg[0].buf    = &cmd;

    ret = i2c_transfer(client->adapter, msg, 1);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c cmd write error\n", __func__);
    return ret;
}

#if 0
static int aht10_i2c_read(const struct i2c_client *client,
                          u8 addr, int len, u8 *buf)
{
    int ret;
    struct i2c_msg msg[2];

    msg[0].addr   = client->addr;
    msg[0].flags  = client->flags & I2C_M_TEN;
    msg[0].len    = 1;
    msg[0].buf    = &addr;

    msg[1].addr   = client->addr;
    msg[1].flags  = client->flags & I2C_M_TEN;
    msg[1].flags |= I2C_M_RD;
    msg[1].len    = len;
    msg[1].buf    = buf;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c read error\n", __func__);
    return ret;
}
#endif

static int aht10_i2c_write(const struct i2c_client *client,
                           u8 addr, int len, u8 *buf)
{
    int ret;
    struct i2c_msg msg;
    u8 *send_buf;

    send_buf = kzalloc(len + 1, GFP_KERNEL);
    if (send_buf == NULL) {
        AHT_ERR("Failed to malloc buf\n");
        return -ENOMEM;
    }

    send_buf[0] = addr;
    memcpy(&send_buf[1], buf, len);

    msg.addr   = client->addr;
    msg.flags  = client->flags & I2C_M_TEN;
    msg.len    = len + 1;
    msg.buf    = send_buf;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c write error\n", __func__);

    kfree(send_buf);

    return ret;
}


int aht10_enable(const struct i2c_client *client)
{
    u8 buf[2];
	AHT_FUN();

#if 0
    buf[0] = 0;
    buf[1] = 0;
    aht10_i2c_write(client, AHT10_NORMAL_CMD, 2, buf);
    usleep_range(500 * 1000, 500 * 1000 + 100);
#endif

    buf[0] = 0x08;
    buf[1] = 0x00;
    aht10_i2c_write(client, AHT10_INIT_CMD, 2, buf); // calibrate
    usleep_range(500 * 1000, 500 * 1000 + 100);

    return 0;
}

int aht10_get_state(const struct i2c_client *client)
{
    int ret;
    u8 buf[1] = {0};
	AHT_FUN();

    ret = aht10_reg_read(client, 1, buf);
    if (ret <= 0)
        return -1;
    else
        return buf[0];
}


bool aht10_get_cal_enable(const struct i2c_client *client)
{
	int val = aht10_get_state(client);

    if (val > 0) {
        if ((val & 0x68) == 0x08)
            return true;
        else
            return false;
    }

    return false;
}

bool aht10_get_is_busy(const struct i2c_client *client)
{
    int val = aht10_get_state(client);
	AHT_FUN();
    
    if (val > 0) {
        if ((val & 0x80) == 0x80)
            return true;
        else
            return false;
    }

    return false;
}


void aht10_soft_reset(const struct i2c_client *client)
{
    aht10_cmd_write(client, AHT10_RESET_CMD);
}


int aht10_get_temp(const struct i2c_client *client, int *temp)
{
    u8 buf[6];
    bool is_busy = false;
    u16 try_count = 0;

    buf[0] = 0x33;
    buf[1] = 0x00;
    aht10_i2c_write(client, AHT10_MEASURE_CMD, 2, buf); // trigger measure
    usleep_range(75 * 1000, 75 * 1000 + 100);

    is_busy = aht10_get_is_busy(client);
    while (is_busy && try_count < 100) {
        try_count++;
        usleep_range(1 * 1000, 1 * 1000 + 100);
        is_busy = aht10_get_is_busy(client);
    }

    //AHT_INFO("%s: is_busy=%d\n", __func__, is_busy);

    if (!is_busy) {
        aht10_reg_read(client, 6, buf);
        /*
        AHT_INFO("%s: data: [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]\n", __func__,
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        */
        *temp = ((buf[3] & 0xF) << 16 | buf[4] << 8 | buf[5]) * 10 * 200 / (1 << 20) - 500;
        return 0;
    }

    return -1;
}


int aht10_get_humi(const struct i2c_client *client, int *humi)
{
    u8 buf[6];
    bool is_busy = false;
    u16 try_count = 0;

    buf[0] = 0x33;
    buf[1] = 0x00;
    aht10_i2c_write(client, AHT10_MEASURE_CMD, 2, buf); // trigger measure
    usleep_range(75 * 1000, 75 * 1000 + 100);

    is_busy = aht10_get_is_busy(client);
    while (is_busy && try_count < 100) {
        try_count++;
        usleep_range(1 * 1000, 1 * 1000 + 100);
        is_busy = aht10_get_is_busy(client);
    }

    //AHT_INFO("%s: is_busy=%d\n", __func__, is_busy);

    if (!is_busy) {
        aht10_reg_read(client, 6, buf);
        /*
        AHT_INFO("%s: data: [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]\n", __func__,
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        */
        *humi = (buf[1] << 12 | buf[2] << 4 | (buf[3] & 0xf0) >> 4) * 10 * 100 / (1 << 20);
        return 0;
    }

    return -1;
}



static void aht10_init_events(struct work_struct *work)
{
    struct aht10_dev_data *dev_data = container_of(work, struct aht10_dev_data, init_work);
    bool cal_enable = false;
    int try_count = 0;

    AHT_INFO("%s Enter!\n", __func__);

    while (!cal_enable && try_count <= 3) {
        aht10_enable(dev_data->client);
        try_count++;
        msleep(2000); 
        cal_enable = aht10_get_cal_enable(dev_data->client);

        AHT_INFO("%s: try [%d] cal_enable=%d.\n", __func__, try_count, cal_enable);

        if (cal_enable) {
            atomic_set(&dev_data->is_init, true);
        }
        else {
            atomic_set(&dev_data->is_init, false);
        }
    }

    return;
}


static int aht_dev_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct aht10_dev_data *dev_data = container_of(miscdev, struct aht10_dev_data, miscdev);

    if (atomic_read(&dev_data->opened)) {
        pr_err("EBUSY\n");
        return -EBUSY;
    }

    atomic_inc(&dev_data->opened);

    return 0;
}

static int aht_dev_release(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct aht10_dev_data *dev_data = container_of(miscdev, struct aht10_dev_data, miscdev);

    atomic_dec(&dev_data->opened);

    return 0;
}

static long aht_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *miscdev = filp->private_data;
    struct aht10_dev_data *dev_data = container_of(miscdev, struct aht10_dev_data, miscdev);

    //unsigned char var = (unsigned char)arg;

    switch(cmd) {
    case AHT_IOC_INIT:
        AHT_INFO("AHT_IOC_INIT, arg=%d\n", arg);
        break;

    default:
        dev_err(&dev_data->client->dev, "Not supported CMD:0x%x\n", cmd);
        return -EINVAL;
    }
    return 0;
}

static struct file_operations aht_dev_fops = {
    .owner          = THIS_MODULE,
    .open           = aht_dev_open,
    .release        = aht_dev_release,
    .unlocked_ioctl = aht_dev_ioctl,
};


/*----------------------------------------------------------------------------*/

static ssize_t aht10_show_temp(struct device_driver *ddri, char *buf)
{
    int res = -EINVAL;
    struct aht10_dev_data *dev_data = aht10_data;
    bool cal_enable = false;

    cal_enable = atomic_read(&dev_data->is_init);
    if (!cal_enable) {
        aht10_enable(dev_data->client);
        cal_enable = aht10_get_cal_enable(dev_data->client);
        if (cal_enable) {
            atomic_set(&dev_data->is_init, true);
        }
    }

    if (!cal_enable || (res = aht10_get_temp(dev_data->client, &dev_data->cur_temp))) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        //usleep_range(100, 200);
        return sprintf(buf, "%d\n", dev_data->cur_temp);
    }
}

static ssize_t aht10_show_humi(struct device_driver *ddri, char *buf)
{
    int res = -EINVAL;
    struct aht10_dev_data *dev_data = aht10_data;
    bool cal_enable = false;

    cal_enable = atomic_read(&dev_data->is_init);
    if (!cal_enable) {
        aht10_enable(dev_data->client);
        cal_enable = aht10_get_cal_enable(dev_data->client);
        if (cal_enable) {
            atomic_set(&dev_data->is_init, true);
        }
    }

    if(!cal_enable || (res = aht10_get_humi(dev_data->client, &dev_data->cur_humi))) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        //usleep_range(100, 200);
        return sprintf(buf, "%d\n", dev_data->cur_humi);
    }
}

/*----------------------------------------------------------------------------*/

static ssize_t aht10_store_init(struct device_driver *ddri, const char *buf, size_t count)
{
    struct aht10_dev_data *dev_data = aht10_data;
    unsigned long value = 0;
    int ret;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        AHT_ERR( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }
    AHT_INFO("%s: init value : %d\n", __func__, (int)value);

    if (value) {
        aht10_enable(dev_data->client);
    }

    return count;
}

/*----------------------------------------------------------------------------*/

static ssize_t aht10_show_init(struct device_driver *ddri, char *buf)
{
    struct aht10_dev_data *dev_data = aht10_data;
    bool cal_enable = aht10_get_cal_enable(dev_data->client);

    return sprintf(buf, "%d\n", cal_enable ? 1 : 0);
}

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(temp, S_IRUGO, aht10_show_temp, NULL);
static DRIVER_ATTR(humi, S_IRUGO, aht10_show_humi, NULL);
static DRIVER_ATTR(init, S_IWUSR | S_IRUGO, aht10_show_init, aht10_store_init);

static struct driver_attribute *aht10_attr_list[] = {
	&driver_attr_temp,
	&driver_attr_humi,
    &driver_attr_init,
};

static int aht10_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(aht10_attr_list) / sizeof(aht10_attr_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = driver_create_file(driver, aht10_attr_list[idx]))) {
            pr_err("driver_create_file (%s) = %d\n", aht10_attr_list[idx]->attr.name, err);
            break;
        }
    }

    return err;
}

static int aht10_delete_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(aht10_attr_list) / sizeof(aht10_attr_list[0]));

    if (NULL == driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        driver_remove_file(driver, aht10_attr_list[idx]);
    }

    return err;
}

static int aht10_open_report_data(int open)
{
	return 0;
}

static int aht10_enable_nodata(int en)
{
	AHT_INFO("auto enable when chip boot up, no register to control\n");
	return 0;
}

static int aht10_set_delay(u64 ns)
{
	AHT_INFO("in aht10_set_delay\n");
	return 0;
}

static int aht10_get_humidity(struct i2c_client *client, char *buf, int bufsize)
{
	int res = -EINVAL;
	struct aht10_dev_data *dev_data = i2c_get_clientdata(client);
	bool cal_enable = false;

	AHT_FUN();

    cal_enable = atomic_read(&dev_data->is_init);
    if (!cal_enable) {
        aht10_enable(dev_data->client);
        cal_enable = aht10_get_cal_enable(dev_data->client);
        if (cal_enable) {
            atomic_set(&dev_data->is_init, true);
        }
    }

    if(!cal_enable || (res = aht10_get_humi(dev_data->client, &dev_data->cur_humi))) {
		AHT_INFO("get humi failed, res=%d\n", res);
		return -1;
    } else {
        //usleep_range(100, 200);
        sprintf(buf, "%d\n", dev_data->cur_humi);
		return 0;
    }
}

static int aht10_get_temperature(struct i2c_client *client, char *buf, int bufsize)
{
	int res = -EINVAL;
	struct aht10_dev_data *dev_data = i2c_get_clientdata(client);
	bool cal_enable = false;

	AHT_FUN();

    cal_enable = atomic_read(&dev_data->is_init);
    if (!cal_enable) {
        aht10_enable(dev_data->client);
        cal_enable = aht10_get_cal_enable(dev_data->client);
        if (cal_enable) {
            atomic_set(&dev_data->is_init, true);
        }
    }

    if(!cal_enable || (res = aht10_get_temp(dev_data->client, &dev_data->cur_temp))) {
		AHT_INFO("get temp failed, res=%d\n", res);
		return -1;
    } else {
        //usleep_range(100, 200);
        sprintf(buf, "%d\n", dev_data->cur_temp);
		return 0;
    }
}

static int aht10_get_humidity_data(int *value, int *status)
{
	char buff[AHT10_BUFSIZE];
	int err = 0;

	AHT_FUN();
	err = aht10_get_humidity(aht10_data->client, buff, AHT10_BUFSIZE);
	if (err) {
		AHT_INFO("get compensated humidity value failed," "err = %d\n", err);
		return -1;
	}
	err = kstrtoint(buff, 10, value);
	//AHT_INFO("humidity data is: %d\n", *value);
	if (err == 0)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int aht10_get_temperature_data(int *value, int *status)
{
	char buff[AHT10_BUFSIZE];
	int err = 0;

	//AHT_FUN();
	err = aht10_get_temperature(aht10_data->client, buff, AHT10_BUFSIZE);
	if (err) {
		AHT_INFO("get compensated temperature value failed," "err = %d\n", err);
		return -1;
	}
	err = kstrtoint(buff, 10, value);
	//AHT_INFO("temperature data is: %d\n", *value);
	if (err == 0)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}


static int aht10_i2c_probe(struct i2c_client *client,
                           const struct i2c_device_id *did)
{
    int ret = 0;
    struct aht10_dev_data *dev_data;
	struct hmdy_control_path hmdy_control_path = {0};
    struct hmdy_data_path hmdy_data_path = {0};
	struct temp_control_path temp_control_path = {0};
    struct temp_data_path temp_data_path = {0};
    struct device_node *np = client->dev.of_node;


    AHT_INFO("%s enter!\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality not supported!\n");
        return -ENODEV;
    }

    if (!np) {
        dev_err(&client->dev, "no device tree\n");
        return -EINVAL;
    }

	//AHT_INFO("ljm === np->name: %s\n", np->name);

	ret = get_hmdy_dts_func("asair,aht10", hw);
	if (0 > ret)
    {
        AHT_INFO("Dts info fail");
        return -EINVAL;
    }

    dev_data = kzalloc(sizeof(struct aht10_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        pr_err("Failed to malloc aht10_dev_data\n");
        return -ENOMEM;
    }

    atomic_set(&dev_data->is_init, false);

	aht10_data = dev_data;
	dev_data->hw = hw;

    dev_data->miscdev.minor = MISC_DYNAMIC_MINOR;
    dev_data->miscdev.name  = client->name;
    dev_data->miscdev.fops  = &aht_dev_fops;

    if (misc_register(&dev_data->miscdev) < 0) {
        dev_err(&client->dev, "misc_register failed\n");
        goto err_misc_register;
    }

	ret = aht10_create_attr(&(aht10_init_info.platform_diver_addr->driver));
	if (ret) {
		AHT_INFO("create attribute failed, ret = %d\n", ret);
		goto exit_create_attr_failed;
	}

	hmdy_control_path.is_use_common_factory = false;
	hmdy_control_path.open_report_data = aht10_open_report_data;
	hmdy_control_path.enable_nodata = aht10_enable_nodata;
	hmdy_control_path.set_delay = aht10_set_delay;
	hmdy_control_path.is_report_input_direct = false;
	hmdy_control_path.is_support_batch = dev_data->hw->is_batch_supported_hmdy;

	ret = hmdy_register_control_path(&hmdy_control_path);
	if (ret) {
		AHT_INFO("register hmdy control path err\n");
		goto exit_delete_attr;
	}

	hmdy_data_path.get_data = aht10_get_humidity_data;
	hmdy_data_path.vender_div = 10;
	ret = hmdy_register_data_path(&hmdy_data_path);
	if (ret) {
		AHT_INFO("hmdy_register_data_path failed, ret = %d\n", ret);
		goto exit_delete_attr;
	}

	temp_control_path.is_use_common_factory = false;
	temp_control_path.open_report_data = aht10_open_report_data;
	temp_control_path.enable_nodata = aht10_enable_nodata;
	temp_control_path.set_delay = aht10_set_delay;
	temp_control_path.is_report_input_direct = false;
	temp_control_path.is_support_batch = dev_data->hw->is_batch_supported_temp;

	ret = temp_register_control_path(&temp_control_path);
	if (ret) {
		AHT_INFO("register temp control path err\n");
		goto exit_delete_attr;
	}

	temp_data_path.get_data = aht10_get_temperature_data;
	temp_data_path.vender_div = 10;
	ret = temp_register_data_path(&temp_data_path);
	if (ret) {
		AHT_INFO("temp_register_data_path failed, ret = %d\n", ret);
		goto exit_delete_attr;
	}

	aht10_init_flag = 0;

    // init work queue
    INIT_WORK(&dev_data->init_work, aht10_init_events);

    dev_data->client = client;
    wake_lock_init(&dev_data->work_wake_lock, WAKE_LOCK_SUSPEND, "aht");

    i2c_set_clientdata(client, dev_data);

    schedule_work(&dev_data->init_work);

    return 0;

exit_delete_attr:
	aht10_delete_attr(&(aht10_init_info.platform_diver_addr->driver));

exit_create_attr_failed:
	misc_deregister(&dev_data->miscdev);

err_misc_register:
    kfree(dev_data);
	aht10_init_flag = -1;

    return ret;
}

static int aht10_i2c_remove(struct i2c_client *client)
{
    struct aht10_dev_data *dev_data = i2c_get_clientdata(client);

    wake_lock_destroy(&dev_data->work_wake_lock);
    misc_deregister(&dev_data->miscdev);
    aht10_delete_attr(&(aht10_init_info.platform_diver_addr->driver));
    kfree(dev_data);

    return 0;
}

static const struct i2c_device_id aht10_ids[] = {
    {AHT_DEV_BASENAME, 0},
    {/*end of list*/}
};

MODULE_DEVICE_TABLE(i2c, aht10_ids);

#ifdef CONFIG_PM

int aht10_suspend(struct device *dev)
{
    /*
        struct i2c_client *client = to_i2c_client(dev);
        struct aht10_dev_data *dev_data = i2c_get_clientdata(client);
    */

    return 0;
}

int aht10_resume(struct device *dev)
{
    /*
        struct i2c_client *client = to_i2c_client(dev);
        struct aht10_dev_data *dev_data = i2c_get_clientdata(client);
    */

    return 0;
}

static SIMPLE_DEV_PM_OPS(aht10_i2c_pm, aht10_suspend, aht10_resume);
#endif

static struct of_device_id aht10_dt_ids[] = {
    { .compatible = "asair,aht10" },
    { }
};

static struct i2c_driver aht10_i2c_driver = {
    .driver = {
        .name  = AHT_DRV_NAME,
#ifdef CONFIG_PM
        .pm    = &aht10_i2c_pm,
#endif
        .of_match_table = of_match_ptr(aht10_dt_ids),
    },
    .probe      = aht10_i2c_probe,
    .remove     = aht10_i2c_remove,
    .id_table   = aht10_ids,
};

static int aht10_local_init(void)
{
    AHT_INFO("aht10_local_init\n");
    if (i2c_add_driver(&aht10_i2c_driver)) {
		AHT_INFO("Add i2c driver failed\n");
		return -1;
	}
	if (-1 == aht10_init_flag) {
		AHT_INFO("aht10_init_flag equal to -1\n");
		return -1;
	}
	return 0;
}

static int aht10_local_uninit(void)
{
    AHT_FUN();
    i2c_del_driver(&aht10_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init aht10_init(void)
{
	hmdy_driver_add(&aht10_init_info);
	AHT_FUN();   
	return 0;
}

static void __exit aht10_exit(void)
{
	AHT_INFO("aht10_exit\n"); 
	AHT_FUN();
}


module_init(aht10_init);
module_exit(aht10_exit);


MODULE_AUTHOR("jimmy <lijiaming@knowin.com>");
MODULE_DESCRIPTION("ASAIR aht10 temperature/humidity Driver");
MODULE_LICENSE("GPL");
