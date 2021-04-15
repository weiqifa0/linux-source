/*
 * linux/drivers/misc/em20918.c
 *
 * em20918 control interface driver
 *
 * Copyright 2018, Howrd <l.tao@onetolink.com>
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
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/sort.h>
#if 0
#include <linux/regulator/consumer.h>
#endif
#include "em20918.h"
#include <cust_alsps.h>
#include <alsps.h>

#define DRV_NAME                         "ir_em"
#define IR_EM_DEV_BASENAME               "ir_em"

#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)              //printk(KERN_INFO APS_TAG"%s %d \n", __FUNCTION__ , __LINE__)
#define APS_ERR(fmt, args...)   printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   //printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   //printk(KERN_ERR APS_TAG fmt, ##args)    


//#define IR_DETECT_OFF_POWER

#define IR_DETECT_IOC_MAGIC              'I'
#define IR_DETECT_IOC_POWER              _IOW(IR_DETECT_IOC_MAGIC, 1, int)   /* set power */
#define IR_DETECT_IOC_IR_INT             _IOR(IR_DETECT_IOC_MAGIC, 2, int)   /* get ir-int value */
#define IR_DETECT_IOC_WAKEUP_EN          _IOW(IR_DETECT_IOC_MAGIC, 3, int)   /* enable wakeup */
#define IR_DETECT_IOC_CAPTURE_EN         _IOR(IR_DETECT_IOC_MAGIC, 4, int)   /* enable capture */
#define IR_DETECT_IOC_PROX_CAL           _IOR(IR_DETECT_IOC_MAGIC, 5, int)   /* proximity calibrate */
#define IR_DETECT_IOC_PROX_SET           _IOW(IR_DETECT_IOC_MAGIC, 6, int)   /* set proximity value */


#define IR_DETECT_POWER_ON               1
#define IR_DETECT_POWER_OFF              0

#define PS_CAL_COUNT                     (10) // min: 5

#if (PS_CAL_COUNT < 5)
#error "The calibrate count should be 5 at least!\n"
#endif

enum {
    INIT_WORK_SCHED    = (1 << 0),     /* init work scheduled or running */
    INT_WORK_SCHED     = (1 << 1),     /* interrupt work scheduled or running */
    CAP_DETECT_CANCEL  = (1 << 2),     /* capture detect flag */
    CAL_WORK_SCHED     = (1 << 3),     /* calibrate work scheduled or running */
    PROX_CALI_CANCEL   = (1 << 4),     /* proximity cancel flag */
};

struct em20918_priv {
    int irq;

    struct em20918_platform_data *pdata;
    struct wake_lock work_wake_lock;
    struct i2c_client *client;
    struct miscdevice miscdev;
    struct input_dev *input;
	struct alsps_hw *hw;

    atomic_t opened;

    bool is_init_ok;
    wait_queue_head_t init_wait;
    struct work_struct init_work;

    struct work_struct int_work;
    unsigned long wsched;
    struct mutex irq_lock;

    bool irq_enable;
    bool wakeup_en;
    bool capture_en;

    int near_threshold;
    bool calibrating;
    bool is_cal_ok;
    wait_queue_head_t cal_wait;
    struct work_struct cal_work;

};

static int	em20918_init_flag = -1;	// 0<==>OK -1 <==> fail
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
static struct em20918_priv *em20918_obj = NULL;

#ifdef IR_DETECT_OFF_POWER
static void ir_em_power(struct em20918_priv *obj, u8 onoff);
#else
static void ir_em_sleep(struct em20918_priv *obj, u8 is_sleep);
#endif
static int em20918_local_init(void);
static int em20918_local_uninit(void);

static DEFINE_MUTEX(sensor_lock);
static struct alsps_init_info em20918_init_info = {
		.name = "em20918",
		.init = em20918_local_init,
		.uninit = em20918_local_uninit,
	
};


static int em20918_i2c_read_page(const struct i2c_client *client, u8 reg_addr, u8 *val, u32 len)
{
    int ret;
    struct i2c_msg msg[2];

    memset(msg, 0x00, sizeof(msg));

    msg[0].addr   = client->addr;
    msg[0].flags  = client->flags & I2C_M_TEN;
    msg[0].len    = 1;
    msg[0].buf    = &reg_addr;

    msg[1].addr   = client->addr;
    msg[1].flags  = client->flags & I2C_M_TEN;
    msg[1].flags |= I2C_M_RD;
    msg[1].len    = len;
    msg[1].buf    = val;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c read error\n", __func__);
    return ret;
}

static int em20918_reg_read(const struct i2c_client *client, u8 addr, u8 *val)
{
#if 0
    //*val = i2c_smbus_read_byte_data(client, reg_addr);

    int ret;
    struct i2c_msg msg[2];

    msg[0].addr   = client->addr;
    msg[0].flags  = client->flags & I2C_M_TEN;
    msg[0].len    = 1;
    msg[0].buf    = &addr;

    msg[1].addr   = client->addr;
    msg[1].flags  = client->flags & I2C_M_TEN;
    msg[1].flags |= I2C_M_RD;
    msg[1].len    = 1;
    msg[1].buf    = val;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c read error\n", __func__);
    return ret;
#else
    int ret;
    u8 tmp[2];

    ret = em20918_i2c_read_page(client, addr, tmp, 2);
    *val = tmp[0];

    return ret;
#endif
}

static int em20918_reg_write(const struct i2c_client *client,
                             u8 addr, u8 val)
{
    int ret;
    struct i2c_msg msg;
    u8 send[2];

    send[0]   = addr;
    send[1]   = val;

    msg.addr  = client->addr;
    msg.flags = client->flags;
    msg.len   = 2;
    msg.buf   = send;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c write error\n", __func__);

    return ret;
}


int em20918_read_ps_data(const struct i2c_client *client, u8 *ps)
{
    u16 i;
    int ret;

    for (i = 0; i < 2; i++) {
        ret = em20918_reg_read(client, EM20918_PS_DATA_REG, ps);
        if (ret > 0) {
            return ret;
        }

        usleep_range(100*1000, 100*1000+100);
    }
    return -1;
}

static void em20918_init_events(struct work_struct *work)
{
    u8 val;
    struct em20918_priv *obj = container_of(work, struct em20918_priv, init_work);
    //struct em20918_platform_data *pdata = obj->pdata;
    int near_thres = obj->near_threshold;

    // soft reset
    em20918_reg_write(obj->client, EM20918_RESET_REG, RESET_REG_RST);
    em20918_reg_write(obj->client, EM20918_OFFSET_REG, OFFSET_REG_RST);
    // clear interrupt
    em20918_reg_write(obj->client, EM20918_INTERRUPT_REG, 0x00);
    // low threshold
    em20918_reg_write(obj->client, EM20918_PS_LT_REG, 0x07);
    // normal mode
    em20918_reg_write(obj->client, EM20918_OFFSET_REG, 0x00);

    // high threshold
    if (near_thres > 0) {
        em20918_reg_write(obj->client, EM20918_PS_HT_REG, near_thres);
    } else {
        pr_err("prox thres invalid value: %d!\n", near_thres);
        if (!obj->calibrating) {
            goto out;
        }
    }

    APS_LOG("== weiqifa === %s:%d\n", __func__, __LINE__);

    // power on & enable interrupt at last
    val = CONFIG_REG_PS_EN | CONFIG_REG_PS_SLP | CONFIG_REG_PS_DR_200MA;
    em20918_reg_write(obj->client, EM20918_CONFIG_REG, val);

    obj->is_init_ok = true;

out:
    if (waitqueue_active(&obj->init_wait)) {
        wake_up(&obj->init_wait);
    }

    clear_bit(INIT_WORK_SCHED, &obj->wsched);

    return;
}

void em20918_set_irq(struct em20918_priv *obj, bool is_enable)
{
    mutex_lock(&obj->irq_lock);

    if (is_enable) {
        if (!obj->irq_enable) {
            enable_irq(obj->irq);
            obj->irq_enable = true;
        }
    } else {
        if (obj->irq_enable) {
            disable_irq(obj->irq);
            obj->irq_enable = false;
        }
    }

    mutex_unlock(&obj->irq_lock);
}

static void em20918_reset_config(struct em20918_priv *obj)
{
    // soft reset
    em20918_reg_write(obj->client, EM20918_RESET_REG, RESET_REG_RST);
    em20918_reg_write(obj->client, EM20918_OFFSET_REG, OFFSET_REG_RST);

    // clear interrupt
    em20918_reg_write(obj->client, EM20918_INTERRUPT_REG, 0x00);
    if (obj->near_threshold > 0) {
        em20918_reg_write(obj->client, EM20918_PS_LT_REG, 0x07);
        em20918_reg_write(obj->client, EM20918_PS_HT_REG, obj->near_threshold);
    }

    // normal mode
    em20918_reg_write(obj->client, EM20918_OFFSET_REG, 0x00);
}

static void em20918_int_workhandler(struct work_struct *work)
{
    struct em20918_priv *obj =
        container_of(work, struct em20918_priv, int_work);
    uint8_t reg_data;
    u8 ps_val;
    int i, ps_near_num = 0;
    int ret;

    wake_lock(&obj->work_wake_lock);

    if (!obj->wakeup_en && obj->capture_en) {
        pr_info("em20918 ==> report mute key...\n");
        input_report_key(obj->input, KEY_MUTE, 1);
        input_sync(obj->input);
        input_report_key(obj->input, KEY_MUTE, 0);
        input_sync(obj->input);
    }

    //em20918_set_irq(obj, false);

    // read interrupt status
    ret = em20918_reg_read(obj->client, EM20918_INTERRUPT_REG, &reg_data);
    pr_info("em20918 interrupt happen, interrupt=0x%02X\n", reg_data);

    // clear interrupt
    em20918_reg_write(obj->client, EM20918_INTERRUPT_REG, 0x00);

    // Indicates that the DATA_PS is higher than PIHT.
    if(reg_data & INTERRUPT_REG_PS_FLAG) {

        ret = em20918_reg_read(obj->client, EM20918_PS_DATA_REG, &ps_val);

        //pr_info("em20918: ps=0x%02X\n", ps_val);

        if (ps_val >= obj->near_threshold) { // near
            pr_info("em20918 ==> something near...\n");

            if (obj->wakeup_en) {
                pr_info("em20918 ==> report wakeup key...\n");
                input_report_key(obj->input, KEY_WAKEUP, 1);
                input_sync(obj->input);
                input_report_key(obj->input, KEY_WAKEUP, 0);
                input_sync(obj->input);
            }
        } else { // far
            pr_info("em20918 ==> something far...\n");
            goto out;
        }

    }


    if (obj->capture_en) {
        u8 config;
        pr_info("em20918 ==> begin to check capture...\n");

        em20918_reg_read(obj->client, EM20918_CONFIG_REG, &config);
        reg_data = config & ~CONFIG_REG_PS_SLP;
        if (reg_data != config) {
            em20918_reset_config(obj);
            em20918_reg_write(obj->client, EM20918_CONFIG_REG, reg_data);
        }

        for (i = 0; i < 60; i++) {
            ret = em20918_read_ps_data(obj->client, &ps_val);
            if (ps_val >= obj->near_threshold) {
                ps_near_num++;
            }
            //pr_info("em20918 ==> ps=%d, %d\n", ps_val, ps_near_num);
            if (ps_near_num > 36) {
                break;
            }

            if (test_bit(CAP_DETECT_CANCEL, &obj->wsched)) {
                goto cancel_cap;
            }
            usleep_range(100*1000, 100*1000+100);
        }

        if (ps_near_num > 36) {
            pr_info("em20918 ==> report camera key...\n");
            input_report_key(obj->input, KEY_CAMERA, 1);
            input_sync(obj->input);
            input_report_key(obj->input, KEY_CAMERA, 0);
            input_sync(obj->input);
        }

	APS_LOG("== weiqifa === %s:%d\n", __func__, __LINE__);
cancel_cap:
        em20918_reset_config(obj);
        em20918_reg_write(obj->client, EM20918_CONFIG_REG, config);
    }

out:

#if 0
    if (obj->wakeup_en || obj->capture_en) {
        em20918_set_irq(obj, true);
    }
#endif

    wake_unlock(&obj->work_wake_lock);
    clear_bit(INT_WORK_SCHED, &obj->wsched);
}

static int compare_val(const void *ap, const void *bp)
{
    const int *a = ap;
    const int *b = bp;
    return (int)(*a - *b);
}

static void s32_swap(void *a, void *b, int size)
{
    s32 t = *(s32 *)a;
    *(s32 *)a = *(s32 *)b;
    *(s32 *)b = t;
}

static void em20918_cali_workhandler(struct work_struct *work)
{
    struct em20918_priv *obj =
        container_of(work, struct em20918_priv, cal_work);
    u8 reg_data, config, val;
    int ps_val[PS_CAL_COUNT] = {0};
    int ps_sum = 0;
    int i;
    int ret;

    clear_bit(PROX_CALI_CANCEL, &obj->wsched);

    // clear interrupt
    em20918_reg_write(obj->client, EM20918_INTERRUPT_REG, 0x00);

    pr_info("em20918 ==> begin to calibrate...\n");
    // change PS low threshold & high threshold to disable interrupt
    em20918_reg_write(obj->client, EM20918_PS_LT_REG, 0x00);
    em20918_reg_write(obj->client, EM20918_PS_HT_REG, 0xFF);

    // disable PS_SLP
    em20918_reg_read(obj->client, EM20918_CONFIG_REG, &config);
    reg_data = config & ~CONFIG_REG_PS_SLP;
    if (reg_data != config) {
        em20918_reg_write(obj->client, EM20918_CONFIG_REG, reg_data);
    }

    for (i = 0; i < PS_CAL_COUNT; i++) {
        ret = em20918_read_ps_data(obj->client, &val);
        ps_val[i] = val;
        pr_info("em20918 ==> ps[%d]=%d, ret=%d\n", i, ps_val[i], ret);

        ps_sum += ps_val[i];

        if (test_bit(PROX_CALI_CANCEL, &obj->wsched)) {
            goto cancel_cal;
        }
        usleep_range(100*1000, 100*1000+100);
    }

    // sort ps value
    sort(ps_val, PS_CAL_COUNT, sizeof(ps_val[0]), compare_val, s32_swap);

    pr_info("after sort:\n");
    for (i = 0; i < PS_CAL_COUNT; i++) {
        pr_info("[%d]: %d\n", i, ps_val[i]);
    }

    ps_sum = ps_sum - ps_val[0] - ps_val[1] \
             - ps_val[PS_CAL_COUNT - 2] - ps_val[PS_CAL_COUNT - 1];

    obj->near_threshold = ps_sum / (PS_CAL_COUNT - 4) + 1;

    if (obj->near_threshold >= 250) { // max: 0xFF
        obj->near_threshold = 100;
    } else if (obj->near_threshold < 10) {
        obj->near_threshold = 10;
    }
    pr_info("em20918: near_threshold=%d\n", obj->near_threshold);

    obj->is_cal_ok = true;


cancel_cal:

#ifndef IR_DETECT_OFF_POWER
    if (!obj->wakeup_en && !obj->capture_en) {
        config &= ~CONFIG_REG_PS_EN;
    }
#endif
    pr_info("wakeup_en=%d, capture_en=%d\n", obj->wakeup_en, obj->capture_en);

    em20918_reset_config(obj);
    // restore previous config reg val
    em20918_reg_write(obj->client, EM20918_CONFIG_REG, config);


    if (!obj->wakeup_en && !obj->capture_en) {
#ifdef IR_DETECT_OFF_POWER
        ir_em_power(obj, 0);
#else
        ir_em_sleep(obj, 1);
#endif
    }

    if (waitqueue_active(&obj->cal_wait)) {
        wake_up(&obj->cal_wait);
    }

    clear_bit(CAL_WORK_SCHED, &obj->wsched);
}

#ifdef IR_DETECT_OFF_POWER
static void ir_em_power(struct em20918_priv *obj, u8 onoff)
{
    struct em20918_platform_data *pdata = obj->pdata;

    pr_info("ir_em_power ===> %d\n", onoff);

    if (onoff) {
        gpio_direction_output(pdata->pwr_pin, pdata->pwr_en_level);
        // wait for power stable
        msleep(1);

        // already running, stop it
        if (test_bit(INIT_WORK_SCHED, &obj->wsched)) {
            flush_work(&obj->init_work);
        }

        set_bit(INIT_WORK_SCHED, &obj->wsched);
        schedule_work(&obj->init_work);
    } else {
        em20918_set_irq(obj, false);
        flush_work(&obj->init_work);
        gpio_direction_output(pdata->pwr_pin, !pdata->pwr_en_level);
        obj->is_init_ok = false;
    }
}
#endif

static ssize_t em20918_show_ps(struct device_driver *ddri, char *buf)
{
    int res;
    struct em20918_priv *obj = em20918_obj;
    u8 val;
    if((res = em20918_read_ps_data(obj->client, &val)) < 0) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        APS_LOG("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
        return sprintf(buf, "%d\n", val);
    }
}

static ssize_t em20918_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
    int ret;
    unsigned long value = 0;
    struct em20918_priv *obj = em20918_obj;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        APS_LOG( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    ret = em20918_reg_write(obj->client, EM20918_CONFIG_REG, (u8)value);
    if (ret < 0) {
       APS_LOG("%s: write i2c error\n", __func__);
       return ret;
    }

    return count;
}


static ssize_t em20918_show_config(struct device_driver *ddri, char *buf)
{
   int res;
   u8 val;
   struct em20918_priv *obj = em20918_obj;
   
   if((res = em20918_reg_read(obj->client, EM20918_CONFIG_REG, &val)) < 0) {
      return sprintf(buf, "ERROR: %d\n", res);
   } else {
      return sprintf(buf, "0x%04X\n", val);
   }
}

static ssize_t em20918_show_als(struct device_driver *ddri, char *buf)
{
    int res;
    u8 val = 0;
    struct em20918_priv *obj = em20918_obj;
    u16 light_val = 0;

    /*读取光感高8位*/
    if((res = em20918_reg_read(obj->client, EM30918_LIGHT_DATA_H, &val)) < 0) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        light_val = val;
        APS_LOG("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
    }
    /*读取光感低8位*/
    if((res = em20918_reg_read(obj->client, EM30918_LIGHT_DATA_L, &val)) < 0) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        light_val = light_val<<8 |val;
        APS_LOG("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
        return sprintf(buf, "%d\n", light_val);
    }
}

static DRIVER_ATTR(ps, S_IRUGO, em20918_show_ps, NULL);
static DRIVER_ATTR(als, S_IRUGO, em20918_show_als, NULL);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, em20918_show_config,em20918_store_config);

static struct driver_attribute *em20918_attr_list[] = {
   &driver_attr_ps,
   &driver_attr_als,
   &driver_attr_config,
};

static int em20918_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(em20918_attr_list) / sizeof(em20918_attr_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = driver_create_file(driver, em20918_attr_list[idx]))) {
            APS_ERR("driver_create_file (%s) = %d\n", em20918_attr_list[idx]->attr.name, err);
            break;
        }
    }

    return err;
}

static int em20918_delete_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(em20918_attr_list) / sizeof(em20918_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        driver_remove_file(driver, em20918_attr_list[idx]);
    }

    return err;
}


static void ir_em_sleep(struct em20918_priv *obj, u8 is_sleep)
{
    pr_info("ir_em_sleep ===> %d\n", is_sleep);

    if (!is_sleep) {

        // already running, stop it
        if (test_bit(INIT_WORK_SCHED, &obj->wsched)) {
            flush_work(&obj->init_work);
        }

        set_bit(INIT_WORK_SCHED, &obj->wsched);

        schedule_work(&obj->init_work);
    } else {
        em20918_set_irq(obj, false);

        if (test_and_clear_bit(CAL_WORK_SCHED, &obj->wsched)) {
            flush_work(&obj->cal_work);
        }

        if (test_and_clear_bit(INT_WORK_SCHED, &obj->wsched)) {
            set_bit(CAP_DETECT_CANCEL, &obj->wsched);
            flush_work(&obj->int_work);
            clear_bit(CAP_DETECT_CANCEL, &obj->wsched);
        }

        if (test_and_clear_bit(INIT_WORK_SCHED, &obj->wsched)) {
            flush_work(&obj->init_work);
        }

        em20918_reg_write(obj->client, EM20918_CONFIG_REG, 0x00);
        obj->is_init_ok = false;
    }
}

#if 0
static irqreturn_t em20918_irqhandler(int irq, void *devid)
{
    struct em20918_priv *obj = devid;

    if (!test_and_set_bit(INT_WORK_SCHED, &obj->wsched)) {
        schedule_work(&obj->int_work);
    }

    return IRQ_HANDLED;
}
#endif
static int ir_em_dev_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct em20918_priv *obj = container_of(miscdev, struct em20918_priv, miscdev);

    if (atomic_read(&obj->opened)) {
        pr_err("EBUSY\n");
        return -EBUSY;
    }

    atomic_inc(&obj->opened);

    return 0;
}

static int ir_em_dev_release(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct em20918_priv *obj = container_of(miscdev, struct em20918_priv, miscdev);

    atomic_dec(&obj->opened);

    return 0;
}

static long ir_em_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *miscdev = filp->private_data;
    struct em20918_priv *obj = container_of(miscdev, struct em20918_priv, miscdev);

    unsigned char var = (unsigned char)arg;

    switch(cmd) {
    case IR_DETECT_IOC_POWER:
        pr_info("IR_DETECT_IOC_POWER, var=%d\n", var);
    #ifdef IR_DETECT_OFF_POWER
        ir_em_power(obj, var);
    #else
        ir_em_sleep(obj, !var);
    #endif
        break;

    case IR_DETECT_IOC_WAKEUP_EN:
        pr_info("IR_DETECT_IOC_WAKEUP_EN, var=%d\n", var);
        if (var) {
            obj->wakeup_en = true;
            em20918_set_irq(obj, true);
        } else {
            obj->wakeup_en = false;
            if (!obj->capture_en) {
                em20918_set_irq(obj, false);
            }
        }
        break;

    case IR_DETECT_IOC_CAPTURE_EN:
        pr_info("IR_DETECT_IOC_CAPTURE_EN, var=%d\n", var);
        if (var) {
            obj->capture_en = true;
            em20918_set_irq(obj, true);
        } else {
            obj->capture_en = false;

            if (!obj->wakeup_en) {
                em20918_set_irq(obj, false);
            }
        }
        break;

    case IR_DETECT_IOC_PROX_CAL: {
        int __user *cali_prox = (void __user *)arg;

        obj->calibrating = true;

        pr_info("begin to init...\n");

        if (!obj->is_init_ok) {
            pr_info("not init ok, reinit...\n");
        #ifdef IR_DETECT_OFF_POWER
            ir_em_power(obj, true);
        #else
            ir_em_sleep(obj, false);
        #endif

            if (wait_event_timeout(obj->init_wait, obj->is_init_ok, 1 * HZ) == 0) {
                /**
                 * Wait timeout
                 */
                obj->calibrating = false;
                return -ETIMEDOUT;
            } else {
                if (!obj->is_init_ok) {
                    obj->calibrating = false;
                    return -EIO;
                }
                pr_info("init ok!\n");
            }
        }

        pr_info("begin to calibrate...\n");

        if (!test_and_set_bit(CAL_WORK_SCHED, &obj->wsched)) {
            pr_info("calibrate work start...\n");
            schedule_work(&obj->cal_work);
        } else {
            obj->calibrating = false;
            return -EBUSY;
        }

        if (wait_event_timeout(obj->cal_wait, obj->is_cal_ok, 1.5 * HZ) == 0) {
            /**
             * Wait timeout
             */
            obj->calibrating = false;
            return -ETIMEDOUT;
        } else {
            if (!obj->is_cal_ok) {
                obj->calibrating = false;
                return -EIO;
            }
        }

        /**
         * Copy data to userspace
         */
        //mutex_lock(&obj->lock);
        obj->is_cal_ok = false;
        if (copy_to_user(cali_prox, (void *)&obj->near_threshold, sizeof(int))) {
            //mutex_unlock(&obj->lock);
            obj->calibrating = false;
            return -EIO;
        }
        pr_info("calibrate ok...\n");
        //mutex_unlock(&obj->lock);
        obj->calibrating = false;
    }
    break;

    case IR_DETECT_IOC_PROX_SET: {
        int value = (int)arg;
        pr_info("IR_DETECT_IOC_PROX_SET, value=%d\n", value);
        if (var) {
            obj->near_threshold = value;
            if (obj->is_init_ok && !obj->calibrating) {

                if (test_bit(INT_WORK_SCHED, &obj->wsched)) {
                    set_bit(CAP_DETECT_CANCEL, &obj->wsched);
                    flush_work(&obj->int_work);
                    clear_bit(CAP_DETECT_CANCEL, &obj->wsched);
                }

                // already running, stop it
                if (test_bit(INIT_WORK_SCHED, &obj->wsched)) {
                    flush_work(&obj->init_work);
                }

                set_bit(INIT_WORK_SCHED, &obj->wsched);
                schedule_work(&obj->init_work);
            }
        } else {
            return -EINVAL;
        }
    }
    break;

    default:
        dev_err(&obj->client->dev, "Not supported CMD:0x%x\n", cmd);
        return -EINVAL;
    }
    return 0;
}

static struct file_operations ir_em_dev_fops = {
    .owner          = THIS_MODULE,
    .open           = ir_em_dev_open,
    .release        = ir_em_dev_release,
    .unlocked_ioctl = ir_em_dev_ioctl,
};

static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

static int als_enable_nodata(int en)
{
	/* enable by default */
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static ssize_t em20918_read_als(struct i2c_client *client, u16 *buf)
{
    int res;
    u8 val = 0;
    struct em20918_priv *obj = i2c_get_clientdata(client);
    u16 light_val = 0;

    /*读取光感高8位*/
    if((res = em20918_reg_read(obj->client, EM30918_LIGHT_DATA_H, &val)) < 0) {
        return -1;
    } else {
        light_val = val;
        APS_LOG("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
    }
    /*读取光感低8位*/
    if((res = em20918_reg_read(obj->client, EM30918_LIGHT_DATA_L, &val)) < 0) {
        return -1;
    } else {
        light_val = light_val<<8 |val;
        APS_LOG("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
    }
	*buf = light_val;
	APS_LOG("em20918 als value = 0x%04x\n", *buf);
	return 0;
}


static int als_get_data(int *value, int *status)
{
	int err = 0;
	u16 buf;
	char als[5] = {0};

	APS_LOG("em20918 als get data \n");

	if(em20918_obj == NULL) {
		APS_ERR("em20918_obj is null\n");
		return -1;
	}
	mutex_lock(&sensor_lock);
	err = em20918_read_als(em20918_obj->client, &buf);
	mutex_unlock(&sensor_lock);
	if (err) {
		APS_ERR("em20918_read_als failed\n");
		return -1;
	}
	sprintf(als, "%d", buf);
	err = kstrtoint(als, 10, value);
	if (err == 0)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	APS_LOG("em20918 als_get_data, value = %d, err = %d\n", *value, err);
	return err;
}

static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

static int ps_enable_nodata(int en)
{
	/* enable by default */
	return 0;
}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static ssize_t em20918_read_ps(struct i2c_client *client, char *buf)
{
    int res = 0;
    u8 val;
	struct em20918_priv *obj = i2c_get_clientdata(client);

    if((res = em20918_read_ps_data(obj->client, &val)) < 0) {
        return -1;
    } else {
        APS_LOG("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
        sprintf(buf, "%d", val);
    }
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;
	char ps[5] = {0};

	APS_LOG("em20918 ps get data \n");

	if(em20918_obj == NULL) {
		APS_ERR("em20918_obj is null\n");
		return -1;
	}
	mutex_lock(&sensor_lock);
	err = em20918_read_ps(em20918_obj->client, ps);
	mutex_unlock(&sensor_lock);
	if (err) {
		APS_ERR("em20918_read_ps failed\n");
		return -1;
	}
	err = kstrtoint(ps, 10, value);
	if (err == 0)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	APS_LOG("em20918 ps_get_data, value = %d\n",*value);
	return err;
}


static int em20918_i2c_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
    int ret = 0, err = 0;
    struct em20918_priv *obj;
    struct input_dev *input_device;
    u8 val;
	struct device_node *np = client->dev.of_node;
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };

    APS_ERR("%s === weiqifa === enter!\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        APS_ERR("I2C functionality not supported\n");
        return -ENODEV;
    }

	if (!np) {
        APS_ERR("no device tree\n");
        return -EINVAL;
    }

	err = get_alsps_dts_func(np, hw);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		return -1;
	}

    obj = kzalloc(sizeof(struct em20918_priv), GFP_KERNEL);
    if (obj == NULL) {
        APS_ERR("Failed to malloc em20918_priv\n");
        return -ENOMEM;
    }

	memset(obj, 0, sizeof(*obj));
	em20918_obj = obj;
	obj->hw=hw;

    obj->miscdev.minor = MISC_DYNAMIC_MINOR;
    obj->miscdev.name  = client->name;
    obj->miscdev.fops  = &ir_em_dev_fops;

	if (misc_register(&obj->miscdev) < 0) {
        APS_ERR("misc_register failed\n");
        goto err_misc_register;
    }

    ret = em20918_reg_read(client, EM20918_PID_REG, &val);
    APS_LOG("em20918 read pid: 0x%02X, ret=%d\n", val, ret);

#ifdef IR_DETECT_OFF_POWER
    gpio_direction_output(pdata->pwr_pin, !pdata->pwr_en_level);
#else
    em20918_reg_write(client, EM20918_CONFIG_REG, 0x80);
#endif

    input_device = input_allocate_device();
    if (!input_device) {
        ret = -ENOMEM;
        goto err_input_allocate;
    }

    obj->input = input_device;
    input_device->name = "ir-keys";
    input_device->id.bustype = BUS_I2C;
    input_device->dev.parent = &client->dev;
    input_set_drvdata(input_device, obj);

    __set_bit(EV_KEY, input_device->evbit);
    __set_bit(EV_SYN, input_device->evbit);
    input_set_capability(input_device, EV_KEY, KEY_WAKEUP);
    input_set_capability(input_device, EV_KEY, KEY_CAMERA);
    input_set_capability(input_device, EV_KEY, KEY_MUTE);

    ret = input_register_device(input_device);
    if (ret){
        goto err_input_register;
    }

	err = em20918_create_attr(&(em20918_init_info.platform_diver_addr->driver));
	if(err)
	{
		APS_ERR("em20918_create_attr fail = %d\n", err);
		goto err_create_attr;
	}

    init_waitqueue_head(&obj->init_wait);
    // init work queue
    INIT_WORK(&obj->init_work, em20918_init_events);

    // interrupt work queue
    INIT_WORK(&obj->int_work, em20918_int_workhandler);

    init_waitqueue_head(&obj->cal_wait);
    // calibrate work queue
    INIT_WORK(&obj->cal_work, em20918_cali_workhandler);

    obj->client = client;
    //obj->pdata = pdata;
    wake_lock_init(&obj->work_wake_lock, WAKE_LOCK_SUSPEND, "ir_em");
    i2c_set_clientdata(client, obj);

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	ps_ctl.is_polling_mode = obj->hw->polling_mode_ps;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit;
	}

    /* 使用EM30918  使能光感和距离 */
    em20918_reg_write(client, EM20918_CONFIG_REG, 0xBE);
	em20918_init_flag = 0;

    return 0;

exit:
err_create_attr:
	input_unregister_device(input_device);

err_input_register:
	input_free_device(input_device);

err_input_allocate:
	misc_deregister(&obj->miscdev);

err_misc_register:
	kfree(obj);
	em20918_init_flag = -1;
	APS_ERR("%s: err = %d\n", __func__, ret);

    return ret;
}

static int em20918_i2c_remove(struct i2c_client *client)
{
    struct em20918_priv *obj = i2c_get_clientdata(client);

    cancel_work_sync(&obj->cal_work);
    cancel_work_sync(&obj->int_work);
    cancel_work_sync(&obj->init_work);
    wake_lock_destroy(&obj->work_wake_lock);
    mutex_destroy(&obj->irq_lock);
    misc_deregister(&obj->miscdev);
    input_unregister_device(obj->input);
    input_free_device(obj->input);
	em20918_delete_attr(&(em20918_init_info.platform_diver_addr->driver));
    kfree(obj);

    return 0;
}

int em20918_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct em20918_priv *obj = i2c_get_clientdata(client);

    if (test_bit(CAL_WORK_SCHED, &obj->wsched)) {
        set_bit(PROX_CALI_CANCEL, &obj->wsched);
        flush_work(&obj->cal_work);
        clear_bit(PROX_CALI_CANCEL, &obj->wsched);
    }

    if (test_bit(INT_WORK_SCHED, &obj->wsched)) {
        set_bit(CAP_DETECT_CANCEL, &obj->wsched);
        flush_work(&obj->int_work);
        clear_bit(CAP_DETECT_CANCEL, &obj->wsched);
    }

    if (test_bit(INIT_WORK_SCHED, &obj->wsched)) {
        flush_work(&obj->init_work);
    }

    // clear interrupt, to avoid no response
    em20918_reg_write(obj->client, EM20918_INTERRUPT_REG, 0x00);

#if 0
    if (obj->wakeup_en || obj->capture_en) {
        em20918_set_irq(obj, true);
    }
#endif

    return 0;
}

int em20918_resume(struct device *dev)
{
    /*
        struct i2c_client *client = to_i2c_client(dev);
        struct em20918_priv *obj = i2c_get_clientdata(client);
        struct em20918_platform_data *pdata =
            (struct em20918_platform_data *)client->dev.platform_data;
    */

    return 0;
}

static const struct i2c_device_id em20918_ids[] = {
    {IR_EM_DEV_BASENAME, 0},
    {/*end of list*/}
};

MODULE_DEVICE_TABLE(i2c, em20918_ids);

static SIMPLE_DEV_PM_OPS(em20918_i2c_pm, em20918_suspend, em20918_resume);

static struct i2c_driver em20918_i2c_driver = {
    .driver = {
        .name  = EM20918_DRV_NAME,
        .owner = THIS_MODULE,
        .pm    = &em20918_i2c_pm,
    },
    .probe      = em20918_i2c_probe,
    .remove     = em20918_i2c_remove,
    .id_table   = em20918_ids,
};

static int em20918_local_init(void)
{
	if(i2c_add_driver(&em20918_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	if(-1 == em20918_init_flag)
	{
	   return -1;
	}
	APS_LOG("em20918_local_init\n");
	return 0;
}

static int em20918_local_uninit(void)
{
    i2c_del_driver(&em20918_i2c_driver);
	return 0;
}

static int __init em20918_init(void)
{
	alsps_driver_add(&em20918_init_info);
	APS_LOG("em20918_init\n");    
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit em20918_exit(void)
{
	APS_LOG("em20918_exit\n"); 
	APS_FUN();
}


module_init(em20918_init);
module_exit(em20918_exit);

MODULE_AUTHOR("Howrd <l.tao@onetolink.com>");
MODULE_DESCRIPTION("em20918 control driver");
MODULE_LICENSE("GPL");
