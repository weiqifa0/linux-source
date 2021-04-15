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

#define DRV_NAME                         "ir_em"
#define IR_EM_DEV_BASENAME               "ir_em"

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

struct em20918_dev_data {
    int irq;

    struct em20918_platform_data *pdata;
    struct wake_lock work_wake_lock;
    struct i2c_client *client;
    struct miscdevice miscdev;
    struct input_dev *input;

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

#ifdef IR_DETECT_OFF_POWER
static void ir_em_power(struct em20918_dev_data *dev_data, u8 onoff);
#else
static void ir_em_sleep(struct em20918_dev_data *dev_data, u8 is_sleep);
#endif

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
    struct em20918_dev_data *dev_data = container_of(work, struct em20918_dev_data, init_work);
    //struct em20918_platform_data *pdata = dev_data->pdata;
    int near_thres = dev_data->near_threshold;

    // soft reset
    em20918_reg_write(dev_data->client, EM20918_RESET_REG, RESET_REG_RST);
    em20918_reg_write(dev_data->client, EM20918_OFFSET_REG, OFFSET_REG_RST);
    // clear interrupt
    em20918_reg_write(dev_data->client, EM20918_INTERRUPT_REG, 0x00);
    // low threshold
    em20918_reg_write(dev_data->client, EM20918_PS_LT_REG, 0x07);
    // normal mode
    em20918_reg_write(dev_data->client, EM20918_OFFSET_REG, 0x00);

    // high threshold
    if (near_thres > 0) {
        em20918_reg_write(dev_data->client, EM20918_PS_HT_REG, near_thres);
    } else {
        pr_err("prox thres invalid value: %d!\n", near_thres);
        if (!dev_data->calibrating) {
            goto out;
        }
    }

    printk("== weiqifa === %s:%d\n", __func__, __LINE__);

    // power on & enable interrupt at last
    val = CONFIG_REG_PS_EN | CONFIG_REG_PS_SLP | CONFIG_REG_PS_DR_200MA;
    em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, val);

    dev_data->is_init_ok = true;

out:
    if (waitqueue_active(&dev_data->init_wait)) {
        wake_up(&dev_data->init_wait);
    }

    clear_bit(INIT_WORK_SCHED, &dev_data->wsched);

    return;
}

void em20918_set_irq(struct em20918_dev_data *dev_data, bool is_enable)
{
    mutex_lock(&dev_data->irq_lock);

    if (is_enable) {
        if (!dev_data->irq_enable) {
            enable_irq(dev_data->irq);
            dev_data->irq_enable = true;
        }
    } else {
        if (dev_data->irq_enable) {
            disable_irq(dev_data->irq);
            dev_data->irq_enable = false;
        }
    }

    mutex_unlock(&dev_data->irq_lock);
}

static void em20918_reset_config(struct em20918_dev_data *dev_data)
{
    // soft reset
    em20918_reg_write(dev_data->client, EM20918_RESET_REG, RESET_REG_RST);
    em20918_reg_write(dev_data->client, EM20918_OFFSET_REG, OFFSET_REG_RST);

    // clear interrupt
    em20918_reg_write(dev_data->client, EM20918_INTERRUPT_REG, 0x00);
    if (dev_data->near_threshold > 0) {
        em20918_reg_write(dev_data->client, EM20918_PS_LT_REG, 0x07);
        em20918_reg_write(dev_data->client, EM20918_PS_HT_REG, dev_data->near_threshold);
    }

    // normal mode
    em20918_reg_write(dev_data->client, EM20918_OFFSET_REG, 0x00);
}

static void em20918_int_workhandler(struct work_struct *work)
{
    struct em20918_dev_data *dev_data =
        container_of(work, struct em20918_dev_data, int_work);
    uint8_t reg_data;
    u8 ps_val;
    int i, ps_near_num = 0;
    int ret;

    wake_lock(&dev_data->work_wake_lock);

    if (!dev_data->wakeup_en && dev_data->capture_en) {
        pr_info("em20918 ==> report mute key...\n");
        input_report_key(dev_data->input, KEY_MUTE, 1);
        input_sync(dev_data->input);
        input_report_key(dev_data->input, KEY_MUTE, 0);
        input_sync(dev_data->input);
    }

    //em20918_set_irq(dev_data, false);

    // read interrupt status
    ret = em20918_reg_read(dev_data->client, EM20918_INTERRUPT_REG, &reg_data);
    pr_info("em20918 interrupt happen, interrupt=0x%02X\n", reg_data);

    // clear interrupt
    em20918_reg_write(dev_data->client, EM20918_INTERRUPT_REG, 0x00);

    // Indicates that the DATA_PS is higher than PIHT.
    if(reg_data & INTERRUPT_REG_PS_FLAG) {

        ret = em20918_reg_read(dev_data->client, EM20918_PS_DATA_REG, &ps_val);

        //pr_info("em20918: ps=0x%02X\n", ps_val);

        if (ps_val >= dev_data->near_threshold) { // near
            pr_info("em20918 ==> something near...\n");

            if (dev_data->wakeup_en) {
                pr_info("em20918 ==> report wakeup key...\n");
                input_report_key(dev_data->input, KEY_WAKEUP, 1);
                input_sync(dev_data->input);
                input_report_key(dev_data->input, KEY_WAKEUP, 0);
                input_sync(dev_data->input);
            }
        } else { // far
            pr_info("em20918 ==> something far...\n");
            goto out;
        }

    }


    if (dev_data->capture_en) {
        u8 config;
        pr_info("em20918 ==> begin to check capture...\n");

        em20918_reg_read(dev_data->client, EM20918_CONFIG_REG, &config);
        reg_data = config & ~CONFIG_REG_PS_SLP;
        if (reg_data != config) {
            em20918_reset_config(dev_data);
            em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, reg_data);
        }

        for (i = 0; i < 60; i++) {
            ret = em20918_read_ps_data(dev_data->client, &ps_val);
            if (ps_val >= dev_data->near_threshold) {
                ps_near_num++;
            }
            //pr_info("em20918 ==> ps=%d, %d\n", ps_val, ps_near_num);
            if (ps_near_num > 36) {
                break;
            }

            if (test_bit(CAP_DETECT_CANCEL, &dev_data->wsched)) {
                goto cancel_cap;
            }
            usleep_range(100*1000, 100*1000+100);
        }

        if (ps_near_num > 36) {
            pr_info("em20918 ==> report camera key...\n");
            input_report_key(dev_data->input, KEY_CAMERA, 1);
            input_sync(dev_data->input);
            input_report_key(dev_data->input, KEY_CAMERA, 0);
            input_sync(dev_data->input);
        }

	printk("== weiqifa === %s:%d\n", __func__, __LINE__);
cancel_cap:
        em20918_reset_config(dev_data);
        em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, config);
    }

out:

#if 0
    if (dev_data->wakeup_en || dev_data->capture_en) {
        em20918_set_irq(dev_data, true);
    }
#endif

    wake_unlock(&dev_data->work_wake_lock);
    clear_bit(INT_WORK_SCHED, &dev_data->wsched);
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
    struct em20918_dev_data *dev_data =
        container_of(work, struct em20918_dev_data, cal_work);
    u8 reg_data, config, val;
    int ps_val[PS_CAL_COUNT] = {0};
    int ps_sum = 0;
    int i;
    int ret;

    clear_bit(PROX_CALI_CANCEL, &dev_data->wsched);

    // clear interrupt
    em20918_reg_write(dev_data->client, EM20918_INTERRUPT_REG, 0x00);

    pr_info("em20918 ==> begin to calibrate...\n");
    // change PS low threshold & high threshold to disable interrupt
    em20918_reg_write(dev_data->client, EM20918_PS_LT_REG, 0x00);
    em20918_reg_write(dev_data->client, EM20918_PS_HT_REG, 0xFF);

    // disable PS_SLP
    em20918_reg_read(dev_data->client, EM20918_CONFIG_REG, &config);
    reg_data = config & ~CONFIG_REG_PS_SLP;
    if (reg_data != config) {
        em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, reg_data);
    }

    for (i = 0; i < PS_CAL_COUNT; i++) {
        ret = em20918_read_ps_data(dev_data->client, &val);
        ps_val[i] = val;
        pr_info("em20918 ==> ps[%d]=%d, ret=%d\n", i, ps_val[i], ret);

        ps_sum += ps_val[i];

        if (test_bit(PROX_CALI_CANCEL, &dev_data->wsched)) {
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

    dev_data->near_threshold = ps_sum / (PS_CAL_COUNT - 4) + 1;

    if (dev_data->near_threshold >= 250) { // max: 0xFF
        dev_data->near_threshold = 100;
    } else if (dev_data->near_threshold < 10) {
        dev_data->near_threshold = 10;
    }
    pr_info("em20918: near_threshold=%d\n", dev_data->near_threshold);

    dev_data->is_cal_ok = true;


cancel_cal:

#ifndef IR_DETECT_OFF_POWER
    if (!dev_data->wakeup_en && !dev_data->capture_en) {
        config &= ~CONFIG_REG_PS_EN;
    }
#endif
    pr_info("wakeup_en=%d, capture_en=%d\n", dev_data->wakeup_en, dev_data->capture_en);

    em20918_reset_config(dev_data);
    // restore previous config reg val
    em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, config);


    if (!dev_data->wakeup_en && !dev_data->capture_en) {
#ifdef IR_DETECT_OFF_POWER
        ir_em_power(dev_data, 0);
#else
        ir_em_sleep(dev_data, 1);
#endif
    }

    if (waitqueue_active(&dev_data->cal_wait)) {
        wake_up(&dev_data->cal_wait);
    }

    clear_bit(CAL_WORK_SCHED, &dev_data->wsched);
}

#ifdef IR_DETECT_OFF_POWER
static void ir_em_power(struct em20918_dev_data *dev_data, u8 onoff)
{
    struct em20918_platform_data *pdata = dev_data->pdata;

    pr_info("ir_em_power ===> %d\n", onoff);

    if (onoff) {
        gpio_direction_output(pdata->pwr_pin, pdata->pwr_en_level);
        // wait for power stable
        msleep(1);

        // already running, stop it
        if (test_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
            flush_work(&dev_data->init_work);
        }

        set_bit(INIT_WORK_SCHED, &dev_data->wsched);
        schedule_work(&dev_data->init_work);
    } else {
        em20918_set_irq(dev_data, false);
        flush_work(&dev_data->init_work);
        gpio_direction_output(pdata->pwr_pin, !pdata->pwr_en_level);
        dev_data->is_init_ok = false;
    }
}
#endif

static ssize_t em20918_show_ps(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    int res;
    struct em20918_dev_data *dev_data =dev_get_drvdata(dev);
    u8 val;
    if((res = em20918_read_ps_data(dev_data->client, &val)) < 0) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        printk("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
        return sprintf(buf, "0x%04X\n", val);
    }
}

static ssize_t em20918_store_config(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    int ret;
    unsigned long value = 0;
    struct em20918_dev_data *dev_data =dev_get_drvdata(dev);

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        printk( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    ret = em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, (u8)value);
    if (ret < 0) {
       printk("%s: write i2c error\n", __func__);
       return ret;
    }

    return count;
}


static ssize_t em20918_show_config(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
   int res;
   u8 val;
   struct em20918_dev_data *dev_data =dev_get_drvdata(dev);
   
   if((res = em20918_reg_read(dev_data->client, EM20918_CONFIG_REG, &val)) < 0) {
      return sprintf(buf, "ERROR: %d\n", res);
   } else {
      return sprintf(buf, "0x%04X\n", val);
   }
}

static ssize_t em20918_show_light(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    int res;
    u8 val = 0;
    struct em20918_dev_data *dev_data =dev_get_drvdata(dev);
    u16 light_val = 0;


    /*读取光感高8位*/
    if((res = em20918_reg_read(dev_data->client, EM30918_LIGHT_DATA_H, &val)) < 0) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        light_val = val;
        printk("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
    }
    /*读取光感低8位*/
    if((res = em20918_reg_read(dev_data->client, EM30918_LIGHT_DATA_L, &val)) < 0) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        light_val = light_val<<8 |val;
        printk("%s === weiqifa === %d,val-%d\n", __func__ ,__LINE__,val);
        return sprintf(buf, "0x%04x\n", light_val);
    }
}

static DEVICE_ATTR(ps, S_IRUGO, em20918_show_ps, NULL);
static DEVICE_ATTR(config, S_IWUSR | S_IRUGO, em20918_show_config,em20918_store_config);
static DEVICE_ATTR(light, S_IRUGO, em20918_show_light, NULL);

static struct device_attribute *em20918_attr_list[] = {
   &dev_attr_ps,
   &dev_attr_config,
   &dev_attr_light,
};



static int em20918_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(em20918_attr_list) / sizeof(em20918_attr_list[0]));

    if (dev == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = device_create_file(dev, em20918_attr_list[idx]))) {
            pr_err("driver_create_file (%s) = %d\n", em20918_attr_list[idx]->attr.name, err);
            break;
        }
    }

    return err;
}

static int em20918_delete_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(em20918_attr_list) / sizeof(em20918_attr_list[0]));

    if (!dev)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        device_remove_file(dev, em20918_attr_list[idx]);
    }

    return err;
}


static void ir_em_sleep(struct em20918_dev_data *dev_data, u8 is_sleep)
{
    pr_info("ir_em_sleep ===> %d\n", is_sleep);

    if (!is_sleep) {

        // already running, stop it
        if (test_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
            flush_work(&dev_data->init_work);
        }

        set_bit(INIT_WORK_SCHED, &dev_data->wsched);

        schedule_work(&dev_data->init_work);
    } else {
        em20918_set_irq(dev_data, false);

        if (test_and_clear_bit(CAL_WORK_SCHED, &dev_data->wsched)) {
            flush_work(&dev_data->cal_work);
        }

        if (test_and_clear_bit(INT_WORK_SCHED, &dev_data->wsched)) {
            set_bit(CAP_DETECT_CANCEL, &dev_data->wsched);
            flush_work(&dev_data->int_work);
            clear_bit(CAP_DETECT_CANCEL, &dev_data->wsched);
        }

        if (test_and_clear_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
            flush_work(&dev_data->init_work);
        }

        em20918_reg_write(dev_data->client, EM20918_CONFIG_REG, 0x00);
        dev_data->is_init_ok = false;
    }
}

#if 0
static irqreturn_t em20918_irqhandler(int irq, void *devid)
{
    struct em20918_dev_data *dev_data = devid;

    if (!test_and_set_bit(INT_WORK_SCHED, &dev_data->wsched)) {
        schedule_work(&dev_data->int_work);
    }

    return IRQ_HANDLED;
}
#endif
static int ir_em_dev_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct em20918_dev_data *dev_data = container_of(miscdev, struct em20918_dev_data, miscdev);

    if (atomic_read(&dev_data->opened)) {
        pr_err("EBUSY\n");
        return -EBUSY;
    }

    atomic_inc(&dev_data->opened);

    return 0;
}

static int ir_em_dev_release(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct em20918_dev_data *dev_data = container_of(miscdev, struct em20918_dev_data, miscdev);

    atomic_dec(&dev_data->opened);

    return 0;
}

static long ir_em_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *miscdev = filp->private_data;
    struct em20918_dev_data *dev_data = container_of(miscdev, struct em20918_dev_data, miscdev);

    unsigned char var = (unsigned char)arg;

    switch(cmd) {
    case IR_DETECT_IOC_POWER:
        pr_info("IR_DETECT_IOC_POWER, var=%d\n", var);
    #ifdef IR_DETECT_OFF_POWER
        ir_em_power(dev_data, var);
    #else
        ir_em_sleep(dev_data, !var);
    #endif
        break;

    case IR_DETECT_IOC_WAKEUP_EN:
        pr_info("IR_DETECT_IOC_WAKEUP_EN, var=%d\n", var);
        if (var) {
            dev_data->wakeup_en = true;
            em20918_set_irq(dev_data, true);
        } else {
            dev_data->wakeup_en = false;
            if (!dev_data->capture_en) {
                em20918_set_irq(dev_data, false);
            }
        }
        break;

    case IR_DETECT_IOC_CAPTURE_EN:
        pr_info("IR_DETECT_IOC_CAPTURE_EN, var=%d\n", var);
        if (var) {
            dev_data->capture_en = true;
            em20918_set_irq(dev_data, true);
        } else {
            dev_data->capture_en = false;

            if (!dev_data->wakeup_en) {
                em20918_set_irq(dev_data, false);
            }
        }
        break;

    case IR_DETECT_IOC_PROX_CAL: {
        int __user *cali_prox = (void __user *)arg;

        dev_data->calibrating = true;

        pr_info("begin to init...\n");

        if (!dev_data->is_init_ok) {
            pr_info("not init ok, reinit...\n");
        #ifdef IR_DETECT_OFF_POWER
            ir_em_power(dev_data, true);
        #else
            ir_em_sleep(dev_data, false);
        #endif

            if (wait_event_timeout(dev_data->init_wait, dev_data->is_init_ok, 1 * HZ) == 0) {
                /**
                 * Wait timeout
                 */
                dev_data->calibrating = false;
                return -ETIMEDOUT;
            } else {
                if (!dev_data->is_init_ok) {
                    dev_data->calibrating = false;
                    return -EIO;
                }
                pr_info("init ok!\n");
            }
        }

        pr_info("begin to calibrate...\n");

        if (!test_and_set_bit(CAL_WORK_SCHED, &dev_data->wsched)) {
            pr_info("calibrate work start...\n");
            schedule_work(&dev_data->cal_work);
        } else {
            dev_data->calibrating = false;
            return -EBUSY;
        }

        if (wait_event_timeout(dev_data->cal_wait, dev_data->is_cal_ok, 1.5 * HZ) == 0) {
            /**
             * Wait timeout
             */
            dev_data->calibrating = false;
            return -ETIMEDOUT;
        } else {
            if (!dev_data->is_cal_ok) {
                dev_data->calibrating = false;
                return -EIO;
            }
        }

        /**
         * Copy data to userspace
         */
        //mutex_lock(&dev_data->lock);
        dev_data->is_cal_ok = false;
        if (copy_to_user(cali_prox, (void *)&dev_data->near_threshold, sizeof(int))) {
            //mutex_unlock(&dev_data->lock);
            dev_data->calibrating = false;
            return -EIO;
        }
        pr_info("calibrate ok...\n");
        //mutex_unlock(&dev_data->lock);
        dev_data->calibrating = false;
    }
    break;

    case IR_DETECT_IOC_PROX_SET: {
        int value = (int)arg;
        pr_info("IR_DETECT_IOC_PROX_SET, value=%d\n", value);
        if (var) {
            dev_data->near_threshold = value;
            if (dev_data->is_init_ok && !dev_data->calibrating) {

                if (test_bit(INT_WORK_SCHED, &dev_data->wsched)) {
                    set_bit(CAP_DETECT_CANCEL, &dev_data->wsched);
                    flush_work(&dev_data->int_work);
                    clear_bit(CAP_DETECT_CANCEL, &dev_data->wsched);
                }

                // already running, stop it
                if (test_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
                    flush_work(&dev_data->init_work);
                }

                set_bit(INIT_WORK_SCHED, &dev_data->wsched);
                schedule_work(&dev_data->init_work);
            }
        } else {
            return -EINVAL;
        }
    }
    break;

    default:
        dev_err(&dev_data->client->dev, "Not supported CMD:0x%x\n", cmd);
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

static int em20918_i2c_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
    int ret = 0;
	#if 0
    struct em20918_platform_data *pdata =
        (struct em20918_platform_data *)client->dev.platform_data;
    #endif
    struct class *dev_class;
    struct device *ctl_dev;

    struct em20918_dev_data *dev_data;
    struct input_dev *input_device;
    //int error;
    u8 val;
    //unsigned int volt;
    //unsigned int isenable;
#if 0
    struct regulator *vmch_supply = devm_regulator_get(&client->dev,"reg-vmch");

    ret = regulator_get_voltage(vmch_supply);
    printk("%s === weiqifa === vmch:%d!\n", __func__,ret);

    ret = regulator_set_voltage(vmch_supply,3300000,3300000);
    if(ret != 0){
        dev_err(&client->dev, "regulator_set_voltage error\n");
    }
#endif
    printk("%s === weiqifa === enter!\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality not supported\n");
        return -ENODEV;
    }

    dev_data = kzalloc(sizeof(struct em20918_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        pr_err("Failed to malloc em20918_dev_data\n");
        return -ENOMEM;
    }

    dev_data->miscdev.minor = MISC_DYNAMIC_MINOR;
    dev_data->miscdev.name  = client->name;
    dev_data->miscdev.fops  = &ir_em_dev_fops;


    
#if 0
    dev_data->near_threshold = pdata->init_prox;
    if (misc_register(&dev_data->miscdev) < 0) {
        dev_err(&client->dev, "misc_register failed\n");
        goto err_misc_register;
    }

    if (gpio_is_valid(pdata->pwr_pin)) {
        ret = gpio_request(pdata->pwr_pin, "em20918 pwr");
        if (ret < 0) {
            dev_err(&client->dev, "%s:failed to set gpio pwr.\n", __func__);
            goto err_req_pwr_pin;
        }

        gpio_direction_output(pdata->pwr_pin, !pdata->pwr_en_level);
    }

    gpio_direction_output(pdata->pwr_pin, pdata->pwr_en_level);
    msleep(1);
#endif
    ret = em20918_reg_read(client, EM20918_PID_REG, &val);
    printk("em20918 read pid: 0x%02X, ret=%d\n", val, ret);
#if 0
    if (val != EM20918_PRODUCT_ID) {
        ret = -ENODEV;
        gpio_direction_output(pdata->pwr_pin, !pdata->pwr_en_level);
        goto err_pid;
    }
#endif
#ifdef IR_DETECT_OFF_POWER
    gpio_direction_output(pdata->pwr_pin, !pdata->pwr_en_level);
#else
    em20918_reg_write(client, EM20918_CONFIG_REG, 0x80);
#endif

#if 0
    mutex_init(&dev_data->irq_lock);
    if (gpio_is_valid(pdata->int_pin)) {
        ret = gpio_request(pdata->int_pin, "em20918 int");
        if (ret < 0) {
            dev_err(&client->dev, "%s:failed to set gpio int.\n", __func__);
            goto err_req_int_pin;
        }

        gpio_direction_input(pdata->int_pin);

        /**
         * Request MCU IO interrupt source
         */
        dev_data->irq = gpio_to_irq(pdata->int_pin);
        if (dev_data->irq < 0) {
            error = dev_data->irq;
            dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n",
                    pdata->int_pin, error);
            goto err_request_irq;
        }

        error = request_any_context_irq(dev_data->irq,                          \
                                        em20918_irqhandler,                     \
                                        /*IRQF_TRIGGER_FALLING | IRQF_DISABLED,   \*/
                                        IRQF_TRIGGER_FALLING | IRQF_ONESHOT,    \
                                        dev_name(&client->dev),                 \
                                        dev_data);
        if (error < 0) {
            dev_err(&client->dev, "Unable to clain irq %d, error %d\n",
                    dev_data->irq, error);
            goto err_request_irq;
        } else {
            enable_irq_wake(dev_data->irq);
        }

        mutex_lock(&dev_data->irq_lock);
        disable_irq(dev_data->irq);
        mutex_unlock(&dev_data->irq_lock);
    }
#endif

    input_device = input_allocate_device();
    if (!input_device) {
        ret = -ENOMEM;
        goto error_alloc_dev;
    }

    dev_data->input = input_device;
    input_device->name = "ir-keys";
    input_device->id.bustype = BUS_I2C;
    input_device->dev.parent = &client->dev;
    input_set_drvdata(input_device, dev_data);

    __set_bit(EV_KEY, input_device->evbit);
    __set_bit(EV_SYN, input_device->evbit);
    input_set_capability(input_device, EV_KEY, KEY_WAKEUP);
    input_set_capability(input_device, EV_KEY, KEY_CAMERA);
    input_set_capability(input_device, EV_KEY, KEY_MUTE);

    ret = input_register_device(input_device);
    if (ret){
        goto err_input_register;
    }


	dev_class = class_create(THIS_MODULE, "em20918");
    ctl_dev = device_create(dev_class, NULL, 0, NULL, "control");
    if (IS_ERR(ctl_dev)) {
        dev_err(ctl_dev, "Failed to create bt char device\n");
        ret = PTR_ERR(ctl_dev);
        goto err_create_dev;
    }

    dev_set_drvdata(ctl_dev, dev_data);

    if (em20918_create_attr(ctl_dev)) {
        ret = -EINVAL;
        goto err_create_attr;
    }

    init_waitqueue_head(&dev_data->init_wait);
    // init work queue
    INIT_WORK(&dev_data->init_work, em20918_init_events);

    // interrupt work queue
    INIT_WORK(&dev_data->int_work, em20918_int_workhandler);

    init_waitqueue_head(&dev_data->cal_wait);
    // calibrate work queue
    INIT_WORK(&dev_data->cal_work, em20918_cali_workhandler);

    dev_data->client = client;
    //dev_data->pdata = pdata;
    wake_lock_init(&dev_data->work_wake_lock, WAKE_LOCK_SUSPEND, "ir_em");
    i2c_set_clientdata(client, dev_data);

    printk("=== weiqifa === em20918_probe end.\n");

    /* 使用EM30918  使能光感和距离 */
    em20918_reg_write(client, EM20918_CONFIG_REG, 0xBE);

    return 0;

    cancel_work_sync(&dev_data->init_work);
#if 0
error_wq_create:
    input_unregister_device(input_device);
#endif
err_input_register:
    input_free_device(input_device);

error_alloc_dev:
    free_irq(dev_data->irq, dev_data);
#if 0
err_request_irq:
    if (gpio_is_valid(pdata->int_pin)) {
        gpio_free(pdata->int_pin);
    }

err_req_int_pin:
    mutex_destroy(&dev_data->irq_lock);

err_pid:
    if (gpio_is_valid(pdata->pwr_pin)) {
        gpio_free(pdata->pwr_pin);
    }

err_req_pwr_pin:
    misc_deregister(&dev_data->miscdev);

err_misc_register:
    kfree(dev_data);
#endif

err_create_dev:
	device_unregister(ctl_dev);

err_create_attr:
    if (dev_class) {
        class_destroy(dev_class);
    }
	em20918_delete_attr(ctl_dev);

    return ret;
}

static int em20918_i2c_remove(struct i2c_client *client)
{
    struct em20918_platform_data *pdata =
        (struct em20918_platform_data *)client->dev.platform_data;
    struct em20918_dev_data *dev_data = i2c_get_clientdata(client);

    cancel_work_sync(&dev_data->cal_work);
    cancel_work_sync(&dev_data->int_work);
    cancel_work_sync(&dev_data->init_work);
    wake_lock_destroy(&dev_data->work_wake_lock);
    mutex_destroy(&dev_data->irq_lock);
    misc_register(&dev_data->miscdev);
    gpio_free(pdata->int_pin);
    input_unregister_device(dev_data->input);
    input_free_device(dev_data->input);
    kfree(dev_data);

    return 0;
}

int em20918_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct em20918_dev_data *dev_data = i2c_get_clientdata(client);

    if (test_bit(CAL_WORK_SCHED, &dev_data->wsched)) {
        set_bit(PROX_CALI_CANCEL, &dev_data->wsched);
        flush_work(&dev_data->cal_work);
        clear_bit(PROX_CALI_CANCEL, &dev_data->wsched);
    }

    if (test_bit(INT_WORK_SCHED, &dev_data->wsched)) {
        set_bit(CAP_DETECT_CANCEL, &dev_data->wsched);
        flush_work(&dev_data->int_work);
        clear_bit(CAP_DETECT_CANCEL, &dev_data->wsched);
    }

    if (test_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
        flush_work(&dev_data->init_work);
    }

    // clear interrupt, to avoid no response
    em20918_reg_write(dev_data->client, EM20918_INTERRUPT_REG, 0x00);

#if 0
    if (dev_data->wakeup_en || dev_data->capture_en) {
        em20918_set_irq(dev_data, true);
    }
#endif

    return 0;
}

int em20918_resume(struct device *dev)
{
    /*
        struct i2c_client *client = to_i2c_client(dev);
        struct em20918_dev_data *dev_data = i2c_get_clientdata(client);
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

static int __init em20918_init(void)
{
    return i2c_add_driver(&em20918_i2c_driver);
}

static void __exit em20918_exit(void)
{
    i2c_del_driver(&em20918_i2c_driver);
}

late_initcall(em20918_init);
module_exit(em20918_exit);

MODULE_AUTHOR("Howrd <l.tao@onetolink.com>");
MODULE_DESCRIPTION("em20918 control driver");
MODULE_LICENSE("GPL");
