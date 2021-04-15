/*
 * linux/drivers/misc/plgs_stk3420.c
 *
 * stk3420 control interface driver
 *
 * Copyright 2019, Howrd <howrd@21cn.com>
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
#include <linux/sort.h>
#include "plgs_stk3420.h"
#include "gs_algorithm.h"
#include "stk_cust_alsps.h"



//#define STK_GESTURE_LOG

/////////////////////////////////////////////////

#define APGS_TAG                  "[ALS/PS/GS] "
#define APGS_FUN(f)               printk(KERN_INFO APGS_TAG"%s\n", __FUNCTION__)
#define APGS_ERR(fmt, args...)    printk(KERN_ERR  APGS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APGS_INFO(fmt, args...)   printk(KERN_INFO APGS_TAG fmt, ##args)
#define APGS_WARN(fmt, args...)   printk(KERN_WARNING APGS_TAG fmt, ##args)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STK_FOR_TEST
#define DRV_NAME                         "ir_stk"
#define IR_STK_DEV_BASENAME              "ir_stk"

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum {
    INIT_WORK_SCHED    = (1 << 0),     /* init work scheduled or running */
    INT_WORK_SCHED     = (1 << 1),     /* interrupt work scheduled or running */
    CAP_DETECT_CANCEL  = (1 << 2),     /* capture detect flag */
    CAL_WORK_SCHED     = (1 << 3),     /* calibrate work scheduled or running */
    PROX_CALI_CANCEL   = (1 << 4),     /* proximity cancel flag */
};

struct stk3420_regs {
    uint8_t state_reg;
    uint8_t psgsctrl1_reg;
    uint8_t alsctrl1_reg;
    uint8_t ledctrl_reg;
    uint8_t wait1_psgs_reg_ps;
    uint8_t wait1_psgs_reg_gs;
    uint8_t alsctrl2_reg;
    uint8_t wait_als_reg;
    uint8_t wait2_ps_reg;
    uint8_t psgsctrl2_reg;
    uint8_t int_reg;
    uint8_t fifoctrl_reg;
};

struct stk3420_dev_data {
    struct stk_alsps_hw *hw;

    int irq;
    int irq_pin;
    unsigned long irq_flags;
    int pwr_pin;
    int pwr_en_level;

    int init_prox;

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

    atomic_t recv_reg;

    // data
    u16 ps;
    u16 als;

    atomic_t ps_thd_high;
    atomic_t ps_thd_low;

    struct stk3420_regs reg;

    /****************************     poll timer     ****************************/
    struct hrtimer ps_poll_timer;
    int timer_run;
    struct workqueue_struct *ps_poll_wq;
    struct work_struct ps_poll_work;
    ktime_t ps_poll_delay;
    /****************************************************************************/
    uint16_t ges_data_last[4];

    /****************************************************************************/
    struct wake_lock gs_wake_lock;

    int ges_enabled;

    /****************************************************************************/


};



#ifdef IR_DETECT_OFF_POWER
static void ir_stk_power(struct stk3420_dev_data *dev_data, u8 onoff);
#else
//static void ir_stk_sleep(struct stk3420_dev_data *dev_data, u8 is_sleep);
#endif


static int stk3420_reg_read(const struct i2c_client *client, u8 addr, u8 *val)
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
    msg[1].len    = 1;
    msg[1].buf    = val;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c read error\n", __func__);
    return ret;
}

static int stk3420_reg_write(const struct i2c_client *client,
                             u8 addr, u8 val)
{
    int ret;
    struct i2c_msg msg;
    u8 send[2];

    send[0]   = addr;
    send[1]   = val;

    msg.addr  = client->addr;
    msg.flags = client->flags & I2C_M_TEN;
    msg.len   = 2;
    msg.buf   = send;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        dev_err(&client->dev, "%s:i2c write error\n", __func__);

    return ret;
}

static int stk3420_i2c_read(const struct i2c_client *client,
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

static int stk3420_i2c_write(const struct i2c_client *client,
                             u8 addr, int len, u8 *buf)
{
    int ret;
    struct i2c_msg msg;
    u8 *send_buf;

    send_buf = kzalloc(len + 1, GFP_KERNEL);
    if (send_buf == NULL) {
        APGS_ERR("Failed to malloc buf\n");
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

int stk3420_check_pid(const struct i2c_client *client)
{
    int ret;
    u8 data;

    ret = stk3420_reg_read(client, STK3420_PDT_ID_REG, &data);

    if (ret < 0) {
        return ret;
    }

    APGS_INFO("stk3420_check_pid: pid=0x%02X\n", data);

    return 0;
}

int stk3420_get_als(const struct i2c_client *client, u16 *als)
{
    int ret;
    u8 buf[2];

    ret = stk3420_i2c_read(client, STK3420_DATA1_ALS_REG, 2, buf);
    if (ret < 0) {
        APGS_ERR("read als error: %d\n", ret);
        return -EFAULT;
    }

    *als = (buf[0] << 8) | buf[1];

    return 0;
}

int stk3420_get_ps(const struct i2c_client *client, u16 *ps)
{
    int ret;
    u8 buf[2];

    ret = stk3420_i2c_read(client, STK3420_DATA1_PS_REG, 2, buf);
    if (ret < 0) {
        APGS_ERR("read ps error: %d\n", ret);
        return -EFAULT;
    }

    *ps = (buf[0] << 8) | buf[1];

    return 0;
}

static int stk3420_set_ps_thd_l(const struct i2c_client *client, u16 thd_l)
{
    u8 val[2];
    int ret;

    val[0] = (thd_l & 0xFF00) >> 8;
    val[1] = thd_l & 0x00FF;
    ret = stk3420_i2c_write(client, STK3420_THDL1_PS_REG, 2, val);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int stk3420_set_ps_thd_h(const struct i2c_client *client, u16 thd_h)
{
    u8 val[2];
    int ret;

    val[0] = (thd_h & 0xFF00) >> 8;
    val[1] = thd_h & 0x00FF;
    ret = stk3420_i2c_write(client, STK3420_THDH1_PS_REG, 2, val);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int stk3420_set_gsflag(const struct i2c_client *client, u8 gsflag)
{
    int ret;

    ret = stk3420_reg_write(client, STK3420_GSFLAG_REG, gsflag);
    if(ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int stk3420_get_gsflag(const struct i2c_client *client, u8 *gsflag)
{
    int ret;

    ret = stk3420_reg_read(client, STK3420_GSFLAG_REG, gsflag);

    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int stk3420_set_state(const struct i2c_client *client, u8 state)
{
    int ret;

    ret = stk3420_reg_write(client, STK3420_STATE_REG, state);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int stk3420_get_state(const struct i2c_client *client, u8 *state)
{
    int ret;

    ret = stk3420_reg_read(client, STK3420_STATE_REG, state);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int stk3420_set_wait1_psgs(const struct i2c_client *client, u8 wait1)
{
    int ret;

    ret = stk3420_reg_write(client, STK3420_WAIT1_PSGS_REG, wait1);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int stk3420_set_psgs_ctrl2(const struct i2c_client *client, uint8_t ctrl2)
{
    int ret;

    ret = stk3420_reg_write(client, STK3420_PSGSCTRL2_REG, ctrl2);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}

static int32_t stk3420_get_psgs_ctrl2(const struct i2c_client *client, u8 *ctrl2)
{
    int ret;

    ret = stk3420_reg_read(client, STK3420_PSGSCTRL2_REG, ctrl2);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }
    return ret;
}


void stk3420_set_irq(struct stk3420_dev_data *dev_data, bool is_enable)
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

/* 使能光感 */
static int stk3420_enable_als(struct i2c_client *client)
{
    int ret;

	u8 buf[1];

	/* 读取使能寄存器里面原来的值 */
	ret = stk3420_i2c_read(client, STK3420_STATE_REG, 1, buf);
	if (ret < 0) {
		APGS_ERR("read als error: %d\n", ret);
		return -EFAULT;
	}
	APGS_INFO("=== weiqifa ==== reg:%x",buf[0]);
	/*或上标志位*/
	buf[0] |= STATE_EN_ALS;
	APGS_INFO("=== weiqifa ==== reg:%x",buf[0]);
	/* 写入新的值 */
    ret = stk3420_i2c_write(client, STK3420_STATE_REG, 1, buf);
    if (ret < 0) {
        APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
    }

	APGS_INFO("=== weiqifa ==== success.");
    return ret;

}

static int stk3420_enable_ges(struct i2c_client *client, int enable)
{
    struct stk3420_dev_data *dev_data = i2c_get_clientdata(client);
    int err;
    uint8_t state_val, psgs_ctrl2_val;
    uint8_t ps_enabled = (dev_data->reg.state_reg & STATE_EN_PS) ? 1 : 0;
	/*使能光照强度感应*/
    //uint8_t als_enabled = (dev_data->reg.state_reg & STATE_EN_ALS) ? 1 : 0;

    if (enable == dev_data->ges_enabled) {
        return 0;
    }

    APGS_INFO("%s: enable=%d\n", __func__, enable);
    err = stk3420_get_state(dev_data->client, &state_val);
    if (err < 0) {
        return err;
    }

    if (enable) {
        stk3420_set_wait1_psgs(dev_data->client, dev_data->reg.wait1_psgs_reg_gs);
        err = stk3420_get_psgs_ctrl2(dev_data->client, &psgs_ctrl2_val);
        if (err < 0) {
            return err;
        }

        enable &= (PSGS2_EN_GSEW | PSGS2_EN_GSNS);
        err = stk3420_set_psgs_ctrl2(dev_data->client, enable | psgs_ctrl2_val);
        if (err < 0) {
            return err;
        }

        if (!ps_enabled) {
            state_val |= STATE_EN_WAIT_PSGS;
            err = stk3420_set_state(dev_data->client, state_val);
            if (err < 0) {
                return err;
            }
        }

        stk3420_gesture_func_init();

        if (!ps_enabled || dev_data->timer_run == 0) {
            dev_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
            hrtimer_start(&dev_data->ps_poll_timer, dev_data->ps_poll_delay, HRTIMER_MODE_REL);
            dev_data->timer_run = 1;
            APGS_INFO("%s: start poll timer\n", __func__);
        }

        wake_lock(&dev_data->work_wake_lock);

    } else {

        wake_unlock(&dev_data->work_wake_lock);

        if(!ps_enabled /*|| dev_data->tune0.psi_set != 0*/) {
            hrtimer_cancel(&dev_data->ps_poll_timer);
            dev_data->timer_run = 0;
            APGS_INFO("%s: stop poll timer\n", __func__);
        }

        err = stk3420_get_psgs_ctrl2(dev_data->client, &psgs_ctrl2_val);
        if (err < 0) {
            return err;
        }

        err = stk3420_set_psgs_ctrl2(dev_data->client,
                                     psgs_ctrl2_val & (~(PSGS2_EN_GSEW | PSGS2_EN_GSNS)));
        if (err < 0) {
            return err;
        }

        if (ps_enabled) {
            stk3420_set_wait1_psgs(dev_data->client, dev_data->reg.wait1_psgs_reg_ps);
        } else {
            state_val &= ~(STATE_EN_WAIT_PSGS | STATE_EN_BGIR);
            err = stk3420_set_state(dev_data->client, state_val);
            if (err < 0) {
                return err;
            }
        }

    }

    dev_data->ges_enabled = enable;
    dev_data->reg.state_reg = state_val;
    APGS_INFO("%s: finish \n", __func__);

    return 0;
}


static void stk3420_init_events(struct work_struct *work)
{
#if 0
    struct stk3420_dev_data *dev_data = container_of(work, struct stk3420_dev_data, init_work);

    stk3420_enable_ges(dev_data->client, PSGS2_EN_GSEW | PSGS2_EN_GSNS);
#else
    u8 val;
    struct stk3420_dev_data *dev_data = container_of(work, struct stk3420_dev_data, init_work);

	APGS_INFO("stk3420_init_events start.\n");
    // soft reset
    stk3420_reg_write(dev_data->client, STK3420_SOFT_RESET_REG, 0x00);

	/*使能手势*/
	stk3420_enable_ges(dev_data->client, PSGS2_EN_GSEW | PSGS2_EN_GSNS);

    msleep(1);

    APGS_INFO("set ps threshold...\n");

    /*Config PS threshold value*/
    stk3420_set_ps_thd_h(dev_data->client, STK3420_THDH_PS_VALUE);
    stk3420_set_ps_thd_l(dev_data->client, STK3420_THDL_PS_VALUE);

    val = STATE_EN_WAIT_PSGS | STATE_EN_PS;
    stk3420_reg_write(dev_data->client, STK3420_STATE_REG, val);

    // enable interrupt at last
    val = INTERRUPT_EN_PS_INT;
    stk3420_reg_write(dev_data->client, STK3420_INT_REG, val);

    dev_data->is_init_ok = true;

	/*使能光感*/
	stk3420_enable_als(dev_data->client);

    if (waitqueue_active(&dev_data->init_wait)) {
        wake_up(&dev_data->init_wait);
    }

    clear_bit(INIT_WORK_SCHED, &dev_data->wsched);

    stk3420_set_irq(dev_data, true);
	APGS_INFO("stk3420_init_events end.\n");
#endif

    return;
}

#if 0
static void stk3420_reset_config(struct stk3420_dev_data *dev_data)
{
    // soft reset
    stk3420_reg_write(dev_data->client, STK3420_SOFT_RESET_REG, RESET_REG_RST);
}
#endif


static void stk3420_int_workhandler(struct work_struct *work)
{
    struct stk3420_dev_data *dev_data =
        container_of(work, struct stk3420_dev_data, int_work);
    u8 reg_data;
    u16 ps_val;
    int ret;

    wake_lock(&dev_data->work_wake_lock);

	APGS_INFO(" ###	IRQ	### \n");
	APGS_INFO(" ###		### \n");
	APGS_INFO("###############\n");
	APGS_INFO(" ###		### \n");
	APGS_INFO(" ###		### \n");


    if (!dev_data->wakeup_en && dev_data->capture_en) {
        APGS_INFO("report mute key...\n");
        //input_report_key(dev_data->input, KEY_MUTE, 1);
        //input_sync(dev_data->input);
        //input_report_key(dev_data->input, KEY_MUTE, 0);
        //input_sync(dev_data->input);
    }

    stk3420_set_irq(dev_data, false);

    // read interrupt status
    ret = stk3420_reg_read(dev_data->client, STK3420_FLAG_REG, &reg_data);
    APGS_INFO("stk3420 interrupt happen, interrupt=0x%02X\n", reg_data);

    // clear interrupt
    stk3420_reg_write(dev_data->client, STK3420_FLAG_REG, 0x00);

	/* 读取距离的寄存器数据 */
    if (reg_data & FLAG_FLG_PS_INT) {
        ret = stk3420_get_ps(dev_data->client, &ps_val);
        APGS_INFO("ps = 0x%04X\n", ps_val);

        if (reg_data & FLAG_FLG_NF) {
            APGS_INFO("#### far >>>>>>>>>\n");
        } else {
            APGS_INFO("#### near <<<<<<<<\n");
        }
    }

    //if (dev_data->wakeup_en || dev_data->capture_en) {
    stk3420_set_irq(dev_data, true);
    //}

    wake_unlock(&dev_data->work_wake_lock);
    clear_bit(INT_WORK_SCHED, &dev_data->wsched);
}


#if 0
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
#endif

#if 0
static void stk3420_cali_workhandler(struct work_struct *work)
{
    struct stk3420_dev_data *dev_data =
        container_of(work, struct stk3420_dev_data, cal_work);
    u8 reg_data, config, val;
    int ps_val[PS_CAL_COUNT] = {0};
    int ps_sum = 0;
    int i;
    int ret;

    clear_bit(PROX_CALI_CANCEL, &dev_data->wsched);

    // clear interrupt
    stk3420_reg_write(dev_data->client, STK3420_INTERRUPT_REG, 0x00);

    APGS_INFO("stk3420 ==> begin to calibrate...\n");
    // change PS low threshold & high threshold to disable interrupt
    stk3420_reg_write(dev_data->client, STK3420_PS_LT_REG, 0x00);
    stk3420_reg_write(dev_data->client, STK3420_PS_HT_REG, 0xFF);

    // disable PS_SLP
    stk3420_reg_read(dev_data->client, STK3420_CONFIG_REG, &config);
    reg_data = config & ~CONFIG_REG_PS_SLP;
    if (reg_data != config) {
        stk3420_reg_write(dev_data->client, STK3420_CONFIG_REG, reg_data);
    }

    for (i = 0; i < PS_CAL_COUNT; i++) {
        ret = stk3420_read_ps_data(dev_data->client, &val);
        ps_val[i] = val;
        APGS_INFO("stk3420 ==> ps[%d]=%d, ret=%d\n", i, ps_val[i], ret);

        ps_sum += ps_val[i];

        if (test_bit(PROX_CALI_CANCEL, &dev_data->wsched)) {
            goto cancel_cal;
        }
        usleep_range(100 * 1000, 100 * 1000 + 100);
    }

    // sort ps value
    sort(ps_val, PS_CAL_COUNT, sizeof(ps_val[0]), compare_val, s32_swap);

    APGS_INFO("after sort:\n");
    for (i = 0; i < PS_CAL_COUNT; i++) {
        APGS_INFO("[%d]: %d\n", i, ps_val[i]);
    }

    ps_sum = ps_sum - ps_val[0] - ps_val[1] \
             - ps_val[PS_CAL_COUNT - 2] - ps_val[PS_CAL_COUNT - 1];

    dev_data->near_threshold = ps_sum / (PS_CAL_COUNT - 4) + 1;

    if (dev_data->near_threshold >= 250) { // max: 0xFF
        dev_data->near_threshold = 100;
    } else if (dev_data->near_threshold < 10) {
        dev_data->near_threshold = 10;
    }
    APGS_INFO("stk3420: near_threshold=%d\n", dev_data->near_threshold);

    dev_data->is_cal_ok = true;


cancel_cal:

#ifndef IR_DETECT_OFF_POWER
    if (!dev_data->wakeup_en && !dev_data->capture_en) {
        config &= ~CONFIG_REG_PS_EN;
    }
#endif
    APGS_INFO("wakeup_en=%d, capture_en=%d\n", dev_data->wakeup_en, dev_data->capture_en);

    stk3420_reset_config(dev_data);
    // restore previous config reg val
    stk3420_reg_write(dev_data->client, STK3420_CONFIG_REG, config);


    if (!dev_data->wakeup_en && !dev_data->capture_en) {
#ifdef IR_DETECT_OFF_POWER
        ir_stk_power(dev_data, 0);
#else
        ir_stk_sleep(dev_data, 1);
#endif
    }

    if (waitqueue_active(&dev_data->cal_wait)) {
        wake_up(&dev_data->cal_wait);
    }

    clear_bit(CAL_WORK_SCHED, &dev_data->wsched);
}
#endif

static int stk_get_gesture(struct stk3420_dev_data *dev_data)
{
    int ret, len;
    int unit_data_size = 0;
    u8 ges[8];
    u8 gsflag;
    uint32_t gesture = 0;
#ifdef STK_GESTURE_LOG
    int phase[2] = {0, 0}, correl[2] = {0, 0};
#endif

    ret = stk3420_get_gsflag(dev_data->client, &gsflag);
    if(ret < 0) {
        return ret;
    }

    //APGS_INFO("%s: gsflag = 0x%02X\n", __func__, gsflag);

    len = gsflag & GSFLAG_GS_FIFO_LEN;
    if (len == 0) {
        APGS_WARN("%s: gs_fifo_len = 0, gsflag = 0x%02X\n", __func__, gsflag);
        return 0;
    }

    while (len > 0) {

        ret = stk3420_i2c_read(dev_data->client, STK3420_DATA1_GSE_REG, 8, ges);
        if(ret < 0) {
            APGS_ERR("%s: fail, ret=%d\n", __func__, ret);
            return ret;
        }

        len--;
        unit_data_size++;
        dev_data->ges_data_last[0] = (ges[0] << 8) | ges[1];
        dev_data->ges_data_last[1] = (ges[2] << 8) | ges[3];
        dev_data->ges_data_last[2] = (ges[4] << 8) | ges[5];
        dev_data->ges_data_last[3] = (ges[6] << 8) | ges[7];

        stk3420_gesture_determine(dev_data->ges_data_last, &gesture);
#ifdef STK_GESTURE_LOG
        stk3420_gesture_get_MaxPhase(phase);
        stk3420_gesture_get_MaxCorr(correl);
#endif

#if 0
        if (gesture != 0) {
            APGS_INFO("========================================\n");
            APGS_INFO("-------------  gesture=%02X  -------------\n", gesture);
            APGS_INFO("========================================\n");
        }
#endif

        if (gesture != 0) {
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");

            if (gesture == GESTURE_EVENT_UP) {
                APGS_INFO("           ##\n");
                APGS_INFO("        ## ## ##\n");
                APGS_INFO("     ###   ##   ###\n");
                APGS_INFO("    ##     ##     ##\n");
                APGS_INFO("           ##\n");
                APGS_INFO("           ##\n");
                APGS_INFO("           ##\n");
                APGS_INFO("           ##\n");
                input_report_key(dev_data->input, KEY_F1, 1);
                input_sync(dev_data->input);
                input_report_key(dev_data->input, KEY_F1, 0);
                input_sync(dev_data->input);
            } else if (gesture == GESTURE_EVENT_DOWN) {
                APGS_INFO("           ##\n");
                APGS_INFO("           ##\n");
                APGS_INFO("           ##\n");
                APGS_INFO("           ##\n");
                APGS_INFO("    ##     ##     ##\n");
                APGS_INFO("     ###   ##   ###\n");
                APGS_INFO("        ## ## ##\n");
                APGS_INFO("           ##\n");
                input_report_key(dev_data->input, KEY_F2, 1);
                input_sync(dev_data->input);
                input_report_key(dev_data->input, KEY_F2, 0);
                input_sync(dev_data->input);

            } else if (gesture == GESTURE_EVENT_LEFT) {
                APGS_INFO("      #######\n");
                APGS_INFO("   ######\n");
                APGS_INFO("############################\n");
                APGS_INFO("   ######\n");
                APGS_INFO("      #######\n");
                input_report_key(dev_data->input, KEY_F3, 1);
                input_sync(dev_data->input);
                input_report_key(dev_data->input, KEY_F3, 0);
                input_sync(dev_data->input);
            } else if (gesture == GESTURE_EVENT_RIGHT) {
                APGS_INFO("                #######\n");
                APGS_INFO("                   ######\n");
                APGS_INFO("############################\n");
                APGS_INFO("                   ######\n");
                APGS_INFO("                #######\n");
                input_report_key(dev_data->input, KEY_F4, 1);
                input_sync(dev_data->input);
                input_report_key(dev_data->input, KEY_F4, 0);
                input_sync(dev_data->input);
            } else if (gesture != 0) {
                APGS_INFO("############################\n");
                APGS_INFO("############################\n");
                APGS_INFO("----    gesture=0x%02X    ----\n", gesture);
                APGS_INFO("############################\n");
                APGS_INFO("############################\n");
                input_report_key(dev_data->input, KEY_F5, 1);
                input_sync(dev_data->input);
                input_report_key(dev_data->input, KEY_F5, 0);
                input_sync(dev_data->input);
            }

            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
            APGS_INFO("\n");
        }

#if 0
        if(gesture != 0) {
            if(ps_data->ges_enabled) {
                stk_ges_input_key_event(ps_data, gesture);
            }
        }
#endif

#ifdef STK_GESTURE_LOG
        APGS_INFO("stk:%d->%d,%d,%d,%d\n", unit_data_size, ges[0] << 8 | ges[1], ges[2] << 8 | ges[3], ges[4] << 8 | ges[5], ges[6] << 8 | ges[7]);
        APGS_INFO("stk:phase[0]=%d, phase[1]=%d, correl[0]=%d, correl[1]=%d \n", phase[0], phase[1], correl[0], correl[1]);
#endif
    }

    if(gsflag & GSFLAG_FLG_GS_FIFO_OV) {
        APGS_INFO("%s: clr gsflag\n", __func__);
        stk3420_set_gsflag(dev_data->client, 0);
    }
    return unit_data_size;
}

static void ps_poll_work_func(struct work_struct *work)
{
    struct stk3420_dev_data *dev_data =
        container_of(work, struct stk3420_dev_data, ps_poll_work);

    stk_get_gesture(dev_data);
}

static enum hrtimer_restart ps_poll_timer_func(struct hrtimer *timer)
{
    struct stk3420_dev_data *dev_data =
        container_of(timer, struct stk3420_dev_data, ps_poll_timer);

    queue_work(dev_data->ps_poll_wq, &dev_data->ps_poll_work);
    hrtimer_forward_now(&dev_data->ps_poll_timer, dev_data->ps_poll_delay);

    return HRTIMER_RESTART;
}

#ifdef IR_DETECT_OFF_POWER
static void ir_stk_power(struct stk3420_dev_data *dev_data, u8 onoff)
{
    APGS_INFO("ir_stk_power ===> %d\n", onoff);

    if (onoff) {
        gpio_direction_output(dev_data->pwr_pin, dev_data->pwr_en_level);
        // wait for power stable
        msleep(1);

        // already running, stop it
        if (test_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
            flush_work(&dev_data->init_work);
        }

        set_bit(INIT_WORK_SCHED, &dev_data->wsched);
        schedule_work(&dev_data->init_work);
    } else {
        stk3420_set_irq(dev_data, false);
        flush_work(&dev_data->init_work);
        gpio_direction_output(dev_data->pwr_pin, !dev_data->pwr_en_level);
        dev_data->is_init_ok = false;
    }
}
#endif

#if 0
static void ir_stk_sleep(struct stk3420_dev_data *dev_data, u8 is_sleep)
{
    APGS_INFO("ir_stk_sleep ===> %d\n", is_sleep);

    if (!is_sleep) {

        // already running, stop it
        if (test_bit(INIT_WORK_SCHED, &dev_data->wsched)) {
            flush_work(&dev_data->init_work);
        }

        set_bit(INIT_WORK_SCHED, &dev_data->wsched);

        schedule_work(&dev_data->init_work);
    } else {
        stk3420_set_irq(dev_data, false);

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

        stk3420_reg_write(dev_data->client, STK3420_CONFIG_REG, 0x00);
        dev_data->is_init_ok = false;
    }
}
#endif

static irqreturn_t stk3420_irqhandler(int irq, void *devid)
{
    struct stk3420_dev_data *dev_data = devid;

    if (!test_and_set_bit(INT_WORK_SCHED, &dev_data->wsched)) {
        schedule_work(&dev_data->int_work);
    }

    return IRQ_HANDLED;
}

static int ir_stk_dev_open(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct stk3420_dev_data *dev_data = container_of(miscdev, struct stk3420_dev_data, miscdev);

    if (atomic_read(&dev_data->opened)) {
        pr_err("EBUSY\n");
        return -EBUSY;
    }

    atomic_inc(&dev_data->opened);

    return 0;
}

static int ir_stk_dev_release(struct inode *inode, struct file *filp)
{
    struct miscdevice *miscdev = filp->private_data;
    struct stk3420_dev_data *dev_data = container_of(miscdev, struct stk3420_dev_data, miscdev);

    atomic_dec(&dev_data->opened);

    return 0;
}

static long ir_stk_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *miscdev = filp->private_data;
    struct stk3420_dev_data *dev_data = container_of(miscdev, struct stk3420_dev_data, miscdev);

    unsigned char var = (unsigned char)arg;

    switch(cmd) {
    case IR_DETECT_IOC_POWER:
        APGS_INFO("IR_DETECT_IOC_POWER, var=%d\n", var);
#ifdef IR_DETECT_OFF_POWER
        ir_stk_power(dev_data, var);
#else
        //ir_stk_sleep(dev_data, !var);
#endif
        break;

    case IR_DETECT_IOC_WAKEUP_EN:
        APGS_INFO("IR_DETECT_IOC_WAKEUP_EN, var=%d\n", var);
        if (var) {
            dev_data->wakeup_en = true;
            stk3420_set_irq(dev_data, true);
        } else {
            dev_data->wakeup_en = false;
            if (!dev_data->capture_en) {
                stk3420_set_irq(dev_data, false);
            }
        }
        break;

    case IR_DETECT_IOC_CAPTURE_EN:
        APGS_INFO("IR_DETECT_IOC_CAPTURE_EN, var=%d\n", var);
        if (var) {
            dev_data->capture_en = true;
            stk3420_set_irq(dev_data, true);
        } else {
            dev_data->capture_en = false;

            if (!dev_data->wakeup_en) {
                stk3420_set_irq(dev_data, false);
            }
        }
        break;

    case IR_DETECT_IOC_PROX_CAL: {
        int __user *cali_prox = (void __user *)arg;

        dev_data->calibrating = true;

        APGS_INFO("begin to init...\n");

        if (!dev_data->is_init_ok) {
            APGS_INFO("not init ok, reinit...\n");
#ifdef IR_DETECT_OFF_POWER
            ir_stk_power(dev_data, true);
#else
            //ir_stk_sleep(dev_data, false);
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
                APGS_INFO("init ok!\n");
            }
        }

        APGS_INFO("begin to calibrate...\n");

        if (!test_and_set_bit(CAL_WORK_SCHED, &dev_data->wsched)) {
            APGS_INFO("calibrate work start...\n");
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
        APGS_INFO("calibrate ok...\n");
        //mutex_unlock(&dev_data->lock);
        dev_data->calibrating = false;
    }
    break;

    case IR_DETECT_IOC_PROX_SET: {
        int value = (int)arg;
        APGS_INFO("IR_DETECT_IOC_PROX_SET, value=%d\n", value);
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

static struct file_operations ir_stk_dev_fops = {
    .owner          = THIS_MODULE,
    .open           = ir_stk_dev_open,
    .release        = ir_stk_dev_release,
    .unlocked_ioctl = ir_stk_dev_ioctl,
};

/*----------------------------------------------------------------------------*/
static ssize_t stk3420_show_send(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_store_send(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    int addr, cmd, ret;
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);

    if(2 != sscanf(buf, "%x %x", &addr, &cmd)) {
        pr_err("invalid format: '%s'\n", buf);
        return 0;
    }

    ret = stk3420_reg_write(dev_data->client, (u8)addr, (u8)cmd);
    if (ret < 0) {
        pr_err("%s: write i2c error\n", __func__);
        return ret;
    }

    APGS_INFO("send(%02X, %02X) = %d\n", addr, cmd, ret);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_show_recv(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);

    return sprintf(buf, "0x%04X\n", atomic_read(&dev_data->recv_reg));
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_store_recv(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);
    int addr;
    u8 dat;

    if (1 != sscanf(buf, "%x", &addr)) {
        pr_err("invalid format: '%s'\n", buf);
        return 0;
    }

    stk3420_reg_read(dev_data->client, (u8)addr, &dat);
    APGS_INFO("recv(%02X) = 0x%02X\n", addr, dat);
    atomic_set(&dev_data->recv_reg, dat);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_show_als(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int res;
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);

    if ((res = stk3420_get_als(dev_data->client, &dev_data->als))) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        return sprintf(buf, "0x%04X\n", dev_data->als);
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_show_ps(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    int res;
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);

    if((res = stk3420_get_ps(dev_data->client, &dev_data->ps))) {
        return sprintf(buf, "ERROR: %d\n", res);
    } else {
        return sprintf(buf, "0x%04X\n", dev_data->ps);
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_store_enable(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        APGS_ERR( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }
    APGS_INFO("%s: Enable GES : %d\n", __func__, (int)value);

    stk3420_enable_ges(dev_data->client, value);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3420_show_enable(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct stk3420_dev_data *dev_data = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", dev_data->ges_enabled);
}
/*----------------------------------------------------------------------------*/

static DEVICE_ATTR(send, S_IWUSR | S_IRUGO, stk3420_show_send, stk3420_store_send);
static DEVICE_ATTR(recv, S_IWUSR | S_IRUGO, stk3420_show_recv, stk3420_store_recv);
static DEVICE_ATTR(als, S_IRUGO, stk3420_show_als, NULL);
static DEVICE_ATTR(ps, S_IRUGO, stk3420_show_ps, NULL);
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, stk3420_show_enable, stk3420_store_enable);


static struct device_attribute *stk3420_attr_list[] = {
    &dev_attr_send,
    &dev_attr_recv,
    &dev_attr_als,
    &dev_attr_ps,
    &dev_attr_enable,
};

static int stk3420_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(stk3420_attr_list) / sizeof(stk3420_attr_list[0]));

    if (dev == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = device_create_file(dev, stk3420_attr_list[idx]))) {
            pr_err("driver_create_file (%s) = %d\n", stk3420_attr_list[idx]->attr.name, err);
            break;
        }
    }

    return err;
}

static int stk3420_delete_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(stk3420_attr_list) / sizeof(stk3420_attr_list[0]));

    if (!dev)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        device_remove_file(dev, stk3420_attr_list[idx]);
    }

    return err;
}

static int stk3420_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *did)
{
    int ret = 0;
    struct stk3420_dev_data *dev_data;
    struct input_dev *input_device;
    int error;
    int gpio;
    enum of_gpio_flags irq_flags, pwr_flags;
    struct device_node *np = client->dev.of_node;

    struct class *dev_class;
    struct device *ctl_dev;

    APGS_INFO("%s enter!\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality not supported!\n");
        return -ENODEV;
    }

    if (stk3420_check_pid(client) != 0) {
        ret = -ENODEV;
        #if 0
        if (gpio_is_valid(dev_data->pwr_pin)) {
            gpio_direction_output(dev_data->pwr_pin, !dev_data->pwr_en_level);
        }
        goto err_pid;
        #endif

        return -ENOMEM;
    }

    APGS_INFO("== Howrd === %s:%d\n", __func__, __LINE__);
    dev_data = kzalloc(sizeof(struct stk3420_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        pr_err("Failed to malloc stk3420_dev_data\n");
        return -ENOMEM;
    }

    if (!np) {
        dev_err(&client->dev, "no device tree\n");
        return -EINVAL;
    }

    dev_data->hw = stk_get_cust_alsps_hw();

    dev_data->reg.state_reg = 0;
    dev_data->reg.psgsctrl1_reg = dev_data->hw->psgsctrl1_reg;
    dev_data->reg.alsctrl1_reg = dev_data->hw->alsctrl1_reg;
    dev_data->reg.ledctrl_reg = dev_data->hw->ledctrl_reg;
    dev_data->reg.wait1_psgs_reg_ps = dev_data->hw->wait1_psgs_reg_ps;
    dev_data->reg.wait1_psgs_reg_gs = dev_data->hw->wait1_psgs_reg_gs;
    dev_data->reg.alsctrl2_reg = dev_data->hw->alsctrl2_reg;
    dev_data->reg.wait_als_reg = dev_data->hw->wait_als_reg;
    dev_data->reg.wait2_ps_reg = dev_data->hw->wait2_ps_reg;
    dev_data->reg.psgsctrl2_reg = dev_data->hw->psgsctrl2_reg;
    dev_data->reg.fifoctrl_reg = dev_data->hw->fifoctrl_reg;
    dev_data->reg.int_reg = 0;

    dev_data->miscdev.minor = MISC_DYNAMIC_MINOR;
    dev_data->miscdev.name  = client->name;
    dev_data->miscdev.fops  = &ir_stk_dev_fops;

    gpio = of_get_named_gpio_flags(np, "irq-stk", 0, &irq_flags);
    if (gpio_is_valid(gpio)) {
        dev_data->irq_pin = gpio;
        dev_data->irq_flags = irq_flags;
		APGS_INFO("== weiqifa === %s:%d irq:%d\n", __func__, __LINE__,gpio);
    } else {
        dev_data->irq_pin = -1;
		APGS_INFO("== weiqifa === %s:%d irq:%d\n", __func__, __LINE__,gpio);
    }

    gpio = of_get_named_gpio_flags(np, "power-gpio", 0, &pwr_flags);
    if (gpio_is_valid(gpio)) {
        dev_data->pwr_pin = of_get_named_gpio_flags(np, "power-gpio", 0, &pwr_flags);
        dev_data->pwr_en_level = (pwr_flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    } else {
        dev_data->pwr_pin = -1;
    }

    atomic_set(&dev_data->recv_reg, 0);

    if (misc_register(&dev_data->miscdev) < 0) {
        dev_err(&client->dev, "misc_register failed\n");
        goto err_misc_register;
    }

    dev_class = class_create(THIS_MODULE, "gesture");
    ctl_dev = device_create(dev_class, NULL, 0, NULL, "control");
    if (IS_ERR(ctl_dev)) {
        dev_err(ctl_dev, "Failed to create bt char device\n");
        ret = PTR_ERR(ctl_dev);
        goto err_create_dev;
    }

    dev_set_drvdata(ctl_dev, dev_data);

    if (stk3420_create_attr(ctl_dev)) {
        ret = -EINVAL;
        goto err_create_attr;
    }

    if (gpio_is_valid(dev_data->pwr_pin)) {
        ret = gpio_request(dev_data->pwr_pin, "stk3420 pwr");
        if (ret < 0) {
            dev_err(&client->dev, "%s:failed to set gpio pwr.\n", __func__);
            goto err_req_pwr_pin;
        }

        gpio_direction_output(dev_data->pwr_pin, !dev_data->pwr_en_level);
    }

    APGS_INFO("== Howrd === %s:%d\n", __func__, __LINE__);

    if (gpio_is_valid(dev_data->pwr_pin)) {
        gpio_direction_output(dev_data->pwr_pin, dev_data->pwr_en_level);
        msleep(1);
    }

    APGS_INFO("== Howrd === %s:%d\n", __func__, __LINE__);


#ifdef IR_DETECT_OFF_POWER
    if (gpio_is_valid(dev_data->pwr_pin)) {
        gpio_direction_output(dev_data->pwr_pin, !dev_data->pwr_en_level);
    }
#else
    stk3420_reg_write(client, STK3420_STATE_REG, 0x00);
#endif

    mutex_init(&dev_data->irq_lock);

	/* 注册传感器中断 */
    if (gpio_is_valid(dev_data->irq_pin)) {
        ret = gpio_request(dev_data->irq_pin, "stk3420 irq");
        if (ret < 0) {
            dev_err(&client->dev, "%s:failed to set gpio int.\n", __func__);
            goto err_req_int_pin;
        }

        gpio_direction_input(dev_data->irq_pin);

        /**
         * Request MCU IO interrupt source
         */
        dev_data->irq = gpio_to_irq(dev_data->irq_pin);
        if (dev_data->irq < 0) {
            error = dev_data->irq;
            dev_err(&client->dev, "Unable to get irq number for GPIO %d, error %d\n",
                    dev_data->irq_pin, error);
            goto err_request_irq;
        }

        error = devm_request_threaded_irq(&client->dev, dev_data->irq, NULL,      \
                                          stk3420_irqhandler,                     \
                                          IRQF_TRIGGER_FALLING | IRQF_ONESHOT,    \
                                          dev_name(&client->dev),                 \
                                          dev_data);
        if (error < 0) {
            dev_err(&client->dev, "request irq(%d) failed:%d\n",
                    dev_data->irq, error);
            goto err_request_irq;
        } else {
            enable_irq_wake(dev_data->irq);
        }

        mutex_lock(&dev_data->irq_lock);
        disable_irq(dev_data->irq);
        mutex_unlock(&dev_data->irq_lock);

		APGS_INFO("== weiqifa === %s:%d irq request success\n", __func__, __LINE__);
    }else {
		APGS_INFO("== weiqifa === %s:%d irq request failed\n", __func__, __LINE__);
	}

    input_device = input_allocate_device();
    if (!input_device) {
        ret = -ENOMEM;
        goto error_alloc_dev;
    }

    dev_data->input = input_device;
    input_device->name = "gesture";
    input_device->id.bustype = BUS_I2C;
    input_device->dev.parent = &client->dev;
    input_set_drvdata(input_device, dev_data);

    __set_bit(EV_KEY, input_device->evbit);
    __set_bit(EV_SYN, input_device->evbit);
    input_set_capability(input_device, EV_KEY, KEY_F1);
    input_set_capability(input_device, EV_KEY, KEY_F2);
    input_set_capability(input_device, EV_KEY, KEY_F3);
    input_set_capability(input_device, EV_KEY, KEY_F4);
    input_set_capability(input_device, EV_KEY, KEY_F5);

    ret = input_register_device(input_device);
    if (ret) {
        goto err_input_register;
    }

    init_waitqueue_head(&dev_data->init_wait);
    // init work queue
    INIT_WORK(&dev_data->init_work, stk3420_init_events);

    // interrupt work queue
    INIT_WORK(&dev_data->int_work, stk3420_int_workhandler);

    //init_waitqueue_head(&dev_data->cal_wait);
    // calibrate work queue
    //INIT_WORK(&dev_data->cal_work, stk3420_cali_workhandler);


    dev_data->ps_poll_wq = create_singlethread_workqueue("stk_ps_poll_wq");
    INIT_WORK(&dev_data->ps_poll_work, ps_poll_work_func);
    hrtimer_init(&dev_data->ps_poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    dev_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
    dev_data->ps_poll_timer.function = ps_poll_timer_func;
    dev_data->timer_run = 0;

    dev_data->client = client;
    wake_lock_init(&dev_data->work_wake_lock, WAKE_LOCK_SUSPEND, "ir_stk");

    wake_lock_init(&dev_data->gs_wake_lock, WAKE_LOCK_SUSPEND, "gs_wake_lock");

    i2c_set_clientdata(client, dev_data);

	/*初始化运行*/
	set_bit(INIT_WORK_SCHED, &dev_data->wsched);
    schedule_work(&dev_data->init_work);

    stk3420_enable_ges(dev_data->client, 3);

    return 0;

#if 0
    cancel_work_sync(&dev_data->init_work);

    if (input_device)
        input_unregister_device(input_device);
#endif

err_input_register:
    if (input_device)
        input_free_device(input_device);

error_alloc_dev:
    free_irq(dev_data->irq, dev_data);

err_request_irq:
    if (gpio_is_valid(dev_data->irq_pin)) {
        gpio_free(dev_data->irq_pin);
    }

err_req_int_pin:
    mutex_destroy(&dev_data->irq_lock);

err_req_pwr_pin:
    stk3420_delete_attr(ctl_dev);

err_create_attr:
    device_unregister(ctl_dev);

err_create_dev:
    if (dev_class) {
        class_destroy(dev_class);
    }

err_misc_register:
    kfree(dev_data);
#if 0
err_pid:
    if (gpio_is_valid(dev_data->pwr_pin)) {
        gpio_free(dev_data->pwr_pin);
    }
#endif
    return ret;
}

static int stk3420_i2c_remove(struct i2c_client *client)
{
    struct stk3420_dev_data *dev_data = i2c_get_clientdata(client);

    //cancel_work_sync(&dev_data->cal_work);
    //cancel_work_sync(&dev_data->int_work);
    //cancel_work_sync(&dev_data->init_work);
    wake_lock_destroy(&dev_data->work_wake_lock);
    //mutex_destroy(&dev_data->irq_lock);
    misc_deregister(&dev_data->miscdev);
    //gpio_free(dev_data->irq_pin);
    stk3420_delete_attr(&client->dev);
    //input_unregister_device(dev_data->input);
    //input_free_device(dev_data->input);
    kfree(dev_data);

    return 0;
}

static const struct i2c_device_id stk3420_ids[] = {
    {IR_STK_DEV_BASENAME, 0},
    {/*end of list*/}
};

MODULE_DEVICE_TABLE(i2c, stk3420_ids);

#ifdef CONFIG_PM

int stk3420_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct stk3420_dev_data *dev_data = i2c_get_clientdata(client);

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
    //stk3420_reg_write(dev_data->client, STK3420_INTERRUPT_REG, 0x00);

#if 0
    if (dev_data->wakeup_en || dev_data->capture_en) {
        stk3420_set_irq(dev_data, true);
    }
#endif

    return 0;
}

int stk3420_resume(struct device *dev)
{
    /*
        struct i2c_client *client = to_i2c_client(dev);
        struct stk3420_dev_data *dev_data = i2c_get_clientdata(client);
    */

    return 0;
}

static SIMPLE_DEV_PM_OPS(stk3420_i2c_pm, stk3420_suspend, stk3420_resume);
#endif

static struct of_device_id stk3420_dt_ids[] = {
    { .compatible = "sensortek,stk3420" },
    { }
};

static struct i2c_driver stk3420_i2c_driver = {
    .driver = {
        .name  = STK3420_DRV_NAME,
#ifdef CONFIG_PM
        .pm    = &stk3420_i2c_pm,
#endif
        .of_match_table = of_match_ptr(stk3420_dt_ids),
    },
    .probe      = stk3420_i2c_probe,
    .remove     = stk3420_i2c_remove,
    .id_table   = stk3420_ids,
};

static int __init stk3420_init(void)
{
    APGS_INFO("stk3420_init\n");
    return i2c_add_driver(&stk3420_i2c_driver);
}

static void __exit stk3420_exit(void)
{
    APGS_INFO("stk3420_exit\n");
    i2c_del_driver(&stk3420_i2c_driver);
}

//module_init(stk3420_init);
late_initcall(stk3420_init);
module_exit(stk3420_exit);


MODULE_AUTHOR("Howrd <howrd@21cn.com>");
MODULE_DESCRIPTION("Sensortek STK3420 ALS/PS/GS Driver");
MODULE_LICENSE("GPL");
