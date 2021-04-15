/*
 * linux/drivers/leds/leds-aw9110b.c
 *
 * aw9110b i2c led driver
 *
 * Copyright 2020, jimmy <lijiaming@knowin.com>
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/i2c/aw9110b.h>
#ifdef CONFIG_LEDS_TRIGGER_MULTI_CTRL
#include "leds-multi.h"
#endif
#include "leds.h"

#define DRV_NAME  "aw9110b-leds"

static const struct i2c_device_id aw9110b_id[] = {
    { DRV_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, aw9110b_id);

int aw9110b_leds_read(struct i2c_client *client, u8 reg)
{
    int ret = i2c_smbus_read_byte_data(client, reg);

    if (ret < 0)
        dev_err(&client->dev, "Read Error\n");

    return ret;
}
EXPORT_SYMBOL_GPL(aw9110b_leds_read);

int aw9110b_leds_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);

	//pr_info("ljm : i2c write === 0x%x\n", reg);
    if (ret < 0)
        dev_err(&client->dev, "Write Error\n");

    return ret;
}

EXPORT_SYMBOL_GPL(aw9110b_leds_write);

static void aw9110b_brightness_work(struct work_struct *work)
{
    const struct aw9110b_led_data *led_data = container_of(work,
            struct aw9110b_led_data, brightness_work);
    u8 aw_reg = 0xFF;
    u8 out_bit, mode;
    
    //pr_info("%s: set %s to %d\n", __func__, led_data->cdev.name, led_data->new_brightness);
    mutex_lock(&led_data->priv->led_mutex);

    out_bit = AW_GET_OUTBIT(led_data->flags);
    mode = AW_GET_MODE(led_data->flags);

    //pr_info("%s: out_bit:%d, mode:%d\n", __func__, out_bit, mode);

    switch (mode) {
        case AW_MODE_GPIO:
            pr_info("This mode(GPIO) is not available at present\n");
            break;
        case AW_MODE_LED:
			aw_reg = GPIO_GBO0_CC + out_bit;
            if (aw_reg != 0xFF) {
                aw9110b_leds_write(led_data->priv->client, aw_reg, led_data->new_brightness);
            }
            break;
    }
    mutex_unlock(&led_data->priv->led_mutex);
}

static void aw9110b_brightness_set(struct led_classdev *led_cdev,
                                   enum led_brightness brightness)
{
    struct aw9110b_led_data *led_data =
        container_of(led_cdev, struct aw9110b_led_data, cdev);

    led_data->new_brightness = brightness;
    //pr_info("AW9110 set %s brightness to %d\n", led_cdev->name, brightness);
    schedule_work(&led_data->brightness_work);
}

static void aw9110b_led_resume_work(struct work_struct *work)
{
    const struct aw9110b_priv *priv = container_of(work,
                                      struct aw9110b_priv, resume_work);
    struct i2c_client *client = priv->client;
    struct gpio_desc *power_gpio = priv->power_gpio;
    struct gpio_desc *reset_gpio = priv->reset_gpio;
    int i;
    u8 bmode;

    //pr_info("aw9110b_leds_resume:: set gpio default state\n");

    if (!IS_ERR(power_gpio)) {
        gpiod_direction_output(power_gpio, 1);
        // wait power stable
        msleep(1);
    }

    if (!IS_ERR(reset_gpio)) {
        gpiod_direction_output(reset_gpio, 1);
        udelay(50);
        gpiod_direction_output(reset_gpio, 0);
        udelay(50);
    }

    /* set LED mode */
    aw9110b_leds_write(client, GPMD_A, priv->modeA);
    aw9110b_leds_write(client, GPMD_B, priv->modeB);

    /* Configure output: open-drain or totem pole (push-pull) */
    if (priv->out_drv == AW9110B_TOTEM_POLE) {
        bmode = aw9110b_leds_read(client, CTL);
        bmode |= (1 << 4);
        aw9110b_leds_write(client, CTL, bmode);
    }


    /* set default current control value */
    for (i = 0; i < 10; i++) {
        aw9110b_leds_write(client, GPIO_GBO0_CC + i, priv->defcc[i]);
    }
}

static int aw9110b_parse_child_dt(const struct device *dev,
                                  struct device_node *child,
                                  struct aw9110b_led_data *led_data)
{
    struct led_classdev *cdev = &led_data->cdev;
    int ret = 0;
    u32 reg = -1;
    u32 blink_delay_on = 0;
    u32 blink_delay_off = 0;
    u8 group, out_bit, bmode, defval;

    if (of_property_read_string(child, "label", &cdev->name))
        cdev->name = child->name;

    ret = of_property_read_u32(child, "reg", &reg);
    if (ret || reg < 1) {
        dev_err(dev,
                "Child node %pOF does not have a valid reg property\n",
                child);
        return -EINVAL;
    }
    led_data->channel = reg;

    of_property_read_u32(child, "flags", &led_data->flags);

    bmode = AW_GET_MODE(led_data->flags);
    group = AW_GET_GROUP(led_data->flags);
    out_bit = AW_GET_OUTBIT(led_data->flags);
    defval = AW_GET_DEFVAL(led_data->flags);

    if (bmode == AW_MODE_LED) {
		led_data->priv->defcc[out_bit % 10] = defval;
    } else {
        pr_info("This mode(GPIO) is not available at present\n");
    }

    of_property_read_string(child, "linux,default-trigger",
                            &cdev->default_trigger);

    of_property_read_u32(child, "linux,blink-delay-on-ms",
                         &blink_delay_on);
    cdev->blink_delay_on = (u64)blink_delay_on;

    of_property_read_u32(child, "linux,blink-delay-off-ms",
                         &blink_delay_off);
    cdev->blink_delay_off = (u64)blink_delay_off;

    of_property_read_u32(child, "linux,default-trigger-delay-ms",
                         &led_data->register_delay);

    cdev->brightness_set = aw9110b_brightness_set;

    INIT_WORK(&led_data->brightness_work, aw9110b_brightness_work);
    return 0;
}

static struct aw9110b_led_data *aw9110b_find_led_data(
    struct aw9110b_priv *priv,
    u8 channel)
{
    size_t i;

    for (i = 0; i < priv->num_leds; i++) {
        if (priv->leds[i].channel == channel)
            return &priv->leds[i];
    }

    return NULL;
}

static void register_classdev_delayed(struct work_struct *ws)
{
    struct aw9110b_led_data *led_data =
        container_of(ws, struct aw9110b_led_data,
                     register_work.work);
    int ret;

    ret = devm_led_classdev_register(led_data->dev, &led_data->cdev);
    if (ret) {
        dev_err(led_data->dev, "failed to register PWM led for %s: %d\n",
                led_data->cdev.name, ret);
        return;
    }
}

static int aw9110b_parse_dt(struct device *dev,
                            struct aw9110b_priv *priv)
{
    struct device_node *child;
    int ret = 0;

    for_each_child_of_node(dev->of_node, child) {
        struct aw9110b_led_data *led_data =
                &priv->leds[priv->num_leds];
        const struct aw9110b_led_data *other_led_data;

        led_data->priv = priv;
        led_data->dev = dev;

        ret = aw9110b_parse_child_dt(dev, child, led_data);
        if (ret)
            goto err;

        /* Detect if channel is already in use by another child */
        other_led_data = aw9110b_find_led_data(priv,
                                               led_data->channel);
        if (other_led_data) {
            dev_err(dev,
                    "%s and %s both attempting to use channel %d\n",
                    led_data->cdev.name,
                    other_led_data->cdev.name,
                    led_data->channel);
            goto err;
        }
        if (led_data->register_delay) {
            INIT_DELAYED_WORK(&led_data->register_work,
                              register_classdev_delayed);
            schedule_delayed_work(&led_data->register_work,
                                  msecs_to_jiffies(led_data->register_delay));
        } else {
            ret = devm_led_classdev_register(dev, &led_data->cdev);
            if (ret) {
                dev_err(dev, "failed to register PWM led for %s: %d\n",
                        led_data->cdev.name, ret);
                goto err;
            }
        }
        priv->num_leds++;
    }

    return 0;

err:
    of_node_put(child);
    return ret;
}

static inline size_t sizeof_aw9110b_priv(int num_leds)
{
    return sizeof(struct aw9110b_priv) +
           (sizeof(struct aw9110b_led_data) * num_leds);
}

//20H DIM0 OUT0 LED current control
//21H DIM1 OUT1 LED current control
//22H DIM2 OUT2 LED current control
//23H DIM3 OUT3 LED current control
//24H DIM4 OUT4 LED current control
//25H DIM5 OUT5 LED current control
//26H DIM6 OUT6 LED current control
//27H DIM7 OUT7 LED current control
//28H DIM8 OUT8 LED current control
//29H DIM9 OUT9 LED current control
static int aw9110b_leds_probe(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct aw9110b_priv *priv;
    int count;
	//int reg_val;
    int i, ret;
    struct gpio_desc *power_gpio;
    struct gpio_desc *reset_gpio;
	u8 bmode;
    u32 gpio_out_drv;

    pr_info("%s Enter!\n", __func__);

    power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
    if (IS_ERR(power_gpio)) {
        int error = PTR_ERR(power_gpio);
        dev_err(dev, "Failed to get power gpio: %d\n", error);
    } else {
        gpiod_direction_output(power_gpio, 1);
        // wait power stable
        msleep(1);
    }

    reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(reset_gpio)) {
        int error = PTR_ERR(reset_gpio);
        dev_err(dev, "Failed to get reset gpio: %d\n", error);
    } else {
        gpiod_direction_output(reset_gpio, 1);
        udelay(30);
        gpiod_direction_output(reset_gpio, 0);
        udelay(5);
    }

    ret = of_property_read_u32(dev->of_node, "gpio_out_drv", &gpio_out_drv);
    if (ret) {
        dev_err(dev, "Child node %pOF does not have a valid gpio_out_drv property\n",
                dev->of_node);
    }

    pr_info("%s parse gpio_out_drv:%d\n", __func__, gpio_out_drv);

    count = of_get_child_count(dev->of_node);
    if (!count) {
        dev_err(dev, "count is invalid\n");
        return -EINVAL;
    }

    priv = devm_kzalloc(dev, sizeof_aw9110b_priv(count), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    mutex_init(&priv->led_mutex);
    priv->client = client;
    priv->power_gpio = power_gpio;
    priv->reset_gpio = reset_gpio;
    priv->out_drv = gpio_out_drv;
	priv->modeA = 0x0;
	priv->modeB = 0x0;
    i2c_set_clientdata(client, priv);

    aw9110b_parse_dt(dev, priv);


    /* set LED mode */
    aw9110b_leds_write(client, GPMD_A, priv->modeA);
    aw9110b_leds_write(client, GPMD_B, priv->modeB);

    /* Configure output: open-drain or totem pole (push-pull) */
    if (priv->out_drv == AW9110B_TOTEM_POLE) {
        bmode = aw9110b_leds_read(client, CTL);
        bmode |= (1 << 4);
        aw9110b_leds_write(client, CTL, bmode);
    }


    /* set default current control value */
    for (i = 0; i < 10; i++) {
        aw9110b_leds_write(client, GPIO_GBO0_CC + i, priv->defcc[i]);
    }


    INIT_WORK(&priv->resume_work, aw9110b_led_resume_work);
    return 0;

}

static int aw9110b_leds_remove(struct i2c_client *client)
{
    struct aw9110b_priv *priv = i2c_get_clientdata(client);
    int i;

    for (i = 0; i < priv->num_leds; i++) {
        struct aw9110b_led_data *led_data = &priv->leds[i];
        cancel_delayed_work_sync(&led_data->register_work);
        cancel_work_sync(&led_data->brightness_work);
    }

    if (!IS_ERR(priv->power_gpio)) {
        gpiod_direction_output(priv->power_gpio, 0);
    }

    return 0;
}

static int aw9110b_leds_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct aw9110b_priv *priv = i2c_get_clientdata(client);
    int i;
    u8 bmode;

    if (!IS_ERR(priv->power_gpio)) {
        pr_info("aw9110b_leds_suspend:: power down...\n");
        gpiod_direction_output(priv->power_gpio, 0);
        return 0;
    }

    pr_info("aw9110b_leds_suspend:: set default state\n");

    /* set LED mode */
    aw9110b_leds_write(client, GPMD_A, priv->modeA);
    aw9110b_leds_write(client, GPMD_B, priv->modeB);

    /* Configure output: open-drain or totem pole (push-pull) */
    if (priv->out_drv == AW9110B_TOTEM_POLE) {
        bmode = aw9110b_leds_read(client, CTL);
        bmode |= (1 << 4);
        aw9110b_leds_write(client, CTL, bmode);
    }

    /* set default current control value */
    for (i = 0; i < 10; i++) {
        aw9110b_leds_write(client, GPIO_GBO0_CC + i, priv->defcc[i]);
    }
    return 0;
}

static int aw9110b_leds_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct aw9110b_priv *priv = i2c_get_clientdata(client);

    if (!IS_ERR(priv->power_gpio)) {
        schedule_work(&priv->resume_work);
    } else {
        pr_info("aw9110b_leds_resume:: do nothing...\n");
    }
    return 0;
}


static SIMPLE_DEV_PM_OPS(aw9110b_leds_pm, aw9110b_leds_suspend, aw9110b_leds_resume);


static struct i2c_driver aw9110b_driver = {
    .driver = {
        .name   = DRV_NAME,
        .owner  = THIS_MODULE,
        .pm = &aw9110b_leds_pm,
    },
    .probe  = aw9110b_leds_probe,
    .remove = aw9110b_leds_remove,
    .id_table = aw9110b_id,
};

static int __init aw9110b_leds_init(void)
{
    return i2c_add_driver(&aw9110b_driver);
}

static void __exit aw9110b_leds_exit(void)
{
    i2c_del_driver(&aw9110b_driver);
}

//late_initcall_sync(aw9110b_leds_init);
module_init(aw9110b_leds_init);
module_exit(aw9110b_leds_exit);

MODULE_AUTHOR("lijiaming@knowin.com");
MODULE_DESCRIPTION("aw9110b LED driver");
MODULE_LICENSE("GPL v2");
