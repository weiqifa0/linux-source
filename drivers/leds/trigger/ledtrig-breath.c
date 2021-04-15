/*
 * LED Breath Trigger
 *
 * Copyright (C) 2020 Howrd <howrd@21cn.com>
 *
 * Based on Richard Purdie's ledtrig-timer.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/leds.h>
#include <linux/reboot.h>
#include "../leds.h"
#include <linux/i2c/aw9110b.h>

#define LED_BREATH_EFFECT1

#define ARR_SIZE(_a)   (sizeof(_a) / sizeof((_a)[0]))

static int panic_breaths;

struct breath_trig_data {
    unsigned int phase;
    unsigned int period;
    struct timer_list timer;
    unsigned int invert;
};

// 100 level brightness
// rising cureve
// y = 0.5 * x^2
// falling curve
// y = 0.1 * (x - 100) * (x - 100);

#if defined(LED_BREATH_EFFECT1)

static const uint8_t s_breath_effect[] = {
    1, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
    120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
    130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
    140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
    150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
    160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
    170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
    180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
    190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
    200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
    200, 199, 198, 197, 196, 196, 195, 194, 193, 192,
    192, 191, 190, 189, 188, 188, 187, 186, 185, 184,
    184, 183, 182, 181, 180, 180, 179, 178, 177, 176,
    176, 175, 174, 173, 172, 172, 171, 170, 169, 168,
    168, 167, 166, 165, 164, 164, 163, 162, 161, 160,
    160, 159, 158, 157, 157, 156, 155, 154, 153, 153,
    152, 151, 150, 149, 149, 148, 147, 146, 145, 145,
    144, 143, 142, 141, 141, 140, 139, 138, 137, 137,
    136, 135, 134, 133, 133, 132, 131, 130, 129, 129,
    128, 127, 126, 125, 125, 124, 123, 122, 121, 121,
    120, 119, 118, 118, 117, 116, 115, 114, 114, 113,
    112, 111, 110, 110, 109, 108, 107, 106, 106, 105,
    104, 103, 102, 102, 101, 100, 99, 98, 98, 97,
    96, 95, 94, 94, 93, 92, 91, 90, 90, 89,
    88, 87, 86, 86, 85, 84, 83, 82, 82, 81,
    80, 79, 79, 78, 77, 76, 75, 75, 74, 73,
    72, 71, 71, 70, 69, 68, 67, 67, 66, 65,
    64, 63, 63, 62, 61, 60, 59, 59, 58, 57,
    56, 55, 55, 54, 53, 52, 51, 51, 50, 49,
    48, 47, 47, 46, 45, 44, 43, 43, 42, 41,
    40, 40, 39, 38, 37, 36, 36, 35, 34, 33,
    32, 32, 31, 30, 29, 28, 28, 27, 26, 25,
    24, 24, 23, 22, 21, 20, 20, 19, 18, 17,
    16, 16, 15, 14, 13, 12, 12, 11, 10, 9,
    8, 8, 7, 6, 5, 4, 4, 3, 2, 1,
};

#elif defined LED_BREATH_EFFECT2

static const uint8_t s_breath_effect[] = {
    0, 0, 0, 0, 1, 2, 3, 4, 6, 8,
    10, 12, 14, 16, 19, 22, 25, 28, 32, 36,
    40, 44, 48, 52, 57, 62, 67, 72, 78, 84,
    90, 96, 102, 108, 115, 122, 129, 136, 144, 152,
    160, 168, 176, 184, 193, 202, 211, 220, 230, 240,
    250, 240, 230, 220, 211, 202, 193, 184, 176, 168,
    160, 152, 144, 136, 129, 122, 115, 108, 102, 96,
    90, 84, 78, 72, 67, 62, 57, 52, 48, 44,
    40, 36, 32, 28, 25, 22, 19, 16, 14, 12,
    10, 8, 6, 4, 3, 2, 1, 0, 0, 0,
};

#else

static const uint8_t s_breath_effect[] = {
    0, 9, 19, 29, 38, 47, 56, 65, 73, 81,
    90, 97, 105, 113, 120, 127, 134, 141, 147, 153,
    160, 165, 171, 177, 182, 187, 192, 197, 201, 205,
    210, 213, 217, 221, 224, 227, 230, 233, 235, 237,
    240, 241, 243, 245, 246, 247, 248, 249, 249, 249,
    250, 240, 230, 220, 211, 202, 193, 184, 176, 168,
    160, 152, 144, 136, 129, 122, 115, 108, 102, 96,
    90, 84, 78, 72, 67, 62, 57, 52, 48, 44,
    40, 36, 32, 28, 25, 22, 19, 16, 14, 12,
    10, 8, 6, 4, 3, 2, 1, 0, 0, 0,
};

#endif

#if 1
static void led_breath_function(unsigned long data)
{
    struct led_classdev *led_cdev = (struct led_classdev *) data;
    struct breath_trig_data *breath_data = led_cdev->trigger_data;
    unsigned long brightness = LED_OFF;
    unsigned long delay = 0;

    if (unlikely(panic_breaths)) {
        led_set_brightness(led_cdev, LED_OFF);
        return;
    }

    /* acts like an actual breath... */
    if (breath_data->phase < ARR_SIZE(s_breath_effect)) {
        delay = msecs_to_jiffies(10);
        brightness = s_breath_effect[breath_data->phase];
        breath_data->phase++;
        breath_data->phase %= ARR_SIZE(s_breath_effect);
        if (breath_data->invert)
            brightness = led_cdev->max_brightness - brightness;
    }
    else {
        delay = msecs_to_jiffies(10);
        breath_data->phase = 0;
        brightness = 0;
        if (breath_data->invert)
            brightness = led_cdev->max_brightness;
    }

    led_set_brightness_async(led_cdev, brightness);
    mod_timer(&breath_data->timer, jiffies + delay);
}

static ssize_t led_invert_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct breath_trig_data *breath_data = led_cdev->trigger_data;

    return sprintf(buf, "%u\n", breath_data->invert);
}

static ssize_t led_invert_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct breath_trig_data *breath_data = led_cdev->trigger_data;
    unsigned long state;
    int ret;

    ret = kstrtoul(buf, 0, &state);
    if (ret)
        return ret;

    breath_data->invert = !!state;

    return size;
}

static DEVICE_ATTR(invert, 0644, led_invert_show, led_invert_store);

static void breath_trig_activate(struct led_classdev *led_cdev)
{
    struct breath_trig_data *breath_data;
    int rc;

    breath_data = kzalloc(sizeof(*breath_data), GFP_KERNEL);
    if (!breath_data)
        return;

    led_cdev->trigger_data = breath_data;
    rc = device_create_file(led_cdev->dev, &dev_attr_invert);
    if (rc) {
        kfree(led_cdev->trigger_data);
        return;
    }

    setup_timer(&breath_data->timer,
            led_breath_function, (unsigned long) led_cdev);
    breath_data->phase = 0;
    led_breath_function(breath_data->timer.data);
    led_cdev->activated = true;
}

static void breath_trig_deactivate(struct led_classdev *led_cdev)
{
    struct breath_trig_data *breath_data = led_cdev->trigger_data;

    if (led_cdev->activated) {
        del_timer_sync(&breath_data->timer);
        device_remove_file(led_cdev->dev, &dev_attr_invert);
        kfree(breath_data);
        led_cdev->activated = false;
    }
}
#endif

#if 0
static void breath_trig_activate_new(struct led_classdev *led_cdev)
{
	u8 out_bit, reg_val;
	struct aw9110b_led_data *led_data =
        container_of(led_cdev, struct aw9110b_led_data, cdev);

	mutex_lock(&led_data->priv->led_mutex);
	pr_info("ljm : enter breath_trig_activate_new\n");

	out_bit = AW_GET_OUTBIT(led_data->flags);

	/* enable breath mode */
	reg_val = aw9110b_leds_read(led_data->priv->client, EN_BRE);
	pr_info("ljm : before set: reg_val=0x%x\n", reg_val);
	reg_val |= (1<<out_bit); /* OUT3~OUT5 */
	aw9110b_leds_write(led_data->priv->client, EN_BRE, reg_val);
	reg_val = aw9110b_leds_read(led_data->priv->client, EN_BRE);
	pr_info("ljm : after set: reg_val=0x%x\n", reg_val);
	/* breath led time setting */
	/* fade-on and fade-off */
	aw9110b_leds_write(led_data->priv->client, FADE_TMR, (FADE_TMR_630<<0)|(FADE_TMR_630<<3));
	/* all-on and all-off */
	aw9110b_leds_write(led_data->priv->client, FULL_TMR, (FULL_TMR_630<<0)|(FULL_TMR_630<<3));
	

	/*enable BLINK mode*/
	if (out_bit == AW_OUT3) {
		reg_val = aw9110b_leds_read(led_data->priv->client, GPIO_CFG_B);
		reg_val |= (1<<out_bit);
		aw9110b_leds_write(led_data->priv->client, GPIO_CFG_B, reg_val);
	} else {
		out_bit -= 4;
		reg_val = aw9110b_leds_read(led_data->priv->client, GPIO_CFG_A);
		reg_val |= (1<<out_bit);
		aw9110b_leds_write(led_data->priv->client, GPIO_CFG_A, reg_val);
	}

	/* enable breathing in BLINK mode */
	reg_val = aw9110b_leds_read(led_data->priv->client, CTL);
    reg_val |= (1 << 7);
    aw9110b_leds_write(led_data->priv->client, CTL, reg_val);

	led_cdev->activated = true;

	mutex_unlock(&led_data->priv->led_mutex);
}

static void breath_trig_deactivate_new(struct led_classdev *led_cdev)
{
	u8 out_bit, reg_val;
	struct aw9110b_led_data *led_data =
        container_of(led_cdev, struct aw9110b_led_data, cdev);
	pr_info("ljm : in breath_trig_deactivate_new\n");

    if (led_cdev->activated) {
		mutex_lock(&led_data->priv->led_mutex);
		out_bit = AW_GET_OUTBIT(led_data->flags);

		/*disable BLINK mode*/
		if (out_bit == AW_OUT3) {
			reg_val = aw9110b_leds_read(led_data->priv->client, GPIO_CFG_B);
			reg_val &= ~(1<<out_bit);
			aw9110b_leds_write(led_data->priv->client, GPIO_CFG_B, reg_val);
		} else {
			out_bit -= 4;
			reg_val = aw9110b_leds_read(led_data->priv->client, GPIO_CFG_A);
			reg_val &= ~(1<<out_bit);
			aw9110b_leds_write(led_data->priv->client, GPIO_CFG_A, reg_val);
		}
        led_cdev->activated = false;
		mutex_unlock(&led_data->priv->led_mutex);
    }
}
#endif


#if 1
static struct led_trigger breath_led_trigger = {
    .name     = "breath",
    .activate = breath_trig_activate,
    .deactivate = breath_trig_deactivate,
};
#else
static struct led_trigger breath_led_trigger = {
    .name     = "breath",
    .activate = breath_trig_activate_new,
    .deactivate = breath_trig_deactivate_new,
};

#endif

static int breath_reboot_notifier(struct notifier_block *nb,
                     unsigned long code, void *unused)
{
    led_trigger_unregister(&breath_led_trigger);
    return NOTIFY_DONE;
}

static int breath_panic_notifier(struct notifier_block *nb,
                     unsigned long code, void *unused)
{
    panic_breaths = 1;
    return NOTIFY_DONE;
}

static struct notifier_block breath_reboot_nb = {
    .notifier_call = breath_reboot_notifier,
};

static struct notifier_block breath_panic_nb = {
    .notifier_call = breath_panic_notifier,
};

static int __init breath_trig_init(void)
{
    int rc = led_trigger_register(&breath_led_trigger);

    if (!rc) {
        atomic_notifier_chain_register(&panic_notifier_list,
                           &breath_panic_nb);
        register_reboot_notifier(&breath_reboot_nb);
    }
    return rc;
}

static void __exit breath_trig_exit(void)
{
    unregister_reboot_notifier(&breath_reboot_nb);
    atomic_notifier_chain_unregister(&panic_notifier_list,
                     &breath_panic_nb);
    led_trigger_unregister(&breath_led_trigger);
}

module_init(breath_trig_init);
module_exit(breath_trig_exit);

MODULE_AUTHOR("Howrd <howrd@21cn.com>");
MODULE_DESCRIPTION("Breath LED trigger");
MODULE_LICENSE("GPL");
