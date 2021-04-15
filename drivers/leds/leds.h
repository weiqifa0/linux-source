/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/rwsem.h>
#include <linux/leds.h>

struct aw9110b_priv;
struct aw9110b_led_data {
    struct led_classdev cdev;
    u8 channel; /* 1-based, max priv->cdef->channels */
    u32 flags; /* see AW_MAKE_FLAGS */
    struct aw9110b_priv *priv;
    u32 register_delay;
    struct device *dev;
    struct delayed_work register_work;
    struct work_struct brightness_work;
    enum led_brightness new_brightness;
};

struct aw9110b_priv {
    struct i2c_client *client;
    u32 num_leds;
    struct mutex led_mutex;
    struct gpio_desc *power_gpio;
    struct gpio_desc *reset_gpio;

    u32 out_drv;

    u8 modeA:6;
	u8 modeB:4;
    u8 defcc[10];
    struct work_struct resume_work;

    struct aw9110b_led_data leds[0];
};


static inline void led_set_brightness_async(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	value = min(value, led_cdev->max_brightness);
	led_cdev->brightness = value;

	if (!(led_cdev->flags & LED_SUSPENDED))
		led_cdev->brightness_set(led_cdev, value);
}

static inline int led_set_brightness_sync(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	int ret = 0;

	led_cdev->brightness = min(value, led_cdev->max_brightness);

	if (!(led_cdev->flags & LED_SUSPENDED))
		ret = led_cdev->brightness_set_sync(led_cdev,
						led_cdev->brightness);
	return ret;
}

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

void led_init_core(struct led_classdev *led_cdev);
void led_stop_software_blink(struct led_classdev *led_cdev);

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

int aw9110b_leds_read(struct i2c_client *client, u8 reg);
int aw9110b_leds_write(struct i2c_client *client, u8 reg, u8 val);


#endif	/* __LEDS_H_INCLUDED */
