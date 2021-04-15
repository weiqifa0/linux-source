/*
 * AW9110B I2C I/O Expander
 *
 * Author: jimmy <lijiaming@knowin.com>
 *
 */

#ifndef _AW9110B_H
#define _AW9110B_H

#ifdef CONFIG_LEDS_AW9110B
#include <linux/leds.h>
#endif


#define GPIO_INPUT_A                 0x00
#define GPIO_INPUT_B                 0x01
#define GPIO_OUTPUT_A                0x02
#define GPIO_OUTPUT_B                0x03
#define GPIO_CFG_A                   0x04
#define GPIO_CFG_B                   0x05
#define GPIO_INTN_A                  0x06
#define GPIO_INTN_B                  0x07
#define CTL                          0x11 /* Global control register */

#define GPMD_A                       0x12
#define GPMD_B                       0x13

#define EN_BRE                       0x14
#define FADE_TMR                     0x15
#define FULL_TMR                     0x16

#define GPIO_GBO0_CC                 0x20
#define GPIO_GBO1_CC                 0x21
#define GPIO_GBO2_CC                 0x22
#define GPIO_GBO3_CC                 0x23
#define GPIO_GAO4_CC                 0x24
#define GPIO_GAO5_CC                 0x25
#define GPIO_GAO6_CC                 0x26
#define GPIO_GAO7_CC                 0x27
#define GPIO_GAO8_CC                 0x28
#define GPIO_GAO9_CC                 0x29

#define RESET_REGITER                0x7F // write 0x00 to reset

/*fade-on or fade-off time setting*/
#define FADE_TMR_0                   0x00 /* 0ms */
#define FADE_TMR_315                 0x01 /* 315ms */
#define FADE_TMR_630                 0x02 /* 630ms */
#define FADE_TMR_1260                0x03 /* 1260ms */
#define FADE_TMR_2520                0x04 /* 2520ms */
#define FADE_TMR_5040                0x05 /* 5040ms */

/*all-on or all-off time setting*/
#define FULL_TMR_0                   0x00 /* 0ms */
#define FULL_TMR_315                 0x01 /* 315ms */
#define FULL_TMR_630                 0x02 /* 630ms */
#define FULL_TMR_1260                0x03 /* 1260ms */
#define FULL_TMR_2520                0x04 /* 2520ms */
#define FULL_TMR_5040                0x05 /* 5040ms */
#define FULL_TMR_10080               0x06 /* 10080ms */
#define FULL_TMR_20160               0x07 /* 20160ms */


/* ID register (0x10) */
#define	AW9523_ID                   0x23

/* Global Control register (0x11) */
#define AW9110B_P0_PP	           (1 << 4)

#define AW9110B_IRANGE_ALL         (0x00)
#define AW9110B_IRANGE_34          (0x01)
#define AW9110B_IRANGE_24          (0x02)
#define AW9110B_IRANGE_14          (0x03)


#define AW9110B_MAXGPIO		16
#define AW9110B_BANK(offs)	((offs) >> 3)
#define AW9110B_BIT(offs)	(1u << ((offs) & 0x7))


//struct i2c_client; /* forward declaration */

#ifdef CONFIG_GPIO_AW9110B
struct AW9110B_gpio_platform_data {
    unsigned int reset; /* reset pin */
    unsigned int irq; /* interrupt pin */
    int wakeup; /* whether wakeup en */
	int gpio_start;		/* GPIO Chip base # */
	const char *const *names;
	unsigned irq_base;	/* interrupt base # */
	int	(*setup)(struct i2c_client *client,
				int gpio, unsigned ngpio,
				void *context);
	int	(*teardown)(struct i2c_client *client,
				int gpio, unsigned ngpio,
				void *context);
	void *context;
};
#endif


#ifdef CONFIG_LEDS_AW9110B

#define AW_GROUPA  0
#define AW_GROUPB  1

#define AW_OUT0  0
#define AW_OUT1  1
#define AW_OUT2  2
#define AW_OUT3  3
#define AW_OUT4  4
#define AW_OUT5  5
#define AW_OUT6  6
#define AW_OUT7  7
#define AW_OUT8  8
#define AW_OUT9  9

#define AW_MODE_GPIO  1
#define AW_MODE_LED   0

#define AW_MAKE_FLAGS(_group, _bit, _mode, _defval)  (((_group) << 24) | \
                                                    ((_bit) << 16) | \
                                                    ((_mode) << 8)| \
                                                    (_defval))

#define AW_GET_GROUP(_flags)  ((unsigned char)((unsigned int)(_flags) >> 24))
#define AW_GET_OUTBIT(_flags)  ((unsigned char)((unsigned int)(_flags) >> 16))
#define AW_GET_MODE(_flags)  ((unsigned char)((unsigned int)(_flags) >> 8))
#define AW_GET_DEFVAL(_flags)  ((unsigned char)(_flags))

enum AW9110B_outdrv {
	AW9110B_OPEN_DRAIN,
	AW9110B_TOTEM_POLE, /* push-pull */
};

struct AW9110B_leds_platform_data {
    unsigned int power; /* power pin */
    unsigned int reset; /* reset pin */
    enum AW9110B_outdrv outdrv;
    struct led_platform_data leds;
};

#endif

#endif
