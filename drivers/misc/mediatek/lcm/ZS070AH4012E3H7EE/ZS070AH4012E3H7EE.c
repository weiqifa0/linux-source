/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif

#include "lcm_drv.h"

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE	0xFF   /* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE	0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

/**
 * Local Variables
 */
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/**
 * Local Functions
 */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg				lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifndef ASSERT
#define ASSERT(expr)					\
	do {						\
		if (expr)				\
			break;				\
		pr_debug("DDP ASSERT FAILED %s, %d\n",	\
		       __FILE__, __LINE__);		\
		BUG();					\
	} while (0)
#endif

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/*
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out 
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },

	// Display ON 
	{0x29, 0, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};



static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Sleep Mode On 
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
*/

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;
		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}

#ifndef BUILD_LK
static struct regulator *lcm_vgp;
static unsigned int GPIO_LCD_PWR;
static unsigned int GPIO_LCD_RST;


/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	printk("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	printk("LCM: lcm get supply ok.\n");

	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	printk("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

 int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	printk("LCM: lcm_vgp_supply_enable\n");

	if (lcm_vgp == NULL)
		return 0;

	printk("LCM: set regulator voltage lcm_vgp voltage to 3.3V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		printk("LCM: check regulator voltage=3300000 pass!\n");
	else
		pr_debug("LCM: check regulator voltage=3300000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

 int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;
    int count=6;
	if (lcm_vgp == NULL)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	printk("LCM: lcm query regulator enable status[%d]\n", isenable);

	if (isenable) {
				while(count-- && isenable)
			{
				ret = regulator_disable(lcm_vgp);
				isenable = regulator_is_enabled(lcm_vgp);
			}
				if (ret != 0) {
					pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
					return ret;
				}
				/* verify */
				isenable = regulator_is_enabled(lcm_vgp);
				if (!isenable)
					pr_err("LCM: lcm regulator disable pass\n");
			}

	return ret;
}

static void lcm_request_gpio_control(struct device *dev)
{
	
	GPIO_LCD_PWR = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr", 0);
	gpio_request(GPIO_LCD_PWR, "GPIO_LCD_PWR");

	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
}
static int lcm_driver_probe(struct device *dev, void const *data)
{
    printk("LCM: lcm_driver_probe\n");
	lcm_request_gpio_control(dev);
	lcm_get_vgp_supply(dev);
	lcm_vgp_supply_enable();

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "ZS,ZS070AH4012E3H7EE",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "ZS070AH4012E3H7EE",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_init(void)
{
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
#endif
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH  (1024)
#define FRAME_HEIGHT (600)

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params,0,sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	
	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->dsi.data_format.format= LCM_DSI_FORMAT_RGB888;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active 	= 10;
	params->dsi.vertical_backporch 		= 23;
	params->dsi.vertical_frontporch 	= 12;
	params->dsi.vertical_active_line 		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active 	= 20;
	params->dsi.horizontal_backporch 	= 160;
	params->dsi.horizontal_frontporch 	= 160;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PLL_CLOCK 				= 160;

	params->dsi.ssc_disable = 1;
	params->dsi.cont_clock  = 1;
	//params->dsi.PLL_CLOCK   = 247;


	params->dsi.clk_lp_per_line_enable   = 0;
//	params->dsi.esd_check_enable = 0;
//	params->dsi.customization_esd_check_enable = 0;
//	params->dsi.lcm_esd_check_table[0].cmd = 0x53;
//	params->dsi.lcm_esd_check_table[0].count = 1;
//	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	
	gpio_set_value(GPIO, output);
#endif
}

static void lcm_init_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_power() enter\n");

	SET_RESET_PIN(0);
	MDELAY(20);
	
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(20);

	mt6392_upmu_set_rg_vgp2_vosel(3);
	mt6392_upmu_set_rg_vgp2_en(0x1);

	SET_RESET_PIN(1);
	MDELAY(20);
#else
	pr_err("[Kernel/LCM] lcm_init_power() enter\n");
#endif
}
static struct LCM_setting_table lcm_initialization_setting[] = {

	/*
	Note :

	Data ID will depends on the following rule.

	    count of parameters > 1 => Data ID = 0x39
	    count of parameters = 1 => Data ID = 0x15
	    count of parameters = 0 => Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},
	...
	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{REGFLAG_DELAY, 50, {}}, 
	
	{0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {}},					
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};
static void lcm_init_lcm(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
	if(1)
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
	pr_err("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);	
	//SET_RESET_PIN(0);
	MDELAY(50);
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);	
	MDELAY(50);
	lcm_vgp_supply_disable();
	MDELAY(50);
#endif
}

static void lcm_resume_power(void)
{
#ifndef BUILD_LK
	pr_err("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_vgp_supply_enable();
	MDELAY(30);
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(30);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);
#endif
	lcm_init_lcm();
}

static void lcm_suspend(void)
{
	
	//push_table(lcm_deep_sleep_mode_in_setting,
//		   sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

  //  lcm_vgp_supply_disable();
//	MDELAY(20);

}


static void lcm_resume(void)
{
	//lcm_vgp_supply_enable();
	//MDELAY(20);
	//push_table(lcm_sleep_out_setting,
//		   sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


LCM_DRIVER ZS070AH4012E3H7EE_lcm_drv = {
	.name		= "ZS070AH4012E3H7EE_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power     = lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
/* .esd_check		= lcm_esd_check, */
/* .esd_recover	= lcm_esd_recover, */
#if (LCM_DSI_CMD_MODE)
	/*.set_backlight	= lcm_setbacklight,*/
	/* .set_pwm        = lcm_setpwm, */
	/* .get_pwm        = lcm_getpwm, */
	/*.update         = lcm_update, */
#endif
};
