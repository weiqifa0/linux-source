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

#include "fan5405.h"
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <mt-plat/charging.h>
//#include <mt-plat/mt_gpio.h>

#define fan5405_SLAVE_ADDR_WRITE   0xD4
#define fan5405_SLAVE_ADDR_Read    0xD5

static int ic_vendor=0;


static struct i2c_client *new_client;
static const struct i2c_device_id fan5405_i2c_id[] = { {"fan5405", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;
static int fan5405_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id fan5405_of_match[] = {
	{.compatible = "fan5405",},
	{},
};

MODULE_DEVICE_TABLE(of, fan5405_of_match);
#endif

static struct i2c_driver fan5405_driver = {
	.driver = {
		   .name = "fan5405",
#ifdef CONFIG_OF
		   .of_match_table = fan5405_of_match,
#endif
	},

	.probe = fan5405_driver_probe,
	.id_table = fan5405_i2c_id,
};

u8 fan5405_reg[fan5405_REG_NUM] = { 0 };

static DEFINE_MUTEX(fan5405_i2c_access);
struct pinctrl *fan_pinctrl;
struct pinctrl_state *pinctrl_otg_drvvbus_low;
struct pinctrl_state *pinctrl_otg_drvvbus_high;
struct pinctrl_state *pinctrl_chr_dis_low;
struct pinctrl_state *pinctrl_chr_dis_high;
int temp_sync = 0;
int fan5405_read_byte(u8 cmd, u8 *returnData)
{
	char readData = 0;
	int ret = 0;
		struct i2c_msg msg[2];
		struct i2c_adapter *adap ;//= new_client->adapter;
		if(new_client == NULL)
			return -1;
		if(temp_sync)
		   return 0;
		temp_sync++;
		adap = new_client->adapter;

		//mutex_lock(&fan5405_i2c_access);
		msg[0].addr = new_client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &cmd;

		msg[1].addr = new_client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = 1;
		msg[1].buf = &readData;

		ret = i2c_transfer(adap, msg, 2);
	if (ret < 0) {
		//new_client->ext_flag = 0;
        temp_sync--;
			//mutex_unlock(&fan5405_i2c_access);
		return 0;
	}

	*returnData = readData;

	//new_client->ext_flag = 0;

		//mutex_unlock(&fan5405_i2c_access);
	temp_sync--;
	return 1;
}

int fan5405_write_byte(u8 cmd, u8 writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
		struct i2c_msg msg;
		struct i2c_adapter *adap ;//= new_client->adapter;
		if(new_client == NULL)
			return -1;
		adap = new_client->adapter;
	//	printk("%s:reg[%x]=%x\n",__FUNCTION__,cmd,writeData);
	   if(temp_sync)
		   return 0;
		temp_sync++;
	//	mutex_lock(&fan5405_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;
		msg.addr = new_client->addr;
		msg.flags = 0;
		msg.len = 2;
		msg.buf = (char *)write_data;

		ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {

		//new_client->ext_flag = 0;
			//mutex_unlock(&fan5405_i2c_access);
		temp_sync--;
		return 0;
	}
     temp_sync--;
	//new_client->ext_flag = 0;
		//mutex_unlock(&fan5405_i2c_access);
	return 1;
}

u32 fan5405_read_interface(u8 RegNum, u8 *val, u8 MASK, u8 SHIFT)
{
	u8 fan5405_reg = 0;
	int ret = 0;

	ret = fan5405_read_byte(RegNum, &fan5405_reg);

	battery_log(BAT_LOG_FULL, "[fan5405_read_interface] Reg[%x]=0x%x\n", RegNum, fan5405_reg);

	fan5405_reg &= (MASK << SHIFT);
	*val = (fan5405_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[fan5405_read_interface] val=0x%x\n", *val);

	return ret;
}

u32 fan5405_config_interface(u8 RegNum, u8 val, u8 MASK, u8 SHIFT)
{
	u8 fan5405_reg = 0;
	int ret = 0;

	ret = fan5405_read_byte(RegNum, &fan5405_reg);
	battery_log(BAT_LOG_FULL, "[fan5405_config_interface] Reg[%x]=0x%x\n", RegNum, fan5405_reg);

	fan5405_reg &= ~(MASK << SHIFT);
	fan5405_reg |= (val << SHIFT);

	if (RegNum == fan5405_CON4 && val == 1 && MASK == CON4_RESET_MASK
	    && SHIFT == CON4_RESET_SHIFT) {
		/* RESET bit */
	} else if (RegNum == fan5405_CON4) {
		fan5405_reg &= ~0x80;	/* RESET bit read returs 1, so clear it */
	}

	ret = fan5405_write_byte(RegNum, fan5405_reg);
	battery_log(BAT_LOG_FULL, "[fan5405_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    fan5405_reg);

	return ret;
}

/* write one register directly */
u32 fan5405_reg_config_interface(u8 RegNum, u8 val)
{
	int ret = 0;

	ret = fan5405_write_byte(RegNum, val);

	return ret;
}

/* CON0 */

void fan5405_set_tmr_rst(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON0),
				       (u8) (val),
				       (u8) (CON0_TMR_RST_MASK), (u8) (CON0_TMR_RST_SHIFT)
	    );
}

u32 fan5405_get_otg_status(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON0),
				     (&val), (u8) (CON0_OTG_MASK), (u8) (CON0_OTG_SHIFT)
	    );
	return val;
}

void fan5405_set_en_stat(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON0),
				       (u8) (val),
				       (u8) (CON0_EN_STAT_MASK), (u8) (CON0_EN_STAT_SHIFT)
	    );
}

u32 fan5405_get_chip_status(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON0),
				     (&val), (u8) (CON0_STAT_MASK), (u8) (CON0_STAT_SHIFT)
	    );
	return val;
}

u32 fan5405_get_boost_status(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON0),
				     (&val), (u8) (CON0_BOOST_MASK), (u8) (CON0_BOOST_SHIFT)
	    );
	return val;
}

u32 fan5405_get_fault_status(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON0),
				     (&val), (u8) (CON0_FAULT_MASK), (u8) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void fan5405_set_input_charging_current(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON1),
				       (u8) (val),
				       (u8) (CON1_LIN_LIMIT_MASK), (u8) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void fan5405_set_v_low(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON1),
				       (u8) (val), (u8) (CON1_LOW_V_MASK), (u8) (CON1_LOW_V_SHIFT)
	    );
}

void fan5405_set_te(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON1),
				       (u8) (val), (u8) (CON1_TE_MASK), (u8) (CON1_TE_SHIFT)
	    );
}

void fan5405_set_ce(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON1),
				       (u8) (val), (u8) (CON1_CE_MASK), (u8) (CON1_CE_SHIFT)
	    );
}

void fan5405_set_hz_mode(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON1),
				       (u8) (val),
				       (u8) (CON1_HZ_MODE_MASK), (u8) (CON1_HZ_MODE_SHIFT)
	    );
}

void fan5405_set_opa_mode(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON1),
				       (u8) (val),
				       (u8) (CON1_OPA_MODE_MASK), (u8) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void fan5405_set_oreg(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON2),
				       (u8) (val), (u8) (CON2_OREG_MASK), (u8) (CON2_OREG_SHIFT)
	    );
}

void fan5405_set_otg_pl(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON2),
				       (u8) (val), (u8) (CON2_OTG_PL_MASK), (u8) (CON2_OTG_PL_SHIFT)
	    );
}

void fan5405_set_otg_en(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON2),
				       (u8) (val), (u8) (CON2_OTG_EN_MASK), (u8) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

u32 fan5405_get_vender_code(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON3),
				     (&val),
				     (u8) (CON3_VENDER_CODE_MASK), (u8) (CON3_VENDER_CODE_SHIFT)
	    );
	if(0x7==val || 0x2==val){
		ic_vendor=1;
	}else{
		ic_vendor=0;
	}
	return val;
}

u32 fan5405_get_pn(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON3),
				     (&val), (u8) (CON3_PIN_MASK), (u8) (CON3_PIN_SHIFT)
	    );
	return val;
}

u32 fan5405_get_revision(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON3),
				     (&val), (u8) (CON3_REVISION_MASK), (u8) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void fan5405_set_reset(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON4),
				       (u8) (val), (u8) (CON4_RESET_MASK), (u8) (CON4_RESET_SHIFT)
	    );
}

void fan5405_set_iocharge(u32 val)
{
	u32 ret = 0;

	if(1==ic_vendor){
		ret = fan5405_config_interface((u8) (fan5405_CON4),
				       (u8) (val), (u8) (0x7), (u8) (4)
	    );	
	}else{
		ret = fan5405_config_interface((u8) (fan5405_CON4),
					       (u8) (val), (u8) (CON4_I_CHR_MASK), (u8) (CON4_I_CHR_SHIFT)
		    );
	}
}

void fan5405_set_iterm(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON4),
				       (u8) (val), (u8) (CON4_I_TERM_MASK), (u8) (CON4_I_TERM_SHIFT)
	    );
}

/* CON5 */

void fan5405_set_dis_vreg(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON5),
				       (u8) (val),
				       (u8) (CON5_DIS_VREG_MASK), (u8) (CON5_DIS_VREG_SHIFT)
	    );
}

void fan5405_set_io_level(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON5),
				       (u8) (val),
				       (u8) (CON5_IO_LEVEL_MASK), (u8) (CON5_IO_LEVEL_SHIFT)
	    );
}

u32 fan5405_get_sp_status(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON5),
				     (&val), (u8) (CON5_SP_STATUS_MASK), (u8) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

u32 fan5405_get_en_level(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = fan5405_read_interface((u8) (fan5405_CON5),
				     (&val), (u8) (CON5_EN_LEVEL_MASK), (u8) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}

void fan5405_set_vsp(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON5),
				       (u8) (val), (u8) (CON5_VSP_MASK), (u8) (CON5_VSP_SHIFT)
	    );
}

/* CON6 */

void fan5405_set_i_safe(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON6),
				       (u8) (val), (u8) (CON6_ISAFE_MASK), (u8) (CON6_ISAFE_SHIFT)
	    );
}

void fan5405_set_v_safe(u32 val)
{
	u32 ret = 0;

	ret = fan5405_config_interface((u8) (fan5405_CON6),
				       (u8) (val), (u8) (CON6_VSAFE_MASK), (u8) (CON6_VSAFE_SHIFT)
	    );
}
void tbl_charger_otg_vbus(int mode)
{
  if(mode){
  	 if (!IS_ERR(pinctrl_otg_drvvbus_high))
		 pinctrl_select_state(fan_pinctrl, pinctrl_otg_drvvbus_high);	
	// fan5405_set_ce(1);
	 fan5405_set_opa_mode(1);
	 
  	}
  else{
  	  if (!IS_ERR(pinctrl_otg_drvvbus_low))
		 pinctrl_select_state(fan_pinctrl, pinctrl_otg_drvvbus_low);
	// fan5405_set_ce(1);
	 fan5405_set_opa_mode(0);
  	}
  
}
void charger_dis_set(int enable)
{
  if(enable){
  	 if (!IS_ERR(pinctrl_chr_dis_high))
		 pinctrl_select_state(fan_pinctrl, pinctrl_chr_dis_high);	 	
  	}
  else{
  	  if (!IS_ERR(pinctrl_chr_dis_low))
		 pinctrl_select_state(fan_pinctrl, pinctrl_chr_dis_low);
  	}
  
}

void fan5405_dump_register(void)
{
	int i = 0;

	for (i = 0; i < fan5405_REG_NUM; i++) {
		fan5405_read_byte(i, &fan5405_reg[i]);
		printk("LQ -->  %s [0x%x]=0x%x ",__func__, i, fan5405_reg[i]);
	}
	battery_log(BAT_LOG_CRTI, "\n");
}

static int fan5405_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
       int ret = 0;
	new_client = client;
	fan5405_dump_register();
	chargin_hw_init_done = KAL_TRUE;
	fan_pinctrl = devm_pinctrl_get(&client->dev);
		if (IS_ERR(fan_pinctrl)) {
			ret = PTR_ERR(fan_pinctrl);
			printk("Cannot find fan5405 pinctrl!\n");
			return ret;
		}
	pinctrl_otg_drvvbus_low = pinctrl_lookup_state(fan_pinctrl, "otg_off");
		if (IS_ERR(pinctrl_otg_drvvbus_low)) {
			ret = PTR_ERR(pinctrl_otg_drvvbus_low);
			printk("Cannot find fan5405 pinctrl pinctrl_otg_drvvbus_low!\n");
			return ret;
		}
        pinctrl_otg_drvvbus_high= pinctrl_lookup_state(fan_pinctrl, "otg_on");
		if (IS_ERR(pinctrl_otg_drvvbus_high)) {
			ret = PTR_ERR(pinctrl_otg_drvvbus_high);
			printk("Cannot find fan5405 pinctrl pinctrl_otg_drvvbus_high!\n");
			return ret;
		}
	pinctrl_chr_dis_low = pinctrl_lookup_state(fan_pinctrl, "chr_dis_low");
		if (IS_ERR(pinctrl_chr_dis_low)) {
			ret = PTR_ERR(pinctrl_chr_dis_low);
			printk("Cannot find fan5405 pinctrl pinctrl_chr_dis_low!\n");
			return ret;
		}
	pinctrl_chr_dis_high= pinctrl_lookup_state(fan_pinctrl, "chr_dis_high");
		if (IS_ERR(pinctrl_chr_dis_high)) {
			ret = PTR_ERR(pinctrl_chr_dis_high);
			printk("Cannot find fan5405 pinctrl pinctrl_chr_dis_high!\n");
			return ret;
		}	
	return 0;
}

u8 g_reg_value_fan5405 = 0;
static ssize_t show_fan5405_access(struct device *dev, struct device_attribute *attr, char *buf)
{

	battery_log(BAT_LOG_FULL, "[show_fan5405_access] 0x%x\n", g_reg_value_fan5405);
	return sprintf(buf, "%u\n", g_reg_value_fan5405);
}

static ssize_t store_fan5405_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

        battery_log(BAT_LOG_FULL, "[store_fan5405_access]\n");
		
	if (buf != NULL && size != 0) {

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_fan5405_access] write fan5405 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = fan5405_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = fan5405_read_interface(reg_address, &g_reg_value_fan5405, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_fan5405_access] read fan5405 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_fan5405);
			battery_log(BAT_LOG_CRTI,
			    "[store_fan5405_access] Please use \"cat fan5405_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(fan5405_access, 0664, show_fan5405_access, store_fan5405_access);	/* 664 */

static int fan5405_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** fan5405_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_fan5405_access);

	return 0;
}

struct platform_device fan5405_user_space_device = {
	.name = "fan5405-user",
	.id = -1,
};

static struct platform_driver fan5405_user_space_driver = {
	.probe = fan5405_user_space_probe,
	.driver = {
		   .name = "fan5405-user",
	},
};

static int __init fan5405_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&fan5405_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[fan5405_init] failed to register fan5405 i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[fan5405_init] Success to register fan5405 i2c driver.\n");
	}

	/* fan5405 user space access interface */
	ret = platform_device_register(&fan5405_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[fan5405_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&fan5405_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[fan5405_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

subsys_initcall(fan5405_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C fan5405 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
