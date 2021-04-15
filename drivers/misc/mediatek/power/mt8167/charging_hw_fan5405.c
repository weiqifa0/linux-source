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
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <mt-plat/charging.h>
//#include <mt-plat/mt_boot.h>
//#include <mt-plat/mt_gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/upmu_common.h>

//#ifdef CONFIG_PSC5415A_CHARGER
#include <linux/of_gpio.h>
#include <mt-plat/battery_common.h>
#include <asm-generic/gpio.h>
#include <linux/gpio.h>
//#endif
#include <mt-plat/mtk_boot.h>
#include <mt-plat/battery_meter.h>
#include <mach/mtk_battery_meter.h>
#include <mach/mtk_charging.h>
#include <mach/mtk_pmic.h>

#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

static enum charger_type g_charger_type = CHARGER_UNKNOWN;

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0
int wireless_charger_gpio_number = (168 | 0x80000000);
#endif

/*int gpio_off_dir = GPIO_DIR_OUT;
int gpio_off_out = GPIO_OUT_ONE;
int gpio_on_dir = GPIO_DIR_OUT;
int gpio_on_out = GPIO_OUT_ZERO;*/

kal_bool charging_type_det_done = KAL_TRUE;

static int ic_vendor=0; /*default:HL7007*/

const u32 CS_VTH[]={CHARGE_CURRENT_550_00_MA,CHARGE_CURRENT_650_00_MA,CHARGE_CURRENT_750_00_MA,
	CHARGE_CURRENT_850_00_MA,CHARGE_CURRENT_1050_00_MA,CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1350_00_MA,CHARGE_CURRENT_1450_00_MA,CHARGE_CURRENT_1525_00_MA,
	CHARGE_CURRENT_1650_00_MA,CHARGE_CURRENT_1750_00_MA,CHARGE_CURRENT_1825_00_MA,
	CHARGE_CURRENT_2050_00_MA,CHARGE_CURRENT_2125_00_MA,CHARGE_CURRENT_2350_00_MA,
	CHARGE_CURRENT_2425_00_MA};

const u32 CS_VTH_PSC5415A[] = {
	CHARGE_CURRENT_425_00_MA,CHARGE_CURRENT_785_00_MA,CHARGE_CURRENT_946_00_MA,
	CHARGE_CURRENT_1160_00_MA,CHARGE_CURRENT_1339_00_MA,CHARGE_CURRENT_1464_00_MA
};
/*hl7005  rsence 68*/
/*
 u32 CS_VTH_HL7005[] = {
	CHARGE_CURRENT_550_00_MA,CHARGE_CURRENT_650_00_MA,CHARGE_CURRENT_750_00_MA,
	CHARGE_CURRENT_850_00_MA,CHARGE_CURRENT_1050_00_MA,CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1350_00_MA,CHARGE_CURRENT_1450_00_MA
};*/
/*hl7005  rsence 56*/

 u32 CS_VTH_HL7005[] = {
	 CHARGE_CURRENT_667_00_MA,CHARGE_CURRENT_789_00_MA,CHARGE_CURRENT_910_00_MA,
	 CHARGE_CURRENT_1032_00_MA,CHARGE_CURRENT_1275_00_MA,CHARGE_CURRENT_1396_00_MA,
	 CHARGE_CURRENT_1639_00_MA,CHARGE_CURRENT_1760_00_MA
 };

const u32 VBAT_CV_VTH[] = {
	BATTERY_VOLT_03_500000_V, BATTERY_VOLT_03_520000_V, BATTERY_VOLT_03_540000_V,
	    BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V, BATTERY_VOLT_03_600000_V, BATTERY_VOLT_03_620000_V,
	    BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V, BATTERY_VOLT_03_680000_V, BATTERY_VOLT_03_700000_V,
	    BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V, BATTERY_VOLT_03_760000_V, BATTERY_VOLT_03_780000_V,
	    BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V, BATTERY_VOLT_03_840000_V, BATTERY_VOLT_03_860000_V,
	    BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V, BATTERY_VOLT_03_920000_V, BATTERY_VOLT_03_940000_V,
	    BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V, BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_020000_V,
	    BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V, BATTERY_VOLT_04_080000_V, BATTERY_VOLT_04_100000_V,
	    BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V, BATTERY_VOLT_04_160000_V, BATTERY_VOLT_04_180000_V,
	    BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V, BATTERY_VOLT_04_240000_V, BATTERY_VOLT_04_260000_V,
	    BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_320000_V, BATTERY_VOLT_04_340000_V,
	    BATTERY_VOLT_04_360000_V,
	BATTERY_VOLT_04_380000_V, BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_420000_V,
	    BATTERY_VOLT_04_440000_V
};
#if 0
#if defined(CONFIG_PSC5415A_CHARGER)
const u32 CS_VTH[] = {
	CHARGE_CURRENT_350_00_MA,CHARGE_CURRENT_650_00_MA,CHARGE_CURRENT_800_00_MA,
	CHARGE_CURRENT_950_00_MA,CHARGE_CURRENT_1100_00_MA,CHARGE_CURRENT_1200_00_MA,
	CHARGE_CURRENT_1400_00_MA,CHARGE_CURRENT_1525_00_MA
};
#elif defined(CONFIG_HL7005_CHARGER)
const u32 CS_VTH[] = {
	CHARGE_CURRENT_550_00_MA,CHARGE_CURRENT_650_00_MA,CHARGE_CURRENT_750_00_MA,
	CHARGE_CURRENT_850_00_MA,CHARGE_CURRENT_1050_00_MA,CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1350_00_MA,CHARGE_CURRENT_1450_00_MA
};
#else
const u32 CS_VTH[] = {
	CHARGE_CURRENT_550_00_MA,CHARGE_CURRENT_650_00_MA,CHARGE_CURRENT_750_00_MA,
	CHARGE_CURRENT_850_00_MA,CHARGE_CURRENT_1050_00_MA,CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1350_00_MA,CHARGE_CURRENT_1450_00_MA,CHARGE_CURRENT_1525_00_MA,
	CHARGE_CURRENT_1650_00_MA,CHARGE_CURRENT_1750_00_MA,CHARGE_CURRENT_1825_00_MA,
	CHARGE_CURRENT_2050_00_MA,CHARGE_CURRENT_2125_00_MA,CHARGE_CURRENT_2350_00_MA,
	CHARGE_CURRENT_2425_00_MA
};
#endif
#endif
const u32 INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_800_00_MA,
	    CHARGE_CURRENT_MAX
};

/*const u32 VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};*/
const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_100000_V,	 BATTERY_VOLT_04_150000_V,
	BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,  BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V,
	BATTERY_VOLT_04_450000_V, BATTERY_VOLT_06_000000_V,  BATTERY_VOLT_06_500000_V,
	BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,  BATTERY_VOLT_09_500000_V,
	BATTERY_VOLT_10_500000_V
};

//#ifdef CONFIG_PSC5415A_CHARGER
static void chg_set_gpio_output( int enable)
{
	charger_dis_set(enable);
}
//#endif

u32 charging_value_to_parameter(const u32 *parameter, const u32 array_size, const u32 val)
{
	if (val < array_size)
		return parameter[val];
	battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
	return parameter[0];
}

u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
{
	u32 i;

	for (i = 0; i < array_size; i++)
		if (val == *(parameter + i))
			return i;
    dump_stack();
	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");

	return 0;
}


static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--)	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];

		battery_log(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++)	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];

		battery_log(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

static u32 charging_hw_init(void *data)
{
	u32 status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
#endif

	 upmu_set_rg_usbdl_set(0);       //force leave USBDL mode 
	 upmu_set_rg_usbdl_rst(1);             //force leave USBDL mode 
//	pmic_set_register_value(PMIC_RG_USBDL_SET, 0x0);
//	pmic_set_register_value(PMIC_RG_USBDL_RST, 0x1);
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)

if(1==ic_vendor){
  fan5405_reg_config_interface(0x06, 0x7f); // set ISAFE and HW CV point (4.4v)
}
else if (ic_vendor == 5145){
  fan5405_reg_config_interface(0x06, 0x38); // set ISAFE and HW CV point (4.4v)
}
else
	fan5405_reg_config_interface(0x06, 0x7a);	/* ISAFE = 1250mA, VSAFE = 4.34V */


#else
	fan5405_reg_config_interface(0x06, 0x70);
#endif

if(1==ic_vendor){
	fan5405_reg_config_interface(0x01, 0xf8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	fan5405_reg_config_interface(0x05, 0x04);
	if (!charging_init_flag) {
		fan5405_reg_config_interface(0x04, 0x20);	/* 647mA */
		charging_init_flag = KAL_TRUE;
	}	
}else if(5145 ==ic_vendor){
	fan5405_reg_config_interface(0x00, 0x40);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	fan5405_reg_config_interface(0x01, 0xC8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	fan5405_reg_config_interface(0x02, 0xA8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	fan5405_reg_config_interface(0x03, 0xFF);
	fan5405_reg_config_interface(0x05, 0x04);
	if (!charging_init_flag) {
			fan5405_reg_config_interface(0x04, 0x23);	/* 647mA */ //iterm set to 200ma (HL7005)
			charging_init_flag = KAL_TRUE;
		}
}
else{
	fan5405_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
	fan5405_reg_config_interface(0x01, 0xf8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	fan5405_reg_config_interface(0x05, 0x03);
	if (!charging_init_flag) {
		fan5405_reg_config_interface(0x04, 0x1A);	/* 146mA */
		charging_init_flag = KAL_TRUE;
	}
}
	return status;
}


static u32 charging_dump_register(void *data)
{
	u32 status = STATUS_OK;

	fan5405_dump_register();

	return status;
}


static u32 charging_enable(void *data)
{
	u32 status = STATUS_OK;
	u32 enable = *(u32 *) (data);
	if (KAL_TRUE == enable) {
		fan5405_set_ce(0);
		fan5405_set_hz_mode(0);
		fan5405_set_opa_mode(0);
		if(1==ic_vendor || 5145 == ic_vendor)
			chg_set_gpio_output( 0); //disable charger ic PSC5415A

	} else {

#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
#endif

		fan5405_set_ce(1);
		if(1==ic_vendor || 5145 == ic_vendor)
			chg_set_gpio_output( 1);
	}

	return status;
}


static u32 charging_set_cv_voltage(void *data)
{
	u32 status = STATUS_OK;
	u16 register_value;

	register_value =
	    charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), *(u32 *) (data));
	fan5405_set_oreg(register_value);

	return status;
}


static u32 charging_get_current(void *data)
{
	u32 status = STATUS_OK;
	u32 array_size;
	u8 reg_value;


	if(0 ==ic_vendor){
		array_size = GETARRAYNUM(CS_VTH_HL7005);
		fan5405_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
		*(u32 *) data = charging_value_to_parameter(CS_VTH_HL7005, array_size, reg_value);
	}else if(5145 ==ic_vendor){
		array_size = GETARRAYNUM(CS_VTH_PSC5415A);
		fan5405_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
		*(u32 *) data = charging_value_to_parameter(CS_VTH_PSC5415A, array_size, reg_value);
	}
	else{
		/* Get current level */
		array_size = GETARRAYNUM(CS_VTH_HL7005);
		fan5405_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
		*(u32 *) data = charging_value_to_parameter(CS_VTH_HL7005, array_size, reg_value);
	}

	return status;
}

u32 charging_set_sw_cv_current(int current_now)
{
      int i = 0, temp = 0;
      for (;i < 7 ; i++){
           if(current_now <= CS_VTH_HL7005[i]){
		    if(i > 0){
		          temp =    CS_VTH_HL7005[i -1] ;
		    }else{
                          temp = 0;
			}
	     break;
           	}
      	}

	return temp;
}

static u32 charging_set_current(void *data)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;
	u32 current_value = *(u32 *) data;
	if (current_value <= CHARGE_CURRENT_350_00_MA) {
		fan5405_set_io_level(1);
	} else {
		fan5405_set_io_level(0);
		if(0==ic_vendor){	
			array_size = GETARRAYNUM(CS_VTH_HL7005);
			set_chr_current = bmt_find_closest_level(CS_VTH_HL7005, array_size, current_value);
			register_value = charging_parameter_to_value(CS_VTH_HL7005, array_size, set_chr_current);
		}else  if(5145==ic_vendor){	
			array_size = GETARRAYNUM(CS_VTH_PSC5415A);
			set_chr_current = bmt_find_closest_level(CS_VTH_PSC5415A, array_size, current_value);
			register_value = charging_parameter_to_value(CS_VTH_PSC5415A, array_size, set_chr_current);
		}
		else{
			array_size = GETARRAYNUM(CS_VTH_HL7005);
			set_chr_current = bmt_find_closest_level(CS_VTH_HL7005, array_size, current_value);
			register_value = charging_parameter_to_value(CS_VTH_HL7005, array_size, set_chr_current);
		}
		printk("register_value=%d\n",register_value);
		fan5405_set_iocharge(register_value);
	}

	return status;
}


static u32 charging_set_input_current(void *data)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;

	if (*(u32 *) data > CHARGE_CURRENT_500_00_MA) {
		register_value = 0x3;
	} else {
		array_size = GETARRAYNUM(INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(u32 *) data);
		register_value =
		    charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);
	}

	fan5405_set_input_charging_current(register_value);

	return status;
}


static u32 charging_get_charging_status(void *data)
{
	u32 status = STATUS_OK;
	u32 ret_val;

	ret_val = fan5405_get_chip_status();

	if (ret_val == 0x2)
		*(u32 *) data = KAL_TRUE;
	else
		*(u32 *) data = KAL_FALSE;

	return status;
}


static u32 charging_reset_watch_dog_timer(void *data)
{
	u32 status = STATUS_OK;

	fan5405_set_tmr_rst(1);

	return status;
}


static u32 charging_set_hv_threshold(void *data)
{
	u32 status = STATUS_OK;

	u32 set_hv_voltage;
	u32 array_size;
	u16 register_value;
	u32 voltage = *(u32 *) (data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
		upmu_set_rg_vcdt_hv_vth(register_value);


	return status;
}


static u32 charging_get_hv_status(void *data)
{
	u32 status = STATUS_OK;

		*(kal_bool *)(data) = upmu_get_rgs_vcdt_hv_det();
	
	return status;
}


static u32 charging_get_battery_status(void *data)
{
	u32 status = STATUS_OK;

		upmu_set_baton_tdet_en(1);
		upmu_set_rg_baton_en(1);
		*(kal_bool *)(data) = upmu_get_rgs_baton_undet();

	return status;
}


static u32 charging_get_charger_det_status(void *data)
{
	u32 status = STATUS_OK;
#if defined(CHRDET_SW_MODE_EN)
		unsigned int vchr_val = 0;
	
		vchr_val = PMIC_IMM_GetOneChannelValue(VCHARGER_CHANNEL_NUMBER, 1, 1);
		vchr_val = (((330+39)*100*vchr_val)/39)/100;
	
		if (vchr_val > 4300) {
			battery_log(BAT_LOG_FULL, "[CHRDET_SW_WORKAROUND_EN] upmu_is_chr_det=Y (%d)\n", vchr_val);
			*(unsigned int *)data = KAL_TRUE;
		} else {
			battery_log(BAT_LOG_FULL, "[CHRDET_SW_WORKAROUND_EN] upmu_is_chr_det=N (%d)\n", vchr_val);
			*(unsigned int *)data = KAL_FALSE;
		}
#else

			 *(kal_bool *)(data) = upmu_get_rgs_chrdet();
#endif
		if (upmu_get_rgs_chrdet() == 0)
		g_charger_type = CHARGER_UNKNOWN;

	return status;
}


kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}

static u32 charging_get_charger_type(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(enum charger_type *) (data) = hw_charging_get_charger_type();
#endif

	return status;
}

static u32 charging_get_is_pcm_timer_trigger(void *data)
{
	u32 status = STATUS_OK;
/* M migration
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;
	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
*/
	*(kal_bool *)(data) = KAL_FALSE;
	return status;
}

static u32 charging_set_platform_reset(void *data)
{
	u32 status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");

	return status;
}

static u32 charging_get_platform_boot_mode(void *data)
{
	u32 status = STATUS_OK;

	*(u32 *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());

	return status;
}

static u32 charging_set_power_off(void *data)
{
	u32 status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	
	kernel_power_off();
	

	return status;
}

static u32 charging_get_power_source(void *data)
{
	u32 status = STATUS_UNSUPPORTED;

	return status;
}

static u32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static u32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

static u32 charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static u32 charging_get_vender_code(void *data)
{
	u32 status = STATUS_OK;
	u32 ret_val;
	
	ret_val = fan5405_get_vender_code();
	printk("LQ --> %s ,ret_val=0x%x \n",__func__,ret_val);

	if (ret_val == 0x7 || 0x2== ret_val){
		*(u32 *) data = KAL_TRUE;
		ic_vendor =1 ;
	}else{
		*(u32 *) data = KAL_FALSE;
		ic_vendor =0;
	}
	if(ret_val == 0x7)
		ic_vendor = 5145;
	printk("LQ --> %s ,ic_vendor=%d\n",__func__,ic_vendor);
	return status;

}

#if 0
static u32(*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
charging_hw_init, charging_dump_register, charging_enable, charging_set_cv_voltage,
	    charging_get_current, charging_set_current, charging_set_input_current,
	    charging_get_charging_status, charging_reset_watch_dog_timer,
	    charging_set_hv_threshold, charging_get_hv_status, charging_get_battery_status,
	    charging_get_charger_det_status, charging_get_charger_type,
	    charging_get_is_pcm_timer_trigger, charging_set_platform_reset,
	    charging_get_platform_boot_mode, charging_set_power_off,
	    charging_get_power_source, charging_get_csdac_full_flag,
	    charging_set_ta_current_pattern, charging_set_error_state};
#endif
	
static u32(*charging_func[CHARGING_CMD_NUMBER]) (void *data);

s32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	s32 status;
	static bool is_init;
	
	if (is_init == false) {
		charging_func[CHARGING_CMD_INIT] = charging_hw_init;
		charging_func[CHARGING_CMD_DUMP_REGISTER] = charging_dump_register;
		charging_func[CHARGING_CMD_ENABLE] = charging_enable;
		charging_func[CHARGING_CMD_SET_CV_VOLTAGE] = charging_set_cv_voltage;
		charging_func[CHARGING_CMD_GET_CURRENT] = charging_get_current;
		charging_func[CHARGING_CMD_SET_CURRENT] = charging_set_current;
		charging_func[CHARGING_CMD_SET_INPUT_CURRENT] = charging_set_input_current;
		charging_func[CHARGING_CMD_GET_CHARGING_STATUS] = charging_get_charging_status;
		charging_func[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = charging_reset_watch_dog_timer;
		charging_func[CHARGING_CMD_SET_HV_THRESHOLD] = charging_set_hv_threshold;
		charging_func[CHARGING_CMD_GET_HV_STATUS] = charging_get_hv_status;
		charging_func[CHARGING_CMD_GET_BATTERY_STATUS] = charging_get_battery_status;
		charging_func[CHARGING_CMD_GET_CHARGER_DET_STATUS] =charging_get_charger_det_status;
		charging_func[CHARGING_CMD_GET_CHARGER_TYPE] = charging_get_charger_type;
		charging_func[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] =charging_get_is_pcm_timer_trigger;
		charging_func[CHARGING_CMD_SET_PLATFORM_RESET] = charging_set_platform_reset;
		charging_func[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] =charging_get_platform_boot_mode;
		charging_func[CHARGING_CMD_SET_POWER_OFF] = charging_set_power_off;
		charging_func[CHARGING_CMD_GET_POWER_SOURCE] = charging_get_power_source;
		charging_func[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = charging_get_csdac_full_flag;
		charging_func[CHARGING_CMD_SET_TA_CURRENT_PATTERN] = charging_set_ta_current_pattern;
		charging_func[CHARGING_CMD_SET_ERROR_STATE] = charging_set_error_state;
		charging_func[CHARGING_CMD_GET_VENDOR_CODE] = charging_get_vender_code;

		is_init = true;
	}

	if (cmd < CHARGING_CMD_NUMBER && charging_func[cmd])
		status = charging_func[cmd] (data);
	else {
		pr_err("Unsupported charging command:%d!\n", cmd);
		return STATUS_UNSUPPORTED;
	}

#if 0
	if (cmd < CHARGING_CMD_NUMBER) {
		if (charging_func[cmd] != NULL)
			status = charging_func[cmd](data);
		else {
			battery_log(BAT_LOG_CRTI, "[chr_control_interface]cmd:%d not supported\n", cmd);
			status = STATUS_UNSUPPORTED;
		}
	} else
		status = STATUS_UNSUPPORTED;
#endif
	return status;
}
