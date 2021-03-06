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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, args...)       do {} while (0)
#define PK_DBG_FUNC(fmt, args...)       pr_err(PFX  fmt, ##args)

#undef DEBUG_CAMERA_HW_K
//#define DEBUG_CAMERA_HW_K 
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, args...)		pr_err(fmt, ##args)
#define PK_INFO(fmt, args...)		pr_debug(PFX  fmt, ##args)
#define PK_XLOG_INFO(fmt, args...)      pr_debug(PFX  fmt, ##args)
#else
#define PK_DBG(a, ...)
#define PK_ERR(fmt, args...)
#define PK_INFO(fmt, args...)
#define PK_XLOG_INFO(fmt, args...)
#endif

#define BYPASS_GPIO		0

#ifndef DEMO_BOARD_SUPPORT
#define DEMO_BOARD_SUPPORT	1
#endif

/* GPIO Pin control*/
struct platform_device *cam_plt_dev;
struct pinctrl *camctrl;
struct pinctrl_state *cam0_pnd_h;
struct pinctrl_state *cam0_pnd_l;
struct pinctrl_state *cam0_rst_h;
struct pinctrl_state *cam0_rst_l;
struct pinctrl_state *cam1_pnd_h;
struct pinctrl_state *cam1_pnd_l;
struct pinctrl_state *cam1_rst_h;
struct pinctrl_state *cam1_rst_l;
struct pinctrl_state *cam_ldo0_h;
struct pinctrl_state *cam_ldo0_l;

/* for not demo board, eg: EVB*/
#if (DEMO_BOARD_SUPPORT == 0)

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	PK_DBG("---- EVB board mtkcam_gpio_init ----\n");

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/*externel LDO enable */
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	PK_DBG("---- EVB board mtkcam_gpio_set----\n");

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
	case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}
#else
/* for demo board */
int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
	/* struct device        *dev = &pdev->dev; */

	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	PK_DBG("---- Demo board mtkcam_gpio_set----\n");

	/* if (!check_pdn_rst_pin()) */
	PK_INFO("mtkcam_gpio_set: pinIdx:%d, pwrType:%d, Val:%d\n", PinIdx, PwrType, Val);

#if BYPASS_GPIO
	return 0;
#endif

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				gpio_direction_output(GPIO_CAM0_RST, 0);
			else
				gpio_direction_output(GPIO_CAM0_RST, 1);	/* if CAM1_RST, FFC can preview ok. */
		} else {
			if (Val == 0)
				gpio_direction_output(GPIO_CAM1_RST, 0);
			else
				gpio_direction_output(GPIO_CAM1_RST, 1);
		}

		break;
	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				gpio_direction_output(GPIO_CAM0_PDN, 0);
			else
				gpio_direction_output(GPIO_CAM0_PDN, 1);
		} else {
			if (Val == 0)
				gpio_direction_output(GPIO_CAM1_PDN, 0);
			else
				gpio_direction_output(GPIO_CAM1_PDN, 1);
		}

		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

#endif


int cntVCAMD;
int cntVCAMA;
int cntVCAMIO;
int cntVCAMAF;
int cntVCAMD_SUB;

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{
	if (_hwPowerOn(PinIdx, powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMAF)
			cntVCAMAF += 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(PinIdx, powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(int PinIdx, char *mode_name)
{

	int i = 0;

	PK_DBG
	    ("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
	     cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(PinIdx, VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(PinIdx, VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(PinIdx, VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(PinIdx, VCAMAF);
	for (i = 0; i < cntVCAMD_SUB; i++)
		_hwPowerDown(PinIdx, SUB_VCAMD);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;

}

#define SUB_USE_MAIN		0

#if SUB_USE_MAIN
#define MAIN_PINSET		1
#define SUB_PINSET		0

#else
#define MAIN_PINSET		0
#define SUB_PINSET		1

#endif

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
		       char *mode_name)
{

	u32 pinSetIdx = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

	u32 pinSet[3][8] = {

		{CAMERA_CMRST_PIN,
		 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 CAMERA_CMPDN_PIN,
		 CAMERA_CMPDN_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{CAMERA_CMRST1_PIN,
		 CAMERA_CMRST1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 CAMERA_CMPDN1_PIN,
		 CAMERA_CMPDN1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 }
	};

	if (SensorIdx == DUAL_CAMERA_MAIN_SENSOR)
		pinSetIdx = 0;
	else if (SensorIdx == DUAL_CAMERA_SUB_SENSOR)
		pinSetIdx = 1;
	else if (SensorIdx == DUAL_CAMERA_MAIN_2_SENSOR)
		pinSetIdx = 2;

	PK_DBG("----sesnor name: %s, pinSetIdx = %d, ON = %d\n", currSensorName, pinSetIdx, On);
	if (On) {
		ISP_MCLK1_EN(1);

		
    if (pinSetIdx == 1 && currSensorName
		&& (strcmp(currSensorName, SENSOR_DRVNAME_GC5025_MIPI_RAW) == 0)) {	/* demo board RFC */
			if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
				mtkcam_gpio_set(pinSetIdx, CAMPDN,
						pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);

			if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			/* VCAM_IO */
			if (_hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d\n",
				     VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

			if (_hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1800, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n",
				     VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(15);

			/* VCAM_A */
			if (_hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n",
				     VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

		
		
			mdelay(10);

			if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID) {
				mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);



			}
			mdelay(10);
			/* enable active sensor */
			if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID) {
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
			}

		}  		
		else {
			/* VCAM_IO */
			if (_hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n",
				     VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			
		}
	}

	else {			/* power OFF */
		ISP_MCLK1_EN(0);
		 if (pinSetIdx == 1 && currSensorName
		 && (strcmp(currSensorName, SENSOR_DRVNAME_GC5025_MIPI_RAW) == 0)) {	/* demo board RFC */
			/* Set Power Pin low and Reset Pin Low */
			if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
				mtkcam_gpio_set(pinSetIdx, CAMPDN,
						pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
			/* Set Reset Pin Low */
			if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
				mtkcam_gpio_set(pinSetIdx, CAMRST,
						pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			if (_hwPowerDownCnt(pinSetIdx, VCAMA, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id = %d\n",
				     VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

			if (_hwPowerDownCnt(pinSetIdx, VCAMD, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAMD), power id = %d\n",
				     VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			
					

			/* VCAM_IO */
			if (_hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
				     VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			 
		}
		else {
			/* VCAM_IO */
			if (_hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name) != TRUE) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
				     VCAMIO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
		}
	}

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;

}
EXPORT_SYMBOL(kdCISModulePowerOn);
