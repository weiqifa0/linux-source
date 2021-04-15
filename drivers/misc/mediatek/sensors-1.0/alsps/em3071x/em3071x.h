/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for em3071 als/ps sensor chip.
 */
#ifndef __EM3071X_H__
#define __EM3071X_H__

#include <linux/ioctl.h>
// #include <linux/hct_include/hct_board_config.h>

#define EM3071X_CMM_PID 		0X00
#define EM3071X_CMM_ENABLE 		0X01
#define EM3071X_CMM_STATUS		0x02
#define EM3071X_CMM_INT_PS_LT   0X03
#define EM3071X_CMM_INT_PS_HT   0X04
#define EM3071X_CMM_PDATA_L 	0X08
#define EM3071X_CMM_C0DATA_L 	0X09
#define EM3071X_CMM_C0DATA_H 	0X0A

#define EM3071X_SUCCESS						0
#define EM3071X_ERR_I2C						-1



#define EM3071X_CMM_ID			0x00

#define EM3071X_CMM_INT_PS_LB		0x03
#define EM3071X_CMM_INT_PS_HB		0x04
#define EM3071X_CMM_PDATA		0x08
#define EM3071X_CMM_RESET		0x0E
#define EM3071X_CMM_OFFSET		0x0F

#define PS_DR_200MA					0x7
#define PS_DR_100MA					0x6
#define PS_DR_50MA					0x5
#define PS_DR_25MA					0x4
#define PS_DR_120MA					0x3
#define PS_DR_60MA					0x2
#define PS_DR_30MA					0x1
#define PS_DR_15MA					0x0
#define PS_DR_SHIFT					3

#define PS_ENABLE_MASK			0xb8
#define PS_DR_VALUE					(0x80|(PS_DR_200MA<<PS_DR_SHIFT))
#define PS_DR_VALUE_INIT					(0x80|(PS_DR_50MA<<PS_DR_SHIFT))
#define ALS_ENABLE_MASK			0x06


#if defined(__HCT_EM3071X_PS_DYNAMIC_THRELD_LOW__)
#define CUST_EM3071X_PS_THRES_FAR __HCT_EM3071X_PS_DYNAMIC_THRELD_LOW__
#else
#define CUST_EM3071X_PS_THRES_FAR	15
#endif


#if defined(__HCT_EM3071X_PS_DYNAMIC_THRELD_HIGH__)
#define CUST_EM3071X_PS_THRES_CLOSE	__HCT_EM3071X_PS_DYNAMIC_THRELD_HIGH__
#else
#define CUST_EM3071X_PS_THRES_CLOSE	20
#endif


#endif


