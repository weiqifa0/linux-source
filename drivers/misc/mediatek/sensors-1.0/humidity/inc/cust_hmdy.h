/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#ifndef __CUST_HMDY_H__
#define __CUST_HMDY_H__

#include <linux/types.h>

#define B_CUST_I2C_ADDR_NUM 2

struct hmdy_hw {
	int i2c_num;		/*!< the i2c bus used by the chip */
	int direction;		/*!< the direction of the chip */
	int power_id;		/*!< the LDO ID of the chip, MT6516_POWER_NONE means the power is always on */
	int power_vol;		/*!< the Power Voltage used by the chip */
	int firlen;		/*!< the length of low pass filter */
	int (*power)(struct hmdy_hw *hw, unsigned int on, char *devname);
	/*!< i2c address list,for chips which has different addresses with different HW layout */
	unsigned char i2c_addr[B_CUST_I2C_ADDR_NUM];
	bool is_batch_supported_hmdy;
	bool is_batch_supported_temp;
};

int get_hmdy_dts_func(const char *name, struct hmdy_hw *hw);

#endif
