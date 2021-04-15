#include <linux/types.h>
#include "stk_cust_alsps.h"


static struct stk_alsps_hw cust_alsps_hw = {
    /* i2c bus number, for mt657x, default=0. For mt6589, default=3 */
#ifdef MT6589
    .i2c_num    = 3,
#elif defined(MT6572)
    .i2c_num    = 1,
#else
    .i2c_num    = 0,
#endif
    //.polling_mode = 1,
    .polling_mode_ps = 1,
    .polling_mode_als = 1,
    //.power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    //.power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .als_level  = {5,  9, 36, 59, 82, 132, 205, 273, 500, 845, 1136, 1545, 2364, 4655, 6982},	/* als_code */
    .als_value  = {10, 10, 40, 65, 90, 145, 225, 300, 550, 930, 1250, 1700, 2600, 5120, 7680, 10240},    /* lux */
    .state_val = 0x00,		/* disable all */
    .psgsctrl1_reg = 0x02,	// PRST_PS = 0, IT_PSGS = 390us,
    .alsctrl1_reg = 0x33,	// PRST_ALS = 1, GAIN_ALS = 8x, IT_ALS = 100ms
    .wait1_psgs_reg_ps = 0x3F, 	// 50ms
    .wait1_psgs_reg_gs = 0x06,	// 5.46ms
    .ledctrl_reg = 0x40,	// 100mA
    .alsctrl2_reg = 0x74,
    .wait_als_reg = 0x00,
    .wait2_ps_reg = 0x03,
    .psgsctrl2_reg = 0x00,
    .fifoctrl_reg = 0x08,
    .ps_high_thd_val = 5000,
    .ps_low_thd_val = 4000,
    .is_batch_supported_ps = false,
    .is_batch_supported_als = false,
};

struct stk_alsps_hw *stk_get_cust_alsps_hw(void)
{
    return &cust_alsps_hw;
}
