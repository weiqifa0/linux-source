
#ifndef __STK_CUST_ALSPS_H__
#define __STK_CUST_ALSPS_H__

#define C_CUST_ALS_LEVEL    16
#define C_CUST_I2C_ADDR_NUM 4

struct stk_alsps_hw {
    int i2c_num;                                    /*!< the i2c bus used by ALS/PS */
    int power_id;                                   /*!< the power id of the chip */
    int power_vol;                                  /*!< the power voltage of the chip */
	//int polling_mode;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_ps;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_als;                               /*!< 1: polling mode ; 0:interrupt mode*/
    unsigned int    als_level[C_CUST_ALS_LEVEL-1];  /*!< (C_CUST_ALS_LEVEL-1) levels divides all range into C_CUST_ALS_LEVEL levels*/
    unsigned int    als_value[C_CUST_ALS_LEVEL];    /*!< the value reported in each level */
    //unsigned int    ps_threshold;                   /*!< the threshold of proximity sensor */	
	unsigned int	state_val;
	unsigned int 	psgsctrl1_reg;
	unsigned int 	alsctrl1_reg;
	unsigned int 	wait1_psgs_reg_ps;	
	unsigned int 	wait1_psgs_reg_gs;	
	unsigned int 	ledctrl_reg;
	unsigned int 	alsctrl2_reg;
	unsigned int 	wait_als_reg;
	unsigned int 	wait2_ps_reg;
	unsigned int 	psgsctrl2_reg;
	unsigned int 	fifoctrl_reg;
    unsigned int    ps_high_thd_val;
    unsigned int    ps_low_thd_val;
	unsigned int    als_window_loss;                /*!< the window loss  */	
    bool is_batch_supported_ps;
    bool is_batch_supported_als;	
};

struct stk_alsps_hw *stk_get_cust_alsps_hw(void);

#endif 
