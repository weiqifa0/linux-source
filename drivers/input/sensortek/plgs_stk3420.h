#ifndef _PLGS_STK3420_H_
#define _PLGS_STK3420_H_

#define STK3420_DRV_NAME				"sensortek,stk3420"

#define STK3420_STATE_REG				(0x00)
#define STK3420_PSGSCTRL1_REG			(0x01)
#define STK3420_ALSCTRL1_REG			(0x02)
#define STK3420_LEDCTRL_REG				(0x03)
#define STK3420_INT_REG					(0x04)
#define STK3420_WAIT1_PSGS_REG			(0x05)
#define STK3420_THDH1_PS_REG			(0x06)
#define STK3420_THDH2_PS_REG			(0x07)
#define STK3420_THDL1_PS_REG			(0x08)
#define STK3420_THDL2_PS_REG			(0x09)
#define STK3420_THDH1_ALS_REG			(0x0A)
#define STK3420_THDH2_ALS_REG			(0x0B)
#define STK3420_THDL1_ALS_REG			(0x0C)
#define STK3420_THDL2_ALS_REG			(0x0D)
#define STK3420_FLAG_REG				(0x10)
#define STK3420_DATA1_PS_REG			(0x11)
#define STK3420_DATA2_PS_REG			(0x12)
#define STK3420_DATA1_ALS_REG			(0x13)
#define STK3420_DATA2_ALS_REG			(0x14)
#define STK3420_DATA1_IRS_REG			(0x17)
#define STK3420_DATA2_IRS_REG			(0x18)
#define STK3420_ALSCTRL2_REG			(0x19)
#define STK3420_WAIT_ALS_REG			(0x1B)
#define STK3420_WAIT2_PS_REG			(0x1C)
#define STK3420_PSGSCTRL2_REG			(0x1D)
#define STK3420_GSFLAG_REG				(0x1E)
#define STK3420_GSFIFOCTRL_REG			(0x1F)
#define STK3420_DATA1_GSE_REG			(0x20)
#define STK3420_DATA2_GSE_REG			(0x21)
#define STK3420_DATA1_GSW_REG			(0x22)
#define STK3420_DATA2_GSW_REG			(0x23)
#define STK3420_DATA1_GSN_REG			(0x24)
#define STK3420_DATA2_GSN_REG			(0x25)
#define STK3420_DATA1_GSS_REG			(0x26)
#define STK3420_DATA2_GSS_REG			(0x27)
#define STK3420_PDT_ID_REG				(0x3E)
#define STK3420_RESERVED_REG			(0x3F)
#define STK3420_SOFT_RESET_REG			(0x80)


// State Register
#define STATE_EN_IRS					(1 << 7)
#define STATE_EN_BGIR                   (1 << 6) //???
#define STATE_EN_ALS_RO					(1 << 5)
#define STATE_EN_PS_RO					(1 << 4)
#define STATE_EN_WAIT_ALS				(1 << 3)
#define STATE_EN_WAIT_PSGS				(1 << 2)
#define STATE_EN_ALS					(1 << 1)
#define STATE_EN_PS						(1 << 0)

// PS/GSCTRL1 Register
#define IT_PSGS_97US                    (0x00)
#define IT_PSGS_195US                   (0x01)
#define IT_PSGS_390US                   (0x02)
#define IT_PSGS_780US                   (0x03)
#define IT_PSGS_1P56MS                  (0x04)
#define IT_PSGS_3P12MS                  (0x05)
#define IT_PSGS_6P25MS                  (0x06)

#define PRST_PS_1TIMES                  (0 << 6)
#define PRST_PS_2TIMES                  (1 << 6)
#define PRST_PS_4TIMES                  (2 << 6)
#define PRST_PS_8TIMES                  (3 << 6)

// INTERRUPT Register
#define INTERRUPT_INT_CTRL              (1 << 7)
#define INTERRUPT_EN_GS_INT             (1 << 4)
#define INTERRUPT_EN_ALS_INT            (1 << 3)
#define INTERRUPT_PS_MODE               (1 << 1)
#define INTERRUPT_EN_PS_INT             (1 << 0)

// FLAG Register
#define FLAG_FLG_ALS_DR                 (1 << 7)
#define FLAG_FLG_PS_DR                  (1 << 6)
#define FLAG_FLG_ALS_INT                (1 << 5)
#define FLAG_FLG_PS_INT                 (1 << 4)
#define FLAG_FLG_GS_INT                 (1 << 3)
#define FLAG_FLG_IRS_DR                 (1 << 1)
#define FLAG_FLG_NF                     (1 << 0)

// GSFLAG Register
#define GSFLAG_GS_FIFO_LEN              (0x1F)
#define GSFLAG_FLG_GS_FIFO_OV           (1 << 7)

// PSGS_CTRL2 Register
#define PSGS2_EN_GSEW                   (1 << 0)
#define PSGS2_EN_GSNS                   (1 << 1)
#define PSGS2_EN_INTELLIGENT_PRST       (1 << 6)


#define STK3420_THDH_PS_VALUE			0x008c
#define STK3420_THDL_PS_VALUE			0x0003
#define STK3420_PS_MAX_VALUE			9000


struct stk3420_reg_info {
    u8 addr;
    u8 data;
};

#endif
