#ifndef _EM20918_H_
#define _EM20918_H_

#define EM20918_DRV_NAME          "ir_em"

#define EM20918_PID_REG           0x00
#define EM20918_CONFIG_REG        0x01
#define EM20918_INTERRUPT_REG     0x02
#define EM20918_PS_LT_REG         0x03
#define EM20918_PS_HT_REG         0x04
#define EM20918_PS_DATA_REG       0x08
#define EM30918_LIGHT_DATA_L      0X09
#define EM30918_LIGHT_DATA_H      0X0A
#define EM20918_RESET_REG         0x0E
#define EM20918_OFFSET_REG        0x0F

// pid register
#define EM20918_PRODUCT_ID        0x31

// config register
#define CONFIG_REG_PS_EN          (1 << 7)
#define CONFIG_REG_PS_SLP         (1 << 6)
#define CONFIG_REG_PS_DR_200MA    (7 << 3)
#define CONFIG_REG_PS_DR_100MA    (6 << 3)
#define CONFIG_REG_PS_DR_50MA     (5 << 3)
#define CONFIG_REG_PS_DR_25MA     (4 << 3)
#define CONFIG_REG_PS_DR_120MA    (3 << 3)
#define CONFIG_REG_PS_DR_60MA     (2 << 3)
#define CONFIG_REG_PS_DR_30MA     (1 << 3)
#define CONFIG_REG_PS_DR_15MA     (0 << 3)

// interrupt register
#define INTERRUPT_REG_PS_FLAG     (1 << 7)

// reset register
#define RESET_REG_RST             (0x9C)

// offset register
#define OFFSET_REG_RST            (0xE1)


struct em20918_reg_info {
    u8 addr;
    u8 data;
};

/* The platform data for the em20918 driver */
struct em20918_platform_data {
    int pwr_pin;
    int pwr_en_level;
    int int_pin;
    int init_prox;
};

#endif

