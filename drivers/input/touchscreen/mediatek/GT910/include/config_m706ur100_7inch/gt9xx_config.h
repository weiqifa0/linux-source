#ifndef _GT9XX_CONFIG_H_
#define _GT9XX_CONFIG_H_

/* ***************************PART2:TODO define********************************** */
/* STEP_1(REQUIRED):Change config table. */
/*TODO: puts the config info corresponded to your TP here, the following is just
a sample config, send this config should cause the chip cannot work normally*/
#if 0
#define CTP_CFG_GROUP0 {\
0x48,0x58,0x02,0x00,0x04,0x05,0x35,0x10,0x01,0x0A,\
0x19,0x0F,0x50,0x3C,0x03,0x05,0x00,0x00,0x00,0x00,\
0x00,0x00,0x0C,0x17,0x19,0x1E,0x14,0x88,0x07,0x0A,\
0x37,0x39,0x0C,0x08,0x00,0x00,0x00,0x02,0x02,0x11,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x1E,0x50,0x94,0xC5,0x85,0x08,0x00,0x00,0x04,\
0xCC,0x21,0x00,0xAA,0x28,0x00,0x8E,0x31,0x00,0x78,\
0x3B,0x00,0x65,0x48,0x00,0x65,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,\
0x00,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,\
0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x02,0x04,0x05,0x06,0x08,0x0A,0x0C,\
0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0xFF,0xFF,0xFF,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x67,0x01\
}
#else
//800*1280
#define CTP_CFG_GROUP0 {\
0x45,0x58,0x02,0x00,0x04,0x0A,0x05,0x00,0x01,0x08,0x28,0x05,0x5A,0x3C,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x16,0x19,0x14,0x89,0x29,0x0A,0x26,0x28,0x0C,0x08,0x00,0x00,0x00,0xBA,0x32,0x1D,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0x41,0x94,0xC5,0x02,0x00,0x00,0x00,0x04,0xF1,0x19,0x00,0xC8,0x1F,0x00,0xA8,0x26,0x00,0xA8,0x2F,0x00,0x79,0x3A,0x00,0x79,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x09,0x33,0x00,0x03,0x08,0x05,0x0F,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A,0x29,0x28,0x24,0x22,0x20,0x1F,0x1E,0x1D,0x0E,0x0C,0x0A,0x08,0x06,0x05,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB3,0x01\
}
#endif
/* TODO puts your group2 config info here,if need. */
#define CTP_CFG_GROUP1 {\
}

/* TODO puts your group2 config info here,if need. */
#define GTP_CFG_GROUP1_CHARGER {\
}

/* TODO puts your group3 config info here,if need. */
#define CTP_CFG_GROUP2 {\
}

/* TODO puts your group3 config info here,if need. */
#define GTP_CFG_GROUP2_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 3 here, if needed */
#define CTP_CFG_GROUP3 {\
}

/* TODO puts your group3 config info here,if need. */
#define GTP_CFG_GROUP3_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 4 here, if needed */
#define CTP_CFG_GROUP4 {\
}

/* TODO puts your group4 config info here,if need. */
#define GTP_CFG_GROUP4_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 5 here, if needed */
#define CTP_CFG_GROUP5 {\
}

/* TODO puts your group5 config info here,if need. */
#define GTP_CFG_GROUP5_CHARGER {\
}


#endif /* _GT1X_CONFIG_H_ */