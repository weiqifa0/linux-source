/*
 * AW9110B I2C I/O Expander
 *
 * Author: jimmy <lijiaming@knowin.com>
 *
 */

#ifndef _AW9110B_H
#define _AW9110B_H

#define AW_GROUPA  0
#define AW_GROUPB  1

#define AW_OUT0  0
#define AW_OUT1  1
#define AW_OUT2  2
#define AW_OUT3  3
#define AW_OUT4  4
#define AW_OUT5  5
#define AW_OUT6  6
#define AW_OUT7  7
#define AW_OUT8  8
#define AW_OUT9  9

#define AW_MODE_GPIO  1
#define AW_MODE_LED   0

#define AW_MAKE_FLAGS(_group, _bit, _mode, _defval)  (((_group) << 24) | \
                                                    ((_bit) << 16) | \
                                                    ((_mode) << 8)| \
                                                    (_defval))

#define AW_GET_GROUP(_flags)  ((unsigned char)((unsigned int)(_flags) >> 24))
#define AW_GET_OUTBIT(_flags)  ((unsigned char)((unsigned int)(_flags) >> 16))
#define AW_GET_MODE(_flags)  ((unsigned char)((unsigned int)(_flags) >> 8))
#define AW_GET_DEFVAL(_flags)  ((unsigned char)(_flags))

#define AW9110B_OPEN_DRAIN    0
#define AW9110B_TOTEM_POLE    1


#endif
