
#ifndef _GS_ALGORITHM_H_
#define _GS_ALGORITHM_H_

enum {
	GESTURE_EVENT_NA = 0x00,
	GESTURE_EVENT_NF = 0x04,

	GESTURE_EVENT_UP = 0x01, /* backword compatibility */
	GESTURE_EVENT_DOWN = 0x02,
	GESTURE_EVENT_LEFT = 0x08,
	GESTURE_EVENT_RIGHT = 0x10,
	GESTURE_EVENT_DIR_MASK = 0x1E,

	GESTURE_EVENT_UP_THEN_DOWN      = 0x20,
	GESTURE_EVENT_DOWN_THEN_UP      = 0x40,
	GESTURE_EVENT_LEFT_THEN_RIGHT   = 0x60,
	GESTURE_EVENT_RIGHT_THEN_LEFT   = 0x80,
	GESTURE_EVENT_COMBO_MASK        = 0xE0,
};

#if 0
#define GESTURE_INTP_LIMIT 		32
#define GESTURE_BUF_SZ_LIMIT		(36*(GESTURE_INTP_LIMIT+1))
#define GESTURE_PHASE_MAX_LIMIT	((8*(GESTURE_INTP_LIMIT+1))*2+1)
#endif

int stk3420_gesture_determine(u16 pd_val[], u32 *gesture);
int stk3420_gesture_func_init(void);
void stk3420_gesture_get_MaxPhase(int *phase);
void stk3420_gesture_get_MaxCorr(int *correl);


#endif /* _GS_ALGORITHM_H_ */

