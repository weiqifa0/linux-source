/*
 * linux/drivers/misc/gs_algorithm.c
 *
 * gesture sensor algorithm for stk3420
 *
 * Copyright 2019, Howrd <howrd@21cn.com>
 *
 */

#include "linux/types.h"
#include <linux/module.h>
#include "gs_algorithm.h"


#define SUN_ALS_LUX_MED_THD		10000	//3000
#define SUN_ALS_LUX_HIGH_THD	200000

#ifndef abs
#define abs(a)	((a) >= 0 ? (a) : -(a))
#endif

enum {
    OPT_STK6020,
    // OPT_STK6015,
    // OPT_STK6015_PHONE,
    // OPT_STK6018,
    OPT_NUM,
};


// #define GESTURE_QMAX            ((1<<GESTURE_QBIT) - 1)
// #define GESTURE_PHASE_ZERO      GESTURE_PHASE_RANGE
// #define GESTURE_PHASE_MAX       (GESTURE_PHASE_RANGE*2+1)

/*-------------------*/

typedef struct tagGesture_opt {
    int CTTRACK_RATIO_RISE;
    int CTTRACK_RATIO_FALL;
    int CTTRACK_RATIO_STEP;
    int CTTRACK_STABLE_TH;
    int CTTRACK_STABLE_CNT;
    int CTTRACK_UNSTABLE_TH;
    int CTTRACK_UNSTABLE_CNT;
    int CTTRACK_OFFSET_WAIT_CNT;
    //#define GESTURE_COMBO_CNT;
    int GESTURE_COMBO_CNT;

    int GESTURE_AXIS2_EN;
    int GESTURE_AXIS1_EN;
    //int GESTURE_USE_CENTER;

    int GESTURE_INTP;
    //int GESTURE_INTP_LARGR;

    int GESTURE_CH1_RATIO_M;
    int GESTURE_CH1_RATIO_N;

    int GESTURE_CH0_TH;
    int GESTURE_CH1_TH;
    int GESTURE_CH2_TH;
    int GESTURE_CH3_TH;
    int GESTURE_CORR_TH;

    //int GESTURE_UP_TH;
    //int GESTURE_DOWN_TH;
    //int GESTURE_LEFT_TH;
    //int GESTURE_RIGHT_TH;
    //int GESTURE_UP_TH2;
    //int GESTURE_DOWN_TH2;
    //int GESTURE_LEFT_TH2;
    //int GESTURE_RIGHT_TH2;

    int GESTURE_BUF_SZ;

    int GESTURE_PHASE_RANGE;

    int GESTURE_QBIT;
    int GESTURE_QSTEP;

    int GESTURE_SIGNAL_CNT;
    int GESTURE_SIGNAL_NF_CNT;
    int GESTURE_NF_MAX_CNT;
    int GESTURE_NF_MIN_CNT;
    int GESTURE_END_CNT;

    int GESTURE_QMAX;
    int GESTURE_PHASE_ZERO;
    int GESTURE_PHASE_MAX;
} tfGesture_opt;

const tfGesture_opt xtGesture_opt[OPT_NUM] = {
    //OPT_STK6020
    {
        /*CTTRACK_RATIO_RISE     */8,
        /*CTTRACK_RATIO_FALL     */ 2,
        /*CTTRACK_RATIO_STEP     */ 32,
        /*CTTRACK_STABLE_TH      */15, //gesture threshold, CTTRACK_UNSTABLE_TH > CTTRACK_STABLE_TH > GESTURE_CH0~3_TH
        /*CTTRACK_STABLE_CNT     */10,
        /*CTTRACK_UNSTABLE_TH    */35, //gesture threshold
        /*CTTRACK_UNSTABLE_CNT   */90,
        /*CTTRACK_OFFSET_WAIT_CNT*/ 0,
        ///*GESTURE_COMBO_CNT       28,
        /*GESTURE_COMBO_CNT      */     0,

        /*GESTURE_AXIS2_EN       */1,
        /*GESTURE_AXIS1_EN       */1,
        //	/*GESTURE_USE_CENTER     */  0,

        /*GESTURE_INTP           */  32,
        //	/*GESTURE_INTP_LARGR     */  0,

        /*GESTURE_CH1_RATIO_M    */1,
        /*GESTURE_CH1_RATIO_N    */1,

        /*GESTURE_CH0_TH         */     10, //gesture threshold
        /*GESTURE_CH1_TH         */     10, //gesture threshold
        /*GESTURE_CH2_TH         */     10, //gesture threshold
        /*GESTURE_CH3_TH         */     10, //gesture threshold
        /*GESTURE_CORR_TH        */    400,

        // /*GESTURE_UP_TH          */    ((32+1)*0+10),	// GESTURE_INTP */ 32
        // /*GESTURE_DOWN_TH        */    ((32+1)*0+10),
        // /*GESTURE_LEFT_TH        */    ((32+1)*0+10),
        // /*GESTURE_RIGHT_TH       */    ((32+1)*0+10),

        // /*GESTURE_UP_TH2         */    ((32+1)*0+1),
        // /*GESTURE_DOWN_TH2       */    ((32+1)*0+1),
        // /*GESTURE_LEFT_TH2       */    ((32+1)*0+1),
        // /*GESTURE_RIGHT_TH2      */    ((32+1)*0+1),

        /*GESTURE_BUF_SZ         */    (80 * (32 + 1)),

        /*GESTURE_PHASE_RANGE    */    (10 * (32 + 1)),

        /*GESTURE_QBIT           */    16,
        /*GESTURE_QSTEP          */     4,

        /*GESTURE_SIGNAL_CNT     */   1,
        /*GESTURE_SIGNAL_NF_CNT  */ 8,
        /*GESTURE_NF_MAX_CNT     */   80,
        /*GESTURE_NF_MIN_CNT     */    1,
        /* GESTURE_END_CNT		*/		6,
        /*Useless, GESTURE_QMAX     */	((1 << 16) - 1),
        /*Useless, GESTURE_PHASE_ZERO     */	(4 * (32 + 1)),
        /*Useless, GESTURE_PHASE_MAX     */	(4 * (32 + 1)) * 2 + 1,
    },
};

/*-------------------*/


typedef int (*tfCT_Offset)(int iSub0, int iSub1, int *iOffset0, int *iOffset1);

typedef struct tagCTTrack {
    int iVal[2];
    int iInt[2];
    int iDiff[2];

    int iState;
    int iCnt;
    int iStepCnt[2];

    tfCT_Offset cbCT_Offset;
    int iOffsetStatus;
    int iOffset[2];
} tfCTTrack;

#define CTTrack_ChkSignal(p, i) ( abs((p)->iDiff[i]) > CTTRACK_UNSTABLE_TH ? 1 : 0)
#define CTTrack_ChkSignal2(p, i, th) ( abs((p)->iDiff[i]) > th ? 1 : 0)
#define iCTTrack_GetInt(p, ch)  ((p)->iInt[ch]/* - (p)->iOffset[ch]*/)
void CTTrack_Init(tfCTTrack *ptInst);
void CTTrack_DoTrack(tfCTTrack *ptInst, int iCh0, int iCh1);
int iCTTrack_Do(tfCTTrack *ptInst, int iCh0, int iCh1);


enum {
    S_STATE_INIT,
    S_STATE_NO_SUN,
    S_STATE_MED_SUN,
    S_STATE_HIGH_SUN,
};

#define SUN_IS_HIGH_SUN_COUNT 	800
#define SUN_IS_MED_SUN_COUNT 	800
#define SUN_IS_NO_SUN_COUNT 	800


typedef struct tagSunDet {
    int iHighSunCnt;
    int iMedSunCnt;
    int iNoSunCnt;
    int iSunState;
    int iALSLux;
    int iMedLuxThd;
    int iHighLuxThd;
} tfSunDet;


enum {
    CTTRACK_STATE_INIT = 0,
    CTTRACK_STATE_UNSTABLE,
    CTTRACK_STATE_STABLE,

    CTTRACK_STATE_OFFSET_WAIT,
};

enum {
    GESTURE_STATE_INIT = 0,
    GESTURE_STATE_IDLE,
    GESTURE_STATE_SIGNAL,
    GESTURE_STATE_NF,
    GESTURE_STATE_SIGNAL_END,
    // GESTURE_STATE_HOLD,
};


#define GESTURE_EVENT_UL  (GESTURE_EVENT_UP | GESTURE_EVENT_LEFT)
#define GESTURE_EVENT_UR  (GESTURE_EVENT_UP | GESTURE_EVENT_RIGHT)
#define GESTURE_EVENT_DL  (GESTURE_EVENT_DOWN | GESTURE_EVENT_LEFT)
#define GESTURE_EVENT_DR  (GESTURE_EVENT_DOWN | GESTURE_EVENT_RIGHT)
#define GESTURE_EVENT_AR  (GESTURE_EVENT_UP | GESTURE_EVENT_DOWN)


#define GESTURE_INTP_LIMIT 		 32
#define GESTURE_BUF_SZ_LIMIT     (80*(GESTURE_INTP_LIMIT+1))
#define GESTURE_PHASE_MAX_LIMIT	 ((12*(GESTURE_INTP_LIMIT+1))*2+1)


enum {
    GESTURE_MAP_NA = 0,
    GESTURE_MAP_UP_DOWN,
    GESTURE_MAP_LEFT_RIGHT,
    GESTURE_MAP_LEFT_RIGHT_UP_DOWN,
    GESTURE_MAP_45_DEG,
    GESTURE_MAP_45_DEG_UP_DOWN,
    GESTURE_MAP_45_DEG_LEFT_RIGHT,
    GESTURE_MAP_45_DEG_UP_DOW_LEFT_RIGHT,
};

typedef struct tagGesture {
    int iSensorType;
    tfCTTrack xtCTTrack[2];

    // Ring Buffer
    // int iBuf[4][GESTURE_BUF_SZ];
    int iBuf[4][GESTURE_BUF_SZ_LIMIT];
    int iPtr;

    // Cross-Correlation Results
    int iCor[2][GESTURE_PHASE_MAX_LIMIT];
    int iMaxCorr[2], iMaxPhase[2];
    int iCenter[4], iCenterPwr[4];
    int iSpeed;
    int iGesLen;
    int iAngle;

    // Gesture Detection
    int iMapIdx;
    int iGesLv[2];

    // Statemachine
    int iState;
    int iPrevState;
    int iCnt;
    int iNoSigCnt;

    // Combo Gesture
    int iComboRet;
    int iComboCnt;

    // Signal Interpolation
    int iLastVal[4];
    int iIntp[4][GESTURE_INTP_LIMIT];

    // Sun Detection
    tfSunDet xtSunDet;
} tfGesture;

#define bGesture_ChkSignal(p, a, i, s, th) ( (s - p->xtCTTrack[a].iInt[i]) > th)

int iGesture_Quantize(int iVal, int iTh);
void Gesture_Enq(tfGesture *ptInst, int iCh0, int iCh1, int iCh2, int iCh3);
int iGesture_Interpolation(int x1, int x2, int y1, int y2, int x);
void Gesture_IntpEnq(tfGesture *ptInst, int iCh0, int iCh1, int iCh2, int iCh3, int iIntp);

void Gesture_DoCorr(tfGesture *ptInst, int iAxis);
int iGesture_Detect(tfGesture *ptInst);

#define NFDET_HOLD_HB       (1024 + 200)
#define NFDET_HOLD_LB       (1024 - 200)
#define NFDET_HOLD_CNT      150
#define NFDET_BACK_CNT      20
#define NFDET_EXIT_CNT      100

#define RATIO_1X            1024

enum {
    NFDET_STATE_IDLE = 0,
    NFDET_STATE_HOLD_DET,
    NFDET_STATE_HOLDING,
    NFDET_STATE_NEAR_FAR,
};

enum {
    NFDET_EVENT_NA = 0x100,
    NFDET_EVENT_HOLD,
    NFDET_EVENT_TOO_FAR,
    NFDET_EVENT_EXIT,
    NFDET_EVENT_NEAR,
    NFDET_EVENT_FAR,
    NFDET_EVENT_BACK,
};

typedef struct tagNFDet {
    int iState;
    int iCnt;
    int iDebounce;

    int iLastVal;
    int iCT;
    int iThHold;
    int iThExit;

    int iZeroVal;
    int iDiff;
    int iRatio;
} tfNFDet;

#define iNFDet_GetRatio(ptInst)  ((ptInst)->iRatio)
#define iNFDet_GetDiff(ptInst)   ((ptInst)->iDiff)

void NFDet_Init(tfNFDet *ptInst);
int iNFDet_Do(tfNFDet *ptInst, int iPS);

tfGesture xtGes;
tfNFDet xtNFDet;
tfGesture_opt currGesture_opt;

int iAngleMap[][9][3] = {
    // GESTURE_MAP_NA
    {
        // -pi, +pi
        {-1024, 1024, 	GESTURE_EVENT_NA},
    },
    // GESTURE_MAP_UP_DOWN
    {
        // -0.5pi, +0.5pi
        {-512, 512, 	GESTURE_EVENT_UP},
        // -pi, -0.5pi
        {-1024, -512, 	GESTURE_EVENT_DOWN},
        // +0.5pi, +pi
        {512, 1024, 	GESTURE_EVENT_DOWN},
    },
    // GESTURE_MAP_LEFT_RIGHT
    {
        // -pi, 0
        {-1024, 0, 		GESTURE_EVENT_LEFT},
        // 0, +pi
        {0, 1024, 		GESTURE_EVENT_RIGHT},
    },
    // // GESTURE_MAP_LEFT_RIGHT_UP_DOWN
    {
        // -pi, -0.75pi
        {-1024, -768, 	GESTURE_EVENT_DOWN},
        // -0.75pi, -0.25pi
        {-768, -256, 	GESTURE_EVENT_LEFT},
        // -0.25pi, +0.25pi
        {-256, 256, 	GESTURE_EVENT_UP},
        // +0.25pi, +0.75pi
        {256, 768, 		GESTURE_EVENT_RIGHT},
        // +0.75pi, +pi
        {768, 1024, 	GESTURE_EVENT_DOWN},
    },
    // GESTURE_MAP_45_DEG
    {
        // -pi, -0.5pi
        {-1024, -512, 	GESTURE_EVENT_DL},
        // -0.5pi, 0
        {-512, 0, 		GESTURE_EVENT_UL},
        // 0, +0.5pi
        {0, 512, 		GESTURE_EVENT_UR},
        // +0.5pi, pi
        {512, 1024, 	GESTURE_EVENT_DR},
    },
    // GESTURE_MAP_45_DEG_UP_DOWN
    {
        // -pi, -0.875pi
        {-1024, -896, 	GESTURE_EVENT_DOWN},
        // -0.875pi, -0.5pi
        {-896, -512, 	GESTURE_EVENT_DL},
        // -0.5pi, -0.125pi
        {-512, -128, 	GESTURE_EVENT_UL},
        // -0.125pi, +0.125pi
        {-128, 128, 	GESTURE_EVENT_UP},
        // +0.125pi, +0.5pi
        {128, 512, 		GESTURE_EVENT_UR},
        // +0.5pi, +0.875pi
        {512, 896, 		GESTURE_EVENT_DR},
        // +0.875pi, +pi
        {896, 1024, 	GESTURE_EVENT_DOWN},
    },
    // GESTURE_MAP_45_DEG_LEFT_RIGHT
    {
        // -pi, -0.625pi
        {-1024, -640, 	GESTURE_EVENT_DL},
        // -0.625pi, -0.375pi
        {-640, -384, 	GESTURE_EVENT_LEFT},
        // -0.375pi, 0
        {-384, 0, 		GESTURE_EVENT_UL},
        // +0, +0.375pi
        {0, 384, 		GESTURE_EVENT_UR},
        // +0.375pi, +0.625pi
        {384, 640, 		GESTURE_EVENT_RIGHT},
        // +0.625pi, +pi
        {640, 1024, 	GESTURE_EVENT_DR},
    },
    // GESTURE_MAP_45_DEG_UP_DOW_LEFT_RIGHT
    {
        // -pi, -0.875pi
        {-1024, -896, 	GESTURE_EVENT_DOWN},
        // -0.875pi, -0.625pi
        {-896, -640, 	GESTURE_EVENT_DL},
        // -0.625pi, -0.375pi
        {-640, -384, 	GESTURE_EVENT_LEFT},
        // -0.375pi, -0.125pi
        {-384, -128, 	GESTURE_EVENT_UL},
        // -0.125pi, +0.125pi
        {-128, 128, 	GESTURE_EVENT_UP},
        // +0.125pi, +0.375pi
        {128, 384, 		GESTURE_EVENT_UR},
        // +0.375pi, +0.625pi
        {384, 640, 		GESTURE_EVENT_RIGHT},
        // +0.625pi, +0.875pi
        {640, 896, 		GESTURE_EVENT_DR},
        // +0.875pi, +pi
        {896, 1024, 	GESTURE_EVENT_DOWN},
    }
};

/******************************************************************************
  CTTrack
 ******************************************************************************/

void CTTrack_Init(tfCTTrack *ptInst)
{
    ptInst->iInt[0]		= 0;
    ptInst->iInt[1]		= 0;
    ptInst->iDiff[0]	= 0;
    ptInst->iDiff[1]	= 0;
    ptInst->iInt[1]		= 0;
    ptInst->iStepCnt[0] = 0;
    ptInst->iStepCnt[1] = 0;
    ptInst->iCnt		= 0;
    ptInst->iState		= CTTRACK_STATE_INIT;

    ptInst->cbCT_Offset   = NULL;
    ptInst->iOffsetStatus = 0;
    ptInst->iOffset[0]    = 0;
    ptInst->iOffset[1]    = 0;
}

void CTTrack_DoTrack(tfCTTrack *ptInst, int iCh0, int iCh1)
{
    int iStep[2], iCh;

    ptInst->iDiff[0] = (iCh0 - ptInst->iInt[0]);
    ptInst->iDiff[1] = (iCh1 - ptInst->iInt[1]);

    for (iCh = 0; iCh < 2; iCh ++) {
        if (ptInst->iDiff[iCh] == 0)
            iStep[iCh] = 0;
        else if (ptInst->iState == CTTRACK_STATE_STABLE) {
            ptInst->iStepCnt[iCh] ++;
            if (ptInst->iStepCnt[iCh] == currGesture_opt.CTTRACK_RATIO_STEP) {
                ptInst->iStepCnt[iCh] = 0;
                if (ptInst->iDiff[iCh] > 0)
                    iStep[iCh] = 1;
                else
                    iStep[iCh] = -1;
            } else
                iStep[iCh]  = 0;
        } else if (ptInst->iState == CTTRACK_STATE_UNSTABLE) {
            if (ptInst->iDiff[iCh] > 0) {
                iStep[iCh] = ptInst->iDiff[iCh] / currGesture_opt.CTTRACK_RATIO_RISE;
                if (iStep[iCh] == 0)
                    iStep[iCh] = 1;
            } else {
                iStep[iCh] = ptInst->iDiff[iCh] / currGesture_opt.CTTRACK_RATIO_FALL;
                if (iStep[iCh] == 0)
                    iStep[iCh] = -1;
            }
        }

        ptInst->iInt[iCh] += iStep[iCh];
    }
}
int iCTTrack_Do(tfCTTrack *ptInst, int iCh0, int iCh1)
{
    ptInst->iVal[0]	= iCh0;
    ptInst->iVal[1]	= iCh1;
    if (ptInst->iState == CTTRACK_STATE_INIT) {
        ptInst->iInt[0]	= iCh0;
        ptInst->iInt[1]	= iCh1;
        ptInst->iState	= CTTRACK_STATE_UNSTABLE;
        ptInst->iCnt    = currGesture_opt.CTTRACK_STABLE_CNT;
    } else if (ptInst->iState == CTTRACK_STATE_UNSTABLE) {
        CTTrack_DoTrack(ptInst, iCh0, iCh1);

        if (abs(ptInst->iDiff[0]) <= currGesture_opt.CTTRACK_STABLE_TH &&
           abs(ptInst->iDiff[1]) <= currGesture_opt.CTTRACK_STABLE_TH) {
            ptInst->iCnt --;
            if (ptInst->iCnt == 0) {
                {
                    ptInst->iState	= CTTRACK_STATE_STABLE;
                    ptInst->iCnt    = currGesture_opt.CTTRACK_UNSTABLE_CNT;
                }
            }
        } else {
            ptInst->iCnt = currGesture_opt.CTTRACK_STABLE_CNT;
        }


        if (ptInst->iState == CTTRACK_STATE_UNSTABLE && ptInst->cbCT_Offset != NULL) {
            ptInst->iOffsetStatus = (ptInst->cbCT_Offset)
                                    ( ptInst->iInt[0], ptInst->iInt[1],
                                      &(ptInst->iOffset[0]), &(ptInst->iOffset[1]) );

            if ((ptInst->iOffsetStatus & 0x80) != 0) {
                ptInst->iState = CTTRACK_STATE_OFFSET_WAIT;
                ptInst->iCnt   = currGesture_opt.CTTRACK_OFFSET_WAIT_CNT;
            }
        }
    } else if (ptInst->iState == CTTRACK_STATE_OFFSET_WAIT) {
        //CTTrack_DoTrack(ptInst, iCh0, iCh1);
        ptInst->iCnt --;
        if (ptInst->iCnt == 0) {
            ptInst->iState = CTTRACK_STATE_UNSTABLE;
            ptInst->iCnt   = currGesture_opt.CTTRACK_STABLE_CNT;
            //ptInst->iState = CTTRACK_STATE_STABLE;
            //ptInst->iCnt = currGesture_opt.CTTRACK_UNSTABLE_CNT;
        }
    } else if (ptInst->iState == CTTRACK_STATE_STABLE) {
        CTTrack_DoTrack(ptInst, iCh0, iCh1);

        if (abs(ptInst->iDiff[0]) >= currGesture_opt.CTTRACK_UNSTABLE_TH ||
           abs(ptInst->iDiff[1]) >= currGesture_opt.CTTRACK_UNSTABLE_TH) {
            ptInst->iCnt --;
            if (ptInst->iCnt == 0) {
                ptInst->iState = CTTRACK_STATE_UNSTABLE;
                ptInst->iCnt   = currGesture_opt.CTTRACK_STABLE_CNT;
            }
        } else {
            ptInst->iCnt = currGesture_opt.CTTRACK_UNSTABLE_CNT;
        }

        if (ptInst->cbCT_Offset != NULL) {
            ptInst->iOffsetStatus = (ptInst->cbCT_Offset)
                                    ( ptInst->iInt[0], ptInst->iInt[1],
                                      &(ptInst->iOffset[0]), &(ptInst->iOffset[1]) );

            if ((ptInst->iOffsetStatus & 0x80) != 0) {
                ptInst->iState = CTTRACK_STATE_OFFSET_WAIT;
                ptInst->iCnt   = currGesture_opt.CTTRACK_OFFSET_WAIT_CNT;
            }
        }
    }

    return ptInst->iState;
}


/******************************************************************************
  Gesture
 ******************************************************************************/
int Gesture_Init(tfGesture *ptInst)
{
    currGesture_opt = xtGesture_opt[ptInst->iSensorType];

    ptInst->iState = GESTURE_STATE_INIT;
    ptInst->iMaxPhase[0] = currGesture_opt.GESTURE_PHASE_ZERO;
    ptInst->iMaxPhase[1] = currGesture_opt.GESTURE_PHASE_ZERO;
    ptInst->iCnt = 0;
    ptInst->iNoSigCnt = 0;
    ptInst->iPtr = 0;
    ptInst->iMapIdx = GESTURE_MAP_LEFT_RIGHT_UP_DOWN;
    if (currGesture_opt.GESTURE_COMBO_CNT > 0) {
        ptInst->iComboRet = GESTURE_EVENT_NA;
        ptInst->iComboCnt = 0;
    }
    currGesture_opt.GESTURE_QMAX       = ((1 << currGesture_opt.GESTURE_QBIT) - 1);
    currGesture_opt.GESTURE_PHASE_ZERO = currGesture_opt.GESTURE_PHASE_RANGE;
    currGesture_opt.GESTURE_PHASE_MAX  = (currGesture_opt.GESTURE_PHASE_RANGE * 2 + 1);

    if (currGesture_opt.GESTURE_INTP > GESTURE_INTP_LIMIT) {
        //printf("ERROR! currGesture_opt.GESTURE_INTP(%d) exceed limit(%d)", currGesture_opt.GESTURE_INTP, GESTURE_INTP_LIMIT);
        return -1;
    }

    ptInst->xtSunDet.iSunState = S_STATE_INIT;
    ptInst->xtSunDet.iNoSunCnt = 0;
    ptInst->xtSunDet.iMedLuxThd = SUN_ALS_LUX_MED_THD;
    ptInst->xtSunDet.iHighLuxThd = SUN_ALS_LUX_HIGH_THD;
    ptInst->xtSunDet.iHighSunCnt = SUN_IS_HIGH_SUN_COUNT;
    ptInst->xtSunDet.iMedSunCnt = SUN_IS_MED_SUN_COUNT;

    return 0;
}

int iGesture_Interpolation(int x1, int x2, int y1, int y2, int x)
{
    int a, b;

    if (y2 == y1)
        return y1;

    a = x - x1;
    b = x2 - x;
    return (y1 * b + y2 * a) / (x2 - x1);
}

//#if (GESTURE_COMBO_CNT>0)
int iGesture_Combo(tfGesture *ptInst, int iNew)
{
    if (iNew == GESTURE_EVENT_UP && (ptInst->iComboRet & GESTURE_EVENT_DOWN) != GESTURE_EVENT_NA) {
        return GESTURE_EVENT_DOWN_THEN_UP;
    } else if (iNew == GESTURE_EVENT_DOWN && (ptInst->iComboRet & GESTURE_EVENT_UP) != GESTURE_EVENT_NA) {
        return GESTURE_EVENT_UP_THEN_DOWN;
    } else if (iNew == GESTURE_EVENT_LEFT && (ptInst->iComboRet & GESTURE_EVENT_RIGHT) != GESTURE_EVENT_NA) {
        return GESTURE_EVENT_RIGHT_THEN_LEFT;
    } else if (iNew == GESTURE_EVENT_RIGHT && (ptInst->iComboRet & GESTURE_EVENT_LEFT) != GESTURE_EVENT_NA) {
        return GESTURE_EVENT_LEFT_THEN_RIGHT;
    } else {
        return iNew;
    }
}
//#endif



void iGesture_MonitorS(tfGesture *ptInst, int als)
{
    ptInst->xtSunDet.iALSLux = als;
}

void iGesture_SunDo(tfGesture *ptInst)
{
    if (ptInst->xtSunDet.iSunState == S_STATE_INIT) {
        ptInst->xtSunDet.iSunState = S_STATE_NO_SUN;
        ptInst->xtSunDet.iALSLux = 0;
    } else if (ptInst->xtSunDet.iSunState == S_STATE_NO_SUN) {
        if (ptInst->xtSunDet.iALSLux >= ptInst->xtSunDet.iMedLuxThd) {
            ptInst->xtSunDet.iMedSunCnt--;
            if (ptInst->xtSunDet.iMedSunCnt == 0) {
                ptInst->xtSunDet.iSunState = S_STATE_MED_SUN;
                ptInst->xtSunDet.iNoSunCnt = SUN_IS_NO_SUN_COUNT;
                ptInst->xtSunDet.iHighSunCnt = SUN_IS_HIGH_SUN_COUNT;
                currGesture_opt.GESTURE_CH0_TH *= 4;
                currGesture_opt.GESTURE_CH1_TH *= 4;
                currGesture_opt.GESTURE_CH2_TH *= 4;
                currGesture_opt.GESTURE_CH3_TH *= 4;
                currGesture_opt.CTTRACK_STABLE_TH *= 4;
                currGesture_opt.CTTRACK_UNSTABLE_TH *= 4;
                ptInst->iState = GESTURE_STATE_IDLE;
            }
        } else {
            ptInst->xtSunDet.iMedSunCnt = SUN_IS_MED_SUN_COUNT;
        }
    } else if (ptInst->xtSunDet.iSunState == S_STATE_MED_SUN) {
        if (ptInst->xtSunDet.iALSLux >= ptInst->xtSunDet.iHighLuxThd) {
            ptInst->xtSunDet.iHighSunCnt--;
            ptInst->xtSunDet.iNoSunCnt = SUN_IS_NO_SUN_COUNT;
            if (ptInst->xtSunDet.iHighSunCnt == 0) {
                ptInst->xtSunDet.iSunState = S_STATE_HIGH_SUN;
                ptInst->xtSunDet.iMedSunCnt = SUN_IS_MED_SUN_COUNT;
                ptInst->iState = GESTURE_STATE_IDLE;
            }
        } else if (ptInst->xtSunDet.iALSLux >= ptInst->xtSunDet.iMedLuxThd) {
            ptInst->xtSunDet.iHighSunCnt = SUN_IS_HIGH_SUN_COUNT;
            ptInst->xtSunDet.iNoSunCnt = SUN_IS_NO_SUN_COUNT;
        } else {
            ptInst->xtSunDet.iHighSunCnt = SUN_IS_HIGH_SUN_COUNT;
            ptInst->xtSunDet.iNoSunCnt--;
            if (ptInst->xtSunDet.iNoSunCnt == 0) {
                ptInst->xtSunDet.iSunState = S_STATE_NO_SUN;
                ptInst->xtSunDet.iMedSunCnt = SUN_IS_MED_SUN_COUNT;
                currGesture_opt.GESTURE_CH0_TH /= 4;
                currGesture_opt.GESTURE_CH1_TH /= 4;
                currGesture_opt.GESTURE_CH2_TH /= 4;
                currGesture_opt.GESTURE_CH3_TH /= 4;
                currGesture_opt.CTTRACK_STABLE_TH /= 4;
                currGesture_opt.CTTRACK_UNSTABLE_TH /= 4;

                ptInst->iState = GESTURE_STATE_IDLE;
            }
        }
    } else if (ptInst->xtSunDet.iSunState == S_STATE_HIGH_SUN) {
        if (ptInst->xtSunDet.iALSLux >= ptInst->xtSunDet.iHighLuxThd) {
            ptInst->xtSunDet.iMedSunCnt = SUN_IS_MED_SUN_COUNT;
        } else {
            ptInst->xtSunDet.iMedSunCnt--;
            if (ptInst->xtSunDet.iMedSunCnt == 0) {
                ptInst->xtSunDet.iSunState = S_STATE_MED_SUN;
                ptInst->xtSunDet.iHighSunCnt = SUN_IS_HIGH_SUN_COUNT;
                ptInst->xtSunDet.iNoSunCnt = SUN_IS_NO_SUN_COUNT;
            }
        }
    }
}


int iGesture_Do(tfGesture *ptInst, int iCh0, int iCh1, int iCh2, int iCh3)
{
    int iRet = GESTURE_EVENT_NA;

    iGesture_SunDo(ptInst);

    if (ptInst->xtSunDet.iSunState == S_STATE_HIGH_SUN) {
        return iRet;
    }

    iCh0 += ptInst->xtCTTrack[0].iOffset[0];
    iCh1 += ptInst->xtCTTrack[0].iOffset[1];
    iCh2 += ptInst->xtCTTrack[1].iOffset[0];
    iCh3 += ptInst->xtCTTrack[1].iOffset[1];

    iCh1 = iCh1 * currGesture_opt.GESTURE_CH1_RATIO_M / currGesture_opt.GESTURE_CH1_RATIO_N;

    ptInst->iPrevState = ptInst->iState;

    if (ptInst->iState == GESTURE_STATE_INIT) {
        CTTrack_Init(&ptInst->xtCTTrack[0]);
        CTTrack_Init(&ptInst->xtCTTrack[1]);
    }

    iCTTrack_Do(&ptInst->xtCTTrack[0], iCh0, iCh1);
    iCTTrack_Do(&ptInst->xtCTTrack[1], iCh2, iCh3);

    if (ptInst->iState ==  GESTURE_STATE_INIT) {
        ptInst->iState = GESTURE_STATE_IDLE;
        ptInst->iCnt   = 0;
    } else if (ptInst->iState == GESTURE_STATE_IDLE) {

        //        if (CTTrack_ChkSignal(&(ptInst->xtCTTrack[0]), 0) != 0 || CTTrack_ChkSignal(&(ptInst->xtCTTrack[0]), 1) != 0 ||
        //            CTTrack_ChkSignal(&(ptInst->xtCTTrack[1]), 0) != 0 || CTTrack_ChkSignal(&(ptInst->xtCTTrack[1]), 1) != 0
        if (bGesture_ChkSignal(ptInst, 0, 0, iCh0, currGesture_opt.GESTURE_CH0_TH)
            || bGesture_ChkSignal(ptInst, 0, 1, iCh1, currGesture_opt.GESTURE_CH1_TH)
            || bGesture_ChkSignal(ptInst, 1, 0, iCh2, currGesture_opt.GESTURE_CH2_TH)
            || bGesture_ChkSignal(ptInst, 1, 1, iCh3, currGesture_opt.GESTURE_CH3_TH)
           ) {
            ptInst->iCnt    ++;
            if (ptInst->iCnt >= currGesture_opt.GESTURE_SIGNAL_CNT) {
                if ( 1 &&
                    (currGesture_opt.GESTURE_AXIS1_EN ? (ptInst->xtCTTrack[0].iDiff[0] >= 2 && ptInst->xtCTTrack[0].iDiff[1] >= 2) : 1)
                    &&
                    (currGesture_opt.GESTURE_AXIS2_EN ? (ptInst->xtCTTrack[1].iDiff[0] >= 2 && ptInst->xtCTTrack[1].iDiff[1] >= 2) : 1)
                  ) {
                    ptInst->iState = GESTURE_STATE_SIGNAL;
                    ptInst->iNoSigCnt = 0;
                }

                ptInst->iCnt = 0;
            }

            Gesture_IntpEnq(ptInst, iCh0, iCh1, iCh2, iCh3, 1);
        } else {
            if (currGesture_opt.GESTURE_COMBO_CNT > 0) {
                if (ptInst->iComboCnt > 0) {
                    ptInst->iComboCnt--;
                } else {
                    iRet              = ptInst->iComboRet;
                    ptInst->iComboRet = GESTURE_EVENT_NA;
                }
            }
            ptInst->iCnt    = 0;
            Gesture_IntpEnq(ptInst, iCh0, iCh1, iCh2, iCh3, 0);
        }
    } else if (ptInst->iState == GESTURE_STATE_SIGNAL) {
        ptInst->iCnt++;
        if (ptInst->xtCTTrack[0].iState != CTTRACK_STATE_STABLE ||
           ptInst->xtCTTrack[1].iState != CTTRACK_STATE_STABLE) {
            ptInst->iState  = GESTURE_STATE_NF;
            ptInst->iCnt    = 0;
        }
        //        else if (CTTrack_ChkSignal(&(ptInst->xtCTTrack[0]), 0) == 0 && CTTrack_ChkSignal(&(ptInst->xtCTTrack[0]), 1) == 0 &&
        //            CTTrack_ChkSignal(&(ptInst->xtCTTrack[1]), 0) == 0 && CTTrack_ChkSignal(&(ptInst->xtCTTrack[1]), 1) == 0
        else if ( !bGesture_ChkSignal(ptInst, 0, 0, iCh0, currGesture_opt.GESTURE_CH0_TH)
                 && !bGesture_ChkSignal(ptInst, 0, 1, iCh1, currGesture_opt.GESTURE_CH1_TH)
                 && !bGesture_ChkSignal(ptInst, 1, 0, iCh2, currGesture_opt.GESTURE_CH2_TH)
                 && !bGesture_ChkSignal(ptInst, 1, 1, iCh3, currGesture_opt.GESTURE_CH3_TH)
               ) {
            ptInst->iNoSigCnt++;
            if (ptInst->iNoSigCnt >= 2) {
                if (ptInst->iCnt >= currGesture_opt.GESTURE_NF_MIN_CNT &&
                   ptInst->iCnt <= currGesture_opt.GESTURE_NF_MAX_CNT) {
                    iRet = GESTURE_EVENT_NA;

                    if (currGesture_opt.GESTURE_AXIS1_EN)
                        Gesture_DoCorr(ptInst, 0);

                    if (currGesture_opt.GESTURE_AXIS2_EN)
                        Gesture_DoCorr(ptInst, 1);
                    ptInst->iGesLen	= ptInst->iCnt;
                    ptInst->iCnt = 0;

                    iRet = iGesture_Detect(ptInst);
                    ptInst->iState  = GESTURE_STATE_IDLE;

                    if (currGesture_opt.GESTURE_COMBO_CNT > 0) {
                        ptInst->iComboCnt = currGesture_opt.GESTURE_COMBO_CNT;

                        if (ptInst->iComboRet != GESTURE_EVENT_NA) {
                            iRet               = iGesture_Combo(ptInst, iRet);
                            ptInst->iComboRet  = GESTURE_EVENT_NA;
                        } else {
                            ptInst->iComboRet  = iRet;
                            iRet               = GESTURE_EVENT_NA;
                        }
                    }

                    if (iRet != GESTURE_STATE_IDLE) {
                        ptInst->iState = GESTURE_STATE_SIGNAL_END;
                        ptInst->iCnt = currGesture_opt.GESTURE_END_CNT;
                    }
                } else {
                    ptInst->iState  = GESTURE_STATE_NF;

                    ptInst->iGesLen	= ptInst->iCnt;
                    ptInst->iCnt = 0;
                }
            }
        }

        Gesture_IntpEnq(ptInst, iCh0, iCh1, iCh2, iCh3, 1);
    } else if (ptInst->iState == GESTURE_STATE_NF) {
        if (ptInst->xtCTTrack[0].iState == CTTRACK_STATE_STABLE &&
           ptInst->xtCTTrack[1].iState == CTTRACK_STATE_STABLE) {
            ptInst->iCnt++;
            if (ptInst->iCnt >= currGesture_opt.GESTURE_SIGNAL_NF_CNT) {
                ptInst->iState = GESTURE_STATE_IDLE;
                ptInst->iCnt = 0;
                iRet = GESTURE_EVENT_NF;
            }
        } else {
            ptInst->iCnt = 0;
        }
        Gesture_IntpEnq(ptInst, iCh0, iCh1, iCh2, iCh3, 0);
    } else if (ptInst->iState ==  GESTURE_STATE_SIGNAL_END) {
        ptInst->iCnt--;
        if (ptInst->iCnt == 0)
            ptInst->iState = GESTURE_STATE_IDLE;
    }

    return iRet;
}

void Gesture_IntpEnq(tfGesture *ptInst, int iCh0, int iCh1, int iCh2, int iCh3, int iIntp)
{
    int iV0, iV1, iV2, iV3;
    int x;

    if (currGesture_opt.GESTURE_INTP > 0) {
        x = 1;
        for (x = 1; x < (currGesture_opt.GESTURE_INTP + 1); x++) {
            if (iIntp != 0)
            {
                iV0 = iGesture_Interpolation(0, (currGesture_opt.GESTURE_INTP + 1), ptInst->iLastVal[0], iCh0, x);
                iV1 = iGesture_Interpolation(0, (currGesture_opt.GESTURE_INTP + 1), ptInst->iLastVal[1], iCh1, x);
                iV2 = iGesture_Interpolation(0, (currGesture_opt.GESTURE_INTP + 1), ptInst->iLastVal[2], iCh2, x);
                iV3 = iGesture_Interpolation(0, (currGesture_opt.GESTURE_INTP + 1), ptInst->iLastVal[3], iCh3, x);
            } else {
                iV0 = iCh0;
                iV1 = iCh1;
                iV2 = iCh2;
                iV3 = iCh3;
            }

            ptInst->iIntp[0][x - 1] = iV0;
            ptInst->iIntp[1][x - 1] = iV1;
            ptInst->iIntp[2][x - 1] = iV2;
            ptInst->iIntp[3][x - 1] = iV3;

            Gesture_Enq(ptInst, iV0, iV1, iV2, iV3);
        }
    }

    Gesture_Enq(ptInst, iCh0, iCh1, iCh2, iCh3);

    ptInst->iLastVal[0] = iCh0;
    ptInst->iLastVal[1] = iCh1;
    ptInst->iLastVal[2] = iCh2;
    ptInst->iLastVal[3] = iCh3;
}

int iGesture_Quantize(int iVal, int iTh)
{
    if (iVal < iTh)
        return 0;
    else {
        iVal = (iVal - iTh) / currGesture_opt.GESTURE_QSTEP;
        if (iVal > currGesture_opt.GESTURE_QMAX)
            iVal = currGesture_opt.GESTURE_QMAX;
        return iVal;
    }
}

void Gesture_Enq(tfGesture *ptInst, int iCh0, int iCh1, int iCh2, int iCh3)
{
    if (iCh0 < (iCTTrack_GetInt(&(ptInst->xtCTTrack[0]), 0) + currGesture_opt.GESTURE_CH0_TH))
        ptInst->iBuf[0][ptInst->iPtr] = 0;
    else
        ptInst->iBuf[0][ptInst->iPtr] = iGesture_Quantize(iCh0, iCTTrack_GetInt(&(ptInst->xtCTTrack[0]), 0) + currGesture_opt.GESTURE_CH0_TH);

    if (iCh1 < (iCTTrack_GetInt(&(ptInst->xtCTTrack[0]), 1) + currGesture_opt.GESTURE_CH1_TH))
        ptInst->iBuf[1][ptInst->iPtr] = 0;
    else
        ptInst->iBuf[1][ptInst->iPtr] = iGesture_Quantize(iCh1, iCTTrack_GetInt(&(ptInst->xtCTTrack[0]), 1) + currGesture_opt.GESTURE_CH1_TH);

    if (iCh2 < (iCTTrack_GetInt(&(ptInst->xtCTTrack[1]), 0) + currGesture_opt.GESTURE_CH2_TH))
        ptInst->iBuf[2][ptInst->iPtr] = 0;
    else
        ptInst->iBuf[2][ptInst->iPtr] = iGesture_Quantize(iCh2, iCTTrack_GetInt(&(ptInst->xtCTTrack[1]), 0) + currGesture_opt.GESTURE_CH2_TH);

    if (iCh3 < (iCTTrack_GetInt(&(ptInst->xtCTTrack[1]), 1) + currGesture_opt.GESTURE_CH3_TH))
        ptInst->iBuf[3][ptInst->iPtr] = 0;
    else
        ptInst->iBuf[3][ptInst->iPtr] = iGesture_Quantize(iCh3, iCTTrack_GetInt(&(ptInst->xtCTTrack[1]), 1) + currGesture_opt.GESTURE_CH3_TH);

    ptInst->iPtr++;
    if (ptInst->iPtr == currGesture_opt.GESTURE_BUF_SZ)
        ptInst->iPtr = 0;
}

void Gesture_DoCorr(tfGesture *ptInst, int iAx)
{
    int i, p1, p2, n, iAxis;

    iAxis                 = iAx * 2;
    ptInst->iMaxCorr[iAx] = -1;
    //for (i=0; i<currGesture_opt.GESTURE_PHASE_ZERO; i++)
    for (i = currGesture_opt.GESTURE_PHASE_ZERO - 1; i >= 0; i--) {
        ptInst->iCor[iAx][i] = 0;
        p1 = ptInst->iPtr - 1;
        p2 = ptInst->iPtr - 1 - (currGesture_opt.GESTURE_PHASE_ZERO - i);
        //        iZCnt   = 0;
        for (n = 0; n < (currGesture_opt.GESTURE_BUF_SZ - i); n++) {
            if (n > ((ptInst->iCnt + 2) * (currGesture_opt.GESTURE_INTP + 1)) - (currGesture_opt.GESTURE_PHASE_ZERO - i))
                break;

            if (p1 < 0)
                p1 += currGesture_opt.GESTURE_BUF_SZ;
            if (p2 < 0)
                p2 += currGesture_opt.GESTURE_BUF_SZ;
#if (0)
            if (ptInst->iCor[iAx][i] > 0 && ptInst->iBuf[iAxis][p1] == 0 && ptInst->iBuf[iAxis + 1][p2] == 0) {
                iZCnt++;
                if (iZCnt > (2 * (currGesture_opt.GESTURE_INTP + 1)))
                    break;
            } else
                iZCnt = 0;
#endif
            ptInst->iCor[iAx][i] += ptInst->iBuf[iAxis][p1] * ptInst->iBuf[iAxis + 1][p2];
            p1--;
            p2--;
        }

        //ptInst->iCor[iAx][i] /= 64;
        if (n > 0) {
            //ptInst->iCor[iAx][i] = (ptInst->iCor[iAx][i] * ((ptInst->iCnt+2) * (currGesture_opt.GESTURE_INTP+1)) / n);
            ptInst->iCor[iAx][i] = (ptInst->iCor[iAx][i] / n * ((ptInst->iCnt + 2) * (currGesture_opt.GESTURE_INTP + 1)));
        } else {
            n = n;
        }
        if (ptInst->iMaxCorr[iAx] < ptInst->iCor[iAx][i]) {
            ptInst->iMaxCorr[iAx]  = ptInst->iCor[iAx][i];
            ptInst->iMaxPhase[iAx] = i;
        }
    }

    for (i = currGesture_opt.GESTURE_PHASE_ZERO; i < currGesture_opt.GESTURE_PHASE_MAX; i++) {
        ptInst->iCor[iAx][i] = 0;
        p1 = ptInst->iPtr - 1 - (i - currGesture_opt.GESTURE_PHASE_ZERO);
        p2 = ptInst->iPtr - 1;
        //iZCnt = 0;
        for (n = 0; n < (currGesture_opt.GESTURE_BUF_SZ - i); n++) {
            if (n > ((ptInst->iCnt + 2) * (currGesture_opt.GESTURE_INTP + 1) - (i - currGesture_opt.GESTURE_PHASE_ZERO)))
                break;

            if (p1 < 0)
                p1  += currGesture_opt.GESTURE_BUF_SZ;
            if (p2 < 0)
                p2  += currGesture_opt.GESTURE_BUF_SZ;

#if 0
            if (ptInst->iCor[iAx][i] > 0 && ptInst->iBuf[iAxis][p1] == 0 && ptInst->iBuf[iAxis + 1][p2] == 0) {
                iZCnt ++;
                if (iZCnt > (2 * (currGesture_opt.GESTURE_INTP + 1)))
                    break;
            } else
                iZCnt = 0;
#endif

            ptInst->iCor[iAx][i]    += ptInst->iBuf[iAxis][p1] * ptInst->iBuf[iAxis + 1][p2];
            p1--;
            p2--;
        }

        //ptInst->iCor[iAx][i] /= 64;
        if (n > 0) {
            ptInst->iCor[iAx][i] = (ptInst->iCor[iAx][i] / n * ((ptInst->iCnt + 2) * (currGesture_opt.GESTURE_INTP + 1)));
        }
        if (ptInst->iMaxCorr[iAx] < ptInst->iCor[iAx][i]) {
            ptInst->iMaxCorr[iAx]  = ptInst->iCor[iAx][i];
            ptInst->iMaxPhase[iAx] = i;
        }
    }
}


// {tan * 1024, rad * 1024 / pi}
const int iatan[][2] = {
    {0, 0},
    {51, 16},
    {102, 32},
    {154, 49},
    {205, 64},
    {256, 80},
    {307, 95},
    {358, 110},
    {410, 124},
    {461, 138},
    {512, 151},
    {563, 164},
    {614, 176},
    {666, 188},
    {717, 199},
    {768, 210},
    {819, 220},
    {870, 230},
    {922, 239},
    {973, 248},
    {1024, 256},
};

int iATan2_Search(int iTan)
{
    int i;
    for (i = 0; i < (sizeof(iatan) / sizeof(int) / 2); i++)
        if (iTan <= iatan[i][0])
            break;
    if (i == (sizeof(iatan) / sizeof(int) / 2))
        i--;

    return iatan[i][1];
}

int iATan2_Pos(int iY, int iX)
{
    int iRad;

    if (iX > iY) {
        iRad = iATan2_Search(iY * 1024 / iX);
    } else {
        iRad = 512 - iATan2_Search(iX * 1024 / iY);
    }

    return iRad;
}

int iATan2(int iY, int iX)
{
    if (iX == 0) {
        if (iY == 0)
            return 0x12345678;
        else if (iY >= 0)
            return 512;
        else
            return -512;
    } else {
        if (iX > 0) {
            if (iY >= 0)
                return iATan2_Pos(iY, iX);
            else
                return 0 - iATan2_Pos(0 - iY, iX);
        } else { //if (iX < 0)
            if (iY >= 0)
                return 1024 - iATan2_Pos(iY, 0 - iX);
            else
                return iATan2_Pos(0 - iY, 0 - iX) - 1024;
        }
    }
}

int iGesture_Detect(tfGesture *ptInst)
{
    int iRet = GESTURE_EVENT_NA;
    int iPhase[2];
    int i;
    int iATan;

    if (1 &&
       (currGesture_opt.GESTURE_AXIS1_EN ? (ptInst->iMaxCorr[0] < currGesture_opt.GESTURE_CORR_TH) : 1)
       &&
       (currGesture_opt.GESTURE_AXIS2_EN ? (ptInst->iMaxCorr[1] < currGesture_opt.GESTURE_CORR_TH) : 1)
      ) {
        return GESTURE_EVENT_NA;
    }

    if (currGesture_opt.GESTURE_AXIS1_EN)
        iPhase[0] = abs(ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO);
    else
        iPhase[0] = currGesture_opt.GESTURE_PHASE_ZERO;

    if (currGesture_opt.GESTURE_AXIS2_EN)
        iPhase[1] = abs(ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO);
    else
        iPhase[1] = currGesture_opt.GESTURE_PHASE_ZERO;

    // Lex, 2014/7/11
#if 1
    {
        int iS1, iS2;

        iS1 = ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO;
        iS2 = ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO;
        //ptInst->iSpeed  = currGesture_opt.GESTURE_PHASE_MAX - sqrt(iS1 * iS1 + iS2 * iS2);
    }
    if (ptInst->iMaxCorr[1] > currGesture_opt.GESTURE_CORR_TH) {
        if (ptInst->iMaxCorr[0] > currGesture_opt.GESTURE_CORR_TH) {
            if (ptInst->iMaxPhase[1] != currGesture_opt.GESTURE_PHASE_ZERO ||
               ptInst->iMaxPhase[0] != currGesture_opt.GESTURE_PHASE_ZERO) {
                //dATan = atan2(ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO, ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO);
                iATan = iATan2(ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO, ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO);
                ptInst->iAngle = iATan;

                //printf("atan: %d %d\n", (int)(dATan /pi * 1024.0), iATan);
                //				printf("atan: %d\n", iATan);
                //iRet = GESTURE_EVENT_NA;
                for (i = 0; i < 9; i++) {
                    /*if (dATan >= xtAngleMap[ptInst->iMapIdx][i].dAngleMin &&
                    	dATan <= xtAngleMap[ptInst->iMapIdx][i].dAngleMax)
                    {
                    	iRet = xtAngleMap[ptInst->iMapIdx][i].iEvent;
                    	break;
                    }*/
                    if (iATan >= iAngleMap[ptInst->iMapIdx][i][0] &&
                       iATan <= iAngleMap[ptInst->iMapIdx][i][1]) {
                        iRet = iAngleMap[ptInst->iMapIdx][i][2];
                        break;
                    }
                }
                //return iRet;
            }

        } else {
            if ((ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO) > 0)
                iRet = GESTURE_EVENT_RIGHT;
            else if ((ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO) < 0)
                iRet = GESTURE_EVENT_LEFT;
        }
    } else if (ptInst->iMaxCorr[0] > currGesture_opt.GESTURE_CORR_TH) {
        if ((ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO) > 0)
            iRet = GESTURE_EVENT_UP;
        if ((ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO) < 0)
            iRet = GESTURE_EVENT_DOWN;
    }
#else
    if (ptInst->iMaxPhase[1] != currGesture_opt.GESTURE_PHASE_ZERO ||
       ptInst->iMaxPhase[0] != currGesture_opt.GESTURE_PHASE_ZERO) {
        //dATan = atan2(ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO, ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO);
        iATan = iATan2(ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO, ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO);
        ptInst->iAngle = iATan;

        iRet = GESTURE_EVENT_NA;
    } else {
        return GESTURE_EVENT_NF;
    }

    // Speed
    {
        int iS1, iS2;

        iS1 = ptInst->iMaxPhase[0] - currGesture_opt.GESTURE_PHASE_ZERO;
        iS2 = ptInst->iMaxPhase[1] - currGesture_opt.GESTURE_PHASE_ZERO;
        //ptInst->iSpeed  = currGesture_opt.GESTURE_PHASE_MAX - sqrt(iS1 * iS1 + iS2 * iS2);
    }

    for (i = 0; i < 9; i++) {
        if (iATan >= iAngleMap[ptInst->iMapIdx][i][0] &&
           iATan <= iAngleMap[ptInst->iMapIdx][i][1]) {
            iRet = iAngleMap[ptInst->iMapIdx][i][2];
            break;
        }
    }
#endif
    return iRet;
}

/******************************************************************************
  Near/Far
 ******************************************************************************/

void NFDet_Init(tfNFDet *ptInst)
{
    ptInst->iState    = NFDET_STATE_IDLE;
    ptInst->iCnt      = 0;
    ptInst->iLastVal  = -1;
    ptInst->iCT       = 0;
    ptInst->iThHold   = 400;
    ptInst->iThExit   = 150;
    ptInst->iZeroVal  = 0;
    ptInst->iDiff     = 0;
    ptInst->iRatio    = RATIO_1X;
    ptInst->iDebounce = 0;
}

int iNFDet_Do(tfNFDet *ptInst, int iPS)
{
    int iOutput;

    iOutput = NFDET_EVENT_NA;
    if (ptInst->iDebounce != 0) {
        ptInst->iDebounce --;
    } else if (ptInst->iState == NFDET_STATE_IDLE) {
        if (iPS >= (ptInst->iCT + ptInst->iThHold)) {
            ptInst->iState   = NFDET_STATE_HOLD_DET;
            ptInst->iZeroVal = iPS;
            ptInst->iCnt     = 0;
        }
    } else if (ptInst->iState == NFDET_STATE_HOLD_DET) {
        if (iPS < (ptInst->iCT + ptInst->iThHold)) {
            /*
            if (ptInst->iCnt > NFDET_BACK_CNT)
            {
                iOutput = NFDET_EVENT_BACK;
                ptInst->iDebounce = 48;
            }
            else
                iOutput = NFDET_EVENT_NA;
            */

            ptInst->iState = NFDET_STATE_IDLE;
            ptInst->iCnt   = 0;
        } else if (iPS < (ptInst->iZeroVal * NFDET_HOLD_HB / RATIO_1X) &&
                  iPS > (ptInst->iZeroVal * NFDET_HOLD_LB / RATIO_1X)) {
            if (ptInst->iCnt == NFDET_HOLD_CNT) {
                ptInst->iState = NFDET_STATE_HOLDING;
                ptInst->iCnt   = 0;
            } else {
                ptInst->iCnt ++;
            }
        }
        ptInst->iZeroVal = iPS;
    } else if (ptInst->iState == NFDET_STATE_HOLDING) {
        //if (iPS < (ptInst->iZeroVal * NFDET_HOLD_HB / RATIO_1X) &&
        //iPS > (ptInst->iZeroVal * NFDET_HOLD_LB / RATIO_1X))
        // Lex, 20140805
        if (iPS > (ptInst->iZeroVal * NFDET_HOLD_HB / RATIO_1X) ||
           iPS < (ptInst->iZeroVal * NFDET_HOLD_LB / RATIO_1X)) {
            ptInst->iState = NFDET_STATE_NEAR_FAR;
            ptInst->iCnt   = 0;
        }
        iOutput                 = NFDET_EVENT_HOLD;
    } else if (ptInst->iState == NFDET_STATE_NEAR_FAR) {
        if (iPS < (ptInst->iCT + ptInst->iThExit)) {
            if (ptInst->iCnt == NFDET_EXIT_CNT) {
                ptInst->iState = NFDET_STATE_IDLE;
                ptInst->iCnt   = 0;
                iOutput = NFDET_EVENT_EXIT;
            } else {
                ptInst->iCnt++;
                iOutput = NFDET_EVENT_TOO_FAR;
            }
        } else {
            ptInst->iCnt = 0;
            if (iPS < ptInst->iZeroVal) {
                iOutput = NFDET_EVENT_FAR;
            } else {
                iOutput = NFDET_EVENT_NEAR;
            }
        }
    }

    if (ptInst->iState == NFDET_STATE_NEAR_FAR) {
        ptInst->iDiff  = iPS - ptInst->iZeroVal;
        ptInst->iRatio = iPS * RATIO_1X / ptInst->iZeroVal;
    } else {
        ptInst->iDiff  = 0;
        ptInst->iRatio = RATIO_1X;
    }

    return iOutput;
}

/******************************************************************************
  APIs
 ******************************************************************************/
int stk3420_gesture_determine(u16 pd_val[], u32 *gesture)
{
    int iGesAct = GESTURE_EVENT_NA;
    //static int idx = 0;
    // int iNF = NFDET_EVENT_NA;
    //int iGesEvent = GESTURE_EVENT_NA;

    iGesAct = iGesture_Do(&xtGes, (int)pd_val[0], (int)pd_val[1], (int)pd_val[3], (int)pd_val[2]);
    /*
    if (idx == 50)
        xtNFDet.iCT = (pd_val[0] + pd_val[1] + pd_val[2] + pd_val[3]);
    else if (idx > 50)
        iNF = iNFDet_Do(&(xtNFDet), (int)pd_val[0] + pd_val[1] + pd_val[2] + pd_val[3]);
    else
        iNF = NFDET_EVENT_NA;
    if (idx < 1000)
        idx++;
    */

    /*
    	if (iNF != NFDET_EVENT_NA)
    	{
    		iGesEvent = iNF;
    	}
    	else if (iGesAct != GESTURE_EVENT_NA)
    	{
    		iGesEvent = iGesAct;
    	}
    */

    //iGesEvent = iGesAct;
    //*gesture = iGesEvent;
    *gesture = iGesAct;

    //return xtNFDet->iZeroVal;
    return 0;
}

int stk3420_gesture_func_init(void)
{
    xtGes.iSensorType = OPT_STK6020;
    Gesture_Init(&xtGes);
    //NFDet_Init(&xtNFDet);
    return 0;
}

void stk3420_gesture_get_MaxPhase(int *phase)
{
    phase[0] = xtGes.iMaxPhase[0];
    phase[1] = xtGes.iMaxPhase[1];
}

void stk3420_gesture_get_MaxCorr(int *correl)
{
    correl[0] = xtGes.iMaxCorr[0];
    correl[1] = xtGes.iMaxCorr[1];
}

void stk3420_gesture_set_sun(int als_lux)
{
    iGesture_MonitorS(&xtGes, als_lux);
}

MODULE_AUTHOR("Howrd <howrd@21cn.com>");
MODULE_DESCRIPTION("Sensortek STK3420 ALS/PS/GS Driver");
MODULE_LICENSE("GPL");