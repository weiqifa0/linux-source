/*
 * File: drivers/video/omap_new/debug.c
 *
 * Debug support for the omapfb driver
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 */

#ifndef __MTKFB_DEBUG_H
#define __MTKFB_DEBUG_H

void DBG_Init(void);
void DBG_Deinit(void);

void DBG_OnTriggerLcd(void);
void DBG_OnTeDelayDone(void);
void DBG_OnLcdDone(void);

#include "mmprofile.h"
#include "mmprofile_function.h"
extern struct MTKFB_MMP_Events_t {
	mmp_event MTKFB;
	mmp_event CreateSyncTimeline;
	mmp_event PanDisplay;
	mmp_event SetOverlayLayer;
	mmp_event SetOverlayLayers;
	mmp_event SetMultipleLayers;
	mmp_event CreateSyncFence;
	mmp_event IncSyncTimeline;
	mmp_event SignalSyncFence;
	mmp_event TrigOverlayOut;
	mmp_event UpdateScreenImpl;
	mmp_event VSync;
	mmp_event UpdateConfig;
	mmp_event ConfigOVL;
	mmp_event ConfigAAL;
	mmp_event ConfigMemOut;
	mmp_event ScreenUpdate;
	mmp_event CaptureFramebuffer;
	mmp_event RegUpdate;
	mmp_event EarlySuspend;
	mmp_event DispDone;
	mmp_event DSICmd;
	mmp_event DSIIRQ;
	mmp_event EsdCheck;
	mmp_event WaitVSync;
	mmp_event LayerDump;
	mmp_event Layer[4];
	mmp_event OvlDump;
	mmp_event FBDump;
	mmp_event DSIRead;
	mmp_event GetLayerInfo;
	mmp_event LayerInfo[4];
	mmp_event IOCtrl;
	mmp_event Debug;
} MTKFB_MMP_Events;

#ifdef MTKFB_DBG
#include "disp_drv_log.h"

#define DBG_BUF_SIZE		    2048
#define MAX_DBG_INDENT_LEVEL	5
#define DBG_INDENT_SIZE		    3
#define MAX_DBG_MESSAGES	    0

static int dbg_indent;
static int dbg_cnt;
static char dbg_buf[DBG_BUF_SIZE];
static spinlock_t dbg_spinlock = SPIN_LOCK_UNLOCKED;

#if 0 /*DVFS*/
extern unsigned int dvfs_test;
extern int primary_display_switch_mmsys_clk(int mmsys_clk_old, int mmsys_clk_new);
#endif

static inline void dbg_print(int level, const char *fmt, ...)
{
	if (level <= MTKFB_DBG) {
		if (!MAX_DBG_MESSAGES || dbg_cnt < MAX_DBG_MESSAGES) {
			va_list args;
			int ind = dbg_indent;
			unsigned long flags;

			spin_lock_irqsave(&dbg_spinlock, flags);
			dbg_cnt++;
			if (ind > MAX_DBG_INDENT_LEVEL)
				ind = MAX_DBG_INDENT_LEVEL;

			pr_info("DISP/DBG %*s", ind * DBG_INDENT_SIZE, "");
			va_start(args, fmt);
			vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, args);
			pr_info("DISP/DBG " dbg_buf);
			va_end(args);
			spin_unlock_irqrestore(&dbg_spinlock, flags);
		}
	}
}

#define DBGPRINT	dbg_print

#define DBGENTER(level)	do {					\
		dbg_print(level, "%s: Enter\n", __func__);	\
		dbg_indent++;					\
	} while (0)

#define DBGLEAVE(level)	do {					\
		dbg_indent--;					\
		dbg_print(level, "%s: Leave\n", __func__);	\
	} while (0)

/* Debug Macros */

#define MTKFB_DBG_EVT_NONE    0x00000000
#define MTKFB_DBG_EVT_FUNC    0x00000001	/* Function Entry     */
#define MTKFB_DBG_EVT_ARGU    0x00000002	/* Function Arguments */
#define MTKFB_DBG_EVT_INFO    0x00000003	/* Information        */

#define MTKFB_DBG_EVT_MASK    (MTKFB_DBG_EVT_NONE)

#define MSG(evt, fmt, args...)					\
	do {							\
		if ((MTKFB_DBG_EVT_##evt) & MTKFB_DBG_EVT_MASK)	\
			pr_info("DISP/DBG " fmt, ##args);	\
	} while (0)

#define MSG_FUNC_ENTER(f)   MSG(FUNC, "<FB_ENTER>: %s\n", __func__)
#define MSG_FUNC_LEAVE(f)   MSG(FUNC, "<FB_LEAVE>: %s\n", __func__)


#else				/* MTKFB_DBG */

#define DBGPRINT(level, format, ...)
#define DBGENTER(level)
#define DBGLEAVE(level)

/* Debug Macros */

#define MSG(evt, fmt, args...)
#define MSG_FUNC_ENTER()
#define MSG_FUNC_LEAVE()
void _debug_pattern(unsigned long mva, unsigned long va, unsigned int w, unsigned int h,
		    unsigned int linepitch, unsigned int color, unsigned int layerid,
		    unsigned int bufidx);
#ifdef CONFIG_MTK_M4U
void _debug_fps_meter(unsigned long mva, unsigned long va, unsigned int w, unsigned int h,
		      unsigned int linepitch, unsigned int color, unsigned int layerid,
		      unsigned int bufidx);
#endif

bool get_ovl1_to_mem_on(void);

#endif				/* MTKFB_DBG */

#endif				/* __MTKFB_DEBUG_H */
