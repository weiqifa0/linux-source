/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/of_fdt.h>
#include <linux/lockdep.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

/* TODO: re-enable wdt later */
/* #include <mach/wd_api.h> */
#include <mt-plat/mtk_cirq.h>

#include "mtk_spm_idle.h"
#include "mtk_spm_pcm.h"
#include "mtk_cpuidle.h"

#include "mtk_spm_internal.h"

/*
 * only for internal debug
 */
#define DPIDLE_TAG     "[DP] "
#define dpidle_dbg(fmt, args...)	pr_debug(DPIDLE_TAG fmt, ##args)

#define SPM_PWAKE_EN            0
#define SPM_BYPASS_SYSPWREQ     0

#define WAKE_SRC_FOR_DPIDLE \
	(WAKE_SRC_KP | WAKE_SRC_GPT | WAKE_SRC_EINT | WAKE_SRC_CONN_WDT| \
	WAKE_SRC_CONN2AP | WAKE_SRC_USB_CD | WAKE_SRC_USB_PDN | \
	WAKE_SRC_AFE | WAKE_SRC_SYSPWREQ)

#define I2C_CHANNEL 2

#define spm_is_wakesrc_invalid(wakesrc)     (!!((u32)(wakesrc) & 0xc0003803))

#ifdef CONFIG_MTK_RAM_CONSOLE
#define SPM_AEE_RR_REC 1
#else
#define SPM_AEE_RR_REC 0
#endif
#define SPM_USE_TWAM_DEBUG	0

#define	DPIDLE_LOG_PRINT_TIMEOUT_CRITERIA	20
#define	DPIDLE_LOG_DISCARD_CRITERIA			5000	/* ms */

#if SPM_AEE_RR_REC
enum spm_deepidle_step {
	SPM_DEEPIDLE_ENTER = 0,
	SPM_DEEPIDLE_ENTER_UART_SLEEP,
	SPM_DEEPIDLE_ENTER_WFI,
	SPM_DEEPIDLE_LEAVE_WFI,
	SPM_DEEPIDLE_ENTER_UART_AWAKE,
	SPM_DEEPIDLE_LEAVE
};
#endif

static struct pwr_ctrl dpidle_ctrl = {
	.wake_src = WAKE_SRC_FOR_DPIDLE,
	.r0_ctrl_en = 1,
	.r7_ctrl_en = 1,
	.infra_dcm_lock = 1,
	.wfi_op = WFI_OP_AND,
	.ca15_wfi0_en = 1,
	.ca15_wfi1_en = 1,
	.ca15_wfi2_en = 1,
	.ca15_wfi3_en = 1,
	.ca7_wfi0_en = 1,
	.ca7_wfi1_en = 1,
	.ca7_wfi2_en = 1,
	.ca7_wfi3_en = 1,
	.disp0_req_mask = 1,
	.disp1_req_mask = 1,
	.mfg_req_mask = 1,
#if SPM_BYPASS_SYSPWREQ
	.syspwreq_mask = 1,
#endif
};

static unsigned int dpidle_log_discard_cnt;
static unsigned int dpidle_log_print_prev_time;

struct spm_lp_scen __spm_dpidle = {
	.pcmdesc = &suspend_pcm,
	.pwrctrl = &dpidle_ctrl,
};

int __attribute__ ((weak)) request_uart_to_sleep(void)
{
	return 0;
}

int __attribute__ ((weak)) request_uart_to_wakeup(void)
{
	return 0;
}

#if SPM_AEE_RR_REC
void __attribute__ ((weak)) aee_rr_rec_deepidle_val(u32 val)
{
}

u32 __attribute__ ((weak)) aee_rr_curr_deepidle_val(void)
{
	return 0;
}
#endif

void __attribute__ ((weak)) mt_cirq_clone_gic(void)
{
}

void __attribute__ ((weak)) mt_cirq_enable(void)
{
}

void __attribute__ ((weak)) mt_cirq_flush(void)
{
}

void __attribute__ ((weak)) mt_cirq_disable(void)
{
}

u32 __attribute__ ((weak)) spm_get_sleep_wakesrc(void)
{
	return 0;
}

#if 0
void __attribute__ ((weak)) mt_cpufreq_set_pmic_phase(enum pmic_wrap_phase_id phase)
{
}
#endif

static long int idle_get_current_time_ms(void)
{
	struct timeval t;

	do_gettimeofday(&t);
	return ((t.tv_sec & 0xFFF) * 1000000 + t.tv_usec) / 1000;
}

static void spm_trigger_wfi_for_dpidle(struct pwr_ctrl *pwrctrl)
{
	if (is_cpu_pdn(pwrctrl->pcm_flags)) {
		mt_cpu_dormant(CPU_DEEPIDLE_MODE);
	} else {
		/*
		 * MT6735/MT6735M: Mp0_axi_config[4] is one by default. No need to program it before entering deepidle.
		 * MT6753:         Have to program it before entering deepidle.
		 */
		wfi_with_sync();
	}
}

void spm_dpidle_set_sramrom_on(bool on)
{
	if (on)
		__spm_dpidle.pwrctrl->param1 |= PWR_CTRL_PARAM1_SRAMROM;
	else
		__spm_dpidle.pwrctrl->param1 &= ~PWR_CTRL_PARAM1_SRAMROM;
}

static bool spm_dpidle_is_sramrom_on(void)
{
	if (__spm_dpidle.pwrctrl->param1 & PWR_CTRL_PARAM1_SRAMROM)
		return true;
	else
		return false;
}

void spm_dpidle_set_pll_ldo_on(bool on)
{
	if (on)
		__spm_dpidle.pwrctrl->param1 |= PWR_CTRL_PARAM1_PLL_LDO;
	else
		__spm_dpidle.pwrctrl->param1 &= ~PWR_CTRL_PARAM1_PLL_LDO;
}

static bool spm_dpidle_is_pll_ldo_on(void)
{
	if (__spm_dpidle.pwrctrl->param1 & PWR_CTRL_PARAM1_PLL_LDO)
		return true;
	else
		return false;
}

/*
 * wakesrc: WAKE_SRC_XXX
 * enable : enable or disable @wakesrc
 * replace: if true, will replace the default setting
 */
int spm_set_dpidle_wakesrc(u32 wakesrc, bool enable, bool replace)
{
	unsigned long flags;

	if (spm_is_wakesrc_invalid(wakesrc))
		return -EINVAL;

	spin_lock_irqsave(&__spm_lock, flags);
	if (enable) {
		if (replace)
			__spm_dpidle.pwrctrl->wake_src = wakesrc;
		else
			__spm_dpidle.pwrctrl->wake_src |= wakesrc;
	} else {
		if (replace)
			__spm_dpidle.pwrctrl->wake_src = 0;
		else
			__spm_dpidle.pwrctrl->wake_src &= ~wakesrc;
	}
	spin_unlock_irqrestore(&__spm_lock, flags);

	return 0;
}

static unsigned int spm_output_wake_reason(struct wake_status *wakesta, struct pcm_desc *pcmdesc, u32 dump_log)
{
	unsigned int wr = WR_NONE;
	unsigned long int dpidle_log_print_curr_time = 0;
	bool log_print = false;
	static bool timer_out_too_short;

	if (dump_log == DEEPIDLE_LOG_FULL) {
		wr = __spm_output_wake_reason(wakesta, pcmdesc, false);
	} else if (dump_log == DEEPIDLE_LOG_REDUCED) {
		/* Determine print SPM log or not */
		dpidle_log_print_curr_time = idle_get_current_time_ms();

		if (wakesta->assert_pc != 0)
			log_print = true;
		else if ((dpidle_log_print_curr_time - dpidle_log_print_prev_time) >
			 DPIDLE_LOG_DISCARD_CRITERIA)
			log_print = true;

		if (wakesta->timer_out <= DPIDLE_LOG_PRINT_TIMEOUT_CRITERIA)
			timer_out_too_short = true;

		/* Print SPM log */
		if (log_print == true) {
			dpidle_dbg("dpidle_log_discard_cnt = %d, timer_out_too_short = %d\n",
						dpidle_log_discard_cnt,
						timer_out_too_short);
			wr = __spm_output_wake_reason(wakesta, pcmdesc, false);

			dpidle_log_print_prev_time = dpidle_log_print_curr_time;
			dpidle_log_discard_cnt = 0;
			timer_out_too_short = false;
		} else {
			dpidle_log_discard_cnt++;

			wr = WR_NONE;
		}
	}

	return wr;
}

static void spm_dpidle_pre_process(void)
{
	/* set PMIC WRAP table for deepidle power control */
	__spm_set_pmic_phase(PMIC_WRAP_PHASE_DEEPIDLE);
}

static void spm_dpidle_post_process(void)
{
	/* set PMIC WRAP table for normal power control */
/*	__spm_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL); */
}

unsigned int spm_go_to_dpidle(u32 spm_flags, u32 spm_data, u32 dump_log)
{
	struct wake_status wakesta;
	unsigned long flags;
	struct mtk_irq_mask mask;
	struct irq_desc *desc = irq_to_desc(spm_irq_0);
	unsigned int wr = WR_NONE;
	struct pcm_desc *pcmdesc = __spm_dpidle.pcmdesc;
	struct pwr_ctrl *pwrctrl = __spm_dpidle.pwrctrl;

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(1 << SPM_DEEPIDLE_ENTER);
#endif

	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

#ifndef CONFIG_MTK_PMIC_WRAP
	pwrctrl->pcm_flags |= SPM_NO_PMIC_WRAP;
#endif

	if (spm_dpidle_is_sramrom_on())
		pwrctrl->pcm_flags |= SPM_SRAMROM_PDN_DIS | SPM_26M_DIS;

	if (spm_dpidle_is_pll_ldo_on())
		pwrctrl->pcm_flags |= SPM_PLLGP_OFF_DIS | SPM_26M_DIS;

	spm_dpidle_before_wfi();

	lockdep_off();
	spin_lock_irqsave(&__spm_lock, flags);

	mt_irq_mask_all(&mask);
	if (desc)
		unmask_irq(desc);
	mt_cirq_clone_gic();
	mt_cirq_enable();

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val() | (1 << SPM_DEEPIDLE_ENTER_UART_SLEEP));
#endif

#if CONFIG_SUPPORT_PCM_ALLINONE
	if (!__spm_is_pcm_loaded())
		__spm_init_pcm_AllInOne(pcmdesc);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	spm_dpidle_pre_process();

	__spm_kick_pcm_to_run(pwrctrl);

	__spm_set_pcm_cmd(PCM_CMD_SUSPEND_PCM);
#else
	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	spm_dpidle_pre_process();

	__spm_kick_pcm_to_run(pwrctrl);
#endif

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val() | (1 << SPM_DEEPIDLE_ENTER_WFI));
#endif

#ifdef SPM_DEEPIDLE_PROFILE_TIME
	gpt_get_cnt(SPM_PROFILE_APXGPT, &dpidle_profile[1]);
#endif
	spm_trigger_wfi_for_dpidle(pwrctrl);

#ifdef SPM_DEEPIDLE_PROFILE_TIME
	gpt_get_cnt(SPM_PROFILE_APXGPT, &dpidle_profile[2]);
#endif

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val() | (1 << SPM_DEEPIDLE_LEAVE_WFI));
#endif

	spm_dpidle_post_process();

	__spm_get_wakeup_status(&wakesta);

	__spm_clean_after_wakeup();

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(aee_rr_curr_deepidle_val() | (1 << SPM_DEEPIDLE_ENTER_UART_AWAKE));
#endif

	wr = spm_output_wake_reason(&wakesta, pcmdesc, dump_log);

	mt_cirq_flush();
	mt_cirq_disable();

	mt_irq_mask_restore(&mask);

	spin_unlock_irqrestore(&__spm_lock, flags);
	lockdep_on();
	spm_dpidle_after_wfi();

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(0);
#endif
	return wr;
}

/*
 * cpu_pdn:
 *    true  = CPU dormant
 *    false = CPU standby
 * pwrlevel:
 *    0 = AXI is off
 *    1 = AXI is 26M
 * pwake_time:
 *    >= 0  = specific wakeup period
 */
unsigned int spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data)
{
	u32 sec = 0;
/*	int wd_ret; */
	struct wake_status wakesta;
	unsigned long flags;
	struct mtk_irq_mask mask;
	struct irq_desc *desc = irq_to_desc(spm_irq_0);
/*	struct wd_api *wd_api; */
	static unsigned int last_wr = WR_NONE;
	struct pcm_desc *pcmdesc = __spm_dpidle.pcmdesc;
	struct pwr_ctrl *pwrctrl = __spm_dpidle.pwrctrl;

	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

#if SPM_PWAKE_EN
	sec = spm_get_wake_period(-1 /* FIXME */, last_wr);
#endif
	pwrctrl->timer_val = sec * 32768;

	pwrctrl->wake_src = spm_get_sleep_wakesrc();

#if 0
	wd_ret = get_wd_api(&wd_api);
	if (!wd_ret)
		wd_api->wd_suspend_notify();
#endif
	spin_lock_irqsave(&__spm_lock, flags);

	mt_irq_mask_all(&mask);
	if (desc)
		unmask_irq(desc);
	mt_cirq_clone_gic();
	mt_cirq_enable();

	/* set PMIC WRAP table for deepidle power control */
/*	mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_DEEPIDLE); */

	spm_crit2("sleep_deepidle, sec = %u, wakesrc = 0x%x [%u]\n",
		  sec, pwrctrl->wake_src, is_cpu_pdn(pwrctrl->pcm_flags));

#if CONFIG_SUPPORT_PCM_ALLINONE
	if (!__spm_is_pcm_loaded())
		__spm_init_pcm_AllInOne(pcmdesc);

	if (request_uart_to_sleep()) {
		last_wr = WR_UART_BUSY;
		goto RESTORE_IRQ;
	}

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	__spm_kick_pcm_to_run(pwrctrl);

	spm_dpidle_pre_process();

	__spm_set_pcm_cmd(PCM_CMD_SUSPEND_PCM);
#else
	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	if (request_uart_to_sleep()) {
		last_wr = WR_UART_BUSY;
		goto RESTORE_IRQ;
	}

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	__spm_kick_pcm_to_run(pwrctrl);

	spm_dpidle_pre_process();
#endif

	spm_trigger_wfi_for_dpidle(pwrctrl);

	spm_dpidle_post_process();

	__spm_get_wakeup_status(&wakesta);

	__spm_clean_after_wakeup();

	request_uart_to_wakeup();

	last_wr = __spm_output_wake_reason(&wakesta, pcmdesc, true);

RESTORE_IRQ:

	/* set PMIC WRAP table for normal power control */
/*	mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL); */

	mt_cirq_flush();
	mt_cirq_disable();

	mt_irq_mask_restore(&mask);
	spin_unlock_irqrestore(&__spm_lock, flags);
#if 0
	if (!wd_ret)
		wd_api->wd_resume_notify();
#endif
	return last_wr;
}

#if SPM_USE_TWAM_DEBUG
#define SPM_TWAM_MONITOR_TICK 333333


static void twam_handler(struct twam_sig *twamsig)
{
	spm_crit("sig_high = %u%%  %u%%  %u%%  %u%%, r13 = 0x%x\n",
		 get_percent(twamsig->sig0, SPM_TWAM_MONITOR_TICK),
		 get_percent(twamsig->sig1, SPM_TWAM_MONITOR_TICK),
		 get_percent(twamsig->sig2, SPM_TWAM_MONITOR_TICK),
		 get_percent(twamsig->sig3, SPM_TWAM_MONITOR_TICK), spm_read(SPM_PCM_REG13_DATA));
}
#endif

void spm_deepidle_init(void)
{
#if SPM_USE_TWAM_DEBUG
	unsigned long flags;
	struct twam_sig twamsig = {
		.sig0 = 26,	/* md1_srcclkena */
		.sig1 = 22,	/* md_apsrc_req_mux */
		.sig2 = 25,	/* md2_srcclkena */
		.sig3 = 21,	/* md2_apsrc_req_mux */
	};

	spm_twam_register_handler(twam_handler);
	spm_twam_enable_monitor(&twamsig, false, SPM_TWAM_MONITOR_TICK);
#endif
}

MODULE_DESCRIPTION("SPM-DPIdle Driver v0.1");
