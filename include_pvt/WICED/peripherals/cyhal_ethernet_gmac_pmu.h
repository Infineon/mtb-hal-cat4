/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * HND SiliconBackplane PMU support.
 *
 * $Id: cyhal_ethernet_gmac_pmu.h 456199 2014-02-18 03:06:41Z sudhirbs $
 */

#pragma once

#if !defined(BCMDONGLEHOST)
#define SET_LDO_VOLTAGE_LDO1		1
#define SET_LDO_VOLTAGE_LDO2		2
#define SET_LDO_VOLTAGE_LDO3		3
#define SET_LDO_VOLTAGE_PAREF		4
#define SET_LDO_VOLTAGE_CLDO_PWM	5
#define SET_LDO_VOLTAGE_CLDO_BURST	6
#define SET_LDO_VOLTAGE_CBUCK_PWM	7
#define SET_LDO_VOLTAGE_CBUCK_BURST	8
#define SET_LDO_VOLTAGE_LNLDO1		9
#define SET_LDO_VOLTAGE_LNLDO2_SEL	10
#define SET_LNLDO_PWERUP_LATCH_CTRL	11

#define BBPLL_NDIV_FRAC_BITS		24
#define P1_DIV_SCALE_BITS			12

#define PMUREQTIMER (1 << 0)

extern void _cyhal_gmac_gmac_si_pmu_init(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_gmac_si_pmu_chip_init(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_gmac_si_pmu_pll_init(_cyhal_gmac_si_t *sih, uint32 xtalfreq);
extern bool _cyhal_gmac_gmac_si_pmu_is_autoresetphyclk_disabled(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_gmac_si_pmu_res_init(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_si_pmu_swreg_init(_cyhal_gmac_si_t *sih);
extern uint32 _cyhal_gmac_si_pmu_force_ilp(_cyhal_gmac_si_t *sih, bool force);
extern void _cyhal_gmac_si_pmu_res_minmax_update(_cyhal_gmac_si_t *sih);

extern uint32 _cyhal_gmac_si_pmu_si_clock(_cyhal_gmac_si_t *sih);   /* returns [Hz] units */
extern uint32 _cyhal_gmac_si_pmu_cpu_clock(_cyhal_gmac_si_t *sih);  /* returns [hz] units */
extern uint32 _cyhal_gmac_si_pmu_mem_clock(_cyhal_gmac_si_t *sih);  /* returns [Hz] units */
extern uint32 _cyhal_gmac_si_pmu_alp_clock(_cyhal_gmac_si_t *sih);  /* returns [Hz] units */
extern void _cyhal_gmac_si_pmu_ilp_clock_set(uint32 cycles);
extern uint32 _cyhal_gmac_si_pmu_ilp_clock(_cyhal_gmac_si_t *sih);  /* returns [Hz] units */

extern void _cyhal_gmac_si_pmu_set_switcher_voltage(_cyhal_gmac_si_t *sih, uint8 bb_voltage, uint8 rf_voltage);
extern void _cyhal_gmac_si_pmu_set_ldo_voltage(_cyhal_gmac_si_t *sih, uint8 ldo, uint8 voltage);
extern void _cyhal_gmac_si_pmu_paref_ldo_enable(_cyhal_gmac_si_t *sih, bool enable);
extern uint16 _cyhal_gmac_si_pmu_fast_pwrup_delay(_cyhal_gmac_si_t *sih);
extern uint si_pll_minresmask_reset(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_si_pmu_rcal(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_si_pmu_spuravoid(_cyhal_gmac_si_t *sih, uint8 spuravoid);
/* below function are only for BBPLL parallel purpose */
extern void _cyhal_gmac_si_pmu_spuravoid_isdone(_cyhal_gmac_si_t *sih, uint32 min_res_mask,
uint32 max_res_mask, uint32 clk_ctl_st, uint8 spuravoid);
extern void _cyhal_gmac_si_pmu_pll_off_PARR(_cyhal_gmac_si_t *sih, uint32 *min_res_mask,
uint32 *max_res_mask, uint32 *clk_ctl_st);
/* below function are only for BBPLL parallel purpose */
extern void _cyhal_gmac_si_pmu_gband_spurwar(_cyhal_gmac_si_t *sih);
extern uint32 _cyhal_gmac_si_pmu_cal_fvco(_cyhal_gmac_si_t *sih);

extern bool _cyhal_gmac_si_pmu_is_otp_powered(_cyhal_gmac_si_t *sih);
extern uint32 _cyhal_gmac_si_pmu_measure_alpclk(_cyhal_gmac_si_t *sih);

extern uint32 _cyhal_gmac_si_pmu_chipcontrol(_cyhal_gmac_si_t *sih, uint reg, uint32 mask, uint32 val);
extern uint32 _cyhal_gmac_si_pmu_regcontrol(_cyhal_gmac_si_t *sih, uint reg, uint32 mask, uint32 val);
extern uint32 _cyhal_gmac_si_pmu_pllcontrol(_cyhal_gmac_si_t *sih, uint reg, uint32 mask, uint32 val);
extern void _cyhal_gmac_si_pmu_pllupd(_cyhal_gmac_si_t *sih);
extern bool _cyhal_gmac_si_pmu_is_sprom_enabled(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_si_pmu_sprom_enable(_cyhal_gmac_si_t *sih, bool enable);

extern void _cyhal_gmac_si_pmu_radio_enable(_cyhal_gmac_si_t *sih, bool enable);
extern uint32 _cyhal_gmac_si_pmu_waitforclk_on_backplane(_cyhal_gmac_si_t *sih, uint32 clk, uint32 delay);
extern void _cyhal_gmac_si_pmu_set_4330_plldivs(_cyhal_gmac_si_t *sih, uint8 dacrate);
extern void _cyhal_gmac_si_pmu_pllreset(_cyhal_gmac_si_t *sih);
extern uint32 _cyhal_gmac_si_pmu_get_bb_vcofreq(_cyhal_gmac_si_t *sih, int xtalfreq);
typedef void (*_cyhal_gmac_si_pmu_callback_t)(void* arg);

extern uint32 si_mac_clk(_cyhal_gmac_si_t *sih);
extern void _cyhal_gmac_si_pmu_switch_on_PARLDO(_cyhal_gmac_si_t *sih);
extern int _cyhal_gmac_si_pmu_fvco_pllreg(_cyhal_gmac_si_t *sih, uint32 *fvco, uint32 *pllreg);

#endif /* !defined(BCMDONGLEHOST) */

extern void _cyhal_gmac_si_pmu_otp_power(_cyhal_gmac_si_t *sih, bool on, uint32* min_res_mask);
extern void si_sdiod_drive_strength_init(_cyhal_gmac_si_t *sih, uint32 drivestrength);

extern void _cyhal_gmac_si_pmu_minresmask_htavail_set(_cyhal_gmac_si_t *sih, bool set_clear);
extern void _cyhal_gmac_si_pmu_slow_clk_reinit(_cyhal_gmac_si_t *sih);

