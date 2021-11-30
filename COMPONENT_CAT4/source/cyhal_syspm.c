/***************************************************************************//**
* \file cyhal_syspm.c
*
* Description:
* Provides a high level interface for interacting with the Power Management.
*
********************************************************************************
* \copyright
* Copyright 2021 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* float_t types conflict with the same typedefs from the standard ANSI-C
 * math.h header file. Don't re-typedef them in typedefs.h header file.
 */
#define TYPEDEF_FLOAT_T
#include <math.h>
#include "typedefs.h"
#include "wiced_osl.h"
#undef TYPEDEF_FLOAT_T

#include <limits.h>
#include <math.h>
#include "cyhal_lptimer.h"
#include "cyhal_timer.h"
#include "cyhal_syspm.h"
#include "cyhal_system.h"
#include "cyhal_clock_impl.h"
#include "cy_utils.h"
#include "platform_appscr4.h"
#include "cr4.h"

#include "sbchipc.h"
#include "aidmp.h"
#include "hndsoc.h"

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
*       Internal - Cores powersave Apps functions
*******************************************************************************/

typedef struct
{
    uint32_t        core_addr;
    uint32_t        wrapper_addr;
    bool            is_enabled;
} _cyhal_syspm_cores_powersave_clock_gate_core_t;

static void _cyhal_syspm_cores_powersave_shutoff_usb20d(void)
{
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_USBPHY_MODE_OVR_VAL_REG, GCI_CHIPCONTROL_USBPHY_MODE_OVR_VAL_MASK, GCI_CHIPCONTROL_USBPHY_MODE_OVR_VAL_USB);
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_USBPHY_MODE_OVR_EN_REG, GCI_CHIPCONTROL_USBPHY_MODE_OVR_EN_MASK, GCI_CHIPCONTROL_USBPHY_MODE_OVR_EN_SET);

    osl_wrapper_enable((void*)PLATFORM_USB20D_MASTER_WRAPPER_REGBASE(0x0));
    _cyhal_system_common_chipcontrol((void*)PLATFORM_USB20D_PHY_UTMI_CTL1_REG, PLATFORM_USB20D_PHY_UTMI1_CTL_PHY_SHUTOFF_MASK, PLATFORM_USB20D_PHY_UTMI1_CTL_PHY_SHUTOFF_DISABLE);
    cyhal_system_delay_us(50);
    _cyhal_system_common_chipcontrol((void*)PLATFORM_USB20D_PHY_UTMI_CTL1_REG, PLATFORM_USB20D_PHY_UTMI1_CTL_PHY_SHUTOFF_MASK, PLATFORM_USB20D_PHY_UTMI1_CTL_PHY_SHUTOFF_ENABLE);
    osl_wrapper_disable((void*)PLATFORM_USB20D_MASTER_WRAPPER_REGBASE(0x0));

    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_USBPHY_MODE_OVR_EN_REG, GCI_CHIPCONTROL_USBPHY_MODE_OVR_EN_MASK, 0x0);
}

static void _cyhal_syspm_cores_powersave_enable_memory_clock_gating(void)
{
    PLATFORM_SOCSRAM_POWERCONTROL_REG->bits.enable_mem_clk_gate = 1;
}

static void _cyhal_syspm_cores_powersave_clock_gate_core(const _cyhal_syspm_cores_powersave_clock_gate_core_t* core)
{
    /*
     * The clock gating only works when the core is out of reset.
     * But when the core is out of reset it always requests HT.
     * Let's bring core out of reset (so that clock gating works and the
     * clock tree within the core is not running) but also write to ClockControlStatus
     * to force off the HT request.
     * This reduce power consumption as long as core is not used.
     * When driver is going to use core it resets core and cancel changes done here.
     */

    volatile aidmp_t* dmp                        = (aidmp_t*)core->wrapper_addr;
    volatile clock_control_status_t* core_status = PLATFORM_CLOCKSTATUS_REG(core->core_addr);
    uint32_t dummy;
    CY_UNUSED_PARAMETER(dummy);

    dmp->ioctrl    = SICF_FGC | SICF_CLOCK_EN; /* turn on clocks */
    dmp->resetctrl = AIRC_RESET;               /* turn on reset */
    dummy = dmp->resetctrl;                    /* read back to ensure write propagated */
    cyhal_system_delay_us(1);

    dmp->resetctrl = 0;                        /* turn off reset */
    dummy = dmp->resetctrl;                    /* read back to ensure write propagated */
    cyhal_system_delay_us(1);

    core_status->bits.force_hw_clock_req_off = 1; /* assert ForceHWClockReqOff */
    dummy = core_status->raw;                     /* read back to ensure write propagated */
    cyhal_system_delay_us(1);

    CY_ASSERT(dummy != 0);
    dmp->ioctrl = SICF_CLOCK_EN; /* turn off force clock */
    dmp->ioctrl = 0;             /* turn off clock */
}

static void _cyhal_syspm_cores_powersave_init_apps_domain(void)
{
    const _cyhal_syspm_cores_powersave_clock_gate_core_t cores[] =
    {
        { .core_addr = PLATFORM_GMAC_REGBASE(0x0), .wrapper_addr = PLATFORM_GMAC_MASTER_WRAPPER_REGBASE(0x0), true },
        { .core_addr = PLATFORM_I2S0_REGBASE(0x0), .wrapper_addr = PLATFORM_I2S0_MASTER_WRAPPER_REGBASE(0x0), true },
        { .core_addr = PLATFORM_I2S1_REGBASE(0x0), .wrapper_addr = PLATFORM_I2S1_MASTER_WRAPPER_REGBASE(0x0), true }
    };

    // Clock gate peripherals
    for (unsigned i = 0; i < ARRAYSIZE(cores); ++i)
    {
        if (cores[i].is_enabled == true)
        {
            _cyhal_syspm_cores_powersave_clock_gate_core(&cores[i]);
        }
    }

    // Shutoff USB PHY using USB20D core
    _cyhal_syspm_cores_powersave_shutoff_usb20d();
    // Enable memory clock gate
    _cyhal_syspm_cores_powersave_enable_memory_clock_gating();
}

static void _cyhal_syspm_cores_powersave_disable_wl_reg_on_pulldown(bool disable)
{
    _cyhal_system_pmu_regulatorcontrol(PMU_REGULATOR_WL_REG_ON_PULLDOWN_REG, PMU_REGULATOR_WL_REG_ON_PULLDOWN_MASK,
                                   disable ? PMU_REGULATOR_WL_REG_ON_PULLDOWN_DIS : PMU_REGULATOR_WL_REG_ON_PULLDOWN_EN);
}


/*******************************************************************************
*       Internal - Cores powersave WLAN functions
*******************************************************************************/

#define _CYHAL_CORES_POWERSAVE_WLAN_RESOURCES          \
    (PMU_RES_MASK(PMU_RES_SR_CLK_START)              | \
      PMU_RES_MASK(PMU_RES_WL_CORE_READY)            | \
      PMU_RES_MASK(PMU_RES_WL_CORE_READY_BUF)        | \
      PMU_RES_MASK(PMU_RES_MINI_PMU)                 | \
      PMU_RES_MASK(PMU_RES_RADIO_PU)                 | \
      PMU_RES_MASK(PMU_RES_SR_CLK_STABLE)            | \
      PMU_RES_MASK(PMU_RES_SR_SAVE_RESTORE)          | \
      PMU_RES_MASK(PMU_RES_SR_VDDM_PWRSW)            | \
      PMU_RES_MASK(PMU_RES_SR_SUBCORE_AND_PHY_PWRSW) | \
      PMU_RES_MASK(PMU_RES_SR_SLEEP)                 | \
      PMU_RES_MASK(PMU_RES_MAC_PHY_CLK_AVAIL))

static void _cyhal_syspm_cores_powersave_down_core(const uint32_t wrapper_addr)
{
    volatile aidmp_t* dmp = (aidmp_t*)wrapper_addr;
    dmp->resetctrl = AIRC_RESET; /* turn on reset */
    dmp->ioctrl    = 0;          /* turn off clocks */

    uint32_t dummy = dmp->resetctrl;      /* read back to ensure write propagated */
    CY_UNUSED_PARAMETER(dummy);
    cyhal_system_delay_us(1);
    CY_ASSERT(dummy != 0);
}

static void _cyhal_syspm_cores_powersave_down_all_wlan_cores(void)
{
    const uint32_t wlan_domain_wrappers[] =
    {
        PLATFORM_WLANCR4_MASTER_WRAPPER_REGBASE(0x0),
        PLATFORM_DOT11MAC_MASTER_WRAPPER_REGBASE(0x0)
    };

    /*
     * Pull up wlan resource before accessing WLAN domain and then revert
     */
    uint32_t min_res_orig = PLATFORM_PMU->min_res_mask;
    PLATFORM_PMU->min_res_mask = _CYHAL_CORES_POWERSAVE_WLAN_RESOURCES;
    for (unsigned i = 0; i < ARRAYSIZE(wlan_domain_wrappers); ++i)
    {
        _cyhal_syspm_cores_powersave_down_core(wlan_domain_wrappers[i]);
    }
    PLATFORM_PMU->min_res_mask = min_res_orig;
}


/*******************************************************************************
*       Internal - MCU powersave functions
*******************************************************************************/

static void _cyhal_syspm_mcu_powersave_set_mode(bool deep_sleep)
{
    /*
     * Force power switches on for non deep-sleep modes, otherwise clear forcing.
     * If forced on then APPS is not going into deep-sleep even if all resources are down and clocks switched off.
     */
    _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_APP_DIGITAL_POWER_FORCE_REG,
                              PMU_CHIPCONTROL_APP_DIGITAL_POWER_FORCE_MASK,
                              deep_sleep ? 0x0 : PMU_CHIPCONTROL_APP_DIGITAL_POWER_FORCE_EN);
    _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_APP_SOCSRAM_POWER_FORCE_REG,
                              PMU_CHIPCONTROL_APP_SOCSRAM_POWER_FORCE_MASK,
                              deep_sleep ? 0x0 : PMU_CHIPCONTROL_APP_SOCSRAM_POWER_FORCE_EN);
}

static void _cyhal_syspm_mcu_powersave_set_aon(bool on)
{
    /* Force app always-on memory on/off */
    uint32_t vddm_mask = on ? PMU_CHIPCONTROL_APP_VDDM_POWER_FORCE_EN : 0;

    _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_APP_VDDM_POWER_FORCE_REG,
                                (uint32_t)PMU_CHIPCONTROL_APP_VDDM_POWER_FORCE_MASK,
                                vddm_mask);
}

static void _cyhal_syspm_mcu_powersave_init(void)
{
    /* Define resource mask used to wake up application domain. */
    PLATFORM_PMU->res_req_mask1 = PMU_RES_APPS_UP_MASK;

    /* To reduce deep-sleep current reduce voltage used in deep-sleep. */
    _cyhal_system_pmu_regulatorcontrol(PMU_REGULATOR_LPLDO1_REG,
                                    PMU_REGULATOR_LPLDO1_MASK,
                                    PLATFORM_LPLDO_VOLTAGE);

    /*
    * Spread over the time power-switch up-delays.
    * This should reduce inrush current which may break digital logic functionality.
    */
    _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_APP_POWER_UP_DELAY_REG,
                                PMU_CHIPCONTROL_APP_POWER_UP_DELAY_DIGITAL_MASK |
                                PMU_CHIPCONTROL_APP_POWER_UP_DELAY_SOCSRAM_MASK |
                                PMU_CHIPCONTROL_APP_POWER_UP_DELAY_VDDM_MASK,
                                PMU_CHIPCONTROL_APP_POWER_UP_DELAY_DIGITAL_VAL(0xF) |
                                PMU_CHIPCONTROL_APP_POWER_UP_DELAY_SOCSRAM_VAL(0x1) |
                                PMU_CHIPCONTROL_APP_POWER_UP_DELAY_VDDM_VAL(0x1));

    /* Increase time-up to accomodate above change which spreads power-switch up-delays. */
    _cyhal_system_pmu_res_updown_time(PMU_RES_APP_DIGITAL_PWRSW,
                                    PMU_RES_UPDOWN_TIME_UP_MASK,
                                    PMU_RES_UPDOWN_TIME_UP_VAL(38));

    /* Increase VDDM pwrsw up time and make digital pwrsw to depend on VDDM pwrsw to reduce inrush current */
    _cyhal_system_pmu_res_updown_time(PMU_RES_APP_VDDM_PWRSW,
                                    PMU_RES_UPDOWN_TIME_UP_MASK,
                                    PMU_RES_UPDOWN_TIME_UP_VAL(12));
    _cyhal_system_pmu_res_dep_mask(PMU_RES_APP_DIGITAL_PWRSW,
                                PMU_RES_MASK(PMU_RES_APP_VDDM_PWRSW),
                                PMU_RES_MASK(PMU_RES_APP_VDDM_PWRSW));

    /* Force app always-on memory on. */
    _cyhal_syspm_mcu_powersave_set_aon(true);

    /*
    * Set deep-sleep flag.
    * It is reserved for software.
    * Does not trigger any hardware reaction.
    * Used by software during warm boot to know whether it should go normal boot path or warm boot.
    */
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_SW_DEEP_SLEEP_FLAG_REG,
                                GCI_CHIPCONTROL_SW_DEEP_SLEEP_FLAG_MASK,
                                GCI_CHIPCONTROL_SW_DEEP_SLEEP_FLAG_SET);

    /* Do not allow deep-sleep unless explicitly called */
    _cyhal_syspm_mcu_powersave_set_mode(false);
}


/*******************************************************************************
*       Internal - Hibernate functions
*******************************************************************************/

#define _CYHAL_SYSPM_HIB_WAKE_CTRL_REG_RCCODE -1
#define _CYHAL_SYSPM_HIBERNATE_CLOCK_EXTERNAL_DEFAULT_FREQ 32768

typedef enum
{
    _CYHAL_SYSPM_HIBERNATION_CLOCK_EXTERNAL,
    _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_128KHZ,
    _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_32KHZ,
    _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_16KHZ
} _cyhal_syspm_hibernation_clock_t;

typedef struct
{
    _cyhal_syspm_hibernation_clock_t clock;          /* Defines which clock - external or internal to use, and if internal then which frequency. */
    uint32_t                     hib_ext_clock_freq; /* Defines external clock frequency, if set 0 then use default configuration. */
    int32_t                      rc_code;            /* Defines RC parameter of internal clock (the higher value, the lower frequency). Has to be created by calibration algorithm and applied by firmware. Negative value means default. */
} _cyhal_syspm_hibernation_t;

static uint32_t _cyhal_syspm_hibernation_ext_clock_freq;

static const _cyhal_syspm_hibernation_t hibernation_config =
{
    .clock              = _CYHAL_SYSPM_HIBERNATION_CLOCK_EXTERNAL,
    .hib_ext_clock_freq = 0, /* use default settings */
    .rc_code            = _CYHAL_SYSPM_HIB_WAKE_CTRL_REG_RCCODE,
};

static cy_rslt_t _cyhal_syspm_hibernate_clock_power_up(const _cyhal_syspm_hibernation_t* hib)
{
    const bool external_clock = (hib->clock == _CYHAL_SYSPM_HIBERNATION_CLOCK_EXTERNAL);
    uint32_t wake_ctl_mask            = 0x0;
    uint32_t wake_ctl_val             = 0x0;

    if (!external_clock)
    {
        switch (hib->clock)
        {
            case _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_128KHZ:
                wake_ctl_val |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_FREQ_128KHZ;
                break;
            case _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_32KHZ:
                wake_ctl_val |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_FREQ_32KHZ;
                break;
            case _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_16KHZ:
                wake_ctl_val |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_FREQ_16KHZ;
                break;
            default:
                return CYHAL_SYSPM_RSLT_BAD_ARGUMENT;
        }

        wake_ctl_mask |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_FREQ_MASK;
    }

    if (hib->rc_code >= 0)
    {
        wake_ctl_val  |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_RC_VAL((uint32_t)hib->rc_code);
        wake_ctl_mask |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_RC_MASK;
    }

    if (wake_ctl_mask || wake_ctl_val)
    {
        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_CTL_REG, wake_ctl_mask, wake_ctl_val);
    }

    if (external_clock)
    {
        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_CTL_REG, GCI_CHIPCONTROL_HIB_WAKE_CTL_XTAL_DOWN_MASK, 0x0);
    }
    else
    {
        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_CTL_REG, GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_DOWN_MASK, 0x0);
    }

    return CY_RSLT_SUCCESS;
}

static uint32_t _cyhal_syspm_hibernation_get_raw_status(uint32_t selector)
{
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_READ_SEL_REG,
                              GCI_CHIPCONTROL_HIB_READ_SEL_MASK,
                              selector);

    return (_cyhal_system_gci_chipstatus(GCI_CHIPSTATUS_HIB_READ_REG) & GCI_CHIPSTATUS_HIB_READ_MASK) >> GCI_CHIPSTATUS_HIB_READ_SHIFT;
}

static hib_status_t _cyhal_syspm_hibernation_get_status(void)
{
    hib_status_t status = { .raw = _cyhal_syspm_hibernation_get_raw_status(GCI_CHIPCONTROL_HIB_READ_SEL_STATUS) };
    return status;
}

static cy_rslt_t _cyhal_syspm_hibernation_clock_init(const _cyhal_syspm_hibernation_t* hib, uint32_t* hib_ext_clock_freq)
{
    const bool      external_clock  = (hib->clock == _CYHAL_SYSPM_HIBERNATION_CLOCK_EXTERNAL);
    const cy_rslt_t power_up_result = _cyhal_syspm_hibernate_clock_power_up(hib);

    if (power_up_result != CY_RSLT_SUCCESS)
    {
        return power_up_result;
    }

    if (external_clock)
    {
        /* If you are hung here, check whether your platform supports an external clock! */
        while (_cyhal_syspm_hibernation_get_status().bits.pmu_ext_lpo_avail == 0);

        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_FORCE_EXT_LPO_REG, GCI_CHIPCONTROL_HIB_FORCE_EXT_LPO_MASK, GCI_CHIPCONTROL_HIB_FORCE_EXT_LPO_EXEC);
        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_FORCE_INT_LPO_REG, GCI_CHIPCONTROL_HIB_FORCE_INT_LPO_MASK, 0x0);

        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_CTL_REG, GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_DOWN_MASK, GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_DOWN_EXEC);
    }
    else
    {
        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_FORCE_INT_LPO_REG, GCI_CHIPCONTROL_HIB_FORCE_INT_LPO_MASK, GCI_CHIPCONTROL_HIB_FORCE_INT_LPO_EXEC);
        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_FORCE_EXT_LPO_REG, GCI_CHIPCONTROL_HIB_FORCE_EXT_LPO_MASK, 0x0);

        _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_CTL_REG, GCI_CHIPCONTROL_HIB_WAKE_CTL_XTAL_DOWN_MASK, GCI_CHIPCONTROL_HIB_WAKE_CTL_XTAL_DOWN_EXEC);
    }

    if (hib_ext_clock_freq != NULL)
    {
        *hib_ext_clock_freq = (hib->hib_ext_clock_freq == 0) ? _CYHAL_SYSPM_HIBERNATE_CLOCK_EXTERNAL_DEFAULT_FREQ : hib->hib_ext_clock_freq;
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t _cyhal_syspm_hibernation_start(uint32_t ticks_to_wakeup)
{
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_COUNT_REG,
                              GCI_CHIPCONTROL_HIB_WAKE_COUNT_MASK,
                              GCI_CHIPCONTROL_HIB_WAKE_COUNT_VAL(ticks_to_wakeup));

    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_START_REG,
                              GCI_CHIPCONTROL_HIB_START_MASK,
                              GCI_CHIPCONTROL_HIB_START_EXEC);
    /*
     * Hibernation mode essentially power off all components, except special hibernation block.
     * This block wakes up whole system when its timer expired.
     * Returning from hibernation mode is same as powering up, and whole boot sequence is involved.
     * So if we continue here it means system not entered hibernation mode, and so return error.
     */
    return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
}


/*******************************************************************************
*       Internal - Hibernate internal clock calibration functions
*******************************************************************************/

/* Set in clock driver */
extern uint32_t _cyhal_clock_cpu_clock_hz;

#define _CYHAL_SYSPM_HIBERNATE_CLOCK_INTERNAL_FREQ_128KHZ  128000
#define _CYHAL_SYSPM_HIBERNATE_CLOCK_INTERNAL_FREQ_32KHZ   32000
#define _CYHAL_SYSPM_HIBERNATE_CLOCK_INTERNAL_FREQ_16KHZ   16000

/* Low threshold for HIB internal clock frequency in Hz */
#define _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_FREQ_THRESHOLD_LOW(set)     ((set) - 1500)
/* High threshold for HIB internal clock frequency in Hz */
#define _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_FREQ_THRESHOLD_HIGH(set)    ((set) + 500)

#define _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_RC_CODES_COUNT     ((GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_RC_MASK >> GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_RC_SHIFT) + 1)

/*
 * HIB internal clock RC codes in monotonically
 * increasing order of LPO output frequencies.
 */
static uint8_t  _cyhal_syspm_hib_clock_internal_rc_codes_mono[_CYHAL_SYSPM_HIB_CLOCK_INTERNAL_RC_CODES_COUNT] =
                    { 19, 18, 17, 16, 23, 22, 21, 20, 27, 26, 25, 24, 31, 30, 29, 28,
                    3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 15, 14, 13, 12 };


static inline bool _cyhal_syspm_hib_clock_internal_rc_code_valid (uint32_t rc_code)
{
    return (rc_code < _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_RC_CODES_COUNT);
}

static void _cyhal_syspm_hib_clock_internal_rc_code_init(uint32_t rc_code)
{
    uint32_t wake_ctl_val = 0x0;
    uint32_t wake_ctl_mask = 0x0;

    wake_ctl_val  |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_RC_VAL(rc_code);
    wake_ctl_mask |= GCI_CHIPCONTROL_HIB_WAKE_CTL_LPO_RC_MASK;
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_WAKE_CTL_REG, wake_ctl_mask, wake_ctl_val);
}

static inline bool _cyhal_syspm_hib_clock_internal_error_value (uint32_t val, uint32_t set)
{
    return ((val > set) ? (val - set) : (set - val));
}

static inline bool _cyhal_syspm_hib_clock_internal_freq_valid (uint32_t val, uint32_t set)
{
    /* Low and high threshold for HIB internal clock frequency in Hz */
    return ((val >= _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_FREQ_THRESHOLD_LOW(set))
            && (val <= _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_FREQ_THRESHOLD_HIGH(set)));
}

static uint32_t _cyhal_syspm_hib_clock_internal_freq_measure (void)
{
    cyhal_system_delay_us(100); /* HIB internal clock frequency settling time (us) */
    osl_ilp_clock_measure(_cyhal_clock_cpu_clock_hz);
    return osl_ilp_clock();
}

static uint32_t _cyhal_syspm_hib_clock_measure_error(uint32_t hib_clock_set)
{
    uint32_t hib_clock_val;
    uint32_t hib_clock_err_val;

    hib_clock_val = _cyhal_syspm_hib_clock_internal_freq_measure();
    hib_clock_err_val = _cyhal_syspm_hib_clock_internal_error_value(hib_clock_val, hib_clock_set);

    return hib_clock_err_val;
}

/*
 * Calibration of HIB internal clock by calculating the
 * RC code using a quadratic polynomial derived from
 * statistical curve fitting of observed data points.
 */
static cy_rslt_t _cyhal_syspm_hib_clock_calibration_curve_fitting(uint32_t hib_clock_set)
{
    double a, b, c, d, o;
    int x0, x1;
    uint32_t rc_code_mid;
    uint32_t rc_code_set;
    uint32_t hib_clock_val;

    if (hib_clock_set != _CYHAL_SYSPM_HIBERNATE_CLOCK_INTERNAL_FREQ_32KHZ)
    {
        return CYHAL_SYSPM_RSLT_BAD_ARGUMENT;
    }

    /* The RC code corresponding to the median frequency */
    rc_code_mid = (uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[(_CYHAL_SYSPM_HIB_CLOCK_INTERNAL_RC_CODES_COUNT / 2)];

    _cyhal_syspm_hib_clock_internal_rc_code_init(rc_code_mid);
    hib_clock_val = _cyhal_syspm_hib_clock_internal_freq_measure();

    /* Coefficients derived from curve-fitting a board population */
    a = 0.0106376;
    b = 0.607115;

    /* Coefficient for adjusting the curve-fit to the current board */
    c = -((double)((int)hib_clock_set - (int)hib_clock_val) / 1000);
    d = sqrt((b * b) - (4 * a * c));

    /* Offset to adjust the curve-fit into the valid RC code range */
    o = 16.0;

    /* Perform a quadratic curve-fit using the computed coefficients */
    x0 = (int)(round((-b + d) / (2 * a)) + o);
    x1 = (int)(round((-b - d) / (2 * a)) + o);

    if ((!_cyhal_syspm_hib_clock_internal_rc_code_valid(x0)) && (!_cyhal_syspm_hib_clock_internal_rc_code_valid(x1)))
    {
        return CYHAL_SYSPM_RSLT_INIT_ERROR;
    }

    if ((_cyhal_syspm_hib_clock_internal_rc_code_valid(x0)) && (_cyhal_syspm_hib_clock_internal_rc_code_valid(x1)))
    {
        return CYHAL_SYSPM_RSLT_INIT_ERROR;
    }

    rc_code_set = (uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[(_cyhal_syspm_hib_clock_internal_rc_code_valid(x0) ? x0 : x1)];

    _cyhal_syspm_hib_clock_internal_rc_code_init(rc_code_set);
    hib_clock_val = _cyhal_syspm_hib_clock_internal_freq_measure();

    if (_cyhal_syspm_hib_clock_internal_freq_valid(hib_clock_val, hib_clock_set))
    {
        return CY_RSLT_SUCCESS;
    }
    else
    {
        return CYHAL_SYSPM_RSLT_INIT_ERROR;
    }
}

/*
 * Calibration of HIB internal clock by calculating
 * the RC code by running feedback-based binary
 * optimization over the entire set of RC codes.
 */
static cy_rslt_t _cyhal_syspm_hib_clock_calibration_binary_scanning(uint32_t hib_clock_set)
{
    cy_rslt_t ret;
    int rc_code_idx_low = 0;
    int rc_code_idx_mid = 0;
    int rc_code_idx_high = 0;
    int rc_code_idx_set = 0;
    uint32_t hib_clock_val;
    uint32_t hib_clock_err_val = 0xFFFFFFFFU;
    uint32_t hib_clock_err_min = 0xFFFFFFFFU;

    rc_code_idx_low = 0;
    rc_code_idx_high = _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_RC_CODES_COUNT - 1;

    /* Perform a binary scan to determine the RC code with the optimal (closest) frequency */
    while (rc_code_idx_low <= rc_code_idx_high)
    {
        rc_code_idx_mid = (rc_code_idx_low + rc_code_idx_high) / 2;

        _cyhal_syspm_hib_clock_internal_rc_code_init((uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[rc_code_idx_mid]);
        hib_clock_err_val = _cyhal_syspm_hib_clock_measure_error(hib_clock_set);
        hib_clock_val = osl_ilp_clock();

        if (hib_clock_err_val < hib_clock_err_min)
        {
            rc_code_idx_set = rc_code_idx_mid;
            hib_clock_err_min = hib_clock_err_val;
        }

        if (hib_clock_set == hib_clock_val)
        {
            break;
        }
        else if (hib_clock_set < hib_clock_val)
        {
            rc_code_idx_high = rc_code_idx_mid - 1;
        }
        else
        {
            rc_code_idx_low = rc_code_idx_mid + 1;
        }
    }

    _cyhal_syspm_hib_clock_internal_rc_code_init((uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[rc_code_idx_set]);
    hib_clock_val = _cyhal_syspm_hib_clock_internal_freq_measure();

    if (_cyhal_syspm_hib_clock_internal_freq_valid(hib_clock_val, hib_clock_set))
    {
        ret = CY_RSLT_SUCCESS;
    }
    else
    {
        /* Check neighbor RC codes for existence of a sub-optimal frequency in the valid range */
        if (hib_clock_val > _CYHAL_SYSPM_HIB_CLOCK_INTERNAL_FREQ_THRESHOLD_HIGH(hib_clock_set))
        {
            _cyhal_syspm_hib_clock_internal_rc_code_init((uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[rc_code_idx_set - 1]);
        }
        else
        {
            _cyhal_syspm_hib_clock_internal_rc_code_init((uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[rc_code_idx_set + 1]);
        }

        hib_clock_val = _cyhal_syspm_hib_clock_internal_freq_measure();

        if (_cyhal_syspm_hib_clock_internal_freq_valid(hib_clock_val, hib_clock_set))
        {
            ret = CY_RSLT_SUCCESS;
        }
        else
        {
            _cyhal_syspm_hib_clock_internal_rc_code_init((uint32_t)_cyhal_syspm_hib_clock_internal_rc_codes_mono[rc_code_idx_set]);
            _cyhal_syspm_hib_clock_internal_freq_measure();

            ret = CYHAL_SYSPM_RSLT_INIT_ERROR;
        }
    }

    return ret;
}

/*
 * Calibrates the imprecise internal HIB clock by selecting the optimal RC code that
 * compensates for the clock drift error. Assumes the CPU clock has been previously
 * initialized. Assumes the precise external HIB clock does not require any calibration.
 */
static cy_rslt_t _cyhal_syspm_hibernation_calibrate_internal_32khz_clock(void)
{
    uint32_t hib_clock_set = _CYHAL_SYSPM_HIBERNATE_CLOCK_INTERNAL_FREQ_32KHZ;
    uint32_t hib_clock_val = _cyhal_syspm_hib_clock_internal_freq_measure();

    /* Check if internal HIB clock is within error threshold */
    if (_cyhal_syspm_hib_clock_internal_freq_valid(hib_clock_val, hib_clock_set))
    {
        return CY_RSLT_SUCCESS;
    }

    /* Calibrate the internal HIB clock using curve-fitting */
    if (_cyhal_syspm_hib_clock_calibration_curve_fitting(hib_clock_set) == CY_RSLT_SUCCESS)
    {
        return CY_RSLT_SUCCESS;
    }

    /* Calibrate the internal HIB clock using binary-scanning */
    if (_cyhal_syspm_hib_clock_calibration_binary_scanning(hib_clock_set) == CY_RSLT_SUCCESS)
    {
        return CY_RSLT_SUCCESS;
    }

    /* Could not calibrate the internal HIB clock */
    return CYHAL_SYSPM_RSLT_INIT_ERROR;
}

static cy_rslt_t _cyhal_syspm_hibernation_init(void)
{
    cy_rslt_t rslt;
    rslt = _cyhal_syspm_hibernation_clock_init(&hibernation_config, &_cyhal_syspm_hibernation_ext_clock_freq);
    if (CY_RSLT_SUCCESS == rslt && hibernation_config.clock  == _CYHAL_SYSPM_HIBERNATION_CLOCK_INTERNAL_32KHZ)
    {
        rslt = _cyhal_syspm_hibernation_calibrate_internal_32khz_clock();
    }
    return rslt;
}


/*******************************************************************************
*       Internal - Backplane and clock setup functions
*******************************************************************************/

typedef union
{
    uint32_t raw;
    struct
    {
        unsigned bus_error_reset      : 1;
        unsigned bus_error_interrupt  : 1;
        unsigned timeout_reset        : 1;
        unsigned timeout_interrupt    : 1;
        unsigned timeout_exponent     : 5;
        unsigned timeout_enable       : 1;
        unsigned __reserved           : 22;
    } bits;
} _cyhal_syspm_dmp_error_log_control_t;

typedef enum
{
    DMP_ERROR_LOG_STATUS_CAUSE_NO_ERROR      = 0x0,
    DMP_ERROR_LOG_STATUS_CAUSE_SLAVE_ERROR   = 0x1,
    DMP_ERROR_LOG_STATUS_CAUSE_TIMEOUT_ERROR = 0x2,
    DMP_ERROR_LOG_STATUS_CAUSE_DECODE_ERROR  = 0x3,
} _cyhal_syspm_dmp_error_log_status_cause_t;

typedef union
{
    uint32_t raw;
    struct
    {
        _cyhal_syspm_dmp_error_log_status_cause_t cause : 2;
        unsigned overflow                  : 1;
        unsigned __reserved                : 29;
    } bits;
} _cyhal_syspm_dmp_error_log_status_t;

typedef enum
{
    _CYHAL_SYSPM_BACKPLANE_APPS,
    _CYHAL_SYSPM_BACKPLANE_WLAN,
    _CYHAL_SYSPM_BACKPLANE_AON
} _cyhal_syspm_backplane_t;

typedef enum
{
    _CYHAL_SYSPM_WRAPPER_SLAVE,
    _CYHAL_SYSPM_WRAPPER_MASTER
} _cyhal_syspm_wrapper_type_t;

typedef struct
{
    uint32_t                addr;
    _cyhal_syspm_backplane_t    bp;
    _cyhal_syspm_wrapper_type_t type;
} _cyhal_syspm_wrapper_descr_t;

static void _cyhal_syspm_wrappers_foreach(void (*callback)(volatile aidmp_t*, void*), void* ptr, uint32_t type_mask, uint32_t backplane_mask)
{
    static const _cyhal_syspm_wrapper_descr_t wrappers[] =
    {
        { .addr = PLATFORM_DDR_CONTROLLER_SLAVE_WRAPPER_REGBASE(0x0), .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_SOCSRAM_CH0_SLAVE_WRAPPER_REGBASE(0x0),    .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_SOCSRAM_CH1_SLAVE_WRAPPER_REGBASE(0x0),    .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_APPSCR4_SLAVE_WRAPPER_REGBASE(0x0),        .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_CHIPCOMMON_SLAVE_WRAPPER_REGBASE(0x0),     .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_WLANCR4_SLAVE_WRAPPER_REGBASE(0x0),        .bp = _CYHAL_SYSPM_BACKPLANE_WLAN, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_WLAN_APB0_SLAVE_WRAPPER_REGBASE(0x0),      .bp = _CYHAL_SYSPM_BACKPLANE_WLAN, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_WLAN_DEFAULT_SLAVE_WRAPPER_REGBASE(0x0),   .bp = _CYHAL_SYSPM_BACKPLANE_WLAN, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_AON_APB0_SLAVE_WRAPPER_REGBASE(0x0),       .bp = _CYHAL_SYSPM_BACKPLANE_AON,  .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_APPS_APB0_SLAVE_WRAPPER_REGBASE(0x0),      .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_APPS_DEFAULT_SLAVE_WRAPPER_REGBASE(0x0),   .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_SLAVE  },
        { .addr = PLATFORM_CHIPCOMMON_MASTER_WRAPPER_REGBASE(0x0),    .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_I2S0_MASTER_WRAPPER_REGBASE(0x0),          .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_I2S1_MASTER_WRAPPER_REGBASE(0x0),          .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_APPSCR4_MASTER_WRAPPER_REGBASE(0x0),       .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_M2M_MASTER_WRAPPER_REGBASE(0x0),           .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_GMAC_MASTER_WRAPPER_REGBASE(0x0),          .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_USB20H_MASTER_WRAPPER_REGBASE(0x0),        .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_USB20D_MASTER_WRAPPER_REGBASE(0x0),        .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_SDIOH_MASTER_WRAPPER_REGBASE(0x0),         .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_SDIOD_MASTER_WRAPPER_REGBASE(0x0),         .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_CRYPTO_MASTER_WRAPPER_REGBASE(0x0),        .bp = _CYHAL_SYSPM_BACKPLANE_APPS, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_DOT11MAC_MASTER_WRAPPER_REGBASE(0x0),      .bp = _CYHAL_SYSPM_BACKPLANE_WLAN, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
        { .addr = PLATFORM_WLANCR4_MASTER_WRAPPER_REGBASE(0x0),       .bp = _CYHAL_SYSPM_BACKPLANE_WLAN, .type = _CYHAL_SYSPM_WRAPPER_MASTER },
    };

    for (unsigned i = 0; i < ARRAYSIZE(wrappers); ++i)
    {
        const _cyhal_syspm_wrapper_descr_t* wrapper = &wrappers[i];

        if ((type_mask & (1 << wrapper->type)) && (backplane_mask & (1 << wrapper->bp)))
        {
            callback((volatile aidmp_t*)wrapper->addr, ptr);
        }
    }
}

static void _cyhal_syspm_wrapper_set_timeout(volatile aidmp_t* dmp, void* ptr)
{
    #define _CYHAL_SYSPM_BACKPLANE_MIN_TIMEOUT 4
    #define _CYHAL_SYSPM_BACKPLANE_MAX_TIMEOUT 19

    uint8_t timeout = *(uint8_t*)ptr;
    _cyhal_syspm_dmp_error_log_control_t error_log_control = { .raw = dmp->errlogctrl };

    if (timeout == 0)
    {
        error_log_control.bits.timeout_enable = 0;
    }
    else
    {
        timeout = MAX(MIN(timeout, _CYHAL_SYSPM_BACKPLANE_MAX_TIMEOUT), _CYHAL_SYSPM_BACKPLANE_MIN_TIMEOUT);
        error_log_control.bits.timeout_enable   = 1;
        error_log_control.bits.timeout_exponent = timeout;
    }

    dmp->errlogctrl = error_log_control.raw;
}

static uint32_t _cyhal_syspm_backplane_all_mask(void)
{
    return (1 << _CYHAL_SYSPM_BACKPLANE_APPS) | (1 << _CYHAL_SYSPM_BACKPLANE_AON) | (1 << _CYHAL_SYSPM_BACKPLANE_WLAN);
}

static void _cyhal_syspm_backplane_init(uint8_t timeout_power_2_cycles)
{
    _cyhal_syspm_wrappers_foreach(_cyhal_syspm_wrapper_set_timeout, &timeout_power_2_cycles, 1 << _CYHAL_SYSPM_WRAPPER_SLAVE, _cyhal_syspm_backplane_all_mask());
}

static void _cyhal_syspm_early_misc_init(void)
{
    /* Fixup max resource mask register. */
    _cyhal_system_common_chipcontrol(&PLATFORM_PMU->max_res_mask,
                                 PMU_RES_MAX_CLEAR_MASK,
                                 PMU_RES_MAX_SET_MASK);

    /*
     * Boot strap pins may force ILP be always requested. This prevents deep sleep from working properly.
     * Current register modification clears this request.
     * Boot strap pin issue was observed with FTDI chip which shared strap pin and pulled its up during board reset.
     */
    _cyhal_system_common_chipcontrol(&PLATFORM_PMU->pmucontrol_ext,
                                 PMU_CONTROL_EXT_ILP_ON_BP_DEBUG_MODE_MASK,
                                 0x0);

    /*
     * Make sure that board is waking up after reset pin reboot triggered.
     * Software drops min_res_mask to low value for deep sleep, need to restore mask during board resetting.
     * Below statement forces chip to restore min_res_mask to default value when reset pin reboot triggered.
     */
    _cyhal_system_common_chipcontrol(&PLATFORM_PMU->pmucontrol,
                                 PMU_CONTROL_RESETCONTROL_MASK,
                                 PMU_CONTROL_RESETCONTROL_RESTORE_RES_MASKS);
}

static void _cyhal_syspm_slow_clock_init(void)
{
    pmu_slowclkperiod_t period     = { .raw = 0 };
    /* 1 usec period of ALP clock measured in 1/16384 usec of ALP clock */
    unsigned long       alp_period = ((16UL * 1000000UL) / (osl_alp_clock() / 1024UL));

    period.bits.one_mhz_toggle_en = 1;
    period.bits.alp_period        = alp_period & PMU_SLOWCLKPERIOD_ALP_PERIOD_MASK;

    PLATFORM_PMU->slowclkperiod.raw = period.raw;
}

static void _cyhal_syspm_apps_clock_control(bool deep_sleep, bool enable)
{
    uint32_t intr_status = cyhal_system_critical_section_enter();
    volatile clock_control_status_t* core_status = PLATFORM_CLOCKSTATUS_REG(PLATFORM_APPSCR4_REGBASE(0));
    uint32_t readback;
    CY_UNUSED_PARAMETER(readback);
    
    if (enable)
    {
        core_status->bits.force_ilp_request = 1;          /* ILP is required for both */
        core_status->bits.force_hw_clock_req_off = 1;     /* assert ForceHWClockReqOff */
        if (!deep_sleep)
        {
            core_status->bits.force_ht_request = 1;       /* HT on backplane */
            core_status->bits.hq_clock_required = 1;      /* Force the HQ clock to be on */
            core_status->bits.ht_avail_request = 1;       /* enable HT request */
            core_status->bits.alp_avail_request = 1;      /* enable ALP request */
        }
        else
        {
            core_status->bits.force_ht_request = 0;       /* Need ILP on backplane */
            core_status->bits.hq_clock_required = 0;      /* HQ clock off */
            core_status->bits.ht_avail_request = 0;       /* disable HT request */
            core_status->bits.alp_avail_request = 0;      /* disable ALP request */
        }
    }
    else
    {
        core_status->bits.hq_clock_required = 0;      /* Let CPU request HQ clock */
        core_status->bits.force_ht_request = 1;       /* HT on backplane */
        core_status->bits.force_ilp_request = 0;      /* Let LPTimer request ILP */
        core_status->bits.force_hw_clock_req_off = 0; /* deassert ForceHWClockReqOff */
        core_status->bits.ht_avail_request = 0;       /* disable HT request */
        core_status->bits.alp_avail_request = 0;      /* disable ALP request */
    }

    readback = core_status->raw;                  /* read back to ensure write propagated */
    cyhal_system_delay_us(1);
    cyhal_system_critical_section_exit(intr_status);
}


/*******************************************************************************
*       Internal - WLAN powersave
*******************************************************************************/

#if defined WLAN_POWERSAVE_RES

#define PMU_RES_WLAN_UP_EVENT_MASK            PMU_RES_MASK(PMU_RES_WLAN_UP_EVENT)

static bool _cyhal_syspm_wlan_powersave_pmu_timer_slow_write_pending(void)
{
    return (PLATFORM_PMU->pmustatus & PST_SLOW_WR_PENDING) ? true : false;
}

static void _cyhal_syspm_wlan_powersave_pmu_timer_slow_write(uint32_t val)
{
    while (_cyhal_syspm_wlan_powersave_pmu_timer_slow_write_pending()){};
    PLATFORM_PMU->res_req_timer1.raw = val;
    while (_cyhal_syspm_wlan_powersave_pmu_timer_slow_write_pending()){};
}

static bool _cyhal_syspm_wlan_powersave_res_mask_event(bool enable)
{
    pmu_intstatus_t mask;

    /*
     * ISR acks interrupt, but it take a few ILP cycles to do it.
     * As result if resource mask is changing quickly we may ack and lose some
     * of the subsequent interrupts and hang by waiting on a semaphore.
     * To avoid this, do not try to enable interrupts till ack is settled.
     */
    cyhal_system_delay_us(((PMU_MAX_WRITE_LATENCY_ILP_TICKS + 1) * 1000000UL) / osl_ilp_clock());

    mask.raw                  = 0;
    mask.bits.rsrc_event_int1 = 1;

    if (enable)
        _cyhal_system_common_chipcontrol(&PLATFORM_PMU->pmuintmask1.raw, 0x0, mask.raw);
    else
        _cyhal_system_common_chipcontrol(&PLATFORM_PMU->pmuintmask1.raw, mask.raw, 0x0);

    return true;
}

/*
 * Function return synchronized state of 2 registers.
 * As resources can go down let's remove pending one from current resource mask.
 */
static uint32_t _cyhal_syspm_wlan_powersave_res_get_current_resources(uint32 mask)
{
    while (true)
    {
        uint32_t res_state   = PLATFORM_PMU->res_state & mask;
        uint32_t res_pending = PLATFORM_PMU->res_pending & mask;

        if (res_state != (PLATFORM_PMU->res_state & mask))
            continue;

        if (res_pending != (PLATFORM_PMU->res_pending & mask))
            continue;

        return (res_state & ~res_pending);
    }
}

static void _cyhal_syspm_wlan_powersave_res_wait_event(void)
{
    while (true)
    {
        uint32_t mask = _cyhal_syspm_wlan_powersave_res_get_current_resources(PMU_RES_WLAN_UP_MASK);

        if (mask == PMU_RES_WLAN_UP_MASK)
            break; /* Done */

        if ((mask | PMU_RES_WLAN_UP_EVENT_MASK) == PMU_RES_WLAN_UP_MASK)
            continue; /* Nearly here, switch to polling */

        _cyhal_syspm_wlan_powersave_res_mask_event(true);
    }
}

/* Force WLAN up by setting res_req_timer1 */
static void _cyhal_syspm_wlan_powersave_res_up(void)
{
    pmu_res_req_timer_t timer;

    timer.raw                   = 0;
    timer.bits.req_active       = 1;
    timer.bits.force_ht_request = 1;
    timer.bits.clkreq_group_sel = pmu_res_clkreq_apps_group;

    PLATFORM_PMU->res_req_mask1 = PMU_RES_WLAN_UP_MASK;
    _cyhal_syspm_wlan_powersave_pmu_timer_slow_write(timer.raw);
    _cyhal_syspm_wlan_powersave_res_wait_event();
}

/* Turn WLAN down by clearing res_req_timer1 */
static void _cyhal_syspm_wlan_powersave_res_down(void)
{
    _cyhal_syspm_wlan_powersave_pmu_timer_slow_write(0x0);
    PLATFORM_PMU->res_req_mask1 = PMU_RES_APPS_UP_MASK;
}

static void _cyhal_syspm_wlan_powersave_res_ack_event(void)
{
    pmu_intstatus_t status;

    status.raw                  = 0;
    status.bits.rsrc_event_int1 = 1;

    PLATFORM_PMU->pmuintstatus.raw = status.raw;
}

void _cyhal_syspm_wlan_powersave_res_event(void)
{
    _cyhal_syspm_wlan_powersave_res_ack_event();
    _cyhal_syspm_wlan_powersave_res_mask_event(false);
}

#endif


/*******************************************************************************
*       Internal - Helper functions
*******************************************************************************/

/* Hz to KHz */
#define _CYHAL_HZ_TO_KHZ_CONVERSION_FACTOR (1000)

/* Set in timer driver */
extern cyhal_timer_t*    _cyhal_timer_copy;

static cyhal_syspm_system_state_t _cyhal_syspm_system_state = CYHAL_SYSPM_SYSTEM_NORMAL;
static bool _cyhal_disable_systick_before_sleep_deepsleep = false;

/* Syspm event handler for general LPM needs */
static bool _cyhal_syspm_general_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    CY_UNUSED_PARAMETER(callback_arg);

    if (state == CYHAL_SYSPM_CB_CPU_DEEPSLEEP)
    {
        switch(mode)
        {
            case CYHAL_SYSPM_CHECK_READY:
            case CYHAL_SYSPM_CHECK_FAIL:
            case CYHAL_SYSPM_BEFORE_TRANSITION:
                _cyhal_syspm_mcu_powersave_set_mode(true);
                break;
            case CYHAL_SYSPM_AFTER_TRANSITION:
                _cyhal_syspm_mcu_powersave_set_mode(false);
                break;
            default:
                break;
        }
    }

    return true;
}

#define _CYHAL_SYSPM_CB_ALL ((cyhal_syspm_callback_state_t)(CYHAL_SYSPM_CB_CPU_SLEEP \
                                                        | CYHAL_SYSPM_CB_CPU_DEEPSLEEP \
                                                        | CYHAL_SYSPM_CB_SYSTEM_HIBERNATE \
                                                        | CYHAL_SYSPM_CB_SYSTEM_NORMAL \
                                                        | CYHAL_SYSPM_CB_SYSTEM_LOW))

static uint16_t _cyhal_deep_sleep_lock = 0;
static uint32_t _cyhal_syspm_supply_voltages[((size_t)CYHAL_VOLTAGE_SUPPLY_MAX) + 1] = { 0 };

/* The first entry in the callback chain is always reserved for the user set
 * cyhal_syspm_register_callback callback. This may be set to a sentinel value
 * indicating it is the end of the list. All subsequent slots are where
 * peripheral drivers are tracked. This makes it very easy to determine whether
 * the user registered a callback and to make sure we run that first. */
static cyhal_syspm_callback_data_t* _cyhal_syspm_callback_ptr = CYHAL_SYSPM_END_OF_LIST;
static cyhal_syspm_callback_data_t* _cyhal_syspm_peripheral_callback_ptr = CYHAL_SYSPM_END_OF_LIST;

/* General syspm callback */
static cyhal_syspm_callback_data_t _cyhal_syspm_general_callback_data =
{
    .callback = &_cyhal_syspm_general_callback,
    .states = _CYHAL_SYSPM_CB_ALL,
    .next = NULL,
    .args = NULL,
    .ignore_modes = (cyhal_syspm_callback_mode_t)0,
};

static cyhal_syspm_callback_data_t* _cyhal_syspm_call_all_pm_callbacks(
    cyhal_syspm_callback_data_t* entry, bool* allow, cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode)
{
    while(entry != CYHAL_SYSPM_END_OF_LIST)
    {
        if (entry->callback != NULL &&
            (entry->states & state) == state &&
            (entry->ignore_modes & mode) != mode)
        {
            *allow = entry->callback(state, mode, entry->args) || mode != CYHAL_SYSPM_CHECK_READY;
            if (!(*allow))
            {
                // Do not increment pointer so that backtracking stop at the correct location
                break;
            }
        }
        entry = entry->next;
    }
    return entry;
}

static void _cyhal_syspm_backtrack_all_pm_callbacks(cyhal_syspm_callback_data_t* start, cyhal_syspm_callback_data_t* end, cyhal_syspm_callback_state_t state)
{
    while(start != end)
    {
        if (start->callback != NULL &&
            (start->states & state) == state &&
            (start->ignore_modes & CYHAL_SYSPM_CHECK_FAIL) != CYHAL_SYSPM_CHECK_FAIL)
        {
            start->callback(state, CYHAL_SYSPM_CHECK_FAIL, start->args);
        }
        start = start->next;
    }
}

static cy_rslt_t _cyhal_syspm_common_cb(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode)
{
    if ((state == CYHAL_SYSPM_CB_CPU_DEEPSLEEP) && (mode == CYHAL_SYSPM_CHECK_READY) && (_cyhal_deep_sleep_lock != 0))
    {
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }
    else
    {
        bool allow = true;
        cyhal_syspm_callback_data_t *first, *second;

        if (mode == CYHAL_SYSPM_CHECK_FAIL || mode == CYHAL_SYSPM_AFTER_TRANSITION)
        {
            first = _cyhal_syspm_peripheral_callback_ptr;
            second = _cyhal_syspm_callback_ptr;
        }
        else
        {
            second = _cyhal_syspm_peripheral_callback_ptr;
            first = _cyhal_syspm_callback_ptr;
        }

        cyhal_syspm_callback_data_t* first_current = _cyhal_syspm_call_all_pm_callbacks(first, &allow, state, mode);
        cyhal_syspm_callback_data_t* second_current = allow
            ? _cyhal_syspm_call_all_pm_callbacks(second, &allow, state, mode)
            : second;

        if (!allow && (CYHAL_SYSPM_CHECK_READY == mode))
        {
            _cyhal_syspm_backtrack_all_pm_callbacks(second, second_current, state);
            _cyhal_syspm_backtrack_all_pm_callbacks(first, first_current, state);
        }

        if (allow
            && (state == CYHAL_SYSPM_CB_CPU_DEEPSLEEP)
            && (_cyhal_timer_copy != NULL))
        {
            static bool timerOn = false;

            if (mode == CYHAL_SYSPM_BEFORE_TRANSITION)
            {
                if ((_cyhal_timer_copy->running) && _cyhal_disable_systick_before_sleep_deepsleep)
                {
                    timerOn = true;
                    allow = (cyhal_timer_stop(_cyhal_timer_copy) == CY_RSLT_SUCCESS);
                }
                else
                {
                    timerOn = false;
                }
            }
            else if (mode == CYHAL_SYSPM_AFTER_TRANSITION)
            {
                if (timerOn)
                {
                    allow = (cyhal_timer_start(_cyhal_timer_copy) == CY_RSLT_SUCCESS);
                    timerOn = false;
                }
            }
        }

        return allow ? CY_RSLT_SUCCESS : CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }
}

static void _cyhal_syspm_remove_callback_from_list(cyhal_syspm_callback_data_t **list, cyhal_syspm_callback_data_t *remove)
{
    uint32_t intr_status = cyhal_system_critical_section_enter();
    while(*list != CYHAL_SYSPM_END_OF_LIST)
    {
        if (*list == remove)
        {
            *list = remove->next;
            remove->next = NULL;
            break;
        }
        list = &((*list)->next);
    }
    cyhal_system_critical_section_exit(intr_status);
}

static bool _cyhal_syspm_is_registered(cyhal_syspm_callback_data_t *callback)
{
    // If callback->next is NULL it must not be registered since all registered
    // next ptrs in the list must point to the next callback or be equal to
    // CYHAL_SYSPM_END_OF_LIST
    return (callback->next != NULL);
}

void _cyhal_syspm_register_peripheral_callback(cyhal_syspm_callback_data_t *callback_data)
{
    CY_ASSERT(callback_data != NULL);
    uint32_t intr_status = cyhal_system_critical_section_enter();
    if(!_cyhal_syspm_is_registered(callback_data))
    {
        callback_data->next = _cyhal_syspm_peripheral_callback_ptr;
        _cyhal_syspm_peripheral_callback_ptr = callback_data;
    }
    cyhal_system_critical_section_exit(intr_status);
}

void _cyhal_syspm_unregister_peripheral_callback(cyhal_syspm_callback_data_t *callback_data)
{
    _cyhal_syspm_remove_callback_from_list(&_cyhal_syspm_peripheral_callback_ptr, callback_data);
}

cy_rslt_t _cyhal_syspm_tickless_sleep_deepsleep(cyhal_lptimer_t *obj, uint32_t desired_ms, uint32_t *actual_ms, bool deep_sleep)
{
    CY_ASSERT(obj != NULL);
    uint32_t initial_ticks;
    uint32_t sleep_ticks;
    cyhal_lptimer_info_t timer_info;

    *actual_ms = 0;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if(desired_ms > 0)
    {
        cyhal_lptimer_get_info(obj, &timer_info);

        sleep_ticks = desired_ms * (timer_info.frequency_hz / _CYHAL_HZ_TO_KHZ_CONVERSION_FACTOR);
        initial_ticks = cyhal_lptimer_read(obj);

        result = cyhal_lptimer_set_delay(obj, sleep_ticks);
        if(result == CY_RSLT_SUCCESS)
        {
            /* Disabling and enabling the system timer is handled in _cyhal_syspm_common_cb in order
             * to prevent loosing kernel ticks when sleep/deep-sleep is rejected causing the time spent
             * in the callback handlers to check if the system can make the sleep/deep-sleep transition
             * to be not accounted for.
             */
            _cyhal_disable_systick_before_sleep_deepsleep = true;
            cyhal_lptimer_enable_event(obj, CYHAL_LPTIMER_COMPARE_MATCH, CYHAL_ISR_PRIORITY_DEFAULT, true);

            result = deep_sleep ? cyhal_syspm_deepsleep() : cyhal_syspm_sleep();
            if(result == CY_RSLT_SUCCESS)
            {
                uint32_t final_ticks = cyhal_lptimer_read(obj);
                uint32_t ticks = (final_ticks < initial_ticks)
                                ? (timer_info.max_counter_value - initial_ticks) + final_ticks
                                : final_ticks - initial_ticks;
                *actual_ms = ticks / (timer_info.frequency_hz / _CYHAL_HZ_TO_KHZ_CONVERSION_FACTOR);
            }

            cyhal_lptimer_enable_event(obj, CYHAL_LPTIMER_COMPARE_MATCH, CYHAL_ISR_PRIORITY_DEFAULT, false);
            _cyhal_disable_systick_before_sleep_deepsleep = false;
        }
    }

    return result;
}


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_syspm_init(void)
{
    // Set up backplane and perform fixups
    _cyhal_syspm_early_misc_init();
    _cyhal_syspm_backplane_init(PLATFORM_BP_TIMEOUT);
    _cyhal_syspm_slow_clock_init();
    
    // Note: Possible HW latch issue that randomly resets the device. 
    // Explicitly turning off the WDT mitigates it from occurring.
    PLATFORM_PMU->pmuwatchdog = 0;

    // Set up hibernation clock and timer
    cy_rslt_t rslt = _cyhal_syspm_hibernation_init();

    if (CY_RSLT_SUCCESS == rslt)
    {
        // MCU powersave
        _cyhal_syspm_mcu_powersave_init();
        // Regulator control
        _cyhal_syspm_cores_powersave_disable_wl_reg_on_pulldown(true);
        // Cores Apps powersave
        _cyhal_syspm_cores_powersave_init_apps_domain();
        // Cores WLAN powersave. Disable entire WLAN
        _cyhal_syspm_cores_powersave_down_all_wlan_cores();

        // Register general callback
        _cyhal_syspm_register_peripheral_callback(&_cyhal_syspm_general_callback_data);

        rslt = _cyhal_clock_system_init();
    }

    return rslt;
}

cy_rslt_t cyhal_syspm_hibernate(cyhal_syspm_hibernate_source_t wakeup_source)
{
    if (wakeup_source != CYHAL_SYSPM_HIBERNATE_WDT)
        return CYHAL_SYSPM_RSLT_ERR_NOT_SUPPORTED;

    // If watchdog, check that it was configured
    if (PLATFORM_PMU->pmuwatchdog == 0)
        return CYHAL_SYSPM_RSLT_BAD_ARGUMENT;

    cy_rslt_t result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_HIBERNATE, CYHAL_SYSPM_CHECK_READY);
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_HIBERNATE, CYHAL_SYSPM_BEFORE_TRANSITION);
        if (result == CY_RSLT_SUCCESS)
        {
            // Set the hibernation tick counter to be the same as the watchdog
            uint32_t ticks_to_wakeup = PLATFORM_PMU->pmuwatchdog;

            // Should not return from this point onwards.
            _cyhal_syspm_hibernation_start(ticks_to_wakeup);
        }
    }
    return result;
}

cy_rslt_t cyhal_syspm_set_system_state(cyhal_syspm_system_state_t state)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    switch (state)
    {
        case CYHAL_SYSPM_SYSTEM_NORMAL:
            result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_NORMAL, CYHAL_SYSPM_CHECK_READY);
            if (result == CY_RSLT_SUCCESS)
            {
                result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_NORMAL, CYHAL_SYSPM_BEFORE_TRANSITION);
                if (result == CY_RSLT_SUCCESS)
                {
                    #if defined WLAN_POWERSAVE_RES
                    _cyhal_syspm_mcu_powersave_set_aon(true);
                    _cyhal_syspm_wlan_powersave_res_up();
                    #endif
                    _cyhal_syspm_system_state = state;
                    result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_NORMAL, CYHAL_SYSPM_AFTER_TRANSITION);
                }
            }
            break;
        case CYHAL_SYSPM_SYSTEM_LOW:
            result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_LOW, CYHAL_SYSPM_CHECK_READY);
            if (result == CY_RSLT_SUCCESS)
            {
                result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_SYSTEM_LOW, CYHAL_SYSPM_BEFORE_TRANSITION);
                if (result == CY_RSLT_SUCCESS)
                {
                    #if defined WLAN_POWERSAVE_RES
                    _cyhal_syspm_mcu_powersave_set_aon(false);
                    _cyhal_syspm_wlan_powersave_res_down();
                    #endif
                    _cyhal_syspm_system_state = state;
                }
            }
            break;
        default:
            result = CYHAL_SYSPM_RSLT_BAD_ARGUMENT;
            break;
    }

    return result;
}

cyhal_syspm_system_state_t cyhal_syspm_get_system_state(void)
{
    return _cyhal_syspm_system_state;
}

void cyhal_syspm_register_callback(cyhal_syspm_callback_data_t *callback_data)
{
    CY_ASSERT(callback_data != NULL);
    uint32_t intr_status = cyhal_system_critical_section_enter();
    if(!_cyhal_syspm_is_registered(callback_data))
    {
        callback_data->next = _cyhal_syspm_callback_ptr;
        _cyhal_syspm_callback_ptr = callback_data;
    }
    cyhal_system_critical_section_exit(intr_status);
}

void cyhal_syspm_unregister_callback(cyhal_syspm_callback_data_t *callback_data)
{
    _cyhal_syspm_remove_callback_from_list(&_cyhal_syspm_callback_ptr, callback_data);
}

cy_rslt_t cyhal_syspm_sleep(void)
{
    cy_rslt_t result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_CPU_SLEEP, CYHAL_SYSPM_CHECK_READY);
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_CPU_SLEEP, CYHAL_SYSPM_BEFORE_TRANSITION);
        if (result == CY_RSLT_SUCCESS)
        {
            _cyhal_syspm_apps_clock_control(false, true);
            cpu_wait_for_interrupt();
            _cyhal_syspm_apps_clock_control(false, false);
            result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_CPU_SLEEP, CYHAL_SYSPM_AFTER_TRANSITION);
        }
    }

    return result;
}

cy_rslt_t cyhal_syspm_deepsleep(void)
{
    cy_rslt_t result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_CPU_DEEPSLEEP, CYHAL_SYSPM_CHECK_READY);
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_CPU_DEEPSLEEP, CYHAL_SYSPM_BEFORE_TRANSITION);
        if (result == CY_RSLT_SUCCESS)
        {
            _cyhal_syspm_apps_clock_control(true, true);
            cpu_wait_for_interrupt();
            _cyhal_syspm_apps_clock_control(true, false);
            result = _cyhal_syspm_common_cb(CYHAL_SYSPM_CB_CPU_DEEPSLEEP, CYHAL_SYSPM_AFTER_TRANSITION);
        }
    }

    return result;
}

void cyhal_syspm_lock_deepsleep(void)
{
    CY_ASSERT(_cyhal_deep_sleep_lock != USHRT_MAX);
    uint32_t intr_status = cyhal_system_critical_section_enter();
    if (_cyhal_deep_sleep_lock < USHRT_MAX)
    {
        _cyhal_deep_sleep_lock++;
    }
    cyhal_system_critical_section_exit(intr_status);
}

void cyhal_syspm_unlock_deepsleep(void)
{
    CY_ASSERT(_cyhal_deep_sleep_lock != 0U);
    uint32_t intr_status = cyhal_system_critical_section_enter();
    if (_cyhal_deep_sleep_lock > 0U)
    {
        _cyhal_deep_sleep_lock--;
    }
    cyhal_system_critical_section_exit(intr_status);
}

cy_rslt_t cyhal_syspm_tickless_deepsleep(cyhal_lptimer_t *obj, uint32_t desired_ms, uint32_t *actual_ms)
{
    return _cyhal_syspm_tickless_sleep_deepsleep(obj, desired_ms, actual_ms, true);
}

cy_rslt_t cyhal_syspm_tickless_sleep(cyhal_lptimer_t *obj, uint32_t desired_ms, uint32_t *actual_ms)
{
    return _cyhal_syspm_tickless_sleep_deepsleep(obj, desired_ms, actual_ms, false);
}

void cyhal_syspm_set_supply_voltage(cyhal_syspm_voltage_supply_t supply, uint32_t mvolts)
{
    CY_ASSERT((size_t)supply <= CYHAL_VOLTAGE_SUPPLY_MAX);
    _cyhal_syspm_supply_voltages[(size_t)supply] = mvolts;
}

uint32_t cyhal_syspm_get_supply_voltage(cyhal_syspm_voltage_supply_t supply)
{
    CY_ASSERT((size_t)supply <= CYHAL_VOLTAGE_SUPPLY_MAX);
    return _cyhal_syspm_supply_voltages[(size_t)supply];
}

#if defined(__cplusplus)
}
#endif
