/*******************************************************************************
* File Name: cyhal_clock.c
*
* Description:
* Provides an implementation for high level interface for interacting with the
* Device Clocks.
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

#include <string.h>
#include "cyhal_clock.h"
#include "cyhal_hwmgr.h"
#include "cyhal_system.h"
#include "cyhal_system_impl.h"
#include "cy_utils.h"
#include "platform_appscr4.h"
#include "cr4.h"

#include "wiced_osl.h"

#if defined(__cplusplus)
extern "C"
{
#endif

const cyhal_clock_tolerance_t CYHAL_TOLERANCE_0_P = {CYHAL_TOLERANCE_PERCENT, 0};
const cyhal_clock_tolerance_t CYHAL_TOLERANCE_1_P = {CYHAL_TOLERANCE_PERCENT, 1};
const cyhal_clock_tolerance_t CYHAL_TOLERANCE_5_P = {CYHAL_TOLERANCE_PERCENT, 5};


/*******************************************************************************
*       Clock resources
*******************************************************************************/

const cyhal_resource_inst_t CYHAL_CLOCK_RSC_ALP  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_ALP, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_HT   = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_HT, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_ILP  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_ILP, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_LPO  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_LPO, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_XTAL = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_XTAL, 0 };

const cyhal_resource_inst_t CYHAL_CLOCK_RSC_BB_PLL  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_BB_PLL, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_AUDIO_PLL  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_AUDIO_PLL, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_USB_PLL  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_USB_PLL, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_HSIC_PLL  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_HSIC_PLL, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_WLAN_PLL  = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_WLAN_PLL, 0 };

const cyhal_resource_inst_t CYHAL_CLOCK_RSC_CPU = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_CPU, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_BACKPLANE = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_BACKPLANE, 0 };
const cyhal_resource_inst_t CYHAL_CLOCK_RSC_FAST_UART = { CYHAL_RSC_CLOCK, (uint8_t)CYHAL_CLOCK_BLOCK_FAST_UART, 0 };


#define _CYHAL_CLOCK_CREATE(x)	{ .block = (CYHAL_CLOCK_BLOCK_##x), .channel = 0, .reserved = false }

const cyhal_clock_t CYHAL_CLOCK_ALP         = _CYHAL_CLOCK_CREATE(ALP);
const cyhal_clock_t CYHAL_CLOCK_HT          = _CYHAL_CLOCK_CREATE(HT);
const cyhal_clock_t CYHAL_CLOCK_ILP         = _CYHAL_CLOCK_CREATE(ILP);
const cyhal_clock_t CYHAL_CLOCK_LPO         = _CYHAL_CLOCK_CREATE(LPO);
const cyhal_clock_t CYHAL_CLOCK_XTAL        = _CYHAL_CLOCK_CREATE(XTAL);

const cyhal_clock_t CYHAL_CLOCK_BB_PLL      = _CYHAL_CLOCK_CREATE(BB_PLL);
const cyhal_clock_t CYHAL_CLOCK_AUDIO_PLL   = _CYHAL_CLOCK_CREATE(AUDIO_PLL);
const cyhal_clock_t CYHAL_CLOCK_USB_PLL     = _CYHAL_CLOCK_CREATE(USB_PLL);
const cyhal_clock_t CYHAL_CLOCK_HSIC_PLL    = _CYHAL_CLOCK_CREATE(HSIC_PLL);
const cyhal_clock_t CYHAL_CLOCK_WLAN_PLL    = _CYHAL_CLOCK_CREATE(WLAN_PLL);

const cyhal_clock_t CYHAL_CLOCK_CPU         = _CYHAL_CLOCK_CREATE(CPU);
const cyhal_clock_t CYHAL_CLOCK_BACKPLANE   = _CYHAL_CLOCK_CREATE(BACKPLANE);
const cyhal_clock_t CYHAL_CLOCK_FAST_UART   = _CYHAL_CLOCK_CREATE(FAST_UART);


/*******************************************************************************
*       Internal - types and prototypes
*******************************************************************************/

typedef enum
{
#if PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_24_MHZ,
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_48_MHZ,
#endif  /* PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED */
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_60_MHZ,
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_80_MHZ,
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_120_MHZ,
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_160_MHZ,
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_320_MHZ,
#if defined(PLATFORM_4390X_OVERCLOCK)
    _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_480_MHZ
#endif  /* PLATFORM_4390X_OVERCLOCK */
} _cyhal_clock_cpu_clock_frequency_t;

typedef enum
{
    _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE = 0x0, /* CPU runs on backplane clock */
    _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM       = 0x4  /* CPU runs on own ARM clock */
} _cyhal_clock_cpu_clock_source_t;

typedef struct
{
    uint32_t                    apps_cpu_freq_reg_value;
    uint32_t                    apps_bp_freq_reg_value;
    _cyhal_clock_cpu_clock_source_t clock_source;
    bool                        clock_source_only;
#if defined(PLATFORM_4390X_OVERCLOCK)
    uint8_t                     pll_channel;
    uint8_t                     pll_divider;
#endif /* PLATFORM_4390X_OVERCLOCK */
} _cyhal_clock_cpu_clock_freq_config_t;

static void _cyhal_clock_lpo_clock_init(void);
static uint32_t _cyhal_clock_backplane_clock_get_freq(void);
static void _cyhal_clock_backplane_clock_init(void);
static cy_rslt_t _cyhal_clock_ht_clock_init(void);
static cy_rslt_t _cyhal_clock_cpu_core_init(void);
static _cyhal_clock_cpu_clock_source_t _cyhal_clock_cpu_clock_get_source(void);
static void _cyhal_clock_cpu_clock_source(_cyhal_clock_cpu_clock_source_t clock_source);
static bool _cyhal_clock_cpu_clock_drives_backplane_clock(void);
static uint32_t _cyhal_clock_cpu_clock_get_freq_for_source(_cyhal_clock_cpu_clock_source_t source);
static uint32_t _cyhal_clock_cpu_clock_get_freq(void);
static cy_rslt_t _cyhal_clock_cpu_clock_freq_to_config(_cyhal_clock_cpu_clock_frequency_t freq, _cyhal_clock_cpu_clock_freq_config_t* config);
static cy_rslt_t _cyhal_clock_cpu_clock_init(_cyhal_clock_cpu_clock_frequency_t freq);


/*******************************************************************************
*       Internal - LPO and ILP clocks
*******************************************************************************/

// LPO and ILP init are same for apps
static void _cyhal_clock_lpo_clock_init(void)
{
    uint32_t flags = cyhal_system_critical_section_enter();
    uint32_t cpu_freq = _cyhal_clock_cpu_clock_get_freq();

#if PLATFORM_LPO_CLOCK_EXT
    osl_set_ext_lpoclk(cpu_freq);
#else
    osl_set_int_lpoclk(cpu_freq);
#endif /* PLATFORM_LPO_CLK_EXT */

    cyhal_system_critical_section_exit(flags);
}


/*******************************************************************************
*       Internal - Backplane and HT clocks
*******************************************************************************/

#define _CYHAL_CLOCK_BP_CLK_FROM_ARMCR4_REG_VALUE    0x0

static bool _cyhal_clock_cpu_clock_drives_backplane_clock(void)
{
    const uint32_t val = _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_REG, 0x0, 0x0);
    return ((val & GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_MASK) == GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_SET) ? true : false;
}

static uint32_t _cyhal_clock_backplane_clock_get_freq(void)
{
    uint32_t flags = cyhal_system_critical_section_enter();
    uint32_t freq = (_cyhal_clock_cpu_clock_drives_backplane_clock()) ?
                    _cyhal_clock_cpu_clock_get_freq_for_source(_CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM) / 2 :
                    _cyhal_clock_cpu_clock_get_freq_for_source(_CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE);
    cyhal_system_critical_section_exit(flags);

    return freq;
}

static void _cyhal_clock_backplane_clock_init(void)
{
    /* Ensure backplane clock is sourced from backplane BBPLL path */
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_REG,
                                  GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_MASK,
                                  0x0);
}

static cy_rslt_t _cyhal_clock_ht_clock_init(void)
{
    uint32_t timeout;
    for (timeout = 100000000; timeout != 0; timeout--)
    {
        if (PLATFORM_CLOCKSTATUS_REG(PLATFORM_APPSCR4_REGBASE(0x0))->bits.bp_on_ht != 0)
        {
            break;
        }
    }
    return (timeout == 0) ? CYHAL_CLOCK_RSLT_ERR_LOCK : CY_RSLT_SUCCESS;
}


/*******************************************************************************
*       Internal - CPU clock
*******************************************************************************/

/* Preset CPU and backplane clock frequencies */
#define _CYHAL_CLOCK_HZ_24MHZ                 (24000000)
#define _CYHAL_CLOCK_HZ_48MHZ                 (48000000)
#define _CYHAL_CLOCK_HZ_60MHZ                 (60000000)
#define _CYHAL_CLOCK_HZ_80MHZ                 (80000000)
#define _CYHAL_CLOCK_HZ_120MHZ                (120000000)
#define _CYHAL_CLOCK_HZ_160MHZ                (160000000)
#define _CYHAL_CLOCK_HZ_320MHZ                (320000000)
#if defined(PLATFORM_4390X_OVERCLOCK)
#define _CYHAL_CLOCK_HZ_480MHZ                (480000000)
#endif /* PLATFORM_4390X_OVERCLOCK */
#define _CYHAL_CLOCK_HZ_960MHZ                (960000000)

#define _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(cpu_freq, backplane_freq, source, source_only, pll_ch1, pll_ch2) \
    { \
        config->apps_cpu_freq_reg_value = (cpu_freq);         \
        config->apps_bp_freq_reg_value  = (backplane_freq);   \
        config->clock_source            = (source);           \
        config->clock_source_only       = (source_only);      \
        channels[0]                     = (uint8_t)(pll_ch1); \
        channels[1]                     = (uint8_t)(pll_ch2); \
    }

#if defined(PLATFORM_4390X_OVERCLOCK)
#define _CYHAL_CLOCK_CPU_CLOCK_FREQ_PLL_CONFIG_DEFINE( channel, divider ) \
    { \
        config->pll_channel = (channel); \
        config->pll_divider = (divider); \
    }
#warning "Overclocking functionality is experimental. Do not attempt to use this (or program) unsupervised."

/* Verify CPU overclocking configuration. */
#if (PLATFORM_BACKPLANE_ON_CPU_CLOCK_ENABLE == 0)
#error "Overclocking unimplemented when local div-2 disabled."
#endif
#endif /* PLATFORM_4390X_OVERCLOCK */

/* CPU clock frequency is needed by system and syspm drivers */
uint32_t _cyhal_clock_cpu_clock_hz = _CYHAL_CLOCK_HZ_160MHZ;

static cy_rslt_t _cyhal_clock_cpu_core_init(void)
{
    appscr4_core_ctrl_reg_t core_ctrl;

    /* Initialize core control register. */
    core_ctrl.raw = PLATFORM_APPSCR4->core_ctrl.raw;

    core_ctrl.bits.force_clock_source     = 0;
    core_ctrl.bits.wfi_clk_stop           = 1;
    core_ctrl.bits.s_error_int_en         = 1;
    core_ctrl.bits.pclk_dbg_stop          = 0;
    core_ctrl.bits.sleeping_clk_req       = 0;
    core_ctrl.bits.not_sleeping_clk_req_0 = 1;
    core_ctrl.bits.not_sleeping_clk_req_1 = 0;

    PLATFORM_APPSCR4->core_ctrl.raw = core_ctrl.raw;

    /* Core is configured to request HT clock. Let's wait for the backplane to actually run on HT. */
    return _cyhal_clock_ht_clock_init();
}

static _cyhal_clock_cpu_clock_source_t _cyhal_clock_cpu_clock_get_source(void)
{
    return (_cyhal_clock_cpu_clock_source_t)(PLATFORM_APPSCR4->core_ctrl.bits.clock_source);
}

static void _cyhal_clock_cpu_clock_source(_cyhal_clock_cpu_clock_source_t clock_source)
{
    /*
     * Source change actually happen when WFI instruction is executed and _only_ if WFI
     * actually wait for interrupts. I.e. if interrupt was pending before WFI, then no
     * switch happen; interrupt must fire after WFI executed.
     *
     * When switching into 2:1 mode (_CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM) SW must ensure that the HT clock is
     * available and backplane runs on HT.
     * All clock values are valid provided there is always a pair of clocks; one at 2x and one at 1x.
     * Apps CPU needs to downshift to 1:1 (_CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE) before going to ALP.
     */

    const uint32_t          clockstable_mask = IRQN2MASK(ClockStable_ExtIRQn);
    uint32_t                irq_mask;
    uint32_t                int_mask;
    appscr4_core_ctrl_reg_t core_ctrl, core_ctrl_restore;

    /* Return if already use requested source */
    if (_cyhal_clock_cpu_clock_get_source() == clock_source)
    {
        return;
    }

    /* Save masks and disable all interrupts */
    irq_mask = PLATFORM_APPSCR4->irq_mask;
    int_mask = PLATFORM_APPSCR4->int_mask;
    PLATFORM_APPSCR4->irq_mask = 0;
    PLATFORM_APPSCR4->int_mask = 0;

    /* Tell hw which source want */
    core_ctrl.raw = PLATFORM_APPSCR4->core_ctrl.raw;
    core_ctrl.bits.clock_source = clock_source;
    core_ctrl_restore.raw = core_ctrl.raw;
    core_ctrl.bits.wfi_clk_stop = 1;
    PLATFORM_APPSCR4->core_ctrl.raw = core_ctrl.raw;

    /*
     * Ack, unmask ClockStable interrupt and call WFI.
     * We should not have pending interrupts at the moment, so calling should
     * trigger clock switch. At the end of switch CPU will be woken up
     * by ClockStable interrupt.
     */
    PLATFORM_APPSCR4->fiqirq_status = clockstable_mask;
    PLATFORM_APPSCR4->irq_mask = clockstable_mask;
    cpu_wait_for_interrupt();
    CY_ASSERT(PLATFORM_APPSCR4->fiqirq_status & clockstable_mask);
    PLATFORM_APPSCR4->fiqirq_status = clockstable_mask;

    /* Restore interrupt masks */
    PLATFORM_APPSCR4->core_ctrl.raw = core_ctrl_restore.raw;
    PLATFORM_APPSCR4->int_mask      = int_mask;
    PLATFORM_APPSCR4->irq_mask      = irq_mask;
}

static uint32_t _cyhal_clock_cpu_clock_get_freq_for_source(_cyhal_clock_cpu_clock_source_t source)
{
    uint32_t ret = _CYHAL_CLOCK_HZ_160MHZ;

    switch (source)
    {
        case _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM:
            switch(_cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_APPS_CPU_FREQ_REG, 0x0, 0x0) & GCI_CHIPCONTROL_APPS_CPU_FREQ_MASK)
            {
#if defined(PLATFORM_4390X_OVERCLOCK)
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_320_480:
                    switch (_cyhal_system_pmu_pllcontrol_mdiv_get(PLL_FREQ_320_480_MHZ_CHANNEL))
                    {
                        case PLL_FREQ_480_MHZ_DIVIDER:
                            ret = _CYHAL_CLOCK_HZ_480MHZ;
                            break;
                        case PLL_FREQ_320_MHZ_DIVIDER:
                            ret = _CYHAL_CLOCK_HZ_320MHZ;
                            break;
                        default:
                            CY_ASSERT(false);
                            break;
                    }
                    break;
#else  /* PLATFORM_4390X_OVERCLOCK */
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_320:
                    ret = _CYHAL_CLOCK_HZ_320MHZ;
                    break;
#endif /* PLATFORM_4390X_OVERCLOCK */
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_160:
                    ret = _CYHAL_CLOCK_HZ_160MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_120:
                    ret = _CYHAL_CLOCK_HZ_120MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_80:
                    ret = _CYHAL_CLOCK_HZ_80MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_60:
                    ret = _CYHAL_CLOCK_HZ_60MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_48:
                    ret = _CYHAL_CLOCK_HZ_48MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_CPU_FREQ_24:
                    ret = _CYHAL_CLOCK_HZ_24MHZ;
                    break;
                default:
                    CY_ASSERT(false);
                    break;
            }
            break;
        case _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE:
            switch(_cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_APPS_BP_FREQ_REG, 0x0, 0x0) & GCI_CHIPCONTROL_APPS_BP_FREQ_MASK)
            {
                case GCI_CHIPCONTROL_APPS_BP_FREQ_160:
                    ret = _CYHAL_CLOCK_HZ_160MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_BP_FREQ_120:
                    ret = _CYHAL_CLOCK_HZ_120MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_BP_FREQ_80:
                    ret = _CYHAL_CLOCK_HZ_80MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_BP_FREQ_60:
                    ret = _CYHAL_CLOCK_HZ_60MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_BP_FREQ_48:
                    ret = _CYHAL_CLOCK_HZ_48MHZ;
                    break;
                case GCI_CHIPCONTROL_APPS_BP_FREQ_24:
                    ret = _CYHAL_CLOCK_HZ_24MHZ;
                    break;
                default:
                    CY_ASSERT(false);
                    break;
            }
            break;
        default: /* switch (source) */
            CY_ASSERT(false);
            break;
    }

    return ret;
}

static uint32_t _cyhal_clock_cpu_clock_get_freq(void)
{
    uint32_t flags = cyhal_system_critical_section_enter();
    uint32_t freq = (_cyhal_clock_cpu_clock_get_source() == _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM) ?
                    _cyhal_clock_cpu_clock_get_freq_for_source(_CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM) :
                    _cyhal_clock_backplane_clock_get_freq();
    cyhal_system_critical_section_exit(flags);

    return freq;
}

static cy_rslt_t _cyhal_clock_cpu_clock_freq_to_config(_cyhal_clock_cpu_clock_frequency_t freq, _cyhal_clock_cpu_clock_freq_config_t* config)
{
    uint8_t  channels[2];
    unsigned i;

#if defined(PLATFORM_4390X_OVERCLOCK)
    _CYHAL_CLOCK_CPU_CLOCK_FREQ_PLL_CONFIG_DEFINE(PLL_FREQ_320_480_MHZ_CHANNEL, PLL_FREQ_320_MHZ_DIVIDER);
#endif /* PLATFORM_4390X_OVERCLOCK */

    switch (freq)
    {
#if PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED
        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_24_MHZ:
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_24, GCI_CHIPCONTROL_APPS_BP_FREQ_24, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, false, 5, -1);
            break;

        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_48_MHZ:
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_48, GCI_CHIPCONTROL_APPS_BP_FREQ_48, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, false, 5, -1);
            break;
#endif /* PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED */

        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_60_MHZ:
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_60, GCI_CHIPCONTROL_APPS_BP_FREQ_60, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, false, 2, -1);
            break;

        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_80_MHZ:
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_80, GCI_CHIPCONTROL_APPS_BP_FREQ_80, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, false, 3, -1);
            break;

        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_120_MHZ:
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_120, GCI_CHIPCONTROL_APPS_BP_FREQ_120, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, false, 2, -1);
            break;

        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_160_MHZ:
            if (_cyhal_clock_cpu_clock_get_freq() == _CYHAL_CLOCK_HZ_320MHZ)
            {
                _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(0, 0, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, true, -1, -1);
            }
            else
            {
                _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_160, GCI_CHIPCONTROL_APPS_BP_FREQ_160, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE, false, 3, -1);
            }
            break;

        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_320_MHZ:
#if defined(PLATFORM_4390X_OVERCLOCK)
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_320_480, GCI_CHIPCONTROL_APPS_BP_FREQ_160, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM, false, 1, 3);
#else
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_320, GCI_CHIPCONTROL_APPS_BP_FREQ_160, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM, false, 1, 3);
#endif /* PLATFORM_4390X_OVERCLOCK */
            break;

#if defined(PLATFORM_4390X_OVERCLOCK)
        case _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_480_MHZ:
            /* 480MHZ must run on ARM clock source and backplane use div by 2 ARM clock */
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_CONFIG_DEFINE(GCI_CHIPCONTROL_APPS_CPU_FREQ_320_480, GCI_CHIPCONTROL_APPS_BP_FREQ_160, _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM, false, 1, 3);
            _CYHAL_CLOCK_CPU_CLOCK_FREQ_PLL_CONFIG_DEFINE(PLL_FREQ_320_480_MHZ_CHANNEL, PLL_FREQ_480_MHZ_DIVIDER);
            break;
#endif /* PLATFORM_4390X_OVERCLOCK */

        default:
            return CYHAL_CLOCK_RSLT_ERR_FREQ;
    }

    for (i = 0; i < ARRAYSIZE(channels); i++)
    {
        if (channels[i] != (uint8_t)-1)
        {
            uint32_t field = ((_cyhal_system_pmu_pllcontrol(PLL_CONTROL_ENABLE_CHANNEL_REG, 0, 0) & PLL_CONTROL_ENABLE_CHANNEL_MASK) >> PLL_CONTROL_ENABLE_CHANNEL_SHIFT);

            if (((1U << (channels[i] - 1)) & field) != 0)
            {
                /* Channel disabled */
                return CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
            }
        }
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t _cyhal_clock_cpu_clock_init(_cyhal_clock_cpu_clock_frequency_t freq)
{
    _cyhal_clock_cpu_clock_freq_config_t config;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = _cyhal_clock_cpu_clock_freq_to_config(freq, &config);
    if (result == CY_RSLT_SUCCESS)
    {
        uint32_t flags = cyhal_system_critical_section_enter();

        /* Switch cpu clock to known state before making modifications */
        result = _cyhal_clock_cpu_core_init();
        if (result == CY_RSLT_SUCCESS)
        {
#if defined(PLATFORM_4390X_OVERCLOCK)
            if (_cyhal_system_pmu_pllcontrol_mdiv_get(config.pll_channel) != config.pll_divider)
            {
                _cyhal_system_pmu_pllcontrol_mdiv_set(config.pll_channel, config.pll_divider);
                _cyhal_clock_ht_clock_init();
            }
#endif /* PLATFORM_4390X_OVERCLOCK */

            if (config.clock_source_only)
            {
                _cyhal_clock_cpu_clock_source(config.clock_source);
            }
            else if (config.clock_source == _CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM)
            {
                /* Moving into ARMCR4 2:1 mode. */

                /* Select ARM frequency from selection mux. */
                _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_APPS_CPU_FREQ_REG,
                                        GCI_CHIPCONTROL_APPS_CPU_FREQ_MASK,
                                        config.apps_cpu_freq_reg_value);

                /* Select source of backplane clock, from:
                * (1) local divide-by-2 of APPS CPU clock selection; or
                * (2) APP backplane clock selection.
                * Div-2 intended to source backplane/CPU from a single clock,
                * where skew might be an issue.
                * Beware: There is a silicon bug where div-2 is 180-degrees
                * phase shifted causing issues with some peripherals (like USB),
                * which are intolerant to phase differences.
                */
                _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_REG,
                                        GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_MASK,
                                        _CYHAL_CLOCK_BP_CLK_FROM_ARMCR4_REG_VALUE);

                /* Select backplane frequency from selection mux.
                * This setting is superfluous if local div-2 is enabled.
                */
                _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_APPS_BP_FREQ_REG,
                                        GCI_CHIPCONTROL_APPS_BP_FREQ_MASK,
                                        config.apps_bp_freq_reg_value);

                /* Change clock source.
                * Do this last, so that backplane and CPU frequencies are
                * already set appropriately.
                */
                _cyhal_clock_cpu_clock_source(config.clock_source);
            }
            else if (config.clock_source == _CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE)
            {
                /* Moving into ARMCR4 1:1 mode. */

                /* Change source.
                * After this point, ARM and backplane clocks will be running from
                * apps_backplane_clk (ARM 1:1 ratio).
                * Do this first!  Moving directly from 2:1 to 1:1 (120/120) wedges
                * when not using local div-2.
                *
                */
                _cyhal_clock_cpu_clock_source(config.clock_source);

                /* Select backplane frequency from selection mux. */
                _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_APPS_BP_FREQ_REG,
                                        GCI_CHIPCONTROL_APPS_BP_FREQ_MASK,
                                        config.apps_bp_freq_reg_value);

                /* Ensure backplane clock is sourced from backplane BBPLL path and
                * not from the local div-2.
                */
                _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_REG,
                                        GCI_CHIPCONTROL_BP_CLK_FROM_ARMCR4_CLK_MASK,
                                        0x0);

                /* No need to select ARM frequency from selection mux, since
                * ARM 1:1 ratio will only make use of apps_backplane_clk.
                */
            }
            else
            {
                CY_ASSERT(false); // Should never get here
            }
        }
        cyhal_system_critical_section_exit(flags);
    }

    _cyhal_clock_cpu_clock_hz = _cyhal_clock_cpu_clock_get_freq();

    return result;
}

cy_rslt_t _cyhal_clock_system_init(void)
{
    cy_rslt_t rslt;
    rslt = _cyhal_clock_ht_clock_init();
    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = _cyhal_clock_cpu_clock_init(_CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_160_MHZ); // default
    }
    if (CY_RSLT_SUCCESS == rslt)
    {
        _cyhal_clock_lpo_clock_init();
        _cyhal_clock_backplane_clock_init();
    }

    return rslt;
}



/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_clock_allocate(cyhal_clock_t *clock, cyhal_clock_block_t block)
{
    CY_ASSERT(NULL != clock);

    cyhal_resource_inst_t clock_resource = { CYHAL_RSC_CLOCK, block, 0u };
    cy_rslt_t status = cyhal_hwmgr_reserve(&clock_resource);

    if (CY_RSLT_SUCCESS == status)
    {
        clock->block = block;
        clock->channel = 0;
        clock->reserved = true;
    }

    return status;
}

cy_rslt_t cyhal_clock_get(cyhal_clock_t *clock, const cyhal_resource_inst_t *resource)
{
    CY_ASSERT(NULL != clock);
    CY_ASSERT(NULL != resource);
    CY_ASSERT(CYHAL_RSC_CLOCK == resource->type);

    clock->block = (cyhal_clock_block_t)resource->block_num;
    clock->channel = resource->channel_num;
    clock->reserved = false;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_clock_reserve(cyhal_clock_t *clock, const cyhal_clock_t *clock_)
{
    CY_ASSERT(NULL != clock);
    CY_ASSERT(NULL != clock_);

    cyhal_resource_inst_t clock_resource = { CYHAL_RSC_CLOCK, clock_->block, clock_->channel };
    cy_rslt_t rslt = cyhal_hwmgr_reserve(&clock_resource);
    if (CY_RSLT_SUCCESS == rslt)
    {
        memcpy(clock, clock_, sizeof(cyhal_clock_t));
        clock->reserved = true;
    }
    return rslt;
}

cyhal_clock_feature_t cyhal_clock_get_features(const cyhal_clock_t *clock)
{
    CY_ASSERT(NULL != clock);

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
        case CYHAL_CLOCK_BLOCK_HT:
        case CYHAL_CLOCK_BLOCK_ILP:
        case CYHAL_CLOCK_BLOCK_LPO:
        case CYHAL_CLOCK_BLOCK_XTAL:
            return CYHAL_CLOCK_FEATURE_NONE;
        case CYHAL_CLOCK_BLOCK_BB_PLL:
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            return CYHAL_CLOCK_FEATURE_NONE;
        case CYHAL_CLOCK_BLOCK_CPU:
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
            return (cyhal_clock_feature_t)(CYHAL_CLOCK_FEATURE_SOURCE | CYHAL_CLOCK_FEATURE_FREQUENCY);
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            return CYHAL_CLOCK_FEATURE_NONE;
        default:
            CY_ASSERT(false); // Unhandled clock
            return CYHAL_CLOCK_FEATURE_NONE;
    }
}

bool cyhal_clock_is_enabled(const cyhal_clock_t *clock)
{
    CY_ASSERT(NULL != clock);

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
        case CYHAL_CLOCK_BLOCK_HT:
        case CYHAL_CLOCK_BLOCK_ILP:
        case CYHAL_CLOCK_BLOCK_LPO:
        case CYHAL_CLOCK_BLOCK_XTAL:
        case CYHAL_CLOCK_BLOCK_BB_PLL:
            return true;
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            // Add when working on peripherals
            return false;
        case CYHAL_CLOCK_BLOCK_CPU:
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            return true;
        default:
            CY_ASSERT(false); //Unhandled clock
            return false;
    }
}

cy_rslt_t cyhal_clock_set_enabled(cyhal_clock_t *clock, bool enabled, bool wait_for_lock)
{
    CY_UNUSED_PARAMETER(enabled);
    CY_UNUSED_PARAMETER(wait_for_lock);

    CY_ASSERT(NULL != clock);

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
        case CYHAL_CLOCK_BLOCK_HT:
        case CYHAL_CLOCK_BLOCK_ILP:
        case CYHAL_CLOCK_BLOCK_LPO:
        case CYHAL_CLOCK_BLOCK_XTAL:
        case CYHAL_CLOCK_BLOCK_BB_PLL:
            return CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            // Add when working on peripherals
            return CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
        case CYHAL_CLOCK_BLOCK_CPU:
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            return CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
        default:
            return CYHAL_CLOCK_RSLT_ERR_SOURCE;
    }
}

uint32_t cyhal_clock_get_frequency(const cyhal_clock_t *clock)
{
    CY_ASSERT(NULL != clock);

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
            return osl_alp_clock();
        case CYHAL_CLOCK_BLOCK_HT:
            return _cyhal_clock_backplane_clock_get_freq();
        case CYHAL_CLOCK_BLOCK_ILP:
            return osl_ilp_clock();
        case CYHAL_CLOCK_BLOCK_LPO:
            return osl_ilp_clock();
        case CYHAL_CLOCK_BLOCK_XTAL:
            return osl_alp_clock();
        case CYHAL_CLOCK_BLOCK_BB_PLL:
            return _CYHAL_CLOCK_HZ_960MHZ; /* 960.1 MHz nominal */
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            return 0; // Add when working on peripherals
        case CYHAL_CLOCK_BLOCK_CPU:
            return _cyhal_clock_cpu_clock_get_freq();
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
            return _cyhal_clock_backplane_clock_get_freq();
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            return _CYHAL_CLOCK_HZ_160MHZ; /* use fixed PLL output */
        default:
            CY_ASSERT(false); /* Not supported clock */
            return 0;
    }
}

cy_rslt_t cyhal_clock_set_frequency(cyhal_clock_t *clock, uint32_t hz, const cyhal_clock_tolerance_t *tolerance)
{
    CY_UNUSED_PARAMETER(tolerance);

    CY_ASSERT(NULL != clock);
    cy_rslt_t status = CY_RSLT_SUCCESS;
    _cyhal_clock_cpu_clock_frequency_t cpu_freq;

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
        case CYHAL_CLOCK_BLOCK_HT:
        case CYHAL_CLOCK_BLOCK_ILP:
        case CYHAL_CLOCK_BLOCK_LPO:
            status = CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
            break;
        case CYHAL_CLOCK_BLOCK_XTAL:
        case CYHAL_CLOCK_BLOCK_BB_PLL:
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            // Dividers are preset and should not be configurable
            status = CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
            break;
        case CYHAL_CLOCK_BLOCK_CPU:
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
            switch (hz)
            {
#if PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED
                case _CYHAL_CLOCK_HZ_24MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_24_MHZ;
                    break;
                case _CYHAL_CLOCK_HZ_48MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_48_MHZ;
                    break;
#endif  /* PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED */
                case _CYHAL_CLOCK_HZ_60MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_60_MHZ;
                    break;
                case _CYHAL_CLOCK_HZ_80MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_80_MHZ;
                    break;
                case _CYHAL_CLOCK_HZ_120MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_120_MHZ;
                    break;
                case _CYHAL_CLOCK_HZ_160MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_160_MHZ;
                    break;
                case _CYHAL_CLOCK_HZ_320MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_320_MHZ;
                    break;
#if defined(PLATFORM_4390X_OVERCLOCK)
                case _CYHAL_CLOCK_HZ_480MHZ:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_480_MHZ;
                    break;
#endif  /* PLATFORM_4390X_OVERCLOCK */
                default:
                    cpu_freq = _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_160_MHZ;
                    status = CYHAL_CLOCK_RSLT_ERR_FREQ;
                    break;
            }
            if ((clock->block == CYHAL_CLOCK_BLOCK_BACKPLANE)
                && (cpu_freq == _CYHAL_CLOCK_CPU_CLOCK_FREQUENCY_320_MHZ))
            {
                // Backplane frequency cannot go beyond 160 MHz
                status = CYHAL_CLOCK_RSLT_ERR_FREQ;
            }
            else if (status == CY_RSLT_SUCCESS)
            {
                /*
                * Note: CPU and backplane frequencies cannot be separated and must be done together.
                *       Otherwise it can lead to incorrect peripheral operation such as problems
                *       in DMA transactions.
                */
                status = _cyhal_clock_cpu_clock_init(cpu_freq);
            }
            break;
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            status = CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
            break;
        default:
            status = CYHAL_CLOCK_RSLT_ERR_SOURCE;
            break;
    }

    return status;
}

cy_rslt_t cyhal_clock_set_divider(cyhal_clock_t *clock, uint32_t divider)
{
    // This functionality is not available
    CY_UNUSED_PARAMETER(clock);
    CY_UNUSED_PARAMETER(divider);
    return CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
}

cy_rslt_t cyhal_clock_get_sources(const cyhal_clock_t *clock, const cyhal_resource_inst_t **sources[], uint32_t *count)
{
    /* Clock source definitions */
    static const cyhal_resource_inst_t *_CYHAL_CLOCK_SOURCE_XTAL[] = { &CYHAL_CLOCK_RSC_XTAL };
    static const cyhal_resource_inst_t *_CYHAL_CLOCK_SOURCE_BB_PLL[] = { &CYHAL_CLOCK_RSC_BB_PLL };
    static const cyhal_resource_inst_t *_CYHAL_CLOCK_SOURCE_HT[] = { &CYHAL_CLOCK_RSC_HT };
    static const cyhal_resource_inst_t *_CYHAL_CLOCK_SOURCE_LPO[] = { &CYHAL_CLOCK_RSC_LPO };
    static const cyhal_resource_inst_t *_CYHAL_CLOCK_SOURCE_CPU[] = { &CYHAL_CLOCK_RSC_CPU, &CYHAL_CLOCK_RSC_BACKPLANE };

    CY_ASSERT(NULL != clock);
    cy_rslt_t status = CY_RSLT_SUCCESS;

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
            *sources = _CYHAL_CLOCK_SOURCE_XTAL;
            *count = 1;
            break;
        case CYHAL_CLOCK_BLOCK_HT:
            *sources = _CYHAL_CLOCK_SOURCE_BB_PLL;
            *count = 1;
            break;
        case CYHAL_CLOCK_BLOCK_ILP:
            *sources = _CYHAL_CLOCK_SOURCE_LPO;
            *count = 1;
            break;
        case CYHAL_CLOCK_BLOCK_LPO:
        case CYHAL_CLOCK_BLOCK_XTAL:
            *count = 0;
            break;
        case CYHAL_CLOCK_BLOCK_BB_PLL:
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            *sources = _CYHAL_CLOCK_SOURCE_XTAL;
            *count = 1;
            break;
        case CYHAL_CLOCK_BLOCK_CPU:
            *sources = _CYHAL_CLOCK_SOURCE_CPU;
            *count = 2;
            break;
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            *sources = _CYHAL_CLOCK_SOURCE_HT;
            *count = 1;
            break;
        default:
            status = CYHAL_CLOCK_RSLT_ERR_SOURCE;
            *count = 0;
            break;
    }

    return status;
}

cy_rslt_t cyhal_clock_set_source(cyhal_clock_t *clock, const cyhal_clock_t *source)
{
    CY_ASSERT(NULL != clock);
    cy_rslt_t status = CY_RSLT_SUCCESS;

    switch (clock->block)
    {
        case CYHAL_CLOCK_BLOCK_ALP:
        case CYHAL_CLOCK_BLOCK_HT:
        case CYHAL_CLOCK_BLOCK_ILP: // Can be either LPO or ALP but no real use case for using ALP
        case CYHAL_CLOCK_BLOCK_LPO:
        case CYHAL_CLOCK_BLOCK_XTAL:
        case CYHAL_CLOCK_BLOCK_BB_PLL:
        case CYHAL_CLOCK_BLOCK_AUDIO_PLL:
        case CYHAL_CLOCK_BLOCK_USB_PLL:
        case CYHAL_CLOCK_BLOCK_HSIC_PLL:
        case CYHAL_CLOCK_BLOCK_WLAN_PLL:
            status = CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
            break;
        case CYHAL_CLOCK_BLOCK_CPU:
            if (source->block == CYHAL_CLOCK_BLOCK_CPU)
                _cyhal_clock_cpu_clock_source(_CYHAL_CLOCK_CPU_CLOCK_SOURCE_ARM);
            else if (source->block == CYHAL_CLOCK_BLOCK_BACKPLANE)
                _cyhal_clock_cpu_clock_source(_CYHAL_CLOCK_CPU_CLOCK_SOURCE_BACKPLANE);
            else
                status = CYHAL_CLOCK_RSLT_ERR_SOURCE;
            break;
        case CYHAL_CLOCK_BLOCK_BACKPLANE:
        case CYHAL_CLOCK_BLOCK_FAST_UART:
            status = CYHAL_CLOCK_RSLT_ERR_NOT_SUPPORTED;
            break;
        default:
            status = CYHAL_CLOCK_RSLT_ERR_SOURCE;
            break;
    }

    return status;
}

void cyhal_clock_free(cyhal_clock_t *clock)
{
    CY_ASSERT(NULL != clock);
    CY_ASSERT(clock->reserved);

    cyhal_resource_inst_t rsc = { CYHAL_RSC_CLOCK, clock->block, clock->channel };
    cyhal_hwmgr_free(&rsc);
    clock->reserved = false;
}

#if defined(__cplusplus)
}
#endif
