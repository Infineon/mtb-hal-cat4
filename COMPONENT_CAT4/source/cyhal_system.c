/***************************************************************************//**
* \file cyhal_system.c
*
* Description:
* Provides a high level interface for interacting with the System driver.
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

#include "cyhal_system.h"
#include "cy_utils.h"
#include "cyhal_pin_package.h"
#include "platform_appscr4.h"

#include "typedefs.h"
#include "sbchipc.h"
#include "aidmp.h"

#if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
#include "cyabs_rtos.h"
#endif

/*******************************************************************************
*       Internal - Chip Control/Common and GCI
*******************************************************************************/

uint32_t _cyhal_system_chipcontrol(volatile uint32_t* addr_reg, volatile uint32_t* ctrl_reg,
                     uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    uint32_t ret;
    uint32_t val;
    uint32_t flags;

    flags = cyhal_system_critical_section_enter();

    *addr_reg = reg_offset;
    val = *ctrl_reg;
    ret = (val & ~clear_mask) | set_mask;
    if (val != ret)
    {
        *ctrl_reg = ret;
    }

    cyhal_system_critical_section_exit(flags);

    return ret;
}

uint32_t _cyhal_system_common_chipcontrol(volatile uint32_t* reg, uint32_t clear_mask, uint32_t set_mask)
{
    uint32_t ret;
    uint32_t val;
    uint32_t flags;

    flags = cyhal_system_critical_section_enter();

    val = *reg;
    ret = (val & ~clear_mask) | set_mask;
    if (val != ret)
    {
        *reg = ret;
    }

    cyhal_system_critical_section_exit(flags);

    return ret;
}

uint32_t _cyhal_system_chipstatus(volatile uint32_t* addr_reg, volatile uint32_t* status_reg,
                    uint8_t reg_offset)
{
    uint32_t ret;
    uint32_t flags = cyhal_system_critical_section_enter();

    *addr_reg = reg_offset;
    ret = *status_reg;

    cyhal_system_critical_section_exit(flags);

    return ret;
}


/*******************************************************************************
*       Internal - PMU PLL control
*******************************************************************************/

#if defined(PLATFORM_4390X_OVERCLOCK)
static bool _cyhal_system_pmu_pllcontrol_mdiv_get_init_params(uint8_t channel, uint8_t divider, uint8_t* reg_offset, uint32_t* mask, uint32_t* val, uint16_t* shift)
{
    switch (channel)
    {
        case 1:
            *reg_offset = PLL_CONTROL_M1DIV_REG;
            *mask       = PLL_CONTROL_M1DIV_MASK;
            *val        = PLL_CONTROL_M1DIV_VAL(divider);
            *shift      = PLL_CONTROL_M1DIV_SHIFT;
            return true;

        case 2:
            *reg_offset = PLL_CONTROL_M2DIV_REG;
            *mask       = PLL_CONTROL_M2DIV_MASK;
            *val        = PLL_CONTROL_M2DIV_VAL(divider);
            *shift      = PLL_CONTROL_M2DIV_SHIFT;
            return true;

        case 3:
            *reg_offset = PLL_CONTROL_M3DIV_REG;
            *mask       = PLL_CONTROL_M3DIV_MASK;
            *val        = PLL_CONTROL_M3DIV_VAL(divider);
            *shift      = PLL_CONTROL_M3DIV_SHIFT;
            return true;

        case 4:
            *reg_offset = PLL_CONTROL_M4DIV_REG;
            *mask       = PLL_CONTROL_M4DIV_MASK;
            *val        = PLL_CONTROL_M4DIV_VAL(divider);
            *shift      = PLL_CONTROL_M4DIV_SHIFT;
            return true;

        case 5:
            *reg_offset = PLL_CONTROL_M5DIV_REG;
            *mask       = PLL_CONTROL_M5DIV_MASK;
            *val        = PLL_CONTROL_M5DIV_VAL(divider);
            *shift      = PLL_CONTROL_M5DIV_SHIFT;
            return true;

        case 6:
            *reg_offset = PLL_CONTROL_M6DIV_REG;
            *mask       = PLL_CONTROL_M6DIV_MASK;
            *val        = PLL_CONTROL_M6DIV_VAL(divider);
            *shift      = PLL_CONTROL_M6DIV_SHIFT;
            return true;

        default:
            return false;
    }
}

uint8_t _cyhal_system_pmu_pllcontrol_mdiv_get(uint8_t channel)
{
    uint8_t  reg_offset;
    uint32_t mask;
    uint32_t val;
    uint16_t shift;

    if (!_cyhal_system_pmu_pllcontrol_mdiv_get_init_params(channel, 0, &reg_offset, &mask, &val, &shift))
    {
        return 0;
    }

    return ((_cyhal_system_pmu_pllcontrol(reg_offset, 0x0, 0x0) & mask) >> shift);
}

void _cyhal_system_pmu_pllcontrol_mdiv_set(uint8_t channel, uint8_t divider)
{
    uint32_t flags;
    uint32_t val;
    uint8_t  reg_offset;
    uint32_t mask;
    uint16_t shift;
    uint32_t new_val;
    uint32_t load_enable;
    uint32_t min_res_mask;
    uint32_t max_res_mask;
    uint32_t pmucontrol;

    if (!_cyhal_system_pmu_pllcontrol_mdiv_get_init_params(channel, divider, &reg_offset, &mask, &new_val, &shift))
    {
        return;
    }

    flags = cyhal_system_critical_section_enter();

    PLATFORM_PMU->pllcontrol_addr = reg_offset;
    val                           = PLATFORM_PMU->pllcontrol_data;

    if (new_val != (val & mask))
    {
        /* Force dropping HT resources */
        min_res_mask = PLATFORM_PMU->min_res_mask;
        max_res_mask = PLATFORM_PMU->max_res_mask;
        PLATFORM_PMU->min_res_mask = min_res_mask & ~PMU_RES_HT_CLOCKS_MASK;
        PLATFORM_PMU->max_res_mask = max_res_mask & ~PMU_RES_HT_CLOCKS_MASK;

        /* Wait till backplane start to run on ALP clock */
        while (PLATFORM_CLOCKSTATUS_REG(PLATFORM_APPSCR4_REGBASE(0x0))->bits.ht_clock_available);
        cyhal_system_delay_us(100);

        /* Enable divider loading into PLL */
        PLATFORM_PMU->pllcontrol_addr = PLL_CONTROL_LOAD_ENABLE_REG;
        load_enable                   = PLATFORM_PMU->pllcontrol_data & ~PLL_CONTROL_LOAD_ENABLE_MASK;
        PLATFORM_PMU->pllcontrol_data = load_enable | PLL_CONTROL_LOAD_ENABLE_VAL(channel);

        /* Set new divider */
        PLATFORM_PMU->pllcontrol_addr = reg_offset;
        PLATFORM_PMU->pllcontrol_data = new_val | (val & ~mask);

        /* Trigger PLL update */
        pmucontrol = (PLATFORM_PMU->pmucontrol & ~PMU_CONTROL_PLLCTL_UPDATE_MASK) | PMU_CONTROL_PLLCTL_UPDATE_EXEC;
        PLATFORM_PMU->pmucontrol = pmucontrol;

        /* Disable divider loading */
        PLATFORM_PMU->pllcontrol_addr = PLL_CONTROL_LOAD_ENABLE_REG;
        PLATFORM_PMU->pllcontrol_data = load_enable;

        /* Trigger PLL update */
        PLATFORM_PMU->pmucontrol = pmucontrol;

        /* Restore masks */
        PLATFORM_PMU->max_res_mask = max_res_mask;
        PLATFORM_PMU->min_res_mask = min_res_mask;
    }

    cyhal_system_critical_section_exit(flags);
}
#endif  /* PLATFORM_4390X_OVERCLOCK */


/*******************************************************************************
*       Internal - Pin mux
*******************************************************************************/

void _cyhal_system_pinmux_connect(cyhal_gpio_t pin, int gci_chipcontrol_mux)
{
    cyhal_pin_gci_mapping_t gci = cyhal_pin_gci_map[pin];

    /* Ony set mux if it's not a direct connection */
    if ((gci.gci_chipcontrol_reg != GCI_DIRECT_CONNECT) && (gci.gci_chipcontrol_pos != GCI_DIRECT_CONNECT))
    {
        /* 4-bit mask is only applicable for REG 0, 1, and 2. The rest are 2-bits */
        uint32_t gci_chipcontrol_mask = (gci.gci_chipcontrol_reg <= GCI_CHIPCONTROL_REG_2)
                                        ? GCI_PIN_MASK_4BITS : GCI_PIN_MASK_2BITS;
        uint32_t clr_val = gci_chipcontrol_mask << gci.gci_chipcontrol_pos;
        uint32_t set_val = ((uint32_t)gci_chipcontrol_mux << gci.gci_chipcontrol_pos) & clr_val;
        _cyhal_system_gci_chipcontrol(gci.gci_chipcontrol_reg, clr_val, set_val);
    }
}


/*******************************************************************************
*       Internal - Interrupts
*******************************************************************************/

bool _cyhal_system_irq_remap_sink( uint8_t bus_line_num, uint8_t sink_num )
{
    volatile uint32_t* oobselina = (volatile uint32_t*)( PLATFORM_APPSCR4_MASTER_WRAPPER_REGBASE(0x0) + ( ( sink_num < 4) ? AI_OOBSELINA30 : AI_OOBSELINA74 ) );
    int shift = ( sink_num % 4 ) * 8;

    if ( ( bus_line_num >= AI_OOBSEL_BUSLINE_COUNT ) || ( sink_num >= AI_OOBSEL_SINK_COUNT ) )
    {
        return false;
    }

    *oobselina = ( *oobselina & ~( (uint32_t)AI_OOBSEL_MASK << shift ) ) | ( (uint32_t)bus_line_num << shift );

    return true;
}

bool _cyhal_system_irq_remap_source( uint32_t wrapper_addr, uint8_t source_num, uint8_t bus_line_num )
{
    volatile uint32_t* oobselouta = (volatile uint32_t*)( wrapper_addr + ( ( source_num < 4) ? AI_OOBSELOUTA30 : AI_OOBSELOUTA74 ) );
    int shift = ( source_num % 4 ) * 8;

    if ( ( bus_line_num >= AI_OOBSEL_BUSLINE_COUNT ) || ( source_num >= AI_OOBSEL_SOURCE_COUNT ) )
    {
        return false;
    }

    *oobselouta = ( *oobselouta & ~( (uint32_t)AI_OOBSEL_MASK << shift ) ) | ( (uint32_t)bus_line_num << shift );

    return true;
}

/* Enable external interrupts from Timer to APPS Core */
void _cyhal_system_timer_enable_irq(void)
{
    bool status;
    /*
    * Let's have APPSCR4 timer and PMU interrupts (timer1 and event1) share same OOB line
    * and be delivered to same Timer_ExtIRQn interrupt status bit.
    *
    * First, let's make APPSCR4 timer output irq requests to same bus line as PMU timer1.
    */
    status = _cyhal_system_irq_remap_source(PLATFORM_APPSCR4_MASTER_WRAPPER_REGBASE(0x0),
                            OOB_APPSCR4_TIMER_IRQ_NUM,
                            OOB_AOUT_PMU_INTR1);
    CY_ASSERT(status);
    /* Second, let's route this bus line to Timer_ExtIRQn bit. */
    status = _cyhal_system_irq_remap_sink(OOB_AOUT_PMU_INTR1, Timer_ExtIRQn);
    CY_ASSERT(status);
    CY_UNUSED_PARAMETER(status);
    /* Third, enable this line. */
    platform_irq_enable_irq(Timer_ExtIRQn);
}

/* List of interrupt handlers to call in Timer_ISR */
extern void _cyhal_timer_irq_handler(void);
extern void _cyhal_lptimer_irq_handler(void);
#if defined WLAN_POWERSAVE_RES
extern void _cyhal_syspm_wlan_powersave_res_event(void);
#endif

void _cyhal_timer_isr(void)
{
    static pmu_ext_wakeup_t  _cyhal_timer_pmu_stored_ext_wakeup;

    uint32_t intr_status;
    uint32_t intr_mask;

    intr_status = PLATFORM_PMU->ext_wakeup_status.raw;
    intr_mask = PLATFORM_PMU->ext_wake_mask1.raw;
    if (intr_status & intr_mask)
    {
        /*
         * External wake-up event triggered.
         * We don't want to have storm of interrupts, so ack it here.
         * Wake-up event sets same bit in pmu intstatus register as timer.
         * So clear here wake-up bit and if pmu intstatus bit does not disappear -
         * it means we also have timer interrupt, and as result ISR will be re-triggered.
         *
         * For the wake-up event to be able to triggered again, it needs to be acked first.
         */
        _cyhal_timer_pmu_stored_ext_wakeup.raw  = intr_status & intr_mask;
        PLATFORM_PMU->ext_wakeup_status.raw = _cyhal_timer_pmu_stored_ext_wakeup.raw;
    }

    /* PMU timer reset */
    intr_status = PLATFORM_PMU->pmuintstatus.bits.rsrc_req_timer_int1;
    intr_mask = PLATFORM_PMU->pmuintmask1.bits.rsrc_req_timer_int1;
    if (intr_status && intr_mask)
    {
        _cyhal_lptimer_irq_handler();
    }

    /* CPU timer reset */
    intr_status = PLATFORM_APPSCR4->int_status;
    intr_mask = PLATFORM_APPSCR4->int_mask;
    if (intr_status & intr_mask & IRQN2MASK(Timer_IRQn))
    {
        _cyhal_timer_irq_handler();
    }

    /* WLAN resource event */
    intr_status = PLATFORM_PMU->pmuintstatus.bits.rsrc_event_int1;
    intr_mask = PLATFORM_PMU->pmuintmask1.bits.rsrc_event_int1;
    if (intr_status && intr_mask)
    {
        #if defined WLAN_POWERSAVE_RES
        _cyhal_syspm_wlan_powersave_res_event();
        #endif
    }
}

/* Map the Timer ISR to the interrupt vector */
PLATFORM_DEFINE_ISR(_cyhal_timer_isr);
PLATFORM_MAP_ISR(_cyhal_timer_isr, Timer_ISR);


/* List of interrupt handlers to call in ChipCommon_ISR */
extern void _cyhal_gpio_irq_handler(void);
extern void _cyhal_uart_irq_handler_fast(void);
extern void _cyhal_uart_irq_handler_dbg(void);
extern void _cyhal_uart_irq_handler_gci(void);

/* This is a shared ISR for chip common interrupts, based on platform_chipcommon.c */
void _cyhal_chipcommon_isr(void)
{
    /* ChipCommon IntStatus and IntMask register bits */
    #define _CYHAL_CC_INT_STATUS_MASK_GPIOINT             (1 << 0)
    #define _CYHAL_CC_INT_STATUS_MASK_EXTINT              (1 << 1)
    #define _CYHAL_CC_INT_STATUS_MASK_ECIGCIINT           (1 << 4)
    #define _CYHAL_CC_INT_STATUS_MASK_PMUINT              (1 << 5)
    #define _CYHAL_CC_INT_STATUS_MASK_UARTINT             (1 << 6)
    #define _CYHAL_CC_INT_STATUS_MASK_SECIGCIWAKEUPINT    (1 << 7)
    #define _CYHAL_CC_INT_STATUS_MASK_SPMINT              (1 << 8)
    #define _CYHAL_CC_INT_STATUS_MASK_ASCURXINT           (1 << 9)
    #define _CYHAL_CC_INT_STATUS_MASK_ASCUTXINT           (1 << 10)
    #define _CYHAL_CC_INT_STATUS_MASK_ASCUASTPINT         (1 << 11)

    uint32_t interrupt_mask   = PLATFORM_CHIPCOMMON->interrupt.mask.raw;
    uint32_t interrupt_status = PLATFORM_CHIPCOMMON->interrupt.status.raw;

    if (((interrupt_status & _CYHAL_CC_INT_STATUS_MASK_GPIOINT) != 0) && ((interrupt_mask & _CYHAL_CC_INT_STATUS_MASK_GPIOINT) != 0))
    {
        _cyhal_gpio_irq_handler();
    }
    else if (((interrupt_status & _CYHAL_CC_INT_STATUS_MASK_UARTINT) != 0) && ((interrupt_mask & _CYHAL_CC_INT_STATUS_MASK_UARTINT) != 0))
    {
        _cyhal_uart_irq_handler_dbg();
    }
    else if (((interrupt_status & _CYHAL_CC_INT_STATUS_MASK_ECIGCIINT) != 0) && ((interrupt_mask & _CYHAL_CC_INT_STATUS_MASK_ECIGCIINT) != 0))
    {
        /* With ECIGCI we can't tell if this is the fast UART or the debug UART, so call both and let the
         * handlers check their associated status registers to tell which one(s) should actually act */
         _cyhal_uart_irq_handler_fast();
         _cyhal_uart_irq_handler_gci();
    }
}

/* Map the ChipCommon ISR to the interrupt vector */
PLATFORM_DEFINE_ISR(_cyhal_chipcommon_isr);
PLATFORM_MAP_ISR(_cyhal_chipcommon_isr, ChipCommon_ISR);


/*******************************************************************************
*       Internal - HIB status
*******************************************************************************/

static uint32_t _cyhal_system_get_hib_status( uint32_t selector )
{
    _cyhal_system_gci_chipcontrol(GCI_CHIPCONTROL_HIB_READ_SEL_REG,
                                GCI_CHIPCONTROL_HIB_READ_SEL_MASK,
                                selector);
    return (_cyhal_system_gci_chipstatus(GCI_CHIPSTATUS_HIB_READ_REG)
                        & GCI_CHIPSTATUS_HIB_READ_MASK) >> GCI_CHIPSTATUS_HIB_READ_SHIFT;
}

static bool _cyhal_system_is_hib_wakeup(void)
{
    bool status = false;
    appscr4_core_status_reg_t appscr4_saved_core_status;
    appscr4_saved_core_status.raw = PLATFORM_APPSCR4->core_status.raw;

    if (!(appscr4_saved_core_status.bits.s_error_log || appscr4_saved_core_status.bits.s_bp_reset_log))
    {
        hib_status_t raw_status = {.raw =
            _cyhal_system_get_hib_status(GCI_CHIPCONTROL_HIB_READ_SEL_STATUS)};
        if (raw_status.bits.boot_from_wake != 0)
        {
            uint32_t hib_cnt =
            (_cyhal_system_get_hib_status( GCI_CHIPCONTROL_HIB_READ_SEL_COUNT_0_7   ) << 0  ) |
            (_cyhal_system_get_hib_status( GCI_CHIPCONTROL_HIB_READ_SEL_COUNT_15_8  ) << 8  ) |
            (_cyhal_system_get_hib_status( GCI_CHIPCONTROL_HIB_READ_SEL_COUNT_23_16 ) << 16 ) |
            (_cyhal_system_get_hib_status( GCI_CHIPCONTROL_HIB_READ_SEL_COUNT_31_24 ) << 24 );

            status = (hib_cnt != 0);
        }
    }

    return status;
}


/*******************************************************************************
*       Internal - Delays implementation
*******************************************************************************/

/* Set in clock driver */
extern uint32_t _cyhal_clock_cpu_clock_hz;

#define _CYHAL_SYSTEM_US_PER_SECOND         (1000000UL)
#define _CYHAL_SYSTEM_CYCLES_PER_US         (_cyhal_clock_cpu_clock_hz / _CYHAL_SYSTEM_US_PER_SECOND)
#define _CYHAL_SYSTEM_MS_MAX                (UINT32_MAX / (_CYHAL_SYSTEM_CYCLES_PER_US * 1000))

void _cyhal_system_delay_cycles(uint32_t cycles)
{
    __ASM volatile
    (
        "Cy_SysLib_DelayCycles:; "      /* cycles bytes */
            "ADDS r0, r0, #2 ;"         /*    1    2    Round to nearest multiple of 4 */
            "LSRS r0, r0, #2 ;"         /*    1    2    Divide by 4 and set flags */
            "BEQ Cy_DelayCycles_done ;" /*    2    2    Skip if 0 */

        /* The align below is added to ensure that the instructions in the loop are in the
        * same 8 byte block boundary so that they can be fetched as a single block from the
        * instruction cache.
        */
        ".align 3;"
        "Cy_DelayCycles_loop:;"
            "ADDS r0, r0, #1 ;"         /*    1    2    Increment counter */
            "SUBS r0, r0, #2 ;"         /*    1    2    Decrement counter by 2 */
            "BNE Cy_DelayCycles_loop ;" /*   (1)2  2    2 CPU cycles (if branch is taken) */
            "NOP ;"                     /*    1    2    Loop alignment padding */

        "Cy_DelayCycles_done: ;"
            "NOP ;"                     /*    1    2    Loop alignment padding */
            "BX lr ;"                   /*    3    2 */
    );
    CY_UNUSED_PARAMETER(cycles);
}


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

uint32_t cyhal_system_critical_section_enter(void)
{
    uint32_t state;
    PLATFORM_SAVE_INTERRUPTS(state);
    return state;
}

void cyhal_system_critical_section_exit(uint32_t old_state)
{
    PLATFORM_RESTORE_INTERRUPTS(old_state);
}

cy_rslt_t cyhal_system_delay_ms(uint32_t milliseconds)
{
#if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
    return cy_rtos_delay_milliseconds(milliseconds);
#else
    while (milliseconds > _CYHAL_SYSTEM_MS_MAX)
    {
        _cyhal_system_delay_cycles(UINT32_MAX);
        milliseconds -= _CYHAL_SYSTEM_MS_MAX;
    }
    _cyhal_system_delay_cycles(milliseconds * 1000 * _CYHAL_SYSTEM_CYCLES_PER_US);
    return CY_RSLT_SUCCESS;
#endif
}

void cyhal_system_delay_us(uint16_t microseconds)
{
    _cyhal_system_delay_cycles(microseconds * _CYHAL_SYSTEM_CYCLES_PER_US);
}

cyhal_reset_reason_t cyhal_system_get_reset_reason(void)
{
    cyhal_reset_reason_t reason = CYHAL_SYSTEM_RESET_NONE;

    if (PLATFORM_PMU->pmustatus & PST_WDRESET)
    {
        reason |= CYHAL_SYSTEM_RESET_WDT;
    }
    if (_cyhal_system_is_hib_wakeup())
    {
        reason |= CYHAL_SYSTEM_RESET_HIB_WAKEUP;
    }

    return reason;
}

void cyhal_system_clear_reset_reason(void)
{
    if (PLATFORM_PMU->pmustatus & PST_WDRESET)
    {
        PLATFORM_PMU->pmustatus = PST_WDRESET; /* Clear the reset flag */
    }
    PLATFORM_APPSCR4->core_status.raw = 0;
}

cy_rslt_t cyhal_system_set_isr(int32_t irq_num, int32_t irq_src, uint8_t priority, cyhal_irq_handler handler)
{
    /*
    * Note: All peripheral interrupts go though irq_vector_external_interrupt().
    *       This is demuxed and each interrupt handler is called in sequence
    *       according to interrupt_vector_table[].
    */
    CY_UNUSED_PARAMETER(irq_num);
    CY_UNUSED_PARAMETER(irq_src);
    CY_UNUSED_PARAMETER(priority);
    CY_UNUSED_PARAMETER(handler);
    return CYHAL_SYSTEM_RSLT_ERR_NOT_SUPPORTED;
}

#if defined(__cplusplus)
}
#endif
