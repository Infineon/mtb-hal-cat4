/***************************************************************************//**
* \file cyhal_lptimer.c
*
* \brief
* Provides a high level interface for interacting with the Cypress Low-Power Timer.
* This interface abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
*
********************************************************************************
* \copyright
* Copyright 2018-2022 Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cyhal_lptimer.h"
#include "cyhal_hwmgr.h"
#include "cyhal_system.h"
#include "cy_utils.h"
#include "platform_appscr4.h"

#include "typedefs.h"
#include "sbchipc.h"
#include "wiced_osl.h"

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
*       Internal - Tick PMU timer
*******************************************************************************/

/* Set in clock driver */
extern uint32_t _cyhal_clock_cpu_clock_hz;

static cyhal_lptimer_t* _cyhal_lptimer_copy  = NULL;

#define _CYHAL_LPTIMER_MIN_DELAY_TICKS       (1u)
#define _CYHAL_LPTIMER_MAX_COUNTER           (0x00FFFFFF)

static inline bool _cyhal_lptimer_slow_write_pending(void)
{
    return PLATFORM_PMU->pmustatus & PST_SLOW_WR_PENDING;
}

static void _cyhal_lptimer_slow_write_barrier(void)
{
    /*
     * Ensures that the previous write to slow (ILP) register is completed.
     * Write is placed in intermediate buffer and returns immediately.
     * Then it takes few slow ILP clocks to move the write from intermediate
     * buffer to actual register. If subsequent write happens before previous is
     * moved to actual register, then second write may take long time and also
     * occupy the bus. This busy loop tries to ensure that previous write is completed.
     * It can be used to avoid long writes, and also as synchronization barrier
     * to ensure that previous register update is completed.
     */

    // Theoretically required timeout
    uint32_t timeout = _cyhal_clock_cpu_clock_hz/osl_ilp_clock();
    uint32_t count;

    for (count = timeout; count > 0; count--)
    {
        if(!_cyhal_lptimer_slow_write_pending())
            break;
    }

    CY_ASSERT(count != 0);
}

static void _cyhal_lptimer_slow_write(volatile uint32_t *reg, uint32_t val)
{
    *reg = val;
    _cyhal_lptimer_slow_write_barrier();
}

static inline void _cyhal_lptimer_set_pmu_timer(uint32_t value)
{
    _cyhal_lptimer_slow_write(&PLATFORM_PMU->pmutimer, value);
}

static inline uint32_t _cyhal_lptimer_read_pmu_timer(void)
{
    return PLATFORM_PMU->pmutimer;
}

const pmu_intstatus_t _cyhal_lptimer_intstatus_init = { .bits.rsrc_req_timer_int1 = 1 };

static inline void _cyhal_lptimer_clear_irq(void)
{
    PLATFORM_PMU->pmuintstatus.raw = _cyhal_lptimer_intstatus_init.raw;
}

static pmu_res_req_timer_t _cyhal_lptimer_res_timer(uint32_t ticks)
{
    pmu_res_req_timer_t timer;

    timer.raw                   = 0x0;
    timer.bits.time             = (ticks & _CYHAL_LPTIMER_MAX_COUNTER);
    timer.bits.int_enable       = 1;
    timer.bits.force_ht_request = 0;
    timer.bits.clkreq_group_sel = pmu_res_clkreq_apps_group;

    return timer;
}

static void _cyhal_lptimer_set_resource_timer(uint32_t ticks)
{
    _cyhal_lptimer_slow_write(&PLATFORM_PMU->res_req_timer1.raw, _cyhal_lptimer_res_timer(ticks).raw);
}

static void _cyhal_lptimer_start(cyhal_lptimer_t *timer)
{
    _cyhal_system_common_chipcontrol(&PLATFORM_PMU->pmuintmask1.raw, 0x0, _cyhal_lptimer_intstatus_init.raw);
    timer->started = true;
}

static void _cyhal_lptimer_stop(void)
{
    _cyhal_system_common_chipcontrol(&PLATFORM_PMU->pmuintmask1.raw, _cyhal_lptimer_intstatus_init.raw, 0x0);
    _cyhal_lptimer_slow_write(&PLATFORM_PMU->res_req_timer1.raw, _cyhal_lptimer_res_timer(0x0).raw);
}

static void _cyhal_lptimer_set_delay(cyhal_lptimer_t *timer, uint32_t delay)
{
    _cyhal_lptimer_set_resource_timer(delay);
    _cyhal_system_timer_enable_irq();
    if (!timer->started)
        _cyhal_lptimer_start(timer);
}


/*******************************************************************************
*       Internal - Interrupt
*******************************************************************************/

void _cyhal_lptimer_irq_handler(void)
{
    CY_ASSERT(NULL != _cyhal_lptimer_copy);

    // Expected behavior after calling cyhal_lptimer_set_match() is for the
    // timer to roll-over and the interrupt to fire on the next match, i.e.
    // after a full timer period. Since the resource timers don't roll over,
    // we reset the timer to its maximum value to obtain the same behavior.
    if (_cyhal_lptimer_copy->matching)
        _cyhal_lptimer_set_resource_timer(_CYHAL_LPTIMER_MAX_COUNTER);

    if ((NULL != _cyhal_lptimer_copy->callback_data.callback)
        && (_cyhal_lptimer_copy->compare_event_enabled))
    {
        cyhal_lptimer_event_callback_t callback = (cyhal_lptimer_event_callback_t) _cyhal_lptimer_copy->callback_data.callback;
        (callback)(_cyhal_lptimer_copy->callback_data.callback_arg, CYHAL_LPTIMER_COMPARE_MATCH);
    }

    _cyhal_lptimer_clear_irq();
}

cyhal_lptimer_t *_cyhal_lptimer_get_ptr(void)
{
	return _cyhal_lptimer_copy;
}

/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_lptimer_init(cyhal_lptimer_t *obj)
{
    CY_ASSERT(NULL != obj);

    cy_rslt_t rslt = cyhal_hwmgr_allocate(CYHAL_RSC_LPTIMER, &(obj->resource));

    if (CY_RSLT_SUCCESS == rslt)
    {
        obj->started                    = false;
        obj->matching                   = false;
        obj->reset_ticks                = 0;
        obj->compare_event_enabled      = false;
        obj->callback_data.callback     = NULL;
        obj->callback_data.callback_arg = NULL;

        /* Keep a copy of the pmu timer for ISR */
        _cyhal_lptimer_copy = obj;
    }

    return rslt;
}

void cyhal_lptimer_free(cyhal_lptimer_t *obj)
{
    if (CYHAL_RSC_INVALID != obj->resource.type)
    {
        _cyhal_lptimer_stop();
        cyhal_hwmgr_free(&(obj->resource));
        obj->resource.type = CYHAL_RSC_INVALID;
        _cyhal_lptimer_copy = NULL;
    }
}

cy_rslt_t cyhal_lptimer_reload(cyhal_lptimer_t *obj)
{
    _cyhal_lptimer_set_pmu_timer(0);
    if (obj->started)
        _cyhal_lptimer_set_resource_timer(obj->reset_ticks);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_lptimer_set_match(cyhal_lptimer_t *obj, uint32_t value)
{
    cy_rslt_t status = CYHAL_LPTIMER_RSLT_ERR_BAD_ARGUMENT;

    if (value <= _CYHAL_LPTIMER_MAX_COUNTER)
    {
        obj->matching = true;
        obj->reset_ticks = value;
        _cyhal_lptimer_set_delay(obj, value - _cyhal_lptimer_read_pmu_timer());
        status = CY_RSLT_SUCCESS;
    }

    return status;
}

cy_rslt_t cyhal_lptimer_set_delay(cyhal_lptimer_t *obj, uint32_t delay)
{
    cy_rslt_t status = CYHAL_LPTIMER_RSLT_ERR_BAD_ARGUMENT;

    if (delay >= _CYHAL_LPTIMER_MIN_DELAY_TICKS)
    {
        obj->matching = false;
        obj->reset_ticks = delay;
        _cyhal_lptimer_set_delay(obj, delay);
        status = CY_RSLT_SUCCESS;
    }

    return status;
}

uint32_t cyhal_lptimer_read(const cyhal_lptimer_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    return _cyhal_lptimer_read_pmu_timer() & _CYHAL_LPTIMER_MAX_COUNTER;
}

void cyhal_lptimer_register_callback(cyhal_lptimer_t *obj, cyhal_lptimer_event_callback_t callback, void *callback_arg)
{
    CY_ASSERT(CYHAL_RSC_INVALID != obj->resource.block_num);

    uint32_t savedIntrStatus = cyhal_system_critical_section_enter();
    obj->callback_data.callback = (cy_israddress) callback;
    obj->callback_data.callback_arg = callback_arg;
    cyhal_system_critical_section_exit(savedIntrStatus);
}

void cyhal_lptimer_enable_event(cyhal_lptimer_t *obj, cyhal_lptimer_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(intr_priority);

    CY_ASSERT(event == CYHAL_LPTIMER_COMPARE_MATCH);
    obj->compare_event_enabled = enable;
}

void cyhal_lptimer_irq_trigger(cyhal_lptimer_t *obj)
{
    CY_ASSERT(CYHAL_RSC_INVALID != obj->resource.block_num);
    if (obj->callback_data.callback)
    {
        cyhal_lptimer_event_callback_t callback = (cyhal_lptimer_event_callback_t) obj->callback_data.callback;
        (callback)(obj->callback_data.callback_arg, CYHAL_LPTIMER_COMPARE_MATCH);
    }
}

void cyhal_lptimer_get_info(cyhal_lptimer_t *obj, cyhal_lptimer_info_t *info)
{
    CY_UNUSED_PARAMETER(obj);
    CY_ASSERT(info != NULL);

    info->frequency_hz = osl_ilp_clock();
    info->min_set_delay = _CYHAL_LPTIMER_MIN_DELAY_TICKS;
    info->max_counter_value = _CYHAL_LPTIMER_MAX_COUNTER;
}

#if defined(__cplusplus)
}
#endif
