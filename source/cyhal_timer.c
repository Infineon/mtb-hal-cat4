/***************************************************************************//**
* \file cyhal_timer.c
*
* Description:
* Provides a high level interface for interacting with the Timer.
*
********************************************************************************
* \copyright
* Copyright 2021-2022 Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cyhal_timer.h"
#include "cyhal_system.h"
#include "cyhal_hwmgr.h"
#include "cy_utils.h"
#include "platform_appscr4.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * \addtogroup group_hal_impl_timer Timer
 * \ingroup group_hal_impl
 * \{
 * \section section_hal_impl_timer_features Timer features
 *
 * The CAT4 Timer is an active power domain counter that supports the following features:
 * * 1 instance
 * * Up count
 * * Continuous and one-shot
 * * Compare event and terminal count event
 *
 * The following features are not available on these devices:
 * * Capture mode and capture event
 * * Down count and Up/down count
 *
 * The Timer hardware is always clocked using the CPU clock. NULL should be passed in
 * the initialization function for the clk parameter. There is only one instance of the
 * timer available on this device. If using an RTOS, it will be used as the tick source
 * and hence it is not available for generic application use.
 *
 * \} group_hal_impl_timer
 */

/*******************************************************************************
*       Internal - Tick CPU timer
*******************************************************************************/

/* Set in clock driver */
extern uint32_t _cyhal_clock_cpu_clock_hz;

/* Needed in syspm for tickless sleep */
cyhal_timer_t *_cyhal_timer_copy  = NULL;

#define _CYHAL_TIMER_MAX_HW_TICKS (0xFFFFFFFFU)

static inline void _cyhal_timer_write_sync(volatile uint32_t *reg, uint32_t val)
{
    *reg = val;
    (void)*reg; /* read back to ensure write completed */
}

// Because we can't change the actual frequency used by the timer hardware,
// we emulate frequency changes by converting between "HW ticks" used by
// the actual hardware and "FW ticks" provided by the user via
// cyhal_timer_configure().

static inline uint32_t _cyhal_timer_to_hw_ticks(const cyhal_timer_t *timer, uint32_t fw_ticks)
{
    return fw_ticks * timer->freq_ratio;
}

static inline uint32_t _cyhal_timer_to_fw_ticks(const cyhal_timer_t *timer, uint32_t hw_ticks)
{
    return hw_ticks / timer->freq_ratio;
}

// Takes HW ticks
static inline void _cyhal_timer_set_timer(uint32_t hw_ticks)
{
    _cyhal_timer_write_sync(&PLATFORM_APPSCR4->int_timer, hw_ticks);
}

// Returns HW ticks
static inline uint32_t _cyhal_timer_read_timer(void)
{
    return PLATFORM_APPSCR4->int_timer;
}

static inline void _cyhal_timer_clear_irq(void)
{
    PLATFORM_APPSCR4->int_status = IRQN2MASK(Timer_IRQn);
}

static inline void _cyhal_timer_reset(cyhal_timer_t *timer)
{
    // Skip compare phase if the compare value is equal to the starting value
    if (timer->hw_ticks_to_compare == 0)
    {
        timer->in_compare_phase = false;
        _cyhal_timer_set_timer(timer->hw_ticks_to_terminal);
    }
    else
    {
        timer->in_compare_phase = true;
        _cyhal_timer_set_timer(timer->hw_ticks_to_compare);
    }
}

static inline void _cyhal_timer_set_frequency(cyhal_timer_t *timer, uint32_t frequency)
{
    // Used to emulate frequency changes - see above comment and conversion functions.
    timer->freq_ratio = (_cyhal_clock_cpu_clock_hz / frequency);
    timer->max_fw_ticks = _cyhal_timer_to_fw_ticks(timer, _CYHAL_TIMER_MAX_HW_TICKS);
}

static inline void _cyhal_timer_recompute_hw_ticks(cyhal_timer_t *timer)
{
    timer->hw_ticks_to_compare = _cyhal_timer_to_hw_ticks(timer, timer->fw_ticks_to_compare - timer->fw_start_ticks);
    timer->hw_ticks_to_terminal = _cyhal_timer_to_hw_ticks(timer, timer->fw_tick_period - timer->fw_ticks_to_compare);
}

/*******************************************************************************
*       Internal - Interrupt
*******************************************************************************/

void _cyhal_timer_irq_handler(void)
{
    CY_ASSERT(NULL != _cyhal_timer_copy);

    _cyhal_timer_clear_irq();

    if (_cyhal_timer_copy->running)
    {
        if (NULL != _cyhal_timer_copy->callback_data.callback)
        {
            cyhal_timer_event_callback_t callback = (cyhal_timer_event_callback_t) _cyhal_timer_copy->callback_data.callback;

            if (_cyhal_timer_copy->in_compare_phase && _cyhal_timer_copy->compare_event_enabled)
            {
                (callback)(_cyhal_timer_copy->callback_data.callback_arg, CYHAL_TIMER_IRQ_CAPTURE_COMPARE);
            }
            else if (!_cyhal_timer_copy->in_compare_phase && _cyhal_timer_copy->terminal_event_enabled)
            {
                (callback)(_cyhal_timer_copy->callback_data.callback_arg, CYHAL_TIMER_IRQ_TERMINAL_COUNT);
            }
        }

        if (_cyhal_timer_copy->in_compare_phase)
        {
            _cyhal_timer_copy->in_compare_phase = false;
            _cyhal_timer_set_timer(_cyhal_timer_copy->hw_ticks_to_terminal);
        }
        else if (_cyhal_timer_copy->is_continuous)
        {
            _cyhal_timer_reset(_cyhal_timer_copy);
        }
        else
        {
            _cyhal_timer_copy->running = false;
        }
    }
}

/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_timer_init(cyhal_timer_t *obj, cyhal_gpio_t pin, const cyhal_clock_t *clk)
{
    CY_UNUSED_PARAMETER(pin);
    CY_UNUSED_PARAMETER(clk);
    CY_ASSERT(NULL != obj);

    cy_rslt_t rslt = cyhal_hwmgr_allocate(CYHAL_RSC_TIMER, &(obj->resource));

    if (CY_RSLT_SUCCESS == rslt)
    {
        obj->running                    = false;
        obj->ever_stopped               = false;
        obj->in_compare_phase           = true;
        obj->is_continuous              = true;
        obj->fw_start_ticks             = 0;
        obj->fw_ticks_to_compare        = _CYHAL_TIMER_MAX_HW_TICKS / 2;
        obj->fw_tick_period             = _CYHAL_TIMER_MAX_HW_TICKS;
        obj->compare_event_enabled      = false;
        obj->terminal_event_enabled     = false;
        obj->callback_data.callback     = NULL;
        obj->callback_data.callback_arg = NULL;

        _cyhal_timer_set_frequency(obj, CYHAL_TIMER_DEFAULT_FREQ);
        _cyhal_timer_recompute_hw_ticks(obj);
        obj->previous_stop_hw_ticks = obj->hw_ticks_to_compare;

        /* Keep a copy of the cpu timer for ISR */
        _cyhal_timer_copy = obj;

        /* Need to always enable the interrupt as the counter needs to be cleared manually at compare match */
        _cyhal_system_timer_enable_irq();
    }

    return rslt;
}

cy_rslt_t cyhal_timer_init_cfg(cyhal_timer_t *obj, const cyhal_timer_configurator_t *cfg)
{
    /* No configurators supported on this architecture */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);
    return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
}

void cyhal_timer_free(cyhal_timer_t *obj)
{
    if (CYHAL_RSC_INVALID != obj->resource.type)
    {
        cyhal_timer_stop(obj); // Only returns CY_RSLT_SUCCESS
        cyhal_hwmgr_free(&(obj->resource));
        obj->resource.type = CYHAL_RSC_INVALID;
        _cyhal_timer_copy = NULL;
    }
}

cy_rslt_t cyhal_timer_configure(cyhal_timer_t *obj, const cyhal_timer_cfg_t *cfg)
{
    cy_rslt_t status = CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;

    if ((cfg->direction == CYHAL_TIMER_DIR_UP)
        && (cfg->is_compare)
        && (cfg->compare_value >= cfg->value)
        && (cfg->compare_value < cfg->period)
        && (cfg->period <= obj->max_fw_ticks))
    {
        obj->is_continuous = cfg->is_continuous;
        obj->fw_start_ticks = cfg->value;

        obj->fw_ticks_to_compare = cfg->compare_value;
        obj->fw_tick_period = cfg->period;

        _cyhal_timer_recompute_hw_ticks(obj);
        obj->previous_stop_hw_ticks = obj->hw_ticks_to_compare;
        obj->ever_stopped = false;

        status = CY_RSLT_SUCCESS;
    }

    return status;
}

cy_rslt_t cyhal_timer_set_frequency(cyhal_timer_t *obj, uint32_t hz)
{
    if ((hz == 0) || (hz > _cyhal_clock_cpu_clock_hz))
        return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;

    // Convert to FW ticks first so that we can convert back to HW ticks with the new frequency
    uint32_t current_fw_ticks = _cyhal_timer_to_fw_ticks(obj, obj->running ? _cyhal_timer_read_timer() : obj->previous_stop_hw_ticks);

    uint32_t old_freq_ratio = obj->freq_ratio;
    uint32_t old_max_fw_ticks = obj->max_fw_ticks;
    _cyhal_timer_set_frequency(obj, hz);
    if (obj->max_fw_ticks < obj->fw_tick_period)
    {
        obj->freq_ratio = old_freq_ratio;
        obj->max_fw_ticks = old_max_fw_ticks;
        return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
    }

    _cyhal_timer_recompute_hw_ticks(obj);

    if (obj->running)
        _cyhal_timer_set_timer(_cyhal_timer_to_hw_ticks(obj, current_fw_ticks));
    else
        obj->previous_stop_hw_ticks = _cyhal_timer_to_hw_ticks(obj, current_fw_ticks);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_timer_start(cyhal_timer_t *obj)
{
    if (obj->running)
    {
        _cyhal_timer_reset(obj);
    }
    else
    {
        _cyhal_timer_clear_irq();

        platform_irq_enable_int(Timer_IRQn);

        obj->running = true;

        /* if timer previous_stop_hw_ticks==0, timer will not trigger */
        if ( (obj->ever_stopped) && (obj->previous_stop_hw_ticks != 0) )
        {
            _cyhal_timer_set_timer(obj->previous_stop_hw_ticks);
        }
        else
        {
            _cyhal_timer_reset(obj);
        }
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_timer_stop(cyhal_timer_t *obj)
{
    obj->running = false;
    obj->ever_stopped = true;
    obj->previous_stop_hw_ticks = _cyhal_timer_read_timer();

    platform_irq_disable_int(Timer_IRQn);

    _cyhal_timer_set_timer(0);

    _cyhal_timer_clear_irq();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_timer_reset(cyhal_timer_t *obj)
{
    _cyhal_timer_clear_irq();
    _cyhal_timer_reset(obj);
    obj->ever_stopped = false;
    return CY_RSLT_SUCCESS;
}

uint32_t cyhal_timer_read(const cyhal_timer_t *obj)
{
    uint32_t current_hw_ticks = obj->running ? _cyhal_timer_read_timer() : obj->previous_stop_hw_ticks;
    if (obj->in_compare_phase)
        return _cyhal_timer_to_fw_ticks(obj, obj->hw_ticks_to_compare - current_hw_ticks) + obj->fw_start_ticks;
    else
        return _cyhal_timer_to_fw_ticks(obj, obj->hw_ticks_to_terminal - current_hw_ticks) + obj->fw_ticks_to_compare;
}

void cyhal_timer_register_callback(cyhal_timer_t *obj, cyhal_timer_event_callback_t callback, void *callback_arg)
{
    CY_ASSERT(CYHAL_RSC_INVALID != obj->resource.block_num);

    uint32_t savedIntrStatus = cyhal_system_critical_section_enter();
    obj->callback_data.callback = (cy_israddress) callback;
    obj->callback_data.callback_arg = callback_arg;
    cyhal_system_critical_section_exit(savedIntrStatus);
}

void cyhal_timer_enable_event(cyhal_timer_t *obj, cyhal_timer_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(intr_priority);

    if (event & CYHAL_TIMER_IRQ_CAPTURE_COMPARE)
    {
        obj->compare_event_enabled = enable;
    }

    if (event & CYHAL_TIMER_IRQ_TERMINAL_COUNT)
    {
        obj->terminal_event_enabled = enable;
    }
}

cy_rslt_t cyhal_timer_connect_digital(cyhal_timer_t *obj, cyhal_source_t source, cyhal_timer_input_t signal)
{
    /* Not supported */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(signal);
    return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
}

cy_rslt_t cyhal_timer_connect_digital2(cyhal_timer_t *obj, cyhal_source_t source, cyhal_timer_input_t signal, cyhal_edge_type_t edge_type)
{
    /* Not supported */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(signal);
    CY_UNUSED_PARAMETER(edge_type);
    return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
}

cy_rslt_t cyhal_timer_enable_output(cyhal_timer_t *obj, cyhal_timer_output_t signal, cyhal_source_t *source)
{
    /* Not supported */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(signal);
    CY_UNUSED_PARAMETER(source);

    return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
}

cy_rslt_t cyhal_timer_disconnect_digital(cyhal_timer_t *obj, cyhal_source_t source, cyhal_timer_input_t signal)
{
    /* Not supported */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(signal);

    return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
}

cy_rslt_t cyhal_timer_disable_output(cyhal_timer_t *obj, cyhal_timer_output_t signal)
{
    /* Not supported */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(signal);

    return CYHAL_TIMER_RSLT_ERR_BAD_ARGUMENT;
}

#if defined(__cplusplus)
}
#endif
