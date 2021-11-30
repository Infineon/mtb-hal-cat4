/***************************************************************************//**
* \file cyhal_pwm.c
*
* Description:
* Provides a high level interface for interacting with the PWM.
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

#include "cyhal_pwm.h"
#include "cyhal_hwmgr.h"
#include "cyhal_utils.h"
#include "cyhal_system.h"
#include "cyhal_syspm.h"
#include "cyhal_clock.h"
#include "cyhal_pin_package.h"
#include "cy_utils.h"
#include "platform_appscr4.h"

#include "hndsoc.h"
#include "wiced_osl.h"

#if defined(__cplusplus)
extern "C" {
#endif


/**
 * \addtogroup group_hal_impl_pwm PWM (Pulse Width Modulator)
 * \ingroup group_hal_impl
 * \{
 * \section section_hal_impl_pwm_compl_pins Complementary PWM output
 *
 * In order to setup complimentary output on the PWM using the HAL,
 * a pin that connects to the i+1 instance must be passed into
 * \ref cyhal_pwm_init_adv where i is the instance that the
 * non-complimentary pin connects to. \ref cyhal_pwm_init_adv will
 * return an error if the pin that connects to i+1 instance is not
 * passed or if the i+1 instance is not available or cannot be
 * reserved.
 *
 * Internally, the HAL reserves the i+1 instance and the hardware then
 * copies over the settings of the i instance to the i+1 instance
 * with the output inverted on the i+1 instance. As a result, two PWM
 * instances (i, i+1) will be consumed when the complimentary output
 * is setup on this platform.
 *
 * \note Both instances (i, i+1) operate in a synchronized manner i.e.
 * any setting and operation (start,stop etc.) on the PWM instance i
 * will also have the same effect on the i+1 instance
 *
 * \note Complimentary output is not supported on the last PWM
 * instance as there is no i+1 instance available.
 *
 * \note The PWM signal on instance i cannot be inverted. Hence, the
 * invert parameter in \ref cyhal_pwm_init_adv is not supported
 * and must always be false on this platform.
 *
 * \} group_hal_impl_pwm
 */

/*******************************************************************************
*       Internal
*******************************************************************************/

#define _CYHAL_PWM_INSTANCES             (6U)
#define _CYHAL_PWM_US_PER_SECOND         (1000000UL)
#define _CYHAL_PWM_PERIOD_MAX            (0xFFFFUL)
#define _CYHAL_PWM_PERIOD_MIN            (0U)

static bool pwm_core_initialized = false;
static cyhal_clock_block_t pwm_core_clock;

static volatile pwm_channel_t* _cyhal_pwm_get_channel(cyhal_resource_inst_t pwm_rsc)
{
    return (pwm_rsc.block_num < _CYHAL_PWM_INSTANCES) ? &PLATFORM_CHIPCOMMON->pwm.pwm_channels[pwm_rsc.block_num] : NULL;
}

static uint32_t _cyhal_pwm_get_clock_frequency(void)
{
    uint32_t frequency;

    frequency = ((_cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_PWM_CLK_SLOW_REG, 0x0, 0x0) & PMU_CHIPCONTROL_PWM_CLK_SLOW_MASK) == 0)
        ? cyhal_clock_get_frequency(&CYHAL_CLOCK_BACKPLANE)
        : cyhal_clock_get_frequency(&CYHAL_CLOCK_ALP);

    return frequency;
}

static void _cyhal_pwm_enable_clock(bool enable)
{
    uint32_t flags = cyhal_system_critical_section_enter();
    if (enable)
    {
        _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_PWM_CLK_EN_REG,
                                    PMU_CHIPCONTROL_PWM_CLK_EN_MASK,
                                    PMU_CHIPCONTROL_PWM_CLK_EN_MASK);
        cyhal_system_delay_us(10); /* let it stabilize */
    }
    else
    {
        _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_PWM_CLK_EN_REG,
                                    PMU_CHIPCONTROL_PWM_CLK_EN_MASK,
                                    0x0);
    }
    cyhal_system_critical_section_exit(flags);
}

static void _cyhal_pwm_stop_channel(volatile pwm_channel_t* chan)
{
    pwm_channel_ctrl_t* ctrl_ptr = (pwm_channel_ctrl_t *)&(chan->ctrl.raw);
    pwm_channel_ctrl_t ctrl = *ctrl_ptr;
    ctrl.bits.enable = 0;
    ctrl.bits.update = 0;
    ctrl.bits.update_all = 0;
    chan->ctrl.raw = ctrl.raw;
}


/*******************************************************************************
*       Internal - LPM
*******************************************************************************/

#define _CYHAL_PWM_PM_MASK           (0x01UL)
static uint32_t _cyhal_pwm_pm_state = 0UL;

static bool _cyhal_pwm_pm_has_enabled(void)
{
    for (uint8_t idx = 0; idx < _CYHAL_PWM_INSTANCES; idx++)
    {
        if ((_cyhal_pwm_pm_state >> idx) & _CYHAL_PWM_PM_MASK)
            return true;
    }
    return false;
}

static bool _cyhal_pwm_pm_transition_pending_value = false;

static bool _cyhal_pwm_pm_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);

    switch(mode)
    {
        case CYHAL_SYSPM_CHECK_FAIL:
        case CYHAL_SYSPM_AFTER_TRANSITION:
            _cyhal_pwm_pm_transition_pending_value = false;
            break;
        case CYHAL_SYSPM_CHECK_READY:
            for (uint8_t ch = 0; ch < _CYHAL_PWM_INSTANCES; ch++)
            {
                if (PLATFORM_CHIPCOMMON->pwm.pwm_channels[ch].ctrl.bits.enable != 0)
                    return false;
            }
            _cyhal_pwm_pm_transition_pending_value = true;
            break;
        default:
            break;
    }

    return true;
}

static cyhal_syspm_callback_data_t _cyhal_pwm_syspm_callback_data =
{
    .callback = &_cyhal_pwm_pm_callback,
    .states = (cyhal_syspm_callback_state_t)(CYHAL_SYSPM_CB_CPU_DEEPSLEEP | CYHAL_SYSPM_CB_SYSTEM_HIBERNATE),
    .next = NULL,
    .args = NULL,
    .ignore_modes = CYHAL_SYSPM_BEFORE_TRANSITION,
};


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_pwm_init_adv(cyhal_pwm_t *obj, cyhal_gpio_t pin, cyhal_gpio_t compl_pin,
                            cyhal_pwm_alignment_t pwm_alignment, bool continuous, uint32_t dead_time_us,
                            bool invert, const cyhal_clock_t *clk)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    volatile pwm_channel_t* chan = NULL;
    memset(obj, 0, sizeof(*obj));

    obj->resource.type = CYHAL_RSC_INVALID;
    obj->pin = NC;
    obj->compl_resource.type = CYHAL_RSC_INVALID;
    obj->compl_pin = NC;

    const cyhal_resource_pin_mapping_t* compl_map = NULL;
    const cyhal_resource_pin_mapping_t* map = _CYHAL_UTILS_TRY_ALLOC(pin, CYHAL_RSC_PWM, cyhal_pin_map_pwm_line);

    // Setting the invert bit on the last PWM channel causes the channel to
    // remain high (100% duty cycle). Based on this it is assumed that complimentary
    // pin is not supported on the last pwm instance.
    if (invert || map == NULL || (map != NULL && compl_pin != NC && map->block_num == (_CYHAL_PWM_INSTANCES - 1)))
    {
        status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
    }
    else
    {
        CY_ASSERT(map->block_num <= (_CYHAL_PWM_INSTANCES - 1));
        _CYHAL_UTILS_ASSIGN_RESOURCE(obj->resource, CYHAL_RSC_PWM, map);
        obj->dead_time_us = dead_time_us;

        if (CY_RSLT_SUCCESS == status)
        {
            cyhal_resource_inst_t pin_rsc = { CYHAL_RSC_GPIO, pin, 0 };
            status = cyhal_hwmgr_reserve(&pin_rsc);
            if (CY_RSLT_SUCCESS == status)
            {
                obj->pin = pin;
                chan = _cyhal_pwm_get_channel(obj->resource);
                if (chan == NULL)
                {
                    status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
                }
            }
        }
    }

    /* Setup complimentary pin and instance */
    if (CY_RSLT_SUCCESS == status && compl_pin != NC)
    {
        // If compl_pin is set we must try to reserve the next pwm instance
        // that will be consumed for the complimentary output.
        cyhal_resource_inst_t desired_pwm_inst = { CYHAL_RSC_PWM, map->block_num + 1, 0 };
        compl_map = _cyhal_utils_get_resource(compl_pin, cyhal_pin_map_pwm_line,
                    sizeof(cyhal_pin_map_pwm_line)/sizeof(cyhal_resource_pin_mapping_t), &desired_pwm_inst);
        if (compl_map != NULL)
        {
            status = cyhal_hwmgr_reserve(&desired_pwm_inst);
            if (CY_RSLT_SUCCESS == status)
            {
                CY_ASSERT(compl_map->block_num <= (_CYHAL_PWM_INSTANCES - 1));
                obj->compl_resource = desired_pwm_inst;
            }
        }
        else
        {
            status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
        }

        if (CY_RSLT_SUCCESS == status && compl_pin != NC)
        {
            cyhal_resource_inst_t compl_pin_rsc = { CYHAL_RSC_GPIO, compl_pin, 0 };
            status = cyhal_hwmgr_reserve(&compl_pin_rsc);
            if (CY_RSLT_SUCCESS == status)
            {
                obj->compl_pin = compl_pin;
            }
        }
    }

    /* Initialize core */
    if ((CY_RSLT_SUCCESS == status) && !pwm_core_initialized)
    {
        if (clk == NULL)
        {
            pwm_core_clock = CYHAL_CLOCK_BLOCK_ALP; // Default clock
        }
        else
        {
            if ((clk->block == CYHAL_CLOCK_BLOCK_ALP) || (clk->block == CYHAL_CLOCK_BLOCK_BACKPLANE))
                pwm_core_clock = clk->block;
            else
                status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
        }

        if (CY_RSLT_SUCCESS == status)
        {
            uint32_t flags = cyhal_system_critical_section_enter();
            osl_core_enable(CC_CORE_ID); /* PWM is in chipcommon. Enable core before try to access. */

            // Set clock source
            if ((pwm_core_clock == CYHAL_CLOCK_BLOCK_BACKPLANE))
            {
                _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_PWM_CLK_SLOW_REG,
                                        PMU_CHIPCONTROL_PWM_CLK_SLOW_MASK,
                                        0x0);
            }
            else if (pwm_core_clock == CYHAL_CLOCK_BLOCK_ALP)
            {
                _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_PWM_CLK_SLOW_REG,
                                        PMU_CHIPCONTROL_PWM_CLK_SLOW_MASK,
                                        PMU_CHIPCONTROL_PWM_CLK_SLOW_MASK);
            }
            cyhal_system_critical_section_exit(flags);
        }

        pwm_core_initialized = true;
    }

    if (status == CY_RSLT_SUCCESS)
    {
        _cyhal_system_pinmux_connect(map->pin, map->function);

        if(compl_pin != NC)
        {
            _cyhal_system_pinmux_connect(compl_map->pin, compl_map->function);
        }

        /* Configure default timing */
        chan->dead_time.raw      = 0;
        chan->cycle_cnt.raw      = 0;

        /* Configure behavior */
        pwm_channel_ctrl_t ctrl = {0};
        ctrl.bits.is_single_shot = continuous ? 0 : 1;
        ctrl.bits.invert         = (compl_pin != NC) ? 1 : 0;
        ctrl.bits.alignment      = (pwm_channel_ctrl_alignment_t)pwm_alignment;
        chan->ctrl.raw           = ctrl.raw;

        if (!_cyhal_pwm_pm_has_enabled())
        {
            _cyhal_syspm_register_peripheral_callback(&_cyhal_pwm_syspm_callback_data);
        }
        _cyhal_pwm_pm_state |= (_CYHAL_PWM_PM_MASK << obj->resource.block_num);
    }

    if (status != CY_RSLT_SUCCESS)
    {
        cyhal_pwm_free(obj);
    }

    return status;
}

cy_rslt_t cyhal_pwm_init_cfg(cyhal_pwm_t *obj, const cyhal_pwm_configurator_t *cfg)
{
    /* No configurators supported on this architecture */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);
    return CYHAL_PWM_RSLT_BAD_ARGUMENT;
}

void cyhal_pwm_free(cyhal_pwm_t *obj)
{
    CY_ASSERT(NULL != obj);

    cyhal_pwm_stop(obj);
    _cyhal_pwm_pm_state &= ~(_CYHAL_PWM_PM_MASK << obj->resource.block_num);

    if (!_cyhal_pwm_pm_has_enabled())
    {
        _cyhal_syspm_unregister_peripheral_callback(&_cyhal_pwm_syspm_callback_data);
    }

    _cyhal_utils_release_if_used(&obj->pin);
    cyhal_hwmgr_free(&(obj->resource));
    obj->resource.type = CYHAL_RSC_INVALID;
    if (obj->compl_pin != NC)
    {
        cyhal_hwmgr_free(&(obj->compl_resource));
        obj->compl_resource.type = CYHAL_RSC_INVALID;
    }
    _cyhal_utils_release_if_used(&obj->compl_pin);
}

cy_rslt_t cyhal_pwm_set_period(cyhal_pwm_t *obj, uint32_t period_us, uint32_t pulse_width_us)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    float duty_cycle = ((float)pulse_width_us/(float)period_us) * 100;
    uint32_t frequencyhal_hz = _CYHAL_PWM_US_PER_SECOND / period_us;
    status = cyhal_pwm_set_duty_cycle(obj, duty_cycle, frequencyhal_hz);

    return status;
}

cy_rslt_t cyhal_pwm_set_duty_cycle(cyhal_pwm_t *obj, float duty_cycle, uint32_t frequencyhal_hz)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    volatile pwm_channel_t* chan;

    chan = _cyhal_pwm_get_channel(obj->resource);
    if (chan == NULL)
    {
        status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
    }
    else
    {
        uint32_t high_period = 0, low_period = 0;
        uint32_t clock_freq = _cyhal_pwm_get_clock_frequency();
        if (duty_cycle > 0)
        {
            uint32_t total_period = (clock_freq + (frequencyhal_hz >> 1)) / frequencyhal_hz;
            high_period  =  (uint32_t)(MIN(total_period, (total_period * duty_cycle / 100.0f + 0.5f)));
            low_period   =  total_period - high_period;
        }

        if ((high_period > _CYHAL_PWM_PERIOD_MAX) || (low_period > _CYHAL_PWM_PERIOD_MAX))
        {
            status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
        }
        else
        {
            pwm_channel_dead_time_t dead_time = {0};
            if (duty_cycle > 0)
            {
                /* Configure dead time. */
                uint32_t dead_time_period = (obj->dead_time_us) * clock_freq / _CYHAL_PWM_US_PER_SECOND;
                dead_time.bits.cycle_dead_high = dead_time_period;
                dead_time.bits.cycle_dead_low = dead_time_period;
            }
            chan->dead_time.raw = dead_time.raw;

            pwm_channel_cycle_cnt_t cycle_cnt = {0};
            if (duty_cycle > 0)
            {
                /* Configure cycle timings. */
                cycle_cnt.bits.low_cycle_time   = low_period;
                cycle_cnt.bits.high_cycle_time  = high_period;
            }
            chan->cycle_cnt.raw = cycle_cnt.raw;

            /* Only setting the update bit does not seem to work reliably so we
            * stop and start the channel if it was enabled before. This
            * method for updating to the new register configuration works reliably.
            */
            if (duty_cycle > 0)
            {
                bool is_enabled = obj->is_enabled;
                status = cyhal_pwm_stop(obj);
                if (status == CY_RSLT_SUCCESS && is_enabled)
                {
                    status = cyhal_pwm_start(obj);
                }
            }
            else
            {
               _cyhal_pwm_stop_channel(chan);
            }
        }
    }

    return status;
}

cy_rslt_t cyhal_pwm_start(cyhal_pwm_t *obj)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    volatile pwm_channel_t* chan;
    pwm_channel_ctrl_t      ctrl;

    chan = _cyhal_pwm_get_channel(obj->resource);
    if (chan == NULL)
    {
        status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
    }
    else
    {
        if (_cyhal_pwm_pm_transition_pending_value)
        {
            return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
        }
        else
        {
            _cyhal_pwm_enable_clock(true);
            pwm_channel_ctrl_t* ctrl_ptr = (pwm_channel_ctrl_t *)&(chan->ctrl.raw);
            ctrl = *ctrl_ptr;
            ctrl.bits.enable         = 1;
            ctrl.bits.update         = 1;
            ctrl.bits.update_all     = 0;
            chan->ctrl.raw           = ctrl.raw;
            obj->is_enabled          = true;
        }
    }

    return status;
}

cy_rslt_t cyhal_pwm_stop(cyhal_pwm_t *obj)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    volatile pwm_channel_t* chan;

    chan = _cyhal_pwm_get_channel(obj->resource);
    if (chan == NULL)
    {
        status = CYHAL_PWM_RSLT_BAD_ARGUMENT;
    }
    else
    {
        _cyhal_pwm_stop_channel(chan);
        _cyhal_pwm_enable_clock(false);
        obj->is_enabled          = false;
    }

    return status;
}

void cyhal_pwm_register_callback(cyhal_pwm_t *obj, cyhal_pwm_event_callback_t callback, void *callback_arg)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(callback);
    CY_UNUSED_PARAMETER(callback_arg);
}

void cyhal_pwm_enable_event(cyhal_pwm_t *obj, cyhal_pwm_event_t event, uint8_t intr_priority, bool enable)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(event);
    CY_UNUSED_PARAMETER(intr_priority);
    CY_UNUSED_PARAMETER(enable);
}

cy_rslt_t cyhal_pwm_connect_digital(cyhal_pwm_t *obj, cyhal_source_t source, cyhal_pwm_input_t signal)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(signal);
    return CYHAL_PWM_RSLT_BAD_ARGUMENT;
}

cy_rslt_t cyhal_pwm_enable_output(cyhal_pwm_t *obj, cyhal_pwm_output_t signal, cyhal_source_t *source)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(signal);
    CY_UNUSED_PARAMETER(source);
    return CYHAL_PWM_RSLT_BAD_ARGUMENT;
}

cy_rslt_t cyhal_pwm_disconnect_digital(cyhal_pwm_t *obj, cyhal_source_t source, cyhal_pwm_input_t signal)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(signal);
    return CYHAL_PWM_RSLT_BAD_ARGUMENT;
}

cy_rslt_t cyhal_pwm_disable_output(cyhal_pwm_t *obj, cyhal_pwm_output_t signal)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(signal);
    return CYHAL_PWM_RSLT_BAD_ARGUMENT;
}

#if defined(__cplusplus)
}
#endif
