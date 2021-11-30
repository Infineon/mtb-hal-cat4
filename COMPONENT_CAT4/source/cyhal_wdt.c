/***************************************************************************//**
* \file cyhal_wdt.c
*
* Description:
* Provides a high level interface for interacting with the WDT.
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

#include "cyhal_wdt.h"
#include "cyhal_clock.h"
#include "cy_utils.h"
#include "platform_appscr4.h"

#include "typedefs.h"
#include "sbchipc.h"

#include "wiced_osl.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
*       Internal
*******************************************************************************/

#define _CYHAL_WDT_MAX_RESOLUTION       (28u)
#define _CYHAL_WDT_MAX_TIMEOUT_MS       (uint32_t)(1UL << _CYHAL_WDT_MAX_RESOLUTION)

uint32_t _cyhal_wdt_initial_timeout_ms;
uint32_t _cyhal_wdt_reset_ticks;
bool _cyhal_wdt_initialized;
bool _cyhal_wdt_enabled;

static void _cyhal_wdt_wait_for_slow_write(void)
{
    for (uint16_t count = 0xFFFF; count > 0; count--)
    {
        if (!(PLATFORM_PMU->pmustatus & PST_SLOW_WR_PENDING))
            break;
    }
}

static void _cyhal_wdt_set(uint32_t timeout_ticks)
{
    _cyhal_wdt_wait_for_slow_write();
    PLATFORM_PMU->pmuwatchdog = timeout_ticks;
}


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_wdt_init(cyhal_wdt_t *obj, uint32_t timeout_ms)
{
    CY_UNUSED_PARAMETER(obj);
    if ((timeout_ms == 0) || (timeout_ms > _CYHAL_WDT_MAX_TIMEOUT_MS))
        return CY_RSLT_WDT_INVALID_TIMEOUT;
    if (_cyhal_wdt_initialized)
        return CY_RSLT_WDT_ALREADY_INITIALIZED;

    _cyhal_wdt_initial_timeout_ms = timeout_ms;
    _cyhal_wdt_initialized = true;

    const uint32_t ms_ticks_per_second = _cyhal_wdt_initial_timeout_ms * osl_ilp_clock();
    _cyhal_wdt_reset_ticks = ms_ticks_per_second / 1000;
    const uint32_t remainder = ms_ticks_per_second % 1000;
    if (remainder != 0)
        _cyhal_wdt_reset_ticks++;

    cyhal_wdt_start(obj);

    return CY_RSLT_SUCCESS;
}

void cyhal_wdt_free(cyhal_wdt_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    _cyhal_wdt_initial_timeout_ms = 0UL;
    cyhal_wdt_stop(obj);
    _cyhal_wdt_initialized = false;
}

void cyhal_wdt_kick(cyhal_wdt_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    if(_cyhal_wdt_initialized)
    {
        _cyhal_wdt_set(_cyhal_wdt_reset_ticks);
        _cyhal_wdt_wait_for_slow_write();
    }
}

void cyhal_wdt_start(cyhal_wdt_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    if(_cyhal_wdt_initialized)
    {
        cyhal_wdt_kick(obj);
        _cyhal_wdt_enabled = true;
    }
}

void cyhal_wdt_stop(cyhal_wdt_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    if(_cyhal_wdt_initialized)
    {
        _cyhal_wdt_set(0);
        _cyhal_wdt_enabled = false;
    }
}

uint32_t cyhal_wdt_get_timeout_ms(cyhal_wdt_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    return _cyhal_wdt_initial_timeout_ms;
}

uint32_t cyhal_wdt_get_max_timeout_ms(void)
{
    return (_CYHAL_WDT_MAX_TIMEOUT_MS / osl_ilp_clock()) * 1000;
}

bool cyhal_wdt_is_enabled(cyhal_wdt_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    return _cyhal_wdt_initialized && _cyhal_wdt_enabled;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#ifdef CYHAL_GPIO_IMPL_HEADER
#include CYHAL_GPIO_IMPL_HEADER
#endif /* CYHAL_GPIO_IMPL_HEADER */

/** \} group_hal_gpio */
