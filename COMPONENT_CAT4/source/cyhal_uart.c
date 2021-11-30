/***************************************************************************//**
* \file cyhal_uart.c
*
* Description:
* Provides a high level interface for interacting with the UART.
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

/**
 * \addtogroup group_hal_impl_uart UART
 * \ingroup group_hal_impl
 * \{
 * \section cyhal_uart_impl_features
 * The CAT4 UART supports the following features:
 * * Data bits: 8
 * * Stop bits: 1 or 2
 * * Parity: Even, odd, or none
 * * FIFO Depth: 64
 *
 * On CAT4 devices, there are three types of UART: Fast, Debug (also referred to as "Slow") and SECI
 * (also referred to as GCI). There are slight differences in functionality between the three:
 * * Only the Fast UART supports flow control
 * * The Fast UART can be driven by either the ALP clock or the dedicated "Fast UART" clock. The other UARTs
 *   can only be driven by the ALP clock.
 * * The Debug UART does not support setting arbitrary RX FIFO trigger levels. The level must be
 *   one of: 2, 1/4 full + 1, 1/2 full + 1, or full - 1.
 * * The Debug UART does not support TX FIFO trigger level interrupts (but it does support the "TX Empty" event)
 *
 * The following events are supported:
 * * \ref CYHAL_UART_IRQ_TX_TRANSMIT_IN_FIFO
 * * \ref CYHAL_UART_IRQ_RX_DONE
 * * \ref CYHAL_UART_IRQ_TX_FIFO (except Debug UART)
 * * \ref CYHAL_UART_IRQ_TX_EMPTY (Debug UART only)
 * * \ref CYHAL_UART_IRQ_RX_FIFO
 *
 * See the device datasheet for information about which UARTs can connect to which pins.
 *
 * \} group_hal_impl_uart
 */

#include <stdlib.h>
#include <string.h> // For memset
#include "cyhal_clock.h"
#include "cyhal_gpio.h"
#include "cyhal_hwmgr.h"
#include "cyhal_uart.h"
#include "cyhal_utils.h"
#include "cyhal_syspm.h"
#include "cyhal_system_impl.h"
#include "cyhal_uart_registers.h"
#include "ring_buffer.h"

#include "hndsoc.h"
#include "wiced_osl.h"

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
*       Internal - General
*******************************************************************************/

#define _CYHAL_UART_FAST (0u)
#define _CYHAL_UART_DBG  (1u)
#define _CYHAL_UART_GCI  (2u)
#define _CYHAL_UART_NUM_INSTANCES (3u)

#define _CYHAL_UART_FIFO_DEPTH (64u)

#define _CYHAL_UART_SECI_HIGH_RATE_THRESHOLD_LOW        (0xF1)
#define _CYHAL_UART_SECI_HIGH_RATE_THRESHOLD_HIGH       (0xF8)

#define _CYHAL_UART_SECI_BAUD_RATE_THRESHOLD_LOW        (9600)
#define _CYHAL_UART_SECI_ALP_CLOCK_DEFAULT              (37400000)
#define _CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT        (160000000)
#define _CYHAL_UART_SECI_BAUD_RATE_DIVISOR_MAX          (255)
#define _CYHAL_UART_SECI_BAUD_RATE_DIVISOR_RANGE        (UART_SECI_BAUD_RATE_DIVISOR_MAX + 1)
#define _CYHAL_UART_SECI_BAUD_RATE_ADJUSTMENT_MAX       (15)
#define _CYHAL_UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE     (UART_SECI_BAUD_RATE_ADJUSTMENT_MAX + 1)

/* Internal helpers */

static cy_rslt_t _cyhal_uart_init_fast(cyhal_uart_t *obj, const cyhal_uart_cfg_t *cfg);
static cy_rslt_t _cyhal_uart_init_dbg(cyhal_uart_t *obj, const cyhal_uart_cfg_t *cfg);
static cy_rslt_t _cyhal_uart_init_gci(cyhal_uart_t *obj, const cyhal_uart_cfg_t *cfg);

static cy_rslt_t _cyhal_uart_debug_config(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg);
static cy_rslt_t _cyhal_uart_seci_config(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg);

static _cyhal_uart_seci_t* _cyhal_uart_get_seci_base(cyhal_uart_t* obj);
static cy_rslt_t _cyhal_uart_seci_set_baud(cyhal_uart_t* obj, uint32_t desired_baud, uint32_t *actual_baud);
static cy_rslt_t _cyhal_uart_debug_set_baud(cyhal_uart_t* obj, uint32_t desired_baud, uint32_t* actual_baud);

static void _cyhal_uart_seci_set_flow_control(cyhal_uart_t* obj, bool cts, bool rts);

bool _cyhal_uart_fifo_readable(cyhal_uart_t* obj);
static uint8_t _cyhal_uart_read_from_fifo(cyhal_uart_t *obj);
static void _cyhal_uart_write_to_fifo(cyhal_uart_t *obj, uint8_t data);
static uint32_t _cyhal_uart_get_trigger_level(cyhal_uart_t *obj, bool rx_trigger);
static cy_rslt_t _cyhal_uart_update_fifo_level(cyhal_uart_t* obj, cyhal_uart_fifo_type_t type);
static uint16_t _cyhal_uart_closest_fifo_match(cyhal_uart_t* obj, cyhal_uart_fifo_type_t type, uint16_t level);
static void _cyhal_uart_update_enabled_events(cyhal_uart_t *obj);
static void _cyhal_uart_invoke_user_callback(cyhal_uart_t* obj, cyhal_uart_event_t event);

typedef struct
{
    uint32_t clock_hz;              /* Clock value in HZ */
    uint32_t desired_baud_rate;     /* Desired baud-rate in BPS */
    uint32_t target_baud_rate;      /* Target baud-rate in BPS */
} _cyhal_uart_seci_baud_rate_config_t;

/* SECI UART baud-rate mapping table for fast lookup of target baud-rate */
static const _cyhal_uart_seci_baud_rate_config_t _cyhal_uart_seci_baud_rate_mapping[] =
{
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       9600,    9599},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       14400,   14401},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       19200,   19199},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       38400,   38398},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       57600,   57627},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       115200,  115076},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       230400,  230864},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       460800,  461728},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       921600,  912195},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       1000000, 1010810},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       1500000, 1496000},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       2000000, 1968421},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       2500000, 2493333},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       3000000, 3116666},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       3500000, 3400000},
    {_CYHAL_UART_SECI_ALP_CLOCK_DEFAULT,       4000000, 4155555},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 57600,   57595},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 115200,  115190},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 230400,  230547},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 460800,  461095},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 921600,  919540},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 1000000, 1000000},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 1500000, 1495327},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 2000000, 2000000},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 2500000, 2500000},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 3000000, 3018867},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 3500000, 3478260},
    {_CYHAL_UART_SECI_BACKPLANE_CLOCK_DEFAULT, 4000000, 4000000}
};

static cyhal_uart_t* _cyhal_uart_config_structs[_CYHAL_UART_NUM_INSTANCES] = { NULL };


/*******************************************************************************
*       Internal - LPM
*******************************************************************************/

static bool _cyhal_uart_pm_has_enabled(void)
{
    for (uint8_t group = 0; group < _CYHAL_UART_NUM_INSTANCES; group++)
    {
        if (_cyhal_uart_config_structs[group] != NULL)
            return true;
    }
    return false;
}

static bool _cyhal_uart_pm_transition_pending_value = false;

static bool _cyhal_uart_pm_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);

    switch(mode)
    {
        case CYHAL_SYSPM_CHECK_FAIL:
        case CYHAL_SYSPM_AFTER_TRANSITION:
            for (uint8_t group = 0; group < _CYHAL_UART_NUM_INSTANCES; group++)
            {
                if (_cyhal_uart_config_structs[group] != NULL)
                {
                    if (_cyhal_uart_config_structs[group]->pin_rts != NC)
                    {
                        /* Note: Re-enable AUTO_RTS if it was set up before */
                        _cyhal_uart_seci_set_flow_control(_cyhal_uart_config_structs[group],
                                                          _cyhal_uart_config_structs[group]->enable_cts,
                                                          _cyhal_uart_config_structs[group]->enable_rts);
                    }
                }
            }
            _cyhal_uart_pm_transition_pending_value = false;
            break;
        case CYHAL_SYSPM_BEFORE_TRANSITION:
            for (uint8_t group = 0; group < _CYHAL_UART_NUM_INSTANCES; group++)
            {
                if (_cyhal_uart_config_structs[group] != NULL)
                {
                    if (_cyhal_uart_config_structs[group]->pin_rts != NC)
                    {
                        /* Note: AUTO_RTS is explicitly disabled before deep-sleep as
                        * this asserts RTS Low, indicating readiness to accept data.
                        * When waking up from Deep-Sleep, this causes problems, as UART is not fully
                        * initialized at this time, and interrupts are disabled, so any incoming data
                        * might be lost.
                        */
                        _cyhal_uart_seci_set_flow_control(_cyhal_uart_config_structs[group],
                                                          _cyhal_uart_config_structs[group]->enable_cts, false);
                    }
                }
            }
            break;
        case CYHAL_SYSPM_CHECK_READY:
            for (uint8_t group = 0; group < _CYHAL_UART_NUM_INSTANCES; group++)
            {
                if (_cyhal_uart_config_structs[group] != NULL)
                {
                    if (cyhal_uart_is_tx_active(_cyhal_uart_config_structs[group])
                    || cyhal_uart_is_rx_active(_cyhal_uart_config_structs[group]))
                        return false;
                }
            }
            _cyhal_uart_pm_transition_pending_value = true;
            break;
        default:
            break;
    }

    return true;
}

static cyhal_syspm_callback_data_t _cyhal_uart_syspm_callback_data =
{
    .callback = &_cyhal_uart_pm_callback,
    .states = (cyhal_syspm_callback_state_t)(CYHAL_SYSPM_CB_CPU_DEEPSLEEP | CYHAL_SYSPM_CB_SYSTEM_HIBERNATE),
    .next = NULL,
    .args = NULL,
    .ignore_modes = (cyhal_syspm_callback_mode_t)0,
};


/*******************************************************************************
*       HAL implementation
*******************************************************************************/

cy_rslt_t cyhal_uart_init(cyhal_uart_t *obj, cyhal_gpio_t tx, cyhal_gpio_t rx, cyhal_gpio_t cts, cyhal_gpio_t rts, const cyhal_clock_t *clk, const cyhal_uart_cfg_t *cfg)
{
    CY_ASSERT(NULL != obj);

    memset(obj, 0, sizeof(cyhal_uart_t));
    obj->resource.type = CYHAL_RSC_INVALID;
    obj->pin_rx  = NC;
    obj->pin_tx  = NC;
    obj->pin_rts = NC;
    obj->pin_cts = NC;
    obj->enable_rts = false;
    obj->enable_cts = false;

    const cyhal_resource_pin_mapping_t *rx_map = _CYHAL_UTILS_TRY_ALLOC(rx, CYHAL_RSC_UART, cyhal_pin_map_uart_rx);

    if (NULL != rx_map)
    {
        /* Note now that we reserved as part of alloc so that we can free if something fails later */
        _CYHAL_UTILS_ASSIGN_RESOURCE(obj->resource, CYHAL_RSC_UART, rx_map);
    }

    if(obj->resource.block_num != _CYHAL_UART_FAST && (NC != cts || NC != rts))
    {
        /* Only fast UART supports flow control */
        return CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    const cyhal_resource_pin_mapping_t *tx_map = _cyhal_utils_get_resource(tx, cyhal_pin_map_uart_tx,
        sizeof(cyhal_pin_map_uart_tx) / sizeof(cyhal_pin_map_uart_tx[0]), &(obj->resource));
    const cyhal_resource_pin_mapping_t *cts_map = (NC != cts) ? _cyhal_utils_get_resource(cts, cyhal_pin_map_uart_cts,
        sizeof(cyhal_pin_map_uart_cts) / sizeof(cyhal_pin_map_uart_cts[0]), &(obj->resource)) : NULL;
    const cyhal_resource_pin_mapping_t *rts_map = (NC != rts) ? _cyhal_utils_get_resource(rts, cyhal_pin_map_uart_rts,
        sizeof(cyhal_pin_map_uart_rts) / sizeof(cyhal_pin_map_uart_rts[0]), &(obj->resource)) : NULL;

    cy_rslt_t result = CY_RSLT_SUCCESS;
    if (NULL == rx_map || NULL == tx_map || (tx_map->block_num != rx_map->block_num) ||
       (NC != cts && (NULL == cts_map || tx_map->block_num != cts_map->block_num)) ||
       (NC != rts && (NULL == rts_map || tx_map->block_num != rts_map->block_num)))
    {
        result = CYHAL_UART_RSLT_ERR_INVALID_PIN;
    }

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_resource_inst_t rxRsc = { CYHAL_RSC_GPIO, rx, 0 };
        result = cyhal_hwmgr_reserve(&rxRsc);
        if (CY_RSLT_SUCCESS == result)
        {
            obj->pin_rx = rx;
        }
    }

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_resource_inst_t txRsc = { CYHAL_RSC_GPIO, tx, 0 };
        result = cyhal_hwmgr_reserve(&txRsc);
        if (CY_RSLT_SUCCESS == result)
        {
            obj->pin_tx = tx;
        }
    }
    if (CY_RSLT_SUCCESS == result)
    {
        _cyhal_system_pinmux_connect(rx_map->pin, rx_map->function);
        _cyhal_system_pinmux_connect(tx_map->pin, tx_map->function);
    }
    if ((CY_RSLT_SUCCESS == result) && (NC != cts))
    {
        cyhal_resource_inst_t ctsRsc = { CYHAL_RSC_GPIO, cts, 0 };
        result = cyhal_hwmgr_reserve(&ctsRsc);
        if(CY_RSLT_SUCCESS == result)
        {
            obj->pin_cts = cts;
            _cyhal_system_pinmux_connect(cts_map->pin, cts_map->function);
        }
    }
    if ((CY_RSLT_SUCCESS == result) && (NC != rts))
    {
        cyhal_resource_inst_t rtsRsc = { CYHAL_RSC_GPIO, rts, 0 };
        result = cyhal_hwmgr_reserve(&rtsRsc);
        if(CY_RSLT_SUCCESS == result)
        {
            obj->pin_rts = rts;
            _cyhal_system_pinmux_connect(rts_map->pin, rts_map->function);
        }
    }
    if (CY_RSLT_SUCCESS == result)
    {
        if (clk == NULL)
        {
            /* Default clock is ALP */
            result = cyhal_clock_get(&obj->clk, &CYHAL_CLOCK_RSC_ALP);
        }
        else
        {
            obj->clk = *clk;
        }
    }

    if (CY_RSLT_SUCCESS == result)
    {
        const cyhal_uart_cfg_t DEFAULT_CONFIG =
        {
            .data_bits = 8,
            .stop_bits = 1,
            .parity = CYHAL_UART_PARITY_NONE,
            .rx_buffer = NULL,
            .rx_buffer_size = 0
        };

        const cyhal_uart_cfg_t* config_to_use = (cfg != NULL) ? cfg : &DEFAULT_CONFIG;
        switch(obj->resource.block_num)
        {
            case _CYHAL_UART_FAST:
                result = _cyhal_uart_init_fast(obj, config_to_use);
                break;
            case _CYHAL_UART_DBG:
                result = _cyhal_uart_init_dbg(obj, config_to_use);
                break;
            case _CYHAL_UART_GCI:
                result = _cyhal_uart_init_gci(obj, config_to_use);
                break;
            default:
                CY_ASSERT(false);
                break;
        }
    }

    /* Default both level interrupts to half-full/half-empty */
    if(CY_RSLT_SUCCESS == result)
    {
        /* Slow UART actually needs depth / 2 + 1 due to >= vs > difference. Closest match handles that */
        uint16_t level = _cyhal_uart_closest_fifo_match(obj, CYHAL_UART_FIFO_RX, _CYHAL_UART_FIFO_DEPTH / 2);
        result = cyhal_uart_set_fifo_level(obj, CYHAL_UART_FIFO_RX, level);
    }
    if(CY_RSLT_SUCCESS == result && _CYHAL_UART_DBG != obj->resource.block_num) /* Slow UART doesn't support this */
    {
        result = cyhal_uart_set_fifo_level(obj, CYHAL_UART_FIFO_TX, _CYHAL_UART_FIFO_DEPTH / 2);
    }

    if(CY_RSLT_SUCCESS == result)
    {
        if (!_cyhal_uart_pm_has_enabled())
        {
            _cyhal_syspm_register_peripheral_callback(&_cyhal_uart_syspm_callback_data);
        }
        _cyhal_uart_config_structs[obj->resource.block_num] = obj;
    }

    if ((CY_RSLT_SUCCESS == result) && (NC != cts || NC != rts))
    {
        _cyhal_uart_seci_set_flow_control(obj, NC != cts, NC != rts);
    }

    if(CY_RSLT_SUCCESS != result)
    {
        cyhal_uart_free(obj);
    }

    return result;
}

cy_rslt_t cyhal_uart_init_cfg(cyhal_uart_t *obj, const cyhal_uart_configurator_t *cfg)
{
    /* No configurators supported on this architecture */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);
    return CYHAL_UART_RSLT_ERR_UNSUPPORTED_OPERATION;
}

void cyhal_uart_free(cyhal_uart_t *obj)
{
    CY_ASSERT(NULL != obj);

    if(CYHAL_RSC_INVALID != obj->resource.type)
    {
        switch(obj->resource.block_num)
        {
            case _CYHAL_UART_FAST:
                /* Disable interrupts from fast UART during deinitialization */
                _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), UART_FAST_CC_INT_STATUS_MASK, 0x0u);
                /* Disable receive-enable here at first */
                _CYHAL_UART_FAST_BASE->lcr &= ~UART_SECI_LCR_RX_EN;
                _CYHAL_UART_FAST_BASE->mcr &= ~UART_SECI_MCR_AUTO_RTS;

                /* Set linebreak to set SECI is not ready to communicate */
                CHIPCOMMON_SECI_UART_LCR_REG |= 0x10;

                /* Set force-low bit: we're not ready for data as we are deinitting */
                CHIPCOMMON_SECI_CONFIG_REG |= CC_SECI_ENAB_SECIOUT_DIS;
                break;
            case _CYHAL_UART_DBG:
                /* Disable interrupts from slow UART during deinitialization */
                _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), UART_SLOW_CC_INT_STATUS_MASK, 0x0u);
                /* Disable FIFO */
                _CYHAL_UART_SLOW_BASE->iir_fcr &= ~UART_SLOW_FCR_FIFO_ENABLE;
                break;
            case _CYHAL_UART_GCI:
                /* Disable interrupts from GCI UART during deinitialization */
                _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), UART_GCI_CC_INT_STATUS_MASK, 0x0u);
                /* Set SECI output force-low */
                GCI_CORE_CTRL_REG |= (GCI_CORE_CTRL_FORCE_SECI_OUT_LOW);

                /* Disable the SECI UART TX state machine */
                GCI_SECI_OUT_TX_ENAB_TX_BRK_REG &= ~(GCI_SECI_TX_ENAB);

                /* Disable UART bytes reception from SECI_IN port to RX FIFO */
                GCI_SECI_IN_AUX_FIFO_RX_ENAB_REG &= ~(GCI_SECI_FIFO_RX_ENAB);

                /*Disable SECI data communication */
                GCI_CORE_CTRL_REG &= ~(GCI_CORE_CTRL_ENABLE_SECI);
                break;
        }

        if(NULL != obj->buffer.buffer)
        {
            ring_buffer_deinit(&obj->buffer);
        }

        _cyhal_uart_config_structs[obj->resource.block_num] = NULL;

        if (!_cyhal_uart_pm_has_enabled())
        {
            _cyhal_syspm_unregister_peripheral_callback(&_cyhal_uart_syspm_callback_data);
        }

        cyhal_hwmgr_free(&(obj->resource));
        obj->resource.type = CYHAL_RSC_INVALID;
    }

    _cyhal_utils_release_if_used(&(obj->pin_rx));
    _cyhal_utils_release_if_used(&(obj->pin_tx));
    _cyhal_utils_release_if_used(&(obj->pin_rts));
    _cyhal_utils_release_if_used(&(obj->pin_cts));
}

cy_rslt_t cyhal_uart_set_baud(cyhal_uart_t *obj, uint32_t baudrate, uint32_t *actualbaud)
{
    CY_ASSERT(NULL != obj);
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
        case _CYHAL_UART_GCI:
            /* SECI refers to the type of UART hardware that both Fast and GCI use */
            return _cyhal_uart_seci_set_baud(obj, baudrate, actualbaud);
        case _CYHAL_UART_DBG:
            return _cyhal_uart_debug_set_baud(obj, baudrate, actualbaud);
        default:
            CY_ASSERT(false);
            return CYHAL_UART_RSLT_ERR_UNSUPPORTED_OPERATION;
    }
}

cy_rslt_t cyhal_uart_configure(cyhal_uart_t *obj, const cyhal_uart_cfg_t *cfg)
{
    CY_ASSERT(NULL != obj);
    cy_rslt_t result = CY_RSLT_SUCCESS;
    result = (obj->resource.block_num == _CYHAL_UART_DBG)
        ? _cyhal_uart_debug_config(obj, cfg)
        : _cyhal_uart_seci_config(obj, cfg); /* Fast and GCI both use SECI-type UART hardware */
    return result;
}

cy_rslt_t cyhal_uart_getc(cyhal_uart_t *obj, uint8_t *value, uint32_t timeout)
{
    CY_ASSERT(NULL != obj);

    if (_cyhal_uart_pm_transition_pending_value)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;

    uint32_t timeoutTicks = timeout;
    while (0u == cyhal_uart_readable(obj))
    {
        if(timeout != 0UL)
        {
            if(timeoutTicks > 0UL)
            {
                cyhal_system_delay_ms(1);
                timeoutTicks--;
            }
            else
            {
                return CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT;
            }
        }
    }
    *value = _cyhal_uart_read_from_fifo(obj);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_uart_putc(cyhal_uart_t *obj, uint32_t value)
{
    CY_ASSERT(NULL != obj);

    if (_cyhal_uart_pm_transition_pending_value)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    
    while (0u == cyhal_uart_writable(obj)) { }
    _cyhal_uart_write_to_fifo(obj, value);
    return CY_RSLT_SUCCESS;
}
uint32_t cyhal_uart_readable(cyhal_uart_t *obj)
{
    CY_ASSERT(NULL != obj);
    uint32_t count = 0;
    if(NULL != obj->buffer.buffer)
    {
        count += ring_buffer_used_space(&obj->buffer);
    }
    /* No way to query the exact fifo consumption, so we just add 1 if non-empty.
     * It's better to under count than over count. */
    if(_cyhal_uart_fifo_readable(obj))
    {
        ++count;
    }
    return count;
}

uint32_t cyhal_uart_writable(cyhal_uart_t *obj)
{
    CY_ASSERT(NULL != obj);
    /* For slow fifo, we can only check THRE, which contrary to the name does not tell us whether the
     * FIFO is empty, but rather whether it is below the configured threshold. So we return 0 or the
     * configured TX FIFO trigger level. For the other two, we can only check if it's not full,
     * so we return either 0 or 1. Better to under count than over count. */
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            return (0u == (CHIPCOMMON_SECI_STATUS_REG & CC_SECI_STATUS_TX_FIFO_FULL)) ? 1 : 0;
        case _CYHAL_UART_DBG:
            if(0u != (_CYHAL_UART_SLOW_BASE->lsr & UART_SLOW_LSR_THRE))
            {
                return _CYHAL_UART_FIFO_DEPTH - _cyhal_uart_get_trigger_level(obj, false);
            }
            else if(0u != (_CYHAL_UART_SLOW_BASE->lsr & UART_SLOW_LSR_TEMT))
            {
                return _CYHAL_UART_FIFO_DEPTH;
            }
            else
            {
                return 0u;
            }
        case _CYHAL_UART_GCI:
            return (0u == (GCI_INT_STATUS_REG & GCI_INT_ST_MASK_SECI_TX_FIFO_FULL)) ? 1 : 0;
        default:
            CY_ASSERT(false);
            return 0u;
    }
}

cy_rslt_t cyhal_uart_clear(cyhal_uart_t *obj)
{
    /* No direct clear option available, so just read until the FIFO is empty */
    while(_cyhal_uart_fifo_readable(obj))
    {
        (void)_cyhal_uart_read_from_fifo(obj);
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_uart_enable_flow_control(cyhal_uart_t *obj, bool enable_cts, bool enable_rts)
{
    CY_ASSERT(NULL != obj);
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if(obj->resource.block_num != _CYHAL_UART_FAST)
    {
        /* Only fast UART supports flow control */
        result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    if(CY_RSLT_SUCCESS == result)
    {
        _cyhal_uart_seci_set_flow_control(obj, enable_cts, enable_rts);
    }

    return result;
}

cy_rslt_t cyhal_uart_write(cyhal_uart_t *obj, void *tx, size_t *tx_length)
{
    CY_ASSERT(NULL != obj);

    if (_cyhal_uart_pm_transition_pending_value)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    
    uint8_t* tx_cast = (uint8_t*)tx;
    size_t num_written = 0;

    while(cyhal_uart_writable(obj) > 0 && num_written < *tx_length)
    {
        _cyhal_uart_write_to_fifo(obj, *tx_cast);
        ++num_written;
        ++tx_cast;
    }
    *tx_length = num_written;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_uart_read(cyhal_uart_t *obj, void *rx, size_t *rx_length)
{
    if (_cyhal_uart_pm_transition_pending_value)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    
    /* Read from the ring buffer first, if it's present */
    uint8_t* rx_cast = (uint8_t*)rx;
    uint32_t num_read = 0;
    if(NULL != obj->buffer.buffer)
    {
        ring_buffer_read(&obj->buffer, rx_cast, *rx_length, &num_read);
        rx_cast += num_read;
    }
    /* Read from fifo if we still have data left to read */
    while(num_read < *rx_length && _cyhal_uart_fifo_readable(obj))
    {
        *rx_cast = _cyhal_uart_read_from_fifo(obj);
        ++rx_cast;
        ++num_read;
    }
    *rx_length = (uint32_t)num_read;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_uart_write_async(cyhal_uart_t *obj, void *tx, size_t length)
{
    if (_cyhal_uart_pm_transition_pending_value)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    
    size_t length_written = length;
    cy_rslt_t result = cyhal_uart_write(obj, tx, &length_written);
    if(CY_RSLT_SUCCESS == result)
    {
        size_t remaining = length - length_written;
        if(remaining > 0)
        {
            obj->async_tx_buff = (void*)((size_t)tx + length_written);
            obj->async_tx_length = remaining;
            result = _cyhal_uart_update_fifo_level(obj, CYHAL_UART_FIFO_TX);
            if(CY_RSLT_SUCCESS == result)
            {
                _cyhal_uart_update_enabled_events(obj);
            }
        }
        else
        {
            _cyhal_uart_invoke_user_callback(obj, CYHAL_UART_IRQ_TX_TRANSMIT_IN_FIFO);
        }
    }

    return result;
}

cy_rslt_t cyhal_uart_read_async(cyhal_uart_t *obj, void *rx, size_t length)
{
    if (_cyhal_uart_pm_transition_pending_value)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    
    size_t length_read = length;
    cy_rslt_t result = cyhal_uart_read(obj, rx, &length_read);
    if(CY_RSLT_SUCCESS == result)
    {
        size_t remaining = length - length_read;
        if(remaining > 0)
        {
            obj->async_rx_buff = (void *)((size_t)rx + length_read);
            obj->async_rx_length = remaining;
            result = _cyhal_uart_update_fifo_level(obj, CYHAL_UART_FIFO_RX);
            if(CY_RSLT_SUCCESS == result)
            {
                _cyhal_uart_update_enabled_events(obj);
            }
        }
        else
        {
            _cyhal_uart_invoke_user_callback(obj, CYHAL_UART_IRQ_RX_DONE);
        }
    }
    return result;
}

bool cyhal_uart_is_tx_active(cyhal_uart_t *obj)
{
    CY_ASSERT(NULL != obj);
    bool active = NULL != obj->async_tx_buff;
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            active |= (0u == (_CYHAL_UART_FAST_BASE->lsr & UART_SECI_LSR_TI_MASK));
            break;
        case _CYHAL_UART_DBG:
            active |= (0u == (_CYHAL_UART_SLOW_BASE->lsr & UART_SLOW_LSR_TEMT));
            break;
        case _CYHAL_UART_GCI:
            active |= (0u == (GCI_SECI_OUT_TX_STATUS & GCI_TX_STATUS_TX_IDLE));
            break;
        default:
            CY_ASSERT(false);
            active = false;
            break;
    }
    return active;
}

bool cyhal_uart_is_rx_active(cyhal_uart_t *obj)
{
    CY_ASSERT(NULL != obj);
    return NULL != obj->async_rx_buff;
}

cy_rslt_t cyhal_uart_write_abort(cyhal_uart_t *obj)
{
    obj->async_tx_buff = NULL;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_uart_read_abort(cyhal_uart_t *obj)
{
    obj->async_rx_buff = NULL;
    return CY_RSLT_SUCCESS;
}

void cyhal_uart_register_callback(cyhal_uart_t *obj, cyhal_uart_event_callback_t callback, void *callback_arg)
{
    uint32_t flags = cyhal_system_critical_section_enter();
    obj->callback_data.callback = (cy_israddress) callback;
    obj->callback_data.callback_arg = callback_arg;
    cyhal_system_critical_section_exit(flags);
}

void cyhal_uart_enable_event(cyhal_uart_t *obj, cyhal_uart_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(intr_priority); /* Not supported on 43907 */
    if(enable)
    {
        obj->user_enabled_events |= (uint32_t)event;
    }
    else
    {
        obj->user_enabled_events &= ~((uint32_t)event);
    }

    _cyhal_uart_update_enabled_events(obj);
}

/* Gets the fifo level that will trigger at or before the requested level. This means that if the precise
 * value cannot be achieved, the returned value will trigger with fewer entries than requested in the FIFO
 * for RX, or with more entries that requested in the FIFO for TX */
static uint16_t _cyhal_uart_closest_fifo_match(cyhal_uart_t* obj, cyhal_uart_fifo_type_t type, uint16_t level)
{
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
        case _CYHAL_UART_GCI:
            /* Fast and GCI support arbitrary levels, so this function is a passthrough */
            return level;
        case _CYHAL_UART_DBG:
            /* Unlike the other two, the slow UART doesn't permit arbitrary levels */
            switch(type)
            {
                case CYHAL_UART_FIFO_RX:
                {
                    /* Add 1 because the hardware does >= but we want > */
                    uint16_t supported_levels[] = { 2, (_CYHAL_UART_FIFO_DEPTH / 4) + 1, (_CYHAL_UART_FIFO_DEPTH / 2) + 1,
                                          (_CYHAL_UART_FIFO_DEPTH - 2) + 1 };
                    /* For RX, err on the side of triggering with fewer items than requested. This means we want to
                     * loop from highest to lowest and pick the first one that is smaller than what we need */
                    for(int i = sizeof(supported_levels) / sizeof(supported_levels[0]) - 1; i >= 0; --i)
                    {
                        if(level >= supported_levels[i])
                        {
                            return supported_levels[i];
                        }
                    }
                    CY_ASSERT(false); /* Should never happen */
                    return 1u;
                }
                case CYHAL_UART_FIFO_TX:
                {
                    uint16_t supported_levels[] = { 0, 2, _CYHAL_UART_FIFO_DEPTH / 4, _CYHAL_UART_FIFO_DEPTH / 2};
                    for(size_t i = 0; i < (sizeof(supported_levels) / sizeof(supported_levels[0])); ++i)
                    {
                        /* For TX, err on the side of triggering with more items than requested */
                        if(level <= supported_levels[i])
                        {
                            return supported_levels[i];
                        }
                    }
                    CY_ASSERT(false); /* Should never happen */
                    return 1u;
                }
                default:
                    CY_ASSERT(false);
                    return 1u;
            }
        default:
            CY_ASSERT(false);
            return 1u;
    }
}

static cy_rslt_t _cyhal_uart_update_fifo_level(cyhal_uart_t* obj, cyhal_uart_fifo_type_t type)
{
    uint16_t level;
    /* Use the user-requested level unless both of the following are true:
     * * We have an async transfer pending
     * * The user-requested level will fire later than ideal for the async transfer.
     * Note that the FIFO triggers are defined as strictly greater/less than, so we need to adjust
     * the level we ask for up (for tx) or down (for rx) by one in order to get notified when the
     * fifo equals the level we want.
     */
    switch(type)
    {
        case CYHAL_UART_FIFO_RX:
        {
            /* For RX, err on the side of firing with too few in the FIFO */
            uint16_t async_rx_trigger_level = obj->async_rx_length - 1U;
            level = ((NULL != obj->async_rx_buff) && (obj->user_rx_trigger_level > async_rx_trigger_level))
                    ? _cyhal_uart_closest_fifo_match(obj, type, async_rx_trigger_level)
                    : obj->user_rx_trigger_level;
            break;
        }
        case CYHAL_UART_FIFO_TX:
        {
            if(_CYHAL_UART_DBG == obj->resource.block_num)
            {
                /* The IP doesn't support programmable tx fifo levels, it always fires at "tx empty".
                 * We don't generate an error if we get here because a silent no-op makes the code flow
                 * cleaner in the rest of the driver. We perform error checks at the point where the user
                 * could try to set the tx fifo level to something else */
                 return CY_RSLT_SUCCESS;
            }
            /* For TX, err on the side of firing with too many in the FIFO */
            uint16_t async_tx_trigger_level = obj->async_tx_length + 1U;
            level = (NULL != obj->async_tx_buff && obj->user_tx_trigger_level < async_tx_trigger_level)
                    ? _cyhal_uart_closest_fifo_match(obj, type, async_tx_trigger_level)
                    : obj->user_tx_trigger_level;
            break;
        }
        default:
            CY_ASSERT(false);
            return CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            {
                uint32_t seci_fifo_level = CHIPCOMMON_SECI_FIFO_LEVEL_REG;
                if(CYHAL_UART_FIFO_RX == type)
                {
                    /* Although the register map indicates otherwise, in practice the
                     * RX fifo level triggers on >= , not strictly > */
                    seci_fifo_level &= ~CC_SECI_RX_FIFO_LVL_MASK;
                    seci_fifo_level |= ((level + 1) << CC_SECI_RX_FIFO_LVL_SHIFT);
                }
                else
                {
                    seci_fifo_level &= ~CC_SECI_TX_FIFO_LVL_MASK;
                    seci_fifo_level |= (level << CC_SECI_TX_FIFO_LVL_SHIFT);
                }
                CHIPCOMMON_SECI_FIFO_LEVEL_REG = seci_fifo_level;
            }
            break;
        case _CYHAL_UART_DBG:
            {
                /* Unlike the other two, the slow UART doesn't permit arbitrary levels */
                uint32_t slow_fifo_level = _CYHAL_UART_SLOW_BASE->iir_fcr;
                if(CYHAL_UART_FIFO_RX == type)
                {
                    slow_fifo_level &= ~(UART_SLOW_FCR_RCVR_MASK);
                    /* We add 1 because the hardware matches on equals but our event is
                     * defined as strictly greater than. */
                    switch(level)
                    {
                        case 2:
                            slow_fifo_level |= UART_SLOW_FCR_RCVR_1_VAL;
                            break;
                        case (_CYHAL_UART_FIFO_DEPTH / 4) + 1:
                            slow_fifo_level |= UART_SLOW_FCR_RCVR_1_4_VAL;
                            break;
                        case (_CYHAL_UART_FIFO_DEPTH / 2) + 1:
                            slow_fifo_level |= UART_SLOW_FCR_RCVR_1_2_VAL;
                            break;
                        case (_CYHAL_UART_FIFO_DEPTH - 2) + 1:
                            slow_fifo_level |= UART_SLOW_FCR_RCVR_2_LEFT_VAL;
                            break;
                        default:
                            return CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
                    }
                    _CYHAL_UART_SLOW_BASE->iir_fcr = slow_fifo_level;
                }
            }
            break;
        case _CYHAL_UART_GCI:
            {
                if(CYHAL_UART_FIFO_RX == type)
                {
                    /* Although the register map indicates otherwise, in practice the
                     * RX fifo level triggers on >= , not strictly > */
                    uint32_t gci_rx_fifo_level = GCI_GCI_RX_FIFO_PER_IP_CTRL_REG;
                    gci_rx_fifo_level &= ~(GCI_SECI_RX_FIFO_LVL_MASK);
                    gci_rx_fifo_level |= ((level + 1) << GCI_SECI_RX_FIFO_LVL_SHIFT);
                    GCI_GCI_RX_FIFO_PER_IP_CTRL_REG = gci_rx_fifo_level;
                }
                else
                {
                    uint32_t gci_tx_fifo_level = GCI_SECI_FIFO_LEVEL_REG;
                    gci_tx_fifo_level &= ~(GCI_SECI_TX_FIFO_LVL_MASK);
                    gci_tx_fifo_level |= (level << GCI_SECI_TX_FIFO_LVL_SHIFT);
                    GCI_SECI_FIFO_LEVEL_REG = gci_tx_fifo_level;
                }
            }
            break;
        default:
            CY_ASSERT(false);
            return CYHAL_UART_RSLT_ERR_UNSUPPORTED_OPERATION;
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_uart_set_fifo_level(cyhal_uart_t *obj, cyhal_uart_fifo_type_t type, uint16_t level)
{
    CY_ASSERT(NULL != obj);
    /* Slow UART IP isn't configured to support programmable TX trigger levels */
    if(level > _CYHAL_UART_FIFO_DEPTH || 0u == level
        || (CYHAL_UART_FIFO_TX == type && _CYHAL_UART_DBG == obj->resource.block_num))
    {
        return CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    uint16_t old_level;
    switch(type)
    {
        case CYHAL_UART_FIFO_RX:
            old_level = obj->user_rx_trigger_level;
            obj->user_rx_trigger_level = level;
            break;
        case CYHAL_UART_FIFO_TX:
            old_level = obj->user_tx_trigger_level;
            obj->user_tx_trigger_level = level;
            break;
        default:
            CY_ASSERT(false);
            return CYHAL_UART_RSLT_ERR_UNSUPPORTED_OPERATION;
    }

    cy_rslt_t result = _cyhal_uart_update_fifo_level(obj, type);
    if(CY_RSLT_SUCCESS != result)
    {
        /* Revert to the previous value so that we don't stay in an error state */
        switch(type)
        {
            case CYHAL_UART_FIFO_RX:
                obj->user_rx_trigger_level = old_level;
                break;
            case CYHAL_UART_FIFO_TX:
                obj->user_tx_trigger_level = old_level;
                break;
            default:
                CY_ASSERT(false);
                break;
        }
    }
    return result;
}

cy_rslt_t cyhal_uart_enable_output(cyhal_uart_t *obj, cyhal_uart_output_t output, cyhal_source_t *source)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);
    CY_UNUSED_PARAMETER(source);
    return CYHAL_UART_RSLT_ERR_UNSUPPORTED_OPERATION;
}

cy_rslt_t cyhal_uart_disable_output(cyhal_uart_t *obj, cyhal_uart_output_t output)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);
    return CYHAL_UART_RSLT_ERR_UNSUPPORTED_OPERATION;
}

cy_rslt_t cyhal_uart_config_software_buffer(cyhal_uart_t *obj, uint8_t *rx_buffer, uint32_t rx_buffer_size)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(NULL != rx_buffer);

    return (cy_rslt_t)ring_buffer_init(&obj->buffer, rx_buffer, rx_buffer_size);
}

/*******************************************************************************
*       Internal - SECI and Baud
*******************************************************************************/

static cy_rslt_t _cyhal_uart_seci_check_baud(uint32_t input_freq_hz, uint32_t desired_baud)
{
    int baud_rate_div_factor = 0;

    if ( desired_baud < _CYHAL_UART_SECI_BAUD_RATE_THRESHOLD_LOW )
    {
        return CY_RSLT_WRN_CSP_UART_BAUD_TOLERANCE;
    }

    /* Check if the desired baud-rate and clock are configurable */
    baud_rate_div_factor = UART_SECI_BAUD_RATE_DIVISOR_RANGE - (int)(input_freq_hz/(UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE * desired_baud));

    if ( baud_rate_div_factor < 0 )
    {
        return CY_RSLT_WRN_CSP_UART_BAUD_TOLERANCE;
    }

    return CY_RSLT_SUCCESS;
}

/*
 * This algorithm converges on the optimal target baud-rate that will
 * achieve the least rate error with respect to the desired baud-rate.
 */
static uint32_t _cyhal_uart_seci_target_baud_rate(uint32_t clock_hz, uint32_t desired_baud_rate)
{
    int desired_baud_signed = (int32_t)desired_baud_rate; /* Allow signed subtractions later with less casting */
    /* Fast lookup of the desired baud-rate in the baud-rate mapping table */
    for (size_t i = 0 ; i < sizeof(_cyhal_uart_seci_baud_rate_mapping) / sizeof(_cyhal_uart_seci_baud_rate_mapping[0]); i++)
    {
        if ((_cyhal_uart_seci_baud_rate_mapping[i].clock_hz == clock_hz) && (_cyhal_uart_seci_baud_rate_mapping[i].desired_baud_rate == desired_baud_rate))
        {
            return _cyhal_uart_seci_baud_rate_mapping[i].target_baud_rate;
        }
    }

    int optimal_baud_rate_hrm = 0;
    int optimal_baud_rate_lrm = 0;
    for (int baud_rate_divisor = 0 ; baud_rate_divisor <= _CYHAL_UART_SECI_BAUD_RATE_DIVISOR_MAX ; baud_rate_divisor++)
    {
        if ((baud_rate_divisor >= _CYHAL_UART_SECI_HIGH_RATE_THRESHOLD_LOW) && (baud_rate_divisor <= _CYHAL_UART_SECI_HIGH_RATE_THRESHOLD_HIGH))
        {
            /* Compute optimal target baud-rate in high rate mode for this divisor value */
            int target_baud_rate_hrm = (int)clock_hz / (_CYHAL_UART_SECI_BAUD_RATE_DIVISOR_RANGE - baud_rate_divisor);

            /* Update the optimal target baud-rate achievable in high rate mode */
            if (optimal_baud_rate_hrm == 0)
            {
                optimal_baud_rate_hrm = target_baud_rate_hrm;
            }
            else if (abs(desired_baud_signed  - target_baud_rate_hrm) < abs(desired_baud_signed - optimal_baud_rate_hrm))
            {
                optimal_baud_rate_hrm = target_baud_rate_hrm;
            }
        }

        /* Compute optimal target baud-rate in low rate mode for this divisor value */
        for (int baud_rate_adjustment = _CYHAL_UART_SECI_BAUD_RATE_ADJUSTMENT_MAX ; baud_rate_adjustment >= 0 ; baud_rate_adjustment--)
        {
            int target_baud_rate_lrm = (int)clock_hz / (((_CYHAL_UART_SECI_BAUD_RATE_DIVISOR_RANGE - baud_rate_divisor) * _CYHAL_UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE) + baud_rate_adjustment);

            /* Update the optimal target baud-rate achievable in low rate mode */
            if (optimal_baud_rate_lrm == 0)
            {
                optimal_baud_rate_lrm = target_baud_rate_lrm;
            }
            else if (abs(desired_baud_signed - target_baud_rate_lrm) < abs(desired_baud_signed - optimal_baud_rate_lrm))
            {
                optimal_baud_rate_lrm = target_baud_rate_lrm;
            }
        }
    }

    int optimal_baud_rate = 0;
    if (abs(desired_baud_signed - optimal_baud_rate_lrm) < abs(desired_baud_signed - optimal_baud_rate_hrm))
    {
        optimal_baud_rate = optimal_baud_rate_lrm;
    }
    else
    {
        optimal_baud_rate = optimal_baud_rate_hrm;
    }

    return (uint32_t)optimal_baud_rate;
}

static void _cyhal_uart_seci_set_flow_control(cyhal_uart_t* obj, bool cts, bool rts)
{
    _cyhal_uart_seci_t* uart_seci_base = _cyhal_uart_get_seci_base(obj);
    /* Configure Flow Control */
    if (rts)
    {
        /* RTS+CTS Flow Control */
        uart_seci_base->mcr |= UART_SECI_MCR_AUTO_RTS;
    }
    else
    {
        uart_seci_base->mcr &= ~(UART_SECI_MCR_AUTO_RTS);
    }
    obj->enable_rts = rts;

    if(cts)
    {
         uart_seci_base->mcr |= UART_SECI_MCR_AUTO_TX_DIS;
    }
    else
    {
        uart_seci_base->mcr &= ~(UART_SECI_MCR_AUTO_TX_DIS);
    }
    obj->enable_cts = cts;
}

static cy_rslt_t _cyhal_uart_debug_set_baud(cyhal_uart_t* obj, uint32_t desired_baud, uint32_t* actual_baud)
{
    /* Calculate the necessary divider value for a given baud rate */
    /* divisor = (serial clock frequency/16) / (baud rate)
    *  The baud_rate / 2 is added to reduce error to + / - half of baud rate.
    */
    uint32_t source_freq_hz = cyhal_clock_get_frequency(&obj->clk);
    uint16_t baud_divider = ( (source_freq_hz / 16 ) + (desired_baud/ 2) ) / desired_baud;
    baud_divider  = ( (source_freq_hz / 16 ) + (desired_baud/ 2) ) / desired_baud;


    /* Set DLAB bit to enable access to the divider */
    _CYHAL_UART_SLOW_BASE->lcr |= UART_SLOW_LCR_DLAB;

    /* Setup the baudrate */
    _CYHAL_UART_SLOW_BASE->lcr      |= UART_SLOW_LCR_DLAB;
    _CYHAL_UART_SLOW_BASE->rx_tx_dll = baud_divider & 0xff;
    _CYHAL_UART_SLOW_BASE->ier_dlm   = baud_divider >> 8;
    _CYHAL_UART_SLOW_BASE->lcr      |= UART_SLOW_LCR_WLEN8;

    /* Clear DLAB bit. This bit must be cleared after initial baud rate setup in order to access other registers. */
    _CYHAL_UART_SLOW_BASE->lcr &= ~UART_SLOW_LCR_DLAB;

    if(NULL != actual_baud)
    {
        /* Solve the baud_divider equation for desired_baud */
        *actual_baud = source_freq_hz / ((16 * baud_divider) - 8);
    }
    return CY_RSLT_SUCCESS;
}

static cy_rslt_t _cyhal_uart_seci_set_baud(cyhal_uart_t* obj, uint32_t desired_baud, uint32_t *actual_baud)
{
    _cyhal_uart_seci_t* uart_seci_base = _cyhal_uart_get_seci_base(obj);
    uint32_t source_freq_hz = cyhal_clock_get_frequency(&obj->clk);
    cy_rslt_t result = _cyhal_uart_seci_check_baud(source_freq_hz, CYHAL_UART_DEFAULT_BAUD);
    if(CY_RSLT_SUCCESS == result)
    {
        /* Setup SECI UART baud rate */
        uint32_t uart_seci_baud_rate = _cyhal_uart_seci_target_baud_rate(source_freq_hz, desired_baud);
        uint32_t baud_rate_div_high_rate;
        bool uart_high_rate_mode;

        uint32_t clock_divider = source_freq_hz/uart_seci_baud_rate;
        if ( clock_divider > _CYHAL_UART_SECI_BAUD_RATE_DIVISOR_RANGE )
        {
            uart_high_rate_mode = false;
        }
        else
        {
            baud_rate_div_high_rate = _CYHAL_UART_SECI_BAUD_RATE_DIVISOR_RANGE - clock_divider;
            uart_high_rate_mode = (( baud_rate_div_high_rate < _CYHAL_UART_SECI_HIGH_RATE_THRESHOLD_LOW )
                                || ( baud_rate_div_high_rate > _CYHAL_UART_SECI_HIGH_RATE_THRESHOLD_HIGH ));
        }

        uint32_t baud_rate_adjustment;
        if (uart_high_rate_mode)
        {
            /* Setup in high rate mode, disable baudrate adjustment */
            baud_rate_div_high_rate  = _CYHAL_UART_SECI_BAUD_RATE_DIVISOR_RANGE - (source_freq_hz/uart_seci_baud_rate);

            baud_rate_adjustment = 0x0;
            uart_seci_base->bauddiv = baud_rate_div_high_rate;
            uart_seci_base->baudadj = baud_rate_adjustment;
            uart_seci_base->mcr |= UART_SECI_MCR_HIGHRATE_EN;
            uart_seci_base->mcr &= ~(UART_SECI_MCR_BAUD_ADJ_EN);
        }
        else
        {
            /* Setup in low rate mode, enable baudrate adjustment */
            uint32_t baud_rate_div_low_rate  = UART_SECI_BAUD_RATE_DIVISOR_RANGE - (source_freq_hz/(UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE * uart_seci_baud_rate));
            uint32_t baud_rate_adjust_low  = ((source_freq_hz/uart_seci_baud_rate) % UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE) / 2;
            uint32_t baud_rate_adjust_high = ((source_freq_hz/uart_seci_baud_rate) % UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE) - baud_rate_adjust_low;
            baud_rate_adjustment  = (baud_rate_adjust_high << 4) | baud_rate_adjust_low;

            uart_seci_base->bauddiv = baud_rate_div_low_rate;
            uart_seci_base->baudadj = baud_rate_adjustment;
            uart_seci_base->mcr &= ~(UART_SECI_MCR_HIGHRATE_EN);
            uart_seci_base->mcr |= UART_SECI_MCR_BAUD_ADJ_EN;
        }
        if(NULL != actual_baud)
        {
            *actual_baud = uart_seci_baud_rate;
        }
    }
    return result;
}

static _cyhal_uart_seci_t* _cyhal_uart_get_seci_base(cyhal_uart_t* obj)
{
    CY_ASSERT(obj->resource.block_num == _CYHAL_UART_FAST || obj->resource.block_num == _CYHAL_UART_GCI);
    return (obj->resource.block_num == _CYHAL_UART_FAST) ? _CYHAL_UART_FAST_BASE : _CYHAL_UART_GCI_BASE;
}

static cy_rslt_t _cyhal_uart_seci_setup_internal(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg)
{
    cy_rslt_t result = _cyhal_uart_seci_set_baud(obj, CYHAL_UART_DEFAULT_BAUD, NULL);
    if(CY_RSLT_SUCCESS == result)
    {
        CY_ASSERT(8u == cfg->data_bits); /* Only supported data width */

        result = _cyhal_uart_seci_config(obj, cfg);
    }

    if(CY_RSLT_SUCCESS == result)
    {
        _cyhal_uart_seci_t* uart_seci_base = _cyhal_uart_get_seci_base(obj);

        /* Flow control always defaults to disabled */
        _cyhal_uart_seci_set_flow_control(obj, false, false);

        /* Setup LCR and MCR registers for TX, RX and HW flow control */
        /* Do not enable RX, interrupts are still disabled */
        uart_seci_base->lcr |= UART_SECI_LCR_TXO_EN;
        uart_seci_base->mcr |= UART_SECI_MCR_TX_EN;
    }
    return result;
}


/*******************************************************************************
*       Internal - Init and Config
*******************************************************************************/

static cy_rslt_t _cyhal_uart_init_fast(cyhal_uart_t *obj, const cyhal_uart_cfg_t *cfg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t clk_timeout = 100000000;

    /* The WICED platform_uart code on which this driver is based only supports 8-bit data width.
     * We may be able to relax this in the future after further investigation into why the
     * restriction existed in WICED */
    if (cfg->data_bits != 8)
    {
        result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    if(CY_RSLT_SUCCESS == result)
    {
        /* Make sure ChipCommon core is enabled */
        osl_core_enable(CC_CORE_ID);

        /* The registers here all have "SECI" in their name but they are for the fast UART. The SECI UART
         * registers all have a "GCI" prefix */

        /* Configure SECI Fast UART clock */
        if (obj->clk.block == CYHAL_CLOCK_BLOCK_FAST_UART)
        {
            CHIPCOMMON_SECI_CONFIG_REG |= (CC_SECI_CONFIG_HT_CLOCK);
        }
        else if (obj->clk.block == CYHAL_CLOCK_BLOCK_ALP)
        {
            CHIPCOMMON_SECI_CONFIG_REG &= ~(CC_SECI_CONFIG_HT_CLOCK);
        }
        else
        {
            /* ALP and HT are the only supported clocks */
            result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
        }
    }

    if(CY_RSLT_SUCCESS == result)
    {
        uint32_t flags = cyhal_system_critical_section_enter();

        /* Enable SECI clock */
        CHIPCOMMON_CLK_CTL_STATUS_REG |= CC_CLK_CTL_ST_SECI_CLK_REQ;

        cyhal_system_critical_section_exit(flags);

        /* Wait for the transition to complete */
        while ( ( ( CHIPCOMMON_CLK_CTL_STATUS_REG & CC_CLK_CTL_ST_SECI_CLK_AVAIL ) == 0 ) && ( clk_timeout > 0 ) )
        {
            /* spin wait for clock transition */
            clk_timeout--;
        }

        /* Put SECI block into reset */
        CHIPCOMMON_SECI_CONFIG_REG &= ~(CC_SECI_ENAB_SECI_ECI);
        CHIPCOMMON_SECI_CONFIG_REG |= CC_SECI_RESET;

        /* set force-low, and set EN_SECI for all non-legacy modes */
        CHIPCOMMON_SECI_CONFIG_REG |= CC_SECI_ENAB_SECIOUT_DIS;
        CHIPCOMMON_SECI_CONFIG_REG |= CC_SECI_ENAB_SECI_ECI;

        /* Take SECI block out of reset */
        CHIPCOMMON_SECI_CONFIG_REG &= ~(CC_SECI_RESET);

        /* Set linebreak to set SECI is not ready to communicate */
        CHIPCOMMON_SECI_UART_LCR_REG |= 0x10;

        /* Setup and configure SECI UART */
        result = _cyhal_uart_seci_setup_internal(obj, cfg);
    }

    if(CY_RSLT_SUCCESS == result)
    {
        /* Setup SECI block operation mode */
        CHIPCOMMON_SECI_CONFIG_REG &= ~(CC_SECI_MODE_MASK << CC_SECI_MODE_SHIFT);
        CHIPCOMMON_SECI_CONFIG_REG |= (SECI_MODE_UART << CC_SECI_MODE_SHIFT);

        /* Setup default FIFO levels for flow control. RX and TX are handled outside of this function */
        uint32_t cc_seci_fifo_level = CHIPCOMMON_SECI_FIFO_LEVEL_REG;
        cc_seci_fifo_level &= ~(CC_SECI_RX_FIFO_LVL_FLOW_CTL_MASK << CC_SECI_RX_FIFO_LVL_FLOW_CTL_SHIFT);
        cc_seci_fifo_level |= (CC_SECI_RX_FIFO_LVL_FLOW_CTL_DEFAULT << CC_SECI_RX_FIFO_LVL_FLOW_CTL_SHIFT);
        CHIPCOMMON_SECI_FIFO_LEVEL_REG = cc_seci_fifo_level;

        /* Clear force-low bit */
        CHIPCOMMON_SECI_CONFIG_REG &= ~CC_SECI_ENAB_SECIOUT_DIS;

        /* Wait 10us for enabled SECI block serial output to stabilize */
        cyhal_system_delay_us(10);

        /* Initialization complete, enable interrupts from fast UART */
        _cyhal_uart_update_enabled_events(obj);
        _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), 0x0u, UART_FAST_CC_INT_STATUS_MASK);

        /* Make sure ChipCommon Core external IRQ to APPS core is enabled */
        _cyhal_system_chipcommon_enable_irq();

        /* Disable linebreak to indicate SECI is ready to communicate */
        CHIPCOMMON_SECI_UART_LCR_REG &= ~(0x10);

        /* Enable receive-enable at the end, when everything is set-up */
        _CYHAL_UART_FAST_BASE->lcr |= UART_SECI_LCR_RX_EN;
        _CYHAL_UART_FAST_BASE->mcr |= UART_SECI_MCR_AUTO_RTS;
    }

    return result;
}

static cy_rslt_t _cyhal_uart_common_config(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    if (cfg->data_bits != 8u)
    {
        /* Currently supported data width is 8 bits */
        result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    /* Setup ring buffer */
    if (NULL != cfg->rx_buffer)
    {
        ring_buffer_init(&obj->buffer, cfg->rx_buffer, cfg->rx_buffer_size);
    }
    else if(NULL != obj->buffer.buffer)
    {
        ring_buffer_deinit(&obj->buffer);
    }
    return result;
}

static cy_rslt_t _cyhal_uart_seci_config(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg)
{
    cy_rslt_t result = _cyhal_uart_common_config(obj, cfg);
    if(CY_RSLT_SUCCESS == result)
    {
        _cyhal_uart_seci_t* uart_seci_base = _cyhal_uart_get_seci_base(obj);

        /* Configure parity */
        if (CYHAL_UART_PARITY_ODD == cfg->parity)
        {
            uart_seci_base->lcr |= UART_SECI_LCR_PARITY_EN;
            uart_seci_base->lcr &= ~(UART_SECI_LCR_PARITY);
        }
        else if (CYHAL_UART_PARITY_EVEN == cfg->parity)
        {
            uart_seci_base->lcr |= UART_SECI_LCR_PARITY_EN;
            uart_seci_base->lcr |= UART_SECI_LCR_PARITY;
        }
        else
        {
            CY_ASSERT(CYHAL_UART_PARITY_NONE == cfg->parity);
            uart_seci_base->lcr &= ~(UART_SECI_LCR_PARITY_EN);
        }

        /* Configure stop bits */
        if (1u == cfg->stop_bits)
        {
            uart_seci_base->lcr &= ~(UART_SECI_LCR_STOP_BITS);
        }
        else if (2u == cfg->stop_bits)
        {
            uart_seci_base->lcr |= UART_SECI_LCR_STOP_BITS;
        }
        else
        {
            /* Unsupported */
            result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
        }
    }
    return result;
}

static cy_rslt_t _cyhal_uart_debug_config(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg)
{
    cy_rslt_t result = _cyhal_uart_common_config(obj, cfg);
    /* Configure stop bits and a parity */
    if(CY_RSLT_SUCCESS == result)
    {
        if (CYHAL_UART_PARITY_ODD == cfg->parity)
        {
            _CYHAL_UART_SLOW_BASE->lcr |= ( 1 << UART_PEN_BIT );
            _CYHAL_UART_SLOW_BASE->lcr &= ~( 1 << UART_EPS_BIT );
        }
        else if (CYHAL_UART_PARITY_EVEN == cfg->parity)
        {
            _CYHAL_UART_SLOW_BASE->lcr |= ( 1 << UART_PEN_BIT );
            _CYHAL_UART_SLOW_BASE->lcr |= ( 1 << UART_EPS_BIT );
        }
        else
        {
            CY_ASSERT(CYHAL_UART_PARITY_NONE == cfg->parity);
            _CYHAL_UART_SLOW_BASE->lcr &= ~( 1 << UART_PEN_BIT );
            _CYHAL_UART_SLOW_BASE->lcr &= ~( 1 << UART_EPS_BIT );
        }

        switch(cfg->stop_bits)
        {
            case 1:
                _CYHAL_UART_SLOW_BASE->lcr &= ~( 1 << UART_STOP_BIT );
                break;
            case 2:
                _CYHAL_UART_SLOW_BASE->lcr |= ( 1 << UART_STOP_BIT );
                break;
            default:
                result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
                break;
        }
    }
    return result;
}

static cy_rslt_t _cyhal_uart_init_dbg(cyhal_uart_t *obj, const cyhal_uart_cfg_t* cfg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Check configuration */
    if (obj->clk.block != CYHAL_CLOCK_BLOCK_ALP)
    {
        /* Slow UART is driven only by the ALP clock */
        result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Make sure ChipCommon core is enabled */
        osl_core_enable(CC_CORE_ID);

        /* Enable Slow UART clocking */
        /* Turn on ALP clock for UART, when we are not using a divider
         * Set the override bit so we don't divide it
         */
        CHIPCOMMON_CORE_CTRL_REG &= ~( 1 << 3 );
        CHIPCOMMON_CORE_CTRL_REG |= 0x01;
        CHIPCOMMON_CORE_CTRL_REG |= ( 1 << 3 );

        result = _cyhal_uart_debug_set_baud(obj, CYHAL_UART_DEFAULT_BAUD, NULL);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Enable FIFO */
        _CYHAL_UART_SLOW_BASE->iir_fcr |= UART_SLOW_FCR_FIFO_ENABLE;
        result = _cyhal_uart_debug_config(obj, cfg);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Enable slow UART interrupt in ChipCommon interrupt mask */
        _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), 0x0u, UART_SLOW_CC_INT_STATUS_MASK);

        /* Enable slow UART interrupts for RX data */
        _CYHAL_UART_SLOW_BASE->mcr |= UART_SLOW_MCR_OUT2;
        _cyhal_uart_update_enabled_events(obj);

        /* Make sure ChipCommon Core external IRQ to APPS core is enabled */
        _cyhal_system_chipcommon_enable_irq();

        /* Clean pending data. */
        for (int i = 0; i < UART_SLOW_CLEAN_ATTEMPTS; ++i )
        {
            if ( ( _CYHAL_UART_SLOW_BASE->lsr & UART_SLOW_LSR_RXRDY ) == 0 )
            {
                break;
            }
            (void)_CYHAL_UART_SLOW_BASE->rx_tx_dll; /* read and discard */
        }
    }

    return result;
}

static cy_rslt_t _cyhal_uart_init_gci(cyhal_uart_t *obj, const cyhal_uart_cfg_t *cfg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Check configuration */
    if (obj->clk.block != CYHAL_CLOCK_BLOCK_ALP)
    {
        /* Slow UART is driven only by the ALP clock */
        result = CYHAL_UART_RSLT_ERR_UNSUPPORTED_CONFIG;
    }

    if(CY_RSLT_SUCCESS == result)
    {
        /* Set ForceSeciClk and ForceRegClk */
        GCI_CORE_CTRL_REG |= (GCI_CORE_CTRL_FORCE_SECI_CLK | GCI_CORE_CTRL_FORCE_REG_CLK);

        /* Wait for SECI Clock Available */
        uint32_t clk_timeout = 100000000;
        while ( ( ( GCI_CORE_STATUS_REG & GCI_CORE_STATUS_GCI_CLK_AVAIL ) == 0 ) && ( clk_timeout > 0 ) )
        {
            /* spin wait for clock transition */
            clk_timeout--;
        }

        /* Put SECI block and OffChipCoex into reset */
        GCI_CORE_CTRL_REG |= (GCI_CORE_CTRL_SECI_RESET | GCI_CORE_CTRL_RESET_SECI_LOGIC | GCI_CORE_CTRL_RESET_OFF_CHIP_COEX);

        /* Set SECI output force-low */
        GCI_CORE_CTRL_REG |= (GCI_CORE_CTRL_FORCE_SECI_OUT_LOW);

        /* Take SECI block and OffChipCoex out of reset */
        GCI_CORE_CTRL_REG &= ~(GCI_CORE_CTRL_SECI_RESET | GCI_CORE_CTRL_RESET_SECI_LOGIC | GCI_CORE_CTRL_RESET_OFF_CHIP_COEX);

        /* Clear ForceSeciClk and ForceRegClk */
        GCI_CORE_CTRL_REG &= ~(GCI_CORE_CTRL_FORCE_SECI_CLK | GCI_CORE_CTRL_FORCE_REG_CLK);

        uint32_t flags = cyhal_system_critical_section_enter();

        /* Configure SECI_IN Control */
        GCI_INDIRECT_ADDRESS_REG = 0x0;
        uint32_t reg_val = GCI_SECI_IN_CTRL_REG;
        /* Setup SECI_IN operation mode */
        reg_val &= ~(GCI_SECI_IN_OP_MODE_MASK << GCI_SECI_IN_OP_MODE_SHIFT);
        reg_val |= (SECI_MODE_UART << GCI_SECI_IN_OP_MODE_SHIFT);
        /* Setup GCI_GPIO to SECI_IN (UART_RX) mapping */
        reg_val &= ~(GCI_SECI_IN_GPIO_NUM_MASK << GCI_SECI_IN_GPIO_NUM_SHIFT);
        reg_val |= (GCI_SECI_IN_GPIO_NUM_DEFAULT << GCI_SECI_IN_GPIO_NUM_SHIFT);
        GCI_SECI_IN_CTRL_REG = reg_val;

        /* Configure SECI_OUT Control */
        GCI_INDIRECT_ADDRESS_REG = 0x0;
        reg_val = GCI_SECI_OUT_CTRL_REG;
        /* Setup SECI_OUT operation mode */
        reg_val &= ~(GCI_SECI_OUT_OP_MODE_MASK << GCI_SECI_OUT_OP_MODE_SHIFT);
        reg_val |= (SECI_MODE_UART << GCI_SECI_OUT_OP_MODE_SHIFT);
        /* Setup SECI_OUT (UART_TX) to GCI_GPIO mapping */
        reg_val &= ~(GCI_SECI_OUT_GPIO_NUM_MASK << GCI_SECI_OUT_GPIO_NUM_SHIFT);
        reg_val |= (GCI_SECI_OUT_GPIO_NUM_DEFAULT << GCI_SECI_OUT_GPIO_NUM_SHIFT);
        GCI_SECI_OUT_CTRL_REG = reg_val;

        /* Initialize the SECI_IN GCI_GPIO */
        GCI_INDIRECT_ADDRESS_REG = GCI_GPIO_CTRL_REG_INDEX(GCI_SECI_IN_GPIO_NUM_DEFAULT);
        reg_val = GCI_GPIO_CTRL_REG;
        reg_val &= ~(GCI_GPIO_CTRL_GPIO_MASK(GCI_SECI_IN_GPIO_NUM_DEFAULT));
        reg_val |= (GCI_GPIO_CTRL_INPUT_ENAB(GCI_SECI_IN_GPIO_NUM_DEFAULT));
        reg_val |= (GCI_GPIO_CTRL_PDN(GCI_SECI_IN_GPIO_NUM_DEFAULT));
        GCI_GPIO_CTRL_REG = reg_val;

        /* Initialize the SECI_OUT GCI_GPIO */
        GCI_INDIRECT_ADDRESS_REG = GCI_GPIO_CTRL_REG_INDEX(GCI_SECI_OUT_GPIO_NUM_DEFAULT);
        reg_val = GCI_GPIO_CTRL_REG;
        reg_val &= ~(GCI_GPIO_CTRL_GPIO_MASK(GCI_SECI_OUT_GPIO_NUM_DEFAULT));
        reg_val |= (GCI_GPIO_CTRL_OUTPUT_ENAB(GCI_SECI_OUT_GPIO_NUM_DEFAULT));
        reg_val |= (GCI_GPIO_CTRL_PDN(GCI_SECI_OUT_GPIO_NUM_DEFAULT));
        GCI_GPIO_CTRL_REG = reg_val;

        cyhal_system_critical_section_exit(flags);

        /* Enable SECI data communication */
        GCI_CORE_CTRL_REG |= (GCI_CORE_CTRL_ENABLE_SECI);

        /* Setup and configure SECI UART */
        result = _cyhal_uart_seci_setup_internal(obj, cfg);
    }

    if(CY_RSLT_SUCCESS == result)
    {
        /* Enable UART bytes reception from SECI_IN port to RX FIFO */
        GCI_SECI_IN_AUX_FIFO_RX_ENAB_REG |= (GCI_SECI_FIFO_RX_ENAB);

        /* Enable the SECI UART TX state machine */
        GCI_SECI_OUT_TX_ENAB_TX_BRK_REG |= (GCI_SECI_TX_ENAB);

        /* Clear SECI output force-low */
        GCI_CORE_CTRL_REG &= ~(GCI_CORE_CTRL_FORCE_SECI_OUT_LOW);

        /* Wait 10us for enabled SECI block serial output to stabilize */
        cyhal_system_delay_us(10);

        /* Initialization complete, enable interrupts from GCI UART */
        _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), 0x0u, UART_GCI_CC_INT_STATUS_MASK);

        /* Make sure ChipCommon Core external IRQ to APPS core is enabled */
        _cyhal_system_chipcommon_enable_irq();
    }

    return result;
}


/*******************************************************************************
*       Internal - Events
*******************************************************************************/

static cyhal_uart_event_t _cyhal_uart_event_from_status(cyhal_uart_t *obj, uint32_t status)
{
    /* Note: this is not a direct inverse of _cyhal_uart_mask_from_event because the interrupt status
     * bits aren't the same as the mask bits for UART_DBG. */
     cyhal_uart_event_t event = CYHAL_UART_IRQ_NONE;
     switch(obj->resource.block_num)
     {
         case _CYHAL_UART_FAST:
             if(0u != (status & CC_SECI_STATUS_TX_FIFO_ALMOST_EMPTY))
             {
                 event |= CYHAL_UART_IRQ_TX_FIFO;
             }
             if(0u != (status & CC_SECI_STATUS_RX_FIFO_ALMOST_FULL))
             {
                 event |= CYHAL_UART_IRQ_RX_FIFO;
             }
             break;
         case _CYHAL_UART_DBG:
             if(0u != (status & (UART_SLOW_LSR_THRE | UART_SLOW_LSR_TEMT)))
             {
                 event |= CYHAL_UART_IRQ_TX_EMPTY;
             }
             if(0u != (status & UART_SLOW_LSR_RXRDY))
             {
                 event |= CYHAL_UART_IRQ_RX_FIFO;
             }
             break;
         case _CYHAL_UART_GCI:
             if(0u != (status & CYHAL_UART_IRQ_RX_NOT_EMPTY))
             {
                 event |= GCI_INT_ST_MASK_SECI_RX_FIFO_NOT_EMPTY;
             }
             if(0u != (status & GCI_INT_ST_MASK_SECI_TX_FIFO_ALMOST_EMPTY))
             {
                 event |= CYHAL_UART_IRQ_TX_FIFO;
             }
             if(0u != (status & GCI_INT_ST_MASK_SECI_RX_FIFO_ALMOST_FULL))
             {
                 event |= CYHAL_UART_IRQ_RX_FIFO;
             }
             break;
     }
     return event;
}

static uint32_t _cyhal_uart_mask_from_event(cyhal_uart_t *obj, cyhal_uart_event_t event)
{
    uint32_t mask = 0u;
    /* CYHAL_UART_IRQ_TX_TRANSMIT_IN_FIFO generated in software during async transfer */
    /* CYHAL_UART_IRQ_TX_DONE, CYHAL_UART_IRQ_RX_FULL aren't generated on this hardware */
    /* CYHAL_UART_IRQ_RX_DONE generated in software during async transfer */
    if(0u != (event & CYHAL_UART_IRQ_RX_ERROR ))
    {
        switch(obj->resource.block_num)
        {
            case _CYHAL_UART_FAST:
            case _CYHAL_UART_DBG:
                /* This block can't generate this event */
                break;
            case _CYHAL_UART_GCI:
                mask |= GCI_INT_ST_MASK_SECI_RX_FIFO_OVERFLOW;
                break;
        }
    }
    if(0u != (event & CYHAL_UART_IRQ_RX_NOT_EMPTY))
    {
        switch(obj->resource.block_num)
        {
            case _CYHAL_UART_FAST:
            case _CYHAL_UART_DBG:
                /* This block can't generate this event */
                break;
            case _CYHAL_UART_GCI:
                mask |= GCI_INT_ST_MASK_SECI_RX_FIFO_NOT_EMPTY;
                break;
        }
    }

    if(0u != (event & CYHAL_UART_IRQ_TX_EMPTY))
    {
        /* Only the slow UART can generate this event */
        if(_CYHAL_UART_DBG == obj->resource.block_num)
        {
            mask |= UART_SLOW_IER_THRE;
        }
    }
    if(0u != (event & CYHAL_UART_IRQ_TX_FIFO))
    {
        /* The slow UART can't generate this event */
        switch(obj->resource.block_num)
        {
            case _CYHAL_UART_FAST:
                mask |= CC_SECI_STATUS_TX_FIFO_ALMOST_EMPTY;
                break;
            case _CYHAL_UART_GCI:
                mask |= GCI_INT_ST_MASK_SECI_TX_FIFO_ALMOST_EMPTY;
                break;
        }
    }
    if(0u != (event & CYHAL_UART_IRQ_RX_FIFO))
    {
        switch(obj->resource.block_num)
        {
            case _CYHAL_UART_FAST:
                mask |= CC_SECI_STATUS_RX_FIFO_ALMOST_FULL;
                break;
            case _CYHAL_UART_DBG:
                mask |= UART_SLOW_IER_RXRDY;
                break;
            case _CYHAL_UART_GCI:
                mask |= GCI_INT_ST_MASK_SECI_RX_FIFO_ALMOST_FULL;
                break;
        }
    }
    return mask;
}

static uint32_t _cyhal_uart_get_trigger_level(cyhal_uart_t *obj, bool rx_trigger)
{
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            /* Although the register map indicates otherwise, in practice the RX fifo level triggers on >= ,
             * not strictly >, so we set it to 1 greater than the user requested */
            return rx_trigger
                ? ((CHIPCOMMON_SECI_FIFO_LEVEL_REG & CC_SECI_RX_FIFO_LVL_MASK) >> CC_SECI_RX_FIFO_LVL_SHIFT) - 1
                : (CHIPCOMMON_SECI_FIFO_LEVEL_REG & CC_SECI_TX_FIFO_LVL_MASK) >> CC_SECI_TX_FIFO_LVL_SHIFT;
        case _CYHAL_UART_DBG:
        {
            /* Unlike the other two, the slow UART doesn't permit arbitrary levels */
            uint32_t slow_fifo_level = _CYHAL_UART_SLOW_BASE->iir_fcr;
            if(rx_trigger)
            {
                uint32_t rx_level = slow_fifo_level & UART_SLOW_FCR_RCVR_MASK;
                /* Add one to everything because the hardware does >= but we want > */
                switch(rx_level)
                {
                    case UART_SLOW_FCR_RCVR_1_VAL:
                        return (1u);
                    case UART_SLOW_FCR_RCVR_1_4_VAL:
                        return (_CYHAL_UART_FIFO_DEPTH / 4) + 1;
                    case UART_SLOW_FCR_RCVR_1_2_VAL:
                        return (_CYHAL_UART_FIFO_DEPTH / 2) + 1;
                    case UART_SLOW_FCR_RCVR_2_LEFT_VAL:
                        return (_CYHAL_UART_FIFO_DEPTH - 2) + 1;
                    default:
                        CY_ASSERT(false);
                        return 0u;
                }
            }
            else
            {
                /* The Slow UART IP isn't configured to enable programmable transmit trigger levels. The only
                 * available trigger is "tx empty". We don't expose this as a "TX FIFO" event, but we
                 * let this value be queried internally because it reduces the number of places where we have to
                 * special case the slow UART */
                return 0u;
            }
        }
        case _CYHAL_UART_GCI:
            /* Although the register map indicates otherwise, in practice the RX fifo level triggers on >= ,
             * not strictly >, so we set it to 1 greater than the user requested */
            return rx_trigger
                ? ((GCI_GCI_RX_FIFO_PER_IP_CTRL_REG & GCI_SECI_RX_FIFO_LVL_MASK) >> GCI_SECI_RX_FIFO_LVL_SHIFT) - 1
                : (GCI_SECI_FIFO_LEVEL_REG & GCI_SECI_TX_FIFO_LVL_MASK) >> GCI_SECI_TX_FIFO_LVL_SHIFT;
        default:
            CY_ASSERT(false);
            return 0u;
    }
}

static void _cyhal_uart_update_enabled_events(cyhal_uart_t *obj)
{
    cyhal_uart_event_t events = (cyhal_uart_event_t)obj->user_enabled_events;
    /* If the ring buffer is enabled, always listen internal so that we can copy
     * from the FIFO into the ring buffer whenever the FIFO gets too full. */
    if(NULL != obj->async_rx_buff || NULL != obj->buffer.buffer)
    {
         events |= CYHAL_UART_IRQ_RX_FIFO;
    }
    if(NULL != obj->async_tx_buff)
    {
        /* TX FIFO interrupt isn't supported for the Slow UART IP */
        events |= (_CYHAL_UART_DBG == obj->resource.block_num)
                    ? CYHAL_UART_IRQ_TX_EMPTY : CYHAL_UART_IRQ_TX_FIFO;
    }

    uint32_t mask = _cyhal_uart_mask_from_event(obj, events);
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            CHIPCOMMON_SECI_STATUS_MASK_REG = mask;
            break;
        case _CYHAL_UART_DBG:
            _CYHAL_UART_SLOW_BASE->ier_dlm = mask;
            break;
        case _CYHAL_UART_GCI:
            GCI_INT_MASK_REG = mask;
            break;
    }
}

static void _cyhal_uart_invoke_user_callback(cyhal_uart_t* obj, cyhal_uart_event_t event)
{
    cyhal_uart_event_t user_events = (cyhal_uart_event_t)(event & (cyhal_uart_event_t)obj->user_enabled_events);
    if(CYHAL_UART_IRQ_NONE != user_events)
    {
        cyhal_uart_event_callback_t callback = (cyhal_uart_event_callback_t) obj->callback_data.callback;
        if(NULL != callback)
        {
            callback(obj->callback_data.callback_arg, user_events);
        }
    }
}


/*******************************************************************************
*       Internal - Interrupt
*******************************************************************************/

void _cyhal_uart_irq_handler(cyhal_uart_t* obj, uint32_t int_status)
{
    CY_ASSERT(NULL != obj);
    cyhal_uart_event_t event = _cyhal_uart_event_from_status(obj, int_status);
    if(0u != (event & (CYHAL_UART_IRQ_RX_NOT_EMPTY | CYHAL_UART_IRQ_RX_FIFO)))
    {
        if(NULL != obj->async_rx_buff)
        {
            size_t rx_length = obj->async_rx_length;
            /* Okay to cast away volatile here because no other interrupt will be changing
             * this out from under us */
            cy_rslt_t result = cyhal_uart_read(obj, (void *)obj->async_rx_buff, &rx_length);
            if(CY_RSLT_SUCCESS == result)
            {
                obj->async_rx_buff = (void *)(((size_t)obj->async_rx_buff) + rx_length);
                obj->async_rx_length -= rx_length;
                if(obj->async_rx_length == 0)
                {
                    obj->async_rx_buff = NULL;
                    event |= CYHAL_UART_IRQ_RX_DONE;
                }
                result = _cyhal_uart_update_fifo_level(obj, CYHAL_UART_FIFO_RX);
                _cyhal_uart_update_enabled_events(obj);
            }
            CY_ASSERT(CY_RSLT_SUCCESS == result); /* Read should never fail here in normal operation */
        }
        else if(NULL != obj->buffer.buffer) /* Copy from FIFO into ring buffer if present */
        {
            while(_cyhal_uart_fifo_readable(obj) && ring_buffer_free_space(&obj->buffer) > 0)
            {
                uint8_t val = _cyhal_uart_read_from_fifo(obj);
                ring_buffer_write(&obj->buffer, &val, 1u);
            }
        }
        if(_cyhal_uart_get_trigger_level(obj, true) != obj->user_rx_trigger_level)
        {
            /* We set the trigger level lower than requested to support an async tranfer.
             * Don't notify the user because we haven't reached their requested level yet */
            event &= ~CYHAL_UART_IRQ_RX_FIFO;
        }
    }

    if(0u != (event & (CYHAL_UART_IRQ_TX_EMPTY | CYHAL_UART_IRQ_TX_FIFO)))
    {
        if(NULL != obj->async_tx_buff)
        {
            size_t tx_length = obj->async_tx_length;
            /* Okay to cast away volatile here because no other interrupt will be changing
             * this out from under us */
            cy_rslt_t result = cyhal_uart_write(obj, (void *)obj->async_tx_buff, &tx_length);
            if(CY_RSLT_SUCCESS == result)
            {
                obj->async_tx_buff = (void *)(((size_t)(obj->async_tx_buff)) + tx_length);
                obj->async_tx_length -= tx_length;
                if(obj->async_tx_length == 0)
                {
                    obj->async_tx_buff = NULL;
                    event |= CYHAL_UART_IRQ_TX_TRANSMIT_IN_FIFO;
                }
                result = _cyhal_uart_update_fifo_level(obj, CYHAL_UART_FIFO_RX);
                _cyhal_uart_update_enabled_events(obj);
            }
            CY_ASSERT(CY_RSLT_SUCCESS == result); /* Write should never fail here in normal operation */
        }

        if(_cyhal_uart_get_trigger_level(obj, false) != obj->user_tx_trigger_level)
        {
            /* We set the trigger level lower than requested to support an async tranfer.
             * Don't notify the user because we haven't reached their requested level yet */
            event &= ~CYHAL_UART_IRQ_TX_FIFO;
        }
    }

    _cyhal_uart_invoke_user_callback(obj, event);
}

void _cyhal_uart_irq_handler_fast(void)
{
    uint32_t int_status = CHIPCOMMON_SECI_STATUS_REG;
    cyhal_uart_t* obj = _cyhal_uart_config_structs[_CYHAL_UART_FAST];
    if(NULL != obj)
    {
        /* We can get NULL here because the system driver can't tell whether the interrupt was
         * caused by FAST or GCI */
        _cyhal_uart_irq_handler(obj, int_status);
    }
}

void _cyhal_uart_irq_handler_gci(void)
{
    uint32_t int_status = GCI_INT_STATUS_REG;
    /* We can get NULL here because the system driver can't tell whether the interrupt was
     * caused by FAST or GCI */
    cyhal_uart_t* obj = _cyhal_uart_config_structs[_CYHAL_UART_GCI];
    if(NULL != obj)
    {
        /* We can get NULL here because the system driver can't tell whether the interrupt was
         * caused by FAST or GCI */
        _cyhal_uart_irq_handler(obj, int_status);
    }
}

void _cyhal_uart_irq_handler_dbg(void)
{
    uint32_t int_status = _CYHAL_UART_SLOW_BASE->lsr;
    _cyhal_uart_irq_handler(_cyhal_uart_config_structs[_CYHAL_UART_DBG], int_status);
}


/*******************************************************************************
*       Internal - FIFO
*******************************************************************************/

bool _cyhal_uart_fifo_readable(cyhal_uart_t* obj)
{
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            return (0u == (CHIPCOMMON_SECI_STATUS_REG & CC_SECI_STATUS_RX_FIFO_EMPTY));
        case _CYHAL_UART_DBG:
            return (0u != (_CYHAL_UART_SLOW_BASE->lsr & UART_SLOW_LSR_RXRDY));
        case _CYHAL_UART_GCI:
            return (0u != (GCI_INT_STATUS_REG & GCI_INT_ST_MASK_SECI_RX_FIFO_NOT_EMPTY));
        default:
            CY_ASSERT(false);
            return false;
    }
}

static uint8_t _cyhal_uart_read_from_fifo(cyhal_uart_t* obj)
{
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            return _CYHAL_UART_FAST_BASE->data;
        case _CYHAL_UART_DBG:
            return _CYHAL_UART_SLOW_BASE->rx_tx_dll;
        case _CYHAL_UART_GCI:
            return _CYHAL_UART_GCI_BASE->data;
        default:
            CY_ASSERT(false);
            return 0u;
    }
}

static void _cyhal_uart_write_to_fifo(cyhal_uart_t *obj, uint8_t data)
{
    switch(obj->resource.block_num)
    {
        case _CYHAL_UART_FAST:
            _CYHAL_UART_FAST_BASE->data = data;
            break;
        case _CYHAL_UART_DBG:
            _CYHAL_UART_SLOW_BASE->rx_tx_dll = data;
            break;
        case _CYHAL_UART_GCI:
            _CYHAL_UART_GCI_BASE->data = data;
            break;
        default:
            CY_ASSERT(false);
            break;
    }
}

#if defined(__cplusplus)
}
#endif
