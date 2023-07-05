/***************************************************************************//**
* \file cyhal_spi.c
*
* Description:
* Provides a high level interface for interacting with the SPI.
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

#include "cyhal_spi.h"

#include "cyhal_utils.h"
#include "cyhal_hwmgr.h"
#include "cyhal_system_impl.h"
#include "cyhal_clock.h"

#include "typedefs.h"
#include "sbchipc.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/**
 * \addtogroup group_hal_impl_spi SPI
 * \ingroup group_hal_impl
 * \{
 * \section section_hal_impl_spi_features SPI features
 *
 * The CAT4 SPI is a master-only driver supporting the following features:
 * * Standard motorola SPI CPOL=0, CPHA=0 with MSB first operation (CYHAL_SPI_MODE_00_MSB)
 * * 1 dedicated chip select
 *
 * The following features are not available on these devices:
 * * Interrupts and events
 * * Non-blocking communication
 * * FIFO control
 * * Trigger connections
 *
 * The SPI hardware is always clocked using the HT clock. NULL should be passed in
 * the initialization function for the clk parameter.
 *
 * \section section_hal_impl_spi_data_width Supported transfer data width options
 * CAT4 devices support only transfer data width 8 and 16
 *
 * \} group_hal_impl_spi
 */


/*******************************************************************************
*       Internal
*******************************************************************************/

#define _CYHAL_SPI_MASTER_DEFAULT_FREQ      (1000000)
#define _CYHAL_SPI_INSTANCES                (2)
#define _CYHAL_SPI_DUMMY_DATA               (0x97)
#define _CYHAL_SPI_NOD_8BIT_MASK            (0x000000FFUL)
#define _CYHAL_SPI_NOD_8BIT_SHIFT           (8u)
#define _CYHAL_SPI_NOD_16BIT_MASK           (0x0000FF00UL)
#define _CYHAL_SPI_NOD_16BIT_WRITE(value)   (((value) << _CYHAL_SPI_NOD_8BIT_SHIFT) & _CYHAL_SPI_NOD_16BIT_MASK)
#define _CYHAL_SPI_NOD_16BIT_READ(value)    (((value) >> _CYHAL_SPI_NOD_8BIT_SHIFT) & _CYHAL_SPI_NOD_8BIT_MASK)

// Keep a record of last received word
static uint32_t _cyhal_spi_received[_CYHAL_SPI_INSTANCES];

typedef struct {
    volatile gsio_control_register_t       *reg_control;
    volatile uint32_t                      *reg_data;
    volatile uint32_t                      *reg_clock_div;
}_cyhal_spi_gsio_regs_t;

static const _cyhal_spi_gsio_regs_t spi_gsio_regs[_CYHAL_SPI_INSTANCES] =
{
    /* BCM4390X_GSIO_0 (SPI0) */
    {
        .reg_control = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_0.control),
        .reg_data = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_0.data),
        .reg_clock_div = &(PLATFORM_CHIPCOMMON->clock_control.clock_divider_2.raw)
    },
    /* BCM4390X_GSIO_1 (SPI1) */
    {
        .reg_control = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_1.control),
        .reg_data = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_1.data),
        .reg_clock_div = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_1.clock_divider)
    },
};

static bool _cyhal_spi_is_busy(cyhal_spi_t *obj)
{
    return (bool)(spi_gsio_regs[obj->resource.block_num].reg_control->spi_bits.start_busy);
}

static void _cyhal_spi_wait_for_xfer_complete(cyhal_spi_t *obj, uint32_t timeout_ms)
{
    uint32_t wait_us = 1000;

    while (_cyhal_spi_is_busy(obj))
    {
        // Timeout of 0 means wait forever
        if (timeout_ms != 0UL)
        {
            cyhal_system_delay_us(1);
            if (wait_us-- == 0UL)
            {
                wait_us = 1000;
                if (--timeout_ms == 0UL)
                    break;
            }
        }
    }
}

static uint32_t _cyhal_spi_calculate_divider(uint32_t frequency)
{
    // Refer to section 2.8.1 of 002-18042
    uint32_t backpane_freq = cyhal_clock_get_frequency(&CYHAL_CLOCK_HT);
    uint32_t divider = backpane_freq / frequency;
    divider = (divider > 2) ? (divider - 2) : 0;

    divider <<= GSIO_GD_SHIFT;
    divider &= GSIO_GD_MASK;

    return divider;
}

static void _cyhal_spi_toggle_cs(cyhal_spi_t *obj, bool set)
{
    if (set)
        spi_gsio_regs[obj->resource.block_num].reg_control->raw |=  GSIO_GG_SPI_CHIP_SELECT;
    else
        spi_gsio_regs[obj->resource.block_num].reg_control->raw &= (~GSIO_GG_SPI_CHIP_SELECT);
}

static uint32_t _cyhal_spi_pre(cyhal_spi_t *obj)
{
    _cyhal_spi_toggle_cs(obj, true);
    return spi_gsio_regs[obj->resource.block_num].reg_control->raw;
}

static void _cyhal_spi_post(cyhal_spi_t *obj, uint32_t ctrlReg)
{
    spi_gsio_regs[obj->resource.block_num].reg_control->raw = ctrlReg;
    _cyhal_spi_toggle_cs(obj, false);
}

static uint32_t _cyhal_spi_send_datax(cyhal_spi_t *obj, uint32_t value)
{
    uint32_t gsio_control_register_GSIO_NOD =
        (spi_gsio_regs[obj->resource.block_num].reg_control->raw & ~((uint32_t)(GSIO_NOD_MASK))) | obj->numDataBytes;

    spi_gsio_regs[obj->resource.block_num].reg_control->raw = gsio_control_register_GSIO_NOD;

    *(spi_gsio_regs[obj->resource.block_num].reg_data) = value;
    spi_gsio_regs[obj->resource.block_num].reg_control->raw |= GSIO_SB_BUSY;
    _cyhal_spi_wait_for_xfer_complete(obj, 0);
    uint32_t rx_data = *(spi_gsio_regs[obj->resource.block_num].reg_data);

    return rx_data;
}

/*******************************************************************************
*       System Power Management
*******************************************************************************/

#define _CYHAL_SPI_PM_MASK           (0x01UL)

static uint32_t _cyhal_spi_pm_state = 0UL;
static bool _cyhal_spi_pm_transition_pending = false;

static bool _cyhal_spi_pm_has_enabled(void)
{
    for (uint8_t idx = 0; idx < _CYHAL_SPI_INSTANCES; idx++)
    {
        if ((_cyhal_spi_pm_state >> idx) & _CYHAL_SPI_PM_MASK)
            return true;
    }
    return false;
}

static bool _cyhal_spi_pm_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);

    switch(mode)
    {
        case CYHAL_SYSPM_CHECK_READY:
            _cyhal_spi_pm_transition_pending = true;
            break;
        case CYHAL_SYSPM_CHECK_FAIL:
        case CYHAL_SYSPM_AFTER_TRANSITION:
            _cyhal_spi_pm_transition_pending = false;
            break;
        default:
            break;
    }

    return true;
}

static cyhal_syspm_callback_data_t _cyhal_spi_syspm_callback_data =
{
    .callback = &_cyhal_spi_pm_callback,
    .states = (cyhal_syspm_callback_state_t) (CYHAL_SYSPM_CB_CPU_DEEPSLEEP | CYHAL_SYSPM_CB_SYSTEM_HIBERNATE),
    .ignore_modes = (cyhal_syspm_callback_mode_t) 0,
    .args = NULL,
    .next = NULL
};


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_spi_init(cyhal_spi_t *obj, cyhal_gpio_t mosi, cyhal_gpio_t miso, cyhal_gpio_t sclk, cyhal_gpio_t ssel,
                        const cyhal_clock_t *clk, uint8_t bits, cyhal_spi_mode_t mode, bool is_slave)
{
    CY_ASSERT(NULL != obj);

    if ((NULL != clk) || (mode != CYHAL_SPI_MODE_00_MSB) || !((bits == 8u) || (bits == 16u)) || (is_slave))
        return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;

    if ((NC != miso && NC == sclk) || (NC == mosi && NC == miso))
        return CYHAL_SPI_RSLT_PIN_CONFIG_NOT_SUPPORTED;

    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    obj->resource.type = CYHAL_RSC_INVALID;
    obj->pin_mosi = NC;
    obj->pin_miso = NC;
    obj->pin_clk = NC;
    obj->pin_cs = NC;

    const cyhal_resource_pin_mapping_t *mosi_map = _CYHAL_UTILS_GET_RESOURCE(mosi, cyhal_pin_map_spi_mosi);
    const cyhal_resource_pin_mapping_t *miso_map = _CYHAL_UTILS_GET_RESOURCE(miso, cyhal_pin_map_spi_miso);
    const cyhal_resource_pin_mapping_t *sclk_map = _CYHAL_UTILS_GET_RESOURCE(sclk, cyhal_pin_map_spi_clk);
    const cyhal_resource_pin_mapping_t *ssel_map = _CYHAL_UTILS_GET_RESOURCE(ssel, cyhal_pin_map_spi_cs);
    if (((NC != mosi) && (NULL == mosi_map))
        || ((NC != miso) && (NULL == miso_map))
        || ((NC != sclk) && (NULL == sclk_map))
        || ((NC != ssel) && (NULL == ssel_map))
        || ((NC != mosi) && (NC != miso) && !_cyhal_utils_map_resources_equal(mosi_map, miso_map))
        || ((NC != mosi) && (NC != sclk) && !_cyhal_utils_map_resources_equal(mosi_map, sclk_map))
        || ((NC != mosi) && (NC != ssel) && !_cyhal_utils_map_resources_equal(mosi_map, ssel_map))
        || ((NC != miso) && (NC != sclk) && !_cyhal_utils_map_resources_equal(miso_map, sclk_map))
        || ((NC != miso) && (NC != ssel) && !_cyhal_utils_map_resources_equal(miso_map, ssel_map))
        || ((NC != sclk) && (NC != ssel) && !_cyhal_utils_map_resources_equal(sclk_map, ssel_map)))
    {
        return CYHAL_SPI_RSLT_ERR_INVALID_PIN;
    }

    const cyhal_resource_pin_mapping_t *base_map = (NULL != mosi_map) ? mosi_map : miso_map;

    cyhal_resource_inst_t rsc = { CYHAL_RSC_SPI, base_map->block_num, base_map->channel_num };
    rslt = cyhal_hwmgr_reserve(&rsc);

    cyhal_resource_inst_t pinRsc;
    if (rslt == CY_RSLT_SUCCESS)
    {
        obj->resource = rsc;
    }
    if ((rslt == CY_RSLT_SUCCESS) && (NC != mosi))
    {
        pinRsc = (cyhal_resource_inst_t) { CYHAL_RSC_GPIO, mosi, 0 };
        rslt = cyhal_hwmgr_reserve(&pinRsc);
        if (rslt == CY_RSLT_SUCCESS)
        {
            obj->pin_mosi = mosi;
            _cyhal_system_pinmux_connect(mosi_map->pin, mosi_map->function);
        }
    }
    if ((rslt == CY_RSLT_SUCCESS) && (NC != miso))
    {
        pinRsc = (cyhal_resource_inst_t) { CYHAL_RSC_GPIO, miso, 0 };
        rslt = cyhal_hwmgr_reserve(&pinRsc);
        if (rslt == CY_RSLT_SUCCESS)
        {
            obj->pin_miso = miso;
            _cyhal_system_pinmux_connect(miso_map->pin, miso_map->function);
        }
    }
    if ((rslt == CY_RSLT_SUCCESS) && (NC != sclk))
    {
        pinRsc = (cyhal_resource_inst_t) { CYHAL_RSC_GPIO, sclk, 0 };
        rslt = cyhal_hwmgr_reserve(&pinRsc);
        if (rslt == CY_RSLT_SUCCESS)
        {
            obj->pin_clk = sclk;
            _cyhal_system_pinmux_connect(sclk_map->pin, sclk_map->function);
        }
    }
    if ((rslt == CY_RSLT_SUCCESS) && (NC != ssel))
    {
        pinRsc = (cyhal_resource_inst_t) { CYHAL_RSC_GPIO, ssel, 0 };
        rslt = cyhal_hwmgr_reserve(&pinRsc);
        if (rslt == CY_RSLT_SUCCESS)
        {
            obj->pin_cs = ssel;
            _cyhal_system_pinmux_connect(ssel_map->pin, ssel_map->function);
        }
    }
    if (rslt == CY_RSLT_SUCCESS)
    {
        // Initial value
        spi_gsio_regs[obj->resource.block_num].reg_control->raw = (0 & GSIO_GO_MASK) | GSIO_GC_SPI_NOD_DATA_IO | GSIO_NOD_1 | GSIO_GM_SPI;
        *(spi_gsio_regs[obj->resource.block_num].reg_clock_div) = _cyhal_spi_calculate_divider(_CYHAL_SPI_MASTER_DEFAULT_FREQ);

        obj->numDataBytes = (bits == 8u) ? GSIO_NOD_1 : GSIO_NOD_2; // HAL only supports 8 and 16
        obj->callback_data.callback = NULL;
        obj->callback_data.callback_arg = NULL;

        if (!_cyhal_spi_pm_has_enabled())
        {
            _cyhal_syspm_register_peripheral_callback(&_cyhal_spi_syspm_callback_data);
        }
        _cyhal_spi_pm_state |= (_CYHAL_SPI_PM_MASK << obj->resource.block_num);
    }

    if (rslt != CY_RSLT_SUCCESS)
        cyhal_spi_free(obj);

    return rslt;
}

void cyhal_spi_free(cyhal_spi_t *obj)
{
    CY_ASSERT(NULL != obj);

    if (obj->resource.type != CYHAL_RSC_INVALID)
    {
        cyhal_hwmgr_free(&(obj->resource));
        obj->resource.type = CYHAL_RSC_INVALID;
    }

    _cyhal_utils_release_if_used(&(obj->pin_mosi));
    _cyhal_utils_release_if_used(&(obj->pin_miso));
    _cyhal_utils_release_if_used(&(obj->pin_clk));
    _cyhal_utils_release_if_used(&(obj->pin_cs));

    _cyhal_spi_pm_state &= ~(_CYHAL_SPI_PM_MASK << obj->resource.block_num);

    if (!_cyhal_spi_pm_has_enabled())
    {
        _cyhal_syspm_unregister_peripheral_callback(&_cyhal_spi_syspm_callback_data);
    }
}

cy_rslt_t cyhal_spi_set_frequency(cyhal_spi_t *obj, uint32_t hz)
{
    CY_ASSERT(NULL != obj);

    *(spi_gsio_regs[obj->resource.block_num].reg_clock_div) = _cyhal_spi_calculate_divider(hz);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_spi_slave_select_config(cyhal_spi_t *obj, cyhal_gpio_t ssel, cyhal_spi_ssel_polarity_t polarity)
{
    /* Might be possible to use GPIO in the future */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(ssel);
    CY_UNUSED_PARAMETER(polarity);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_spi_select_active_ssel(cyhal_spi_t *obj, cyhal_gpio_t ssel)
{
    /* Might be possible to use GPIO in the future */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(ssel);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_spi_recv(cyhal_spi_t *obj, uint32_t* value)
{
    CY_ASSERT(NULL != obj);


    if (NC == obj->pin_miso)
        return CYHAL_SPI_RSLT_INVALID_PIN_API_NOT_SUPPORTED;

    uint32_t gsio_control_register_backup = _cyhal_spi_pre(obj);
    uint32_t dummyData = (obj->numDataBytes == GSIO_NOD_1) ? _CYHAL_SPI_DUMMY_DATA : (_CYHAL_SPI_DUMMY_DATA << 8u | _CYHAL_SPI_DUMMY_DATA);
    *value = _cyhal_spi_received[obj->resource.block_num] = _cyhal_spi_send_datax(obj, dummyData);
    _cyhal_spi_post(obj, gsio_control_register_backup);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_spi_send(cyhal_spi_t *obj, uint32_t value)
{
    CY_ASSERT(NULL != obj);

    if (NC == obj->pin_mosi)
        return CYHAL_SPI_RSLT_INVALID_PIN_API_NOT_SUPPORTED;

    uint32_t gsio_control_register_backup = _cyhal_spi_pre(obj);
    _cyhal_spi_received[obj->resource.block_num] = _cyhal_spi_send_datax(obj, value);
    _cyhal_spi_post(obj, gsio_control_register_backup);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_spi_transfer(cyhal_spi_t *obj, const uint8_t *tx, size_t tx_length, uint8_t *rx, size_t rx_length, uint8_t write_fill)
{
    CY_ASSERT(NULL != obj);

    uint32_t gsio_control_register_backup = _cyhal_spi_pre(obj);
    uint32_t iterate = (obj->numDataBytes == GSIO_NOD_1) ? 1 : 2;
    uint32_t write_value;
    uint32_t read_value;

    for (uint32_t idx = 0UL; idx < tx_length; idx = idx + iterate)
    {
        if (obj->numDataBytes == GSIO_NOD_2)
        {
            write_value = _CYHAL_SPI_NOD_16BIT_WRITE(tx[idx]);
            if (idx + 1 < tx_length)
                write_value |= tx[idx + 1];
        }
        else // GSIO_NOD_1
        {
            write_value = tx[idx];
        }

        read_value = _cyhal_spi_send_datax(obj, write_value);

        if (idx < rx_length)
        {
            if (obj->numDataBytes == GSIO_NOD_2)
            {
                rx[idx] = _CYHAL_SPI_NOD_16BIT_READ(read_value);
                if (idx + 1 < rx_length)
                    rx[idx + 1] = read_value & _CYHAL_SPI_NOD_8BIT_MASK;
            }
            else // GSIO_NOD_1
            {
                rx[idx] = read_value;
            }
        }
    }

    // Send dummy data
    if (rx_length > tx_length)
    {
        write_value = write_fill;
        if (obj->numDataBytes == GSIO_NOD_2)
            write_value |= _CYHAL_SPI_NOD_16BIT_WRITE(write_fill);

        for (uint32_t idx = tx_length; idx < rx_length; idx++ )
        {
            read_value = _cyhal_spi_send_datax(obj, write_value);

            if (obj->numDataBytes == GSIO_NOD_2)
            {
                rx[idx] = _CYHAL_SPI_NOD_16BIT_READ(read_value);
                if (idx + 1 < rx_length)
                    rx[idx + 1] = read_value & _CYHAL_SPI_NOD_8BIT_MASK;
            }
            else // GSIO_NOD_1
            {
                rx[idx] = read_value;
            }
        }
    }

    _cyhal_spi_post(obj, gsio_control_register_backup);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_spi_transfer_async(cyhal_spi_t *obj, const uint8_t *tx, size_t tx_length, uint8_t *rx, size_t rx_length)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(tx);
    CY_UNUSED_PARAMETER(tx_length);
    CY_UNUSED_PARAMETER(rx);
    CY_UNUSED_PARAMETER(rx_length);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

bool cyhal_spi_is_busy(cyhal_spi_t *obj)
{
    return (bool)spi_gsio_regs[obj->resource.block_num].reg_control->spi_bits.start_busy;
}

cy_rslt_t cyhal_spi_abort_async(cyhal_spi_t *obj)
{
    CY_UNUSED_PARAMETER(obj);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

void cyhal_spi_register_callback(cyhal_spi_t *obj, cyhal_spi_event_callback_t callback, void *callback_arg)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(callback);
    CY_UNUSED_PARAMETER(callback_arg);
}

void cyhal_spi_enable_event(cyhal_spi_t *obj, cyhal_spi_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(event);
    CY_UNUSED_PARAMETER(intr_priority);
    CY_UNUSED_PARAMETER(enable);
}

cy_rslt_t cyhal_spi_set_fifo_level(cyhal_spi_t *obj, cyhal_spi_fifo_type_t type, uint16_t level)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(type);
    CY_UNUSED_PARAMETER(level);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_spi_enable_output(cyhal_spi_t *obj, cyhal_spi_output_t output, cyhal_source_t *source)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);
    CY_UNUSED_PARAMETER(source);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_spi_disable_output(cyhal_spi_t *obj, cyhal_spi_output_t output)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);

    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_spi_init_cfg(cyhal_spi_t *obj, const cyhal_spi_configurator_t *cfg)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);

    // No configurators supported on this architecture
    return CYHAL_SPI_RSLT_ERR_UNSUPPORTED;
}

#if defined(__cplusplus)
}
#endif
