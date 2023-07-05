/***************************************************************************//**
* \file cyhal_i2c.c
*
* Description:
* Provides a high level interface for interacting with the I2C.
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

#include "cyhal_i2c.h"

#include "cyhal_utils.h"
#include "cyhal_hwmgr.h"
#include "cyhal_system_impl.h"
#include "cyhal_clock.h"
#include "cyhal_syspm.h"

#include "typedefs.h"
#include "sbchipc.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/**
 * \addtogroup group_hal_impl_i2c I2C
 * \ingroup group_hal_impl
 * \{
 * \section section_hal_impl_i2c_features I2C features
 *
 * The CAT4 I2C is a master-only driver supporting the following features:
 * * Frequencies - low speed (10 kHz), standard speed (100 kHz), and high speed (400 kHz)
 * * 7-bit address
 *
 * The following features are not available on these devices:
 * * Interrupts and events
 * * Non-blocking communication
 * * FIFO control
 * * Trigger connections
 *
 * The I2C hardware is always clocked using the HT clock. NULL should be passed in
 * the initialization function for the clk parameter.
 *
 *
 * \} group_hal_impl_i2c
 */

/*******************************************************************************
*       Internal
*******************************************************************************/

#define _CYHAL_I2C_MASTER_DEFAULT_FREQ             (100000)
#define _CYHAL_I2C_INSTANCES                       (2)

typedef struct {
    volatile gsio_control_register_t       *reg_control;
    volatile uint32_t                      *reg_data;
    volatile uint32_t                      *reg_address;
    volatile uint32_t                      *reg_clock_div;
}_cyhal_i2c_gsio_regs_t;

static const _cyhal_i2c_gsio_regs_t i2c_gsio_regs[_CYHAL_I2C_INSTANCES] =
{
    /* BCM4390X_GSIO_2 (I2C0) */
    {
        .reg_control = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_2.control),
        .reg_address = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_2.address),
        .reg_data = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_2.data),
        .reg_clock_div = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_2.clock_divider)
    },
    /* BCM4390X_GSIO_3 (I2C1) */
    {
        .reg_control = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_3.control),
        .reg_address = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_3.address),
        .reg_data = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_3.data),
        .reg_clock_div = &(PLATFORM_CHIPCOMMON->general_serial_io_controllers.interface_3.clock_divider)
    },
};

static const gsio_control_register_t _cyhal_i2c_control_reg_init = {
    .i2c_bits = {
        .i2c_data_source = 0b010, // GsioData register
        .start_condition_and_address_size = 0b00, // No STA
        .number_of_data_bytes = 0, // 1 byte/transaction
        .no_send_or_check_ack = 0, // Send/check ACK
        .generate_stop_condition = 0, // No STOP
        .big_endian = 1, // Big endian
        .i2c_mode = 1 // I2C Mode
    }
};

static const gsio_control_register_t _cyhal_i2c_control_reg_stop = {
    .i2c_bits = {
        .i2c_data_source = 0b010, // GsioData register
        .start_condition_and_address_size = 0b00, // No STA
        .number_of_data_bytes = 0, // 1 byte/transaction
        .read = 1, // Read
        .no_send_or_check_ack = 1, // No send/check ACK
        .generate_stop_condition = 1, // STOP condition
        .big_endian = 1, // Big endian
        .i2c_mode = 1, // I2C Mode
        .start_busy = 1 // Start transmission
    }
};

static bool _cyhal_i2c_is_busy(cyhal_i2c_t *obj)
{
    return (bool)(i2c_gsio_regs[obj->resource.block_num].reg_control->i2c_bits.start_busy);
}

static void _cyhal_i2c_wait_for_xfer_complete(cyhal_i2c_t *obj, uint32_t timeout_ms)
{
    uint32_t wait_us = 1000;

    while (_cyhal_i2c_is_busy(obj))
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

static void _cyhal_i2c_set_divider(cyhal_i2c_t *obj, uint32_t frequency_hz)
{
    // Refer to section 2.4.1.1 of 002-18042
    uint32_t backpane_freq = cyhal_clock_get_frequency(&CYHAL_CLOCK_HT);
    uint32_t divider = backpane_freq / frequency_hz;
    if (backpane_freq % frequency_hz != 0)
        divider++;
    divider /= 4;
    divider -= 1;

    divider <<= GSIO_GD_SHIFT;
    divider &= GSIO_GD_MASK;

    *(i2c_gsio_regs[obj->resource.block_num].reg_clock_div) = divider;
}

/*******************************************************************************
*       System Power Management
*******************************************************************************/

#define _CYHAL_I2C_PM_MASK           (0x01UL)

static uint32_t _cyhal_i2c_pm_state = 0UL;
static bool _cyhal_i2c_pm_transition_pending = false;

static bool _cyhal_i2c_pm_has_enabled(void)
{
    for (uint8_t idx = 0; idx < _CYHAL_I2C_INSTANCES; idx++)
    {
        if ((_cyhal_i2c_pm_state >> idx) & _CYHAL_I2C_PM_MASK)
            return true;
    }
    return false;
}

static bool _cyhal_i2c_pm_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);

    switch(mode)
    {
        case CYHAL_SYSPM_CHECK_READY:
            _cyhal_i2c_pm_transition_pending = true;
            break;
        case CYHAL_SYSPM_CHECK_FAIL:
        case CYHAL_SYSPM_AFTER_TRANSITION:
            _cyhal_i2c_pm_transition_pending = false;
            break;
        default:
            break;
    }

    return true;
}

static cyhal_syspm_callback_data_t _cyhal_i2c_syspm_callback_data =
{
    .callback = &_cyhal_i2c_pm_callback,
    .states = (cyhal_syspm_callback_state_t) (CYHAL_SYSPM_CB_CPU_DEEPSLEEP | CYHAL_SYSPM_CB_SYSTEM_HIBERNATE),
    .ignore_modes = (cyhal_syspm_callback_mode_t) 0,
    .args = NULL,
    .next = NULL
};


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_i2c_init(cyhal_i2c_t *obj, cyhal_gpio_t sda, cyhal_gpio_t scl, const cyhal_clock_t *clk)
{
    CY_ASSERT(NULL != obj);

    if (NULL != clk)
        return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;

    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    obj->resource.type = CYHAL_RSC_INVALID;
    obj->pin_sda = NC;
    obj->pin_scl = NC;

    const cyhal_resource_pin_mapping_t *sda_map = _CYHAL_UTILS_GET_RESOURCE(sda, cyhal_pin_map_i2c_sda);
    const cyhal_resource_pin_mapping_t *scl_map = _CYHAL_UTILS_GET_RESOURCE(scl, cyhal_pin_map_i2c_scl);
    if ((NULL == sda_map) || (NULL == scl_map) || !_cyhal_utils_map_resources_equal(sda_map, scl_map))
    {
        return CYHAL_I2C_RSLT_ERR_INVALID_PIN;
    }

    cyhal_resource_inst_t rsc = { CYHAL_RSC_I2C, sda_map->block_num, sda_map->channel_num };
    rslt = cyhal_hwmgr_reserve(&rsc);

    cyhal_resource_inst_t pinRsc;
    if (rslt == CY_RSLT_SUCCESS)
    {
        obj->resource = rsc;

        pinRsc = (cyhal_resource_inst_t) { CYHAL_RSC_GPIO, sda, 0 };
        rslt = cyhal_hwmgr_reserve(&pinRsc);
        if (rslt == CY_RSLT_SUCCESS)
            obj->pin_sda = sda;
    }
    if (rslt == CY_RSLT_SUCCESS)
    {
        pinRsc = (cyhal_resource_inst_t) { CYHAL_RSC_GPIO, scl, 0 };
        rslt = cyhal_hwmgr_reserve(&pinRsc);
        if (rslt == CY_RSLT_SUCCESS)
            obj->pin_scl = scl;
    }
    if (rslt == CY_RSLT_SUCCESS)
    {
        _cyhal_system_pinmux_connect(sda_map->pin, sda_map->function);
        _cyhal_system_pinmux_connect(scl_map->pin, scl_map->function);

        // Initial values
        i2c_gsio_regs[obj->resource.block_num].reg_control->raw = _cyhal_i2c_control_reg_init.raw;
        _cyhal_i2c_set_divider(obj, _CYHAL_I2C_MASTER_DEFAULT_FREQ);

        obj->callback_data.callback = NULL;
        obj->callback_data.callback_arg = NULL;

        if (!_cyhal_i2c_pm_has_enabled())
        {
            _cyhal_syspm_register_peripheral_callback(&_cyhal_i2c_syspm_callback_data);
        }
        _cyhal_i2c_pm_state |= (_CYHAL_I2C_PM_MASK << obj->resource.block_num);
    }

    if (rslt != CY_RSLT_SUCCESS)
        cyhal_i2c_free(obj);

    return rslt;
}

void cyhal_i2c_free(cyhal_i2c_t *obj)
{
    CY_ASSERT(NULL != obj);

    if (obj->resource.type != CYHAL_RSC_INVALID)
    {
        cyhal_hwmgr_free(&(obj->resource));
        obj->resource.type = CYHAL_RSC_INVALID;
    }

    _cyhal_utils_release_if_used(&(obj->pin_sda));
    _cyhal_utils_release_if_used(&(obj->pin_scl));

    _cyhal_i2c_pm_state &= ~(_CYHAL_I2C_PM_MASK << obj->resource.block_num);

    if (!_cyhal_i2c_pm_has_enabled())
    {
        _cyhal_syspm_unregister_peripheral_callback(&_cyhal_i2c_syspm_callback_data);
    }
}

cy_rslt_t cyhal_i2c_configure(cyhal_i2c_t *obj, const cyhal_i2c_cfg_t *cfg)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(NULL != cfg);

    if ((cfg->is_slave) || (!cfg->is_slave && cfg->address != 0))
    {
        return CYHAL_I2C_RSLT_ERR_BAD_ARGUMENT;
    }

    // According to the 43907 programmer's guide, only these 3 speeds are supported (LS, SS, HS)
    if ((cfg->frequencyhal_hz != 10000) &&
        (cfg->frequencyhal_hz != 100000) &&
        (cfg->frequencyhal_hz != 400000))
    {
        return CYHAL_I2C_RSLT_ERR_CAN_NOT_REACH_DR;
    }

    _cyhal_i2c_set_divider(obj, cfg->frequencyhal_hz);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_i2c_configure_adv(cyhal_i2c_t *obj, const cyhal_i2c_adv_cfg_t *cfg)
{
    CY_ASSERT(NULL != cfg);
    CY_ASSERT(NULL != obj);
    CY_ASSERT(NULL != &cfg->basic_cfg);

    if (cfg->address_mask != 0 || cfg->enable_address_callback)
    {
        return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
    }
    else
    {
        return cyhal_i2c_configure(obj, &cfg->basic_cfg);
    }
}

cy_rslt_t cyhal_i2c_master_write(cyhal_i2c_t *obj, uint16_t dev_addr, const uint8_t *data, uint16_t size, uint32_t timeout, bool send_stop)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(NULL != data);

    if (_cyhal_i2c_pm_transition_pending)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;

    // Skip sending device address if it is 0
    if (dev_addr != 0)
        *(i2c_gsio_regs[obj->resource.block_num].reg_address) = dev_addr;

    cy_rslt_t result = CY_RSLT_SUCCESS;
    gsio_control_register_t control_reg;

    for (uint16_t i = 0; i < size; i++)
    {
        control_reg = _cyhal_i2c_control_reg_init;
        control_reg.i2c_bits.read = 0; // Write

        *(i2c_gsio_regs[obj->resource.block_num].reg_data) = (data[i] << 24); // Place data byte in upper byte of data register

        // First byte -> Generate STA
        if ((i == 0) && (dev_addr != 0))
        {
            control_reg.i2c_bits.start_condition_and_address_size = 0b01; // STA with 7-bit address
        }

        // Last byte -> Generate STOP (if specified)
        if (send_stop && (i == size - 1))
        {
            control_reg.i2c_bits.generate_stop_condition = 1;
        }

        control_reg.i2c_bits.start_busy = 1;
        i2c_gsio_regs[obj->resource.block_num].reg_control->raw = control_reg.raw;
        _cyhal_i2c_wait_for_xfer_complete(obj, timeout);

        // No ACK -> Error
        if (i2c_gsio_regs[obj->resource.block_num].reg_control->i2c_bits.ack_received == 1)
        {
            if (i < size - 1)
            {
                // Stop condition
                i2c_gsio_regs[obj->resource.block_num].reg_control->raw = _cyhal_i2c_control_reg_stop.raw;
                _cyhal_i2c_wait_for_xfer_complete(obj, timeout);
            }
            result = CYHAL_I2C_RSLT_ERR_NO_ACK;
        }
        else if (i2c_gsio_regs[obj->resource.block_num].reg_control->i2c_bits.command_error == 1)
        {
            // Note: Stop condition intentionally skipped as we immediately want to halt comms
            result = CYHAL_I2C_RSLT_ERR_CMD_ERROR;
        }
    }

    return result;
}

cy_rslt_t cyhal_i2c_master_read(cyhal_i2c_t *obj, uint16_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout, bool send_stop)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(NULL != data);

    if (_cyhal_i2c_pm_transition_pending)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;

    // Skip sending device address if it is 0
    if (dev_addr != 0)
        *(i2c_gsio_regs[obj->resource.block_num].reg_address) = dev_addr;

    cy_rslt_t result = CY_RSLT_SUCCESS;
    gsio_control_register_t control_reg;

    for (uint16_t i = 0; i < size; i++)
    {
        control_reg = _cyhal_i2c_control_reg_init;
        control_reg.i2c_bits.read = 1; // Read

        *(i2c_gsio_regs[obj->resource.block_num].reg_data) = 0; // Zero out data register

        // First byte -> Generate STA
        if ((i == 0) && (dev_addr != 0))
        {
            control_reg.i2c_bits.start_condition_and_address_size = 0b01; // STA with 7-bit address
        }

        // Last byte -> Generate STOP (if specified)
        if (send_stop && (i == size - 1))
        {
            control_reg.i2c_bits.generate_stop_condition = 1;
            control_reg.i2c_bits.no_send_or_check_ack = 1; // Do not send ACK in this case
        }

        control_reg.i2c_bits.start_busy = 1;
        i2c_gsio_regs[obj->resource.block_num].reg_control->raw = control_reg.raw;
        _cyhal_i2c_wait_for_xfer_complete(obj, timeout);

        // First byte & no ACK -> Error
        if (i == 0 && size > 1 && i2c_gsio_regs[obj->resource.block_num].reg_control->i2c_bits.ack_received == 1)
        {
            // Stop condition
            i2c_gsio_regs[obj->resource.block_num].reg_control->raw = _cyhal_i2c_control_reg_stop.raw;
            _cyhal_i2c_wait_for_xfer_complete(obj, timeout);
            result = CYHAL_I2C_RSLT_ERR_NO_ACK;
        }
        else if (i2c_gsio_regs[obj->resource.block_num].reg_control->i2c_bits.command_error == 1)
        {
            // Note: Stop condition intentionally skipped as we immediately want to halt comms
            result = CYHAL_I2C_RSLT_ERR_CMD_ERROR;
        }

        data[i] = *(i2c_gsio_regs[obj->resource.block_num].reg_data) >> 24; // Data will be in upper byte of data register
    }

    return result;
}

cy_rslt_t cyhal_i2c_slave_config_write_buffer(cyhal_i2c_t *obj, const uint8_t *data, uint16_t size)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(data);
    CY_UNUSED_PARAMETER(size);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_slave_config_read_buffer(cyhal_i2c_t *obj, uint8_t *data, uint16_t size)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(data);
    CY_UNUSED_PARAMETER(size);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

static cy_rslt_t _cyhal_i2c_master_mem_common(uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *mem_addr_buf)
{
    if (_cyhal_i2c_pm_transition_pending)
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;

    if (mem_addr_size == 1)
    {
        mem_addr_buf[0] = (uint8_t) mem_addr;
    }
    else if (mem_addr_size == 2)
    {
        mem_addr_buf[0] = (uint8_t) (mem_addr >> 8);
        mem_addr_buf[1] = (uint8_t) mem_addr;
    }
    else
    {
        return CYHAL_I2C_RSLT_ERR_INVALID_ADDRESS_SIZE;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_i2c_master_mem_write(cyhal_i2c_t *obj, uint16_t address, uint16_t mem_addr, uint16_t mem_addr_size, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    CY_ASSERT(NULL != obj);

    uint8_t mem_addr_buf[2];
    cy_rslt_t status = _cyhal_i2c_master_mem_common(mem_addr, mem_addr_size, &mem_addr_buf[0]);

    if (status == CY_RSLT_SUCCESS)
    {
        status = cyhal_i2c_master_write(obj, address, mem_addr_buf, mem_addr_size, timeout, false);
    }
    if (status == CY_RSLT_SUCCESS)
    {
        // Note: Skip start and address
        status = cyhal_i2c_master_write(obj, 0, data, size, timeout, true);
    }
    return status;
}

cy_rslt_t cyhal_i2c_master_mem_read(cyhal_i2c_t *obj, uint16_t address, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    CY_ASSERT(NULL != obj);

    uint8_t mem_addr_buf[2];
    cy_rslt_t status = _cyhal_i2c_master_mem_common(mem_addr, mem_addr_size, &mem_addr_buf[0]);

    // Note: Need to generate stop bit as there's no repeat start on this device
    if (status == CY_RSLT_SUCCESS)
    {
        status = cyhal_i2c_master_write(obj, address, mem_addr_buf, mem_addr_size, timeout, true);
    }
    if (status == CY_RSLT_SUCCESS)
    {
        status = cyhal_i2c_master_read(obj, address, data, size, timeout, true);
    }
    return status;
}

cy_rslt_t cyhal_i2c_slave_read(cyhal_i2c_t *obj, uint8_t *dst_buff, uint16_t *size, uint32_t timeout)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(dst_buff);
    CY_UNUSED_PARAMETER(size);
    CY_UNUSED_PARAMETER(timeout);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}
cy_rslt_t cyhal_i2c_slave_write(cyhal_i2c_t *obj, const uint8_t *src_buff, uint16_t *size, uint32_t timeout)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(src_buff);
    CY_UNUSED_PARAMETER(size);
    CY_UNUSED_PARAMETER(timeout);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_slave_abort_read(cyhal_i2c_t *obj)
{
    CY_UNUSED_PARAMETER(obj);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_master_transfer_async(cyhal_i2c_t *obj, uint16_t address, const void *tx, size_t tx_size, void *rx, size_t rx_size)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(address);
    CY_UNUSED_PARAMETER(tx);
    CY_UNUSED_PARAMETER(tx_size);
    CY_UNUSED_PARAMETER(rx);
    CY_UNUSED_PARAMETER(rx_size);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_abort_async(cyhal_i2c_t *obj)
{
    CY_UNUSED_PARAMETER(obj);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

void cyhal_i2c_register_callback(cyhal_i2c_t *obj, cyhal_i2c_event_callback_t callback, void *callback_arg)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(callback);
    CY_UNUSED_PARAMETER(callback_arg);
}

void cyhal_i2c_register_address_callback(cyhal_i2c_t *obj, cyhal_i2c_address_callback_t callback, void *callback_arg)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(callback);
    CY_UNUSED_PARAMETER(callback_arg);
}

void cyhal_i2c_enable_event(cyhal_i2c_t *obj, cyhal_i2c_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(event);
    CY_UNUSED_PARAMETER(intr_priority);
    CY_UNUSED_PARAMETER(enable);
}

void cyhal_i2c_enable_address_event(cyhal_i2c_t *obj, cyhal_i2c_addr_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(event);
    CY_UNUSED_PARAMETER(intr_priority);
    CY_UNUSED_PARAMETER(enable);
}

cy_rslt_t cyhal_i2c_set_fifo_level(cyhal_i2c_t *obj, cyhal_i2c_fifo_type_t type, uint16_t level)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(type);
    CY_UNUSED_PARAMETER(level);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_enable_output(cyhal_i2c_t *obj, cyhal_i2c_output_t output, cyhal_source_t *source)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);
    CY_UNUSED_PARAMETER(source);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_disable_output(cyhal_i2c_t *obj, cyhal_i2c_output_t output)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);

    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

cy_rslt_t cyhal_i2c_init_cfg(cyhal_i2c_t *obj, const cyhal_i2c_configurator_t *cfg)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);

    // No configurators supported on this architecture
    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}

uint32_t cyhal_i2c_slave_readable(cyhal_i2c_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    return 0;
}

uint32_t cyhal_i2c_slave_writable(cyhal_i2c_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    return 0;
}

cy_rslt_t cyhal_i2c_clear(cyhal_i2c_t *obj)
{
    CY_UNUSED_PARAMETER(obj);
    return CYHAL_I2C_RSLT_ERR_UNSUPPORTED;
}


#if defined(__cplusplus)
}
#endif
