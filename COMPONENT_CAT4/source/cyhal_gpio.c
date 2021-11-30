/***************************************************************************//**
* \file cyhal_gpio.c
*
* Description:
* Provides a high level interface for interacting with the GPIO.
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

#include "cyhal_gpio.h"
#include "cyhal_hwmgr.h"
#include "cyhal_system.h"
#include "cy_utils.h"
#include "platform_appscr4.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
*       Internal - MCU powersave LPM
*******************************************************************************/

static void _cyhal_gpio_mcu_powersave_misc_wakeup_config_enable(bool enable)
{
    uint32_t flags = cyhal_system_critical_section_enter();

    if (enable)
    {
        // PMU_GCI2WL_WAKE
        _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_GCI2WL_WAKE_REG,
                                    (uint32_t)PMU_CHIPCONTROL_GCI2WL_WAKE_MASK,
                                    (uint32_t)PMU_CHIPCONTROL_GCI2WL_WAKE_EN);
        // PMU_GCI_WAKE
        PLATFORM_PMU->ext_wake_mask1.bits.gci = 1;
        // GCI_GPIO_WAKE
        GCI_WAKEMASK_REG->bits.gci_gpio_wake = 1;
    }
    else
    {
        // PMU_GCI2WL_WAKE:
        _cyhal_system_pmu_chipcontrol(PMU_CHIPCONTROL_GCI2WL_WAKE_REG,
                                    (uint32_t)PMU_CHIPCONTROL_GCI2WL_WAKE_MASK,
                                    0x0UL);
        // PMU_GCI_WAKE:
        PLATFORM_PMU->ext_wake_mask1.bits.gci = 0;
        // GCI_GPIO_WAKE:
        GCI_WAKEMASK_REG->bits.gci_gpio_wake = 0;
    }

    cyhal_system_critical_section_exit(flags);
}

void _cyhal_gpio_mcu_powersave_gci_gpio_wakeup_enable(cyhal_gpio_t pin, cyhal_gpio_event_t trigger)
{
    /*
     * Acking may be required after the wakeup enabled.
     * This is due to enabling also programs pull up/down, and this may trigger interrupt.
     * Triggering is driven by ILP clock so some small delay may be seen before interrupt triggered.
     */
    uint32_t gpio_wake_mask  = 0x0;

    if (trigger & CYHAL_GPIO_IRQ_RISE)
    {
        gpio_wake_mask |= GCI_CHIPCONTROL_GPIO_WAKEMASK_POS_EDGE(pin);
    }
    if (trigger & CYHAL_GPIO_IRQ_FALL)
    {
        gpio_wake_mask |= GCI_CHIPCONTROL_GPIO_WAKEMASK_NEG_EDGE(pin);
    }

    _cyhal_system_gci_gpiowakemask(GCI_CHIPCONTROL_GPIO_WAKEMASK_REG(pin),
                               GCI_CHIPCONTROL_GPIO_WAKEMASK_MASK(pin),
                               gpio_wake_mask);
    _cyhal_system_gci_gpiocontrol(GCI_CHIPCONTROL_GPIO_CONTROL_REG(pin),
                              GCI_CHIPCONTROL_GPIO_CONTROL_EXTRA_GPIO_ENABLE_MASK(pin),
                              GCI_CHIPCONTROL_GPIO_CONTROL_EXTRA_GPIO_ENABLE_SET(pin));
    _cyhal_gpio_mcu_powersave_misc_wakeup_config_enable(true);

    /* Ack IRQ that might have triggered in configuring GPIO to wake up from Deep sleep */
    PLATFORM_PMU->ext_wakeup_status.bits.gci = 1;
}

void _cyhal_gpio_mcu_powersave_gci_gpio_wakeup_disable(cyhal_gpio_t pin)
{
    _cyhal_gpio_mcu_powersave_misc_wakeup_config_enable(false);
    _cyhal_system_gci_gpiocontrol(GCI_CHIPCONTROL_GPIO_CONTROL_REG(pin),
                              GCI_CHIPCONTROL_GPIO_CONTROL_EXTRA_GPIO_ENABLE_MASK(pin),
                              0x0);
    _cyhal_system_gci_gpiowakemask(GCI_CHIPCONTROL_GPIO_WAKEMASK_REG(pin),
                               GCI_CHIPCONTROL_GPIO_WAKEMASK_MASK(pin),
                               0x0);
}

cy_rslt_t _cyhal_gpio_deepsleep_wakeup_enable(cyhal_gpio_t pin, cyhal_gpio_event_t trigger, bool enable)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* According to WICED, only GPIO0 ~ GPIO15 (excluding GPIO13) support deep-sleep wakeup. */
    if ((pin <= PIN_GPIO_15) && (pin != PIN_GPIO_13))
    {
        if (enable)
            _cyhal_gpio_mcu_powersave_gci_gpio_wakeup_enable(pin, trigger);
        else
            _cyhal_gpio_mcu_powersave_gci_gpio_wakeup_disable(pin);
    }
    else
    {
        result = CYHAL_GPIO_RSLT_ERR_BAD_PARAM;
    }

    return result;
}

/*******************************************************************************
*       Internal - General
*******************************************************************************/

#define _CYHAL_GPIO_INVALID_PIN     (0xFFu)
#define _CYHAL_GPIO_MAX_NUM         (32)

/* Callback array for GPIO interrupts */
static cyhal_gpio_callback_data_t* _cyhal_gpio_callbacks = NULL;

/* Internal gpio configuration struct */
typedef struct
{
    uint8_t output_state;
    uint8_t output_enable;
    uint8_t pullup_enable;
    uint8_t pulldown_enable;
} _cyhal_gpio_config_t;

/* Check if pin allows GPIO functionality */
static inline bool _cyhal_gpio_is_valid(cyhal_gpio_t pin)
{
    return ((pin <= PIN_GPIO_16) || ((PIN_SPI_0_MISO <= pin) && (pin <= PIN_I2S_SCLK1)));
}

/* Map the pin pad enum to the internal bitfield location */
static uint8_t _cyhal_gpio_get_bit(cyhal_gpio_t pin)
{
    uint8_t pin_bit = 0;

    if (pin <= PIN_GPIO_16)
        pin_bit = (uint8_t)pin; // GPIO 0~16
    else if ((PIN_SPI_0_MISO <= pin) && (pin <= PIN_I2S_SCLK1))
        pin_bit = (uint8_t)(pin - PIN_SPI_0_MISO + 17); // GPIO 17~31
    else
        CY_ASSERT(false); // Should not get here if _cyhal_gpio_is_valid() is used

    return pin_bit;
}

/*******************************************************************************
*       Internal - Interrupt Service Routine
*******************************************************************************/

void _cyhal_gpio_irq_handler(void)
{
    cyhal_gpio_event_t polarity_event = (cyhal_gpio_event_t)(PLATFORM_CHIPCOMMON->gpio.int_polarity);
    uint32_t gpio_event = PLATFORM_CHIPCOMMON->gpio.event;
    uint32_t interrupt_masked = gpio_event & PLATFORM_CHIPCOMMON->gpio.event_int_mask;

    // Clear ganged interrupt
    if (interrupt_masked != 0UL)
    {
        PLATFORM_CHIPCOMMON->gpio.event = interrupt_masked;
    }

    // Iterate through the interrupt sources
    cyhal_gpio_callback_data_t* cb_data = _cyhal_gpio_callbacks;
    while (NULL != cb_data)
    {
        uint8_t bit = _cyhal_gpio_get_bit(cb_data->pin);
        uint32_t mask = 1UL << bit;

        if (interrupt_masked & mask)
        {
            cb_data->callback(cb_data->callback_arg, polarity_event); /* Call registered callback */
        }
        cb_data = cb_data->next;
    }
}


/*******************************************************************************
*       HAL Implementation
*******************************************************************************/

cy_rslt_t cyhal_gpio_init(cyhal_gpio_t pin, cyhal_gpio_direction_t direction, cyhal_gpio_drive_mode_t drive_mode, bool init_val)
{
    cy_rslt_t status;

    if (_cyhal_gpio_is_valid(pin))
    {
        cyhal_resource_inst_t pinRsc = { CYHAL_RSC_GPIO, pin, 0 };
        status = cyhal_hwmgr_reserve(&pinRsc);
    }
    else
    {
        status = CYHAL_GPIO_RSLT_ERR_BAD_PARAM;
    }

    if (status == CY_RSLT_SUCCESS)
    {
        status = cyhal_gpio_configure(pin, direction, drive_mode);
        if (status == CY_RSLT_SUCCESS)
        {
            // Set initial value
            uint8_t pin_bit = _cyhal_gpio_get_bit(pin);
            PLATFORM_CHIPCOMMON->gpio.output = (PLATFORM_CHIPCOMMON->gpio.output & (~(1UL << pin_bit))) | ((init_val) ? (1UL << pin_bit) : 0);
        }
    }

    if (status == CY_RSLT_SUCCESS)
    {
        // Note: This is a simplified pin mux for enabling GPIO functionality on dedicated pins.
        //       It's possible to perform a lookup similar to other hardware blocks in the future, if needed.

        /* CC reg mux must be set to 1 (pins 0~16) and to 2 (pins 17~31) for the pin to work as a sw-controlled GPIO */
        int pin_function = (pin <= PIN_GPIO_16) ? PIN_MUX_SEL_1 :
                        ((PIN_SPI_0_MISO <= pin) && (pin <= PIN_I2S_SCLK1)) ? PIN_MUX_SEL_2 : PIN_MUX_SEL_0;

        _cyhal_system_pinmux_connect(pin, pin_function);
    }

    if (status != CY_RSLT_SUCCESS)
    {
        cyhal_gpio_free(pin);
    }

    return status;
}

void cyhal_gpio_free(cyhal_gpio_t pin)
{
    CY_ASSERT(_cyhal_gpio_is_valid(pin));
    if (pin != CYHAL_NC_PIN_VALUE)
    {
        cyhal_gpio_register_callback(pin, NULL);
        cyhal_gpio_enable_event(pin, CYHAL_GPIO_IRQ_NONE, 0, false);

        /* Reset the appropriate ChipCommon GPIO registers */
        (void)cyhal_gpio_configure(pin, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE);

        cyhal_resource_inst_t pinRsc = { CYHAL_RSC_GPIO, pin, 0 };
        cyhal_hwmgr_free(&pinRsc);
    }
}

cy_rslt_t cyhal_gpio_configure(cyhal_gpio_t pin, cyhal_gpio_direction_t direction, cyhal_gpio_drive_mode_t drive_mode)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint32_t flags;
    _cyhal_gpio_config_t pin_gpio_conf = {0};

    if (_cyhal_gpio_is_valid(pin))
    {
        if((direction == CYHAL_GPIO_DIR_OUTPUT) || (direction == CYHAL_GPIO_DIR_BIDIRECTIONAL))
            pin_gpio_conf.output_enable = 1;

        switch (drive_mode)
        {
            case CYHAL_GPIO_DRIVE_NONE:
            case CYHAL_GPIO_DRIVE_ANALOG:
                break;
            case CYHAL_GPIO_DRIVE_PULLUP:
                pin_gpio_conf.pullup_enable = 1;
                break;
            case CYHAL_GPIO_DRIVE_PULLDOWN:
                pin_gpio_conf.pulldown_enable = 1;
                break;
            case CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW:
                break;
            case CYHAL_GPIO_DRIVE_OPENDRAINDRIVESHIGH:
                pin_gpio_conf.output_state = 1;
                break;
            case CYHAL_GPIO_DRIVE_STRONG:
                break;
            case CYHAL_GPIO_DRIVE_PULLUPDOWN:
                pin_gpio_conf.pullup_enable = 1;
                pin_gpio_conf.pulldown_enable = 1;
                break;
            case CYHAL_GPIO_DRIVE_PULL_NONE:
                break;
            default:
                status = CYHAL_GPIO_RSLT_ERR_BAD_PARAM;
                break;
        }
    }
    else
    {
        status = CYHAL_GPIO_RSLT_ERR_BAD_PARAM;
    }

    if (status == CY_RSLT_SUCCESS)
    {
        uint8_t pin_bit = _cyhal_gpio_get_bit(pin);
        flags = cyhal_system_critical_section_enter();

        /* Initialize the appropriate ChipCommon GPIO registers */
        PLATFORM_CHIPCOMMON->gpio.pull_down     = (PLATFORM_CHIPCOMMON->gpio.pull_down     & (~(1UL << pin_bit))) | ((pin_gpio_conf.pulldown_enable)? (1UL << pin_bit) : 0);
        PLATFORM_CHIPCOMMON->gpio.pull_up       = (PLATFORM_CHIPCOMMON->gpio.pull_up       & (~(1UL << pin_bit))) | ((pin_gpio_conf.pullup_enable)  ? (1UL << pin_bit) : 0);
        PLATFORM_CHIPCOMMON->gpio.output        = (PLATFORM_CHIPCOMMON->gpio.output        & (~(1UL << pin_bit))) | ((pin_gpio_conf.output_state)   ? (1UL << pin_bit) : 0);
        PLATFORM_CHIPCOMMON->gpio.output_enable = (PLATFORM_CHIPCOMMON->gpio.output_enable & (~(1UL << pin_bit))) | ((pin_gpio_conf.output_enable)  ? (1UL << pin_bit) : 0);
        PLATFORM_CHIPCOMMON->gpio.control       = (PLATFORM_CHIPCOMMON->gpio.control       & (~(1UL << pin_bit)));

        cyhal_system_critical_section_exit(flags);
    }

    return status;
}

void cyhal_gpio_write(cyhal_gpio_t pin, bool value)
{
    CY_ASSERT(_cyhal_gpio_is_valid(pin));
    uint32_t flags;
    uint8_t pin_bit = _cyhal_gpio_get_bit(pin);

    flags = cyhal_system_critical_section_enter();

    if (value)
        PLATFORM_CHIPCOMMON->gpio.output |= (1UL << pin_bit);
    else
        PLATFORM_CHIPCOMMON->gpio.output &= (~(1UL << pin_bit));

    cyhal_system_critical_section_exit(flags);
}

bool cyhal_gpio_read(cyhal_gpio_t pin)
{
    CY_ASSERT(_cyhal_gpio_is_valid(pin));
    uint8_t pin_bit = _cyhal_gpio_get_bit(pin);
    return ((PLATFORM_CHIPCOMMON->gpio.input & (1UL << pin_bit)) == 0) ? false : true;
}

void cyhal_gpio_toggle(cyhal_gpio_t pin)
{
    CY_ASSERT(_cyhal_gpio_is_valid(pin));
    uint8_t pin_bit = _cyhal_gpio_get_bit(pin);
    cyhal_gpio_write(pin, !cyhal_gpio_read((cyhal_gpio_t)pin_bit));
}

void cyhal_gpio_register_callback(cyhal_gpio_t pin, cyhal_gpio_callback_data_t* callback_data)
{
    CY_ASSERT(_cyhal_gpio_is_valid(pin));
    uint32_t flags = cyhal_system_critical_section_enter();

    // Remove if already registered;
    cyhal_gpio_callback_data_t** ptr = &(_cyhal_gpio_callbacks);
    while (NULL != *ptr)
    {
        if ((*ptr)->pin == pin)
        {
            *ptr = (*ptr)->next;
            break;
        }
        ptr = &((*ptr)->next);
    }
    // Add if requested
    if (NULL != callback_data)
    {
        callback_data->pin = pin;
        callback_data->next = _cyhal_gpio_callbacks;
        _cyhal_gpio_callbacks = callback_data;
    }

    cyhal_system_critical_section_exit(flags);
}

void cyhal_gpio_enable_event(cyhal_gpio_t pin, cyhal_gpio_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(intr_priority);
    CY_ASSERT(_cyhal_gpio_is_valid(pin));
    uint32_t flags;
    uint8_t pin_bit = _cyhal_gpio_get_bit(pin);
    uint32_t cc_gpio_bit_mask = (1UL << pin_bit);

    flags = cyhal_system_critical_section_enter();

    /* Disable GPIO interrupts */
    PLATFORM_CHIPCOMMON->gpio.int_mask &= ~cc_gpio_bit_mask;
    PLATFORM_CHIPCOMMON->gpio.event_int_mask &= ~cc_gpio_bit_mask;

    if ((enable) && (event != CYHAL_GPIO_IRQ_NONE))
    {
        if (event == CYHAL_GPIO_IRQ_RISE)
        {
            PLATFORM_CHIPCOMMON->gpio.event_int_polarity &= ~cc_gpio_bit_mask;
        }
        else if (event == CYHAL_GPIO_IRQ_FALL)
        {
            PLATFORM_CHIPCOMMON->gpio.event_int_polarity |= cc_gpio_bit_mask;
        }
        else
        {
            // Both edge interrupt is not supported
            CY_ASSERT(false);
        }

        /* Clear and enable the GPIO edge interrupt */
        PLATFORM_CHIPCOMMON->gpio.event |= cc_gpio_bit_mask;
        PLATFORM_CHIPCOMMON->gpio.event_int_mask |= cc_gpio_bit_mask;
    }

    cyhal_system_critical_section_exit(flags);

    /* Automatically enable deep-sleep wakeup if available */
    _cyhal_gpio_deepsleep_wakeup_enable(pin, event, enable);

    /* Make sure GPIO interrupts are enabled in ChipCommon interrupt mask */
    _cyhal_system_common_chipcontrol(&(PLATFORM_CHIPCOMMON->interrupt.mask.raw), 0x0, CHIPCOMMON_GPIO_INT_MASK);

    /* Make sure ChipCommon Core external interrupt to APPS core is enabled */
    _cyhal_system_chipcommon_enable_irq();
}

cy_rslt_t cyhal_gpio_connect_digital(cyhal_gpio_t pin, cyhal_source_t source)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(pin);
    CY_UNUSED_PARAMETER(source);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_gpio_enable_output(cyhal_gpio_t pin, cyhal_signal_type_t type, cyhal_source_t *source)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(pin);
    CY_UNUSED_PARAMETER(type);
    CY_UNUSED_PARAMETER(source);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_gpio_disconnect_digital(cyhal_gpio_t pin, cyhal_source_t source)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(pin);
    CY_UNUSED_PARAMETER(source);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_gpio_disable_output(cyhal_gpio_t pin)
{
    /* 43907 does not support this functionality */
    CY_UNUSED_PARAMETER(pin);
    return CY_RSLT_SUCCESS;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#ifdef CYHAL_GPIO_IMPL_HEADER
#include CYHAL_GPIO_IMPL_HEADER
#endif /* CYHAL_GPIO_IMPL_HEADER */

/** \} group_hal_gpio */
