/***************************************************************************//**
* \file cyhal_hw_types.h
*
* \brief
* Provides a struct definitions for configuration resources in the HAL.
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

/**
* \addtogroup group_hal_impl CAT4 (4390X) Implementation Specific
* \{
* This section provides details about the 4390X implementation of the Cypress HAL.
* All information within this section is platform specific and is provided for reference.
* Portable application code should depend only on the APIs and types which are documented
* in the @ref group_hal section.
*
* \note There is no configurator support for 4390X devices. Therefore, all `cyhal_<peripheral>_init_cfg`
* functions are unsupported on this platform, and will return an error if called.
*
* \section group_hal_impl_mapping HAL Resource Hardware Mapping
* The following table shows a mapping of each HAL driver to the lower level firmware driver
* and the corresponding hardware resource. This is intended to help understand how the HAL
* is implemented for 4390X and what features the underlying hardware supports.
*
* | HAL Resource       | CAT4 Hardware                    |
* | ------------------ | -------------------------------- |
* | Clock              | All clocks (system & peripheral) |
* | DMA                | M2M DMA                          |
* | Ethernet           | Ethernet                         |
* | GPIO               | GPIO                             |
* | Hardware Manager   | NA                               |
* | LPTimer            | PMU Timer                        |
* | M2M DMA            | M2M DMA                          |
* | PWM                | PWM                              |
* | SysPM              | System Power Resources           |
* | System             | System Resources                 |
* | Timer              | CPU Timer                        |
* | UART               | Fast, Slow, GCI UARTs            |
* | WDT                | WDT                              |
*/

/**
* \addtogroup group_hal_impl_hw_types 4390X Specific Hardware Types
* \{
* Aliases for types which are part of the public HAL interface but whose representations
* need to vary per HAL implementation
*/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "cmsis_compiler.h"
#include "cyhal_general_types.h"
#include "cyhal_hw_resources.h"
#include "cyhal_pin_package.h"
#include "cyhal_triggers.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CYHAL_ISR_PRIORITY_DEFAULT
/** Priority that is applied by default to all drivers when initialized. Priorities can be
 * overridden on each driver as part of enabling events.
 */
#define CYHAL_ISR_PRIORITY_DEFAULT  (3)
#endif

typedef void (* cy_israddress)(void);   /**< Type of ISR callbacks */
#if defined (__ICCARM__)
    typedef union { cy_israddress __fun; void * __ptr; } cy_intvec_elem;
#endif  /* defined (__ICCARM__) */

/** \addtogroup group_hal_ethernet_header
 *  Ethernet Header field sizes
 *  \{ *//**
 */
#define CYHAL_ETHER_PREAMBLE_LEN       (8)      /**< First part of ethernet frame header, all b10101010 (0xAA) + SFD (0xAB) */
#define CYHAL_ETHER_ADDR_LEN           (6)      /**< The number of bytes (octets) in an ethernet (MAC) address. */
#define CYHAL_ETHER_TYPE_LEN           (2)      /**< The number of bytes in the type field. */
#define CYHAL_ETHER_CRC_LEN            (4)      /**< The number of bytes in the trailing CRC field. */
#define CYHAL_ETHER_HDR_LEN         (CYHAL_ETHER_PREAMBLE_LEN + (CYHAL_ETHER_ADDR_LEN * 2) + CYHAL_ETHER_TYPE_LEN)   /**< The length of the combined header. */
#define CYHAL_ETHER_MIN_LEN           (64)      /**< The minimum packet length. */
#define CYHAL_ETHER_MIN_DATA          (46)      /**< The minimum packet user data length. */
#define CYHAL_ETHER_MAX_LEN         (1518)      /**< The maximum packet length. */
#define CYHAL_ETHER_MAX_DATA        (1500)      /**< The maximum packet user data length. */

/**
 * \}
 */


/** Ethernet MAC address (standard 6-byte (octet) MAC address) */
typedef struct
{
    uint8_t octet[CYHAL_ETHER_ADDR_LEN];    /**< 48-bit Ethernet address */
} cyhal_ether_addr_t;


/** @brief Event callback data object */
typedef struct {
    cy_israddress                       callback;
    void*                               callback_arg;
} cyhal_event_callback_data_t;

/**
* \cond INTERNAL
*/

#define CYHAL_CLOCK_IMPL_HEADER         "cyhal_clock_impl.h"    //!< Implementation specific header for Clocks
#define CYHAL_DMA_IMPL_HEADER           "cyhal_dma_impl.h"      //!< Implementation specific header for DMA
#define _CYHAL_ETHERNET_IMPL_HEADER      "cyhal_ethernet_impl.h" //!< Implementation specific header for Ethernet
#define CYHAL_SYSTEM_IMPL_HEADER        "cyhal_system_impl.h"   //!< Implementation specific header for System
#define CYHAL_LPTIMER_IMPL_HEADER       "cyhal_lptimer_impl.h"  //!< Implementation specific header for Low Power Timer

/** \endcond */

/* Ethernet Clock block and channel */
#define _CYHAL_ETHERNET_CLOCK_BLOCK      CYHAL_CLOCK_BLOCK_BACKPLANE    /**< Clock block */
#define _CYHAL_ETHERNET_CLOCK_CHANNEL    0                              /**< Clock channel */

/**
  * @brief DMA object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    cyhal_resource_inst_t               resource;
    uint8_t                             tx_ch;
    uint8_t                             rx_ch;
    uint8_t                             dma_ch;
    uint8_t                             group;
    uint32_t                            src_addr;
    uint32_t                            dst_addr;
    uint32_t                            bytes;
    uint32_t/* cyhal_dma_direction_t */ direction;
    bool                                tx_started; /* Only used in ISR */
    bool                                is_enabled;
    cyhal_event_callback_data_t         callback_data;
} cyhal_dma_t;

/**
  * @brief DWM configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_dma_configurator_t;

/** @brief Ethernet Info
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases. */
typedef struct
{
    void /* etc_info_t* */              *etc;                       /**< Ethernet Common data */
    cyhal_ether_addr_t                  mac_address;                /**< Ethernet MAC address */
    uint32_t                            watchdog_events;            /**< Ethernet Events since last watchdog call */
    bool                                started;                    /**< Indicates that Ethernet has started */

} _cyhal_ethernet_info_t;

/**
  * @brief Ethernet object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    uint32_t                    tag;                        /**< If == CYHAL_ETHERNET_OBJ_TAG, indicates object is initialized */
    bool                        network_up;                 /**< Indicates if network is up
                                                             *     true  = network is up
                                                             *     false = network is down  */
    uint32_t                    events_from_GMAC_irq;       /**< IRQ from GMAC informs us of interrupts */
    void                        *user_callback;              /**< User callback function */
    void                        *user_data;                 /**< Argument to be passed back to the user while invoking the callback */
    uint32_t                    event_req;                  /**< User callback function event requests */
    uint8_t                     intr_priority;              /**< User callback function interrupt priority */
    cyhal_clock_t               clk;                        /**< Ethernet Clock */

    _cyhal_ethernet_info_t      et_info;                    /**< Platform dependent Ethernet Info */

} cyhal_ethernet_t;

/**
 * @brief I2C object
 *
 * Application code should not rely on the specific contents of this struct.
 * They are considered an implementation detail which is subject to change
 * between platforms and/or HAL releases.
 */
typedef struct {
    cyhal_resource_inst_t                  resource;
    cyhal_gpio_t                           pin_sda;
    cyhal_gpio_t                           pin_scl;
    cyhal_event_callback_data_t            callback_data;
} cyhal_i2c_t;

/**
  * @brief I2C configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_i2c_configurator_t;

/**
  * @brief LPTIMER object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    cyhal_resource_inst_t       resource;
    bool                        started;
    bool                        matching;
    uint32_t                    reset_ticks;
    bool                        compare_event_enabled;
    cyhal_event_callback_data_t callback_data;
} cyhal_lptimer_t;

/**
  * @brief M2M object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    cyhal_dma_t*          dma_obj;
} cyhal_m2m_t;

/**
  * @brief PWM object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    cyhal_resource_inst_t resource;
    cyhal_resource_inst_t compl_resource;
    cyhal_gpio_t          pin;
    cyhal_gpio_t          compl_pin;
    uint32_t              dead_time_us;
    bool                  is_enabled;
} cyhal_pwm_t;

/**
  * @brief PWM configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_pwm_configurator_t;

/**
  * @brief RTC object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    void *empty;
} cyhal_rtc_t;

/**
  * @brief RTC configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_rtc_configurator_t;

/**
 * @brief SPI object
 *
 * Application code should not rely on the specific contents of this struct.
 * They are considered an implementation detail which is subject to change
 * between platforms and/or HAL releases.
 */
typedef struct {
    cyhal_resource_inst_t                  resource;
    cyhal_gpio_t                           pin_mosi;
    cyhal_gpio_t                           pin_miso;
    cyhal_gpio_t                           pin_clk;
    cyhal_gpio_t                           pin_cs;
    uint32_t                               numDataBytes;
    cyhal_event_callback_data_t            callback_data;
} cyhal_spi_t;

/**
  * @brief SPI configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_spi_configurator_t;

/**
  * @brief Timer object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    cyhal_resource_inst_t       resource;
    bool                        running;
    bool                        ever_stopped;
    bool                        in_compare_phase;
    bool                        is_continuous;
    uint32_t                    freq_ratio;
    uint32_t                    max_fw_ticks;
    uint32_t                    fw_start_ticks;
    uint32_t                    fw_ticks_to_compare;
    uint32_t                    fw_tick_period;
    uint32_t                    hw_ticks_to_compare;
    uint32_t                    hw_ticks_to_terminal;
    uint32_t                    previous_stop_hw_ticks;
    bool                        compare_event_enabled;
    bool                        terminal_event_enabled;
    cyhal_event_callback_data_t callback_data;
} cyhal_timer_t;

/**
  * @brief Timer configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_timer_configurator_t;

/**
  * @brief UART object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    cyhal_resource_inst_t           resource;
    cyhal_gpio_t                    pin_rx;
    cyhal_gpio_t                    pin_tx;
    cyhal_gpio_t                    pin_rts;
    bool                            enable_rts;
    cyhal_gpio_t                    pin_cts;
    bool                            enable_cts;
    cyhal_clock_t                   clk;
    wiced_ring_buffer_t             buffer;
    /* Actually cyhal_uart_event_t, stored as uint32_t to avoid circular dependencies */
    uint32_t                        user_enabled_events;
    uint16_t                        user_rx_trigger_level;
    uint16_t                        user_tx_trigger_level;
    volatile const void             *async_tx_buff;
    volatile size_t                 async_tx_length;
    volatile void                   *async_rx_buff;
    volatile size_t                 async_rx_length;
    cyhal_event_callback_data_t     callback_data;
} cyhal_uart_t;

/**
  * @brief UART configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_uart_configurator_t;

/**
  * @brief Ethernet configurator struct
  *
  * 43907 does not support configurators, so this is a stub data type.
  */
typedef void cyhal_ethernet_configurator_t;

/**
  * @brief WDT object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    void *empty;
} cyhal_wdt_t;

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/** \} group_hal_impl_hw_types */
/** \} group_hal_impl */
