/***************************************************************************//**
* \file cyhal_hw_resources.h
*
* \brief
* Provides basic resource type definitions used by the HAL.
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
* \addtogroup group_hal_impl_hw_types
* \ingroup group_hal_impl
* \{
*/

#include <stdbool.h>
#include <stdint.h>
#include "platform_mcu_peripheral.h"

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Documented in cyhal.h
#define CYHAL_API_VERSION                   (2)

#define CYHAL_DRIVER_AVAILABLE_ADC          (0)
#define CYHAL_DRIVER_AVAILABLE_CLOCK        (1)
#define CYHAL_DRIVER_AVAILABLE_COMP         (0)
#define CYHAL_DRIVER_AVAILABLE_CRC          (0)
#define CYHAL_DRIVER_AVAILABLE_DAC          (0)
#define CYHAL_DRIVER_AVAILABLE_DMA          (1)
#define CYHAL_DRIVER_AVAILABLE_EZI2C        (0)
#define CYHAL_DRIVER_AVAILABLE_FLASH        (0)
#define CYHAL_DRIVER_AVAILABLE_GPIO         (1)
#define CYHAL_DRIVER_AVAILABLE_HWMGR        (1)
#define CYHAL_DRIVER_AVAILABLE_I2C          (1)
#define CYHAL_DRIVER_AVAILABLE_I2S          (0)
#define CYHAL_DRIVER_AVAILABLE_I2S_TX       (0)
#define CYHAL_DRIVER_AVAILABLE_I2S_RX       (0)
#define CYHAL_DRIVER_AVAILABLE_INTERCONNECT (0)
#define CYHAL_DRIVER_AVAILABLE_KEYSCAN      (0)
#define CYHAL_DRIVER_AVAILABLE_LPTIMER      (1)
#define CYHAL_DRIVER_AVAILABLE_OPAMP        (0)
#define CYHAL_DRIVER_AVAILABLE_PDMPCM       (0)
#define CYHAL_DRIVER_AVAILABLE_PWM          (1)
#define CYHAL_DRIVER_AVAILABLE_QSPI         (0)
#define CYHAL_DRIVER_AVAILABLE_QUADDEC      (0)
#define CYHAL_DRIVER_AVAILABLE_RTC          (0)
#define CYHAL_DRIVER_AVAILABLE_SDHC         (0)
#define CYHAL_DRIVER_AVAILABLE_SDIO         (0)
#define CYHAL_DRIVER_AVAILABLE_SPI          (0)
#define CYHAL_DRIVER_AVAILABLE_SYSPM        (1)
#define CYHAL_DRIVER_AVAILABLE_SYSTEM       (1)
#define CYHAL_DRIVER_AVAILABLE_TDM          (0)
#define CYHAL_DRIVER_AVAILABLE_TDM_TX       (0)
#define CYHAL_DRIVER_AVAILABLE_TDM_RX       (0)
#define CYHAL_DRIVER_AVAILABLE_TIMER        (1)
#define CYHAL_DRIVER_AVAILABLE_TRNG         (0)
#define CYHAL_DRIVER_AVAILABLE_UART         (1)
#define CYHAL_DRIVER_AVAILABLE_USB_DEV      (0)
#define CYHAL_DRIVER_AVAILABLE_WDT          (1)

/* NOTE: Any changes made to this enum must also be made to the hardware manager resource tracking */
/** Resource types that the hardware manager supports */
typedef enum
{
    CYHAL_RSC_CLOCK,    /*!< Clock */
    CYHAL_RSC_CRYPTO,   /*!< Crypto */
    CYHAL_RSC_DMA,      /*!< DMA */
    CYHAL_RSC_GPIO,     /*!< General purpose I/O pin */
    CYHAL_RSC_I2C,      /*!< I2C block */
    CYHAL_RSC_I2S,      /*!< I2S block */
    CYHAL_RSC_LPTIMER,  /*!< Low power timer */
    CYHAL_RSC_PWM,      /*!< PWM block */
    CYHAL_RSC_QSPI,     /*!< QSPI block */
    CYHAL_RSC_RTC,      /*!< RTC block */
    CYHAL_RSC_SDIO,     /*!< SDIO block */
    CYHAL_RSC_SPI,      /*!< SPI block */
    CYHAL_RSC_TIMER,    /*!< Timer */
    CYHAL_RSC_UART,     /*!< UART block */
    CYHAL_RSC_USB,      /*!< USB block */
    CYHAL_RSC_INVALID,  /*!< Placeholder for invalid type */
} cyhal_resource_t;

/* NOTE: Any changes here must also be made in cyhal_hwmgr.c */
/** Enum for the different types of clocks that exist on the device. */
typedef enum
{
    /**********************************************
     * Simplified illustration of the clock tree
     * ---------------------------------------------
     * 1      2           3       4           5
     * ---------------------------------------------
     * XTAL   ALP
     *        BB_PLL      HT      BACKPLANE   CPU
     *        AUDIO_PLL
     *        USB_PLL
     *        HSIC_PLL
     *        WLAN_PLL
     * LPO    ILP
    */
    CYHAL_CLOCK_BLOCK_ALP,       // source - XTAL
    CYHAL_CLOCK_BLOCK_HT,        // source - BB_PLL
    CYHAL_CLOCK_BLOCK_ILP,       // source - LPO / ALP. Note: ALP option is not available in WICED
    CYHAL_CLOCK_BLOCK_LPO,       // source - N/A
    CYHAL_CLOCK_BLOCK_XTAL,      // source - N/A
    CYHAL_CLOCK_BLOCK_BB_PLL,    // source - XTAL
    CYHAL_CLOCK_BLOCK_AUDIO_PLL, // source - XTAL
    CYHAL_CLOCK_BLOCK_USB_PLL,   // source - XTAL. Verify if derived from BB_PLL
    CYHAL_CLOCK_BLOCK_HSIC_PLL,  // source - XTAL. Verify if derived from BB_PLL
    CYHAL_CLOCK_BLOCK_WLAN_PLL,  // source - XTAL. Verify if derived from BB_PLL
    CYHAL_CLOCK_BLOCK_CPU,       // source - ARM (BB_PLL path) / backplane (BB_PLL path)
    CYHAL_CLOCK_BLOCK_BACKPLANE, // source - HT / ALP. Note: ALP option is not available in WICED
    CYHAL_CLOCK_BLOCK_FAST_UART  // source - Dedicated 160MHz connection from BB_PLL
} cyhal_clock_block_t;

/** @brief Clock object
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases. */
typedef struct
{
    cyhal_clock_block_t     block;
    uint8_t                 channel;
    bool                    reserved;
} cyhal_clock_t;

/**
  * @brief Represents a particular instance of a resource on the chip.
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct
{
    cyhal_resource_t type;      //!< The resource block type
    uint8_t          block_num; //!< The resource block index
    /**
      * The channel number, if the resource type defines multiple channels
      * per block instance. Otherwise, 0 */
    uint8_t          channel_num;
} cyhal_resource_inst_t;

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/** \} group_hal_impl_hw_types */
