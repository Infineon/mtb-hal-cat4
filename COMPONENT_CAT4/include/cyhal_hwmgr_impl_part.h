/***************************************************************************//**
* \file cyhal_hwmgr_impl_part.h
*
* \brief
* Provides device specific information to the hardware manager. This file must
* only ever be included by cyhal_hwmgr.c.
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

/*******************************************************************************
*       Defines
*******************************************************************************/

#define CY_BLOCK_COUNT_CLOCK    (13)
#define CY_BLOCK_COUNT_CRYPTO   (1)
#define CY_BLOCK_COUNT_DMA      (8)
#define CY_BLOCK_COUNT_GPIO     (60)
#define CY_BLOCK_COUNT_I2C      (2)
#define CY_BLOCK_COUNT_I2S      (2)
#define CY_BLOCK_COUNT_LPTIMER  (1)
#define CY_BLOCK_COUNT_PWM      (6)
#define CY_BLOCK_COUNT_QSPI     (1)
#define CY_BLOCK_COUNT_RTC      (1)
#define CY_BLOCK_COUNT_SDIO     (1)
#define CY_BLOCK_COUNT_SPI      (2)
#define CY_BLOCK_COUNT_TIMER    (1)
#define CY_BLOCK_COUNT_UART     (3)
#define CY_BLOCK_COUNT_USB      (1)

/*
    All resources have an offset and a size, offsets are stored in an array
    Subsequent resource offset equals the preceding offset + size
    Offsets are bit indexes in the arrays that track used, configured etc.

    Channel based resources have an extra array for block offsets

    Note these are bit offsets into arrays that are internal to the HW mgr.
*/
#define CY_OFFSET_CLOCK    0
#define CY_SIZE_CLOCK      CY_BLOCK_COUNT_CLOCK
#define CY_OFFSET_CRYPTO   (CY_OFFSET_CLOCK + CY_SIZE_CLOCK)
#define CY_SIZE_CRYPTO     CY_BLOCK_COUNT_CRYPTO
#define CY_OFFSET_DMA      (CY_OFFSET_CRYPTO + CY_SIZE_CRYPTO)
#define CY_SIZE_DMA        CY_BLOCK_COUNT_DMA
#define CY_OFFSET_GPIO     (CY_OFFSET_DMA + CY_SIZE_DMA)
#define CY_SIZE_GPIO       CY_BLOCK_COUNT_GPIO
#define CY_OFFSET_I2C      (CY_OFFSET_GPIO + CY_SIZE_GPIO)
#define CY_SIZE_I2C        CY_BLOCK_COUNT_I2C
#define CY_OFFSET_I2S      (CY_OFFSET_I2C + CY_SIZE_I2C)
#define CY_SIZE_I2S        CY_BLOCK_COUNT_I2S
#define CY_OFFSET_LPTIMER  (CY_OFFSET_I2S + CY_SIZE_I2S)
#define CY_SIZE_LPTIMER    CY_BLOCK_COUNT_LPTIMER
#define CY_OFFSET_PWM      (CY_OFFSET_LPTIMER + CY_SIZE_LPTIMER)
#define CY_SIZE_PWM        CY_BLOCK_COUNT_PWM
#define CY_OFFSET_QSPI     (CY_OFFSET_PWM + CY_SIZE_PWM)
#define CY_SIZE_QSPI       CY_BLOCK_COUNT_QSPI
#define CY_OFFSET_RTC      (CY_OFFSET_QSPI + CY_SIZE_QSPI)
#define CY_SIZE_RTC        CY_BLOCK_COUNT_RTC
#define CY_OFFSET_SDIO     (CY_OFFSET_RTC + CY_SIZE_RTC)
#define CY_SIZE_SDIO       CY_BLOCK_COUNT_SDIO
#define CY_OFFSET_SPI      (CY_OFFSET_SDIO + CY_SIZE_SDIO)
#define CY_SIZE_SPI        CY_BLOCK_COUNT_SPI
#define CY_OFFSET_TIMER    (CY_OFFSET_SPI + CY_SIZE_SPI)
#define CY_SIZE_TIMER      CY_BLOCK_COUNT_TIMER
#define CY_OFFSET_UART     (CY_OFFSET_TIMER + CY_SIZE_TIMER)
#define CY_SIZE_UART       CY_BLOCK_COUNT_UART
#define CY_OFFSET_USB      (CY_OFFSET_UART + CY_SIZE_UART)
#define CY_SIZE_USB        CY_BLOCK_COUNT_USB

#define CY_TOTAL_ALLOCATABLE_ITEMS     (CY_OFFSET_USB + CY_SIZE_USB)

/*******************************************************************************
*       Variables
*******************************************************************************/

/* The order of items here must match the order in cyhal_clock_impl.h
 *
 * Each entry in the array below is the prior entry plus the number of clocks that exist
 * of the prior type. When there is only 1 clock (e.g: ALP/HT) the next number is simply
 * one higher than the previous value. When there are multiple clocks (e.g.: PLL)
 * the subsequent value is increased by the define that specifies how many clocks are
 * actually present. */
static const uint8_t cyhal_block_offsets_clock[13] =
{
    0, // ALP,
    1, // HT,
    2, // ILP,
    3, // LPO,
    4, // XTAL,
    5, // BB_PLL,
    6, // AUDIO_PLL,
    7, // USB_PLL,
    8, // HSIC_PLL,
    9, // WLAN_PLL,
    10, // CPU,
    11, // BACKPLANE,
    12, // FAST_UART
};

static uint8_t cyhal_used[(CY_TOTAL_ALLOCATABLE_ITEMS + 7) / 8] = {0};

// Note: the ordering here needs to be parallel to that of cyhal_resource_t
static const uint8_t cyhal_resource_offsets[] =
{
    CY_OFFSET_CLOCK,
    CY_OFFSET_CRYPTO,
    CY_OFFSET_DMA,
    CY_OFFSET_GPIO,
    CY_OFFSET_I2C,
    CY_OFFSET_I2S,
    CY_OFFSET_LPTIMER,
    CY_OFFSET_PWM,
    CY_OFFSET_QSPI,
    CY_OFFSET_RTC,
    CY_OFFSET_SDIO,
    CY_OFFSET_SPI,
    CY_OFFSET_TIMER,
    CY_OFFSET_UART,
    CY_OFFSET_USB,
};

#define _CYHAL_RESOURCES (sizeof(cyhal_resource_offsets)/sizeof(cyhal_resource_offsets[0]))

/*******************************************************************************
*       Utility helper functions
*******************************************************************************/

static inline uint16_t _cyhal_uses_channels(cyhal_resource_t type)
{
    return (type == CYHAL_RSC_CLOCK);
}

static inline uint16_t _cyhal_get_resource_offset(cyhal_resource_t type)
{
    return cyhal_resource_offsets[type];
}

static inline const uint8_t* _cyhal_get_block_offsets(cyhal_resource_t type)
{
    CY_ASSERT(CYHAL_RSC_CLOCK == type);
    return cyhal_block_offsets_clock;
}

// Gets the number of block offset entries, only valid for blocks which have channels.
static inline uint8_t _cyhal_get_block_offset_length(cyhal_resource_t type)
{
    CY_ASSERT(CYHAL_RSC_CLOCK == type);
    return sizeof(cyhal_block_offsets_clock)/sizeof(cyhal_block_offsets_clock[0]);
}
