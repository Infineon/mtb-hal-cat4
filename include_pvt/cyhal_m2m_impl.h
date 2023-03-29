/***************************************************************************//**
* \file cyhal_m2m_impl.h
*
* \brief
* Implementation (common) details of Infineon M2M DMA.
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

#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/** 
 * \addtogroup group_hal_impl_m2m M2M DMA (Memory-to-Memory Direct Memory Access)
 * \ingroup group_hal_impl
 * \{
 * M2M DMA allows transferring data between memory regions without CPU intervention.
 * It can be used to transfer WIFI data between the Apps processor and the WLAN processor
 * through the WWD driver.
 *
 */

/* WWD M2M DMA channels */
#define ACPU_DMA_TX_CHANNEL                 (1) //!< Application CPU DMA TX channel
#define ACPU_DMA_RX_CHANNEL                 (0) //!< Application CPU DMA RX channel
#define WCPU_DMA_TX_CHANNEL                 (0) //!< WLAN CPU DMA TX channel
#define WCPU_DMA_RX_CHANNEL                 (1) //!< WLAN CPU DMA RX channel
#define MEMCPY_M2M_DMA_CHANNEL              (2) //!< MEMCPY M2M DMA channel

/* User group M2M DMA channels */
#define MEMCPY_USR_DMA_CHANNEL              (3) //!< User M2M DMA channel
#define USR1_DMA_TX_CHANNEL                 (4) //!< User DMA TX channel 1
#define USR1_DMA_RX_CHANNEL                 (5) //!< User DMA RX channel 1
#define USR2_DMA_TX_CHANNEL                 (6) //!< User DMA TX channel 2
#define USR2_DMA_RX_CHANNEL                 (7) //!< User DMA RX channel 2

#define _CYHAL_M2M_GRPS                     (3) //!< Number of available channel groups

#define SDPCMD_RXOFFSET                     (8) //!< RX offset

/** M2M DMA group */
typedef enum
{
    _CYHAL_M2M_GRP_WWD,     //!< Group used for WWD
    _CYHAL_M2M_GRP_USR_1,   //!< User group 1
    _CYHAL_M2M_GRP_USR_2    //!< User group 2
} _cyhal_m2m_group_t;

/** Common M2M initialization routine.
 *
 * Function for initializing a DMA group consisting of TX, RX, and DMA channels.
 * @param[in] group  Group number
 * @param[in] tx_ch  TX channel number
 * @param[in] rx_ch  RX channel number
 * @param[in] rx_buffer_size  RX buffer size
 */
void _cyhal_m2m_init_dma(_cyhal_m2m_group_t group, uint8_t tx_ch, uint8_t rx_ch, uint32_t rx_buffer_size);

/** Common M2M disable DMA routine.
 *
 * Function for disabling a DMA channel
 * @param[in] dma_ch  DMA channel number
 */
void _cyhal_m2m_disable_dma(uint8_t dma_ch);

/** Common M2M deinitialization routine.
 *
 * Function for deinitializing a DMA group.
 * @param[in] group  Group number
 */
void _cyhal_m2m_deinit_dma(_cyhal_m2m_group_t group);

/** Common M2M routine to check if communication is idle
 *
 * @param[in] group  Group number
 * @return Idle (true) or Busy (false)
 */
bool _cyhal_m2m_tx_is_idle(_cyhal_m2m_group_t group);

/** Common M2M routine to get the channels for a given group
 *
 * @param[in] group  Group number
 * @param[in] tx_ch  TX channel number
 * @param[in] rx_ch  RX channel number
 * @param[in] dma_ch DMA channel number
 */
void _cyhal_m2m_get_channels(_cyhal_m2m_group_t group, uint8_t *tx_ch, uint8_t *rx_ch, uint8_t *dma_ch);

/** Common M2M function for initiating a regular DMA transfer
 *
 * @param[in] dma_ch DMA channel number
 * @param[in] destination Destination address
 * @param[in] source Source address
 * @param[in] byte_count Number of bytes to transfer
 */
void _cyhal_m2m_unprotected_dma_memcpy(uint8_t dma_ch, void* destination, const void* source, uint32_t byte_count);

/** \} group_hal_impl_m2m */

#if defined(__cplusplus)
}
#endif
