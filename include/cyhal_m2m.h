/***************************************************************************//**
* \file cyhal_m2m.h
*
* \brief
* Provides a high level interface for interacting with the Infineon M2M DMA.
* This interface abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
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
 * \addtogroup group_hal_m2m M2M DMA (Memory-to-Memory Direct Memory Access)
 * \ingroup group_hal
 * \{
 * High level interface for interacting with the memory-to-memory direct memory access (DMA).
 * This driver allows transferring data to/from the radio RAM from/to the main application RAM.
 * For general-purpose DMA transfers, use the DMA driver.
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "cy_result.h"
#include "cyhal_hw_types.h"

#if defined(__cplusplus)
extern "C" {
#endif


/** Enum of M2M RX channel activity status. */
typedef enum
{
    CYHAL_M2M_RX_DISABLED            = 0x0, //!< Channel is disabled
    CYHAL_M2M_RX_ACTIVE              = 0x1, /**< Channel is either waiting for data to transfer
                                                 or is transferring data */
    CYHAL_M2M_RX_IDLE_WAIT           = 0x2, //!< Channel is waiting for a descriptor to be posted or is suspended
    CYHAL_M2M_RX_STOPPED             = 0x3, //!< Channel is stopped because an error was detected
} cyhal_m2m_rx_status_t;

/** Flags enum of M2M events. Multiple events can be enabled via \ref cyhal_m2m_enable_event and
 * the callback from \ref cyhal_m2m_register_callback will be run to notify. */
typedef enum
{
    CYHAL_M2M_NO_INTR                  = 0,        //!< No interrupt
    CYHAL_M2M_DESCRIPTOR_READ_ERROR    = (1 << 0), //!< Descriptor read error
    CYHAL_M2M_DATA_TRANSFER_ERROR      = (1 << 1), //!< Errors while transferring data to or from memory
    CYHAL_M2M_DESCRIPTOR_PROGRAM_ERROR = (1 << 2), //!< descriptor programming errors
    CYHAL_M2M_NO_DESCRIPTOR_ERROR      = (1 << 3), /**< Channel cannot process an incoming frame because
                                                         no descriptors are available */
    CYHAL_M2M_RX_CHANNEL_INTERRUPT     = (1 << 4), //!< Interrupt from the RX channel
    CYHAL_M2M_TX_CHANNEL_INTERRUPT     = (1 << 5), //!< Interrupt from the TX channel
} cyhal_m2m_event_t;

/** Event handler for M2M interrupts */
typedef void (*cyhal_m2m_event_callback_t)(void *callback_arg, cyhal_m2m_event_t event);


/** Initialize the M2M peripheral
 *
 * @param[out] obj  Pointer to an M2M object. The caller must allocate the memory for this
 * object but the init function will initialize its contents.
 * @param[in]  rx_buffer_size Size of the RX buffer.
 * @return The status of the init request
 */
cy_rslt_t cyhal_m2m_init(cyhal_m2m_t *obj, uint32_t rx_buffer_size);

/** Free the M2M object. Freeing a M2M object while a transfer is in progress
 * (\ref cyhal_m2m_is_busy) is invalid.
 *
 * @param[in,out] obj The M2M object
 */
void cyhal_m2m_free(cyhal_m2m_t *obj);

/** Checks if the transfer has been triggered, but not yet complete (eg: is pending, blocked or running)
 *
 * @param[in] obj    The M2M object
 * @return True if M2M channel is busy
 */
bool cyhal_m2m_is_busy(cyhal_m2m_t *obj);

/** Register an M2M callback handler
 *
 * This function will be called when one of the events enabled by \ref cyhal_m2m_enable_event occurs.
 *
 * @param[in] obj          The M2M object
 * @param[in] callback     The callback handler which will be invoked when an event triggers
 * @param[in] callback_arg Generic argument that will be provided to the callback when called
 */
void cyhal_m2m_register_callback(cyhal_m2m_t *obj, cyhal_m2m_event_callback_t callback, void *callback_arg);

/** Configure M2M event enablement
 *
 * When an enabled event occurs, the function specified by \ref cyhal_m2m_register_callback will be called.
 *
 * @param[in] obj            The M2M object
 * @param[in] event          The M2M event type
 * @param[in] intr_priority  The priority for NVIC interrupt events. The priority from the most
 * recent call will take precedence, i.e all events will have the same priority.
 * @param[in] enable         True to turn on interrupts, False to turn off
 */
void cyhal_m2m_enable_event(cyhal_m2m_t *obj, cyhal_m2m_event_t event, uint8_t intr_priority, bool enable);

/** Send M2M TX data packet
 *
 * This function sends a TX data packet. WHD is expected to set the packet size and send
 * the "data" with whd_buffer_t type. The buffer type is dependent on the network stack.
 * For example for LWIP, it is structured as follows:
 *  p->payload = payload; //packet data
 *  p->len = len;         //packet size
 *
 * @param[in] obj  The M2M object
 * @param[in] buffer Data buffer to use in the packet transaction
 * @return The status of the tx operation
 */
cy_rslt_t cyhal_m2m_tx_send(cyhal_m2m_t *obj, void* buffer);

/** Release next completed M2M TX data packet
 *
 * This function is used to free the TX data packet if TX error occurs.
 *
 * @param[in] obj  The M2M object
 */
void cyhal_m2m_tx_release(cyhal_m2m_t *obj);

/** Receive and read M2M RX data packet
 *
 * This function reads the next available RX packet.
 *
 * @param[in] obj  The M2M object
 * @param[out] rxd_handle RX packet handle
 * @param[out] hwtag  Hardware tag used to read the frame header
 */
void cyhal_m2m_rx_receive(cyhal_m2m_t *obj, void** rxd_handle, uint16_t** hwtag);

/** Prepare M2M RX ring buffer
 *
 * Prepares the internal M2M RX buffer (allocated during initialization)
 * for receiving RX data packets.
 *
 * @param[in] obj  The M2M object
 * @return Success (true) or Failed (false)
 */
bool cyhal_m2m_rx_prepare(cyhal_m2m_t *obj);

/** M2M RX activity status
 *
 * @param[in] obj  The M2M object
 * @return RX activity status
 */
cyhal_m2m_rx_status_t cyhal_m2m_rx_status(cyhal_m2m_t *obj);

/** Read M2M TX and RX interrupt status
 *
 * Reads the interrupt status for TX and RX channels.
 *
 * @param[in] obj  The M2M object
 * @param[out] signal_txdone  TX done state as signaled by the RX interrupt
 * @return Interrupt event status
 */
cyhal_m2m_event_t cyhal_m2m_intr_status(cyhal_m2m_t *obj, bool* signal_txdone);

#if defined(__cplusplus)
}
#endif

/** \} group_hal_m2m */
