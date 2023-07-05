/***************************************************************************//**
* file cyhal_ethernet_internal.h
*
* brief
* Provides an interface for Internal Ethernet functions useful for testing.
*
********************************************************************************
* copyright
* Copyright 2023 Cypress Semiconductor Corporation (an Infineon company) or
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

/** \cond DOXYGEN_HIDE */

#include <stdint.h>
#include <stdbool.h>
#include "cy_result.h"
#include "cyhal_general_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** Ethernet Component loopback modes */
typedef enum
{
    _CYHAL_ETHERNET_LOOPBACK_MODE_NONE = 0,  /**< no Loopback */
    _CYHAL_ETHERNET_LOOPBACK_MODE_ENABLE,    /**< enable loopback */

    _CYHAL_NUM_LOOPBACK_MODES                /**< Not a valid loopback mode */
} _cyhal_ethernet_loopback_mode_t;



/** Ethernet status */
typedef struct {
    bool                            network_up;                 /**< Indicates if network is up
                                                                 *     true  = network is up
                                                                 *     false = network is down  */
    bool                            promiscuous_enable;         /**< Promiscuous Mode
                                                                 *     true  = enable accept all packets
                                                                 *     false = only accept Unicast packets to us */
    bool                            broadcast_enable;           /**< Broadcast Receive Enable
                                                                 *     true  = enable accept broadcast packets
                                                                 *     false = do not accept broadcast packets */
    _cyhal_ethernet_loopback_mode_t loopback_mode;              /**< loopback mode  */
    cyhal_ethernet_event_t          callback_events;            /**< Callback Events enabled \ref cyhal_ethernet_event_t */
} _cyhal_ethernet_status_t;

/** Get MAC address
 *
 * @param[in]   obj             Pointer to an Ethernet object
 * @param[out]  mac_addr        Pointer to store mac_address
 *
 * @return CY_RSLT_SUCCESS or error
 */
cy_rslt_t _cyhal_ethernet_get_mac_addr(cyhal_ethernet_t *obj, cyhal_ether_addr_t *mac_addr);


/** Set Ethernet Loopback mode
 *
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] loopback_mode     One of \ref _cyhal_ethernet_loopback_mode_t
 *
 * @return The status of the loopback mode request.
 */
cy_rslt_t _cyhal_ethernet_set_loopback_mode(cyhal_ethernet_t *obj, _cyhal_ethernet_loopback_mode_t loopback_mode);

/** Get Ethernet status
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] status            Pointer to \ref cyhal_ethernet_status_t to be filled
 *
 * @return The status of the get status request.
 */
cy_rslt_t _cyhal_ethernet_get_status(cyhal_ethernet_t *obj, _cyhal_ethernet_status_t *status);


#if defined(__cplusplus)
}
#endif


/** \endcond */

