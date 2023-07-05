/*
 * Copyright 2023, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 */

#pragma once


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "cy_result.h"
#include "cyhal_ethernet.h"
#include "cyhal_ethernet_etc.h"

/***************************************************************
 *
 *  Defines
 *
 **************************************************************/

#define NUM_BYTES_IN_MAC_ADDR       6

/** Ethernet MAC filter configuration */
typedef struct
{
  /** Ethernet MAC filter for source/destination MAC adress */
    cyhal_ethernet_filter_type_t filter_type;
  /** Store 6byte Ethernet MAC address */
  uint8_t mac_addr[NUM_BYTES_IN_MAC_ADDR];
  /** Ignore number of bytes for in the received packet before comparing (filtering) */
  /** ignore_bytes = 0x01 implies first byte received should not be compared. */
  uint8_t ignore_bytes;
  /** If 1, valid entry here */
  uint8_t in_use;

} cyhal_gmac_filter_info_t;

/***************************************************************
 *
 *  Functions
 *
 **************************************************************/

/** Add Multicast address to the GMAC filter list index
 *
 * @param[in]   index   index to store in filter list ( 0 <= index < _CYHAL_ETHERNET_NUM_FILTERS)
 * @param[in]   config  pointer to configurations structure
 *
 * @result  CY_RSLT_SUCCESS
 *          CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG
 *          CYHAL_ETHERNET_RSLT_FILTER_ALREADY_ADDED
 *          CYHAL_ETHERNET_RSLT_FILTER_NO_SPACE
 */
cy_rslt_t _cyhal_gmac_broadcast_filter_add(uint16_t index, const cyhal_ethernet_filter_config_t *config);

/** Remove Multicast address from the GMAC filter list index
 *
 * @param[in]   index   index to store in filter list ( 0 <= index < _CYHAL_ETHERNET_NUM_FILTERS)
 *
 * @result  CY_RSLT_SUCCESS
 *          CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG     If index already empty
 */
cy_rslt_t _cyhal_gmac_broadcast_filter_remove(uint16_t index);


/** Check if Multicast address is in the GMAC filter list
 *
 * @param[in]   macaddr mac addr to search for in GMAC list
 * @param[out]  config  handle to store pointer to the filter info
 *
 * @result  CY_RSLT_SUCCESS
 *          CYHAL_ETHERNET_RSLT_ERR_FAILED
 */
cy_rslt_t _cyhal_gmac_broadcast_filter_lookup(cyhal_ether_addr_t *macaddr, cyhal_gmac_filter_info_t *config);


/** Check if Multicast address is in the GMAC filter list
 *
 * @param[in]   index   index to store in filter list ( 0 <= index < _CYHAL_ETHERNET_NUM_FILTERS)
 * @param[out]  config  handle to store pointer to the filter info
 *
 * @result  CY_RSLT_SUCCESS
 *          CYHAL_ETHERNET_RSLT_ERR_FAILED
 */
cy_rslt_t _cyhal_gmac_broadcast_filter_lookup_by_index(uint16_t index, cyhal_gmac_filter_info_t *config);


/** Check if Multicast address passes the GMAC filter list
 *
 * The GMAC filter list is a white list for blocking incoming packets.
 *
 * @param[in]   dst_macaddr     destination mac addr to check
 * @param[in]   src_macaddr     source mac addr to check
 *
 * @result  CY_RSLT_SUCCESS                 : Address is allowed to be passed to Application
 *          CYHAL_ETHERNET_RSLT_ERR_FAILED  : Address is not on GMAC list, block it.
 */
cy_rslt_t _cyhal_gmac_broadcast_filter_pass(cyhal_ether_addr_t *dst_mcaddr, cyhal_ether_addr_t *src_mcaddr);

