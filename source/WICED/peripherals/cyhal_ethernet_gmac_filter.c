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
 * This file implements the chip-specific routines for the GMAC core.
 */


#include <stdio.h>
#include <typedefs.h>
#include <osl.h>
#include "bcmdefs.h"
#include "bcmendian.h"
#include "bcmutils.h"
#include "cyhal_ethernet_devices.h"
#include "cyhal_ethernet_phy.h"
#include "cyhal_ethernet.h"
#include "cyhal_system.h"
#include "cyhal_ethernet_gmac_siutils.h"
#include "sbhnddma.h"
#include "sbchipc.h"
#include "hndsoc.h"
#include "cyhal_ethernet_gmac_mib.h"
#include "cyhal_ethernet_gmac_common.h"
#include "cyhal_ethernet_gmac_core.h"
#include "cyhal_ethernet_gmac_dma_funcs.h"

#include "cyhal_ethernet_etc.h"

static cyhal_gmac_filter_info_t _cyhal_gmac_saved_filter_list[CYHAL_ETHERNET_NUM_FILTERS];
static uint16_t                 _cyhal_gmac_num_saved_filters;

cy_rslt_t _cyhal_gmac_broadcast_filter_add(uint16_t add_to_index, const cyhal_ethernet_filter_config_t *config)
{
    uint16_t i;
    cyhal_gmac_filter_info_t *curr_filter;

    if (add_to_index >= CYHAL_ETHERNET_NUM_FILTERS)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    curr_filter = _cyhal_gmac_saved_filter_list;
    for (i=0; i< CYHAL_ETHERNET_NUM_FILTERS; i++)
    {
        if ( (curr_filter->in_use) &&
             (curr_filter->filter_type == config->filter_type) &&
             (curr_filter->ignore_bytes == config->ignore_bytes) &&
             CYHAL_ETHER_ADDR_CMP(&curr_filter->mac_addr, &config->mac_addr) == 0)
        {
            /* Already in our list */
            return CYHAL_ETHERNET_RSLT_FILTER_ALREADY_ADDED;
        }
        curr_filter++;
    }

    /* empty the entry if needed */
    if (_cyhal_gmac_saved_filter_list[add_to_index].in_use)
    {
        if (_cyhal_gmac_broadcast_filter_remove(add_to_index) != CY_RSLT_SUCCESS)
        {
            return CYHAL_ETHERNET_RSLT_ERR_FAILED;
        }
    }

    /* Is the index entry empty? */
    curr_filter = &_cyhal_gmac_saved_filter_list[add_to_index];
    if (curr_filter->in_use == 0)
    {
        curr_filter->filter_type = config->filter_type;
        curr_filter->ignore_bytes = config->ignore_bytes;
        CYHAL_ETHER_ADDR_CPY(&config->mac_addr, &curr_filter->mac_addr);
        curr_filter->in_use = 1;
        _cyhal_gmac_num_saved_filters++;
        return CY_RSLT_SUCCESS;
    }

    return CYHAL_ETHERNET_RSLT_FILTER_NO_SPACE;
}

/* returns  0 success
 *         -1 if not found
 */
cy_rslt_t _cyhal_gmac_broadcast_filter_remove(uint16_t remove_index)
{
    cyhal_gmac_filter_info_t *curr_filter;
    if (remove_index >= CYHAL_ETHERNET_NUM_FILTERS)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    /* is the index entry occupied? */
    curr_filter = &_cyhal_gmac_saved_filter_list[remove_index];
    if ( curr_filter->in_use )
    {
        memset(curr_filter, 0x00, sizeof(cyhal_gmac_filter_info_t));
        CY_ASSERT(_cyhal_gmac_num_saved_filters > 0);
        _cyhal_gmac_num_saved_filters--;
    }
    else
    {
        /* Already empty */
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }


    return CY_RSLT_SUCCESS;
}

cy_rslt_t _cyhal_gmac_broadcast_filter_lookup(cyhal_ether_addr_t *macaddr, cyhal_gmac_filter_info_t *out_config)
{
    int index;
    cyhal_gmac_filter_info_t *curr_filter = _cyhal_gmac_saved_filter_list;

    if (_cyhal_gmac_num_saved_filters > 0)
    {
        for (index=0; index< CYHAL_ETHERNET_NUM_FILTERS; index++)
        {
            if ( memcmp(&curr_filter->mac_addr, &macaddr->octet, sizeof(macaddr->octet)) == 0 )
            {
                /* Already in our list */
                memcpy(out_config, curr_filter, sizeof(cyhal_gmac_filter_info_t) );
                return CY_RSLT_SUCCESS;
            }
            curr_filter++;
        }
    }
    return CYHAL_ETHERNET_RSLT_ERR_FAILED;
}

cy_rslt_t _cyhal_gmac_broadcast_filter_lookup_by_index(uint16_t index, cyhal_gmac_filter_info_t *config)
{
    if ( (index >= CYHAL_ETHERNET_NUM_FILTERS) || (config == NULL) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    if ( (_cyhal_gmac_num_saved_filters > 0) && (_cyhal_gmac_saved_filter_list[index].in_use) )
    {
        memcpy(config, &_cyhal_gmac_saved_filter_list[index], sizeof(cyhal_gmac_filter_info_t));
        return CY_RSLT_SUCCESS;
    }
    return CYHAL_ETHERNET_RSLT_ERR_FAILED;
}

cy_rslt_t _cyhal_gmac_broadcast_filter_pass(cyhal_ether_addr_t *dst_mcaddr, cyhal_ether_addr_t *src_mcaddr)
{
    int index;
    cyhal_gmac_filter_info_t *curr_filter;
    uint8_t *filter_addr;                   /* points to the 6 octets of the filter */
    uint8_t *check_addr;                    /* points to the 6 octets of the addr to test */

    if (_cyhal_gmac_num_saved_filters > 0)
    {
        curr_filter = _cyhal_gmac_saved_filter_list;
        for (index=0; index < CYHAL_ETHERNET_NUM_FILTERS; index++)
        {
            /* always check against the filter */
            filter_addr = (uint8_t *)&curr_filter->mac_addr[0];

            /* check for direction */
            if (curr_filter->filter_type == CYHAL_ETHERNET_FILTER_TYPE_DESTINATION)
            {
                check_addr = (uint8_t *)&dst_mcaddr->octet[0];
            }
            else
            {
                check_addr = (uint8_t *)&src_mcaddr[0];
            }

            uint8_t xor_test = 0;
            switch (curr_filter->ignore_bytes)
            {
            default:
                xor_test = 1;
                break;
            case 0:
                xor_test |= (filter_addr[0] ^ check_addr[0]);
                xor_test |= (filter_addr[1] ^ check_addr[1]);
                xor_test |= (filter_addr[2] ^ check_addr[2]);
                xor_test |= (filter_addr[3] ^ check_addr[3]);
                xor_test |= (filter_addr[4] ^ check_addr[4]);
                xor_test |= (filter_addr[5] ^ check_addr[5]);
                break;
            case 1:
                xor_test |= (filter_addr[1] ^ check_addr[1]);
                xor_test |= (filter_addr[2] ^ check_addr[2]);
                xor_test |= (filter_addr[3] ^ check_addr[3]);
                xor_test |= (filter_addr[4] ^ check_addr[4]);
                xor_test |= (filter_addr[5] ^ check_addr[5]);
                break;
            case 2:
                xor_test |= (filter_addr[2] ^ check_addr[2]);
                xor_test |= (filter_addr[3] ^ check_addr[3]);
                xor_test |= (filter_addr[4] ^ check_addr[4]);
                xor_test |= (filter_addr[5] ^ check_addr[5]);
                break;
            case 3:
                xor_test |= (filter_addr[3] ^ check_addr[3]);
                xor_test |= (filter_addr[4] ^ check_addr[4]);
                xor_test |= (filter_addr[5] ^ check_addr[5]);
                break;
            case 4:
                xor_test |= (filter_addr[4] ^ check_addr[4]);
                xor_test |= (filter_addr[5] ^ check_addr[5]);
                break;
            case 5:
                xor_test |= (filter_addr[5] ^ check_addr[5]);
                break;
            }

            if (xor_test == 0)
            {
                /* the mac addresses from ignore_bytes to the end are equal! */
                return CY_RSLT_SUCCESS;
            }
            curr_filter++;
        }
    }
    return CYHAL_ETHERNET_RSLT_ERR_FAILED;
}
