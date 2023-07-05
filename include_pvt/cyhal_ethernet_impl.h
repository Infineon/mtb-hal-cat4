/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
 * From FreeBSD 2.2.7: Fundamental constants relating to ethernet.
 *
 * $Id: ethernet.h 473241 2014-04-28 19:15:49Z peterwu $
 */

#pragma once

#include "bcmdefs.h"                /* for STATIC_ASSERT macro */

 /******************************************************************************
  *
  * Defaults
  *
 ******************************************************************************/

#ifndef _CYHAL_ETHERNET_DESCNUM_TX
#define _CYHAL_ETHERNET_DESCNUM_TX                       (   8)      /* # tx dma ring descriptors (must be ^2) */
#endif
#ifndef _CYHAL_ETHERNET_DESCNUM_RX
#define _CYHAL_ETHERNET_DESCNUM_RX                       (   4)      /* # rx dma ring descriptors (must be ^2) */
#endif

#define _CYHAL_ETHERNET_NUM_RX_BUFFER_POSTED             MAX( (_CYHAL_ETHERNET_DESCNUM_RX / 2), 1)          /* try to keep this # rbufs posted to the chip */
#define _CYHAL_ETHERNET_RX_BUFFER_SIZE                   (1580 + _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET)      /* max 802.3 ethernet packet size + HW Rx packet header offset */

#ifndef _CYHAL_ETHERNET_DMA_TX_OUTSTAND_READS
#define _CYHAL_ETHERNET_DMA_TX_OUTSTAND_READS            (   2)      /* number of outstanding reads */
#endif

#ifndef _CYHAL_ETHERNET_DMA_TX_PREFETCH_THRESHOLD
#define _CYHAL_ETHERNET_DMA_TX_PREFETCH_THRESHOLD        (   8)      /* DMA Tx flow prefetch threshold */
#endif

#ifndef _CYHAL_ETHERNET_DMA_TX_PREFETCH_DESCRIPTORS
#define _CYHAL_ETHERNET_DMA_TX_PREFETCH_DESCRIPTORS      (  16)      /* max descr allowed in prefetch request */
#endif

#ifndef _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH
#define _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH              ( 128)      /* burst length for dma reads */
#endif

#ifndef _CYHAL_ETHERNET_DMA_RX_PREFETCH_THRESHOLD
#define _CYHAL_ETHERNET_DMA_RX_PREFETCH_THRESHOLD        (   1)      /* prefetch threshold */
#endif

#ifndef _CYHAL_ETHERNET_DMA_RX_PREFETCH_DESCRIPTIONS
#define _CYHAL_ETHERNET_DMA_RX_PREFETCH_DESCRIPTIONS     (   8)      /* max descr allowed in prefetch request */
#endif

#ifndef _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH
#define _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH              ( 128)      /* burst length for dma writes */
#endif

 /******************************************************************************
  *
  * Defines and Macros
  *
 ******************************************************************************/

#ifndef CYHAL_ETHERNET_NUM_FILTERS
#define CYHAL_ETHERNET_NUM_FILTERS 4
#endif

#define _CYHAL_ETHERNET_IS_POWER_OF_2(x)          (((x) != 0) && (((x) & (~(x) + 1)) == (x)))

#if !(_CYHAL_ETHERNET_IS_POWER_OF_2(_CYHAL_ETHERNET_DESCNUM_TX))
#error _CYHAL_ETHERNET_DESCNUM_TX must be a power of 2.
#endif

#if !(_CYHAL_ETHERNET_IS_POWER_OF_2(_CYHAL_ETHERNET_DESCNUM_RX))
#error _CYHAL_ETHERNET_DESCNUM_RX must be a power of 2.
#endif

 /******************************************************************************
  *
  * Forward Declarations
  *
 ******************************************************************************/

 /******************************************************************************
  *
  * Types and Structures
  *
 ******************************************************************************/

/******************************************************************************
 *
 * Functions in cyhal_ethernet_et_funcs.c
 *
 ******************************************************************************/

/* misc callbacks */
void _cyhal_ethernet_et_init(void *et_arg, uint options);
void _cyhal_ethernet_et_reset(void *et_arg);
void _cyhal_ethernet_et_link_up(void *et_arg);
void _cyhal_ethernet_et_link_down(void *et_arg);
int _cyhal_ethernet_et_up(void *et_arg);
void _cyhal_ethernet_et_watchdog(void *et_arg );
int _cyhal_ethernet_et_down(void *et_arg, int reset);
void _cyhal_ethernet_et_intrson(void *et_arg);
void _cyhal_ethernet_et_intrsoff( void *et_arg );
uint16_t _cyhal_ethernet_et_getintrevents(void *et_arg, bool in_isr);
void et_discard(void *et_arg, void *pkt);
int _cyhal_ethernet_et_phy_reset(void *et_arg);
int _cyhal_ethernet_et_phy_free( void *et_arg );
int _cyhal_ethernet_et_set_addrs(void *et_arg);

void _cyhal_ethernet_et_clear_gmac_interrupts(void *et_arg, uint32_t int_bits_to_clear);


