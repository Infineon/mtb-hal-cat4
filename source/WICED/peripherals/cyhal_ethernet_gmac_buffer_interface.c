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
 */

#include <stdio.h>
#include <stdarg.h>
#include <malloc.h>

#include "typedefs.h"
#include "bcmutils.h"
#include "cyhal_ethernet_gmac_siutils.h"
#include "sbchipc.h"
#include "cyhal_ethernet_gmac_pmu.h"
#include "cyhal_ethernet_devices.h"
#include "cyhal_ethernet_gmac_dma.h"
#include "cyhal_ethernet_gmac_buffer_interface.h"
#include "cyhal_system.h"

#include "platform_cache.h"
#include "platform_map.h"
#include "platform_toolchain.h"
#include "platform_mcu_peripheral.h"
#include "platform_appscr4.h"

#include "cy_result.h"
#include "cyhal_ethernet.h"
#include "cyhal_ethernet_etc.h"

#include "cy_utils.h"


#ifndef PLATFORM_L1_CACHE_SHIFT
#define PLATFORM_L1_CACHE_SHIFT   0     /* Value for GMAC DMA */
#endif



#define PMUTICK_CALC_COUNT_SHIFT  4 /* 1<<4 times around loop) */

#if defined (__GNUC__)
#define CLZ(x)                    __builtin_clzl(x)
#elif defined ( __IAR_SYSTEMS_ICC__ )
#define CLZ(x)                    C_bcm_count_leading_zeros(x)
#endif

void*
_cyhal_gmac_malloc_align(uint size, uint align_bits)
{
    return memalign(
        1 << MAX(align_bits, PLATFORM_L1_CACHE_SHIFT),
        PLATFORM_L1_CACHE_ROUND_UP(size));
}

void*
_cyhal_gmac_dma_alloc_consistent(uint size, uint16 align_bits, uint *alloced, dmaaddr_t *descpa)
{
    void *cached_p = _CYHAL_GMAC_MALLOC_ALIGN(size, align_bits);
    void *uncached_p;

    if (cached_p == NULL)
    {
        return NULL;
    }

    platform_dcache_inv_range(cached_p, size); /* prevent any write backs */

    uncached_p = platform_addr_cached_to_uncached(cached_p);

    *alloced = size;
    *descpa = (dmaaddr_t)uncached_p;

    return uncached_p;
}

void
_cyhal_gmac_dma_free_consistent(void *p)
{
    _CYHAL_GMAC_MFREE(platform_addr_uncached_to_cached(p), 0);
}

dmaaddr_t
_cyhal_gmac_dma_map(void *p, uint size, int direction)
{
    if (direction == GMAC_DMA_TX)
    {
        platform_dcache_clean_and_inv_range(p, size);
    }
    else if (direction == GMAC_DMA_RX)
    {
        platform_dcache_inv_range(p, size);
    }
    else
    {
        p = 0;
    }

    return platform_addr_cpu_to_dma(p);
}

void*
_cyhal_gmac_pktget(uint len)
{
    uint8_t *buffer = NULL;
    cy_rslt_t result = _cyhal_gmac_host_read_buffer_get(&buffer, len);

    if (result != CY_RSLT_SUCCESS)
    {
        //_CYHAL_GMAC_PKTGET failed!!
        return NULL;
    }

    return (void *)buffer;
}

void
_cyhal_gmac_pktfree(void *p)
{
    uint8_t *buffer = p;
    _cyhal_gmac_host_read_buffer_release(buffer);
}

/*******************************************************************************
*       Internal - Buffer default functions
*******************************************************************************/
// No-OS version

#define CYHAL_GMAC_NUM_BUFFERS             (12)
#define _CYHAL_ETHERNET_DMA_READ_BUFFER_SIZE   (2*1024)

typedef struct
{
    uint8_t     the_buffer[_CYHAL_ETHERNET_DMA_READ_BUFFER_SIZE];
    void        *buffer_begin;
    uint16_t    buffer_size;

    void        *buffer_use_start;
    uint16_t    current_size;



    bool        in_use;
/* debugging */
    uint32_t    use_count;
    uint32_t    release_count;
} _cyhal_gmac_nons_buffer_init_t;

static _cyhal_gmac_nons_buffer_init_t   internal_buffers[CYHAL_GMAC_NUM_BUFFERS];

void _cyhal_gmac_host_print_buffer_usage(void)
{
    int i;

    for (i = 0; i < CYHAL_GMAC_NUM_BUFFERS; i++)
    {
        if (internal_buffers[i].in_use)
        {
            _CYHAL_ETHERNET_LOG_DEBUG(("DMA Buffer %d (%p) use: %ld  rel:%ld\n", i, internal_buffers[i].buffer_use_start, internal_buffers[i].use_count, internal_buffers[i].release_count ));
        }
    }
}



static int16_t _cyhal_gmac_host_find_buffer_begin_index(void *buffer)
{
    int i;

    for (i = 0; i < CYHAL_GMAC_NUM_BUFFERS; i++)
    {
        if (internal_buffers[i].buffer_begin == buffer)
        {
            if (internal_buffers[i].in_use)
            {
                return i;
            }
        }
    }
    return -1;
}

static int16_t _cyhal_gmac_host_find_buffer_use_index(void *buffer)
{
    int i;

    for (i = 0; i < CYHAL_GMAC_NUM_BUFFERS; i++)
    {
        if (internal_buffers[i].buffer_use_start == buffer)
        {
            if (internal_buffers[i].in_use)
            {
                return i;
            }
        }
    }

    return -1;
}


cy_rslt_t _cyhal_gmac_host_read_buffer_init(void)
{
    int i;

    memset(&internal_buffers, 0x00, sizeof(internal_buffers) );

    for (i = 0; i < CYHAL_GMAC_NUM_BUFFERS; i++)
    {
        internal_buffers[i].buffer_begin = &internal_buffers[i].the_buffer;
        internal_buffers[i].buffer_size  = _CYHAL_ETHERNET_DMA_READ_BUFFER_SIZE;

        ((uint16_t*)internal_buffers[i].buffer_begin)[0]=0xDEAD;
        ((uint16_t*)internal_buffers[i].buffer_begin)[1]=0xBEEF;

        /* Initially set the buffer start _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET bytes into the buffer
         * later, when _CYHAL_GMAC_PKTPUSH() and _CYHAL_GMAC_PKTPULL() gets called, it will work properly
         */
        internal_buffers[i].buffer_use_start = internal_buffers[i].buffer_begin + _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET;
        internal_buffers[i].current_size     = internal_buffers[i].buffer_size  - _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t _cyhal_gmac_host_read_buffer_deinit( void )
{
    memset(&internal_buffers, 0x00, sizeof(internal_buffers) );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t _cyhal_gmac_host_read_buffer_get( uint8_t **buffer, unsigned short size )
{
    int i;

    for (i = 0; i < CYHAL_GMAC_NUM_BUFFERS; i++)
    {
        if (!internal_buffers[i].in_use)
        {
            if ( (uint16_t) size < (uint16_t)(internal_buffers[i].buffer_size - (internal_buffers[i].buffer_use_start - internal_buffers[i].buffer_begin) ) )
            {
                *buffer = internal_buffers[i].buffer_use_start;
                internal_buffers[i].current_size = size;
                internal_buffers[i].in_use = true;
                internal_buffers[i].use_count++;

                _CYHAL_ETHERNET_LOG_DEBUG(("BUFFER GET %d %p (%p) NOW in use!\n", i, internal_buffers[i].buffer_begin ,internal_buffers[i].buffer_use_start));
                return CY_RSLT_SUCCESS;
            }
        }
    }

    *buffer = NULL;
    return CYHAL_ETHERNET_RSLT_ERR_FAILED;
}

void _cyhal_gmac_host_read_buffer_release( uint8_t *buffer )
{
    int16_t i;
    i = _cyhal_gmac_host_find_buffer_begin_index(buffer);
    if (i < 0)
    {
        i = _cyhal_gmac_host_find_buffer_use_index(buffer);
        if (i < 0)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("ERROR: Trying to release a buffer (%p) that is not in use!\n", buffer));
        }
        else
        {
            /* Buffers are not actually allocated in NoNS, hence are not freed */
            _CYHAL_ETHERNET_LOG_DEBUG(("BUFFER FREE %d %p (%p) FREE!\n", i, internal_buffers[i].buffer_begin, internal_buffers[i].buffer_use_start));
            ((uint16_t*)internal_buffers[i].buffer_begin)[0]=0xDEAD;
            ((uint16_t*)internal_buffers[i].buffer_begin)[1]=0xBEEF;
            internal_buffers[i].buffer_use_start = internal_buffers[i].buffer_begin + _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET;
            internal_buffers[i].current_size = internal_buffers[i].buffer_size   - _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET;
            internal_buffers[i].in_use = false;
            internal_buffers[i].release_count++;
        }
    }
    return;
}

uint8_t* _cyhal_gmac_host_read_buffer_get_current_piece_data_pointer( uint8_t *buffer )
{
    int16_t i;
    uint8_t *return_buffer = NULL;

    i = _cyhal_gmac_host_find_buffer_begin_index(buffer);
    if (i < 0)
    {
        i = _cyhal_gmac_host_find_buffer_use_index(buffer);
        if (i < 0)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("ERROR: Trying to Get a buffer ptr (%p) that is not in use!\n", buffer));
        }
        else
        {
            /* Buffer is already defined, return the buffer pointer from the structure */
            return_buffer = internal_buffers[i].buffer_use_start;
        }
    }
    return return_buffer;
}

uint16_t _cyhal_gmac_host_read_buffer_get_current_piece_size( uint8_t *buffer )
{
    uint16_t  size = 0;
    int16_t i;
    i = _cyhal_gmac_host_find_buffer_begin_index(buffer);
    if (i < 0)
    {
        i = _cyhal_gmac_host_find_buffer_use_index(buffer);
        if (i < 0)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("ERROR: Trying to Get a buffer size (%p) that is not in use!\n", buffer));
        }
        else
        {
            size = internal_buffers[i].current_size;
        }
    }

    return size;
}

uint8_t *_cyhal_gmac_host_read_buffer_get_next_piece( uint8_t *buffer )
{
    CY_UNUSED_PARAMETER(buffer);

    return NULL;
}

cy_rslt_t _cyhal_gmac_host_read_buffer_set_next_piece(uint8_t *buffer, uint8_t *next_buffer )
{
    CY_UNUSED_PARAMETER(buffer);
    CY_UNUSED_PARAMETER(next_buffer);

    return CYHAL_ETHERNET_RSLT_ERR_FAILED;
}
cy_rslt_t _cyhal_gmac_host_read_buffer_add_remove_at_front( uint8_t **buffer, int32_t add_remove_amount )
{
    int16_t i;

    if ( (buffer == NULL) || (*buffer == NULL) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    i = _cyhal_gmac_host_find_buffer_begin_index(*buffer);
    if (i < 0)
    {
        i = _cyhal_gmac_host_find_buffer_use_index(*buffer);
        if (i < 0)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("ERROR: Trying to add_remove buffer (%p) that is not in use!\n", *buffer));
        }
        else
        {
            if ( (add_remove_amount < 0) && (internal_buffers[i].buffer_begin == *buffer) )
            {
                _CYHAL_ETHERNET_LOG_ERROR(("ERROR: add_remove buffer (%p) subracting %ld from BEGIN!\n", *buffer, add_remove_amount));
                return CYHAL_ETHERNET_RSLT_ERR_FAILED;
            }

            /* Buffer is already defined, return the buffer pointer from the structure */
            internal_buffers[i].buffer_use_start += add_remove_amount;
            internal_buffers[i].current_size -= add_remove_amount;
            *buffer = internal_buffers[i].buffer_use_start;
            _CYHAL_ETHERNET_LOG_DEBUG(("Add_remove at start %p (%p) \n", internal_buffers[i].buffer_begin, internal_buffers[i].buffer_use_start));
        }
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t _cyhal_gmac_host_read_buffer_set_size( uint8_t *buffer, unsigned short size )
{
    int16_t i;
    i = _cyhal_gmac_host_find_buffer_begin_index(buffer);
    if (i < 0)
    {
        i = _cyhal_gmac_host_find_buffer_use_index(buffer);
        if (i < 0)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("ERROR: Trying to SET a buffer size (%p) that is not in use!\n", buffer));
        }
        else
        {
            internal_buffers[i].current_size = size;
        }
    }
    return CY_RSLT_SUCCESS;
}

