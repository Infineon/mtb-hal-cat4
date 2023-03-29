/***************************************************************************//**
* \file cyhal_dma.c
*
* \brief
* Implements a high level common interface for interacting with the Infineon M2M DMA.
* This implementation abstracts out the chip specific details. If any chip specific
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

#include "cy_utils.h"
#include "cyhal_m2m_impl.h"
#include "cyhal_system.h"
#include "platform_appscr4.h"
#include "cr4.h"
#include "platform_cache.h"

#include "wiced_osl.h"

#include "m2m_hnddma.h"
#include "m2mdma_core.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
*       Internal - Defines and globals
*******************************************************************************/

#define M2M_DESCRIPTOR_ALIGNMENT            16

#ifdef PLATFORM_L1_CACHE_SHIFT
#define M2M_CACHE_WRITE_BACK                1
#define M2M_OPTIMIZED_DESCRIPTOR_ALIGNMENT  MAX(M2M_DESCRIPTOR_ALIGNMENT, PLATFORM_L1_CACHE_BYTES)
#define M2M_MEMCPY_DCACHE_CLEAN(ptr, size)  platform_dcache_clean_range(ptr, size)
#else
#define M2M_CACHE_WRITE_BACK                0
#define M2M_OPTIMIZED_DESCRIPTOR_ALIGNMENT  M2M_DESCRIPTOR_ALIGNMENT
#define M2M_MEMCPY_DCACHE_CLEAN(ptr, size)  do { UNUSED_PARAMETER(ptr); UNUSED_PARAMETER(size); cpu_data_synchronisation_barrier(); } while(0)
#endif /* PLATFORM_L1_CACHE_SHIFT */

#define IRL_FC_SHIFT                        24 /* frame count */
#define DEF_IRL                             (1 << IRL_FC_SHIFT)

/* DMA tunable parameters */
#define M2M_DMA_RX_BUFFER_COUNT             (20)  /*(3)*/    /* Number of rx buffers */
#define M2M_DMA_TX_DESCRIPTOR_COUNT         (32)  /*(3)*/
#define M2M_DMA_RX_DESCRIPTOR_COUNT         (32)

#define M2M_DMA_RX_BUFFER_HEADROOM          (0)

#define ARMCR4_SW_INT0                      (0x1 << 0)
#define ARMCR4_SW_INT0_STATUS               (0x1 << SW0_ExtIRQn)
#define PLATFORM_WLANCR4                    ((volatile wlancr4_regs_t*)PLATFORM_WLANCR4_REGBASE(0x0))

typedef appscr4_regs_t wlancr4_regs_t;

/* M2M register mapping */
#define M2M_REG_ADR                         PLATFORM_M2M_REGBASE(0)
#define M2M_DMA_ADR                         PLATFORM_M2M_REGBASE(0x200)

/* Globals shared across m2m and dma drivers */
volatile sbm2mregs_t* _cyhal_m2m_m2mreg     = (sbm2mregs_t *)M2M_REG_ADR;
m2m_hnddma_t*         _cyhal_m2m_dma_handle[_CYHAL_M2M_GRPS]    = { NULL };
osl_t*                _cyhal_m2m_osh[_CYHAL_M2M_GRPS]           = { NULL };
cyhal_dma_t*          _cyhal_m2m_cb[_CYHAL_M2M_GRPS]            = { NULL };


/*******************************************************************************
*       Internal - Functions
*******************************************************************************/

static void _cyhal_m2m_core_enable(void)
{
    osl_wrapper_enable((void*)PLATFORM_M2M_MASTER_WRAPPER_REGBASE(0x0));
}

// Note: Presumably this is needed instead of standard memcpy due of cache access
static inline void _cyhal_m2m_device_memcpy(void* dest, const void* src, size_t n)
{
    char *d = (char*)dest;
    char *s = (char*)src;

    while(n--)
    {
        *d++ = *s++;
    }
}


/*******************************************************************************
*       Common M2M functions
*******************************************************************************/

bool _cyhal_m2m_tx_is_idle(_cyhal_m2m_group_t group)
{
    if (m2m_dma_txactive(_cyhal_m2m_dma_handle[group]) != 0)
    {
        /* Still have packets in transit towards WLAN. */
        return false;
    }

    // Special handling for WWD DMA as dealt with in WICED
    if (group == _CYHAL_M2M_GRP_WWD)
    {
        if ((_cyhal_m2m_m2mreg->intregs[ACPU_DMA_TX_CHANNEL].intstatus & I_RI) != 0)
        {
            /*
            * WLAN still not see transmitted packet.
            * WLAN ISR pulls APPS resources up, then ack interrupt, then handle packet.
            * After pulling it is WLAN responsibility to keep resources up until handling is completed.
            * APPS must wait till RX interrupt is acknowledged to make sure APPS not went to deep-sleep and WLAN lost received packet.
            */
            return false;
        }
        if ((PLATFORM_WLANCR4->fiqirq_status & ARMCR4_SW_INT0_STATUS) != 0)
        {
            /*
            * WLAN still has pending SWINT interrupt.
            * To handle it WLAN will pull resources up before.
            * To reduce chances (optimization) of APPS going down and immediately going up let's wait WLAN complete SWINT handling.
            */
            return false;
        }
    }

    return true;
}

void _cyhal_m2m_init_dma(_cyhal_m2m_group_t group, uint8_t tx_ch, uint8_t rx_ch, uint32_t rx_buffer_size)
{
    _cyhal_m2m_core_enable();

    _cyhal_m2m_osh[group] = MALLOC(NULL, sizeof(osl_t));
    CY_ASSERT(_cyhal_m2m_osh[group] != 0);

    /* Need to reserve 4 bytes header room so that the manipulation in wwd_bus_read_frame
     * which moves the data offset back 12 bytes would not break the data pointer */
    _cyhal_m2m_osh[group]->rx_pktget_add_remove  = sizeof(wwd_buffer_header_t) - sizeof(wwd_bus_header_t);
    /* No need to do anything during freeing, host_buffer_release() works fine */
    _cyhal_m2m_osh[group]->tx_pktfree_add_remove = 0;

    _cyhal_m2m_dma_handle[group] = m2m_dma_attach(_cyhal_m2m_osh[group], "M2MDEV", NULL,
                                 (volatile void *)&(_cyhal_m2m_m2mreg->dmaregs[tx_ch].tx),
                                 (volatile void *)&(_cyhal_m2m_m2mreg->dmaregs[rx_ch].rx),
                                 M2M_DMA_TX_DESCRIPTOR_COUNT,
                                 M2M_DMA_RX_DESCRIPTOR_COUNT,
                                 rx_buffer_size,
                                 M2M_DMA_RX_BUFFER_HEADROOM,
                                 M2M_DMA_RX_BUFFER_COUNT,
                                 SDPCMD_RXOFFSET,
                                 NULL);
    CY_ASSERT(_cyhal_m2m_dma_handle[group] != 0);

    m2m_dma_rxinit(_cyhal_m2m_dma_handle[group]);
    m2m_dma_rxfill(_cyhal_m2m_dma_handle[group]);
    W_REG(_cyhal_m2m_osh[group], &_cyhal_m2m_m2mreg->intrxlazy[rx_ch], DEF_IRL);

}

void _cyhal_m2m_disable_dma(uint8_t dma_ch)
{
    /* Switch off the DMA */
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.control &= (~D64_XC_XE);
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].rx.control &= (~D64_XC_XE);
    _cyhal_m2m_m2mreg->intregs[dma_ch].intstatus = ~0x0;

    /* Indicate empty table. Otherwise core is not freeing PMU resources. */
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.ptr = _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.addrlow & 0xFFFF;
}

void _cyhal_m2m_deinit_dma(_cyhal_m2m_group_t group)
{
    m2m_dma_txreset  (_cyhal_m2m_dma_handle[group]);
    m2m_dma_rxreset  (_cyhal_m2m_dma_handle[group]);
    m2m_dma_rxreclaim(_cyhal_m2m_dma_handle[group]);
    m2m_dma_txreclaim(_cyhal_m2m_dma_handle[group], HNDDMA_RANGE_ALL);
    m2m_dma_detach   (_cyhal_m2m_dma_handle[group]);
    MFREE(NULL, _cyhal_m2m_osh[group], sizeof(osl_t));
    _cyhal_m2m_osh[group] = NULL;
    _cyhal_m2m_dma_handle[group] = NULL;
}

void _cyhal_m2m_get_channels(_cyhal_m2m_group_t group, uint8_t *tx_ch, uint8_t *rx_ch, uint8_t *dma_ch)
{
    switch(group)
    {
        case _CYHAL_M2M_GRP_WWD:
            *tx_ch  = ACPU_DMA_TX_CHANNEL;
            *rx_ch  = ACPU_DMA_RX_CHANNEL;
            *dma_ch = MEMCPY_M2M_DMA_CHANNEL;
            break;
        case _CYHAL_M2M_GRP_USR_1:
            *tx_ch  = USR1_DMA_TX_CHANNEL;
            *rx_ch  = USR1_DMA_RX_CHANNEL;
            *dma_ch = MEMCPY_USR_DMA_CHANNEL;
            break;
        case _CYHAL_M2M_GRP_USR_2:
            *tx_ch  = USR2_DMA_TX_CHANNEL;
            *rx_ch  = USR2_DMA_RX_CHANNEL;
            *dma_ch = MEMCPY_USR_DMA_CHANNEL;
            break;
        default:
            CY_ASSERT(0);
            *tx_ch = 0u;
            *rx_ch = 0u;
            *dma_ch = 0u;
            break;
    }
}

void _cyhal_m2m_unprotected_dma_memcpy(uint8_t dma_ch, void* destination, const void* source, uint32_t byte_count)
{
#if M2M_CACHE_WRITE_BACK
    /* It is unsafe to perform DMA to partial cache lines, since it is impossible to predict when a
     * cache line will be evicted back to main memory, and such an event would overwrite the DMA data.
     * Hence it is required that any cache lines which contain other data must be manually copied.
     * These will be the first & last cache lines.
     */
    uint32_t start_offset = PLATFORM_L1_CACHE_LINE_OFFSET(destination);
    if (start_offset != 0)
    {
        /* Start of destination not at start of cache line
         * memcpy the first part of source data to destination
         */
        uint32_t start_copy_byte_count = MIN(byte_count, PLATFORM_L1_CACHE_BYTES - start_offset);
        _cyhal_m2m_device_memcpy(destination, source, start_copy_byte_count);
        source      += start_copy_byte_count;
        destination += start_copy_byte_count;
        byte_count  -= start_copy_byte_count;
    }

    uint32_t end_offset   = ((uint32_t) destination + byte_count) & PLATFORM_L1_CACHE_LINE_MASK;
    if ((byte_count > 0) && (end_offset != 0))
    {
        /* End of destination not at end of cache line
         * memcpy the last part of source data to destination
         */
        uint32_t end_copy_byte_count = MIN(byte_count, end_offset);
        uint32_t offset_from_start = byte_count - end_copy_byte_count;

        _cyhal_m2m_device_memcpy((char*)destination + offset_from_start, (char*)source + offset_from_start, end_copy_byte_count);
        byte_count  -= end_copy_byte_count;
    }

    /* Remaining data should be fully aligned to cache lines */
    CY_ASSERT((byte_count == 0) || (((uint32_t)destination & PLATFORM_L1_CACHE_LINE_MASK) == 0));
    CY_ASSERT((byte_count == 0) || ((((uint32_t)destination + byte_count) & PLATFORM_L1_CACHE_LINE_MASK) == 0));
#endif /* M2M_CACHE_WRITE_BACK */

    /* Check if there are any remaining cache lines (that are entirely part of destination buffer) */
    CY_ASSERT (byte_count > 0);

    #define _CYHAL_M2M_DMA_DESCRIPTORS_STR_EXPAND(name)     #name
    #define _CYHAL_M2M_DMA_DESCRIPTORS_SECTION_NAME(name)   ".dma."_CYHAL_M2M_DMA_DESCRIPTORS_STR_EXPAND(name)
    #define _CYHAL_M2M_DMA_DESCRIPTORS_SECTION(var)         SECTION(_CYHAL_M2M_DMA_DESCRIPTORS_SECTION_NAME(var)) var

    /* Allocate descriptors */
    static volatile dma64dd_t _CYHAL_M2M_DMA_DESCRIPTORS_SECTION(m2m_descriptors)[2] __ALIGNED(M2M_OPTIMIZED_DESCRIPTOR_ALIGNMENT);

    /* Prepare M2M engine */
    _cyhal_m2m_core_enable();

    /* Enable interrupts generation after each frame */
    _cyhal_m2m_m2mreg->intrxlazy[dma_ch] = DEF_IRL;

    /* Setup descriptors */
    m2m_descriptors[0].ctrl1 = D64_CTRL1_EOF | D64_CTRL1_SOF;
    m2m_descriptors[1].ctrl1 = D64_CTRL1_EOF | D64_CTRL1_SOF | D64_CTRL1_IOC;

    /* Setup DMA channel transmitter */
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.addrhigh = 0;
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.addrlow  = ((uint32_t)&m2m_descriptors[0]);  // Transmit descriptor table address
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.ptr      = ((uint32_t)&m2m_descriptors[0] + sizeof(dma64dd_t)) & 0xffff; // needs to be lower 16 bits of descriptor address
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.control  = D64_XC_PD | ((2 << D64_XC_BL_SHIFT) & D64_XC_BL_MASK) | ((1 << D64_XC_PC_SHIFT) & D64_XC_PC_MASK);

    /* Setup DMA channel receiver */
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].rx.addrhigh = 0;
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].rx.addrlow  = ((uint32_t)&m2m_descriptors[1]);  // Transmit descriptor table address
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].rx.ptr      = ((uint32_t)&m2m_descriptors[1] + sizeof(dma64dd_t)) & 0xffff; // needs to be lower 16 bits of descriptor address
    _cyhal_m2m_m2mreg->dmaregs[dma_ch].rx.control  = D64_XC_PD | ((2 << D64_XC_BL_SHIFT) & D64_XC_BL_MASK) | ((1 << D64_XC_PC_SHIFT) & D64_XC_PC_MASK);

    /* Flush pending writes in source buffer range to main memory, so they are accessable by the DMA */
    M2M_MEMCPY_DCACHE_CLEAN(source, byte_count);

#if M2M_CACHE_WRITE_BACK
    /* Invalidate destination buffer - required so that any pending write data is not
     * written back (evicted) over the top of the DMA data.
     * This also prepares the cache for reading the finished DMA data.  However if other functions are silly
     * enough to read the DMA data before it is finished, the cache may get out of sync.
     */
    platform_dcache_inv_range(destination, byte_count);
#endif /* M2M_CACHE_WRITE_BACK */

    CY_ASSERT(byte_count <=  D64_CTRL2_BC_USABLE_MASK);

    uint32_t dma_source = platform_addr_cpu_to_dma(source);
    uint32_t dma_destination = platform_addr_cpu_to_dma(destination);
    while (byte_count > 0)
    {
        uint32_t write_size = MIN(byte_count, D64_CTRL2_BC_USABLE_MASK);

        m2m_descriptors[0].addrlow = dma_source;
        m2m_descriptors[1].addrlow = dma_destination;
        m2m_descriptors[0].ctrl2   = D64_CTRL2_BC_USABLE_MASK & write_size;
        m2m_descriptors[1].ctrl2   = D64_CTRL2_BC_USABLE_MASK & write_size;

        /* Flush the DMA descriptors to main memory to ensure DMA picks them up */
        M2M_MEMCPY_DCACHE_CLEAN((void*)&m2m_descriptors[0], sizeof(m2m_descriptors));

        /* Fire off the DMA */
        _cyhal_m2m_m2mreg->dmaregs[dma_ch].rx.control |= D64_XC_XE;
        _cyhal_m2m_m2mreg->dmaregs[dma_ch].tx.control |= D64_XC_XE;

        /* Update variables */
        byte_count      -= write_size;
        dma_destination += write_size;
        dma_source      += write_size;
    }
}


#if defined(__cplusplus)
}
#endif /* __cplusplus */
