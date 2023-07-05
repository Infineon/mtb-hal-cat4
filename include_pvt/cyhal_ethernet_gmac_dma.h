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
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "bcmdefs.h"
#include "bcmutils.h"
#include "platform_toolchain.h"
#include "cy_utils.h"
#include "cyhal_ethernet_etc.h"
#include "cyhal_system.h"

/* Misc */

#define _CYHAL_BUS_SWAP32(v)                            (v)
#define _CYHAL_GMAC_OSL_DELAY(usec)                     cyhal_system_delay_us(usec)
#define _CYHAL_GMAC_OSL_ARCH_IS_COHERENT()              0
#define _CYHAL_GMAC_OSL_ACP_WAR_ENAB()                  0

/* Register access macros */
#define _cyhal_gmac_wreg32(r, v)                        ({*(volatile uint32 *)(r) = (uint32)(v); __COMPILER_BARRIER();})
#define _cyhal_gmac_rreg32(r)                           (*(volatile uint32 *)(r))
#define _cyhal_gmac_wreg16(r, v)                        ({*(volatile uint16 *)(r) = (uint16)(v); __COMPILER_BARRIER();})
#define _cyhal_gmac_rreg16(r)                           (*(volatile uint16 *)(r))
#define _cyhal_gmac_wreg8(r, v)                         ({*(volatile uint8 *)(r) = (uint8)(v); __COMPILER_BARRIER();})
#define _cyhal_gmac_rreg8(r)                            (*(volatile uint8 *)(r))

#define _CYHAL_GMAC_R_REG(r)                            (*((volatile typeof(r))(r)))
#define _CYHAL_GMAC_W_REG(r, v)                         ({*((volatile typeof(r))(r)) = (v); __COMPILER_BARRIER();})

#define _CYHAL_GMAC_SET_REG(r, mask, val)               _CYHAL_GMAC_W_REG( (r), ( (_CYHAL_GMAC_R_REG(r) & ~(mask) ) | (val) ) )

#define _CYHAL_GMAC_AND_REG(r, v)                       _CYHAL_GMAC_W_REG( (r), _CYHAL_GMAC_R_REG(r) & (v) )
#define _CYHAL_GMAC_OR_REG(r, v)                        _CYHAL_GMAC_W_REG( (r), _CYHAL_GMAC_R_REG(r) | (v) )

#define _CYHAL_GMAC_REG_MAP(pa, size)                   ({UNUSED_PARAMETER(size); (void *)(pa);})
#define _CYHAL_GMAC_REG_UNMAP(va)                       UNUSED_PARAMETER(va)

/* Memory allocation - we eventually want to remove memory allocation from Ethernet HAL */
void* _cyhal_gmac_malloc_align(uint size, uint align_bits);

#define _CYHAL_GMAC_MALLOC(size)                        malloc(size)
#define _CYHAL_GMAC_MALLOC_ALIGN(size, align_bits)      _cyhal_gmac_malloc_align(size, align_bits)
#define _CYHAL_GMAC_MFREE(addr, size)                   ({UNUSED_PARAMETER(size); free(addr);})

/* Prefetch */
static inline void _cyhal_gmac_prefetch_32B(const uint8 *addr, const int cachelines_32B)
{
    switch (cachelines_32B)
    {
        case 4:__asm__ __volatile__("pld\t%a0" :: "p"(addr + 96) : "cc");
        case 3:__asm__ __volatile__("pld\t%a0" :: "p"(addr + 64) : "cc");
        case 2:__asm__ __volatile__("pld\t%a0" :: "p"(addr + 32) : "cc");
        case 1:__asm__ __volatile__("pld\t%a0" :: "p"(addr +  0) : "cc");
        default: break;
    }
}

/* DMA memory allocation */

void* _cyhal_gmac_dma_alloc_consistent(uint size, uint16 align_bits, uint *alloced, dmaaddr_t *descpa);
void _cyhal_gmac_dma_free_consistent(void *va);

#define GMAC_DMA_ALLOC_CONSISTENT(size, align, alloced, pap) \
    ({ \
        _cyhal_gmac_dma_alloc_consistent((size), (align), (alloced), (pap)); \
    })
#define GMAC_DMA_FREE_CONSISTENT(va, size, pa) \
    ({ \
        UNUSED_PARAMETER(size); \
        UNUSED_PARAMETER(pa); \
        _cyhal_gmac_dma_free_consistent(va); \
    })

/* DMA memory mapping/unmapping */

dmaaddr_t _cyhal_gmac_dma_map(void *va, uint size, int direction);

#define GMAC_DMA_TX            1    /* TX direction for DMA map/unmap */
#define GMAC_DMA_RX            2    /* RX direction for DMA map/unmap */

#define GMAC_DMA_MAP(va, size, direction, p) \
    ({ \
        UNUSED_PARAMETER(p); \
        _cyhal_gmac_dma_map((va), (size), (direction)); \
    })
#define GMAC_DMA_UNMAP(pa, size, direction, p) \
    ({ \
        UNUSED_PARAMETER(pa); \
        UNUSED_PARAMETER(size); \
        UNUSED_PARAMETER(direction); \
        UNUSED_PARAMETER(p); \
    })

/* Packet primitives */
void *_cyhal_gmac_pktget(uint len);
void _cyhal_gmac_pktfree(void *p);
uint8_t *_cyhal_gmac_host_read_buffer_get_current_piece_data_pointer(uint8_t *buffer);
uint16_t _cyhal_gmac_host_read_buffer_get_current_piece_size(uint8_t *buffer);
cy_rslt_t _cyhal_gmac_host_read_buffer_set_size(uint8_t *buffer, unsigned short size);
cy_rslt_t _cyhal_gmac_host_read_buffer_add_remove_at_front(uint8_t **buffer, int32_t add_remove_amount);
uint8_t *_cyhal_gmac_host_read_buffer_get_next_piece(uint8_t *buffer);
cy_rslt_t _cyhal_gmac_host_read_buffer_set_next_piece(uint8_t *buffer, uint8_t *next_buffer);


#define _CYHAL_GMAC_PKTGET(len)                            _cyhal_gmac_pktget(len)
#define _CYHAL_GMAC_PKTFREE(p)                             _cyhal_gmac_pktfree(p)
#define _CYHAL_GMAC_PKTDATA(p)                             _cyhal_gmac_host_read_buffer_get_current_piece_data_pointer(p)
#define _CYHAL_GMAC_PKTLEN(p)                              _cyhal_gmac_host_read_buffer_get_current_piece_size(p)
#define _CYHAL_GMAC_PKTSETLEN(p, len)                      _cyhal_gmac_host_read_buffer_set_size( (p), (len) )
#define _CYHAL_GMAC_PKTPULL(p, bytes)                      _cyhal_gmac_host_read_buffer_add_remove_at_front( (uint8_t **)&p, (bytes) )
#define _CYHAL_GMAC_PKTPUSH(p, bytes)                      _CYHAL_GMAC_PKTPULL( (p), -(bytes) )
#define _CYHAL_GMAC_PKTPRIO(p)                             ( {UNUSED_PARAMETER(p); 0; } )
#define _CYHAL_GMAC_PKTNEXT(p)                             _cyhal_gmac_host_read_buffer_get_next_piece(p)
#define _CYHAL_GMAC_PKTSETNEXT(p, x)                       _cyhal_gmac_host_read_buffer_set_next_piece( (p), (x) )

/* Lbuf with fraglist */
#define _CYHAL_GMAC_PKTFRAGPKTID(lb)                       (0)
#define _CYHAL_GMAC_PKTSETFRAGPKTID(lb, id)                UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTFRAGTOTNUM(lb)                      (0)
#define _CYHAL_GMAC_PKTSETFRAGTOTNUM(lb, tot)              UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTFRAGTOTLEN(lb)                      (0)
#define _CYHAL_GMAC_PKTSETFRAGTOTLEN(lb, len)              UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTIFINDEX(lb)                         (0)
#define _CYHAL_GMAC_PKTSETIFINDEX(lb, idx)                 UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTGETLF(len, send, lbuf_type)         (0)

/* in rx path, reuse totlen as used len */
#define _CYHAL_GMAC_PKTFRAGUSEDLEN(lb)                     (0)
#define _CYHAL_GMAC_PKTSETFRAGUSEDLEN(lb, len)             UNUSED_PARAMETER(lb)

#define _CYHAL_GMAC_PKTFRAGLEN(lb, ix)                     (0)
#define _CYHAL_GMAC_PKTSETFRAGLEN(lb, ix, len)             UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTFRAGDATA_LO(lb, ix)                 (0)
#define _CYHAL_GMAC_PKTSETFRAGDATA_LO(lb, ix, addr)        UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTFRAGDATA_HI(lb, ix)                 (0)
#define _CYHAL_GMAC_PKTSETFRAGDATA_HI(lb, ix, addr)        UNUSED_PARAMETER(lb)

/* RX FRAG */
#define _CYHAL_GMAC_PKTISRXFRAG(lb)                        (0)
#define _CYHAL_GMAC_PKTSETRXFRAG(lb)                       UNUSED_PARAMETER(lb)
#define _CYHAL_GMAC_PKTRESETRXFRAG(lb)                     UNUSED_PARAMETER(lb)

/* TX FRAG */
#define _CYHAL_GMAC_PKTISTXFRAG(lb)                        (0)
#define _CYHAL_GMAC_PKTSETTXFRAG(lb)                       UNUSED_PARAMETER(lb)

#define _CYHAL_GMAC_PKTISFRAG(lb)                          (0)
#define _CYHAL_GMAC_PKTFRAGISCHAINED(i)                    (0)
/* TRIM Tail bytes from lfrag */
#define _CYHAL_GMAC_PKTFRAG_TRIM_TAILBYTES(p, len)         _CYHAL_GMAC_PKTSETLEN(p, _CYHAL_GMAC_PKTLEN(p) - len)


#ifndef bzero
#define bzero(b, len)                               memset((b), 0, (len))
#endif /* !bzero */

#ifdef __cplusplus
} /* extern "C" */
#endif

