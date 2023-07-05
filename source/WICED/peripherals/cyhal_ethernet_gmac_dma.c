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
 * Generic Broadcom Home Networking Division (HND) DMA module.
 * This supports the following chips: BCM42xx, 44xx, 47xx .
 *
 * $Id: hnddma.c 476741 2014-05-09 18:42:16Z tcarter $
 */

#include <stdio.h>
#include <typedefs.h>
#include "bcmdefs.h"
#include "cyhal_ethernet_devices.h"
#include "bcmendian.h"
#include "hndsoc.h"
#include "bcmutils.h"
#include "cyhal_ethernet_gmac_siutils.h"

#include "sbhnddma.h"
#include "cyhal_ethernet_gmac_mib.h"
#include "cyhal_ethernet_gmac_core.h"

#include "cyhal_ethernet.h"
#include "cyhal_ethernet_gmac_dma.h"
#include "cyhal_ethernet_gmac_dma_funcs.h"

#ifdef BCMDMA32
#define d32txregs    dregs.d32_u.txregs_32
#define d32rxregs    dregs.d32_u.rxregs_32
#define txd32        dregs.d32_u.txd_32
#define rxd32        dregs.d32_u.rxd_32
#endif

#define d64txregs    dregs.d64_u.txregs_64
#define d64rxregs    dregs.d64_u.rxregs_64
#define txd64        dregs.d64_u.txd_64
#define rxd64        dregs.d64_u.rxd_64

#define    MAXNAMEL    8        /* 8 char names */

#define    DI_INFO(dmah)    ((_cyhal_gmac_dma_info_t *)(uintptr)dmah)

/* SplitRX Feature for D11 <TCM,DDR> split pkt reception in Full Dongle Mode */
#if (!defined(__mips__) && !defined(BCM47XX_CA9))
#define D11_SPLIT_RX_FD
#endif

#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
/* Enable/Disable support for Scatter Gather List in RX direction */
#define SGLIST_RX_SUPPORT
#endif

#if defined(BCM47XX_CA9) && !defined(__NetBSD__)
/* Enable/Disable Bulk Descriptor Flushing optimization */
#define BULK_DESCR_FLUSH
#endif

#if defined(BULK_DESCR_FLUSH)
/* Flush length, given number of descriptors */
#define DMA64_SHIFT (4)  /* multiply by sizeof a dma descriptor  (16 Bytes) */
#define DMA64_FLUSH_LEN(num_desc)   ((num_desc) << DMA64_SHIFT)

#define dma64_rxd64(di, ix) (void *)((uint)(&((dma64dd_t *)((di)->rxd64))[ix]))
#define dma64_txd64(di, ix) (void *)((uint)(&((dma64dd_t *)((di)->txd64))[ix]))
#endif /* BULK_DESCR_FLUSH */

/** dma engine software state */
typedef struct dma_info {
    struct hnddma_pub   hnddma;     /* exported structure, don't use hnddma_t, which could be const */
    char                name[MAXNAMEL];    /* callers name for diag msgs */

    _cyhal_gmac_si_t           *sih;       /* sb handle */

    bool                dma64;      /* this dma engine is operating in 64-bit mode */
    bool                addrext;    /* this dma engine supports DmaExtendedAddrChanges */
    uint8               sep_rxhdr;  /* D11_SPLIT_RX_FD: Separate rx header descriptor */

    union {
        struct {
            dma32regs_t *txregs_32; /* 32-bit dma tx engine registers */
            dma32regs_t *rxregs_32; /* 32-bit dma rx engine registers */
            dma32dd_t   *txd_32;    /* pointer to dma32 tx descriptor ring */
            dma32dd_t   *rxd_32;    /* pointer to dma32 rx descriptor ring */
        } d32_u;
        struct {
            dma64regs_t *txregs_64; /* 64-bit dma tx engine registers */
            dma64regs_t *rxregs_64; /* 64-bit dma rx engine registers */
            dma64dd_t   *txd_64;    /* pointer to dma64 tx descriptor ring */
            dma64dd_t   *rxd_64;    /* pointer to dma64 rx descriptor ring */
        } d64_u;
    } dregs;

    uint16              dmadesc_align;    /* alignment requirement for dma descriptors */

    uint16              ntxd;       /* # tx descriptors tunable */
    uint16              txin;       /* index of next descriptor to reclaim */
    uint16              txout;      /* index of next descriptor to post */
    void                **txp;      /* pointer to parallel array of pointers to packets */
//    osldma_t     *tx_dmah;    /* DMA TX descriptor ring handle - not used in GMAC DMA */
    hnddma_seg_map_t    *txp_dmah;  /* DMA MAP meta-data handle */
    dmaaddr_t           txdpa;      /* Aligned physical address of descriptor ring */
    dmaaddr_t           txdpaorig;  /* Original physical address of descriptor ring */
    uint16              txdalign;   /* #bytes added to alloc'd mem to align txd */
    uint32              txdalloc;   /* #bytes allocated for the ring */
    uint32              xmtptrbase; /* When using unaligned descriptors, the ptr register
                                     * is not just an index, it needs all 13 bits to be
                                     * an offset from the addr register.
                                     */

    uint16              nrxd;       /* # rx descriptors tunable */
    uint16              rxin;       /* index of next descriptor to reclaim */
    uint16              rxout;      /* index of next descriptor to post */
    void                **rxp;      /* pointer to parallel array of pointers to packets */
//    osldma_t     *rx_dmah;    /* DMA RX descriptor ring handle */
    hnddma_seg_map_t    *rxp_dmah;  /* DMA MAP meta-data handle */
    dmaaddr_t           rxdpa;      /* Aligned physical address of descriptor ring */
    dmaaddr_t           rxdpaorig;  /* Original physical address of descriptor ring */
    uint16              rxdalign;   /* #bytes added to alloc'd mem to align rxd */
    uint32              rxdalloc;   /* #bytes allocated for the ring */
    uint32              rcvptrbase; /* Base for ptr reg when using unaligned descriptors */

    /* tunables */
    uint16              rxbufsize;  /* rx buffer size in bytes,
                                     * not including the extra headroom
                                     */
    uint                rxextrahdrroom; /* extra rx headroom, reverseved to assist upper stack
                                         *  e.g. some rx pkt buffers will be bridged to tx side
                                         *  without byte copying. The extra headroom needs to be
                                         *  large enough to fit txheader needs.
                                         *  Some dongle driver may not need it.
                                         */
    uint                nrxpost;            /* # rx buffers to keep posted */
    uint                rxoffset;           /* rxcontrol offset */
    uint                ddoffsetlow;        /* add to get dma address of descriptor ring, low 32 bits */
    uint                ddoffsethigh;       /*   high 32 bits */
    uint                dataoffsetlow;      /* add to get dma address of data buffer, low 32 bits */
    uint                dataoffsethigh;     /*   high 32 bits */
    bool                aligndesc_4k;       /* descriptor base need to be aligned or not */
    uint8               rxburstlen;         /* burstlen field for rx (for cores supporting burstlen) */
    uint8               txburstlen;         /* burstlen field for tx (for cores supporting burstlen) */
    uint8               txmultioutstdrd;    /* tx multiple outstanding reads */
    uint8               txprefetchctl;      /* prefetch control for tx */
    uint8               txprefetchthresh;   /* prefetch threshold for tx */
    uint8               rxprefetchctl;      /* prefetch control for rx */
    uint8               rxprefetchthresh;   /* prefetch threshold for rx */
    pktpool_t           *pktpool;           /* pktpool */
    uint                dma_avoidance_cnt;

    uint32              d64_xs0_cd_mask;    /* tx current descriptor pointer mask */
    uint32              d64_xs1_ad_mask;    /* tx active descriptor mask */
    uint32              d64_rs0_cd_mask;    /* rx current descriptor pointer mask */
    uint16              rs0cd;              /* cached value of rcvstatus0 currdescr */
    uint16              xs0cd;              /* cached value of xmtstatus0 currdescr */
    uint16              xs0cd_snapshot;     /* snapshot of xmtstatus0 currdescr */
#ifdef PCIE_PHANTOM_DEV
    uint                *blwar_nsegs;
    uint                *blwar_size;
    uint8               blwar_d11core;
#endif
    uint8               burstsize_ctrl;
    uint                rxavail;    /* rxavail descriptor */
    bool                rxfill_suspended;
    bool                dmapad_required;
    /* XXX: PLEASE READ THE BELOW before adding new fields
     * before adding new small fields at the end, please look at the above struct and see
     * if you could Squeeze the field in the known alignment holes...point is dma_attach is not
     * an aatachfn, because there are cases where this gets called run time for some compiles
    */
} _cyhal_gmac_dma_info_t;

/*
 * If BCMDMA32 is defined, hnddma will support both 32-bit and 64-bit DMA engines.
 * Otherwise it will support only 64-bit.
 *
 * DMA32_ENAB indicates whether hnddma is compiled with support for 32-bit DMA engines.
 * DMA64_ENAB indicates whether hnddma is compiled with support for 64-bit DMA engines.
 *
 * DMA64_MODE indicates whether the current DMA engine is running as 64-bit.
 */
#ifdef BCMDMA32
#define    DMA32_ENAB(di)        1
#define    DMA64_ENAB(di)        1
#define    DMA64_MODE(di)        ((di)->dma64)
#else /* !BCMDMA32 */
#define    DMA32_ENAB(di)        0
#define    DMA64_ENAB(di)        1
#define    DMA64_MODE(di)        1
#endif /* !BCMDMA32 */

/* DMA Scatter-gather list is supported. Note this is limited to TX direction only */
#ifdef BCMDMASGLISTOSL
#define DMASGLIST_ENAB TRUE
#else
#define DMASGLIST_ENAB FALSE
#endif /* BCMDMASGLISTOSL */

/* descriptor bumping macros */
#define    XXD(x, n)     ((x) & ((n) - 1))    /* faster than %, but n must be power of 2 */
#define    TXD(x)        XXD((x), di->ntxd)
#define    RXD(x)        XXD((x), di->nrxd)
#define    NEXTTXD(i)    TXD((i) + 1)
#define    PREVTXD(i)    TXD((i) - 1)
#define    NEXTRXD(i)    RXD((i) + 1)
#define    PREVRXD(i)    RXD((i) - 1)

#define    NTXDACTIVE(h, t)    TXD((t) - (h))
#define    NRXDACTIVE(h, t)    RXD((t) - (h))

/* macros to convert between byte offsets and indexes */
#define    B2I(bytes, type)         ((uint16)((bytes) / sizeof(type)))
#define    I2B(index, type)         ((index) * sizeof(type))

#define    PCI32ADDR_HIGH           0xc0000000  /* address[31:30] */
#define    PCI32ADDR_HIGH_SHIFT             30  /* address[31:30] */

#define    PCI64ADDR_HIGH           0x80000000  /* address[63] */
#define    PCI64ADDR_HIGH_SHIFT             31  /* address[63] */

#ifdef BCM_DMAPAD
    #ifdef BCMROMBUILD
        #define DMAPADREQUIRED(di)        ((di)->dmapad_required)
    #elif defined(BCMSDIODEV_ENABLED)
        #define DMAPADREQUIRED(di)         TRUE
    #else
        #define DMAPADREQUIRED(di)        FALSE
    #endif /* BCMROMBUILD */
#else
    #define DMAPADREQUIRED(di)            FALSE
#endif /* BCM_DMAPAD */

/* Common prototypes */
static bool _dma_isaddrext(_cyhal_gmac_dma_info_t *di);
static bool _dma_descriptor_align(_cyhal_gmac_dma_info_t *di);
static bool _dma_alloc(_cyhal_gmac_dma_info_t *di, uint direction);
static void _dma_detach(_cyhal_gmac_dma_info_t *di);
static void _dma_ddtable_init(_cyhal_gmac_dma_info_t *di, uint direction, dmaaddr_t pa);
static void _dma_rxinit(_cyhal_gmac_dma_info_t *di);
static void *_dma_rx(_cyhal_gmac_dma_info_t *di);
static bool _dma_rxfill(_cyhal_gmac_dma_info_t *di);
static void _dma_rxreclaim(_cyhal_gmac_dma_info_t *di);
static void _dma_rxenable(_cyhal_gmac_dma_info_t *di);
static void *_dma_getnextrxp(_cyhal_gmac_dma_info_t *di, bool forceall);
static void _dma_rx_param_get(_cyhal_gmac_dma_info_t *di, uint16 *rxoffset, uint16 *rxbufsize);

static void _dma_txblock(_cyhal_gmac_dma_info_t *di);
static void _dma_txunblock(_cyhal_gmac_dma_info_t *di);
static uint _dma_txactive(_cyhal_gmac_dma_info_t *di);
static uint _dma_rxactive(_cyhal_gmac_dma_info_t *di);
static uint _dma_activerxbuf(_cyhal_gmac_dma_info_t *di);
static uint _dma_txpending(_cyhal_gmac_dma_info_t *di);
static uint _dma_txcommitted(_cyhal_gmac_dma_info_t *di);

static void *_dma_peeknexttxp(_cyhal_gmac_dma_info_t *di);
static int _dma_peekntxp(_cyhal_gmac_dma_info_t *di, int *len, void *txps[], _cyhal_gmac_txd_range_t range);
static void *_dma_peeknextrxp(_cyhal_gmac_dma_info_t *di);
static uintptr _dma_getvar(_cyhal_gmac_dma_info_t *di, const char *name);
static void _dma_counterreset(_cyhal_gmac_dma_info_t *di);
static void _dma_fifoloopbackenable(_cyhal_gmac_dma_info_t *di, uint32_t mode);
static uint _dma_ctrlflags(_cyhal_gmac_dma_info_t *di, uint mask, uint flags);
static uint8 dma_align_sizetobits(uint size);
static void *dma_ringalloc(uint32 boundary, uint size, uint16 *alignbits, uint* alloced,
                           dmaaddr_t *descpa); //, osldma_t **dmah);
static int _dma_pktpool_set(_cyhal_gmac_dma_info_t *di, pktpool_t *pool);
static bool _dma_rxtx_error(_cyhal_gmac_dma_info_t *di, bool istx);
static void _dma_burstlen_set(_cyhal_gmac_dma_info_t *di, uint8 rxburstlen, uint8 txburstlen);
static uint _dma_avoidancecnt(_cyhal_gmac_dma_info_t *di);
static void _dma_param_set(_cyhal_gmac_dma_info_t *di, uint16 paramid, uint16 paramval);
static bool _dma_glom_enable(_cyhal_gmac_dma_info_t *di, uint32 val);


#ifdef BCMDMA32
/* Prototypes for 32-bit routines */
static bool dma32_alloc(_cyhal_gmac_dma_info_t *di, uint direction);
static bool dma32_txreset(_cyhal_gmac_dma_info_t *di);
static bool dma32_rxreset(_cyhal_gmac_dma_info_t *di);
static bool dma32_txsuspendedidle(_cyhal_gmac_dma_info_t *di);
static int  dma32_txfast(_cyhal_gmac_dma_info_t *di, void *p0, bool commit);
static void *dma32_getnexttxp(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range);
static void *dma32_getnextrxp(_cyhal_gmac_dma_info_t *di, bool forceall);
static void dma32_txrotate(_cyhal_gmac_dma_info_t *di);
static bool dma32_rxidle(_cyhal_gmac_dma_info_t *di);
static void dma32_txinit(_cyhal_gmac_dma_info_t *di);
static bool dma32_txenabled(_cyhal_gmac_dma_info_t *di);
static void dma32_txsuspend(_cyhal_gmac_dma_info_t *di);
static void dma32_txresume(_cyhal_gmac_dma_info_t *di);
static bool dma32_txsuspended(_cyhal_gmac_dma_info_t *di);
#ifdef WL_MULTIQUEUE
static void dma32_txflush(_cyhal_gmac_dma_info_t *di);
static void dma32_txflush_clear(_cyhal_gmac_dma_info_t *di);
#endif /* WL_MULTIQUEUE */
static void dma32_txreclaim(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range);
static bool dma32_txstopped(_cyhal_gmac_dma_info_t *di);
static bool dma32_rxstopped(_cyhal_gmac_dma_info_t *di);
static bool dma32_rxenabled(_cyhal_gmac_dma_info_t *di);
#if defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP)
static void dma32_dumpring(_cyhal_gmac_dma_info_t *di, dma32dd_t *ring, uint start,
    uint end, uint max_num);
static void dma32_dump(_cyhal_gmac_dma_info_t *di, bool dumpring);
static void dma32_dumptx(_cyhal_gmac_dma_info_t *di, bool dumpring);
static void dma32_dumprx(_cyhal_gmac_dma_info_t *di, bool dumpring);
#endif /* defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP) */

static bool _dma32_addrext(dma32regs_t *dma32regs);
#endif  /* BCMDMA32 */

/* Prototypes for 64-bit routines */
static bool dma64_alloc(_cyhal_gmac_dma_info_t *di, uint direction);
static bool dma64_txreset(_cyhal_gmac_dma_info_t *di);
static bool dma64_rxreset(_cyhal_gmac_dma_info_t *di);
static bool dma64_txsuspendedidle(_cyhal_gmac_dma_info_t *di);
static int  dma64_txfast(_cyhal_gmac_dma_info_t *di, void *p0, uint32_t size, bool commit);
static int  dma64_txunframed(_cyhal_gmac_dma_info_t *di, void *p0, uint len, bool commit);
static void *dma64_getpos(_cyhal_gmac_dma_info_t *di, bool direction);
static void *dma64_getnexttxp(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range);
static void *dma64_getnextrxp(_cyhal_gmac_dma_info_t *di, bool forceall);
static INLINE void dma64_dd_upd_64(_cyhal_gmac_dma_info_t *di, dma64dd_t *ddring, dma64addr_t pa, uint outidx,
                                   uint32 *flags, uint32 bufcount);

#ifdef BCMLFRAG
static int  dma64_txfast_lfrag(_cyhal_gmac_dma_info_t *di, void *p0, bool commit);
#endif /* BCMLFRAG */

static void dma64_txrotate(_cyhal_gmac_dma_info_t *di);

static bool dma64_rxidle(_cyhal_gmac_dma_info_t *di);
static void dma64_txinit(_cyhal_gmac_dma_info_t *di);
static bool dma64_txenabled(_cyhal_gmac_dma_info_t *di);
static void dma64_txsuspend(_cyhal_gmac_dma_info_t *di);
static void dma64_txresume(_cyhal_gmac_dma_info_t *di);
static bool dma64_txsuspended(_cyhal_gmac_dma_info_t *di);
#ifdef WL_MULTIQUEUE
static void dma64_txflush(_cyhal_gmac_dma_info_t *di);
static void dma64_txflush_clear(_cyhal_gmac_dma_info_t *di);
#endif /* WL_MULTIQUEUE */
static void dma64_txreclaim(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range);
static bool dma64_txstopped(_cyhal_gmac_dma_info_t *di);
static bool dma64_rxstopped(_cyhal_gmac_dma_info_t *di);
static bool dma64_rxenabled(_cyhal_gmac_dma_info_t *di);
static bool _dma64_addrext(dma64regs_t *dma64regs);

STATIC INLINE uint32 parity32(uint32 data);

#if defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP)
static void dma64_dumpring(_cyhal_gmac_dma_info_t *di, dma64dd_t *ring, uint start,
    uint end, uint max_num);
static void dma64_dump(_cyhal_gmac_dma_info_t *di, bool dumpring);
static void dma64_dumptx(_cyhal_gmac_dma_info_t *di, bool dumpring);
static void dma64_dumprx(_cyhal_gmac_dma_info_t *di, bool dumpring);
#endif /* defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP) */


const di_fcn_t dma64proc = {
    (di_detach_t)_dma_detach,
    (di_txinit_t)dma64_txinit,
    (di_txreset_t)dma64_txreset,
    (di_txenabled_t)dma64_txenabled,
    (di_txsuspend_t)dma64_txsuspend,
    (di_txresume_t)dma64_txresume,
    (di_txsuspended_t)dma64_txsuspended,
    (di_txsuspendedidle_t)dma64_txsuspendedidle,
#ifdef WL_MULTIQUEUE
    (di_txflush_t)dma64_txflush,
    (di_txflush_clear_t)dma64_txflush_clear,
#endif /* WL_MULTIQUEUE */
    (di_txfast_t)dma64_txfast,
    (di_txunframed_t)dma64_txunframed,
    (di_getpos_t)dma64_getpos,
    (di_txstopped_t)dma64_txstopped,
    (di_txreclaim_t)dma64_txreclaim,
    (di_getnexttxp_t)dma64_getnexttxp,
    (di_peeknexttxp_t)_dma_peeknexttxp,
    (di_peekntxp_t)_dma_peekntxp,
    (di_txblock_t)_dma_txblock,
    (di_txunblock_t)_dma_txunblock,
    (di_txactive_t)_dma_txactive,
    (di_txrotate_t)dma64_txrotate,

    (di_rxinit_t)_dma_rxinit,
    (di_rxreset_t)dma64_rxreset,
    (di_rxidle_t)dma64_rxidle,
    (di_rxstopped_t)dma64_rxstopped,
    (di_rxenable_t)_dma_rxenable,
    (di_rxenabled_t)dma64_rxenabled,
    (di_rx_t)_dma_rx,
    (di_rxfill_t)_dma_rxfill,
    (di_rxreclaim_t)_dma_rxreclaim,
    (di_getnextrxp_t)_dma_getnextrxp,
    (di_peeknextrxp_t)_dma_peeknextrxp,
    (di_rxparam_get_t)_dma_rx_param_get,

    (di_fifoloopbackenable_t)_dma_fifoloopbackenable,
    (di_getvar_t)_dma_getvar,
    (di_counterreset_t)_dma_counterreset,
    (di_ctrlflags_t)_dma_ctrlflags,

#if defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP)
    (di_dump_t)dma64_dump,
    (di_dumptx_t)dma64_dumptx,
    (di_dumprx_t)dma64_dumprx,
#else
    NULL,
    NULL,
    NULL,
#endif /* defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP) */
    (di_rxactive_t)_dma_rxactive,
    (di_txpending_t)_dma_txpending,
    (di_txcommitted_t)_dma_txcommitted,
    (di_pktpool_set_t)_dma_pktpool_set,
    (di_rxtxerror_t)_dma_rxtx_error,
    (di_burstlen_set_t)_dma_burstlen_set,
    (di_avoidancecnt_t)_dma_avoidancecnt,
    (di_param_set_t)_dma_param_set,
    (dma_glom_enable_t)_dma_glom_enable,
    (dma_active_rxbuf_t)_dma_activerxbuf,
    40
};

#ifdef BCMDMA32
static const di_fcn_t dma32proc = {
    (di_detach_t)_dma_detach,
    (di_txinit_t)dma32_txinit,
    (di_txreset_t)dma32_txreset,
    (di_txenabled_t)dma32_txenabled,
    (di_txsuspend_t)dma32_txsuspend,
    (di_txresume_t)dma32_txresume,
    (di_txsuspended_t)dma32_txsuspended,
    (di_txsuspendedidle_t)dma32_txsuspendedidle,
#ifdef WL_MULTIQUEUE
    (di_txflush_t)dma32_txflush,
    (di_txflush_clear_t)dma32_txflush_clear,
#endif /* WL_MULTIQUEUE */
    (di_txfast_t)dma32_txfast,
    NULL,
    NULL,
    (di_txstopped_t)dma32_txstopped,
    (di_txreclaim_t)dma32_txreclaim,
    (di_getnexttxp_t)dma32_getnexttxp,
    (di_peeknexttxp_t)_dma_peeknexttxp,
    (di_peekntxp_t)_dma_peekntxp,
    (di_txblock_t)_dma_txblock,
    (di_txunblock_t)_dma_txunblock,
    (di_txactive_t)_dma_txactive,
    (di_txrotate_t)dma32_txrotate,

    (di_rxinit_t)_dma_rxinit,
    (di_rxreset_t)dma32_rxreset,
    (di_rxidle_t)dma32_rxidle,
    (di_rxstopped_t)dma32_rxstopped,
    (di_rxenable_t)_dma_rxenable,
    (di_rxenabled_t)dma32_rxenabled,
    (di_rx_t)_dma_rx,
    (di_rxfill_t)_dma_rxfill,
    (di_rxreclaim_t)_dma_rxreclaim,
    (di_getnextrxp_t)_dma_getnextrxp,
    (di_peeknextrxp_t)_dma_peeknextrxp,
    (di_rxparam_get_t)_dma_rx_param_get,

    (di_fifoloopbackenable_t)_dma_fifoloopbackenable,
    (di_getvar_t)_dma_getvar,
    (di_counterreset_t)_dma_counterreset,
    (di_ctrlflags_t)_dma_ctrlflags,

#if defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP)
    (di_dump_t)dma32_dump,
    (di_dumptx_t)dma32_dumptx,
    (di_dumprx_t)dma32_dumprx,
#else
    NULL,
    NULL,
    NULL,
#endif /* defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP) */
    (di_rxactive_t)_dma_rxactive,
    (di_txpending_t)_dma_txpending,
    (di_txcommitted_t)_dma_txcommitted,
    (di_pktpool_set_t)_dma_pktpool_set,
    (di_rxtxerror_t)_dma_rxtx_error,
    (di_burstlen_set_t)_dma_burstlen_set,
    (di_avoidancecnt_t)_dma_avoidancecnt,
    (di_param_set_t)_dma_param_set,
    NULL,
    NULL,
    40
};
#endif   /* BCMDMA32 */

/**********************************************************************************************
 *
 * Functions
 *
 **********************************************************************************************/
#define CORRECTION_TO_CURR_RX_DESCR_INDEX

#ifdef CORRECTION_TO_CURR_RX_DESCR_INDEX
uint _dma_get_current_buffer_index(_cyhal_gmac_dma_info_t *di)
{
    uint32_t base = (di->rcvptrbase & D64_RS0_CD_MASK);
    uint32_t curr = _CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK;
    if (curr >= base)
    {
        curr = (uint16)B2I( (curr - base), dma64dd_t);
    }
    return curr;
}
uint _dma_get_last_buffer_index(_cyhal_gmac_dma_info_t *di)
{
    uint32_t base = (di->rcvptrbase & D64_RS0_CD_MASK);
    uint32_t end  = _CYHAL_GMAC_R_REG(&di->d64rxregs->ptr) & D64_RS0_CD_MASK;
    if (end >= base)
    {
        end = (uint16)B2I( (end - base), dma64dd_t);
    }
    return end;
}
#endif

/* For SDIO and PCIE, dma_attach() (and some of the functions that it calls) are only invoked in
 * attach context; therefore, they can be reclaimed as a memory optimization. For USB, dma_attach()
 * is called in non-attach context and therefore needs to be a persistent function.
 */
#if defined(BCMSDIODEV_ENABLED) || defined(BCMPCIEDEV_ENABLED)
    #define HNDDMA_RECLAIM_DMA_ATTACH
#endif

#if defined(HNDDMA_RECLAIM_DMA_ATTACH)
    #define BCMATTACHFN_DMA_ATTACH(_fn)    BCMATTACHFN(_fn)
#else
    #define BCMATTACHFN_DMA_ATTACH(_fn)    (_fn)
#endif

hnddma_t *
BCMATTACHFN_DMA_ATTACH(_cyhal_gmac_dma_attach)(const char *name, _cyhal_gmac_si_t *sih,
    volatile void *dmaregstx, volatile void *dmaregsrx,
    uint ntxd, uint nrxd, uint rxbufsize, int rxextheadroom, uint nrxpost, uint rxoffset)
{
    _cyhal_gmac_dma_info_t *di;
    uint size;
    uint32 mask, reg_val;

    /* allocate private info structure */
    if ((di = _CYHAL_GMAC_MALLOC(sizeof (_cyhal_gmac_dma_info_t))) == NULL) {
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
        _CYHAL_ETHERNET_LOG_ERROR(("%s: out of memory, tried too malloc %d bytes\n", __FUNCTION__, sizeof (_cyhal_gmac_dma_info_t)));
#endif
        return (NULL);
    }

    bzero(di, sizeof(_cyhal_gmac_dma_info_t));

    /* old chips w/o sb is no longer supported */
    CY_ASSERT(sih != NULL);

    if (DMA64_ENAB(di))
        di->dma64 = ((_cyhal_gmac_si_core_sflags(sih, 0, 0) & SISF_DMA64) == SISF_DMA64);
    else
        di->dma64 = 0;

    /* check arguments */
    CY_ASSERT(ISPOWEROF2(ntxd));
    CY_ASSERT(ISPOWEROF2(nrxd));

    if (nrxd == 0)
        CY_ASSERT(dmaregsrx == NULL);
    if (ntxd == 0)
        CY_ASSERT(dmaregstx == NULL);

    /* init dma reg pointer */
    if (DMA64_ENAB(di) && DMA64_MODE(di))
    {
        di->d64txregs = (dma64regs_t *)dmaregstx;
        di->d64rxregs = (dma64regs_t *)dmaregsrx;
        di->hnddma.di_fn = (const di_fcn_t *)&dma64proc;
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        CY_ASSERT(ntxd <= D32MAXDD);
        CY_ASSERT(nrxd <= D32MAXDD);
        di->d32txregs = (dma32regs_t *)dmaregstx;
        di->d32rxregs = (dma32regs_t *)dmaregsrx;
        di->hnddma.di_fn = (const di_fcn_t *)&dma32proc;
    }
#endif
    else
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: driver doesn't support 32-bit DMA\n", __FUNCTION__));
        CY_ASSERT(0);
        goto fail;
    }

    /* Default flags (which can be changed by the driver calling dma_ctrlflags
     * before enable): For backwards compatibility both Rx Overflow Continue
     * and Parity are DISABLED.
     *
     * Let us support it for 4390x
     */
    di->hnddma.di_fn->ctrlflags(&di->hnddma, DMA_CTRL_ROC, 0); // | DMA_CTRL_PEN, 0);

    _CYHAL_ETHERNET_LOG_INFO(("%s: %s: %s flags 0x%x ntxd %d nrxd %d rxbufsize %d "
               "rxextheadroom %d nrxpost %d rxoffset %d dmaregstx %p dmaregsrx %p\n",
               name, __FUNCTION__, (DMA64_MODE(di) ? "DMA64" : "DMA32"),
               di->hnddma.dmactrlflags, ntxd, nrxd,
               rxbufsize, rxextheadroom, nrxpost, rxoffset, dmaregstx, dmaregsrx));

    /* make a private copy of our callers name */
    strncpy(di->name, name, MAXNAMEL);
    di->name[MAXNAMEL-1] = '\0';

    di->sih = sih;

    /* save tunables */
    di->ntxd = (uint16)ntxd;
    di->nrxd = (uint16)nrxd;

    /* the actual dma size doesn't include the extra headroom */
    di->rxextrahdrroom = (rxextheadroom == -1) ? BCMEXTRAHDROOM : rxextheadroom;
    if (rxbufsize > BCMEXTRAHDROOM)
        di->rxbufsize = (uint16)(rxbufsize - di->rxextrahdrroom);
    else
        di->rxbufsize = (uint16)rxbufsize;

    di->nrxpost = (uint16)nrxpost;
    di->rxoffset = (uint8)rxoffset;

    /* Get the default values (POR) of the burstlen. This can be overridden by the modules
     * if this has to be different. Otherwise this value will be used to program the control
     * register after the reset or during the init.
     */
    /* XXX: for 4345 PCIE rev 5, writing all 1's to control is broken,
     * you will read all 1's back
     */

    if (dmaregsrx) {
        if (DMA64_ENAB(di) && DMA64_MODE(di)) {

            /* first disable the dma if not already done */
            reg_val = _CYHAL_GMAC_R_REG(&di->d64rxregs->control);
            if (reg_val & D64_XC_XE) {
                reg_val &= ~D64_XC_XE;
                /* Write twice */
                _CYHAL_GMAC_W_REG(&di->d64rxregs->control, reg_val);
                _CYHAL_GMAC_W_REG(&di->d64rxregs->control, reg_val);

            }
            /* detect the dma descriptor address mask,
             * should be 0x1fff before 4360B0, 0xffff start from 4360B0
             */
            _CYHAL_GMAC_W_REG(&di->d64rxregs->addrlow, 0xffffffff);
            /* XXX: for 4345 PCIE rev 5/8, need one more write to make it work */
            if ((_cyhal_gmac_si_coreid(di->sih) == PCIE2_CORE_ID) &&
                ((_cyhal_gmac_si_corerev(di->sih) == 5) || (_cyhal_gmac_si_corerev(di->sih) == 8) ||
                (_cyhal_gmac_si_corerev(di->sih) == 12)))
            {
                _CYHAL_GMAC_W_REG(&di->d64rxregs->addrlow, 0xffffffff);
            }
            mask = _CYHAL_GMAC_R_REG(&di->d64rxregs->addrlow);

            if (mask & 0xfff)
                mask = _CYHAL_GMAC_R_REG(&di->d64rxregs->ptr) | 0xf;
            else
                mask = 0x1fff;

#ifdef BCMM2MDEV_ENABLED
            if (mask == 0xf)
                mask = 0xffff;
#endif

            _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rx_mask: %08x\n", di->name, mask));
            di->d64_rs0_cd_mask = mask;

            if (mask == 0x1fff)
                CY_ASSERT(nrxd <= D64MAXDD);
            else
                CY_ASSERT(nrxd <= D64MAXDD_LARGE);

            di->rxburstlen       = (_CYHAL_GMAC_R_REG(&di->d64rxregs->control) & D64_RC_BL_MASK) >> D64_RC_BL_SHIFT;
            di->rxprefetchctl    = (_CYHAL_GMAC_R_REG(&di->d64rxregs->control) & D64_RC_PC_MASK) >> D64_RC_PC_SHIFT;
            di->rxprefetchthresh = (_CYHAL_GMAC_R_REG(&di->d64rxregs->control) & D64_RC_PT_MASK) >> D64_RC_PT_SHIFT;
        }
#ifdef BCMDMA32
        else if (DMA32_ENAB(di))
        {
            di->rxburstlen       = (_CYHAL_GMAC_R_REG(&di->d32rxregs->control) & RC_BL_MASK) >> RC_BL_SHIFT;
            di->rxprefetchctl    = (_CYHAL_GMAC_R_REG(&di->d32rxregs->control) & RC_PC_MASK) >> RC_PC_SHIFT;
            di->rxprefetchthresh = (_CYHAL_GMAC_R_REG(&di->d32rxregs->control) & RC_PT_MASK) >> RC_PT_SHIFT;
        }
#endif
    }
    if (dmaregstx) {
        if (DMA64_ENAB(di) && DMA64_MODE(di)) {

            /* first disable the dma if not already done */
            reg_val = _CYHAL_GMAC_R_REG(&di->d64txregs->control);
            if (reg_val & D64_XC_XE) {
                reg_val &= ~D64_XC_XE;
                /* Write twice */
                _CYHAL_GMAC_W_REG(&di->d64txregs->control, reg_val);
                _CYHAL_GMAC_W_REG(&di->d64txregs->control, reg_val);
            }

            /* detect the dma descriptor address mask,
             * should be 0x1fff before 4360B0, 0xffff start from 4360B0
             */
            _CYHAL_GMAC_W_REG(&di->d64txregs->addrlow, 0xffffffff);
            /* XXX: for 4345 PCIE rev 5/8, need one more write to make it work */
            if ((_cyhal_gmac_si_coreid(di->sih) == PCIE2_CORE_ID) &&
                ((_cyhal_gmac_si_corerev(di->sih) == 5) || (_cyhal_gmac_si_corerev(di->sih) == 8) ||
                (_cyhal_gmac_si_corerev(di->sih) == 12)))
            {
                _CYHAL_GMAC_W_REG(&di->d64txregs->addrlow, 0xffffffff);
            }
            mask = _CYHAL_GMAC_R_REG(&di->d64txregs->addrlow);

            if (mask & 0xfff)
                mask = _CYHAL_GMAC_R_REG(&di->d64txregs->ptr) | 0xf;
            else
                mask = 0x1fff;

#ifdef BCMM2MDEV_ENABLED
            if (mask == 0xf)
                mask = 0xffff;
#endif

            _CYHAL_ETHERNET_LOG_INFO(("%s: dma_tx_mask: %08x\n", di->name, mask));
            di->d64_xs0_cd_mask = mask;
            di->d64_xs1_ad_mask = mask;

            if (mask == 0x1fff)
                CY_ASSERT(ntxd <= D64MAXDD);
            else
                CY_ASSERT(ntxd <= D64MAXDD_LARGE);

            di->txburstlen       = (_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_BL_MASK) >> D64_XC_BL_SHIFT;
            di->txmultioutstdrd  = (_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_MR_MASK) >> D64_XC_MR_SHIFT;
            di->txprefetchctl    = (_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_PC_MASK) >> D64_XC_PC_SHIFT;
            di->txprefetchthresh = (_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_PT_MASK) >> D64_XC_PT_SHIFT;
        }
#ifdef BCMDMA32
        else if (DMA32_ENAB(di))
        {
            di->txburstlen       = (_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_BL_MASK) >> XC_BL_SHIFT;
            di->txmultioutstdrd  = (_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_MR_MASK) >> XC_MR_SHIFT;
            di->txprefetchctl    = (_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_PC_MASK) >> XC_PC_SHIFT;
            di->txprefetchthresh = (_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_PT_MASK) >> XC_PT_SHIFT;
        }
#endif
    }

    /*
     * figure out the DMA physical address offset for dd and data
     *     PCI/PCIE: they map silicon backplace address to zero based memory, need offset
     *     Other bus: use zero
     *     SI_BUS BIGENDIAN kludge: use sdram swapped region for data buffer, not descriptor
     */
    di->ddoffsetlow = 0;
    di->dataoffsetlow = 0;
    /* for pci bus, add offset */
    if (sih->bustype == PCI_BUS) {
        if ((sih->buscoretype == PCIE_CORE_ID ||
             sih->buscoretype == PCIE2_CORE_ID) &&
            DMA64_MODE(di)) {
            /* pcie with DMA64 */
            di->ddoffsetlow = 0;
            di->ddoffsethigh = SI_PCIE_DMA_H32;
        } else {
            /* pci(DMA32/DMA64) or pcie with DMA32 */
            if ((CHIPID(sih->chip) == BCM4322_CHIP_ID) ||
                (CHIPID(sih->chip) == BCM4342_CHIP_ID) ||
                (CHIPID(sih->chip) == BCM43221_CHIP_ID) ||
                (CHIPID(sih->chip) == BCM43231_CHIP_ID) ||
                (CHIPID(sih->chip) == BCM43111_CHIP_ID) ||
                (CHIPID(sih->chip) == BCM43112_CHIP_ID) ||
                (CHIPID(sih->chip) == BCM43222_CHIP_ID))
                di->ddoffsetlow = SI_PCI_DMA2;
            else
                di->ddoffsetlow = SI_PCI_DMA;

            di->ddoffsethigh = 0;
        }
        di->dataoffsetlow =  di->ddoffsetlow;
        di->dataoffsethigh =  di->ddoffsethigh;
    }

#ifndef DSLCPE
#if defined(__mips__) && defined(IL_BIGENDIAN)
    /* use sdram swapped region for data buffers but not dma descriptors.
     * XXX this assumes that we are running on a 47xx mips with a swap window.
     * But __mips__ is too general, there should be one si_ishndmips() checking
     * for OUR mips
     */
    di->dataoffsetlow = di->dataoffsetlow + SI_SDRAM_SWAPPED;
#endif /* defined(__mips__) && defined(IL_BIGENDIAN) */
#endif /* DSLCPE */
    /* WAR64450 : DMACtl.Addr ext fields are not supported in SDIOD core. */
    if ((_cyhal_gmac_si_coreid(sih) == SDIOD_CORE_ID) && ((_cyhal_gmac_si_corerev(sih) > 0) && (_cyhal_gmac_si_corerev(sih) <= 2)))
        di->addrext = 0;
    else if ((_cyhal_gmac_si_coreid(sih) == I2S_CORE_ID) && /* PR64456: I2S Rev0 and Rev1 lack AddrExt */
             ((_cyhal_gmac_si_corerev(sih) == 0) || (_cyhal_gmac_si_corerev(sih) == 1)))
        di->addrext = 0;
    else
        di->addrext = _dma_isaddrext(di);

    /* does the descriptors need to be aligned and if yes, on 4K/8K or not */
    di->aligndesc_4k = _dma_descriptor_align(di);
    if (di->aligndesc_4k) {
        if (DMA64_MODE(di)) {
            di->dmadesc_align = D64RINGALIGN_BITS;
            if ((ntxd < D64MAXDD / 2) && (nrxd < D64MAXDD / 2)) {
                /* for smaller dd table, HW relax the alignment requirement */
                di->dmadesc_align = D64RINGALIGN_BITS  - 1;
            }
        } else
            di->dmadesc_align = D32RINGALIGN_BITS;
    } else {
        /* The start address of descriptor table should be algined to cache line size,
         * or other structure may share a cache line with it, which can lead to memory
         * overlapping due to cache write-back operation. In the case of MIPS 74k, the
         * cache line size is 32 bytes.
         */
#ifdef __mips__
        di->dmadesc_align = 5;    /* 32 byte alignment */
#else
        di->dmadesc_align = 4;    /* 16 byte alignment */
#endif
    }

    _CYHAL_ETHERNET_LOG_DEBUG(("DMA descriptor align_needed %d, align %d\n",
        di->aligndesc_4k, di->dmadesc_align));

    /* allocate tx packet pointer vector */
    if (ntxd) {
        size = ntxd * sizeof(void *);
        if ((di->txp = _CYHAL_GMAC_MALLOC(size)) == NULL) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: %s: out of tx memory, tried to malloc %d bytes\n",
                       di->name, __FUNCTION__, size));
            goto fail;
        }
        bzero(di->txp, size);
    }

    /* allocate rx packet pointer vector */
    if (nrxd) {
        size = nrxd * sizeof(void *);
        if ((di->rxp = _CYHAL_GMAC_MALLOC(size)) == NULL) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: %s: out of rx memory, tried to malloc %d bytes\n",
                       di->name, __FUNCTION__, size));
            goto fail;
        }
        bzero(di->rxp, size);
    }

    /* allocate transmit descriptor ring, only need ntxd descriptors but it must be aligned */
    if (ntxd) {
        if (!_dma_alloc(di, GMAC_DMA_TX))
            goto fail;
    }

    /* allocate receive descriptor ring, only need nrxd descriptors but it must be aligned */
    if (nrxd) {
        if (!_dma_alloc(di, GMAC_DMA_RX))
            goto fail;
    }

    if ((di->ddoffsetlow != 0) && !di->addrext) {
        if (PHYSADDRLO(di->txdpa) > SI_PCI_DMA_SZ) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: %s: txdpa 0x%x: addrext not supported\n",
                       di->name, __FUNCTION__, (uint32)PHYSADDRLO(di->txdpa)));
            goto fail;
        }
        if (PHYSADDRLO(di->rxdpa) > SI_PCI_DMA_SZ) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: %s: rxdpa 0x%x: addrext not supported\n",
                       di->name, __FUNCTION__, (uint32)PHYSADDRLO(di->rxdpa)));
            goto fail;
        }
    }

    _CYHAL_ETHERNET_LOG_INFO(("ddoffsetlow 0x%x ddoffsethigh 0x%x dataoffsetlow 0x%x dataoffsethigh "
               "0x%x addrext %d\n", di->ddoffsetlow, di->ddoffsethigh, di->dataoffsetlow,
               di->dataoffsethigh, di->addrext));

    /* allocate DMA mapping vectors */
    if (DMASGLIST_ENAB) {
        if (ntxd) {
            size = ntxd * sizeof(hnddma_seg_map_t);
            if ((di->txp_dmah = (hnddma_seg_map_t *)_CYHAL_GMAC_MALLOC(size)) == NULL)
                goto fail;
            bzero(di->txp_dmah, size);
        }

        if (nrxd) {
            size = nrxd * sizeof(hnddma_seg_map_t);
            if ((di->rxp_dmah = (hnddma_seg_map_t *)_CYHAL_GMAC_MALLOC(size)) == NULL)
                goto fail;
            bzero(di->rxp_dmah, size);
        }
    }
#ifdef BCM_DMAPAD
    #if defined(BCMSDIODEV_ENABLED)
        di->dmapad_required = TRUE;
    #else
        di->dmapad_required = FALSE;
    #endif /* BCMSDIODEV_ENABLED */
#else
    di->dmapad_required = FALSE;
#endif /* BCM_DMAPAD */

    return ((hnddma_t *)di);

fail:
    _dma_detach(di);
    return (NULL);
}

/** init the tx or rx descriptor */
static INLINE void
dma32_dd_upd(_cyhal_gmac_dma_info_t *di, dma32dd_t *ddring, dmaaddr_t pa, uint outidx, uint32 *flags,
    uint32 bufcount)
{
    /* dma32 uses 32-bit control to fit both flags and bufcounter */
    *flags = *flags | (bufcount & CTRL_BC_MASK);

    if ((di->dataoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
        W_SM(&ddring[outidx].addr, _CYHAL_BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
        W_SM(&ddring[outidx].ctrl, _CYHAL_BUS_SWAP32(*flags));
    } else {
        /* address extension */
        uint32 ae;
        CY_ASSERT(di->addrext);
        ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
        PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

        *flags |= (ae << CTRL_AE_SHIFT);
        W_SM(&ddring[outidx].addr, _CYHAL_BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
        W_SM(&ddring[outidx].ctrl, _CYHAL_BUS_SWAP32(*flags));
    }
}

/** Check for odd number of 1's */
STATIC INLINE uint32 parity32(uint32 data)
{
    data ^= data >> 16;
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;

    return (data & 1);
}

#define DMA64_DD_PARITY(dd)  parity32((dd)->addrlow ^ (dd)->addrhigh ^ (dd)->ctrl1 ^ (dd)->ctrl2)

/**
 * init the tx or rx descriptor
 * XXX - how to handle native 64-bit addressing AND bit64 extension
 */
static INLINE void
dma64_dd_upd(_cyhal_gmac_dma_info_t *di, dma64dd_t *ddring, dmaaddr_t pa, uint outidx, uint32 *flags,
    uint32 bufcount)
{
    uint32 ctrl2 = bufcount & D64_CTRL2_BC_MASK;

    /* PCI bus with big(>1G) physical address, use address extension */
#if defined(__mips__) && defined(IL_BIGENDIAN)
    if ((di->dataoffsetlow == SI_SDRAM_SWAPPED) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
#else
    if ((di->dataoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
#endif /* defined(__mips__) && defined(IL_BIGENDIAN) */
        /* XXX This is where 64-bit addr ext will come into picture but most likely
         * nobody will be around by the time we have full 64-bit memory addressing
         * requirement
         */
        CY_ASSERT((PHYSADDRHI(pa) & PCI64ADDR_HIGH) == 0);

        W_SM(&ddring[outidx].addrlow, _CYHAL_BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
        W_SM(&ddring[outidx].addrhigh, _CYHAL_BUS_SWAP32(PHYSADDRHI(pa) + di->dataoffsethigh));
        W_SM(&ddring[outidx].ctrl1, _CYHAL_BUS_SWAP32(*flags));
        W_SM(&ddring[outidx].ctrl2, _CYHAL_BUS_SWAP32(ctrl2));
    } else {
        /* address extension for 32-bit PCI */
        uint32 ae;
        CY_ASSERT(di->addrext);

        ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
        PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;
        CY_ASSERT(PHYSADDRHI(pa) == 0);

        ctrl2 |= (ae << D64_CTRL2_AE_SHIFT) & D64_CTRL2_AE;
        W_SM(&ddring[outidx].addrlow, _CYHAL_BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
        W_SM(&ddring[outidx].addrhigh, _CYHAL_BUS_SWAP32(0 + di->dataoffsethigh));
        W_SM(&ddring[outidx].ctrl1, _CYHAL_BUS_SWAP32(*flags));
        W_SM(&ddring[outidx].ctrl2, _CYHAL_BUS_SWAP32(ctrl2));
    }
    if (di->hnddma.dmactrlflags & DMA_CTRL_PEN) {
        if (DMA64_DD_PARITY(&ddring[outidx])) {
            W_SM(&ddring[outidx].ctrl2, _CYHAL_BUS_SWAP32(ctrl2 | D64_CTRL2_PARITY));
        }
    }

#if defined(BCM47XX_CA9) && !defined(__NetBSD__) && !defined(BULK_DESCR_FLUSH)
    GMAC_DMA_MAP((void *)(((uint)(&ddring[outidx])) & ~0x1f), 32, GMAC_DMA_TX, NULL);
#endif /* BCM47XX_CA9 && !__NetBSD__ && !BULK_DESCR_FLUSH */

}

static bool
_dma32_addrext(dma32regs_t *dma32regs)
{
    uint32 w;

    _CYHAL_GMAC_OR_REG(&dma32regs->control, XC_AE);
    w = _CYHAL_GMAC_R_REG(&dma32regs->control);
    _CYHAL_GMAC_AND_REG(&dma32regs->control, ~XC_AE);
    return ((w & XC_AE) == XC_AE);
}

static bool
BCMATTACHFN_DMA_ATTACH(_dma_alloc)(_cyhal_gmac_dma_info_t *di, uint direction)
{
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        return dma64_alloc(di, direction);
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        return dma32_alloc(di, direction);
    }
#endif
    else
    {
        CY_ASSERT(0);
    }
}

/** !! may be called with core in reset */
static void
_dma_detach(_cyhal_gmac_dma_info_t *di)
{

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_detach\n", di->name));

    /* shouldn't be here if descriptors are unreclaimed */
    CY_ASSERT(di->txin == di->txout);
    CY_ASSERT(di->rxin == di->rxout);
#ifdef PCIE_PHANTOM_DEV
    /* Free up burstlength war segs/size */
    if (di->blwar_size)
        _CYHAL_GMAC_MFREE((void *)di->blwar_size, di->ntxd * sizeof(uint));
    if (di->blwar_nsegs)
        _CYHAL_GMAC_MFREE((void *)di->blwar_nsegs, di->ntxd * sizeof(uint));
#endif

    /* free rx packet DMA handles */
    if (di->rxp_dmah)
        _CYHAL_GMAC_MFREE((void *)di->rxp_dmah, di->nrxd * sizeof(hnddma_seg_map_t));
    /* free tx packet DMA handles */
    if (di->txp_dmah)
        _CYHAL_GMAC_MFREE((void *)di->txp_dmah, di->ntxd * sizeof(hnddma_seg_map_t));


    /* free dma descriptor rings */
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        if (di->rxd64)
            GMAC_DMA_FREE_CONSISTENT( ((int8 *)(uintptr)di->rxd64 - di->rxdalign),
                                di->rxdalloc, (di->rxdpaorig) );    //, di->rx_dmah);
        if (di->txd64)
            GMAC_DMA_FREE_CONSISTENT( ((int8 *)(uintptr)di->txd64 - di->txdalign),
                                di->txdalloc, (di->txdpaorig) );    //, di->tx_dmah);
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        if (di->rxd32)
            GMAC_DMA_FREE_CONSISTENT( ((int8 *)(uintptr)di->rxd32 - di->rxdalign),
                                di->rxdalloc, (di->rxdpaorig), di->rx_dmah);
        if (di->txd32)
            GMAC_DMA_FREE_CONSISTENT( ((int8 *)(uintptr)di->txd32 - di->txdalign),
                                di->txdalloc, (di->txdpaorig), di->tx_dmah);
    }
#endif
    else
    {
        CY_ASSERT(0);
    }


    /* free packet pointer vectors */
    if (di->rxp)
        _CYHAL_GMAC_MFREE((void *)di->rxp, (di->nrxd * sizeof(void *)));
    if (di->txp)
        _CYHAL_GMAC_MFREE((void *)di->txp, (di->ntxd * sizeof(void *)));


    /* free our private info structure */
    _CYHAL_GMAC_MFREE((void *)di, sizeof(_cyhal_gmac_dma_info_t));

}

static bool
BCMATTACHFN_DMA_ATTACH(_dma_descriptor_align)(_cyhal_gmac_dma_info_t *di)
{
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        uint32 addrl;

        /* Check to see if the descriptors need to be aligned on 4K/8K or not */
        if (di->d64txregs != NULL) {
            _CYHAL_GMAC_W_REG(&di->d64txregs->addrlow, 0xff0);
            addrl = _CYHAL_GMAC_R_REG(&di->d64txregs->addrlow);
            if (addrl != 0)
                return FALSE;
        } else if (di->d64rxregs != NULL) {
            _CYHAL_GMAC_W_REG(&di->d64rxregs->addrlow, 0xff0);
            addrl = _CYHAL_GMAC_R_REG(&di->d64rxregs->addrlow);
            if (addrl != 0)
                return FALSE;
        }
    }
    return TRUE;
}

/** return TRUE if this dma engine supports DmaExtendedAddrChanges, otherwise FALSE */
static bool
BCMATTACHFN_DMA_ATTACH(_dma_isaddrext)(_cyhal_gmac_dma_info_t *di)
{
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        /* DMA64 supports full 32- or 64-bit operation. AE is always valid */

        /* not all tx or rx channel are available */
        if (di->d64txregs != NULL) {
            if (!_dma64_addrext(di->d64txregs)) {
                _CYHAL_ETHERNET_LOG_ERROR(("%s: _dma_isaddrext: DMA64 tx doesn't have AE set\n",
                    di->name));
                CY_ASSERT(0);
            }
            return TRUE;
        } else if (di->d64rxregs != NULL) {
            if (!_dma64_addrext(di->d64rxregs)) {
                _CYHAL_ETHERNET_LOG_ERROR(("%s: _dma_isaddrext: DMA64 rx doesn't have AE set\n",
                    di->name));
                CY_ASSERT(0);
            }
            return TRUE;
        }
        return FALSE;
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        if (di->d32txregs)
            return (_dma32_addrext(di->d32txregs));
        else if (di->d32rxregs)
            return (_dma32_addrext(di->d32rxregs));
    }
#endif
    else
    {
        CY_ASSERT(0);
    }

    return FALSE;
}

/** initialize descriptor table base address */
static void
_dma_ddtable_init(_cyhal_gmac_dma_info_t *di, uint direction, dmaaddr_t pa)
{
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        if (!di->aligndesc_4k) {
            if (direction == GMAC_DMA_TX)
                di->xmtptrbase = PHYSADDRLO(pa);
            else
                di->rcvptrbase = PHYSADDRLO(pa);
        }

        if ((di->ddoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
            if (direction == GMAC_DMA_TX) {
                _CYHAL_GMAC_W_REG(&di->d64txregs->addrlow, (PHYSADDRLO(pa) +
                                                         di->ddoffsetlow));
                _CYHAL_GMAC_W_REG(&di->d64txregs->addrhigh, (PHYSADDRHI(pa) +
                                                          di->ddoffsethigh));
            } else {
                _CYHAL_GMAC_W_REG(&di->d64rxregs->addrlow, (PHYSADDRLO(pa) +
                                                         di->ddoffsetlow));
                _CYHAL_GMAC_W_REG(&di->d64rxregs->addrhigh, (PHYSADDRHI(pa) +
                                                          di->ddoffsethigh));
            }
        } else {
            /* DMA64 32bits address extension */
            uint32 ae;
            CY_ASSERT(di->addrext);
            CY_ASSERT(PHYSADDRHI(pa) == 0);

            /* shift the high bit(s) from pa to ae */
            ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
            PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

            if (direction == GMAC_DMA_TX) {
                _CYHAL_GMAC_W_REG(&di->d64txregs->addrlow, (PHYSADDRLO(pa) +
                                                         di->ddoffsetlow));
                _CYHAL_GMAC_W_REG(&di->d64txregs->addrhigh, di->ddoffsethigh);
                _CYHAL_GMAC_SET_REG(&di->d64txregs->control, D64_XC_AE,
                    (ae << D64_XC_AE_SHIFT));
            } else {
                _CYHAL_GMAC_W_REG(&di->d64rxregs->addrlow, (PHYSADDRLO(pa) +
                                                         di->ddoffsetlow));
                _CYHAL_GMAC_W_REG(&di->d64rxregs->addrhigh, di->ddoffsethigh);
                _CYHAL_GMAC_SET_REG(&di->d64rxregs->control, D64_RC_AE,
                    (ae << D64_RC_AE_SHIFT));
            }
        }
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        CY_ASSERT(PHYSADDRHI(pa) == 0);
        if ((di->ddoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
            if (direction == GMAC_DMA_TX)
                _CYHAL_GMAC_W_REG(&di->d32txregs->addr, (PHYSADDRLO(pa) +
                                                      di->ddoffsetlow));
            else
                _CYHAL_GMAC_W_REG(&di->d32rxregs->addr, (PHYSADDRLO(pa) +
                                                      di->ddoffsetlow));
        } else {
            /* dma32 address extension */
            uint32 ae;
            CY_ASSERT(di->addrext);

            /* shift the high bit(s) from pa to ae */
            ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
            PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

            if (direction == GMAC_DMA_TX) {
                _CYHAL_GMAC_W_REG(&di->d32txregs->addr, (PHYSADDRLO(pa) +
                                                      di->ddoffsetlow));
                _CYHAL_GMAC_SET_REG(&di->d32txregs->control, XC_AE, ae <<XC_AE_SHIFT);
            } else {
                _CYHAL_GMAC_W_REG(&di->d32rxregs->addr, (PHYSADDRLO(pa) +
                                                      di->ddoffsetlow));
                _CYHAL_GMAC_SET_REG(&di->d32rxregs->control, RC_AE, ae <<RC_AE_SHIFT);
            }
        }
    }
#endif
    else
    {
        CY_ASSERT(0);
    }
}

static void
_dma_fifoloopbackenable(_cyhal_gmac_dma_info_t *di, uint32_t mode)
{
    CY_UNUSED_PARAMETER(mode);

    if (di != NULL)
    {
        _CYHAL_ETHERNET_LOG_INFO(("%s: dma_fifoloopbackenable mode: %ld\n", di->name, mode));

        if (DMA64_ENAB(di) && DMA64_MODE(di))
        {
            /* we are only supporting one type of loopback, it is not DMA loopback */
            _CYHAL_GMAC_AND_REG(&di->d64txregs->control, ~D64_XC_LE);
        }
#ifdef BCMDMA32
        else if (DMA32_ENAB(di))
        {
            if( on_off == false )
            {
                _CYHAL_GMAC_AND_REG(&di->d32txregs->control, ~XC_LE);
            }
            else
            {
                _CYHAL_GMAC_OR_REG(&di->d32txregs->control, XC_LE);
            }
        }
#endif
        else
            CY_ASSERT(0);
    }
}

static void
_dma_rxinit(_cyhal_gmac_dma_info_t *di)
{
#ifdef BCMM2MDEV_ENABLED
    uint32 addrlow;
#endif
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxinit\n", di->name));

    if (di->nrxd == 0)
        return;

    /* During the reset procedure, the active rxd may not be zero if pktpool is
     * enabled, we need to reclaim active rxd to avoid rxd being leaked.
     */
    if ((POOL_ENAB(di->pktpool)) && (NRXDACTIVE(di->rxin, di->rxout))) {
        _dma_rxreclaim(di);
    }

    CY_ASSERT(di->rxin == di->rxout);
    di->rxin = di->rxout = di->rs0cd = 0;
    di->rxavail = di->nrxd - 1;

#if defined(D11_SPLIT_RX_FD)
    /* XXX: to protect cases where driver doesn't post more buffers than the descriptors */
    /* XXX: because each buffer really means multiple descriptors */
    if (di->sep_rxhdr) {
        di->nrxpost = MIN(di->nrxpost, di->rxavail/2);
    }
#endif /* D11_SPLIT_RX_FD */

    /* clear rx descriptor ring */
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        BZERO_SM((void *)(uintptr)di->rxd64, (di->nrxd * sizeof(dma64dd_t)));

        /* DMA engine with out alignment requirement requires table to be inited
         * before enabling the engine
         */
        if (!di->aligndesc_4k)
            _dma_ddtable_init(di, GMAC_DMA_RX, di->rxdpa);

#ifdef BCMM2MDEV_ENABLED
    addrlow = _CYHAL_GMAC_R_REG(&di->d64rxregs->addrlow);
    addrlow &= 0xffff;
    _CYHAL_GMAC_W_REG(&di->d64rxregs->ptr, addrlow);
#endif

        _dma_rxenable(di);

        if (di->aligndesc_4k)
            _dma_ddtable_init(di, GMAC_DMA_RX, di->rxdpa);

#ifdef BCMM2MDEV_ENABLED
    addrlow = _CYHAL_GMAC_R_REG(&di->d64rxregs->addrlow);
    addrlow &= 0xffff;
    _CYHAL_GMAC_W_REG(&di->d64rxregs->ptr, addrlow);
#endif

    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        BZERO_SM((void *)(uintptr)di->rxd32, (di->nrxd * sizeof(dma32dd_t)));
        _dma_rxenable(di);
        _dma_ddtable_init(di, GMAC_DMA_RX, di->rxdpa);
    }
#endif
    else
    {
        CY_ASSERT(0);
    }
}

static void
_dma_rxenable(_cyhal_gmac_dma_info_t *di)
{
    uint dmactrlflags = di->hnddma.dmactrlflags;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxenable\n", di->name));

    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        uint32 control = (_CYHAL_GMAC_R_REG(&di->d64rxregs->control) & D64_RC_AE) | D64_RC_RE;

        if ((dmactrlflags & DMA_CTRL_PEN) == 0)
            control |= D64_RC_PD;

        if (dmactrlflags & DMA_CTRL_ROC)
            control |= D64_RC_OC;

        /* These bits 20:18 (burstLen) of control register can be written but will take
         * effect only if these bits are valid. So this will not affect previous versions
         * of the DMA. They will continue to have those bits set to 0.
         */
        control &= ~D64_RC_BL_MASK;
        control |= (di->rxburstlen << D64_RC_BL_SHIFT);

        control &= ~D64_RC_PC_MASK;
        control |= (di->rxprefetchctl << D64_RC_PC_SHIFT);

        control &= ~D64_RC_PT_MASK;
        control |= (di->rxprefetchthresh << D64_RC_PT_SHIFT);

#if defined(D11_SPLIT_RX_FD)
        /* Separate rx hdr descriptor */
        if (di->sep_rxhdr)
            control |= (di->sep_rxhdr << D64_RC_SHIFT);
#endif /* D11_SPLIT_RX_FD */

        _CYHAL_GMAC_W_REG(&di->d64rxregs->control,
              ((di->rxoffset << D64_RC_RO_SHIFT) | control));
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        uint32 control = (_CYHAL_GMAC_R_REG(&di->d32rxregs->control) & RC_AE) | RC_RE;

        if ((dmactrlflags & DMA_CTRL_PEN) == 0)
            control |= RC_PD;

        if (dmactrlflags & DMA_CTRL_ROC)
            control |= RC_OC;

        /* These bits 20:18 (burstLen) of control register can be written but will take
         * effect only if these bits are valid. So this will not affect previous versions
         * of the DMA. They will continue to have those bits set to 0.
         */
        control &= ~RC_BL_MASK;
        control |= (di->rxburstlen << RC_BL_SHIFT);

        control &= ~RC_PC_MASK;
        control |= (di->rxprefetchctl << RC_PC_SHIFT);

        control &= ~RC_PT_MASK;
        control |= (di->rxprefetchthresh << RC_PT_SHIFT);

        _CYHAL_GMAC_W_REG(&di->d32rxregs->control,
              ((di->rxoffset << RC_RO_SHIFT) | control));
    }
#endif
    else
    {
        CY_ASSERT(0);
    }
}

static void
_dma_rx_param_get(_cyhal_gmac_dma_info_t *di, uint16 *rxoffset, uint16 *rxbufsize)
{
    /* the normal values fit into 16 bits */
    *rxoffset = (uint16)di->rxoffset;
    *rxbufsize = (uint16)di->rxbufsize;
}

/**
 * !! rx entry routine
 * returns a pointer to the next frame received, or NULL if there are no more
 *   if DMA_CTRL_RXMULTI is defined, DMA scattering(multiple buffers) is supported
 *      with pkts chain
 *   otherwise, it's treated as giant pkt and will be tossed.
 *   The DMA scattering starts with normal DMA header, followed by first buffer data.
 *   After it reaches the max size of buffer, the data continues in next DMA descriptor
 *   buffer WITHOUT DMA header
 */
static void * BCMFASTPATH
_dma_rx(_cyhal_gmac_dma_info_t *di)
{
    void *p, *head, *tail;
    uint len;
    uint pkt_len;
    int resid = 0;
#if defined(D11_SPLIT_RX_FD)
    uint tcm_maxsize = 0;        /* max size of tcm descriptor */
#endif

    bool still_processing = true;
    while(still_processing)
    {
        head = _dma_getnextrxp(di, FALSE);
        if (head == NULL)
        {
            return (NULL);
        }

#if (!defined(__mips__) && !defined(BCM47XX_CA9) && !defined(__NetBSD__))
        if (di->hnddma.dmactrlflags & DMA_CTRL_SDIO_RXGLOM)
        {
            /* In case of glommed pkt get length from hwheader */
            len = ltoh16(*((uint16 *)(_CYHAL_GMAC_PKTDATA(head)) + di->rxoffset/2 + 2)) + 4;

            *(uint16 *)(_CYHAL_GMAC_PKTDATA(head)) = (uint16)len;
        }
        else
        {
            len = ltoh16(*(uint16 *)(_CYHAL_GMAC_PKTDATA(head)));
        }
#else
        {
            int read_count = 0;
            /* XXX -PR77137 DMA(Bug) on some chips seems to declare that the
             * packet is ready, but the packet length is not updated
             * (by DMA) by the time we are here
             */
            for (read_count = 200; read_count; read_count--)
            {
                len = ltoh16(*(uint16 *)_CYHAL_GMAC_PKTDATA(head));
                if (len != 0)
                    break;
                GMAC_DMA_MAP(_CYHAL_GMAC_PKTDATA(head), sizeof(uint16), GMAC_DMA_RX, NULL);
                _CYHAL_GMAC_OSL_DELAY(1);
            }

            if (!len)
            {
                _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_rx: frame length (%d)\n", di->name, len));
                _CYHAL_GMAC_PKTFREE(head);
                continue;
            }

        }
#endif /* defined(__mips__) */
        _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rx len %d\n", di->name, len));

        /* set actual length */
        pkt_len = MIN((di->rxoffset + len), di->rxbufsize);

#if defined(D11_SPLIT_RX_FD)
        if (di->sep_rxhdr) {
            /* If separate desc feature is enabled, set length for lcl & host pkt */
            tcm_maxsize = _CYHAL_GMAC_PKTLEN(head);
            /* Dont support multi buffer pkt for now */
            if (pkt_len <= tcm_maxsize)
            {
                /* Full pkt sitting in TCM */
                _CYHAL_GMAC_PKTSETLEN(head, pkt_len);    /* LCL pkt length */
                _CYHAL_GMAC_PKTSETFRAGUSEDLEN(head, 0);    /* Host segment length */
            }
            else
            {
                /* Pkt got split between TCM and host */
                _CYHAL_GMAC_PKTSETLEN(head, tcm_maxsize);    /* LCL pkt length */
                /* use PKTFRAGUSEDLEN to indicate actual length dma ed  by d11 */
                /* Cant use _CYHAL_GMAC_PKTFRAGLEN since if need to reclaim this */
                /* we need fraglen intact */
                _CYHAL_GMAC_PKTSETFRAGUSEDLEN(head, (pkt_len - tcm_maxsize));
            }
        }
        else
#endif /* D11_SPLIT_RX_FD */
        {
            _CYHAL_GMAC_PKTSETLEN(head, pkt_len);
        }

        resid = len - (di->rxbufsize - di->rxoffset);

        if (resid <= 0)
        {
            /* Single frame, all good */
        }
        else if (di->hnddma.dmactrlflags & DMA_CTRL_RXSINGLE)
        {
            _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rx: corrupted length (%d)\n", di->name, len));
            _CYHAL_GMAC_PKTFREE(head);
            di->hnddma.rxgiants++;
            continue;
        }
        else
        {
            /* multi-buffer rx */
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
            /* get rid of compiler warning */
            p = NULL;
#endif /* _CYHAL_ETHERNET_LOG_ENABLE */
            tail = head;
            while ((resid > 0) && (p = _dma_getnextrxp(di, FALSE))) {
                _CYHAL_GMAC_PKTSETNEXT(tail, p);
                pkt_len = MIN(resid, (int)di->rxbufsize);
#if defined(D11_SPLIT_RX_FD)
                /* For split rx case LCL packet length should
                 * not be more than tcm_maxsize.
                 * XXX: multi buffer case is not handled
                 */
                if (di->sep_rxhdr)
                {
                    _CYHAL_GMAC_PKTSETLEN(p, MIN(pkt_len, tcm_maxsize));
                }
                else
#endif /* D11_SPLIT_RX_FD */
                {
                    _CYHAL_GMAC_PKTSETLEN(p, pkt_len);
                }
                tail = p;
                resid -= di->rxbufsize;
            }

#ifdef _CYHAL_ETHERNET_LOG_ENABLE
            if (resid > 0)
            {
                uint16 cur;
                CY_ASSERT(p == NULL);
                cur = (DMA64_ENAB(di) && DMA64_MODE(di)) ?
                    B2I(((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK) - di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t) :
#ifdef BCMDMA32
                    B2I(_CYHAL_GMAC_R_REG(&di->d32rxregs->status) & RS_CD_MASK,dma32dd_t);
#else
                    (0);
#endif
                _CYHAL_ETHERNET_LOG_ERROR(("_dma_rx, rxin %d rxout %d, hw_curr %d\n",
                        di->rxin, di->rxout, cur));
            }
#endif /* _CYHAL_ETHERNET_LOG_ENABLE */

            if ((di->hnddma.dmactrlflags & DMA_CTRL_RXMULTI) == 0)
            {
                _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_rx: bad frame length (%d)\n", di->name, len));
                _CYHAL_GMAC_PKTFREE(head);
                di->hnddma.rxgiants++;
                continue;
            }
        }
        still_processing = false;
    }   /* while(still_processing) */

    return (head);
}

/*
 *  there are cases where teh host mem can't be used when the dongle is in certian
 *  state, so this covers that case
 * XXX: used only for RX FIFO0 with the split rx builds
*/

int
dma_rxfill_suspend(hnddma_t *dmah, bool suspended)
{
    _cyhal_gmac_dma_info_t * di = DI_INFO(dmah);

    di->rxfill_suspended = suspended;
    return 0;
}

/**
 * post receive buffers
 *  return FALSE is refill failed completely and ring is empty
 *  this will stall the rx dma and user might want to call rxfill again asap
 *  This unlikely happens on memory-rich NIC, but often on memory-constrained dongle
 */
static bool BCMFASTPATH
_dma_rxfill(_cyhal_gmac_dma_info_t *di)
{
    void *p;
    uint16 rxin, rxout;
    uint32 flags = 0;
    uint n;
    uint i;
    dmaaddr_t pa;
    uint extra_offset = 0, extra_pad;
    bool ring_empty;
    uint alignment_req = (di->hnddma.dmactrlflags & DMA_CTRL_USB_BOUNDRY4KB_WAR) ?
                16 : 1;    /* MUST BE POWER of 2 */
#if defined(D11_SPLIT_RX_FD)
    uint pktlen;
    dma64addr_t pa64 = {0, 0};
#endif /* D11_SPLIT_RX_FD */

    ring_empty = FALSE;

    /*
     * Determine how many receive buffers we're lacking
     * from the full complement, allocate, initialize,
     * and post them, then update the chip rx lastdscr.
     */

    rxin = di->rxin;
    rxout = di->rxout;

#if defined(D11_SPLIT_RX_FD)
    /* if sep_rxhdr is enabled, for every pkt, two descriptors are programmed */
    /* NRXDACTIVE(rxin, rxout) would show 2 times no of actual full pkts */
    if (di->sep_rxhdr) {
        n = di->nrxpost - (NRXDACTIVE(rxin, rxout) / 2);
    } else
#endif /* D11_SPLIT_RX_FD */
    {
        n = di->nrxpost - NRXDACTIVE(rxin, rxout);
    }

    if (di->rxbufsize > BCMEXTRAHDROOM)
        extra_offset = di->rxextrahdrroom;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxfill: post %d\n", di->name, n));

    for (i = 0; i < n; i++) {
        /* the di->rxbufsize doesn't include the extra headroom, we need to add it to the
           size to be allocated
        */
        if (POOL_ENAB(di->pktpool)) {
            CY_ASSERT(di->pktpool);
            p = pktpool_get(di->pktpool);
#ifdef BCMDBG_POOL
            if (p)
                PKTPOOLSETSTATE(p, POOL_RXFILL);
#endif /* BCMDBG_POOL */
        }
        else {
            p = _CYHAL_GMAC_PKTGET( (di->rxbufsize + extra_offset +  alignment_req - 1) );
        }
        if (p == NULL) {
            _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxfill: out of rxbufs\n", di->name));
            if (i == 0) {
                if (DMA64_ENAB(di) && DMA64_MODE(di)) {
                    if (dma64_rxidle(di)) {
                        _CYHAL_ETHERNET_LOG_INFO(("%s: rxfill64: ring is empty !\n",
                            di->name));
                        ring_empty = TRUE;
                    }
                }
#ifdef BCMDMA32
                else if (DMA32_ENAB(di))
                {
                    if (dma32_rxidle(di))
                    {
                        _CYHAL_ETHERNET_LOG_INFO(("%s: rxfill32: ring is empty !\n",
                            di->name));
                        ring_empty = TRUE;
                    }
                }
#endif
                else
                {
                    CY_ASSERT(0);
                }
            }
            di->hnddma.rxnobuf++;
            break;
        }
        /* reserve an extra headroom, if applicable */
        if (di->hnddma.dmactrlflags & DMA_CTRL_USB_BOUNDRY4KB_WAR) {
            extra_pad = ((alignment_req - (uint)(((unsigned long)_CYHAL_GMAC_PKTDATA(p) -
                (unsigned long)(uchar *)0))) & (alignment_req - 1));
        } else
            extra_pad = 0;

        if (extra_offset + extra_pad)
            _CYHAL_GMAC_PKTPULL(p, extra_offset + extra_pad);

#ifdef CTFMAP
        /* mark as ctf buffer for fast mapping */
        if (CTF_ENAB(kcih)) {
            CY_ASSERT((((uint32)_CYHAL_GMAC_PKTDATA(p)) & 31) == 0);
            PKTSETCTF(p);
        }
#endif /* CTFMAP */

        /* PR3263 & PR3387 & PR4642 war: rxh.len=0 means dma writes not complete */
        /* Do a cached write instead of uncached write since GMAC_DMA_MAP
         * will flush the cache.
        */
        /* Keep it 32 bit-wide to prevent CY_ASSERTs: RB:17293 JIRA:SWWLAN-36682 */
        *(uint32 *)(_CYHAL_GMAC_PKTDATA( p)) = 0;

#if defined(linux) && (defined(BCM47XX_CA9) || defined(__mips__)) && defined(BCM_GMAC3)
        /* Packets tagged as FWDER_BUF are ensured to only have FWDER_PKTMAPSZ
         * (at most) in the cache, with the first 2Bytes dirty in cache.
         */
        if (PKTISFWDERBUF( p)) {
            pa = GMAC_DMA_MAP( _CYHAL_GMAC_PKTDATA( p), FWDER_PKTMAPSZ, GMAC_DMA_TX, NULL);
            PKTCLRFWDERBUF( p);
        } else {
            /* cache flush first 4Byte length */
            GMAC_DMA_MAP( _CYHAL_GMAC_PKTDATA( p), sizeof(uint32), GMAC_DMA_TX, NULL);
            /* cache invalidate entire packet buffer */
            pa = GMAC_DMA_MAP( _CYHAL_GMAC_PKTDATA( p), di->rxbufsize, GMAC_DMA_RX, p);
        }
#else /* !BCM_GMAC3 based FWDER_BUF optimization */

#if defined(linux) && (defined(BCM47XX_CA9) || defined(__mips__))
        GMAC_DMA_MAP( _CYHAL_GMAC_PKTDATA( p), sizeof(uint32), GMAC_DMA_TX, NULL);
#endif
#if defined(SGLIST_RX_SUPPORT)
        if (DMASGLIST_ENAB)
            bzero(&di->rxp_dmah[rxout], sizeof(hnddma_seg_map_t));

        pa = GMAC_DMA_MAP( _CYHAL_GMAC_PKTDATA( p), di->rxbufsize, GMAC_DMA_RX, p);   //,  &di->rxp_dmah[rxout]);

#else  /* !SGLIST_RX_SUPPORT */
        pa = GMAC_DMA_MAP( _CYHAL_GMAC_PKTDATA( p),
                     di->rxbufsize, GMAC_DMA_RX, p, NULL);
#endif /* !SGLIST_RX_SUPPORT */

#endif /* !(linux && (BCM47XX_CA9 || __mips__) && BCM_GMAC3) */

        CY_ASSERT(ISALIGNED(PHYSADDRLO(pa), 4));

        /* save the free packet pointer */
        CY_ASSERT(di->rxp[rxout] == NULL);
        di->rxp[rxout] = p;

#if defined(D11_SPLIT_RX_FD)
        if (!di->sep_rxhdr)
#endif
        {
            /* reset flags for each descriptor */
            flags = 0;
            if (DMA64_ENAB(di) && DMA64_MODE(di)) {
                if (rxout == (di->nrxd - 1))
                    flags = D64_CTRL1_EOT;
                dma64_dd_upd(di, di->rxd64, pa, rxout, &flags, di->rxbufsize);
            }
#ifdef BCMDMA32
            else if (DMA32_ENAB(di))
            {
                if (rxout == (di->nrxd - 1))
                    flags = CTRL_EOT;

                CY_ASSERT(PHYSADDRHI(pa) == 0);
                dma32_dd_upd(di, di->rxd32, pa, rxout, &flags, di->rxbufsize);
            }
#endif
            else
            {
                CY_ASSERT(0);
            }
            rxout = NEXTRXD(rxout);

        }
#if defined(D11_SPLIT_RX_FD)
        else {
            /* TCM Descriptor */
            flags = 0;
            pktlen = _CYHAL_GMAC_PKTLEN( p);
            if (rxout == (di->nrxd - 1))
                flags = D64_CTRL1_EOT;

            /* MAR SOF, so start frame will go to this descriptor */
            flags |= D64_CTRL1_SOF;
            dma64_dd_upd(di, di->rxd64, pa, rxout, &flags, pktlen);
            rxout = NEXTRXD(rxout);        /* update rxout */

            /* Now program host descriptor */
            /* Mark up this descriptor that its a host descriptor */
            /* store 0x80000000 so that dma_rx need'nt process this descriptor */
            di->rxp[rxout] = (void*)PCI64ADDR_HIGH;
            flags = 0;    /* Reset the flags */

            if (rxout == (di->nrxd - 1))
                flags = D64_CTRL1_EOT;

            /* Extract out host length */
            pktlen = _CYHAL_GMAC_PKTFRAGLEN( p, 1);

            /* Extract out host addresses */
            pa64.loaddr = _CYHAL_GMAC_PKTFRAGDATA_LO( p, 1);
            pa64.hiaddr = _CYHAL_GMAC_PKTFRAGDATA_HI( p, 1);

            /* load the descriptor */
            dma64_dd_upd_64(di, di->rxd64, pa64, rxout, &flags, pktlen);
            rxout = NEXTRXD(rxout);    /* update rxout */
        }
#endif /* D11_SPLIT_RX_FD */
    }

#if !defined(BULK_DESCR_FLUSH)
    di->rxout = rxout;
#endif

    /* update the chip lastdscr pointer */
    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
#if defined(BULK_DESCR_FLUSH)
        uint32 flush_cnt = NRXDACTIVE(di->rxout, rxout);
        if (rxout < di->rxout) {
            GMAC_DMA_MAP( dma64_rxd64(di, 0), DMA64_FLUSH_LEN(rxout),
                    GMAC_DMA_TX, NULL, NULL);
            flush_cnt -= rxout;
        }
        GMAC_DMA_MAP( dma64_rxd64(di, di->rxout), DMA64_FLUSH_LEN(flush_cnt),
                GMAC_DMA_TX, NULL, NULL);
#endif /* BULK_DESCR_FLUSH */
        _CYHAL_GMAC_W_REG(&di->d64rxregs->ptr, di->rcvptrbase + I2B(rxout, dma64dd_t));
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        _CYHAL_GMAC_W_REG(&di->d32rxregs->ptr, I2B(rxout, dma32dd_t));
    }
#endif
    else
    {
        CY_ASSERT(0);
    }

#if defined(BULK_DESCR_FLUSH)
    di->rxout = rxout;
#endif

    return ring_empty;
}

/** like getnexttxp but no reclaim */
static void *
_dma_peeknexttxp(_cyhal_gmac_dma_info_t *di)
{
    uint16 end, i;

    if (di->ntxd == 0)
        return (NULL);

    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        end = (uint16)B2I(((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_CD_MASK) -
                   di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);
        di->xs0cd = end;
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        end = (uint16)B2I(_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_CD_MASK, dma32dd_t);
        di->xs0cd = end;
    }
#endif
    else
    {
        CY_ASSERT(0);
    }

    for (i = di->txin; i != end; i = NEXTTXD(i))
        if (di->txp[i])
            return (di->txp[i]);

    return (NULL);
}

int
_dma_peekntxp(_cyhal_gmac_dma_info_t *di, int *len, void *txps[], _cyhal_gmac_txd_range_t range)
{
    uint16 start, end, i;
    uint act;
    void *txp = NULL;
    int k, len_max;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_peekntxp\n", di->name));

    CY_ASSERT(len);
    CY_ASSERT(txps);
    CY_ASSERT(di);
    if (di->ntxd == 0) {
        *len = 0;
        return BCME_ERROR;
    }

    len_max = *len;
    *len = 0;

    start = di->xs0cd;
    if (range == HNDDMA_RANGE_ALL)
        end = di->txout;
    else {
        if (DMA64_ENAB(di)) {
            end = B2I(((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_CD_MASK) -
                di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);

            act = (uint)(_CYHAL_GMAC_R_REG(&di->d64txregs->status1) & D64_XS1_AD_MASK);
            act = (act - di->xmtptrbase) & D64_XS0_CD_MASK;
            act = (uint)B2I(act, dma64dd_t);
        }
#ifdef BCMDMA32
        else
        {
            end = B2I(_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_CD_MASK, dma32dd_t);

            act = (uint)((_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_AD_MASK) >>
                XS_AD_SHIFT);
            act = (uint)B2I(act, dma32dd_t);
        }
#endif

        di->xs0cd = end;
        if (end != act)
            end = PREVTXD(act);
    }

    /* PR4738 - xmt disable/re-enable does not clear CURR */
    if ((start == 0) && (end > di->txout))
        return BCME_ERROR;

    k = 0;
    for (i = start; i != end; i = NEXTTXD(i)) {
        txp = di->txp[i];
        if (txp != NULL) {
            if (k < len_max)
                txps[k++] = txp;
            else
                break;
        }
    }
    *len = k;

    return BCME_OK;
}

/** like getnextrxp but not take off the ring */
static void *
_dma_peeknextrxp(_cyhal_gmac_dma_info_t *di)
{
    uint16 end, i;

    if (di->nrxd == 0)
        return (NULL);

    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
#ifndef CORRECTION_TO_CURR_RX_DESCR_INDEX
        end = (uint16)B2I(((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK) - di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
#else
        end = _dma_get_current_buffer_index(di);
#endif
        di->rs0cd = end;
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        end = (uint16)B2I(_CYHAL_GMAC_R_REG(&di->d32rxregs->status) & RS_CD_MASK, dma32dd_t);
        di->rs0cd = end;
    }
#endif
    else
    {
        CY_ASSERT(0);
    }

    for (i = di->rxin; i != end; i = NEXTRXD(i))
        if (di->rxp[i])
            return (di->rxp[i]);

    return (NULL);
}

static void
_dma_rxreclaim(_cyhal_gmac_dma_info_t *di)
{
    void *p;
    bool origcb = TRUE;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxreclaim\n", di->name));

    if (POOL_ENAB(di->pktpool) &&
        ((origcb = pktpool_emptycb_disabled(di->pktpool)) == FALSE))
        pktpool_emptycb_disable(di->pktpool, TRUE);

    while ((p = _dma_getnextrxp(di, TRUE))) {
        /* For unframed data, we don't have any packets to free */
#if (defined(__mips__) || defined(BCM47XX_CA9) || defined(BCM_WICED)) && !defined(_CFE_)
        if (!(di->hnddma.dmactrlflags & DMA_CTRL_UNFRAMED))
#endif /* (__mips__ || BCM47XX_CA9) && !_CFE_ */
            _CYHAL_GMAC_PKTFREE(p);
    }

//    if (origcb == FALSE)
//        pktpool_emptycb_disable(di->pktpool, FALSE);
}

/** returns entries on the ring, in the order in which they were placed on the ring */
static void * BCMFASTPATH
_dma_getnextrxp(_cyhal_gmac_dma_info_t *di, bool forceall)
{
    if (di->nrxd == 0)
        return (NULL);

    if (DMA64_ENAB(di) && DMA64_MODE(di))
    {
        return dma64_getnextrxp(di, forceall);
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        return dma32_getnextrxp(di, forceall);
    }
#endif
    else
    {
        CY_ASSERT(0);
    }
}

static void
_dma_txblock(_cyhal_gmac_dma_info_t *di)
{
    di->hnddma.txavail = 0;
}

static void
_dma_txunblock(_cyhal_gmac_dma_info_t *di)
{
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;
}

static uint
_dma_txactive(_cyhal_gmac_dma_info_t *di)
{
    return NTXDACTIVE(di->txin, di->txout);
}

static uint
_dma_txpending(_cyhal_gmac_dma_info_t *di)
{
    uint16 curr;

    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        curr = B2I(((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_CD_MASK) -
                   di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);
        di->xs0cd = curr;
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        curr = B2I(_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_CD_MASK, dma32dd_t);
        di->xs0cd = curr;
    }
#endif
    else
    {
        CY_ASSERT(0);
    }

    return NTXDACTIVE(curr, di->txout);
}

static uint
_dma_txcommitted(_cyhal_gmac_dma_info_t *di)
{
    uint16 ptr;
    uint txin = di->txin;

    if (txin == di->txout)
        return 0;

    if (DMA64_ENAB(di) && DMA64_MODE(di)) {
        ptr = B2I(_CYHAL_GMAC_R_REG(&di->d64txregs->ptr), dma64dd_t);
    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        ptr = B2I(_CYHAL_GMAC_R_REG(&di->d32txregs->ptr), dma32dd_t);
    }
#endif
    else
    {
        CY_ASSERT(0);
    }

    return NTXDACTIVE(di->txin, ptr);
}

static uint
_dma_rxactive(_cyhal_gmac_dma_info_t *di)
{
    return NRXDACTIVE(di->rxin, di->rxout);
}

static uint
_dma_activerxbuf(_cyhal_gmac_dma_info_t *di)
{
    uint16 curr, ptr;
#ifndef CORRECTION_TO_CURR_RX_DESCR_INDEX
    curr = B2I(((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK) - di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
    ptr =  B2I(((_CYHAL_GMAC_R_REG(&di->d64rxregs->ptr) & D64_RS0_CD_MASK) - di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);

#else
    curr = _dma_get_current_buffer_index(di);
    ptr = _dma_get_last_buffer_index(di);
#endif

    return NRXDACTIVE(curr, ptr);
}


static void
_dma_counterreset(_cyhal_gmac_dma_info_t *di)
{
    /* reset all software counter */
    di->hnddma.rxgiants = 0;
    di->hnddma.rxnobuf = 0;
    di->hnddma.txnobuf = 0;
}

static uint
_dma_ctrlflags(_cyhal_gmac_dma_info_t *di, uint mask, uint flags)
{
    uint dmactrlflags;

    if (!di) {
        _CYHAL_ETHERNET_LOG_ERROR(("_dma_ctrlflags: NULL dma handle\n"));
        return (0);
    }

    dmactrlflags = di->hnddma.dmactrlflags;
    CY_ASSERT((flags & ~mask) == 0);

    dmactrlflags &= ~mask;
    dmactrlflags |= flags;

    /* If trying to enable parity, check if parity is actually supported */
    if (dmactrlflags & DMA_CTRL_PEN) {
        uint32 control;

        if (DMA64_ENAB(di) && DMA64_MODE(di)) {
            control = _CYHAL_GMAC_R_REG(&di->d64txregs->control);
            _CYHAL_GMAC_W_REG(&di->d64txregs->control, control | D64_XC_PD);
            if (_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_PD) {
                /* We *can* disable it so it is supported,
                 * restore control register
                 */
                _CYHAL_GMAC_W_REG(&di->d64txregs->control, control);
            } else {
                /* Not supported, don't allow it to be enabled */
                dmactrlflags &= ~DMA_CTRL_PEN;
            }
        }
#ifdef BCMDMA32
        else if (DMA32_ENAB(di))
        {
            control = _CYHAL_GMAC_R_REG(&di->d32txregs->control);
            _CYHAL_GMAC_W_REG(&di->d32txregs->control, control | XC_PD);
            if (_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_PD) {
                _CYHAL_GMAC_W_REG(&di->d32txregs->control, control);
            } else {
                /* Not supported, don't allow it to be enabled */
                dmactrlflags &= ~DMA_CTRL_PEN;
            }
        }
#endif
        else
        {
            CY_ASSERT(0);
        }
    }

    di->hnddma.dmactrlflags = dmactrlflags;

    return (dmactrlflags);
}

/** get the address of the var in order to change later */
static uintptr
_dma_getvar(_cyhal_gmac_dma_info_t *di, const char *name)
{
    if (!strcmp(name, "&txavail"))
        return ((uintptr) &(di->hnddma.txavail));
    else if (!strcmp(name, "&rxavail"))
        return ((uintptr) &(di->rxavail));
    else {
        CY_ASSERT(0);
    }
    return (0);
}

static uint
_dma_avoidancecnt(_cyhal_gmac_dma_info_t *di)
{
    return (di->dma_avoidance_cnt);
}

void
dma_txpioloopback(dma32regs_t *regs)
{
    _CYHAL_GMAC_OR_REG(&regs->control, XC_LE);
}

static
uint8 BCMATTACHFN_DMA_ATTACH(dma_align_sizetobits)(uint size)
{
    uint8 bitpos = 0;
    CY_ASSERT(size);
    CY_ASSERT(!(size & (size-1)));
    while (size >>= 1) {
        bitpos ++;
    }
    return (bitpos);
}

/**
 * This function ensures that the DMA descriptor ring will not get allocated
 * across Page boundary. If the allocation is done across the page boundary
 * at the first time, then it is freed and the allocation is done at
 * descriptor ring size aligned location. This will ensure that the ring will
 * not cross page boundary
 */
static void *
BCMATTACHFN_DMA_ATTACH(dma_ringalloc)(uint32 boundary, uint size, uint16 *alignbits,
    uint* alloced, dmaaddr_t *descpa)
{
    void * va;
    uint32 desc_strtaddr;
    uint32 alignbytes = 1 << *alignbits;

    if ((va = GMAC_DMA_ALLOC_CONSISTENT(size, *alignbits, alloced, descpa)) == NULL)
        return NULL;

    desc_strtaddr = (uint32)ROUNDUP((uint)PHYSADDRLO(*descpa), alignbytes);
    if (((desc_strtaddr + size - 1) & boundary) !=
        (desc_strtaddr & boundary)) {
        *alignbits = dma_align_sizetobits(size);
        GMAC_DMA_FREE_CONSISTENT(va, size, *descpa);
        va = GMAC_DMA_ALLOC_CONSISTENT(size, *alignbits, alloced, descpa);
    }
    return va;
}

#if defined(CY_ETHERNET_LOG_ENABLE) || defined(BCMDBG_DUMP)
#ifdef BCMDMA32
static void
dma32_dumpring(_cyhal_gmac_dma_info_t *di, dma32dd_t *ring, uint start, uint end,
    uint max_num)
{
    uint i;

    for (i = start; i != end; i = XXD((i + 1), max_num)) {
        /* in the format of high->low 8 bytes */
        _CYHAL_ETHERNET_LOG_INFO(("ring index %d: 0x%x %x\n",
            i, R_SM(&ring[i].addr), R_SM(&ring[i].ctrl)));
    }
}

static void
dma32_dumptx(_cyhal_gmac_dma_info_t *di, bool dumpring)
{
    if (di->ntxd == 0)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("DMA32: txd32 %p txdpa 0x%lx txp %p txin %d txout %d "
                "txavail %d txnodesc %d\n", di->txd32, PHYSADDRLO(di->txdpa), di->txp, di->txin,
                di->txout, di->hnddma.txavail, di->hnddma.txnodesc));

    _CYHAL_ETHERNET_LOG_INFO(("xmtcontrol 0x%x xmtaddr 0x%x xmtptr 0x%x xmtstatus 0x%x\n",
        _CYHAL_GMAC_R_REG(&di->d32txregs->control),
        _CYHAL_GMAC_R_REG(&di->d32txregs->addr),
        _CYHAL_GMAC_R_REG(&di->d32txregs->ptr),
        _CYHAL_GMAC_R_REG(&di->d32txregs->status)));

    if (dumpring && di->txd32)
        dma32_dumpring(di, b, di->txd32, di->txin, di->txout, di->ntxd);
}

static void
dma32_dumprx(_cyhal_gmac_dma_info_t *di, bool dumpring)
{
    if (di->nrxd == 0)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("DMA32: rxd32 %p rxdpa 0x%lx rxp %p rxin %d rxout %d\n",
                di->rxd32, PHYSADDRLO(di->rxdpa), di->rxp, di->rxin, di->rxout));

    _CYHAL_ETHERNET_LOG_INFO(("rcvcontrol 0x%x rcvaddr 0x%x rcvptr 0x%x rcvstatus 0x%x\n",
        _CYHAL_GMAC_R_REG(&di->d32rxregs->control),
        _CYHAL_GMAC_R_REG(&di->d32rxregs->addr),
        _CYHAL_GMAC_R_REG(&di->d32rxregs->ptr),
        _CYHAL_GMAC_R_REG(&di->d32rxregs->status)));
    if (di->rxd32 && dumpring)
        dma32_dumpring(di, b, di->rxd32, di->rxin, di->rxout, di->nrxd);
}

static void
dma32_dump(_cyhal_gmac_dma_info_t *di, bool dumpring)
{
    dma32_dumptx(di, b, dumpring);
    dma32_dumprx(di, b, dumpring);
}
#endif  // 32-bit code

static void
dma64_dumpring(_cyhal_gmac_dma_info_t *di, dma64dd_t *ring, uint start, uint end,
    uint max_num)
{
    uint i;

    for (i = start; i != end; i = XXD((i + 1), max_num)) {
        /* in the format of high->low 16 bytes */
        _CYHAL_ETHERNET_LOG_INFO(("ring index %d: 0x%x %x %x %x\n",
            i, R_SM(&ring[i].addrhigh), R_SM(&ring[i].addrlow),
            R_SM(&ring[i].ctrl2), R_SM(&ring[i].ctrl1)));
    }
}

static void
dma64_dumptx(_cyhal_gmac_dma_info_t *di, bool dumpring)
{
    if (di->ntxd == 0)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("DMA64: txd64 %p txdpa 0x%lx txdpahi 0x%lx txp %p txin %d txout %d "
                "txavail %d txnodesc %d\n", di->txd64, PHYSADDRLO(di->txdpa),
                PHYSADDRHI(di->txdpaorig), di->txp, di->txin, di->txout, di->hnddma.txavail,
                di->hnddma.txnodesc));

    _CYHAL_ETHERNET_LOG_INFO(("xmtcontrol 0x%x xmtaddrlow 0x%x xmtaddrhigh 0x%x "
               "xmtptr 0x%x xmtstatus0 0x%x xmtstatus1 0x%x\n",
               _CYHAL_GMAC_R_REG(&di->d64txregs->control),
               _CYHAL_GMAC_R_REG(&di->d64txregs->addrlow),
               _CYHAL_GMAC_R_REG(&di->d64txregs->addrhigh),
               _CYHAL_GMAC_R_REG(&di->d64txregs->ptr),
               _CYHAL_GMAC_R_REG(&di->d64txregs->status0),
               _CYHAL_GMAC_R_REG(&di->d64txregs->status1)));

    _CYHAL_ETHERNET_LOG_INFO(("DMA64: DMA avoidance applied %d\n", di->dma_avoidance_cnt));

    if (dumpring && di->txd64) {
        dma64_dumpring(di, b, di->txd64, di->txin, di->txout, di->ntxd);
    }
}

static void
dma64_dumprx(_cyhal_gmac_dma_info_t *di, bool dumpring)
{
    if (di->nrxd == 0)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("DMA64: rxd64 %p rxdpa 0x%lx rxdpahi 0x%lx rxp %p rxin %d rxout %d\n",
                di->rxd64, PHYSADDRLO(di->rxdpa), PHYSADDRHI(di->rxdpaorig), di->rxp,
                di->rxin, di->rxout));

    _CYHAL_ETHERNET_LOG_INFO(("rcvcontrol 0x%x rcvaddrlow 0x%x rcvaddrhigh 0x%x rcvptr "
               "0x%x rcvstatus0 0x%x rcvstatus1 0x%x\n",
               _CYHAL_GMAC_R_REG(&di->d64rxregs->control),
               _CYHAL_GMAC_R_REG(&di->d64rxregs->addrlow),
               _CYHAL_GMAC_R_REG(&di->d64rxregs->addrhigh),
               _CYHAL_GMAC_R_REG(&di->d64rxregs->ptr),
               _CYHAL_GMAC_R_REG(&di->d64rxregs->status0),
               _CYHAL_GMAC_R_REG(&di->d64rxregs->status1)));
    if (di->rxd64 && dumpring) {
        dma64_dumpring(di, b, di->rxd64, di->rxin, di->rxout, di->nrxd);
    }
}

static void
dma64_dump(_cyhal_gmac_dma_info_t *di, bool dumpring)
{
    dma64_dumptx(di, b, dumpring);
    dma64_dumprx(di, b, dumpring);
}

#endif    /* _CYHAL_ETHERNET_LOG_ENABLE || BCMDBG_DUMP */


#ifdef BCMDMA32
/* 32-bit DMA functions */
static void
dma32_txinit(_cyhal_gmac_dma_info_t *di)
{
    uint32 control = XC_XE;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma32_txinit\n", di->name));

    if (di->ntxd == 0)
        return;

    di->txin = di->txout = di->xs0cd = 0;
    di->hnddma.txavail = di->ntxd - 1;

    /* clear tx descriptor ring */
    BZERO_SM(DISCARD_QUAL(di->txd32, void), (di->ntxd * sizeof(dma32dd_t)));

    /* These bits 20:18 (burstLen) of control register can be written but will take
     * effect only if these bits are valid. So this will not affect previous versions
     * of the DMA. They will continue to have those bits set to 0.
     */
    control |= (di->txburstlen << XC_BL_SHIFT);
    control |= (di->txmultioutstdrd << XC_MR_SHIFT);
    control |= (di->txprefetchctl << XC_PC_SHIFT);
    control |= (di->txprefetchthresh << XC_PT_SHIFT);

    if ((di->hnddma.dmactrlflags & DMA_CTRL_PEN) == 0)
        control |= XC_PD;
    _CYHAL_GMAC_W_REG(&di->d32txregs->control, control);
    _dma_ddtable_init(di, GMAC_DMA_TX, di->txdpa);
}

static bool
dma32_txenabled(_cyhal_gmac_dma_info_t *di)
{
    uint32 xc;

    /* If the chip is dead, it is not enabled :-) */
    xc = _CYHAL_GMAC_R_REG(&di->d32txregs->control);
    return ((xc != 0xffffffff) && (xc & XC_XE));
}

static void
dma32_txsuspend(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txsuspend\n", di->name));

    if (di->ntxd == 0)
        return;

    _CYHAL_GMAC_OR_REG(&di->d32txregs->control, XC_SE);
}

static void
dma32_txresume(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txresume\n", di->name));

    if (di->ntxd == 0)
        return;

    _CYHAL_GMAC_AND_REG(&di->d32txregs->control, ~XC_SE);
}

static bool
dma32_txsuspended(_cyhal_gmac_dma_info_t *di)
{
    return (di->ntxd == 0) || ((_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_SE) == XC_SE);
}

#ifdef WL_MULTIQUEUE
static void
dma32_txflush(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txflush\n", di->name));

    if (di->ntxd == 0)
        return;

    _CYHAL_GMAC_OR_REG(&di->d32txregs->control, XC_SE | XC_FL);
}

static void
dma32_txflush_clear(_cyhal_gmac_dma_info_t *di)
{
    uint32 status;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txflush_clear\n", di->name));

    if (di->ntxd == 0)
        return;

    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_XS_MASK))
         != XS_XS_DISABLED) &&
         (status != XS_XS_IDLE) &&
         (status != XS_XS_STOPPED),
         (10000));
    _CYHAL_GMAC_AND_REG(&di->d32txregs->control, ~XC_FL);
}
#endif /* WL_MULTIQUEUE */

static void
dma32_txreclaim(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range)
{
    void *p;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txreclaim %s\n", di->name,
               (range == HNDDMA_RANGE_ALL) ? "all" :
               ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered")));

    if (di->txin == di->txout)
        return;

    while ((p = dma32_getnexttxp(di, range)))
        _CYHAL_GMAC_PKTFREE(p);
}

static bool
dma32_txstopped(_cyhal_gmac_dma_info_t *di)
{
    return ((_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_XS_MASK) == XS_XS_STOPPED);
}

static bool
dma32_rxstopped(_cyhal_gmac_dma_info_t *di)
{
    return ((_CYHAL_GMAC_R_REG(&di->d32rxregs->status) & RS_RS_MASK) == RS_RS_STOPPED);
}

static bool
BCMATTACHFN_DMA_ATTACH(dma32_alloc)(_cyhal_gmac_dma_info_t *di, uint direction)
{
    uint size;
    uint ddlen;
    void *va;
    uint alloced;
    uint16 align;
    uint16 align_bits;

    ddlen = sizeof(dma32dd_t);

    size = (direction == GMAC_DMA_TX) ? (di->ntxd * ddlen) : (di->nrxd * ddlen);

    alloced = 0;
    align_bits = di->dmadesc_align;
    align = (1 << align_bits);

    if (direction == GMAC_DMA_TX) {
        if ((va = dma_ringalloc( D32RINGALIGN, size, &align_bits, &alloced,
            &di->txdpaorig, &di->tx_dmah)) == NULL) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_alloc: GMAC_DMA_ALLOC_CONSISTENT(ntxd) failed\n",
                       di->name));
            return FALSE;
        }

        PHYSADDRHISET(di->txdpa, 0);
        CY_ASSERT(PHYSADDRHI(di->txdpaorig) == 0);
        di->txd32 = (dma32dd_t *)ROUNDUP((uintptr)va, align);
        di->txdalign = (uint)((int8 *)(uintptr)di->txd32 - (int8 *)va);

        PHYSADDRLOSET(di->txdpa, PHYSADDRLO(di->txdpaorig) + di->txdalign);
        /* Make sure that alignment didn't overflow */
        CY_ASSERT(PHYSADDRLO(di->txdpa) >= PHYSADDRLO(di->txdpaorig));

        di->txdalloc = alloced;
        CY_ASSERT(ISALIGNED(di->txd32, align));
    } else {
        if ((va = dma_ringalloc( D32RINGALIGN, size, &align_bits, &alloced,
            &di->rxdpaorig, &di->rx_dmah)) == NULL) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_alloc: GMAC_DMA_ALLOC_CONSISTENT(nrxd) failed\n",
                       di->name));
            return FALSE;
        }

        PHYSADDRHISET(di->rxdpa, 0);
        CY_ASSERT(PHYSADDRHI(di->rxdpaorig) == 0);
        di->rxd32 = (dma32dd_t *)ROUNDUP((uintptr)va, align);
        di->rxdalign = (uint)((int8 *)(uintptr)di->rxd32 - (int8 *)va);

        PHYSADDRLOSET(di->rxdpa, PHYSADDRLO(di->rxdpaorig) + di->rxdalign);
        /* Make sure that alignment didn't overflow */
        CY_ASSERT(PHYSADDRLO(di->rxdpa) >= PHYSADDRLO(di->rxdpaorig));
        di->rxdalloc = alloced;
        CY_ASSERT(ISALIGNED(di->rxd32, align));
    }

    return TRUE;
}

/**
 * PR2414 WAR: When the DMA channel is in the FetchDescriptor state,
 * it does not notice that the enable bit has been turned off. If the
 * enable bit is turned back on before the descriptor fetch completes,
 * at least some of the DMA channel does not get reset. In particular,
 * it will fetch a descriptor from the address it was trying to fetch
 * from when it was disabled.
 *
 * For all cores other than USB, the workaround is simply to clear the
 * enable bit, and then read back status until the state shows up as
 * Disabled before re-enabling the channel.
 */
static bool
dma32_txreset(_cyhal_gmac_dma_info_t *di)
{
    uint32 status;

    if (di->ntxd == 0)
        return TRUE;

    /* address PR8249/PR7577 issue */
    /* suspend tx DMA first */
    _CYHAL_GMAC_W_REG(&di->d32txregs->control, XC_SE);
    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_XS_MASK))
         != XS_XS_DISABLED) &&
         (status != XS_XS_IDLE) &&
         (status != XS_XS_STOPPED),
         (10000));

    /* PR2414 WAR: DMA engines are not disabled until transfer finishes */
    _CYHAL_GMAC_W_REG(&di->d32txregs->control, 0);
    SPINWAIT(((status = (_CYHAL_GMAC_R_REG( &di->d32txregs->status) & XS_XS_MASK)) != XS_XS_DISABLED),
             10000);

    /* We should be disabled at this point */
    if (status != XS_XS_DISABLED) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: status != D64_XS0_XS_DISABLED 0x%x\n", __FUNCTION__, status));
        CY_ASSERT(status == XS_XS_DISABLED);
        _CYHAL_GMAC_OSL_DELAY(300);
    }

    return (status == XS_XS_DISABLED);
}

static bool
dma32_rxidle(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxidle\n", di->name));

    if (di->nrxd == 0)
        return TRUE;

    return ((_CYHAL_GMAC_R_REG(&di->d32rxregs->status) & RS_CD_MASK) ==
            _CYHAL_GMAC_R_REG(&di->d32rxregs->ptr));
}

static bool
dma32_rxreset(_cyhal_gmac_dma_info_t *di)
{
    uint32 status;

    if (di->nrxd == 0)
        return TRUE;

    /* PR2414 WAR: DMA engines are not disabled until transfer finishes */
    _CYHAL_GMAC_W_REG(&di->d32rxregs->control, 0);
    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d32rxregs->status) & RS_RS_MASK)) != RS_RS_DISABLED),
             10000);

    return (status == RS_RS_DISABLED);
}

static bool
dma32_rxenabled(_cyhal_gmac_dma_info_t *di)
{
    uint32 rc;

    rc = _CYHAL_GMAC_R_REG(&di->d32rxregs->control);
    return ((rc != 0xffffffff) && (rc & RC_RE));
}

static bool
dma32_txsuspendedidle(_cyhal_gmac_dma_info_t *di)
{
    if (di->ntxd == 0)
        return TRUE;

    if (!(_CYHAL_GMAC_R_REG(&di->d32txregs->control) & XC_SE))
        return 0;

    if ((_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_XS_MASK) != XS_XS_IDLE)
        return 0;

    /*
     * PR20059 WAR(tested on d11mac only):
     *   when the driver sees the dma status as "idle" it waits for
     * a small amount of time, say a couple of us and then retests the status.
     * If it's still "idle" then the dma has indeed suspended, if not then wait
     * again for "idle", and so on (The false case is handled outside here).
     */
    _CYHAL_GMAC_OSL_DELAY(2);
    return ((_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_XS_MASK) == XS_XS_IDLE);
}

/**
 * !! tx entry routine
 * supports full 32bit dma engine buffer addressing so
 * dma buffers can cross 4 Kbyte page boundaries.
 *
 * WARNING: call must check the return value for error.
 *   the error(toss frames) could be fatal and cause many subsequent hard to debug problems
 */
static int
dma32_txfast(_cyhal_gmac_dma_info_t *di, void *p0, bool commit)
{
    void *p, *next;
    uchar *data;
    uint len;
    uint16 txout;
    uint32 flags = 0;
    dmaaddr_t pa;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txfast\n", di->name));

    txout = di->txout;

    /*
     * Walk the chain of packet buffers
     * allocating and initializing transmit descriptor entries.
     */
    for (p = p0; p; p = next) {
        uint nsegs, j;
        hnddma_seg_map_t *map;

        data = _CYHAL_GMAC_PKTDATA( p);
        len = _CYHAL_GMAC_PKTLEN( p);
#ifdef BCM_DMAPAD
        if (DMAPADREQUIRED(di)) {
            len += PKTDMAPAD( p);
        }
#endif
        next = _CYHAL_GMAC_PKTNEXT( p);

        /* return nonzero if out of tx descriptors */
        if (NEXTTXD(txout) == di->txin)
            goto outoftxd;

        /* PR988 - skip zero length buffers */
        if (len == 0)
            continue;

        if (DMASGLIST_ENAB)
            bzero(&di->txp_dmah[txout], sizeof(hnddma_seg_map_t));

        /* get physical address of buffer start */
        pa = GMAC_DMA_MAP( data, len, GMAC_DMA_TX, p, &di->txp_dmah[txout]);

        if (DMASGLIST_ENAB) {
            map = &di->txp_dmah[txout];

            /* See if all the segments can be accounted for */
            if (map->nsegs > (uint)(di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1))
                goto outoftxd;

            nsegs = map->nsegs;
        } else
            nsegs = 1;

        for (j = 1; j <= nsegs; j++) {
            flags = 0;
            if (p == p0 && j == 1)
                flags |= CTRL_SOF;

            /* With a DMA segment list, Descriptor table is filled
             * using the segment list instead of looping over
             * buffers in multi-chain DMA. Therefore, EOF for SGLIST is when
             * end of segment list is reached.
             */
            if ((!DMASGLIST_ENAB && next == NULL) ||
                (DMASGLIST_ENAB && j == nsegs))
                flags |= (CTRL_IOC | CTRL_EOF);
            if (txout == (di->ntxd - 1))
                flags |= CTRL_EOT;

            if (DMASGLIST_ENAB) {
                len = map->segs[j - 1].length;
                pa = map->segs[j - 1].addr;
            }
            CY_ASSERT(PHYSADDRHI(pa) == 0);

            dma32_dd_upd(di, di->txd32, pa, txout, &flags, len);
            CY_ASSERT(di->txp[txout] == NULL);

            txout = NEXTTXD(txout);
        }

        /* See above. No need to loop over individual buffers */
        if (DMASGLIST_ENAB)
            break;
    }

    /* if last txd eof not set, fix it */
    if (!(flags & CTRL_EOF))
        W_SM(&di->txd32[PREVTXD(txout)].ctrl, _CYHAL_BUS_SWAP32(flags | CTRL_IOC | CTRL_EOF));

    /* save the packet */
    di->txp[PREVTXD(txout)] = p0;

    /* bump the tx descriptor index */
    di->txout = txout;

    /* kick the chip */
    if (commit)
        _CYHAL_GMAC_W_REG(&di->d32txregs->ptr, I2B(txout, dma32dd_t));

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (0);

outoftxd:
    _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: out of txds\n", di->name));
//    _CYHAL_GMAC_PKTFREE(p0);  // We do not alloc Tx buffers
    di->hnddma.txavail = 0;
    di->hnddma.txnobuf++;
    di->hnddma.txnodesc++;
    return (-1);
}

/**
 * Reclaim next completed txd (txds if using chained buffers) in the range
 * specified and return associated packet.
 * If range is HNDDMA_RANGE_TRANSMITTED, reclaim descriptors that have be
 * transmitted as noted by the hardware "CurrDescr" pointer.
 * If range is HNDDMA_RANGE_TRANSFERED, reclaim descriptors that have be
 * transfered by the DMA as noted by the hardware "ActiveDescr" pointer.
 * If range is HNDDMA_RANGE_ALL, reclaim all txd(s) posted to the ring and
 * return associated packet regardless of the value of hardware pointers.
 */
static void *
dma32_getnexttxp(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range)
{
    uint16 start, end, i;
    uint16 active_desc;
    void *txp;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_getnexttxp %s\n", di->name,
               (range == HNDDMA_RANGE_ALL) ? "all" :
               ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered")));

    if (di->ntxd == 0)
        return (NULL);

    txp = NULL;

    start = di->txin;
    if (range == HNDDMA_RANGE_ALL)
        end = di->txout;
    else {
        dma32regs_t *dregs = di->d32txregs;

        if (di->txin == di->xs0cd) {
        end = (uint16)B2I(_CYHAL_GMAC_R_REG(&dregs->status) & XS_CD_MASK, dma32dd_t);
            di->xs0cd = end;
        } else
            end = di->xs0cd;

        if (range == HNDDMA_RANGE_TRANSFERED) {
            active_desc = (uint16)((_CYHAL_GMAC_R_REG(&dregs->status) & XS_AD_MASK) >>
                                   XS_AD_SHIFT);
            active_desc = (uint16)B2I(active_desc, dma32dd_t);
            if (end != active_desc)
                end = PREVTXD(active_desc);
        }
    }

    /* PR4738 - xmt disable/re-enable does not clear CURR */
    if ((start == 0) && (end > di->txout))
        goto bogus;

    for (i = start; i != end && !txp; i = NEXTTXD(i)) {
        dmaaddr_t pa;
        hnddma_seg_map_t *map = NULL;
        uint size, j, nsegs;

        PHYSADDRLOSET(pa, (_CYHAL_BUS_SWAP32(R_SM(&di->txd32[i].addr)) - di->dataoffsetlow));
        PHYSADDRHISET(pa, 0);

        if (DMASGLIST_ENAB) {
            map = &di->txp_dmah[i];
            size = map->origsize;
            nsegs = map->nsegs;
        } else {
            size = (_CYHAL_BUS_SWAP32(R_SM(&di->txd32[i].ctrl)) & CTRL_BC_MASK);
            nsegs = 1;
        }

        for (j = nsegs; j > 0; j--) {
            W_SM(&di->txd32[i].addr, 0xdeadbeef);

            txp = di->txp[i];
            di->txp[i] = NULL;
            if (j > 1)
                i = NEXTTXD(i);
        }

        GMAC_DMA_UNMAP( pa, size, GMAC_DMA_TX, txp, map);
    }

    di->txin = i;

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (txp);

bogus:
    _CYHAL_ETHERNET_LOG_DEBUG(("dma_getnexttxp: bogus curr: start %d end %d txout %d force %d\n",
              start, end, di->txout, forceall));
    return (NULL);
}

static void *
dma32_getnextrxp(_cyhal_gmac_dma_info_t *di, bool forceall)
{
    uint16 i, curr;
    void *rxp;
    dmaaddr_t pa;
    /* if forcing, dma engine must be disabled */
    CY_ASSERT(!forceall || !dma32_rxenabled(di));

    i = di->rxin;

    /* return if no packets posted */
    if (i == di->rxout)
        return (NULL);

    if (di->rxin == di->rs0cd) {
        curr = (uint16)B2I(_CYHAL_GMAC_R_REG(&di->d32rxregs->status) & RS_CD_MASK, dma32dd_t);
        di->rs0cd = curr;
    } else
        curr = di->rs0cd;

    /* ignore curr if forceall */
    if (!forceall && (i == curr))
        return (NULL);

    /* get the packet pointer that corresponds to the rx descriptor */
    rxp = di->rxp[i];
    CY_ASSERT(rxp);
    di->rxp[i] = NULL;

    PHYSADDRLOSET(pa, (_CYHAL_BUS_SWAP32(R_SM(&di->rxd32[i].addr)) - di->dataoffsetlow));
    PHYSADDRHISET(pa, 0);

    /* clear this packet from the descriptor ring */
    GMAC_DMA_UNMAP( pa,
              di->rxbufsize, GMAC_DMA_RX, rxp, &di->rxp_dmah[i]);

    W_SM(&di->rxd32[i].addr, 0xdeadbeef);

    di->rxin = NEXTRXD(i);

    return (rxp);
}

/**
 * Rotate all active tx dma ring entries "forward" by (ActiveDescriptor - txin).
 */
static void
dma32_txrotate(_cyhal_gmac_dma_info_t *di)
{
    uint16 ad;
    uint nactive;
    uint rot;
    uint16 old, new;
    uint32 w;
    uint16 first, last;

    CY_ASSERT(dma32_txsuspendedidle(di));

    nactive = _dma_txactive(di);
    ad = B2I(((_CYHAL_GMAC_R_REG(&di->d32txregs->status) & XS_AD_MASK) >> XS_AD_SHIFT), dma32dd_t);
    rot = TXD(ad - di->txin);

    CY_ASSERT(rot < di->ntxd);

    /* full-ring case is a lot harder - don't worry about this */
    if (rot >= (di->ntxd - nactive)) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txrotate: ring full - punt\n", di->name));
        return;
    }

    first = di->txin;
    last = PREVTXD(di->txout);

    /* move entries starting at last and moving backwards to first */
    for (old = last; old != PREVTXD(first); old = PREVTXD(old)) {
        new = TXD(old + rot);

        /*
         * Move the tx dma descriptor.
         * EOT is set only in the last entry in the ring.
         */
        w = _CYHAL_BUS_SWAP32(R_SM(&di->txd32[old].ctrl)) & ~CTRL_EOT;
        if (new == (di->ntxd - 1))
            w |= CTRL_EOT;
        W_SM(&di->txd32[new].ctrl, _CYHAL_BUS_SWAP32(w));
        W_SM(&di->txd32[new].addr, R_SM(&di->txd32[old].addr));

        /* zap the old tx dma descriptor address field */
        W_SM(&di->txd32[old].addr, _CYHAL_BUS_SWAP32(0xdeadbeef));

        /* move the corresponding txp[] entry */
        CY_ASSERT(di->txp[new] == NULL);
        di->txp[new] = di->txp[old];

        /* Move the segment map as well */
        if (DMASGLIST_ENAB) {
            memcpy(&di->txp_dmah[old], &di->txp_dmah[new], sizeof(hnddma_seg_map_t));
            memcpy(&di->txp_dmah[old], sizeof(hnddma_seg_map_t));
        }

        di->txp[old] = NULL;
    }

    /* update txin and txout */
    di->txin = ad;
    di->txout = TXD(di->txout + rot);
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    /* kick the chip */
    _CYHAL_GMAC_W_REG(&di->d32txregs->ptr, I2B(di->txout, dma32dd_t));
}
#endif /* BCMDMA32 */


/* 64-bit DMA functions */

static void
dma64_txinit(_cyhal_gmac_dma_info_t *di)
{
    uint32 control;
#ifdef BCMM2MDEV_ENABLED
    uint32 addrlow;
#endif

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma64_txinit\n", di->name));

    if (di->ntxd == 0)
        return;

    di->txin = di->txout = di->xs0cd = di->xs0cd_snapshot = 0;
    di->hnddma.txavail = di->ntxd - 1;

    /* clear tx descriptor ring */
    BZERO_SM((void *)(uintptr)di->txd64, (di->ntxd * sizeof(dma64dd_t)));

    /* These bits 20:18 (burstLen) of control register can be written but will take
     * effect only if these bits are valid. So this will not affect previous versions
     * of the DMA. They will continue to have those bits set to 0.
     */
    control = _CYHAL_GMAC_R_REG(&di->d64txregs->control);
    control = (control & ~D64_XC_BL_MASK) | (di->txburstlen << D64_XC_BL_SHIFT);
    control = (control & ~D64_XC_MR_MASK) | (di->txmultioutstdrd << D64_XC_MR_SHIFT);
    control = (control & ~D64_XC_PC_MASK) | (di->txprefetchctl << D64_XC_PC_SHIFT);
    control = (control & ~D64_XC_PT_MASK) | (di->txprefetchthresh << D64_XC_PT_SHIFT);
    _CYHAL_GMAC_W_REG(&di->d64txregs->control, control);


    control = D64_XC_XE;        /* OR in Tx enable -- why to we use 1 in 2 other places? Grrr... */
    /* DMA engine with out alignment requirement requires table to be inited
     * before enabling the engine
     */
    if (!di->aligndesc_4k)
        _dma_ddtable_init(di, GMAC_DMA_TX, di->txdpa);
#ifdef BCMM2MDEV_ENABLED
    addrlow = _CYHAL_GMAC_R_REG(&di->d64txregs->addrlow);
    addrlow &= 0xffff;
    _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, addrlow);
#endif

    /* if "default" setup does not want parity checks, 'or' in the ignore Parity flag */
    if ((di->hnddma.dmactrlflags & DMA_CTRL_PEN) == 0)
        control |= D64_XC_PD;

    _CYHAL_GMAC_OR_REG(&di->d64txregs->control, control);

    /* DMA engine with alignment requirement requires table to be inited
     * before enabling the engine
     */
    if (di->aligndesc_4k)
        _dma_ddtable_init(di, GMAC_DMA_TX, di->txdpa);

#ifdef BCMM2MDEV_ENABLED
    addrlow = _CYHAL_GMAC_R_REG(&di->d64txregs->addrlow);
    addrlow &= 0xffff;
    _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, addrlow);
#endif
}

static bool
dma64_txenabled(_cyhal_gmac_dma_info_t *di)
{
    uint32 xc;

    /* If the chip is dead, it is not enabled :-) */
    xc = _CYHAL_GMAC_R_REG(&di->d64txregs->control);
    return ((xc != 0xffffffff) && (xc & D64_XC_XE));
}

static void
dma64_txsuspend(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txsuspend\n", di->name));

    if (di->ntxd == 0)
        return;

    _CYHAL_GMAC_OR_REG(&di->d64txregs->control, D64_XC_SE);
}

static void
dma64_txresume(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txresume\n", di->name));

    if (di->ntxd == 0)
        return;

    _CYHAL_GMAC_AND_REG(&di->d64txregs->control, ~D64_XC_SE);
}

static bool
dma64_txsuspended(_cyhal_gmac_dma_info_t *di)
{
    return (di->ntxd == 0) ||
            ((_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_SE) == D64_XC_SE);
}

#ifdef WL_MULTIQUEUE
static void
dma64_txflush(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txflush\n", di->name));

    if (di->ntxd == 0)
        return;

    _CYHAL_GMAC_OR_REG(&di->d64txregs->control, D64_XC_SE | D64_XC_FL);
}

static void
dma64_txflush_clear(_cyhal_gmac_dma_info_t *di)
{
    uint32 status;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txflush_clear\n", di->name));

    if (di->ntxd == 0)
        return;

    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_XS_MASK)) !=
              D64_XS0_XS_DISABLED) &&
             (status != D64_XS0_XS_IDLE) &&
             (status != D64_XS0_XS_STOPPED),
             10000);
    _CYHAL_GMAC_AND_REG(&di->d64txregs->control, ~D64_XC_FL);
}

void
dma_txrewind(hnddma_t *dmah)
{
    uint16 start, end, i;
    uint act;
    uint32 flags;
    dma64dd_t *ring;

    _cyhal_gmac_dma_info_t * di = DI_INFO(dmah);

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma64_txrewind\n", di->name));

    CY_ASSERT(dma64_txsuspended(di));

    /* select to read index of already fetched desc */
    _CYHAL_GMAC_AND_REG(&di->d64txregs->control, ~D64_RC_SA);

    act = (uint)(_CYHAL_GMAC_R_REG(&di->d64txregs->status1) & D64_XS1_AD_MASK);
    act = (act - di->xmtptrbase) & D64_XS0_CD_MASK;
    start = (uint16)B2I(act, dma64dd_t);

    end = di->txout;

    ring = di->txd64;
    for (i = start; i != end; i = NEXTTXD(i)) {
        /* find the first one having eof set */
        flags = R_SM(&ring[i].ctrl1);
        if (flags & CTRL_EOF) {
            /* rewind end to (i+1) */
            _CYHAL_GMAC_W_REG( &di->d64txregs->ptr, di->xmtptrbase + I2B(NEXTTXD(i), dma64dd_t));
            _CYHAL_ETHERNET_LOG_INFO(("ActiveIdx %d EndIdx was %d now %d\n", start, end, NEXTTXD(i)));
            break;
        }
    }
}
#endif /* WL_MULTIQUEUE */

static void BCMFASTPATH
dma64_txreclaim(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range)
{
    void *p;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txreclaim %s\n", di->name,
               (range == HNDDMA_RANGE_ALL) ? "all" :
               ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered")));

    if (di->txin == di->txout)
        return;

    while ((p = dma64_getnexttxp(di, range))) {
        /* For unframed data, we don't have any packets to free */
        if (!(di->hnddma.dmactrlflags & DMA_CTRL_UNFRAMED))
        {
//            _CYHAL_GMAC_PKTFREE(p); We do not malloc Tx packets, do not free them!
        }
    }
}

static bool
dma64_txstopped(_cyhal_gmac_dma_info_t *di)
{
    return ((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_XS_MASK) == D64_XS0_XS_STOPPED);
}

static bool
dma64_rxstopped(_cyhal_gmac_dma_info_t *di)
{
    return ((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_RS_MASK) == D64_RS0_RS_STOPPED);
}

static bool
BCMATTACHFN_DMA_ATTACH(dma64_alloc)(_cyhal_gmac_dma_info_t *di, uint direction)
{
    uint32 size;
    uint ddlen;
    void *va;
    uint alloced = 0;
    uint32 align;
    uint16 align_bits;

    ddlen = sizeof(dma64dd_t);

    size = (direction == GMAC_DMA_TX) ? (di->ntxd * ddlen) : (di->nrxd * ddlen);
    align_bits = di->dmadesc_align;

    if (direction == GMAC_DMA_TX) {
        if ((va = dma_ringalloc((di->d64_xs0_cd_mask == 0x1fff) ? D64RINGBOUNDARY : D64RINGBOUNDARY_LARGE,
                                 size, &align_bits, &alloced,
                                 &di->txdpaorig)) == NULL)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: dma64_alloc: GMAC_DMA_ALLOC_CONSISTENT(ntxd) failed\n", di->name));
            return FALSE;
        }
        align = (1 << align_bits);

        /* adjust the pa by rounding up to the alignment */
        PHYSADDRLOSET(di->txdpa, ROUNDUP(PHYSADDRLO(di->txdpaorig), align));
        PHYSADDRHISET(di->txdpa, PHYSADDRHI(di->txdpaorig));

        /* Make sure that alignment didn't overflow */
        CY_ASSERT(PHYSADDRLO(di->txdpa) >= PHYSADDRLO(di->txdpaorig));

        /* find the alignment offset that was used */
        di->txdalign = (uint)(PHYSADDRLO(di->txdpa) - PHYSADDRLO(di->txdpaorig));

        /* adjust the va by the same offset */
        di->txd64 = (dma64dd_t *)((uintptr)va + di->txdalign);

        di->txdalloc = alloced;
        CY_ASSERT(ISALIGNED(PHYSADDRLO(di->txdpa), align));
    } else {
        if ( (va = dma_ringalloc((di->d64_rs0_cd_mask == 0x1fff) ? D64RINGBOUNDARY : D64RINGBOUNDARY_LARGE,
                                 size, &align_bits, &alloced,
                                 &di->rxdpaorig) ) == NULL)
        {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: dma64_alloc: GMAC_DMA_ALLOC_CONSISTENT(nrxd) failed\n",
                       di->name));
            return FALSE;
        }
        align = (1 << align_bits);

        /* adjust the pa by rounding up to the alignment */
        PHYSADDRLOSET(di->rxdpa, ROUNDUP(PHYSADDRLO(di->rxdpaorig), align));
        PHYSADDRHISET(di->rxdpa, PHYSADDRHI(di->rxdpaorig));

        /* Make sure that alignment didn't overflow */
        CY_ASSERT(PHYSADDRLO(di->rxdpa) >= PHYSADDRLO(di->rxdpaorig));

        /* find the alignment offset that was used */
        di->rxdalign = (uint)(PHYSADDRLO(di->rxdpa) - PHYSADDRLO(di->rxdpaorig));

        /* adjust the va by the same offset */
        di->rxd64 = (dma64dd_t *)((uintptr)va + di->rxdalign);

        di->rxdalloc = alloced;
        CY_ASSERT(ISALIGNED(PHYSADDRLO(di->rxdpa), align));
    }

    return TRUE;
}

static bool
dma64_txreset(_cyhal_gmac_dma_info_t *di)
{
    uint32 status;

    if (di->ntxd == 0)
        return TRUE;

    /* address PR8249/PR7577 issue */
    /* suspend tx DMA first */
    _CYHAL_GMAC_W_REG(&di->d64txregs->control, D64_XC_SE);
    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_XS_MASK)) !=
              D64_XS0_XS_DISABLED) &&
             (status != D64_XS0_XS_IDLE) &&
             (status != D64_XS0_XS_STOPPED),
             10000);

    /* PR2414 WAR: DMA engines are not disabled until transfer finishes */
    _CYHAL_GMAC_W_REG(&di->d64txregs->control, 0);
    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_XS_MASK)) !=
              D64_XS0_XS_DISABLED),
             10000);

    /* We should be disabled at this point */
    if (status != D64_XS0_XS_DISABLED) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: status != D64_XS0_XS_DISABLED 0x%x\n", __FUNCTION__, status));
        CY_ASSERT(status == D64_XS0_XS_DISABLED);
        _CYHAL_GMAC_OSL_DELAY(300);
    }

    return (status == D64_XS0_XS_DISABLED);
}

static bool
dma64_rxidle(_cyhal_gmac_dma_info_t *di)
{
    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_rxidle\n", di->name));

    if (di->nrxd == 0)
        return TRUE;

    return ((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK) ==
        (_CYHAL_GMAC_R_REG(&di->d64rxregs->ptr) & D64_RS0_CD_MASK));
}

static bool
dma64_rxreset(_cyhal_gmac_dma_info_t *di)
{
    uint32 status;

    if (di->nrxd == 0)
        return TRUE;

    /* PR2414 WAR: DMA engines are not disabled until transfer finishes */
    _CYHAL_GMAC_W_REG(&di->d64rxregs->control, 0);
    SPINWAIT(((status = (_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_RS_MASK)) !=
              D64_RS0_RS_DISABLED), 10000);

    return (status == D64_RS0_RS_DISABLED);
}

static bool
dma64_rxenabled(_cyhal_gmac_dma_info_t *di)
{
    uint32 rc;

    rc = _CYHAL_GMAC_R_REG(&di->d64rxregs->control);
    return ((rc != 0xffffffff) && (rc & D64_RC_RE));
}

static bool
dma64_txsuspendedidle(_cyhal_gmac_dma_info_t *di)
{
    /* XXX - verify PR20059 has been fixed */

    if (di->ntxd == 0)
        return TRUE;

    if (!(_CYHAL_GMAC_R_REG(&di->d64txregs->control) & D64_XC_SE))
        return 0;

    if ((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_XS_MASK) == D64_XS0_XS_IDLE)
        return 1;

    return 0;
}

/**
 * Useful when sending unframed data.  This allows us to get a progress report from the DMA.
 * We return a pointer to the beginning of the data buffer of the current descriptor.
 * If DMA is idle, we return NULL.
 */
/* XXX - Might be nice if DMA HW could tell us current position rather than current descriptor */
static void*
dma64_getpos(_cyhal_gmac_dma_info_t *di, bool direction)
{
    void *va;
    bool idle;
    uint16 cur_idx;

    if (direction == GMAC_DMA_TX) {
        cur_idx = B2I(((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) & D64_XS0_CD_MASK) -
                       di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);
        idle = !NTXDACTIVE(di->txin, di->txout);
        va = di->txp[cur_idx];
    } else {
#ifndef CORRECTION_TO_CURR_RX_DESCR_INDEX
        cur_idx = B2I(((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK) - di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
#else
        cur_idx = _dma_get_current_buffer_index(di);
#endif
        idle = !NRXDACTIVE(di->rxin, di->rxout);
        va = di->rxp[cur_idx];
    }

    /* If DMA is IDLE, return NULL */
    if (idle) {
        _CYHAL_ETHERNET_LOG_INFO(("%s: DMA idle, return NULL\n", __FUNCTION__));
        va = NULL;
    }

    return va;
}

/**
 * TX of unframed data
 *
 * Adds a DMA ring descriptor for the data pointed to by "buf".
 * This is for DMA of a buffer of data and is unlike other hnddma TX functions
 * that take a pointer to a "packet"
 * Each call to this is results in a single descriptor being added for "len" bytes of
 * data starting at "buf", it doesn't handle chained buffers.
 */
static int
dma64_txunframed(_cyhal_gmac_dma_info_t *di, void *buf, uint len, bool commit)
{
    uint16 txout;
    uint32 flags = 0;
    dmaaddr_t pa; /* phys addr */

    txout = di->txout;

    /* return nonzero if out of tx descriptors */
    if (NEXTTXD(txout) == di->txin)
        goto outoftxd;

    if (len == 0)
        return 0;

    pa = GMAC_DMA_MAP( buf, len, GMAC_DMA_TX, NULL);
    if (DMASGLIST_ENAB) {
        /* XXX Hack to trigger getnexttxp to recognize that
         * txp needs to be cleaned.  This should have been handled
         * as part of GMAC_DMA_MAP.
         */
         di->txp_dmah[txout].nsegs = 1;
    }

    flags = (D64_CTRL1_SOF | D64_CTRL1_IOC | D64_CTRL1_EOF);

    if (txout == (di->ntxd - 1))
        flags |= D64_CTRL1_EOT;

    dma64_dd_upd(di, di->txd64, pa, txout, &flags, len);
    CY_ASSERT(di->txp[txout] == NULL);

#if defined(BULK_DESCR_FLUSH)
    GMAC_DMA_MAP(  dma64_txd64(di, di->txout), DMA64_FLUSH_LEN(1),
        GMAC_DMA_TX, NULL, NULL);
#endif /* BULK_DESCR_FLUSH */

    /* save the buffer pointer - used by dma_getpos */
    di->txp[txout] = buf;

    txout = NEXTTXD(txout);
    /* bump the tx descriptor index */
    di->txout = txout;

    /* kick the chip */
    if (commit) {
        _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, di->xmtptrbase + I2B(txout, dma64dd_t));
    }

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (0);

outoftxd:
    _CYHAL_ETHERNET_LOG_ERROR(("%s: %s: out of txds !!!\n", di->name, __FUNCTION__));
    di->hnddma.txavail = 0;
    di->hnddma.txnobuf++;
    return (-1);
}


/**
 * RX of unframed data
 *
 * Adds a DMA ring descriptor for the data pointed to by "buf".
 * This is for DMA of a buffer of data and is unlike other hnddma RX functions
 * that take a pointer to a "packet"
 * Each call to this is result in a single descriptor being added for "len" bytes of
 * data starting at "buf", it doesn't handle chained buffers.
 */
int
dma_rxfill_unframed(hnddma_t *dmah, void *buf, uint len, bool commit)
{
    uint16 rxout;
    uint32 flags = 0;
    dmaaddr_t pa; /* phys addr */

    _cyhal_gmac_dma_info_t *di = DI_INFO(dmah);

    rxout = di->rxout;

    /* return nonzero if out of rx descriptors */
    if (NEXTRXD(rxout) == di->rxin)
        goto outofrxd;

    CY_ASSERT(len <= di->rxbufsize);

    /* cache invalidate maximum buffer length */
    pa = GMAC_DMA_MAP( buf, di->rxbufsize, GMAC_DMA_RX, NULL);

    if (rxout == (di->nrxd - 1))
        flags |= D64_CTRL1_EOT;

    dma64_dd_upd(di, di->rxd64, pa, rxout, &flags, len);
    CY_ASSERT(di->rxp[rxout] == NULL);

#if defined(BULK_DESCR_FLUSH)
    GMAC_DMA_MAP( dma64_rxd64(di, di->rxout), DMA64_FLUSH_LEN(1),
        GMAC_DMA_TX, NULL, NULL);
#endif /* BULK_DESCR_FLUSH */

    /* save the buffer pointer - used by dma_getpos */
    di->rxp[rxout] = buf;

    rxout = NEXTRXD(rxout);

    /* kick the chip */
    if (commit) {
        _CYHAL_GMAC_W_REG(&di->d64rxregs->ptr, di->rcvptrbase + I2B(rxout, dma64dd_t));
    }

    /* bump the rx descriptor index */
    di->rxout = rxout;

    /* rx flow control */
    di->rxavail = di->nrxd - NRXDACTIVE(di->rxin, di->rxout) - 1;

    return (0);

outofrxd:
    _CYHAL_ETHERNET_LOG_ERROR(("%s: %s: out of rxds !!!\n", di->name, __FUNCTION__));
    di->rxavail = 0;
    di->hnddma.rxnobuf++;
    return (-1);
}


/**
 * !! tx entry routine
 * WARNING: call must check the return value for error.
 *   the error(toss frames) could be fatal and cause many subsequent hard to debug problems
 */
static int BCMFASTPATH
dma64_txfast(_cyhal_gmac_dma_info_t *di, void *p0, uint32_t size, bool commit)
{
//    void *p, *next;   // For old looping code, not using
    uchar *data;
    uint len;
    uint16 txout;
    uint32 flags = 0;
    dmaaddr_t pa;
    bool war;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txfast\n", di->name));

#ifdef BCMLFRAG
    /* new DMA routine for LFRAGS */
    if (BCMLFRAG_ENAB()) {
        if (_CYHAL_GMAC_PKTISTXFRAG( p0))
            return dma64_txfast_lfrag(di, p0, commit);
#ifdef PCIE_PHANTOM_DEV
        /* changes to accomodate segment split up is also handled here */
        if (di->blwar_d11core)
            return dma64_txfast_lfrag(di, p0, commit);
#endif /* PCIE_PHANTOM_DEV */
    }
#endif /* BCMLFRAG */

    txout = di->txout;
    war = (di->hnddma.dmactrlflags & DMA_CTRL_DMA_AVOIDANCE_WAR) ? TRUE : FALSE;

    /*
     * Walk the chain of packet buffers
     * allocating and initializing transmit descriptor entries.
     */
//    for (p = p0; p; p = next) // Just one at a time.
    {
        uint nsegs, j, segsadd;
        hnddma_seg_map_t *map = NULL;

        data = p0;  // _CYHAL_GMAC_PKTDATA( p);
        len = size; // _CYHAL_GMAC_PKTLEN( p);
#ifdef BCM_DMAPAD
        if (DMAPADREQUIRED(di)) {
            len += PKTDMAPAD( p);
        }
#endif /* BCM_DMAPAD */
//        next = _CYHAL_GMAC_PKTNEXT( p);   // Just 1 at a time.

        /* return nonzero if out of tx descriptors */
        if (NEXTTXD(txout) == di->txin)
            goto outoftxd;

        /* PR988 - skip zero length buffers */
//        if (len == 0)
//            continue;     // without the "while" loop, this is an error

        if (len > 0)
        {
            /* get physical address of buffer start */
            if (DMASGLIST_ENAB)
                bzero(&di->txp_dmah[txout], sizeof(hnddma_seg_map_t));

            /* preamble is in the buffer, Send Preamble */
            pa = GMAC_DMA_MAP( data, len, GMAC_DMA_TX, p0); /* GMAC_DMA_MAP only uses 3 args - ( data, len, GMAC_DMA_TX) */
            if (DMASGLIST_ENAB) {
                map = &di->txp_dmah[txout];

                /* See if all the segments can be accounted for */
                if (map->nsegs > (uint)(di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1))
                    goto outoftxd;

                nsegs = map->nsegs;
            } else
                nsegs = 1;

            segsadd = 0;
            for (j = 1; j <= nsegs; j++)
            {
                flags = 0;
//              if (p == p0 && j == 1)        // Only 1 packet per call
                {
                    flags |= D64_CTRL1_SOF;     // Start of frame flag
                }

                /* With a DMA segment list, Descriptor table is filled
                 * using the segment list instead of looping over
                 * buffers in multi-chain DMA. Therefore, EOF for SGLIST is when
                 * end of segment list is reached.
                 */
                if ((!DMASGLIST_ENAB /*&& next == NULL */) ||
                    (DMASGLIST_ENAB && j == nsegs)) {
                    /* Set "interrupt on completion" bit only on last commit packet
                     * to reduce the Tx completion event
                     */
                    flags |= D64_CTRL1_EOF;     // End of frame flag

                    if (commit)
                        flags |= D64_CTRL1_IOC; // Interrupt on completion flag
                }

                if (txout == (di->ntxd - 1))
                    flags |= D64_CTRL1_EOT;     // End of descriptor table flag

                if (di->burstsize_ctrl)
                    flags |= D64_CTRL1_NOTPCIE; // Not PCIE

                if (DMASGLIST_ENAB) {
                    len = map->segs[j - 1].length;
                    pa = map->segs[j - 1].addr;
                    if (len > 128 && war) {
                        uint remain, new_len, align64;
                        /* check for 64B aligned of pa */
                        align64 = (uint)(PHYSADDRLO(pa) & 0x3f);
                        align64 = (64 - align64) & 0x3f;
                        new_len = len - align64;
                        remain = new_len % 128;
                        if (remain > 0 && remain <= 4) {
                            uint32 buf_addr_lo;
                            uint32 tmp_flags =
                                flags & (~(D64_CTRL1_EOF | D64_CTRL1_IOC));
                            flags &= ~(D64_CTRL1_SOF | D64_CTRL1_EOT);
                            remain += 64;
                            dma64_dd_upd(di, di->txd64, pa, txout,
                                &tmp_flags, len-remain);
                            CY_ASSERT(di->txp[txout] == NULL);

                            txout = NEXTTXD(txout);
                            /* return nonzero if out of tx descriptors */
                            if (txout == di->txin) {
                                _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: Out-of-DMA"
                                    " descriptors (txin %d txout %d"
                                    " nsegs %d)\n", __FUNCTION__,
                                    di->txin, di->txout, nsegs));
                                goto outoftxd;
                            }
                            if (txout == (di->ntxd - 1))
                                flags |= D64_CTRL1_EOT;
                            buf_addr_lo = PHYSADDRLO(pa);
                            PHYSADDRLOSET(pa, (PHYSADDRLO(pa) + (len-remain)));
                            if (PHYSADDRLO(pa) < buf_addr_lo) {
                                PHYSADDRHISET(pa, (PHYSADDRHI(pa) + 1));
                            }
                            len = remain;
                            segsadd++;
                            di->dma_avoidance_cnt++;
                        }
                    }
                }

                _CYHAL_ETHERNET_LOG_DEBUG(("CALL dma64_dd_upd       di  %p  di->txd64 %p   pa: 0x%08lx, p0: %p txout: %d  flags 0x%08x  len %d \n", di, di->txd64, pa, p0, txout, flags, len));

                dma64_dd_upd(di, di->txd64, pa, txout, &flags, len);
                CY_ASSERT(di->txp[txout] == NULL);

                txout = NEXTTXD(txout);
                /* return nonzero if out of tx descriptors */
                if (txout == di->txin) {
                    _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: Out-of-DMA descriptors"
                           " (txin %d txout %d nsegs %d)\n", __FUNCTION__,
                           di->txin, di->txout, nsegs));
                    goto outoftxd;
                }
            } /* for segments */
            if (segsadd && DMASGLIST_ENAB)
                map->nsegs += segsadd;

        }   /* len > 0  */

        /* See above. No need to loop over individual buffers */
//        if (DMASGLIST_ENAB)
//            break;        // without "for" loop, this is an error
    } /* end of for(p) loop */

    /* if last txd eof not set, fix it */
    if (!(flags & D64_CTRL1_EOF))
        W_SM(&di->txd64[PREVTXD(txout)].ctrl1,
             _CYHAL_BUS_SWAP32(flags | D64_CTRL1_IOC | D64_CTRL1_EOF));

    /* save the packet */
    di->txp[PREVTXD(txout)] = p0;

#if defined(BULK_DESCR_FLUSH)
    {
        uint32 flush_cnt = NTXDACTIVE(di->txout, txout);
        if (txout < di->txout) {
            GMAC_DMA_MAP( dma64_txd64(di, 0), DMA64_FLUSH_LEN(txout),
                    GMAC_DMA_TX, NULL, NULL);
            flush_cnt -= txout;
        }
        GMAC_DMA_MAP( dma64_txd64(di, di->txout), DMA64_FLUSH_LEN(flush_cnt),
                GMAC_DMA_TX, NULL, NULL);
    }
#endif  /* BULK_DESCR_FLUSH */

    /* bump the tx descriptor index */
    di->txout = txout;

    /* kick the chip */
    if (commit)
    {
        _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, di->xmtptrbase + I2B(txout, dma64dd_t));
    }

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (0);

outoftxd:
    _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: out of txds !!!\n", di->name));
//    _CYHAL_GMAC_PKTFREE(p0);  // We do not alloc Tx buffers
    di->hnddma.txavail = 0;
    di->hnddma.txnobuf++;
    return (-1);
}

/**
 * Reclaim next completed txd (txds if using chained buffers) in the range
 * specified and return associated packet.
 * If range is HNDDMA_RANGE_TRANSMITTED, reclaim descriptors that have be
 * transmitted as noted by the hardware "CurrDescr" pointer.
 * If range is HNDDMA_RANGE_TRANSFERED, reclaim descriptors that have be
 * transfered by the DMA as noted by the hardware "ActiveDescr" pointer.
 * If range is HNDDMA_RANGE_ALL, reclaim all txd(s) posted to the ring and
 * return associated packet regardless of the value of hardware pointers.
 */
static void * BCMFASTPATH
dma64_getnexttxp(_cyhal_gmac_dma_info_t *di, _cyhal_gmac_txd_range_t range)
{
    uint16 start, end, i;
    uint16 active_desc;
    void *txp;

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_getnexttxp %s\n", di->name,
               (range == HNDDMA_RANGE_ALL) ? "all" :
               ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered")));

    if (di->ntxd == 0)
        return (NULL);

    txp = NULL;

    start = di->txin;
    if (range == HNDDMA_RANGE_ALL)
        end = di->txout;
    else {
        dma64regs_t *dregs = di->d64txregs;

        if (di->txin == di->xs0cd) {
            end = (uint16)(B2I(((_CYHAL_GMAC_R_REG(&dregs->status0) & D64_XS0_CD_MASK) -
                  di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t));
            di->xs0cd = end;
        } else
            end = di->xs0cd;

        if (range == HNDDMA_RANGE_TRANSFERED) {
            active_desc = (uint16)(_CYHAL_GMAC_R_REG(&dregs->status1) & D64_XS1_AD_MASK);
            active_desc = (active_desc - di->xmtptrbase) & D64_XS0_CD_MASK;
            active_desc = B2I(active_desc, dma64dd_t);
            if (end != active_desc)
                end = PREVTXD(active_desc);
        }
    }

    /* PR4738 - xmt disable/re-enable does not clear CURR */
    if ((start == 0) && (end > di->txout))
    {
        goto bogus;
    }

    i = start;
    while (i != end && !txp)
    {
        /* XXX - dma 64-bit */
        hnddma_seg_map_t *map = NULL;
        uint size, j, nsegs;

#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
        dmaaddr_t pa;
        PHYSADDRLOSET(pa, (_CYHAL_BUS_SWAP32(R_SM(&di->txd64[i].addrlow)) - di->dataoffsetlow));
        PHYSADDRHISET(pa, (_CYHAL_BUS_SWAP32(R_SM(&di->txd64[i].addrhigh)) - di->dataoffsethigh));
#endif

        if (DMASGLIST_ENAB) {
            map = &di->txp_dmah[i];
            size = map->origsize;
            nsegs = map->nsegs;
            if (nsegs > (uint)NTXDACTIVE(i, end)) {
                di->xs0cd = i;
                break;
            }
        } else {
#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
            size = (_CYHAL_BUS_SWAP32(R_SM(&di->txd64[i].ctrl2)) & D64_CTRL2_BC_MASK);
#endif
            nsegs = 1;
        }

#ifdef PCIE_PHANTOM_DEV
        if (di->blwar_d11core) {
            size = di->blwar_size[i];
            nsegs = di->blwar_nsegs[i];
        }
#endif /* PCIE_PHANTOM_DEV */

        for (j = nsegs; j > 0; j--) {
#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
            W_SM(&di->txd64[i].addrlow, 0xdeadbeef);
            W_SM(&di->txd64[i].addrhigh, 0xdeadbeef);
#endif

            txp = di->txp[i];
            di->txp[i] = NULL;
            if (j > 1)
                i = NEXTTXD(i);
        }

#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
        GMAC_DMA_UNMAP( pa, size, GMAC_DMA_TX, txp);
#endif

        i = NEXTTXD(i);
    }

    di->txin = i;

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (txp);

bogus:
    _CYHAL_ETHERNET_LOG_DEBUG(("dma_getnexttxp: bogus curr: start %d end %d txout %d force %d\n",
              start, end, di->txout, forceall));
    return (NULL);
}

/** returns entries on the ring, in the order in which they were placed on the ring */
static void * BCMFASTPATH
dma64_getnextrxp(_cyhal_gmac_dma_info_t *di, bool forceall)
{
    uint16 i, curr;
    void *rxp;
#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
    dmaaddr_t pa;
#endif

    /* if forcing, dma engine must be disabled */
    CY_ASSERT(!forceall || !dma64_rxenabled(di));

#if defined(D11_SPLIT_RX_FD)
    bool still_processing = true;
    while(still_processing)
    {
#endif
        i = di->rxin;

        /* return if no packets posted */
        if (i == di->rxout)
            return (NULL);

        if (di->rxin == di->rs0cd)
        {
            /* Compute the index based off the RAM address of the Rx pointers */
#ifndef CORRECTION_TO_CURR_RX_DESCR_INDEX
            curr = (uint16)B2I(((_CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK) - di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
#else
            uint32_t base = (di->rcvptrbase & D64_RS0_CD_MASK);
            curr = _CYHAL_GMAC_R_REG(&di->d64rxregs->status0) & D64_RS0_CD_MASK;
            if (curr >= base)
            {
                curr = (uint16)B2I( (curr - base), dma64dd_t);
            }
#endif
            di->rs0cd = curr;
        }
        else
        {
            curr = di->rs0cd;
        }

        /* ignore curr if forceall */
        if (!forceall && (i == curr))
            return (NULL);

        /* get the packet pointer that corresponds to the rx descriptor */
        rxp = di->rxp[i];
        CY_ASSERT(rxp);

#if (defined(__mips__) || defined(BCM47XX_CA9) || defined(BCM_CR4_CACHED)) && !defined(_CFE_)
        if (!(di->hnddma.dmactrlflags & DMA_CTRL_UNFRAMED)) {
            /* Prcessor prefetch of 1 x 32B cacheline carrying 30B HWRXOFF */
            uint8 * addr = _CYHAL_GMAC_PKTDATA( rxp);
            _cyhal_gmac_prefetch_32B(addr, 1);
        }
#endif /* (__mips__ || BCM47XX_CA9) && !_CFE_ */

        di->rxp[i] = NULL;

#if defined(SGLIST_RX_SUPPORT)
        PHYSADDRLOSET(pa, (_CYHAL_BUS_SWAP32(R_SM(&di->rxd64[i].addrlow)) - di->dataoffsetlow));
        PHYSADDRHISET(pa, (_CYHAL_BUS_SWAP32(R_SM(&di->rxd64[i].addrhigh)) - di->dataoffsethigh));

        /* clear this packet from the descriptor ring */
        GMAC_DMA_UNMAP( pa, di->rxbufsize, GMAC_DMA_RX, rxp);

        W_SM(&di->rxd64[i].addrlow, 0xdeadbeef);
        W_SM(&di->rxd64[i].addrhigh, 0xdeadbeef);

#endif /* SGLIST_RX_SUPPORT */

        di->rxin = NEXTRXD(i);

        di->rxavail = di->nrxd - NRXDACTIVE(di->rxin, di->rxout) - 1;

#if defined(D11_SPLIT_RX_FD)
        /* We had marked up host descriptors as 0x80000000 */
        /* if we find that address, just skip that and go to next frame */
        if (di->sep_rxhdr)
        {
            if ((uint32)(uintptr)rxp == PCI64ADDR_HIGH)
            {
                continue;
            }
        }
        still_processing = false;
     }
#endif /* D11_SPLIT_RX_FD */

    return (rxp);
}

static bool
BCMATTACHFN_DMA_ATTACH(_dma64_addrext)(dma64regs_t *dma64regs)
{
    uint32 w;
    _CYHAL_GMAC_OR_REG(&dma64regs->control, D64_XC_AE);
    w = _CYHAL_GMAC_R_REG(&dma64regs->control);
    _CYHAL_GMAC_AND_REG(&dma64regs->control, ~D64_XC_AE);
    return ((w & D64_XC_AE) == D64_XC_AE);
}

/**
 * Rotate all active tx dma ring entries "forward" by (ActiveDescriptor - txin).
 */
static void
dma64_txrotate(_cyhal_gmac_dma_info_t *di)
{
    uint16 ad;
    uint nactive;
    uint rot;
    uint16 old, new;
    uint32 w;
    uint16 first, last;

    CY_ASSERT(dma64_txsuspendedidle(di));

    nactive = _dma_txactive(di);
    ad = B2I((((_CYHAL_GMAC_R_REG(&di->d64txregs->status1) & D64_XS1_AD_MASK)
        - di->xmtptrbase) & D64_XS1_AD_MASK), dma64dd_t);
    rot = TXD(ad - di->txin);

    CY_ASSERT(rot < di->ntxd);

    /* full-ring case is a lot harder - don't worry about this */
    if (rot >= (di->ntxd - nactive)) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txrotate: ring full - punt\n", di->name));
        return;
    }

    first = di->txin;
    last = PREVTXD(di->txout);

    /* move entries starting at last and moving backwards to first */
    for (old = last; old != PREVTXD(first); old = PREVTXD(old)) {
        new = TXD(old + rot);

        /*
         * Move the tx dma descriptor.
         * EOT is set only in the last entry in the ring.
         */
        w = _CYHAL_BUS_SWAP32(R_SM(&di->txd64[old].ctrl1)) & ~D64_CTRL1_EOT;
        if (new == (di->ntxd - 1))
            w |= D64_CTRL1_EOT;
        W_SM(&di->txd64[new].ctrl1, _CYHAL_BUS_SWAP32(w));

        w = _CYHAL_BUS_SWAP32(R_SM(&di->txd64[old].ctrl2));
        W_SM(&di->txd64[new].ctrl2, _CYHAL_BUS_SWAP32(w));

        W_SM(&di->txd64[new].addrlow, R_SM(&di->txd64[old].addrlow));
        W_SM(&di->txd64[new].addrhigh, R_SM(&di->txd64[old].addrhigh));

        /* zap the old tx dma descriptor address field */
        W_SM(&di->txd64[old].addrlow, _CYHAL_BUS_SWAP32(0xdeadbeef));
        W_SM(&di->txd64[old].addrhigh, _CYHAL_BUS_SWAP32(0xdeadbeef));

        /* move the corresponding txp[] entry */
        CY_ASSERT(di->txp[new] == NULL);
        di->txp[new] = di->txp[old];

        /* Move the map */
        if (DMASGLIST_ENAB) {
            bcopy(&di->txp_dmah[old], &di->txp_dmah[new], sizeof(hnddma_seg_map_t));
            bzero(&di->txp_dmah[old], sizeof(hnddma_seg_map_t));
        }

        di->txp[old] = NULL;
    }

    /* update txin and txout */
    di->txin = ad;
    di->txout = TXD(di->txout + rot);
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    /* kick the chip */
    _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, di->xmtptrbase + I2B(di->txout, dma64dd_t));
}

uint
BCMATTACHFN(dma_addrwidth)(_cyhal_gmac_si_t *sih, void *dmaregs)
{
    dma32regs_t *dma32regs;

    /* Perform 64-bit checks only if we want to advertise 64-bit (> 32bit) capability) */
    /* DMA engine is 64-bit capable */
    if ((_cyhal_gmac_si_core_sflags(sih, 0, 0) & SISF_DMA64) == SISF_DMA64) {
        /* backplane are 64-bit capable */
        if (_cyhal_gmac_si_backplane64(sih))
            /* If bus is System Backplane or PCIE then we can access 64-bits */
            if ((BUSTYPE(sih->bustype) == SI_BUS) ||
                ((BUSTYPE(sih->bustype) == PCI_BUS) &&
                 ((sih->buscoretype == PCIE_CORE_ID) ||
                  (sih->buscoretype == PCIE2_CORE_ID))))
                return (DMADDRWIDTH_64);

        /* DMA64 is always 32-bit capable, AE is always TRUE */
        CY_ASSERT(_dma64_addrext((dma64regs_t *)dmaregs));

        return (DMADDRWIDTH_32);
    }

    /* Start checking for 32-bit / 30-bit addressing */
    dma32regs = (dma32regs_t *)dmaregs;

    /* For System Backplane, PCIE bus or addrext feature, 32-bits ok */
    if ((BUSTYPE(sih->bustype) == SI_BUS) ||
        ((BUSTYPE(sih->bustype) == PCI_BUS) &&
         ((sih->buscoretype == PCIE_CORE_ID) ||
          (sih->buscoretype == PCIE2_CORE_ID))) ||
        (_dma32_addrext(dma32regs)))
        return (DMADDRWIDTH_32);

    /* Fallthru */
    return (DMADDRWIDTH_30);
}

static int
_dma_pktpool_set(_cyhal_gmac_dma_info_t *di, pktpool_t *pool)
{
    CY_ASSERT(di);
    CY_ASSERT(di->pktpool == NULL);
    di->pktpool = pool;
    return 0;
}

static bool
_dma_rxtx_error(_cyhal_gmac_dma_info_t *di, bool istx)
{
    uint32 status1 = 0;
    uint16 curr;

    if (DMA64_ENAB(di) && DMA64_MODE(di)) {

        if (istx) {

            status1 = _CYHAL_GMAC_R_REG(&di->d64txregs->status1);

            if ((status1 & D64_XS1_XE_MASK) != D64_XS1_XE_NOERR)
                return TRUE;
            else if (_cyhal_gmac_si_coreid(di->sih) == GMAC_CORE_ID && _cyhal_gmac_si_corerev(di->sih) >= 4) {
                curr = (uint16)(B2I(((_CYHAL_GMAC_R_REG(&di->d64txregs->status0) &
                    D64_XS0_CD_MASK) - di->xmtptrbase) &
                    D64_XS0_CD_MASK, dma64dd_t));

                if (NTXDACTIVE(di->txin, di->txout) != 0 &&
                    curr == di->xs0cd_snapshot) {

                    /* suspicious */
                    return TRUE;
                }
                di->xs0cd_snapshot = di->xs0cd = curr;

                return FALSE;
            }
            else
                return FALSE;
        }
        else {

            status1 = _CYHAL_GMAC_R_REG(&di->d64rxregs->status1);

            if ((status1 & D64_RS1_RE_MASK) != D64_RS1_RE_NOERR)
                return TRUE;
            else
                return FALSE;
        }

    }
#ifdef BCMDMA32
    else if (DMA32_ENAB(di))
    {
        return FALSE;
    }
#endif
    else
    {
        CY_ASSERT(0);
        return FALSE;
    }

}

void
_dma_burstlen_set(_cyhal_gmac_dma_info_t *di, uint8 rxburstlen, uint8 txburstlen)
{
    di->rxburstlen = rxburstlen;
    di->txburstlen = txburstlen;
}
#ifdef PCIE_PHANTOM_DEV
/* Book keeping arrays of nsegs and size while splitting up TCM packets into chunk of 64 bytes */
int
dma_blwar_alloc(hnddma_t *dmah)
{
    uint size;
    _cyhal_gmac_dma_info_t * di = (_cyhal_gmac_dma_info_t *)dmah;
    if ((di->ntxd) && di->blwar_d11core) {
        size  = di->ntxd * sizeof(uint);
        if ((di->blwar_nsegs = (uint*)_CYHAL_GMAC_MALLOC( size)) == NULL)
            goto fail;

        if ((di->blwar_size = (uint*)_CYHAL_GMAC_MALLOC( size)) == NULL)
            goto fail;

        bzero(di->blwar_nsegs, size);
        bzero(di->blwar_size, size);
    }
    return 0;

fail:
    return -1;
}
#endif /* PCIE_PHANTOM_DEV */
void
_dma_param_set(_cyhal_gmac_dma_info_t *di, uint16 paramid, uint16 paramval)
{
    switch (paramid) {
    case HNDDMA_PID_TX_MULTI_OUTSTD_RD:
        di->txmultioutstdrd = (uint8)paramval;
        break;

    case HNDDMA_PID_TX_PREFETCH_CTL:
        di->txprefetchctl = (uint8)paramval;
        break;

    case HNDDMA_PID_TX_PREFETCH_THRESH:
        di->txprefetchthresh = (uint8)paramval;
        break;

    case HNDDMA_PID_TX_BURSTLEN:
        di->txburstlen = (uint8)paramval;
        break;

    case HNDDMA_PID_RX_PREFETCH_CTL:
        di->rxprefetchctl = (uint8)paramval;
        break;

    case HNDDMA_PID_RX_PREFETCH_THRESH:
        di->rxprefetchthresh = (uint8)paramval;
        break;

    case HNDDMA_PID_RX_BURSTLEN:
        di->rxburstlen = (uint8)paramval;
        break;

    case HNDDMA_PID_BURSTLEN_CAP:
        di->burstsize_ctrl = (uint8)paramval;
        break;

#ifdef PCIE_PHANTOM_DEV
    case HNDDMA_PID_BURSTLEN_WAR:
        di->blwar_d11core = (uint8)paramval;
        break;
#endif

#if defined(D11_SPLIT_RX_FD)
    case HNDDMA_SEP_RX_HDR:
        di->sep_rxhdr = (uint8)paramval;    /* indicate sep hdr descriptor is used */
        break;
#endif /* D11_SPLIT_RX_FD */

    default:
        break;
    }
}

static bool
_dma_glom_enable(_cyhal_gmac_dma_info_t *di, uint32 val)
{
    dma64regs_t *dregs = di->d64rxregs;
    bool ret = TRUE;

    di->hnddma.dmactrlflags &= ~DMA_CTRL_SDIO_RXGLOM;
    if (val) {
        _CYHAL_GMAC_OR_REG(&dregs->control, D64_RC_GE);
        if (!(_CYHAL_GMAC_R_REG(&dregs->control) & D64_RC_GE))
            ret = FALSE;
        else
            di->hnddma.dmactrlflags |= DMA_CTRL_SDIO_RXGLOM;
    } else {
        _CYHAL_GMAC_AND_REG(&dregs->control, ~D64_RC_GE);
    }
    return ret;
}
static INLINE void
dma64_dd_upd_64(_cyhal_gmac_dma_info_t *di, dma64dd_t *ddring, dma64addr_t pa, uint outidx, uint32 *flags,
        uint32 bufcount)
{
    uint32 ctrl2 = bufcount & D64_CTRL2_BC_MASK;

    /* XXX: bit 63 is arleady set for host addresses by the caller */
    W_SM(&ddring[outidx].addrlow, _CYHAL_BUS_SWAP32(pa.loaddr + di->dataoffsetlow));
    W_SM(&ddring[outidx].addrhigh, _CYHAL_BUS_SWAP32(pa.hiaddr + di->dataoffsethigh));
    W_SM(&ddring[outidx].ctrl1, _CYHAL_BUS_SWAP32(*flags));
    W_SM(&ddring[outidx].ctrl2, _CYHAL_BUS_SWAP32(ctrl2));

    if (di->hnddma.dmactrlflags & DMA_CTRL_PEN) {
        if (DMA64_DD_PARITY(&ddring[outidx])) {
            W_SM(&ddring[outidx].ctrl2, _CYHAL_BUS_SWAP32(ctrl2 | D64_CTRL2_PARITY));
        }
    }
}
int BCMFASTPATH
dma_rxfast(hnddma_t *dmah, dma64addr_t p, uint32 len)
{
    uint16 rxout;
    uint32 flags = 0;

    _cyhal_gmac_dma_info_t * di = DI_INFO(dmah);
    /*
     * Determine how many receive buffers we're lacking
     * from the full complement, allocate, initialize,
     * and post them, then update the chip rx lastdscr.
     */

    rxout = di->rxout;

    if ((di->nrxd - NRXDACTIVE(di->rxin, di->rxout) - 1) < 1)
    {
        goto outofrxd;
    }

    /* reset flags for each descriptor */
    if (rxout == (di->nrxd - 1))
        flags = D64_CTRL1_EOT;

    /* Update descriptor */
    dma64_dd_upd_64(di, di->rxd64, p, rxout, &flags, len);

    di->rxp[rxout] = (void *)(uintptr)(p.loaddr);

    rxout = NEXTRXD(rxout);

    di->rxout = rxout;

    di->rxavail = di->nrxd - NRXDACTIVE(di->rxin, di->rxout) - 1;
    /* update the chip lastdscr pointer */
    _CYHAL_GMAC_W_REG(&di->d64rxregs->ptr, di->rcvptrbase + I2B(rxout, dma64dd_t));
    return 0;
outofrxd:
    di->rxavail = 0;
    _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_rxfast: out of rxds\n", di->name));
    return -1;
}
int BCMFASTPATH
dma_msgbuf_txfast(hnddma_t *dmah, dma64addr_t p0, bool commit, uint32 len, bool first, bool last)
{
    uint16 txout;
    uint32 flags = 0;
    _cyhal_gmac_dma_info_t * di = DI_INFO(dmah);

    _CYHAL_ETHERNET_LOG_INFO(("%s: dma_txfast\n", di->name));

    txout = di->txout;

    /* return nonzero if out of tx descriptors */
    if (NEXTTXD(txout) == di->txin)
    {
        goto outoftxd;
    }

    if (len == 0)
        return 0;

    if (first)
        flags |= D64_CTRL1_SOF;
    if (last)
        flags |= D64_CTRL1_EOF | D64_CTRL1_IOC;

    if (txout == (di->ntxd - 1))
        flags |= D64_CTRL1_EOT;

    dma64_dd_upd_64(di, di->txd64, p0, txout, &flags, len);

    CY_ASSERT(di->txp[txout] == NULL);

    txout = NEXTTXD(txout);

    /* return nonzero if out of tx descriptors */
    if (txout == di->txin)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: Out-of-DMA descriptors"
               " (txin %d txout %d)\n", __FUNCTION__,
               di->txin, di->txout));
        goto outoftxd;
    }

    /* save the packet */
    di->txp[PREVTXD(txout)] = (void *)(uintptr)(p0.loaddr);

    /* bump the tx descriptor index */
    di->txout = txout;

    /* kick the chip */
    if (commit)
        _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, di->xmtptrbase + I2B(txout, dma64dd_t));

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (0);

outoftxd:
    _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: out of txds !!!\n", di->name));
    di->hnddma.txavail = 0;
    di->hnddma.txnobuf++;
    return (-1);
}


#ifdef BCMLFRAG
/*
 * Sequentially program the pktdata(lfrag) - from TCM, followed by the
 * individual fragments from the HOST.
 */
static int BCMFASTPATH
dma64_txfast_lfrag(_cyhal_gmac_dma_info_t *di, void *p0, bool commit)
{
    void *p, *next;
    uchar *data;
    uint len;
    uint16 txout;
    uint32 flags = 0;
    dmaaddr_t pa;
    uint8 j = 0;
#ifdef BCMLFRAG
    uint8 i = 0;
    dma64addr_t pa64 = {0, 0};
#endif
#ifdef PCIE_PHANTOM_DEV
    uint16 thresh = 64;
    uint16 templen;
    uint8* panew;
#endif

    txout = di->txout;

    /*
     * Lfrag - Program the descriptor for Lfrag data first before
     * considering the individual fragments
     */
    for (p = p0; p; p = next) {
        uint ftot = 0;
        uint nsegs = 1;

        next = _CYHAL_GMAC_PKTNEXT( p);
        data = _CYHAL_GMAC_PKTDATA( p);
        len  = _CYHAL_GMAC_PKTLEN( p);

        if (_CYHAL_GMAC_PKTISTXFRAG( p))
            ftot = _CYHAL_GMAC_PKTFRAGTOTNUM(p);

        if (len == 0)
        {
            /* XXX: Should not happen ideally unless this is a chained lfrag */
#ifdef PCIE_PHANTOM_DEV
            if (di->blwar_d11core) {
                di->blwar_nsegs[txout] = 0 + ftot;
                di->blwar_size[txout] = 0;
            }
#endif
        }
        else    /* (len > 0) */
        {
            pa = GMAC_DMA_MAP( data, len, GMAC_DMA_TX, p);
#ifdef PCIE_PHANTOM_DEV
            panew = (uint8*) pa;
            if (di->blwar_d11core) {
                if (ISALIGNED((uint8 *) pa, 4))
                    templen = len;
                else
                    templen = len + ((uint8 *) pa - (uint8 *) ALIGN_ADDR(pa, 4));

                nsegs = (templen % thresh) ? ((templen / thresh) + 1) : (templen / thresh);
                if ((nsegs+ftot) > (uint)(di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1))
                    goto outoftxd;

                di->blwar_nsegs[txout] = nsegs + ftot;
                di->blwar_size[txout] = len;
            } else
#endif /* PCIE_PHANTOM_DEV */
            {
                if ((nsegs+ftot) > (uint)(di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1))
                    goto outoftxd;
            }

            for (j = 1; j <= nsegs; j++) {
                flags = 0;
                if ((p == p0) && (j == 1))
                    flags |= D64_CTRL1_SOF;
                if (txout == (di->ntxd - 1))
                    flags |= D64_CTRL1_EOT;
                if (di->burstsize_ctrl)
                    flags |= D64_CTRL1_NOTPCIE;

                if ((j == nsegs) && (ftot == 0) && (next == NULL))
                    flags |= (D64_CTRL1_IOC | D64_CTRL1_EOF);
#ifdef PCIE_PHANTOM_DEV
                uint16 dma_len;
                if (di->blwar_d11core) {
                    /* split into chunks of thresh bytes */
                    if (len < thresh)
                        dma_len = len;
                    else
                        dma_len = thresh - ((uintptr)panew % 4);

                    len -= dma_len;
                    dma64_dd_upd(di, di->txd64, (dmaaddr_t)panew,
                        txout, &flags, dma_len);
                    panew = panew + dma_len;
                } else
#endif
                {
                    dma64_dd_upd(di, di->txd64, pa, txout,
                        &flags, len);
                }

                CY_ASSERT(di->txp[txout] == NULL);
                txout = NEXTTXD(txout);
            }

            if (ftot > 0) {
                /*
                 * Now, walk the chain of fragments in this lfrag allocating
                 * and initializing transmit descriptor entries.
                 */
                for (i = 1, j = 1; j <= ftot; i++, j++) {
                    flags = 0;
                    if (_CYHAL_GMAC_PKTFRAGISCHAINED( i)) {
                         i = 1;
                         p = _CYHAL_GMAC_PKTNEXT( p);
                         CY_ASSERT(p != NULL);
                         next = _CYHAL_GMAC_PKTNEXT( p);
                    }

                    len = _CYHAL_GMAC_PKTFRAGLEN( p, i);

#ifdef BCM_DMAPAD
                    if (DMAPADREQUIRED(di)) {
                        len += PKTDMAPAD( p);
                    }
#endif /* BCM_DMAPAD */

                    pa64.loaddr = _CYHAL_GMAC_PKTFRAGDATA_LO( p, i);
                    pa64.hiaddr = _CYHAL_GMAC_PKTFRAGDATA_HI( p, i);

                    if ((j == ftot) && (next == NULL))
                        flags |= (D64_CTRL1_IOC | D64_CTRL1_EOF);
                    if (txout == (di->ntxd - 1))
                        flags |= D64_CTRL1_EOT;

                    /* War to handle 64 bit dma address for now */
                    dma64_dd_upd_64(di, di->txd64, pa64, txout, &flags, len);

                    CY_ASSERT(di->txp[txout] == NULL);
                    txout = NEXTTXD(txout);
                }
            }   /* if (ftot > 0) */
        } /* end of else from len > 0 */
    }

    /* save the packet */
    di->txp[PREVTXD(txout)] = p0;

    /* bump the tx descriptor index */
    di->txout = txout;

    /* kick the chip */
    if (commit)
        _CYHAL_GMAC_W_REG(&di->d64txregs->ptr, di->xmtptrbase + I2B(txout, dma64dd_t));

    /* tx flow control */
    di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

    return (0);

outoftxd:
    _CYHAL_ETHERNET_LOG_ERROR(("%s: dma_txfast: out of txds !!!\n", di->name));
//    _CYHAL_GMAC_PKTFREE(p0);  // We do not alloc Tx buffers
    di->hnddma.txavail = 0;
    di->hnddma.txnobuf++;
    return (-1);
}
#endif /* BCMLFRAG */


