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
 * Common [OS-independent] header file for
 * Broadcom BCM47XX 10/100Mbps Ethernet Device Driver
 *
 * $Id: cyhal_ethernet_etc.h 475115 2014-05-03 06:23:15Z sudhirbs $
 */

#pragma once

#include "cyhal_ethernet.h"
#include "cyhal_ethernet_phy.h"
#include "cyhal_ethernet_devices.h"
#include "cyhal_ethernet_gmac_dma.h"
#include "cyhal_ethernet_gmac_filter.h"

/******************************************************************************
 *
 * Ethernet Logging
 *
******************************************************************************/

#define SUCCESS                 0
#define FAILURE                -1


/* To turn off all logging, define _CYHAL_ETHERNET_LOG_LEVEL_DEFAULT as _CYHAL_ETHERNET_LOG_NONE */
#ifndef _CYHAL_ETHERNET_LOG_LEVEL_DEFAULT
#define _CYHAL_ETHERNET_LOG_LEVEL_DEFAULT _CYHAL_ETHERNET_LOG_LEVEL_ERROR
#endif

#if (CYHAL_ETHERNET_LOG_LEVEL_DEFAULT > _CYHAL_ETHERNET_LOG_LEVEL_NONE)
#define _CYHAL_ETHERNET_LOG_ENABLE
#endif

enum {
    _CYHAL_ETHERNET_LOG_LEVEL_NONE,
    _CYHAL_ETHERNET_LOG_LEVEL_ERROR,       /* Print log message if run-time level is <= _CYHAL_ETHERNET_LOG_ERROR  */
    _CYHAL_ETHERNET_LOG_LEVEL_WARNING,     /* Print log message if run-time level is <= _CYHAL_ETHERNET_LOG_WARNING  */
    _CYHAL_ETHERNET_LOG_LEVEL_NOTICE,      /* Print log message if run-time level is <= _CYHAL_ETHERNET_LOG_NOTICE   */
    _CYHAL_ETHERNET_LOG_LEVEL_INFO,        /* Print log message if run-time level is <= _CYHAL_ETHERNET_LOG_INFO     */
    _CYHAL_ETHERNET_LOG_LEVEL_DEBUG,       /* Print log message if run-time level is <= _CYHAL_ETHERNET_LOG_DEBUG    */

    _CYHAL_ETHERNET_LOG_LEVEL_MAX          /* last value - not an actual level */
};

#ifdef _CYHAL_ETHERNET_LOG_ENABLE

struct cyhal_ether_header;
extern void etc_prhdr(char *msg, struct cyhal_ether_header *eh, uint16_t len, int unit);
extern void etc_prhex(char *msg, uchar *buf, uint16_t nbytes, int unit);

#define _CYHAL_ETHERNET_LOG_ERROR(args)      if (cy_ethernet_log_level <= _CYHAL_ETHERNET_LOG_ERROR) printf args
#define _CYHAL_ETHERNET_LOG_WARNING(args)    if (cy_ethernet_log_level <= _CYHAL_ETHERNET_LOG_WARNING) printf args
#define _CYHAL_ETHERNET_LOG_NOTICE(args)     if (cy_ethernet_log_level <= _CYHAL_ETHERNET_LOG_NOTICE) printf args
#define _CYHAL_ETHERNET_LOG_INFO(args)       if (cy_ethernet_log_level <= _CYHAL_ETHERNET_LOG_INFO) printf args
#define _CYHAL_ETHERNET_LOG_DEBUG(args)      if (cy_ethernet_log_level <= _CYHAL_ETHERNET_LOG_DEBUG) printf args

#else    /* _CYHAL_ETHERNET_LOG_ENABLE */
#define _CYHAL_ETHERNET_LOG_ERROR(args)      printf args     /* Always print errors */
#define _CYHAL_ETHERNET_LOG_WARNING(args)
#define _CYHAL_ETHERNET_LOG_NOTICE(args)
#define _CYHAL_ETHERNET_LOG_INFO(args)
#define _CYHAL_ETHERNET_LOG_DEBUG(args)
#endif    /* _CYHAL_ETHERNET_LOG_ENABLE */

extern uint32_t cy_ethernet_log_level;

/******************************************************************************
 *
 * Macros and Defines
 *
 ******************************************************************************/
#define _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES            (   4)      /* Number of DMA transmit Queues */
#define _CYHAL_ETHERNET_DMA_TX_RECORD_THRESHOLD      (   8)      /* Threshold for Tx records */
#define _CYHAL_ETHERNET_DMA_QUOTA_MAX                (  64)      /* Max quota in dpc mode: bounds IOV_PKTCBND */

#define MACADDR_MASK                        0x0000FFFFFFFFFFFFLL
#define VID_MASK                            0x0FFF000000000000LL

/* interrupt event bitvec */
#define GMAC_INTR_TX                        (0x01)
#define GMAC_INTR_RX                        (0x02)
#define GMAC_INTR_ERROR                     (0x04)

#if defined(BCM_GMAC3)  // Time out for BMC_GMAC3, CYW43907AEVAL1F DOES NOT use this
#define GMAC_INTR_TO                        (0x08)
#endif
#define GMAC_INTR_MDIO                      (0x10)
#define GMAC_INTR_NEW                       (0x80)

/* gmac forcespeed values */
#define GMAC_FORCE_SPEED_10HALF             (1 << 0)
#define GMAC_FORCE_SPEED_10FULL             (1 << 1)
#define GMAC_FORCE_SPEED_100HALF            (1 << 2)
#define GMAC_FORCE_SPEED_100FULL            (1 << 3)
#define GMAC_FORCE_SPEED_1000HALF           (1 << 4)
#define GMAC_FORCE_SPEED_1000FULL           (1 << 5)
#define GMAC_FORCE_SPEED_2500FULL           (1 << 6)
#define GMAC_FORCE_SPEED_AUTO               (1 << 7)

/* init options */
#define ET_INIT_FULL                        (0x01)
#define ET_INIT_INTRON                      (0x02)

/* Specific init options for _cyhal_ethernet_et_init */
#define ET_INIT_DEF_OPTIONS                 (ET_INIT_FULL | ET_INIT_INTRON)
#define ET_INIT_INTROFF                     (ET_INIT_FULL)
#define ET_INIT_PARTIAL                     (0)

/*
 * Least-common denominator rxbuf start-of-data offset:
 * Must be >= size of largest rxhdr
 * Must be 2-mod-4 aligned so IP is 0-mod-4
 */
#define _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET (32)

#define TC_BK                               ( 0)    /* background traffic class */
#define TC_BE                               ( 1)    /* best effort traffic class */
#define TC_CL                               ( 2)    /* controlled load traffic class */
#define TC_VO                               ( 3)    /* voice traffic class */
#define TC_NONE                             (-1)    /* traffic class none */

#define MAXPRIO                             (7)     /* there are 7 priority indexes */
#define NUMPRIO                             (MAXPRIO + 1)

#define RX_Q0                               ( 0)    /* receive DMA queue */
#define NUMRXQ                              ( 1)    /* gmac has one rx queue */

#define _CYHAL_ETHERNET_DMA_TX_CHAN_Q0       TC_BK    /* DMA txq 0 */
#define _CYHAL_ETHERNET_DMA_TX_CHAN_Q1       TC_BE    /* DMA txq 1 */
#define _CYHAL_ETHERNET_DMA_TX_CHAN_Q2       TC_CL    /* DMA txq 2 */
#define _CYHAL_ETHERNET_DMA_TX_CHAN_Q3       TC_VO    /* DMA txq 3 */


/******************************************************************************
 *
 * Forward Declarations used in structures
 *
******************************************************************************/

/******************************************************************************
 *
 * Structures
 *
******************************************************************************/

/*
 * "Common" os-independent software state structure.
 */
typedef struct etc_info {
    void                *et;                /* pointer to os-specific private state _cyhal_ethernet_info_t * */
    uint16_t            unit;               /* device instance number */
    bool                pktc;               /* packet chaining enabled or not */

    uint32_t            bp_ticks_usec;      /* backplane clock ticks per microsec */
    uint32_t            rxlazy_timeout;     /* rxlazy timeout configuration */
    uint32_t            rxlazy_framecnt;    /* rxlazy framecount configuration */

    int                 pktcbnd;            /* max # of packets to chain */
    int                 quota;              /* max # of packets to recv */

    void                *mib;               /* pointer to s/w maintained mib counters */
    bool                up;                 /* interface up and running */
    bool                promisc;            /* allow promiscuous destination address (default true) */
    bool                qos;                /* QoS priority determination on rx */
    bool                broadcast_enable;    /* Broadcast enable */
    uint32_t            loopbk;             /* loopback override mode */
    bool                ethernet_link_up;    /* true when we are linked */

    int                 forcespeed;         /* disable autonegotiation and force speed/duplex */
    uint16_t            advertise;          /* control speed/duplex advertised caps */
    uint16_t            advertise2;         /* control gige speed/duplex advertised caps */
    bool                needautoneg;        /* request restart autonegotiation */
    int                 speed;              /* current speed: 10, 100 */
    int                 duplex;             /* current duplex: 0=half, 1=full */

    bool                piomode;            /* enable programmed io (!dma) */
    void                *pioactive;         /* points to pio packet being transmitted */
    volatile uint32_t   *txavail[_CYHAL_ETHERNET_DMA_TX_NUM_QUEUES];   /* dma: # tx descriptors available */

    uint16_t            vendorid;           /* pci function vendor id */
    uint16_t            deviceid;           /* pci function device id */
    uint16_t            chip;               /* chip number */
    uint16_t            chiprev;            /* chip revision */
    uint16_t            chippkg;            /* chip package option */
    uint16_t            coreid;             /* core id */
    uint32_t            corerev;            /* core revision */

    bool                nicmode;            /* is this core using its own pci i/f */

    struct chops        *chops;             /* pointer to chip-specific opsvec */
    void                *ch;                /* pointer to chip-specific state */

    uint16_t            txq_state;          /* tx queues state bits */
    uint16_t            phyaddr;            /* sb chips: mdio 5-bit phy address */
    uint16_t            mdcport;            /* sb chips: which mii to use (enet core #) to access phy */

    cyhal_ether_addr_t  cur_etheraddr;      /* our local ethernet address */
    cyhal_ether_addr_t  perm_etheraddr;     /* original sprom local ethernet address */

    bool                linkstate;          /* link integrity state */
    bool                pm_modechange;      /* true if mode change is to due pm */

    uint32_t            now;                /* elapsed seconds, incremented in _cyhal_ethernet_etc_watchdog(), whihc needs to be called! */

    uint32_t            boardflags;         /* board flags */
    uint32_t            txrec_thresh;       /* # of tx frames after which reclaim is done */

    /* sw-maintained stat counters */
    uint32_t            txframes[_CYHAL_ETHERNET_DMA_TX_NUM_QUEUES];   /* transmitted frames on each tx fifo */
    uint32_t            txframe;            /* transmitted frames */
    uint32_t            txbyte;             /* transmitted bytes */
    uint32_t            rxframe;            /* received frames */
    uint32_t            rxbyte;             /* received bytes */
    uint32_t            txerror;            /* total tx errors */
    uint32_t            txnobuf;            /* tx out-of-buffer errors */
    uint32_t            rxerror;            /* total rx errors */
    uint32_t            rxgiants;           /* total rx giant frames */
    uint32_t            rxnobuf;            /* rx out-of-buffer errors */
    uint32_t            reset;              /* reset count */
    uint32_t            dmade;              /* pci descriptor errors */
    uint32_t            dmada;              /* pci data errors */
    uint32_t            dmape;              /* descriptor protocol error */
    uint32_t            rxdmauflo;          /* receive descriptor underflow */
    uint32_t            rxoflo;             /* receive fifo overflow */
    uint32_t            txuflo;             /* transmit fifo underflow */
    uint32_t            rxcrcerr;           /* Rx frames discarded due to CRC errors */
    uint32_t            rxoflodiscards;     /* Rx frames discarded during rx fifo overflow */
    uint32_t            rxovrsize;          /* Rx frames discarded due to rx oversized */
    uint32_t            rxbadlen;           /* Rx 802.3 len field != read length */
    uint32_t            dropped_bcast_cnt;  /* Rx dropped due to promisc or broadcast filtering */
    uint32_t            chained;            /* number of frames chained */
    uint32_t            unchained;          /* number of frames not chained */
    uint32_t            maxchainsz;         /* max chain size so far */
    uint32_t            currchainsz;        /* current chain size */
#if defined(CYHAL_ETHERNET_LOG_ENABLE)
    uint32_t            quota_stats[CYHAL_ETHERNET_DMA_QUOTA_MAX];
#endif /* _CYHAL_ETHERNET_LOG_ENABLE */
    uint32_t            rxprocessed;
    uint32_t            reset_countdown;

} etc_info_t;

/******************************************************************************
 *
 * Forward Declarations used in structures
 *
******************************************************************************/
struct bcmgmac;                         /* forward declaration */
#define ch_t        struct bcmgmac

/******************************************************************************
 *
 * Functions
 *
******************************************************************************/

/* exported prototypes */
extern void *_cyhal_ethernet_etc_attach(void *et, uint16_t vendor, uint16_t device, void *regsva);
extern void _cyhal_ethernet_etc_detach(etc_info_t *etc);
extern void _cyhal_ethernet_etc_reset(etc_info_t *etc);
extern void _cyhal_ethernet_etc_init(etc_info_t *etc, uint16_t options);
extern bool _cyhal_ethernet_etc_transmit(etc_info_t *etc, void *p, uint16_t size);
extern uint32_t _cyhal_ethernet_etc_receive_and_copy(etc_info_t *etc, uint8_t *buffer, uint32_t avail_buffer_size);
extern void _cyhal_ethernet_etc_up(etc_info_t *etc);
extern uint16_t _cyhal_ethernet_etc_down(etc_info_t *etc, int reset);
extern void _cyhal_ethernet_etc_promisc(etc_info_t *etc, uint16_t on);
extern void _cyhal_ethernet_etc_qos(etc_info_t *etc, uint16_t on);
extern void _cyhal_ethernet_etc_quota(etc_info_t *etc);
extern int16_t _cyhal_ethernet_etc_set_speed(etc_info_t *etc, uint32_t val);

extern void _cyhal_ethernet_etc_rxlazy(etc_info_t *etc, uint16_t microsecs, uint16_t framecnt);
extern uint16_t _cyhal_ethernet_etc_dump(etc_info_t *etc);
extern void _cyhal_ethernet_etc_watchdog(etc_info_t *etc);

extern int _cyhal_ethernet_etc_broadcast_receive_enable(etc_info_t *etc, bool enable_bcast);

extern void _cyhal_ethernet_etc_set_pause_quanta(etc_info_t *etc, uint16_t pause_quanta);
extern void _cyhal_ethernet_etc_start_pause_frames(etc_info_t *etc, bool start);

void _cyhal_ethernet_etc_clear_interrupts(etc_info_t *etc, uint32_t int_bits_to_clear);

extern int _cyhal_ethernet_etc_read_mac_addr_from_OTP( cyhal_ether_addr_t *mac_addr );
extern int _cyhal_ethernet_etc_read_mac_addr_from_GMAC(cyhal_ether_addr_t *mac_addr);
extern bool _cyhal_gmac_get_promisc(ch_t *ch);
extern void _cyhal_gmac_promisc(ch_t *ch, bool mode);

extern uint32_t _cyhal_ethernet_etc_phyread(etc_info_t *etc, uint8_t reg);
extern int16_t _cyhal_ethernet_etc_phywrite(etc_info_t *etc, uint8_t reg, uint32_t value);


extern void _cyhal_ethernet_etc_dumpetc(etc_info_t *etc);
extern void _cyhal_ethernet_etc_loopback(etc_info_t *etc, uint16_t mode);

/* local prototypes */
void *_cyhal_gmac_chip_attach(etc_info_t *etc, void *regsva);
void _cyhal_gmac_chip_detach(ch_t *ch);
void _cyhal_gmac_chip_reset(ch_t *ch);
void _cyhal_gmac_chip_init(ch_t *ch, uint16_t options);
bool _cyhal_gmac_chip_tx(ch_t *ch, void *p, uint16_t size);
void *_cyhal_gmac_chip_rx(ch_t *ch, uint32_t *size);
void *_cyhal_gmac_chip_rxpeek_and_drop(ch_t *ch);
int  _cyhal_gmac_chip_rxquota(ch_t *ch, int quota, void **rxpkts);
void _cyhal_gmac_chip_rxlazy(ch_t *ch);
void _cyhal_gmac_chip_rxfill(ch_t *ch);
uint16_t _cyhal_gmac_chip_getintrevents(ch_t *ch, bool in_isr);
bool _cyhal_gmac_chip_errors(ch_t *ch);
bool _cyhal_gmac_chip_dmaerrors(ch_t *ch);
void _cyhal_gmac_chip_intrson(ch_t *ch);
void _cyhal_gmac_chip_intrsoff(ch_t *ch);
void _cyhal_gmac_chip_txreclaim(ch_t *ch, bool all);
void _cyhal_gmac_chip_rxreclaim(ch_t *ch);
uint16_t _cyhal_gmac_chip_activerxbuf(ch_t *ch);
void _cyhal_gmac_chip_statsupd(ch_t *ch);
void _cyhal_gmac_chip_dumpmib(ch_t *ch, bool clear);
void _cyhal_gmac_chip_configtimer(ch_t *ch, uint16_t microsecs);
void _cyhal_gmac_chip_phyreset(ch_t *ch, uint16_t phyaddr);
uint16_t _cyhal_gmac_chip_phyrd(ch_t *ch, uint16_t phyaddr, uint16_t reg);
void _cyhal_gmac_chip_phywr(ch_t *ch, uint16_t phyaddr, uint16_t reg, uint16_t v);
uint16_t _cyhal_gmac_chip_macrd(ch_t *ch, uint16_t reg);
void _cyhal_gmac_chip_macwr(ch_t *ch, uint16_t reg, uint16_t val);
uint16_t _cyhal_gmac_chip_dump(ch_t *ch);
void _cyhal_gmac_chip_duplexupd(ch_t *ch);

void _cyhal_gmac_chip_phyinit(ch_t *ch, uint16_t phyaddr);
void _cyhal_gmac_chip_phyor(ch_t *ch, uint16_t phyaddr, uint16_t reg, uint16_t v);
void _cyhal_gmac_chip_phyforce(ch_t *ch, uint16_t phyaddr);
void _cyhal_gmac_chip_phyadvertise(ch_t *ch, uint16_t phyaddr);
void _cyhal_gmac_chip_phyloopback(ch_t *ch, uint32_t mode);
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
void _cyhal_gmac_chip_dumpregs(ch_t *ch, gmacregs_t *regs);
#endif /* _CYHAL_ETHERNET_LOG_ENABLE */
void _cyhal_gmac_pause_quanta(ch_t *ch, uint16_t quanta);
void _cyhal_gmac_start_pause(ch_t *ch, const bool start);
void _cyhal_gmac_miiconfig(ch_t *ch);
int _cyhal_gmac_loopback(ch_t *ch, uint32_t mode);

void _cyhal_gmac_chip_clearinterrupts(ch_t *ch, uint32_t int_bits_to_clear);

