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
 * Broadcom Gigabit Ethernet MAC (Unimac) core.
 *
 * This file implements the chip-specific routines for the GMAC core.
 *
 * $Id: etcgmac.c 477266 2014-05-13 03:49:08Z weitsan $
 */

#include <stdio.h>
#include <typedefs.h>
#include "bcmdefs.h"
#include "bcmendian.h"
#include "bcmutils.h"
#include "cyhal_ethernet_devices.h"
#include "cyhal_ethernet_phy.h"
#include "cyhal_ethernet.h"
#include "cyhal_ethernet_internal.h"
#include "cyhal_system.h"
#include "cyhal_ethernet_gmac_siutils.h"
#include "sbhnddma.h"
#include "sbchipc.h"
#include "hndsoc.h"
#include "cyhal_ethernet_gmac_pmu.h"
#include "cyhal_ethernet_gmac_mib.h"
#include "cyhal_ethernet_gmac_common.h"
#include "cyhal_ethernet_gmac_core.h"
#include "cyhal_ethernet_gmac_rx_hdr.h"
#include "cyhal_ethernet_gmac_filter.h"

#include "cyhal_ethernet_etc.h"

#include "cyhal_ethernet_gmac_dma.h"
#include "cyhal_ethernet_gmac_dma_funcs.h"


/* chip interrupt bit error summary */
#define GMAC_I_ERRORS               (I_PDEE | I_PDE | I_DE | I_RDU | I_RFO | I_XFU)

#define GMAC_DEFAULT_INT_MASK       (I_XI0 | I_XI1 | I_XI2 | I_XI3 | I_RI | GMAC_I_ERRORS)

#define GMAC_RESET_DELAY            2

#define GMAC_MIN_FRAMESIZE         17    /* gmac can only send frames of
                                          * size above 17 octetes.
                                          */

#define GMAC_DMAREG(ch, dir, qnum)    ((dir == GMAC_DMA_TX) ? \
                                       (void *)(uintptr)&(ch->regs->dmaregs[qnum].dmaxmt) : \
                                       (void *)(uintptr)&(ch->regs->dmaregs[qnum].dmarcv))

#define GMAC_NS_COREREV(rev) ((rev == 4) || (rev == 5) || (rev == 7))

#define GMAC_USE_MISC_PLL(ch) \
        (((ch)->etc->corerev > 2) && \
        !BCM4707_CHIP(CHIPID((ch)->sih->chip)) && \
        (CHIPID((ch)->sih->chip) != BCM43909_CHIP_ID))

/* private chip state */
struct bcmgmac {
    void                        *et;            /* pointer to et private state */
    etc_info_t                  *etc;           /* pointer to etc public (common) state */

    _cyhal_gmac_commonregs_t    *regscomm;      /* pointer to GMAC COMMON registers */
    gmacregs_t                  *regs;          /* pointer to chip registers */

    bool                        enable_bcast_recv;  /* If true, allow broadcast receive */

    uint32                      intstatus;      /* saved interrupt condition bits */
    uint32                      intmask;        /* current software interrupt mask */
    uint32                      def_intmask;    /* default interrupt mask */

    hnddma_t                    *di[_CYHAL_ETHERNET_DMA_TX_NUM_QUEUES];    /* dma engine software state TODO: Change from _CYHAL_GMAC_MALLOC for each to static RAM allocations */

    bool                        mibgood;        /* true once mib registers have been cleared */
    gmacmib_t                   mib;            /* mib statistic counters */
    _cyhal_gmac_si_t            *sih;           /* si utils handle */
};

/* 802.1d priority to traffic class mapping. queues correspond one-to-one
 * with traffic classes.
 */
uint32_t up2tc[NUMPRIO] = {
    TC_BE,      /* 0    BE    TC_BE    Best Effort */
    TC_BK,      /* 1    BK    TC_BK    Background */
    TC_BK,      /* 2    --    TC_BK    Background */
    TC_BE,      /* 3    EE    TC_BE    Best Effort */
    TC_CL,      /* 4    CL    TC_CL    Controlled Load */
    TC_CL,      /* 5    VI    TC_CL    Controlled Load */
    TC_VO,      /* 6    VO    TC_VO    Voice */
    TC_VO       /* 7    NC    TC_VO    Voice */
};


 void *
_cyhal_gmac_chip_attach(etc_info_t *etc, void *regsva)
{
    ch_t *ch;
    gmacregs_t *regs;
    uint16_t boardflags, boardtype;
    uint32_t flagbits = 0;

    UNUSED_PARAMETER(boardflags);
    UNUSED_PARAMETER(boardtype);

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_attach: regsva 0x%lx\n", etc->unit, (ulong)regsva));

    ch = (ch_t *)_CYHAL_GMAC_MALLOC(sizeof(struct bcmgmac));
    if (ch == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_attach: out of memory, tried to malloc %d bytes\n", etc->unit, sizeof(struct bcmgmac)));
        return (NULL);
    }
    bzero((char *)ch, sizeof(ch_t));

    ch->etc = etc;
    ch->et = etc->et;

    /* store the pointer to the sw mib */
    etc->mib = (void *)&ch->mib;

    /* get si handle */
    if ((ch->sih = _cyhal_gmac_si_attach(etc->deviceid, regsva, PCI_BUS, NULL, NULL, NULL)) == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_attach: _cyhal_gmac_si_attach error\n", etc->unit));
        goto fail;
    }
    if (_cyhal_gmac_si_corerev(ch->sih) == GMAC_CORE_REV) {
        etc->corerev = GMAC_CORE_REV;
        if ((ch->regscomm = (_cyhal_gmac_commonregs_t *)_cyhal_gmac_si_setcore(ch->sih,
            GMAC_COMMON_4706_CORE_ID, 0)) == NULL) {
            _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_attach: Could not setcore to GMAC common\n",
                etc->unit));
            goto fail;
        }
    }
    if ((regs = (gmacregs_t *)_cyhal_gmac_si_setcore(ch->sih, GMAC_CORE_ID, etc->unit)) == NULL) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_attach: Could not setcore to GMAC\n", etc->unit));
        goto fail;
    }
    if (etc->corerev != GMAC_4706B0_CORE_REV)
        etc->corerev = _cyhal_gmac_si_corerev(ch->sih);

    ch->regs = regs;
    etc->chip = ch->sih->chip;
    etc->chiprev = ch->sih->chiprev;
    etc->chippkg = ch->sih->chippkg;
    etc->coreid = _cyhal_gmac_si_coreid(ch->sih);
    etc->nicmode = !(ch->sih->bustype == SI_BUS);
    etc->boardflags = getintvar(ch->vars, "boardflags");

    boardflags = etc->boardflags;
    boardtype = ch->sih->boardtype;

    /* Backplane clock ticks per microsecs: used by gptimer, intrecvlazy */
    etc->bp_ticks_usec = _cyhal_gmac_si_clock(ch->sih) / 1000000;

    /* set our local ether and phy addresses */
    if (!_cyhal_ethernet_et_set_addrs(etc)) { /* WICED: move address configuration to WICED part of driver */
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_attach: address configuratiin failed\n", etc->unit));
        goto fail;
    }

    /* configure pci core */
    _cyhal_gmac_si_pci_setup(ch->sih, (1 << _cyhal_gmac_si_coreidx(ch->sih)));

    /* Northstar, take all GMAC cores out of reset */
    if (BCM4707_CHIP(CHIPID(ch->sih->chip))) {
        int ns_gmac;
        for (ns_gmac = 0; ns_gmac < MAX_GMAC_CORES_4707; ns_gmac++) {
            /* As northstar requirement, we have to reset all GAMCs before
             * accessing them. et_probe() call pci_enable_device() for etx
             * and do _cyhal_gmac_si_core_reset for GAMCx only.     Then the other three
             * GAMCs didn't reset.  We do it here.
             */
            _cyhal_gmac_si_setcore(ch->sih, GMAC_CORE_ID, ns_gmac);
            if (!_cyhal_gmac_si_iscoreup(ch->sih)) {
                _CYHAL_ETHERNET_LOG_INFO(("et%d: reset GMAC[%d] core\n", etc->unit, ns_gmac));
                _cyhal_gmac_si_core_reset(ch->sih, flagbits, 0);
            }
        }
        _cyhal_gmac_si_setcore(ch->sih, GMAC_CORE_ID, etc->unit);
    }
    /* reset the gmac core */
    _cyhal_gmac_chip_reset(ch);

    uint16_t i;
    char name[18];

    /* dma attach */
    snprintf(name, sizeof(name), "etc%d", etc->unit);

    /* allocate dma resources for txqs */
    /* TX: TC_BK, RX: RX_Q0 */
    ch->di[0] = _cyhal_gmac_dma_attach(name, ch->sih,                  /* ch is struct bcmgmac,   */
                           GMAC_DMAREG(ch, GMAC_DMA_TX, _CYHAL_ETHERNET_DMA_TX_CHAN_Q0),           /* ch->regs->dmaregs[CYHAL_ETHERNET_DMA_TX_CHAN_Q0].dmaxmt */
                           GMAC_DMAREG(ch, GMAC_DMA_RX, RX_Q0),           /* ch->regs->dmaregs[RX_Q0].dmarcv */
                           _CYHAL_ETHERNET_DESCNUM_TX,           /*    8 DMA Tx descriptors */
                           _CYHAL_ETHERNET_DESCNUM_RX,           /*    4 DMA Rx descriptors */
                           _CYHAL_ETHERNET_RX_BUFFER_SIZE,       /* 1580 Rx Buffer Size */
                           0,                                   /*    0 extra headroom in buffer */
                           _CYHAL_ETHERNET_NUM_RX_BUFFER_POSTED, /*    2 try to keep this # Rx buffs posted to the chip */
                           _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET);                             /*   32 */

    /* TX: TC_BE, RX: UNUSED */
    ch->di[1] = _cyhal_gmac_dma_attach(name, ch->sih,
                           GMAC_DMAREG(ch, GMAC_DMA_TX, _CYHAL_ETHERNET_DMA_TX_CHAN_Q1),
                           NULL /* rxq unused */,
                           _CYHAL_ETHERNET_DESCNUM_TX , 0, 0, 0, 0, 0);

    /* TX: TC_CL, RX: UNUSED */
    ch->di[2] = _cyhal_gmac_dma_attach(name, ch->sih,
                           GMAC_DMAREG(ch, GMAC_DMA_TX, _CYHAL_ETHERNET_DMA_TX_CHAN_Q2),
                           NULL /* rxq unused */,
                           _CYHAL_ETHERNET_DESCNUM_TX , 0, 0, 0, 0, 0);

    /* TX: TC_VO, RX: UNUSED */
    ch->di[3] = _cyhal_gmac_dma_attach(name, ch->sih,
                           GMAC_DMAREG(ch, GMAC_DMA_TX, _CYHAL_ETHERNET_DMA_TX_CHAN_Q3),
                           NULL /* rxq unused */,
                           _CYHAL_ETHERNET_DESCNUM_TX , 0, 0, 0, 0, 0);

    for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++)
        if (ch->di[i] == NULL) {
            _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_attach: dma_attach failed\n", etc->unit));
            goto fail;
        }

    for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++)
        if (ch->di[i] != NULL)
            etc->txavail[i] = (uint32_t *)&ch->di[i]->txavail;

    /* override dma parameters, corerev 4 dma channel 1,2 and 3 default burstlen is 0. */
    /* corerev 4,5: NS Ax; corerev 6: BCM43909 no HW prefetch; corerev 7: NS B0 */
    if (etc->corerev == 4 ||
        etc->corerev == 5 ||
        etc->corerev == 7) {
#define DMA_CTL_TX 0
#define DMA_CTL_RX 1

#define DMA_CTL_MR 0
#define DMA_CTL_PC 1
#define DMA_CTL_PT 2
#define DMA_CTL_BL 3
        static uint16_t dmactl[2][4] = {
                        /* TX */
                        { DMA_MR_2, DMA_PC_16, DMA_PT_8, DMA_BL_1024 },
                        { 0, DMA_PC_16, DMA_PT_8, DMA_BL_128 },
                    };

        dmactl[DMA_CTL_TX][DMA_CTL_MR] = (_CYHAL_ETHERNET_DMA_TX_OUTSTAND_READS == 2 ? DMA_MR_2 : DMA_MR_1);

        if (etc->corerev == 7) {
            /* NS B0 can only be configured to DMA_PT_1 and DMA_PC_4 */
            dmactl[DMA_CTL_TX][DMA_CTL_PT] = DMA_PT_1;
            dmactl[DMA_CTL_TX][DMA_CTL_PC] = DMA_PC_4;
        } else {
            dmactl[DMA_CTL_TX][DMA_CTL_PT] = (_CYHAL_ETHERNET_DMA_TX_PREFETCH_THRESHOLD == 8 ? DMA_PT_8 :
                _CYHAL_ETHERNET_DMA_TX_PREFETCH_THRESHOLD == 4 ? DMA_PT_4 :
                _CYHAL_ETHERNET_DMA_TX_PREFETCH_THRESHOLD == 2 ? DMA_PT_2 : DMA_PT_1);
            dmactl[DMA_CTL_TX][DMA_CTL_PC] = (_CYHAL_ETHERNET_DMA_TX_PREFETCH_DESCRIPTORS == 16 ? DMA_PC_16 :
                _CYHAL_ETHERNET_DMA_TX_PREFETCH_DESCRIPTORS == 8 ? DMA_PC_8 :
                _CYHAL_ETHERNET_DMA_TX_PREFETCH_DESCRIPTORS == 4 ? DMA_PC_4 : DMA_PC_0);
        }

        dmactl[DMA_CTL_TX][DMA_CTL_BL] = (_CYHAL_ETHERNET_DMA_TX_BURST_LENGTH == 1024 ? DMA_BL_1024 :
                                          _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH == 512 ? DMA_BL_512 :
                                          _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH == 256 ? DMA_BL_256 :
                                          _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH == 128 ? DMA_BL_128 :
                                          _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH == 64 ? DMA_BL_64 :
                                          _CYHAL_ETHERNET_DMA_TX_BURST_LENGTH == 32 ? DMA_BL_32 : DMA_BL_16);

        dmactl[DMA_CTL_RX][DMA_CTL_PT] =  (_CYHAL_ETHERNET_DMA_RX_PREFETCH_THRESHOLD == 8 ? DMA_PT_8 :
                                           _CYHAL_ETHERNET_DMA_RX_PREFETCH_THRESHOLD == 4 ? DMA_PT_4 :
                                           _CYHAL_ETHERNET_DMA_RX_PREFETCH_THRESHOLD == 2 ? DMA_PT_2 : DMA_PT_1);
        dmactl[DMA_CTL_RX][DMA_CTL_PC] = (_CYHAL_ETHERNET_DMA_RX_PREFETCH_DESCRIPTIONS == 16 ? DMA_PC_16 :
                                          _CYHAL_ETHERNET_DMA_RX_PREFETCH_DESCRIPTIONS == 8 ? DMA_PC_8 :
                                          _CYHAL_ETHERNET_DMA_RX_PREFETCH_DESCRIPTIONS == 4 ? DMA_PC_4 : DMA_PC_0);
        dmactl[DMA_CTL_RX][DMA_CTL_BL] = (_CYHAL_ETHERNET_DMA_RX_BURST_LENGTH == 1024 ? DMA_BL_1024 :
                                          _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH == 512 ? DMA_BL_512 :
                                          _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH == 256 ? DMA_BL_256 :
                                          _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH == 128 ? DMA_BL_128 :
                                          _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH == 64 ? DMA_BL_64 :
                                          _CYHAL_ETHERNET_DMA_RX_BURST_LENGTH == 32 ? DMA_BL_32 : DMA_BL_16);

        for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++) {
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_TX_MULTI_OUTSTD_RD,
                              dmactl[DMA_CTL_TX][DMA_CTL_MR]);
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_TX_PREFETCH_CTL,
                              dmactl[DMA_CTL_TX][DMA_CTL_PC]);
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_TX_PREFETCH_THRESH,
                              dmactl[DMA_CTL_TX][DMA_CTL_PT]);
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_TX_BURSTLEN,
                              dmactl[DMA_CTL_TX][DMA_CTL_BL]);
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_RX_PREFETCH_CTL,
                              dmactl[DMA_CTL_RX][DMA_CTL_PC]);
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_RX_PREFETCH_THRESH,
                              dmactl[DMA_CTL_RX][DMA_CTL_PT]);
                _cyhal_gmac_dma_param_set(ch->di[i], HNDDMA_PID_RX_BURSTLEN,
                              dmactl[DMA_CTL_RX][DMA_CTL_BL]);
        }
    }
    /* set default sofware intmask */
    snprintf(name, sizeof(name), "et%d_no_txint", etc->unit);
    if (getintvar(ch->vars, name)) {
        /* if no_txint variable is non-zero we disable tx interrupts.
         * we do the tx buffer reclaim once every few frames.
         */
        ch->def_intmask = (GMAC_DEFAULT_INT_MASK & ~(I_XI0 | I_XI1 | I_XI2 | I_XI3));
        etc->txrec_thresh = (((_CYHAL_ETHERNET_DESCNUM_TX  >> 2) > _CYHAL_ETHERNET_DMA_TX_RECORD_THRESHOLD) ? _CYHAL_ETHERNET_DMA_TX_RECORD_THRESHOLD - 1 : 1);
    }
    else
    {
        ch->def_intmask = GMAC_DEFAULT_INT_MASK;
    }

    ch->intmask = ch->def_intmask;

    /* reset the external phy */
    if (_cyhal_ethernet_et_phy_reset(etc))
    {
        /* if external phy is present enable auto-negotation and
         * advertise full capabilities as default config.
         */
        CY_ASSERT(etc->phyaddr != EPHY_NOREG);
        etc->needautoneg = TRUE;
        etc->advertise = (EPHY_ADV_100FULL | EPHY_ADV_100HALF | EPHY_ADV_10FULL | EPHY_ADV_10HALF);
        etc->advertise2 = EPHY_ADV_1000FULL;
    }

    /* reset phy: reset it once now */
    _cyhal_gmac_chip_phyreset(ch, etc->phyaddr);

    return ((void *) ch);

fail:
    _cyhal_gmac_chip_detach(ch);
    return (NULL);
}

 void
_cyhal_gmac_chip_detach(ch_t *ch)
{

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_detach\n", ch->etc->unit));

    if (ch == NULL)
        return;

    int32 i;
    /* free dma state in reverse order */
    for (i = (_CYHAL_ETHERNET_DMA_TX_NUM_QUEUES - 1); i >=0 ; i--)
        if (ch->di[i] != NULL) {
            _cyhal_gmac_dma_detach(ch->di[i]);
            ch->di[i] = NULL;
        }

    /* put the core back into reset */
    /* For Northstar, should not disable any GMAC core */
    if (ch->sih && !BCM4707_CHIP(CHIPID(ch->sih->chip)))
        _cyhal_gmac_si_core_disable(ch->sih, 0);

    ch->etc->mib = NULL;

    /* free si handle */
    _cyhal_gmac_si_detach(ch->sih);
    ch->sih = NULL;

    /* free chip private state */
    _CYHAL_GMAC_MFREE(ch, sizeof(ch_t));
}

uint16_t
_cyhal_gmac_chip_macrd(ch_t *ch, uint16_t offset)
{
    CY_ASSERT(offset < 4096); /* GMAC Register space is 4K size */
    uint16_t value = _CYHAL_GMAC_R_REG( (uint16_t *)((uint)(ch->regs) + offset));
    return value;
}

 void
_cyhal_gmac_chip_macwr(ch_t *ch, uint16_t offset, uint16_t val)
{
    CY_ASSERT(offset < 4096); /* GMAC Register space is 4K size */
    _CYHAL_GMAC_W_REG( (uint16_t *)((uint)(ch->regs) + offset), val);
}

 uint16_t
_cyhal_gmac_chip_dump(ch_t *ch)
{
     CY_UNUSED_PARAMETER(ch);
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
//    int32 i;

    _CYHAL_ETHERNET_LOG_INFO(("regs 0x%lx ch->intstatus 0x%x intmask 0x%x\n",
        (ulong)ch->regs, ch->intstatus, ch->intmask));
    _CYHAL_ETHERNET_LOG_INFO(("\n"));

    /* dma engine state */
//    for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++) {
//        dma_dump(ch->di[i], b, TRUE);
//        _CYHAL_ETHERNET_LOG_INFO(("\n"));
//    }

    /* registers */
    _cyhal_gmac_chip_dumpregs(ch, ch->regs);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));

    /* switch registers */
    return b->size;
#else
    return 0;
#endif    /* _CYHAL_ETHERNET_LOG_ENABLE */
}

#ifdef _CYHAL_ETHERNET_LOG_ENABLE

#define    PRREG(name)    _CYHAL_ETHERNET_LOG_INFO((#name " 0x%x ", _CYHAL_GMAC_R_REG( &regs->name))
#define    PRMIBREG(name)    _CYHAL_ETHERNET_LOG_INFO((#name " 0x%x ", _CYHAL_GMAC_R_REG( &regs->mib.name))

 void
_cyhal_gmac_chip_dumpregs(ch_t *ch, gmacregs_t *regs)
{
    uint16_t phyaddr;

    phyaddr = ch->etc->phyaddr;

    PRREG(devcontrol); PRREG(devstatus);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(biststatus);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(intstatus); PRREG(intmask); PRREG(gptimer);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(intrecvlazy);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(flowctlthresh); PRREG(wrrthresh); PRREG(_cyhal_gmac_idle_cnt_thresh);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    if (ch->etc->corerev != GMAC_4706B0_CORE_REV) {
        PRREG(phyaccess); PRREG(phycontrol);
        _CYHAL_ETHERNET_LOG_INFO(("\n"));
    }
    PRREG(txqctl); PRREG(rxqctl);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(gpioselect); PRREG(gpio_output_en);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(clk_ctl_st); PRREG(pwrctl);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));

    /* unimac registers */
    if (!BCM4707_CHIP(CHIPID(ch->sih->chip))) {
        /* BCM4707 doesn't has unimacversion register */
        PRREG(unimacversion);
    }
    PRREG(hdbkpctl);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(cmdcfg);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(macaddrhigh); PRREG(macaddrlow);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(rxmaxlength); PRREG(pausequanta); PRREG(macmode);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(outertag); PRREG(innertag); PRREG(txipg); PRREG(pausectl);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRREG(txflush); PRREG(rxstatus); PRREG(txstatus);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));

    /* mib registers */
    PRMIBREG(tx_good_octets); PRMIBREG(tx_good_pkts); PRMIBREG(tx_octets); PRMIBREG(tx_pkts);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRMIBREG(tx_broadcast_pkts); PRMIBREG(tx_multicast_pkts);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRMIBREG(tx_jabber_pkts); PRMIBREG(tx_oversize_pkts); PRMIBREG(tx_fragment_pkts);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRMIBREG(tx_underruns); PRMIBREG(tx_total_cols); PRMIBREG(tx_single_cols);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRMIBREG(tx_multiple_cols); PRMIBREG(tx_excessive_cols); PRMIBREG(tx_late_cols);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    if (ch->etc->corerev != GMAC_4706B0_CORE_REV) {
        PRMIBREG(tx_defered); PRMIBREG(tx_carrier_lost); PRMIBREG(tx_pause_pkts);
        _CYHAL_ETHERNET_LOG_INFO(("\n"));
    }

    PRMIBREG(rx_good_octets); PRMIBREG(rx_good_pkts); PRMIBREG(rx_octets); PRMIBREG(rx_pkts);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRMIBREG(rx_broadcast_pkts); PRMIBREG(rx_multicast_pkts);
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    PRMIBREG(rx_jabber_pkts);
    if (ch->etc->corerev != GMAC_4706B0_CORE_REV) {
        PRMIBREG(rx_oversize_pkts); PRMIBREG(rx_fragment_pkts);
        _CYHAL_ETHERNET_LOG_INFO(("\n"));
        PRMIBREG(rx_missed_pkts); PRMIBREG(rx_crc_align_errs); PRMIBREG(rx_undersize);
    }
    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    if (ch->etc->corerev != GMAC_4706B0_CORE_REV) {
        PRMIBREG(rx_crc_errs); PRMIBREG(rx_align_errs); PRMIBREG(rx_symbol_errs);
        _CYHAL_ETHERNET_LOG_INFO(("\n"));
        PRMIBREG(rx_pause_pkts); PRMIBREG(rx_nonpause_pkts);
        _CYHAL_ETHERNET_LOG_INFO(("\n"));
    }
    if (phyaddr != EPHY_NOREG) {
        /* print a few interesting phy registers */
        _CYHAL_ETHERNET_LOG_INFO(("phy0 0x%x phy1 0x%x phy2 0x%x phy3 0x%x\n",
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 0),
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 1),
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 2),
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 3)));
        _CYHAL_ETHERNET_LOG_INFO(("phy4 0x%x phy5 0x%x phy24 0x%x phy25 0x%x\n",
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 4),
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 5),
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 24),
                       _cyhal_gmac_chip_phyrd(ch, phyaddr, 25)));
    }

}
#endif    /* _CYHAL_ETHERNET_LOG_ENABLE */

 void
_cyhal_gmac_clearmib(ch_t *ch)
{
    volatile uint32_t *ptr;

    /* XXX Skip temporarily for 4706 */
    if (ch->etc->corerev == GMAC_4706B0_CORE_REV)
        return;

    /* enable clear on read */
    _CYHAL_GMAC_OR_REG( &ch->regs->devcontrol, DC_MROR);

    ptr = &ch->regs->mib.tx_good_octets;
    while(ptr <= &ch->regs->mib.rx_uni_pkts)
    {
        (void)_CYHAL_GMAC_R_REG( ptr);
        if (ptr == &ch->regs->mib.tx_q3_octets_high)
        {
            ptr++;
        }
        ptr++;
    }

    return;
}

 void
_cyhal_gmac_init_reset(ch_t *ch)
{
    _CYHAL_GMAC_OR_REG( &ch->regs->cmdcfg, CC_SR(ch->etc->corerev));
    _CYHAL_GMAC_OSL_DELAY(GMAC_RESET_DELAY);
}

 void
_cyhal_gmac_clear_reset(ch_t *ch)
{
    _CYHAL_GMAC_AND_REG( &ch->regs->cmdcfg, ~CC_SR(ch->etc->corerev));
    _CYHAL_GMAC_OSL_DELAY(GMAC_RESET_DELAY);
}

/* Only called from _cyhal_gmac_chip_reset()
 * _cyhal_ethernet_et_init()->_cyhal_ethernet_et_reset()->_cyhal_ethernet_etc_reset()->(chops)->reset
 * _cyhal_ethernet_etc_down(, reset)->_cyhal_ethernet_et_reset()
 *
 */
 static void
_cyhal_gmac_reset(ch_t *ch)
{
    uint32_t ocmdcfg, cmdcfg;

    /* put the mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* initialize default config */
    ocmdcfg = cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* Disable / clear
     *  CC_TE     Tx Enable
     *  CC_RE     Rx Enable
     *  CC_RPI    Rx Pause Ignore
     *  CC_TAI    GMAC set our mac
     *  CC_HD     Half Duplex
     *  CC_ML     Local loopback
     *  CC_CFE    MAC Control frames with any Opcode other than 0x0001
     *  CC_RL     Enable remote loopback on FIFO side
     *  CC_RED    rx_err_disc "PDF says currently not used"
     *  CC_PE     PDF says unused
     *  CC_TPI    Ignore Tx Pause Frame request
     *  CC_PAD_EN Frame Padding enable
     *  CC_PF     Pause Fwd
     *  CC_AT     If set then out-of-band egress flow control is enabled
     */
    cmdcfg &= ~(CC_TE | CC_RE | CC_RPI | CC_TAI | CC_HD | CC_ML | CC_PROM |
                CC_CFE | CC_RL | CC_RED | CC_PE | CC_TPI | CC_PAD_EN | CC_PF | CC_AT);
    /* Enable/Set
     *  CC_NLC    Payload Length check disable
     *  CC_CFE    MAC Control frames with any Opcode other than 0x0001
     *  CC_TPI    Ignore Tx Pause Frame request
     */
    cmdcfg |= (CC_NLC | CC_CFE | CC_TPI | CC_PROM);

    if (cmdcfg != ocmdcfg)
    {
        _CYHAL_GMAC_W_REG( &ch->regs->cmdcfg, cmdcfg);
    }

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

void
_cyhal_gmac_promisc(ch_t *ch, bool mode)
{
    uint32_t cmdcfg;

    cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* put the mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* enable or disable promiscuous mode */
    if (mode)
        cmdcfg |= CC_PROM;
    else
        cmdcfg &= ~CC_PROM;

    _CYHAL_GMAC_W_REG( &ch->regs->cmdcfg, cmdcfg);

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

bool _cyhal_gmac_get_promisc(ch_t *ch)
{
    uint32_t cmdcfg;
    cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    return (cmdcfg & CC_PROM) ? true : false;
}

 void
_cyhal_gmac_pause_quanta(ch_t *ch, const uint16_t quanta)
{
    uint32_t pausequanta;

    pausequanta = _CYHAL_GMAC_R_REG( &ch->regs->pausequanta);

    /* put the mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* set quanta value */
    pausequanta = quanta;

    _CYHAL_ETHERNET_LOG_INFO(("Set PAUSE QUANTA %p to: 0x%lx\n", &ch->regs->pausequanta, pausequanta));    // Set PAUSE QUANTA 0x18005818 to: quanta passed in

    /* COMMAND_CONFIG  0x18005808
     * 8 rx_pause_ignore    Ignore RX Pause Frame Quanta. If enabled (Set to '1') received pause frames are ignored by the MAC.
     *                      When disabled (Set to reset value '0') the transmit process is stopped for the amount of time specified
     *                      in the pause quanta received within the pause frame.
     *
     *  PAUSE_QUANT     0x18005818
     *  bit 15:0    pause_quant         Receive Pause Quanta (RW). 16-Bit value, sets, in increment of
     *                                  512 Ethernet bit times, the pause quanta used in each Pause Frame
     *                                  sent to the remote Ethernet device (reset value 0x0000ffff)
     */
    _CYHAL_GMAC_W_REG( &ch->regs->pausequanta, pausequanta);

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

 void
_cyhal_gmac_start_pause(ch_t *ch, const bool start)
{
    uint32_t pause;

    pause = _CYHAL_GMAC_R_REG( &ch->regs->pausectl);

    /* put the mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* set pause on/off value */
    if (start)
    {
        pause = 0x0000ffff; // reset value
    }
    else
    {
        pause = 0;
    }

    _CYHAL_ETHERNET_LOG_INFO(("Set PAUSE %p to: 0x%lx\n", &ch->regs->pausectl, pause));
    /* 17 pause_control_en  Repetitive pause frame send enable. When enabled
     *                      and Back Pressured CY_ASSERTed, pauses are transmitted
     *                      when PAUSE_TIMER value expires.
     *  16:0 pause_timer    Pause timer value for repetitive pause frames
     *
     *
     */
    _CYHAL_GMAC_W_REG( &ch->regs->pausectl, pause); // Set PAUSE 0x18005b30 to: pause value

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

 int
_cyhal_gmac_speed(ch_t *ch, uint32_t speed)
{
    uint32_t cmdcfg;
    uint32_t hd_ena = 0;

    switch (speed) {
        case GMAC_FORCE_SPEED_10HALF:
            hd_ena = CC_HD;
            /* FALLTHRU */

        case GMAC_FORCE_SPEED_10FULL:
            speed = 0;
            break;

        case GMAC_FORCE_SPEED_100HALF:
            hd_ena = CC_HD;
            /* FALLTHRU */

        case GMAC_FORCE_SPEED_100FULL:
            speed = 1;
            break;

        case GMAC_FORCE_SPEED_1000FULL:
            speed = 2;
            break;

        case GMAC_FORCE_SPEED_1000HALF:
            _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_speed: supports 1000 mbps full duplex only\n",
                      ch->etc->unit));
            return (FAILURE);

        case GMAC_FORCE_SPEED_2500FULL:
            speed = 3;
            break;

        default:
            _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_speed: speed %ld not supported\n",
                      ch->etc->unit, speed));
            return (FAILURE);
    }

    cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* put mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* set the speed */
    cmdcfg &= ~(CC_ES_MASK | CC_HD);
    cmdcfg |= ((speed << CC_ES_SHIFT) | hd_ena);

    _CYHAL_GMAC_W_REG( &ch->regs->cmdcfg, cmdcfg);

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);

    return (SUCCESS);
}

 void
_cyhal_gmac_macloopback(ch_t *ch, uint32_t mode)
{
    uint32_t ocmdcfg, cmdcfg;

    ocmdcfg = cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* put mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* clear previous mode */
    cmdcfg &= ~(CC_ML | CC_RL);

    /* set the mac loopback mode */
    if (mode == _CYHAL_ETHERNET_LOOPBACK_MODE_ENABLE)   /* Loopback the packet at MAC (GMII/MII loopback) */
    {
        cmdcfg |= CC_ML;
    }

    if (cmdcfg != ocmdcfg)
    {
        _CYHAL_ETHERNET_LOG_NOTICE(("--- _cyhal_gmac_macloopback: change addr %p with 0x%lx  (0x%lx) for mode: %ld\n", &ch->regs->cmdcfg, cmdcfg, (cmdcfg & (CC_ML | CC_RL | CC_MLCON) ), mode));
        _CYHAL_GMAC_W_REG( &ch->regs->cmdcfg, cmdcfg);
    }

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

int
_cyhal_gmac_loopback(ch_t *ch, uint32_t mode)
{
    /* All called functions check for mode level */
    switch (mode)
    {
    case _CYHAL_ETHERNET_LOOPBACK_MODE_NONE:
    case _CYHAL_ETHERNET_LOOPBACK_MODE_ENABLE:
        _cyhal_gmac_macloopback(ch, mode);
        _cyhal_gmac_chip_phyloopback(ch, mode);
        _cyhal_gmac_dma_fifoloopbackenable(ch->di[_CYHAL_ETHERNET_DMA_TX_CHAN_Q0], mode);
        break;
    default:
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_loopback: Unknown loopback mode %ld\n", ch->etc->unit, mode));
        return (FAILURE);
    }

    return (SUCCESS);
}

 void
_cyhal_gmac_enable(ch_t *ch)
{
    uint32_t cmdcfg, rxqctl, bp_clk, mdp, mode;
    gmacregs_t *regs;

    regs = ch->regs;

    cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* put mac in reset */
    _cyhal_gmac_init_reset(ch);

    cmdcfg |= CC_SR(ch->etc->corerev);

    /* first deCY_ASSERT rx_ena and tx_ena while in reset */
    cmdcfg &= ~(CC_RE | CC_TE);
    _CYHAL_GMAC_W_REG( &regs->cmdcfg, cmdcfg);

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
    _CYHAL_GMAC_OSL_DELAY(2);

    /* enable the mac transmit and receive paths now */
    cmdcfg &= ~CC_SR(ch->etc->corerev);
    cmdcfg |= (CC_RE | CC_TE);

    /* CY_ASSERT rx_ena and tx_ena when out of reset to enable the mac */
    _CYHAL_GMAC_W_REG( &regs->cmdcfg, cmdcfg);

    /* WAR to not force ht for 47162 when gmac is in rev mii mode */
    mode = ((_CYHAL_GMAC_R_REG( &regs->devstatus) & DS_MM_MASK) >> DS_MM_SHIFT);
    if ((CHIPID((ch)->sih->chip) != BCM43909_CHIP_ID) && ((CHIPID(ch->sih->chip) != BCM47162_CHIP_ID) || (mode != 0)))
    {
        /* request ht clock */
        _CYHAL_GMAC_OR_REG( &regs->clk_ctl_st, CS_FH);
    }

    /* PR71045: WAR to bypass RXC DLL in RGMII mode */
    if (PMUCTL_ENAB(ch->sih) && (CHIPID(ch->sih->chip) == BCM47162_CHIP_ID) && (mode == 2))
        _cyhal_gmac_si_pmu_chipcontrol(ch->sih, PMU_CHIPCTL1, PMU_CC1_RXC_DLL_BYPASS,
                           PMU_CC1_RXC_DLL_BYPASS);

    /* Set the GMAC flowcontrol on and off thresholds and pause retry timer
     * the thresholds are tuned based on size of buffer internal to GMAC.
     * We are leaving the system reset values alone for CYW943907
     * They can be updated by calling API functions
     */

    /* To prevent any risk of the BCM4707 ROM mdp issue we saw in the QT,
     * we use the mdp register default value
     */
    if (!BCM4707_CHIP(CHIPID(ch->sih->chip))) {
        /* init the mac data period. the value is set according to expr
         * ((128ns / bp_clk) - 3).
         */
        rxqctl = _CYHAL_GMAC_R_REG( &regs->rxqctl);
        rxqctl &= ~RC_MDP_MASK;

        bp_clk = _cyhal_gmac_si_clock(ch->sih) / 1000000;
        mdp = ((bp_clk * 128) / 1000) - 3;
        _CYHAL_GMAC_W_REG( &regs->rxqctl, rxqctl | (mdp << RC_MDP_SHIFT));
    }

    return;
}

 void
_cyhal_gmac_txflowcontrol(ch_t *ch, bool on)
{
    uint32_t cmdcfg;

    cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* put the mac in reset */
    _cyhal_gmac_init_reset(ch);

    /* to enable tx flow control clear the rx pause ignore bit */
    if (on)
        cmdcfg &= ~CC_RPI;
    else
        cmdcfg |= CC_RPI;

    _CYHAL_GMAC_W_REG( &ch->regs->cmdcfg, cmdcfg);

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

 void
_cyhal_gmac_miiconfig(ch_t *ch)
{
    /* BCM4707 GMAC DevStatus register has different definition of "Interface Mode"
     * Bit 12:8  "interface_mode"  This field is programmed through IDM control bits [6:2]
     *
     * Bit 0 : SOURCE_SYNC_MODE_EN - If set, Rx line clock input will be used by Unimac for
     *          sampling data.If this is reset, PLL reference clock (Clock 250 or Clk 125 based
     *          on CLK_250_SEL) will be used as receive side line clock.
     * Bit 1 : DEST_SYNC_MODE_EN - If this is reset, PLL reference clock input (Clock 250 or
     *          Clk 125 based on CLK_250_SEL) will be used as transmit line clock.
     *          If this is set, TX line clock input (from external switch/PHY) is used as
     *          transmit line clock.
     * Bit 2 : TX_CLK_OUT_INVERT_EN - If set, this will invert the TX clock out of AMAC.
     * Bit 3 : DIRECT_GMII_MODE - If direct gmii is set to 0, then only 25 MHz clock needs to
     *          be fed at 25MHz reference clock input, for both 10/100 Mbps speeds.
     *          Unimac will internally divide the clock to 2.5 MHz for 10 Mbps speed
     * Bit 4 : CLK_250_SEL - When set, this selects 250Mhz reference clock input and hence
     *          Unimac line rate will be 2G.
     *          If reset, this selects 125MHz reference clock input.
     */
    if (BCM4707_CHIP(CHIPID(ch->sih->chip))) {
        if (ch->etc->phyaddr == EPHY_NOREG) {
            _cyhal_gmac_si_core_cflags(ch->sih, 0x44, 0x44);
            _cyhal_gmac_speed(ch, GMAC_FORCE_SPEED_2500FULL);
            ch->etc->speed = 2500;
            ch->etc->duplex = 1;
        }
    } else {
        uint32_t devstatus, mode;
        gmacregs_t *regs;

        regs = ch->regs;

        /* Read the devstatus to figure out the configuration
         * mode of the interface.
         */
        devstatus = _CYHAL_GMAC_R_REG( &regs->devstatus);
        mode = ((devstatus & DS_MM_MASK) >> DS_MM_SHIFT);

        /* Set the speed to 100 if the switch interface is
         * using mii/rev mii.
         */
        if ((mode == 0) || (mode == 1)) {
            if (ch->etc->forcespeed == GMAC_FORCE_SPEED_AUTO) {
                _cyhal_gmac_speed(ch, GMAC_FORCE_SPEED_100FULL);
                ch->etc->speed = 100;
                ch->etc->duplex = 1;
            } else
                _cyhal_gmac_speed(ch, ch->etc->forcespeed);
        } else {
            if (ch->etc->phyaddr == EPHY_NOREG) {
                ch->etc->speed = 1000;
                ch->etc->duplex = 1;
            }
        }
    }
}

 void
_cyhal_gmac_chip_reset(ch_t *ch)
{
    gmacregs_t *regs;
    uint32_t sflags, flagbits = 0;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_reset\n", ch->etc->unit));

    regs = ch->regs;

    if (!_cyhal_gmac_si_iscoreup(ch->sih)) {
        if (!ch->etc->nicmode)
            _cyhal_gmac_si_pci_setup(ch->sih, (1 << _cyhal_gmac_si_coreidx(ch->sih)));
        /* power on reset: reset the enet core */
        goto _cyhal_gmac_chip_inreset;
    }

    /* update software counters before resetting the _cyhal_gmac_chip_ */
    if (ch->mibgood)
        _cyhal_gmac_chip_statsupd(ch);

    int i;
    /* reset the tx dma engines */
    for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++) {
        if (ch->di[i]) {
            _CYHAL_ETHERNET_LOG_INFO(("et%d: resetting tx dma %d\n", ch->etc->unit, i));
            _cyhal_gmac_dma_txreset(ch->di[i]);
        }
    }

    /* set gmac into loopback mode to ensure no rx traffic */
    _cyhal_gmac_loopback(ch, _CYHAL_ETHERNET_LOOPBACK_MODE_ENABLE);
    _CYHAL_GMAC_OSL_DELAY(1);

    /* reset the rx dma engine */
    if (ch->di[RX_Q0]) {
        _CYHAL_ETHERNET_LOG_INFO(("et%d: resetting rx dma\n", ch->etc->unit));
        _cyhal_gmac_dma_rxreset(ch->di[RX_Q0]);
    }

_cyhal_gmac_chip_inreset:
    sflags = _cyhal_gmac_si_core_sflags(ch->sih, 0, 0);
    /* Do not enable internal switch for 47186/47188 */
    if ((((CHIPID(ch->sih->chip) == BCM5357_CHIP_ID) ||
          (CHIPID(ch->sih->chip) == BCM4749_CHIP_ID)) &&
         (ch->sih->chippkg == BCM47186_PKG_ID)) ||
        ((CHIPID(ch->sih->chip) == BCM53572_CHIP_ID) && (ch->sih->chippkg == BCM47188_PKG_ID)))
        sflags &= ~SISF_SW_ATTACHED;

    if (sflags & SISF_SW_ATTACHED) {
        _CYHAL_ETHERNET_LOG_INFO(("et%d: internal switch attached\n", ch->etc->unit));
        flagbits = SICF_SWCLKE;
        _CYHAL_ETHERNET_LOG_INFO(("et%d: reseting switch\n", ch->etc->unit));
        flagbits |= SICF_SWRST;
    }

    /* 3GMAC: for BCM4707, only do core reset at chipattach */
    if (CHIPID(ch->sih->chip) != BCM4707_CHIP_ID) {
        /* reset core */
        _cyhal_gmac_si_core_reset(ch->sih, flagbits, 0);
    }

    /* Request Misc PLL for corerev > 2 */
    if (GMAC_USE_MISC_PLL(ch)) {
        _CYHAL_GMAC_OR_REG( &regs->clk_ctl_st, CS_ER);
        SPINWAIT((_CYHAL_GMAC_R_REG( &regs->clk_ctl_st) & CS_ES) != CS_ES, 1000);
    }

    if ((CHIPID(ch->sih->chip) == BCM5357_CHIP_ID) ||
        (CHIPID(ch->sih->chip) == BCM4749_CHIP_ID) ||
        (CHIPID(ch->sih->chip) == BCM53572_CHIP_ID)) {
        char *var;
        uint32_t sw_type = PMU_CC1_SW_TYPE_EPHY | PMU_CC1_IF_TYPE_MII;

        if ((var = getvar(ch->vars, "et_swtype")) != NULL)
            sw_type = (atoi(var) & 0x0f) << 4;
        else if ((CHIPID(ch->sih->chip) == BCM5357_CHIP_ID) &&
                 (ch->sih->chippkg == BCM5358_PKG_ID))
            sw_type = PMU_CC1_SW_TYPE_EPHYRMII;
        else if (((CHIPID(ch->sih->chip) != BCM53572_CHIP_ID) &&
                  (ch->sih->chippkg == BCM47186_PKG_ID)) ||
                 ((CHIPID(ch->sih->chip) == BCM53572_CHIP_ID) &&
                  (ch->sih->chippkg == BCM47188_PKG_ID)) ||
                 (ch->sih->chippkg == HWSIM_PKG_ID))
            sw_type = PMU_CC1_IF_TYPE_RGMII|PMU_CC1_SW_TYPE_RGMII;

        _CYHAL_ETHERNET_LOG_INFO(("%s: sw_type %04lx\n", __FUNCTION__, sw_type));
        if (PMUCTL_ENAB(ch->sih)) {
            _cyhal_gmac_si_pmu_chipcontrol(ch->sih, PMU_CHIPCTL1,
                (PMU_CC1_IF_TYPE_MASK|PMU_CC1_SW_TYPE_MASK),
                sw_type);
        }
    }

    if (sflags & SISF_SW_ATTACHED)
    {
        _CYHAL_ETHERNET_LOG_INFO(("et%d: taking switch out of reset\n", ch->etc->unit));
        _cyhal_gmac_si_core_cflags(ch->sih, SICF_SWRST, 0);
    }

    /* reset gmac */
    _cyhal_gmac_reset(ch);

    /* clear mib */
    _cyhal_gmac_clearmib(ch);
    ch->mibgood = TRUE;

    if (ch->etc->corerev == GMAC_4706B0_CORE_REV)
    {
        _CYHAL_GMAC_OR_REG( &ch->regscomm->phycontrol, EPHY_PC_MTE);
    }
    else
    {
        _CYHAL_GMAC_OR_REG( &regs->phycontrol, EPHY_PC_MTE);
    }

    /* Read the devstatus to figure out the configuration mode of
     * the interface. Set the speed to 100 if the switch interface
     * is mii/rmii.
     */
    _cyhal_gmac_miiconfig(ch);

    if (OSL_ACP_WAR_ENAB() || OSL_ARCH_IS_COHERENT()) {
        uint32_t mask = (0xf << 16) | (0xf << 7) | (0x1f << 25) | (0x1f << 20);
        uint32_t val = (0xb << 16) | (0x7 << 7) | (0x1 << 25) | (0x1 << 20);
        /* _cyhal_gmac_si_core_cflags to change ARCACHE[19:16] to 0xb, and AWCACHE[10:7] to 0x7,
         * ARUSER[29:25] to 0x1, and AWUSER [24:20] to 0x1
         */
        _cyhal_gmac_si_core_cflags(ch->sih, mask, val);
    }

    /* Reset the PHY (gmac doesn't have internal phy) */
    _cyhal_gmac_chip_phyinit(ch, ch->etc->phyaddr);

    /* clear persistent sw intstatus */
    ch->intstatus = 0;
}

int _cyhal_ethernet_etc_broadcast_receive_enable(etc_info_t *etc, bool enable_bcast)
{
    ch_t *ch = etc->ch;

    if (ch == NULL)
    {
        return FAILURE;
    }

    if (ch->enable_bcast_recv != enable_bcast)
    {
        ch->enable_bcast_recv = enable_bcast;
    }

    return SUCCESS;
}

/*
 * Initialize all the chip registers.  If dma mode, init tx and rx dma engines
 * but leave the devcontrol tx and rx (fifos) disabled.
 */
 void
_cyhal_gmac_chip_init(ch_t *ch, uint16_t options)
{
    etc_info_t *etc;
    gmacregs_t *regs;
    uint32_t macaddrhigh;
    uint16_t macaddrlow;
    uint16_t i;

    regs = ch->regs;
    etc = ch->etc;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_init options: 0x%x\n", etc->unit, options));

    /* enable one rx interrupt per received frame */
    _CYHAL_GMAC_W_REG( &regs->intrecvlazy, (1 << IRL_FC_SHIFT));

    /* enable 802.3x tx flow control (honor received PAUSE frames) */
    _cyhal_gmac_txflowcontrol(ch, TRUE);

    /* set our local mac address */
    memcpy( (void *)&macaddrhigh, (void *)&etc->cur_etheraddr.octet[0], sizeof(macaddrhigh)); /* WICED: use memcpy() to get rid from strict-aliasing violation */
    memcpy( (void *)&macaddrlow, (void *)&etc->cur_etheraddr.octet[4], sizeof(macaddrlow));
    _CYHAL_GMAC_W_REG( &regs->macaddrhigh, hton32(macaddrhigh));
    _CYHAL_GMAC_W_REG( &regs->macaddrlow, hton16(macaddrlow));

    /* enable/disable promiscuous mode */
    _cyhal_gmac_promisc(ch, etc->promisc);

    /* enable / disable broadcast mode */
    _cyhal_ethernet_etc_broadcast_receive_enable(etc, etc->broadcast_enable);

    /* optionally enable loopback */
    _cyhal_gmac_loopback(ch, etc->loopbk);

    /* set max frame lengths - account for possible vlan tag */
    _CYHAL_GMAC_W_REG( &regs->rxmaxlength, CYHAL_ETHER_MAX_LEN + 32);

    /*
     * Optionally, disable phy autonegotiation and force our speed/duplex
     * or constrain our advertised capabilities.
     */
    if (etc->forcespeed != GMAC_FORCE_SPEED_AUTO) {
        _cyhal_gmac_speed(ch, etc->forcespeed);
        _cyhal_gmac_chip_phyforce(ch, etc->phyaddr);
    } else if (etc->advertise && etc->needautoneg)
        _cyhal_gmac_chip_phyadvertise(ch, etc->phyaddr);

    /* NS B0 only enables 4 entries x 4 channels */
    if (etc->corerev == 7)
    {
        _CYHAL_GMAC_OR_REG( &regs->pwrctl, 0x1);
    }

    /* enable the overflow continue feature and disable parity
     * MASK
     *  DMA_CTRL_ROC         rx overflow continue
     *  DMA_CTRL_PEN         partity enable
     *  DMA_CTRL_RXSINGLE    always single buffer
     *
     * ENABLE
     *  DMA_CTRL_ROC        rx overflow continue
     *  DMA_CTRL_RXSINGLE    always single buffer
     */
    _cyhal_gmac_dma_ctrlflags(ch->di[0], DMA_CTRL_ROC | DMA_CTRL_PEN | DMA_CTRL_RXSINGLE /* mask */,
                  DMA_CTRL_ROC | DMA_CTRL_RXSINGLE /* value */);

    if (options & ET_INIT_FULL)
    {
        /* initialize the tx and rx dma channels */
        for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++)
            _cyhal_gmac_dma_txinit(ch->di[i]);
        _cyhal_gmac_dma_rxinit(ch->di[RX_Q0]);

        /* post dma receive buffers */
        _cyhal_gmac_dma_rxfill(ch->di[RX_Q0]);

        /* lastly, enable interrupts */
        if (options & ET_INIT_INTRON)
            _cyhal_ethernet_et_intrson(etc->et);
    }
    else
    {
        _cyhal_gmac_dma_rxenable(ch->di[RX_Q0]);
    }

    /* turn on the emac */
    _cyhal_gmac_enable(ch);
}

/* dma transmit */
 bool BCMFASTPATH
_cyhal_gmac_chip_tx(ch_t *ch, void *p0, uint16_t size)
{
    int len;
    uint32_t q = _CYHAL_ETHERNET_DMA_TX_CHAN_Q0;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_tx\n", ch->etc->unit));
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_tx", ch->etc->unit, 0));

    len = size; /* WAS _CYHAL_GMAC_PKTLEN(p0)
                 *   _CYHAL_GMAC_PKTLEN is a _WEAK function expected to be written for each Network Stack.
                 *   Since we are trying to be lower than the network stack (and not integrated)
                 *   The buffer would have had a header with the length value in it.
                 *   Length argument added to function call to remove need for buffer.
                 */

    /* check tx max length */
    if (len > (CYHAL_ETHER_MAX_LEN + 32)) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_tx: max frame length exceeded\n",
                  ch->etc->unit));
//        _CYHAL_GMAC_PKTFREE(p0);  // We don't allocate Tx buffers
        return FALSE;
    }

    /* gmac rev 0 workaround:  unimac can only transmit frames of
     * length 17 bytes or greater. so pad the frame and send a
     * 17 byte frame. to do the padding just modify the packet
     * length that we provide to the dma. unimac does the extra
     * padding * required to send 64 byte frames.
     */
    if ((len < GMAC_MIN_FRAMESIZE) && (ch->etc->corerev == 0))
    {
        _CYHAL_GMAC_PKTSETLEN( p0, GMAC_MIN_FRAMESIZE);
    }

    /* queue the packet based on its priority */
    if (ch->etc->qos)
    {
        if (ch->etc->corerev != 4 && ch->etc->corerev != 5)
        {
            q = up2tc[_CYHAL_GMAC_PKTPRIO(p0)]; // _CYHAL_GMAC_PKTPRIO(p) always returns 0
        }
        else
        {
            /* XXX: for NS AX prefetch buffer corrupt problem */
            q = TC_BE;
        }
    }

    CY_ASSERT(q < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES);

    int error;
    /* if tx completion intr is disabled then do the reclaim
     * once every few frames transmitted.
     */
    if ((ch->etc->txframes[q] & ch->etc->txrec_thresh) == 1)
        _cyhal_gmac_dma_txreclaim(ch->di[q], HNDDMA_RANGE_TRANSMITTED);

    /* We don't want to transmit the preamble, folks! */
    uint8_t *data_to_send = &((uint8_t *)p0)[8];
    uint32_t size_to_send = size - 8;

    error = _cyhal_gmac_dma_txfast(ch->di[q], data_to_send, size_to_send, true);

    if (error) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_tx: out of txds\n", ch->etc->unit));
        ch->etc->txnobuf++;
        return FALSE;
    }

    ch->etc->txframes[q]++;

    return TRUE;
}

/* reclaim completed transmit descriptors and packets */
 void BCMFASTPATH
_cyhal_gmac_chip_txreclaim(ch_t *ch, bool forceall)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_txreclaim\n", ch->etc->unit));

    int32 i;
    for (i = 0; i < _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++) {
        if (*(uint16_t *)(ch->etc->txavail[i]) < _CYHAL_ETHERNET_DESCNUM_TX )
            _cyhal_gmac_dma_txreclaim(ch->di[i], forceall ? HNDDMA_RANGE_ALL :
                                                HNDDMA_RANGE_TRANSMITTED);
        ch->intstatus &= ~(I_XI0 << i);
    }
}

 bool _block_packet_due_to_broadcast_filtering(ch_t *ch, cyhal_ether_addr_t *da, cyhal_ether_addr_t *sa)
{
    bool block_packet = false;
    if ( CYHAL_ETHER_IS_BROADCAST(da) || CYHAL_ETHER_IS_MULTICAST(da) )
    {
        /* Are we allowing broadcast addresses ? */
        if (ch->enable_bcast_recv != true)
        {
            /* Only broadcast/multicast address in the filter list are allowed */
            if (_cyhal_gmac_broadcast_filter_pass(da, sa) != CY_RSLT_SUCCESS)
            {
                block_packet = true;
                _CYHAL_ETHERNET_LOG_INFO(("BLOCKED addr Broadcast enable is OFF and addr NOT in list %2x:%2x:%2x:%2x:%2x:%2x\n",
                        sa->octet[0], sa->octet[1], sa->octet[2], sa->octet[3], sa->octet[4], sa->octet[5]) );
            }
        }
    }

    return block_packet;
}

/* dma receive: returns a pointer to the next frame received, or NULL if there are no more */
 void * BCMFASTPATH
_cyhal_gmac_chip_rxpeek_and_drop(ch_t *ch)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx\n", ch->etc->unit));
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx", ch->etc->unit, 0));

    /* gmac doesn't have a cam to do address filtering. so we implement
     * the multicast address filtering here.
     */
    void *p;
    cyhal_ether_addr_t *da = NULL;
    cyhal_ether_addr_t *sa;

//    p = _cyhal_gmac_dma_peeknextrxp(ch->di[RX_Q0]);
//    if (p != NULL)

    /* peek through packets. If we find an invalid one, drop it */
    while ((p = _cyhal_gmac_dma_peeknextrxp(ch->di[RX_Q0])) != NULL)
    {
        /* skip the rx header */
        _CYHAL_GMAC_PKTPULL( p, _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET);

        /* do filtering using white filter */
        da = (cyhal_ether_addr_t *)_CYHAL_GMAC_PKTDATA( p);
        sa = &da[1];

        _CYHAL_GMAC_PKTPUSH( p, _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET);

        if (_block_packet_due_to_broadcast_filtering(ch, da, sa) == true)
        {
            /* get the packet and drop if */
            p = _cyhal_gmac_dma_rx(ch->di[RX_Q0]);
            _CYHAL_GMAC_PKTFREE(p);
            p = NULL;
        }
        else
        {
            break;
        }
    }

    return p;
}

/* dma receive: returns a pointer to the next frame received, or NULL if there are no more */
 void * BCMFASTPATH
_cyhal_gmac_chip_rx(ch_t *ch, uint32_t *size)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx\n", ch->etc->unit));
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx", ch->etc->unit, 0));

    /* gmac doesn't have a cam to do address filtering. so we implement
     * the multicast address filtering here.
     */
    void *p;
    cyhal_ether_addr_t *da = NULL;
    cyhal_ether_addr_t *sa;

    if (size != NULL)
    {
        *size = 0;
    }
    /* check through the dma channel if the channel has received data */
    while ((p = _cyhal_gmac_dma_rx(ch->di[RX_Q0])) != NULL)
    {
        bool drop_packet = false;

        /* Look at the rx header packet buffer stats */
        bcmgmacrxh_t* rxh = (bcmgmacrxh_t *)_CYHAL_GMAC_PKTDATA(p); /* packet buffer starts with rxhdr */
        uint16_t    rx_flags = rxh->flags;
        /* check for reported frame errors */
        if ( rx_flags & htol16(_CYHAL_ETHERNET_DMA_RX_FLAG_CRC_ERROR) )
        {
            drop_packet = true;
            ch->etc->rxcrcerr++;
        }
        if ( rx_flags & htol16(_CYHAL_ETHERNET_DMA_RX_FLAG_OVERFLOW) )
        {
            drop_packet = true;
            ch->etc->rxoflodiscards++;
        }
        if ( rx_flags & htol16(_CYHAL_ETHERNET_DMA_RX_FLAG_OVERSIZED) )
        {
            drop_packet = true;
            ch->etc->rxovrsize++;
        }
        /* If Unicast check for promiscuous enabled
         *    Multi or Broadcast, check for boradcast enabled
         */
        switch(rx_flags & htol16(GMAC_GRXF_PACKET_TYPE_MASK) )
        {
        case 0:
            /* If GMAC HW lets this Unicast packet through, promiscuous check has passed */
            _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx, Rx packet type %s\n", ch->etc->unit, "0 Unicast"));
            break;
        case 1:
        case 2:
            _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx, Rx packet type %s\n", ch->etc->unit,
                     (rx_flags & htol16(GMAC_GRXF_PACKET_TYPE_MASK) == 1) ? "1 Multicast" : "2 Broadcast" ));

            /* do filtering using white filters */
            _CYHAL_GMAC_PKTPULL(p, _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET );
            da = (cyhal_ether_addr_t *)_CYHAL_GMAC_PKTDATA( p);
            sa = &da[1];
            _CYHAL_GMAC_PKTPUSH(p, _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET );

            if (_block_packet_due_to_broadcast_filtering(ch, da, sa) == true)
            {
                drop_packet = true;
                if ( CYHAL_ETHER_IS_BROADCAST(da) )
                {
                    ch->etc->dropped_bcast_cnt++;
                }
            }
            break;
        default:
            _CYHAL_ETHERNET_LOG_NOTICE(("Rx packet type UNKNOWN\n"));
            break;
        }

        /* drop the Rx frames with errors */
        if ( (rx_flags & htol16( _CYHAL_ETHERNET_DMA_RX_FLAG_CRC_ERROR | _CYHAL_ETHERNET_DMA_RX_FLAG_OVERFLOW | _CYHAL_ETHERNET_DMA_RX_FLAG_OVERSIZED ) ) ||
             (drop_packet == true) )
        {
            _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_rx, CRC, Overflow or Oversize\n", ch->etc->unit));
            _CYHAL_GMAC_PKTFREE(p);
            ch->etc->rxerror++;
            continue;
        }
        else
        {
            *size = _CYHAL_GMAC_PKTLEN(p) - sizeof(uint32_t);   /* reduce packet length reported by the Receive HEADER and CRC at the end. */
            break;
        }
    }

    ch->intstatus &= ~I_RI;

    /* post more rx buffers since we may have consumed one */
    _cyhal_gmac_dma_rxfill(ch->di[RX_Q0]);

    return (p);
}

 int BCMFASTPATH /* dma receive quota number of pkts */
_cyhal_gmac_chip_rxquota(ch_t *ch, int quota, void **rxpkts)
{
    int             rxcnt = 0;

    uint8_t         *rxh;
    void            *pkt;
    hnddma_t        *di_rx_q0;

    CY_UNUSED_PARAMETER(rxh);

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rxquota %d\n", ch->etc->unit, quota));
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rxquota", ch->etc->unit, 0));

    rxcnt = 0;
    di_rx_q0 = ch->di[RX_Q0];


    /* Fetch quota number of pkts (or ring empty) */
    /* dma_rx: prefetches _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET=30B for applying PR77137: len update chk */
    while ((rxcnt < quota) && ((pkt = _cyhal_gmac_dma_rx(di_rx_q0)) != NULL))
    {
        rxh = _CYHAL_GMAC_PKTDATA( pkt); /* start of pkt data */

#if (defined(__mips__) || defined(BCM47XX_CA9) || defined(BCM_CPU_PREFETCH)) && !defined(_CFE_)
        _cyhal_gmac_prefetch_32B(rxh + 32, 1); /* skip 30B of _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET */
#endif /* !_CFE_ */

        rxpkts[rxcnt] = pkt;
        rxcnt++;
    }

    {
        cyhal_ether_addr_t *da;
        cyhal_ether_addr_t *sa;
        int nrx;
        int mfpass = 0; /* pkts that passed mf lkup */

        for (nrx = 0; nrx < rxcnt; nrx++) {
            pkt = rxpkts[nrx];
            da = (cyhal_ether_addr_t *)(_CYHAL_GMAC_PKTDATA( pkt) + _CYHAL_ETHERNET_DMA_RX_HW_HDR_OFFSET);
            sa = &da[1];
            if (_block_packet_due_to_broadcast_filtering(ch, da, sa) == false)
            {
                rxpkts[mfpass++] = pkt; /* repack rxpkts array */
            }
            else
            {
                _CYHAL_GMAC_PKTFREE(pkt);
            }
        }
        rxcnt = mfpass;
    }

    /* post more rx buffers since we consumed a few */
    _cyhal_gmac_dma_rxfill(di_rx_q0);

    if (rxcnt < quota) { /* ring is "possibly" empty, enable et interrupts */
        ch->intstatus &= ~I_RI;
    }

    return rxcnt; /* rxpkts[] has rxcnt number of pkts to be processed */
}

 void BCMFASTPATH
_cyhal_gmac_chip_rxlazy(ch_t *ch)
{
    uint16_t reg_val = ((ch->etc->rxlazy_timeout & IRL_TO_MASK) |
                    (ch->etc->rxlazy_framecnt << IRL_FC_SHIFT));
    _CYHAL_GMAC_W_REG( &ch->regs->intrecvlazy, reg_val);
}


/* reclaim completed dma receive descriptors and packets */
 void
_cyhal_gmac_chip_rxreclaim(ch_t *ch)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rxreclaim\n", ch->etc->unit));

    _cyhal_gmac_dma_rxreclaim(ch->di[RX_Q0]);

    ch->intstatus &= ~I_RI;
}

/* calculate the number of free dma receive descriptors */
 uint16_t BCMFASTPATH
_cyhal_gmac_chip_activerxbuf(ch_t *ch)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_activerxbuf\n", ch->etc->unit));
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_activerxbuf", ch->etc->unit, 0));
//    return _cyhal_gmac_dma_activerxbuf(ch->di[RX_Q0]);
    return _cyhal_gmac_dma_rxactive(ch->di[RX_Q0]);
}

/* allocate and post dma receive buffers */
 void BCMFASTPATH
_cyhal_gmac_chip_rxfill(ch_t *ch)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rxfill\n", ch->etc->unit));
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_rx", ch->etc->unit, 0));
    _cyhal_gmac_dma_rxfill(ch->di[RX_Q0]);
}

/* disable _cyhal_gmac_chip_ interrupts */
void BCMFASTPATH
_cyhal_gmac_chip_clearinterrupts(ch_t *ch, uint32_t int_bits_to_clear)
{
    /* clear the interrupt conditions */
    _CYHAL_GMAC_W_REG( &ch->regs->intstatus, int_bits_to_clear);
    ch->intstatus &= ~int_bits_to_clear;
}

/* get current and pending interrupt events */
 uint16_t BCMFASTPATH
_cyhal_gmac_chip_getintrevents(ch_t *ch, bool in_isr)
{
    uint32_t intstatus;
    uint16_t events;

    events = 0;

    /* read the interrupt status register */
    intstatus = _CYHAL_GMAC_R_REG( &ch->regs->intstatus);

    /* defer unsolicited interrupts */
    intstatus &= (in_isr ? ch->intmask : ch->def_intmask);

    if (intstatus != 0)
        events = GMAC_INTR_NEW;

    /* or new bits into persistent intstatus */
    intstatus = (ch->intstatus |= intstatus);

    /* return if no events */
    if (intstatus == 0)
        return (0);

    /* If we get an I_MDIO interrupt, clear it */
    if (intstatus & I_MDIO)
    {
        events |= GMAC_INTR_MDIO;
    }

    /* convert chip-specific intstatus bits into generic intr event bits */
#if defined(BCM_GMAC3)  // Time out for BMC_GMAC3, CYW43907AEVAL1F DOES NOT use this
    if (intstatus & I_TO)
    {
        events |= GMAC_ INTR_TO;                          /* post to et_dpc */

        _CYHAL_GMAC_W_REG(ch->osh, &ch->regs->intstatus, I_TO); /* explicitly ack I_TO */
        ch->intstatus &= ~(I_TO);                   /* handled in et_linux */
    }
#endif /* BCM_GMAC3 */

    if (intstatus & I_RI)   /* Rx receive complete */
    {
        events |= GMAC_INTR_RX;
    }

    if (intstatus & (I_XI0 | I_XI1 | I_XI2 | I_XI3))    /* Tx complete */
    {
        events |= GMAC_INTR_TX;
    }

#if defined(_CFE_)
    if (intstatus & ~(I_RDU | I_RFO) & GMAC_I_ERRORS)
#else
    if (intstatus & GMAC_I_ERRORS)
#endif
    {
        if (intstatus & I_MRO ) { _CYHAL_ETHERNET_LOG_INFO(("       I_RFO : 0x%lx  Rx Overflow\n",  intstatus & I_MRO)); }
        if (intstatus & I_PDEE) { _CYHAL_ETHERNET_LOG_INFO(("       I_PDEE: 0x%lx  Descriptor error\n", intstatus & I_PDEE)); }
        if (intstatus & I_PDE ) { _CYHAL_ETHERNET_LOG_INFO(("       I_PDE : 0x%lx  Data error\n", intstatus & I_PDE)); }
        if (intstatus & I_DE  ) { _CYHAL_ETHERNET_LOG_INFO(("       I_DE  : 0x%lx  Protocol error\n", intstatus & I_DE)); }
        if (intstatus & I_RDU ) { _CYHAL_ETHERNET_LOG_INFO(("       I_RDU : 0x%lx  Rx descriptor underflow\n", intstatus & I_RDU)); }
        if (intstatus & I_RFO ) { _CYHAL_ETHERNET_LOG_INFO(("       I_RFO : 0x%lx  Rx FIFO overflow\n", intstatus & I_RFO)); }
        if (intstatus & I_XFU ) { _CYHAL_ETHERNET_LOG_INFO(("       I_XFU : 0x%lx  Tx FIFO full\n", intstatus & I_XFU)); }

        events |= GMAC_INTR_ERROR;
    }

    return (events);
}

/* enable chip interrupts */
 void BCMFASTPATH
_cyhal_gmac_chip_intrson(ch_t *ch)
{
    ch->intmask = ch->def_intmask;
    _CYHAL_GMAC_W_REG( &ch->regs->intmask, ch->intmask);
}

/* disable chip interrupts */
 void BCMFASTPATH
_cyhal_gmac_chip_intrsoff(ch_t *ch)
{
    /* disable further interrupts from gmac */
    ch->intmask = 0;

    _CYHAL_GMAC_W_REG( &ch->regs->intmask, ch->intmask);
    (void) _CYHAL_GMAC_R_REG( &ch->regs->intmask);    /* sync readback */

    /* clear the interrupt conditions */
    _CYHAL_GMAC_W_REG( &ch->regs->intstatus, ch->intstatus);
}

/* return true of caller should re-initialize, otherwise false */
 bool BCMFASTPATH
_cyhal_gmac_chip_errors(ch_t *ch)
{
    uint32_t intstatus;
    etc_info_t *etc;

    etc = ch->etc;

    intstatus = ch->intstatus;
    ch->intstatus &= ~(GMAC_I_ERRORS);

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_errors: intstatus 0x%lx\n", etc->unit, intstatus));

    if (intstatus & I_PDEE) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: descriptor error\n", etc->unit));
        etc->dmade++;
    }

    if (intstatus & I_PDE) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: data error\n", etc->unit));
        etc->dmada++;
    }

    if (intstatus & I_DE) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: descriptor protocol error\n", etc->unit));
        etc->dmape++;
    }

    if (intstatus & I_RDU) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: receive descriptor underflow\n", etc->unit));
        etc->rxdmauflo++;
    }

    if (intstatus & I_RFO) {
        _CYHAL_ETHERNET_LOG_INFO(("et%d: receive fifo overflow\n", etc->unit));
        etc->rxoflo++;
    }

    if (intstatus & I_XFU) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: transmit fifo underflow\n", etc->unit));
        etc->txuflo++;
    }

    /* if overflows or decriptors underflow, don't report it
     * as an error and provoque a reset
     */
    if (intstatus & ~(I_RDU | I_RFO) & GMAC_I_ERRORS) {
        return (TRUE);
    }

    return (FALSE);
}

 bool
_cyhal_gmac_chip_dmaerrors(ch_t *ch)
{
    return _cyhal_gmac_dma_rxtxerror(ch->di[_CYHAL_ETHERNET_DMA_TX_CHAN_Q1], TRUE);
}

 void
_cyhal_gmac_chip_statsupd(ch_t *ch)
{
    etc_info_t *etc;
    gmacregs_t *regs;
    volatile uint32_t *s;
    uint32_t *d;

    etc = ch->etc;
    regs = ch->regs;

    /* read the mib counters and update the driver maintained software
     * counters.
     */
    if (etc->corerev != GMAC_4706B0_CORE_REV)
    {
        _CYHAL_GMAC_OR_REG( &regs->devcontrol, DC_MROR);

        s = &regs->mib.tx_good_octets;
        d = &ch->mib.tx_good_octets;
        while(s <= &regs->mib.rx_uni_pkts)
        {
            *d += _CYHAL_GMAC_R_REG( s);
            if (s == &ch->regs->mib.tx_q3_octets_high) {
                s++;
                d++;
            }
            s++;
            d++;
        }
    }


    /*
     * Aggregate transmit and receive errors that probably resulted
     * in the loss of a frame are computed on the fly.
     *
     * We seem to get lots of tx_carrier_lost errors when flipping
     * speed modes so don't count these as tx errors.
     *
     * Arbitrarily lump the non-specific dma errors as tx errors.
     */
    etc->rxgiants = (ch->di[RX_Q0])->rxgiants;
    etc->txerror = ch->mib.tx_jabber_pkts + ch->mib.tx_oversize_pkts
        + ch->mib.tx_underruns + ch->mib.tx_excessive_cols
        + ch->mib.tx_late_cols + etc->txnobuf + etc->dmade
        + etc->dmada + etc->dmape + etc->txuflo;
    etc->rxerror = ch->mib.rx_jabber_pkts + ch->mib.rx_oversize_pkts
        + ch->mib.rx_missed_pkts + ch->mib.rx_crc_align_errs
        + ch->mib.rx_undersize + ch->mib.rx_crc_errs
        + ch->mib.rx_align_errs + ch->mib.rx_symbol_errs
        + etc->rxnobuf + etc->rxdmauflo + etc->rxoflo + etc->rxbadlen + etc->rxgiants;
}

 void
_cyhal_gmac_chip_dumpmib(ch_t *ch, bool clear)
{
    gmacmib_t *m;

    m = &ch->mib;

    if (clear) {
        bzero((char *)m, sizeof(gmacmib_t));
        return;
    }

    _CYHAL_ETHERNET_LOG_INFO(("gmac_dma_txactive(di) : %d\n", _cyhal_gmac_dma_txactive(ch->di[0])));

    _CYHAL_ETHERNET_LOG_INFO(("tx_good_octets %d tx_good_octets_high %d tx_good_pkts %d \n\n",
                    m->tx_good_octets, m->tx_good_octets_high, m->tx_good_pkts));
    _CYHAL_ETHERNET_LOG_INFO(("tx_octets %d tx_octets_high %d tx_pkts %d \n",
                    m->tx_octets, m->tx_octets_high, m->tx_pkts));

    _CYHAL_ETHERNET_LOG_INFO(("\ntx_broadcast_pkts %d tx_multicast_pkts %d tx_jabber_pkts %d "
                   "tx_oversize_pkts %d\n",
                   m->tx_broadcast_pkts, m->tx_multicast_pkts,
                   m->tx_jabber_pkts,
                   m->tx_oversize_pkts));
    _CYHAL_ETHERNET_LOG_INFO(("tx_fragment_pkts %d tx_underruns %d\n",
                   m->tx_fragment_pkts, m->tx_underruns));
    _CYHAL_ETHERNET_LOG_INFO(("tx_total_cols %d tx_single_cols %d tx_multiple_cols %d "
                   "tx_excessive_cols %d\n",
                   m->tx_total_cols, m->tx_single_cols, m->tx_multiple_cols,
                   m->tx_excessive_cols));
    _CYHAL_ETHERNET_LOG_INFO(("tx_late_cols %d tx_defered %d tx_carrier_lost %d tx_pause_pkts %d\n",
                   m->tx_late_cols, m->tx_defered, m->tx_carrier_lost,
                   m->tx_pause_pkts));

    /* receive stat counters */
    /* hardware mib pkt and octet counters wrap too quickly to be useful */
    _CYHAL_ETHERNET_LOG_INFO(("\nrx_broadcast_pkts %d rx_multicast_pkts %d rx_jabber_pkts %d "
                   "rx_oversize_pkts %d\n",
                   m->rx_broadcast_pkts, m->rx_multicast_pkts,
                   m->rx_jabber_pkts, m->rx_oversize_pkts));
    _CYHAL_ETHERNET_LOG_INFO(("rx_fragment_pkts %d rx_missed_pkts %d rx_crc_align_errs %d "
                   "rx_undersize %d\n",
                   m->rx_fragment_pkts, m->rx_missed_pkts,
                   m->rx_crc_align_errs, m->rx_undersize));
    _CYHAL_ETHERNET_LOG_INFO(("rx_crc_errs %d rx_align_errs %d rx_symbol_errs %d\n",
                   m->rx_crc_errs, m->rx_align_errs, m->rx_symbol_errs));
    _CYHAL_ETHERNET_LOG_INFO(("rx_pause_pkts %d rx_nonpause_pkts %d\n",
                   m->rx_pause_pkts, m->rx_nonpause_pkts));
}

void
_cyhal_gmac_chip_duplexupd(ch_t *ch)
{
    uint32_t cmdcfg;
    int32 duplex, speed;

    cmdcfg = _CYHAL_GMAC_R_REG( &ch->regs->cmdcfg);

    /* check if duplex mode changed */
    if (ch->etc->duplex && (cmdcfg & CC_HD))
        duplex = 0;
    else if (!ch->etc->duplex && ((cmdcfg & CC_HD) == 0))
        duplex = CC_HD;
    else
        duplex = -1;

    /* check if the speed changed */
    speed = ((cmdcfg & CC_ES_MASK) >> CC_ES_SHIFT);
    if ((ch->etc->speed == 1000) && (speed != 2))
        speed = 2;
    else if ((ch->etc->speed == 100) && (speed != 1))
        speed = 1;
    else if ((ch->etc->speed == 10) && (speed != 0))
        speed = 0;
    else
        speed = -1;

    /* no duplex or speed change required */
    if ((speed == -1) && (duplex == -1))
        return;

    /* update the speed */
    if (speed != -1) {
        cmdcfg &= ~CC_ES_MASK;
        cmdcfg |= (speed << CC_ES_SHIFT);
    }

    /* update the duplex mode */
    if (duplex != -1) {
        cmdcfg &= ~CC_HD;
        cmdcfg |= duplex;
    }

    _CYHAL_ETHERNET_LOG_INFO(("gmac_chip_duplexupd: updating speed: 0x%x duplex: 0x%x cmdcfg: 0x%08lx\n", speed, duplex, cmdcfg));

    /* put mac in reset */
    _cyhal_gmac_init_reset(ch);

    _CYHAL_GMAC_W_REG( &ch->regs->cmdcfg, cmdcfg);

    /* bring mac out of reset */
    _cyhal_gmac_clear_reset(ch);
}

 uint16
_cyhal_gmac_chip_phyrd(ch_t *ch, uint16_t phyaddr, uint16_t reg)
{
    uint32_t tmp;
    gmacregs_t *regs;
    uint32_t *phycontrol_addr, *phyaccess_addr;

    if (GMAC_NS_COREREV(ch->etc->corerev)) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_phyrd: not supported\n", ch->etc->unit));
        return 0;
    }

    CY_ASSERT(phyaddr < MAXEPHY);
    CY_ASSERT(reg < MAXPHYREG);

    regs = ch->regs;

    if (ch->etc->corerev == GMAC_4706B0_CORE_REV) {
        phycontrol_addr = (uint32_t *)&ch->regscomm->phycontrol;
        phyaccess_addr = (uint32_t *)&ch->regscomm->phyaccess;
    } else {
        phycontrol_addr = (uint32_t *)&regs->phycontrol;    // 0x18005188 -- see 002-18060_0A_V.pdf (GMAC reg map)
        phyaccess_addr = (uint32_t *)&regs->phyaccess;      // 0x18005180 -- see 002-18060_0A_V.pdf (GMAC reg map)
    }

    /* issue the read */
    /* PR59036: Need to write phy address to phycontrol in addition to phyaccess */
    tmp = _CYHAL_GMAC_R_REG( phycontrol_addr);          // read PHY control register
    tmp &= ~0x1f;                                   // drop low 5 bits (4:0 ext_phy_addr This field is the external phy address)
    tmp |= phyaddr;                                 // 'or' in the phyaddr (usually 0x00)

    _CYHAL_GMAC_W_REG( phycontrol_addr, tmp);           // Set up control to point to the phy

    /* issue the read */
    _CYHAL_ETHERNET_LOG_INFO(("          READ   _cyhal_gmac_chip_phyrd(): %p \n", phyaccess_addr,
            (EPHY_PA_START | (phyaddr << EPHY_PA_ADDR_SHIFT) | (reg << EPHY_PA_REG_SHIFT))));
    _CYHAL_GMAC_W_REG( phyaccess_addr,
            (EPHY_PA_START | (phyaddr << EPHY_PA_ADDR_SHIFT) | (reg << EPHY_PA_REG_SHIFT)));


    /* wait for it to complete */
    SPINWAIT((_CYHAL_GMAC_R_REG( phyaccess_addr) & EPHY_PA_START), 1000);
    tmp = _CYHAL_GMAC_R_REG( phyaccess_addr);
    if (tmp & EPHY_PA_START) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_phyrd: DID NOT COMPLETE tmp:0x%08lx\n", ch->etc->unit, tmp));
        tmp = 0xffff;
    }
    else
    {
        _CYHAL_ETHERNET_LOG_INFO(("          READ    _cyhal_gmac_chip_phyrd(): %p regs 0x%lx\n", phyaccess_addr, tmp));
    }

    return (tmp & EPHY_PA_DATA_MASK);
}

 void
_cyhal_gmac_chip_phywr(ch_t *ch, uint16_t phyaddr, uint16_t reg, uint16_t v)
{
    uint32_t tmp;
    gmacregs_t *regs;
    uint32_t *phycontrol_addr, *phyaccess_addr;

    if (GMAC_NS_COREREV(ch->etc->corerev)) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_phywr: not supported\n", ch->etc->unit));
        return;
    }

    CY_ASSERT(phyaddr < MAXEPHY);
    CY_ASSERT(reg < MAXPHYREG);

    regs = ch->regs;

    if (ch->etc->corerev == GMAC_4706B0_CORE_REV) {
        phycontrol_addr = (uint32_t *)&ch->regscomm->phycontrol;
        phyaccess_addr = (uint32_t *)&ch->regscomm->phyaccess;
    } else {
        phycontrol_addr = (uint32_t *)&regs->phycontrol;    // 0x18005188 -- see 002-18060_0A_V.pdf (GMAC reg map)
        phyaccess_addr = (uint32_t *)&regs->phyaccess;      // 0x18005180 -- see 002-18060_0A_V.pdf (GMAC reg map)
    }

    /* clear mdioint bit of intstatus first  */
    /* PR59036: Need to write phy address to phycontrol in addition to phyaccess */
    tmp = _CYHAL_GMAC_R_REG( phycontrol_addr);          // read PHY control register
    tmp &= ~0x1f;                                   // drop low 5 bits (4:0 ext_phy_addr This field is the external phy address)
    tmp |= phyaddr;                                 // 'or' in the phyaddr (usually 0x00)
    _CYHAL_GMAC_W_REG( phycontrol_addr, tmp);           // write into the control register (don't change the other bits)
    _CYHAL_GMAC_W_REG( &regs->intstatus, I_MDIO);       // write the MDIO interrupt bit into 0x18005020 to clear any interrupts
    CY_ASSERT((_CYHAL_GMAC_R_REG( &regs->intstatus) & I_MDIO) == 0); // read back and validate it is 0x00

    /* issue the write */
    _CYHAL_ETHERNET_LOG_INFO(("          WRITE   _cyhal_gmac_chip_phywr(): %p value 0x%x\n", phyaccess_addr,
            (EPHY_PA_START | EPHY_PA_WRITE | (phyaddr << EPHY_PA_ADDR_SHIFT) | (reg << EPHY_PA_REG_SHIFT) | v)));

    /* Use the START (bit 30 - trigger) &  WRITE (bit 29 - wr_cmd) to indicate a write
     *     set the phyaddr  into the bits 20:16 (cpu_phy_addr)
     *     set the register into the bits 28:24 (cpu_reg_addr)
     *     put the value to write into bits 15:0
     */
    _CYHAL_GMAC_W_REG( phyaccess_addr,
          (EPHY_PA_START | EPHY_PA_WRITE | (phyaddr << EPHY_PA_ADDR_SHIFT) | (reg << EPHY_PA_REG_SHIFT) | v));

    /* wait for it to complete */
    SPINWAIT((_CYHAL_GMAC_R_REG( phyaccess_addr) & EPHY_PA_START), 1000);
    if (_CYHAL_GMAC_R_REG( phyaccess_addr) & EPHY_PA_START)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_phywr: did not complete\n", ch->etc->unit));
    }
}

 void
_cyhal_gmac_chip_phyor(ch_t *ch, uint16_t phyaddr, uint16_t reg, uint16_t v)
{
    uint16_t tmp;

    tmp = _cyhal_gmac_chip_phyrd(ch, phyaddr, reg);
    tmp |= v;
    _cyhal_gmac_chip_phywr(ch, phyaddr, reg, tmp);
}

 void
_cyhal_gmac_chip_configtimer(ch_t *ch, uint16_t microsecs)
{
    CY_ASSERT(ch->etc->bp_ticks_usec != 0);

    /* Enable general purpose timer in periodic mode */
    _CYHAL_GMAC_W_REG( &ch->regs->gptimer, microsecs * ch->etc->bp_ticks_usec);
}

 void
_cyhal_gmac_chip_phyreset(ch_t *ch, uint16_t phyaddr)
{
    CY_ASSERT(phyaddr < MAXEPHY);

    if (phyaddr == EPHY_NOREG)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_phyreset: phyaddr %d\n", ch->etc->unit, phyaddr));

    _cyhal_gmac_chip_phywr(ch, phyaddr, 0, EPHY_CTL_RESET);
    cyhal_system_delay_us(100);
    uint32_t reg = _cyhal_gmac_chip_phyrd(ch, phyaddr, 0);
    if (reg & EPHY_CTL_RESET)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_gmac_chip_phyreset: PHY RESET NOT COMPLETE \n", ch->etc->unit));
    }
    _cyhal_gmac_chip_phyinit(ch, phyaddr);
}

 void
_cyhal_gmac_chip_phyinit(ch_t *ch, uint16_t phyaddr)
{
    if (CHIPID(ch->sih->chip) == BCM5356_CHIP_ID) {
        /* PR71971 WAR: Ephy failing IEEE 10BT linearity spec */
        int i;

        for (i = 0; i < 5; i++) {
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x008b);
            _cyhal_gmac_chip_phywr(ch, i, 0x15, 0x0100);
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x000f);
            _cyhal_gmac_chip_phywr(ch, i, 0x12, 0x2aaa);
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x000b);
        }
    }

    if ((((CHIPID(ch->sih->chip) == BCM5357_CHIP_ID) ||
          (CHIPID(ch->sih->chip) == BCM4749_CHIP_ID)) &&
         (ch->sih->chippkg != BCM47186_PKG_ID)) ||
        ((CHIPID(ch->sih->chip) == BCM53572_CHIP_ID) &&
         (ch->sih->chippkg != BCM47188_PKG_ID))) {
        /* PR80324: optimized EPHY register settings for IEEE conformance tests */
        int i;

        /* Clear ephy power down bits in case it was set for coma mode */
        if (PMUCTL_ENAB(ch->sih)) {
            _cyhal_gmac_si_pmu_chipcontrol(ch->sih, 2, 0xc0000000, 0);
            _cyhal_gmac_si_pmu_chipcontrol(ch->sih, 4, 0x80000000, 0);
        }

        for (i = 0; i < 5; i++) {
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x000f);
            _cyhal_gmac_chip_phywr(ch, i, 0x16, 0x5284);
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x000b);
            _cyhal_gmac_chip_phywr(ch, i, 0x17, 0x0010);
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x000f);
            _cyhal_gmac_chip_phywr(ch, i, 0x16, 0x5296);
            _cyhal_gmac_chip_phywr(ch, i, 0x17, 0x1073);
            _cyhal_gmac_chip_phywr(ch, i, 0x17, 0x9073);
            _cyhal_gmac_chip_phywr(ch, i, 0x16, 0x52b6);
            _cyhal_gmac_chip_phywr(ch, i, 0x17, 0x9273);
            _cyhal_gmac_chip_phywr(ch, i, 0x1f, 0x000b);
        }
    }

    if (phyaddr == EPHY_NOREG)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_phyinit: phyaddr %d\n", ch->etc->unit, phyaddr));
}

 void
_cyhal_gmac_chip_phyforce(ch_t *ch, uint16_t phyaddr)
{
    etc_info_t *etc;
    uint16_t ctl;

    CY_ASSERT(phyaddr < MAXEPHY);

    if (phyaddr == EPHY_NOREG)
        return;

    etc = ch->etc;

    if (etc->forcespeed == GMAC_FORCE_SPEED_AUTO)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_phyforce: phyaddr %d speed %d\n",
              ch->etc->unit, phyaddr, etc->forcespeed));

    ctl = _cyhal_gmac_chip_phyrd(ch, phyaddr, 0);
    ctl &= ~(EPHY_CTL_SPEED | EPHY_CTL_SPEED_MSB | EPHY_CTL_ANENAB | EPHY_CTL_DUPLEX);

    switch (etc->forcespeed) {
        case GMAC_FORCE_SPEED_10HALF:
            break;

        case GMAC_FORCE_SPEED_10FULL:
            ctl |= EPHY_CTL_DUPLEX;
            break;

        case GMAC_FORCE_SPEED_100HALF:
            ctl |= EPHY_CTL_SPEED_100;
            break;

        case GMAC_FORCE_SPEED_100FULL:
            ctl |= (EPHY_CTL_SPEED_100 | EPHY_CTL_DUPLEX);
            break;

        case GMAC_FORCE_SPEED_1000FULL:
            ctl |= (EPHY_CTL_SPEED_1000 | EPHY_CTL_DUPLEX);
            break;
    }

    _cyhal_gmac_chip_phywr(ch, phyaddr, 0, ctl);
}

 void _cyhal_gmac_chip_phyloopback(ch_t *ch, uint32_t mode)
{
    etc_info_t *etc = ch->etc;
    uint16_t phyaddr = etc->phyaddr;
    uint16_t ctl;
    uint16_t new_ctl;
    CY_UNUSED_PARAMETER(mode);

    CY_ASSERT(phyaddr < MAXEPHY);

    if (phyaddr == EPHY_NOREG)
        return;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_phyloopback: phyaddr %d mode: %ld\n",
              ch->etc->unit, phyaddr, mode));

    ctl = _cyhal_gmac_chip_phyrd(ch, phyaddr, 0);

/* we are only supporting one type of loopback, not PHY loopback */
    new_ctl = ctl & ~EPHY_CTL_LOOP;

    if (new_ctl != ctl) // always change
    {
        _CYHAL_ETHERNET_LOG_INFO(("gmac_chip_phyloopback: change 0x%x mode: %ld\n", new_ctl, mode));
        _CYHAL_ETHERNET_LOG_INFO(("gmac_chip_phyloopback() write reg 0 with 0x%x (loopback bit 0x%x)\n", new_ctl, new_ctl & EPHY_CTL_LOOP));
        _cyhal_gmac_chip_phywr(ch, phyaddr, 0, new_ctl);
    }


}

/* set selected capability bits in autonegotiation advertisement */
 void
_cyhal_gmac_chip_phyadvertise(ch_t *ch, uint16_t phyaddr)
{
    etc_info_t *etc;
    uint16_t adv, adv2;

    CY_ASSERT(phyaddr < MAXEPHY);

    if (phyaddr == EPHY_NOREG)
        return;

    etc = ch->etc;

    if ((etc->forcespeed != GMAC_FORCE_SPEED_AUTO) || !etc->needautoneg)
        return;

    CY_ASSERT(etc->advertise);

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_phyadvertise: phyaddr %d advertise %x\n",
              ch->etc->unit, phyaddr, etc->advertise));

    /* reset our advertised capabilitity bits */
    adv = _cyhal_gmac_chip_phyrd(ch, phyaddr, 4);
    adv &= ~(EPHY_ADV_100FULL | EPHY_ADV_100HALF | EPHY_ADV_10FULL | EPHY_ADV_10HALF);
    adv |= etc->advertise;
    _cyhal_gmac_chip_phywr(ch, phyaddr, 4, adv);

    adv2 = _cyhal_gmac_chip_phyrd(ch, phyaddr, 9);
    adv2 &= ~(EPHY_ADV_1000FULL | EPHY_ADV_1000HALF);
    adv2 |= etc->advertise2;
    _cyhal_gmac_chip_phywr(ch, phyaddr, 9, adv2);

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_gmac_chip_phyadvertise: phyaddr %d adv %x adv2 %x phyad0 %x\n",
              ch->etc->unit, phyaddr, adv, adv2, _cyhal_gmac_chip_phyrd(ch, phyaddr, 0)));

    /* restart autonegotiation */
    _cyhal_gmac_chip_phyor(ch, phyaddr, 0, EPHY_CTL_RESTART);

    etc->needautoneg = FALSE;
}

void
_cyhal_ethernet_etc_set_pause_quanta(etc_info_t *etc, const uint16_t pause_quanta)
{
    _cyhal_gmac_pause_quanta((ch_t *)etc->ch, pause_quanta);
}
void
_cyhal_ethernet_etc_start_pause_frames(etc_info_t *etc, bool start)
{
    _cyhal_gmac_start_pause((ch_t *)etc->ch, start);

}

void _cyhal_ethernet_etc_clear_interrupts(etc_info_t *etc, uint32_t int_bits_to_clear)
{
    _cyhal_gmac_chip_clearinterrupts(etc->ch, int_bits_to_clear);
}


cy_rslt_t _cyhal_ethernet_get_mac_addr(cyhal_ethernet_t *obj, cyhal_ether_addr_t *mac_addr)
{
    cyhal_ether_addr_t temp;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    if (mac_addr == NULL)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    if (!CYHAL_ETHER_IS_NULL_ADDR(&obj->et_info.mac_address.octet))
    {
        /* read from our copy */
        memcpy (&mac_addr->octet, &obj->et_info.mac_address.octet, CYHAL_ETHER_ADDR_LEN);
    }
    else if (_cyhal_ethernet_etc_read_mac_addr_from_GMAC( &temp ) > 0)
    {
        /* Read the MAC addr from the GMAC chip directly */
        _CYHAL_ETHERNET_LOG_DEBUG(("_cyhal_ethernet_etc_read_mac_addr_from_GMAC() %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                   temp.octet[0],temp.octet[1],temp.octet[2],
                   temp.octet[3],temp.octet[4],temp.octet[5]));

        memcpy (&mac_addr->octet, &temp.octet, CYHAL_ETHER_ADDR_LEN);
    }
    else if(_cyhal_ethernet_etc_read_mac_addr_from_OTP(&temp) > 0)
    {
        /* Read the MAC addr from OTP */
        _CYHAL_ETHERNET_LOG_DEBUG(("_cyhal_ethernet_etc_read_mac_addr_from_OTP() %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                   temp.octet[0],temp.octet[1],temp.octet[2],
                   temp.octet[3],temp.octet[4],temp.octet[5]));

        memcpy (&mac_addr->octet, &temp.octet, CYHAL_ETHER_ADDR_LEN);
    }

    return CY_RSLT_SUCCESS;

}


cy_rslt_t _cyhal_ethernet_set_loopback_mode(cyhal_ethernet_t *obj, _cyhal_ethernet_loopback_mode_t loopback_mode)
{
    cy_rslt_t  result = CY_RSLT_SUCCESS;
    etc_info_t *etc   = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }
    if (loopback_mode >= _CYHAL_NUM_LOOPBACK_MODES)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    etc = obj->et_info.etc;

    _cyhal_ethernet_etc_loopback(etc, (uint32_t)loopback_mode);

    return result;
}

cy_rslt_t _cyhal_ethernet_get_status(cyhal_ethernet_t *obj, _cyhal_ethernet_status_t *status)
{
    etc_info_t  *etc  = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    if (status == NULL)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }
    memset(status, 0x00, sizeof(_cyhal_ethernet_status_t) );

    etc = obj->et_info.etc;

    status->network_up          = etc->ethernet_link_up;
    status->promiscuous_enable  = etc->promisc;
    status->broadcast_enable    = etc->broadcast_enable;
    status->loopback_mode       = etc->loopbk;
    if (obj->event_req)
    {
        status->callback_events     = obj->event_req;
    }

    return CY_RSLT_SUCCESS;
}
