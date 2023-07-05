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
 * Common [OS-independent] portion of
 * Broadcom Home Networking Division 10/100 Mbit/s Ethernet
 * Device Driver.
 *
 * $Id: cyhal_ethernet_etc.c 479508 2014-05-21 10:21:25Z amitsi $
 */

#include <stdio.h>
#include <typedefs.h>
#include <osl.h>
#include "cyhal_ethernet_gmac_rx_hdr.h"
#include "bcmendian.h"
#include "cyhal_ethernet_gmac_mib.h"
#include "cyhal_ethernet.h"
#include "cyhal_ethernet_phy.h"
#include "cyhal_ethernet_etc.h"
#include "cyhal_ethernet_internal.h"
#include "bcmutils.h"
#include "hndsoc.h"
#include <sbpcmcia.h>
#include "platform_otp.h"

#define ETC_TXERR_COUNTDOWN 3

uint32_t cy_ethernet_log_level = _CYHAL_ETHERNET_LOG_LEVEL_DEFAULT;

/* local prototypes */

char *
_cyhal_ether_to_ascii(const cyhal_ether_addr_t *ea, char *buf)
{
    static const char hex[] =
      {
          '0', '1', '2', '3', '4', '5', '6', '7',
          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
      };
    const uint8_t*octet = ea->octet;
    char *p = buf;
    int i;

    for (i = 0; i < 6; i++) {
        *p++ = hex[(*octet >> 4) & 0xf];
        *p++ = hex[*octet & 0xf];
        *p++ = ':';
        octet++;
    }

    *(p-1) = '\0';

    return (buf);
}


void*
_cyhal_ethernet_etc_attach(void *et_arg, uint16_t vendor, uint16_t device, void *regsva)
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t *etc = (etc_info_t *)et->etc;

    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_ethernet_etc_attach: vendor 0x%x device 0x%x\n", unit, vendor, device));

    /* some code depends on packed structures */
    CY_ASSERT(sizeof(cyhal_ether_addr_t) == CYHAL_ETHER_ADDR_LEN);
    CY_ASSERT(sizeof(struct cyhal_ether_header) == CYHAL_ETHER_HDR_LEN);

    if ( (vendor != _CYHAL_VENDOR_BROADCOM) || (device != _CYHAL_BCM47XX_GMAC_ID) )
    {
        return NULL;;
    }

    etc->et = et;
    etc->unit = 0;
    etc->vendorid = (uint16) vendor;
    etc->deviceid = (uint16) device;
    etc->forcespeed = GMAC_FORCE_SPEED_AUTO;
    etc->linkstate = FALSE;
    etc->promisc = TRUE;            /* defaults to TRUE */
    etc->broadcast_enable= TRUE;    /* defaults to TRUE */

    _cyhal_ethernet_etc_quota(etc);

    /* chip attach */
    if ((etc->ch = (_cyhal_gmac_chip_attach)(etc, regsva)) == NULL) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: chipattach error\n", etc->unit));
        goto fail;
    }

    return ((void*)etc);

fail:
    _cyhal_ethernet_etc_detach(etc);
    etc->ch = NULL;
    return (NULL);
}

void
_cyhal_ethernet_etc_detach(etc_info_t *etc)
{
    if (etc == NULL)
        return;

    /* free chip private state */
    if (etc->ch) {
        (_cyhal_gmac_chip_detach)(etc->ch);
        etc->chops = etc->ch = NULL;
    }
}

void
_cyhal_ethernet_etc_reset(etc_info_t *etc)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_ethernet_etc_reset\n", etc->unit));

    etc->reset++;

    /* reset the chip */
    (_cyhal_gmac_chip_reset)(etc->ch);

    /* free any posted tx packets */
    (_cyhal_gmac_chip_txreclaim)(etc->ch, TRUE);

    /* free any posted rx packets */
    (_cyhal_gmac_chip_rxreclaim)(etc->ch);
}

void
_cyhal_ethernet_etc_init(etc_info_t *etc, uint16_t options)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_ethernet_etc_init\n", etc->unit));

    CY_ASSERT(etc->pioactive == NULL);
    CY_ASSERT(!CYHAL_ETHER_IS_NULL_ADDR(&etc->cur_etheraddr));
    CY_ASSERT(!CYHAL_ETHER_IS_MULTICAST(&etc->cur_etheraddr));

    /* init the chip */
    (_cyhal_gmac_chip_init)(etc->ch, options);
}

bool
_cyhal_ethernet_etc_transmit(etc_info_t *etc, void *p, uint16_t size) /* transmit frame */
{
    /* send it on its way */
    return (_cyhal_gmac_chip_tx)(etc->ch, p, size);
}

int
_cyhal_ethernet_etc_read_mac_addr_from_OTP( cyhal_ether_addr_t *mac_addr )
{
    uint16_t mac_size;
    cy_rslt_t result;

    mac_size = sizeof(mac_addr->octet);
    result = _cyhal_platform_otp_read_tag( _CYHAL_PLATFORM_OTP_HW_RGN, HNBU_MACADDR, &mac_addr->octet, &mac_size );
    if ( result == CY_RSLT_SUCCESS )
    {
        if ( mac_size == sizeof(mac_addr->octet) )
        {
            mac_addr->octet[mac_size - 1]++;
        }
        else
        {
            _CYHAL_ETHERNET_LOG_ERROR(( "WARNING: OTP has bad MAC address\n" ));
            result = CYHAL_ETHERNET_RSLT_ERR_FAILED;
        }
    }

    return (result == CY_RSLT_SUCCESS) ? 0 : -1;
}


int
_cyhal_ethernet_etc_read_mac_addr_from_GMAC(cyhal_ether_addr_t *mac_addr)
{
    if (mac_addr == NULL)
    {
        return -1;
    }

    /* get the MAC address directly from the GMAC chip */
    uint32_t MAC_0_REG = _CYHAL_GMAC_R_REG((volatile uint32_t *)0x1800580C);          // bit 31: 0 SA[47:16] // Upper 4 octets
    uint32_t MAC_1_REG = _CYHAL_GMAC_R_REG((volatile uint32_t *)0x18005810);  // bit 31:16 SA[15:0]  // low 2 octets

    mac_addr->octet[0] = (MAC_0_REG >> 24) & 0x00ff;
    mac_addr->octet[1] = (MAC_0_REG >> 16) & 0x00ff;
    mac_addr->octet[2] = (MAC_0_REG >>  8) & 0x00ff;
    mac_addr->octet[3] = (MAC_0_REG >>  0) & 0x00ff;

    mac_addr->octet[4] = (MAC_1_REG >>  8) & 0x00ff;
    mac_addr->octet[5] = (MAC_1_REG >>  0) & 0x00ff;

    return 0;
}

static void
*etc_receive(etc_info_t *etc, uint32_t *size)          /* receive frame */
{
    void *in_buffer;
    /* get the packet if available */
    in_buffer = (_cyhal_gmac_chip_rx)(etc->ch, size);
    if ( in_buffer != NULL )
    {
        /* update statistics - non-error packets THAT WE ACTUALLY READ  */
        etc->rxframe++;
        etc->rxbyte += _CYHAL_GMAC_PKTLEN(in_buffer);

        /* re-fill descriptors */
        (_cyhal_gmac_chip_rxfill)( etc->ch );
    }

    return in_buffer;
}

uint32_t
_cyhal_ethernet_etc_receive_and_copy(etc_info_t *etc, uint8_t *buffer, uint32_t avail_buffer_size)
{
    uint32_t data_in_size = 0;
    void *in_buffer;

    if ( (buffer == NULL) || (avail_buffer_size == 0) )
    {
        return 0;
    }

    /* check for a received buffer */
    in_buffer = etc_receive(etc, &data_in_size);

    if ( (in_buffer != NULL) && (data_in_size > 0) )
    {
        /* Don't write too much into the buffer */
        if (data_in_size > avail_buffer_size)
        {
            data_in_size = avail_buffer_size;
        }

        if (data_in_size <= avail_buffer_size)
        {
            /* Only copy part of the data? or fail because the buffer is too small ? */
            /*
             *  TODO: Can we use m2m DMA for this transfer to go faster?
             */
            memcpy((void *)buffer, in_buffer, data_in_size);
        }
        else
        {
            data_in_size = 0;
        }

        /* We copied or are tossing the data, free the buffer ! */
        _CYHAL_GMAC_PKTFREE(in_buffer);
    }
    return data_in_size;
}

/* mark interface up */
void
_cyhal_ethernet_etc_up(etc_info_t *etc)
{
    etc->up = TRUE;

    _cyhal_ethernet_et_init(etc->et, ET_INIT_FULL | ET_INIT_INTRON);
}

/* mark interface down */
uint16_t
_cyhal_ethernet_etc_down(etc_info_t *etc, int reset)
{
    uint16_t callback;

    callback = 0;

    (_cyhal_gmac_chip_intrsoff)(etc->ch);
    etc->up = FALSE;

    if (reset)
        _cyhal_ethernet_et_reset(etc->et);

    /* suppress link state changes during power management mode changes */
    if (etc->linkstate) {
        etc->linkstate = FALSE;
        if (!etc->pm_modechange)
            _cyhal_ethernet_et_link_down(etc->et);
    }

    return (callback);
}

/* called once per second by timer interrupt */
void
_cyhal_ethernet_etc_watchdog(etc_info_t *etc)
{
    uint16 status;
    uint16 lpa;

    etc->now++;         /* This is meant to increment once per second */

    /* no local phy registers */
    if (etc->phyaddr == EPHY_NOREG) {
        etc->linkstate = TRUE;
        etc->duplex = 1;
        /* keep emac txcontrol duplex bit consistent with current phy duplex */
        (_cyhal_gmac_chip_duplexupd)(etc->ch);
        return;
    }

    status = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, 1);
    /* check for bad mdio read */
    if (status == 0xffff) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: _cyhal_ethernet_etc_watchdog: bad mdio read: ch %p phyaddr 0x%x mdcport %d\n",
            etc->unit, etc->ch, etc->phyaddr, etc->mdcport));
        return;
    }

    if (etc->forcespeed == GMAC_FORCE_SPEED_AUTO) {
        uint16 adv, adv2 = 0, status2 = 0, estatus;

        adv = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, 4);
        lpa = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, 5);

        /* read extended status register. if we are 1000BASE-T
         * capable then get our advertised capabilities and the
         * link partner capabilities from 1000BASE-T control and
         * status registers.
         */
        estatus = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, 15);
        if ((estatus != 0xffff) && (estatus & EPHY_EST_1000TFULL)) {
            /* read 1000BASE-T control and status registers */
            adv2 = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, 9);
            status2 = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, 10);
        }

        /* update current speed and duplex */
        if ((adv2 & EPHY_ADV_1000FULL) && (status2 & EPHY_LPA_1000FULL)) {
            etc->speed = 1000;
            etc->duplex = 1;
        } else if ((adv2 & EPHY_ADV_1000HALF) && (status2 & EPHY_LPA_1000HALF)) {
            etc->speed = 1000;
            etc->duplex = 0;
        } else if ((adv & EPHY_ADV_100FULL) && (lpa & EPHY_LPA_100FULL)) {
            etc->speed = 100;
            etc->duplex = 1;
        } else if ((adv & EPHY_ADV_100HALF) && (lpa & EPHY_LPA_100HALF)) {
            etc->speed = 100;
            etc->duplex = 0;
        } else if ((adv & EPHY_ADV_10FULL) && (lpa & EPHY_LPA_10FULL)) {
            etc->speed = 10;
            etc->duplex = 1;
        } else {
            etc->speed = 10;
            etc->duplex = 0;
        }
    }

    if (etc->loopbk != _CYHAL_ETHERNET_LOOPBACK_MODE_NONE) {
        status |= EPHY_STAT_LINK;
    }

    /* monitor link state */
    if (!etc->linkstate && (status & EPHY_STAT_LINK)) {
        etc->linkstate = TRUE;
        if (etc->pm_modechange)
            etc->pm_modechange = FALSE;
        else
            _cyhal_ethernet_et_link_up(etc->et);
    } else if (etc->linkstate && !(status & EPHY_STAT_LINK)) {
        etc->linkstate = FALSE;
        if (!etc->pm_modechange)
            _cyhal_ethernet_et_link_down(etc->et);
    }

    /* keep emac txcontrol duplex bit consistent with current phy duplex */
    (_cyhal_gmac_chip_duplexupd)(etc->ch);

    /* check for remote fault error */
    if (status & EPHY_STAT_REMFAULT) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: remote fault\n", etc->unit));
    }

    /* check for jabber error */
    if (status & EPHY_STAT_JAB) {
        _CYHAL_ETHERNET_LOG_ERROR(("et%d: jabber\n", etc->unit));
    }

    /*
     * Read chip mib counters occasionally before the 16bit ones can wrap.
     * We don't use the high-rate mib counters.
     */
    if ((etc->now % 30) == 0)
        (_cyhal_gmac_chip_statsupd)(etc->ch);
}

void
_cyhal_ethernet_etc_loopback(etc_info_t *etc, uint16_t mode)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_ethernet_etc_loopback: 0x%x\n", etc->unit, mode));

    if (etc->loopbk != (uint32_t)mode)
    {
        etc->loopbk = mode;
        if (etc->ch != NULL)
        {
            _cyhal_gmac_loopback(etc->ch, mode);
        }
    }
}

void
_cyhal_ethernet_etc_promisc(etc_info_t *etc, uint16_t on)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_ethernet_etc_promisc: %d\n", etc->unit, on));

    if (etc->promisc != (bool)on)
    {
        etc->promisc = (bool) on;

#if 1 /* Just set the registers directly, do not call _cyhal_ethernet_et_init() */
        _cyhal_gmac_promisc(etc->ch, on);
#else
        _cyhal_ethernet_et_init(etc->et, ET_INIT_INTRON);
#endif
    }
}

bool
etc_get_promisc(etc_info_t *etc)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: etc_get_promisc: %d\n", etc->unit));
    return etc->promisc;
}

void
_cyhal_ethernet_etc_qos(etc_info_t *etc, uint16_t on)
{
    _CYHAL_ETHERNET_LOG_INFO(("et%d: _cyhal_ethernet_etc_qos: %d\n", etc->unit, on));

    if (etc->qos != (bool) on)
    {
        etc->qos = (bool) on;
        _cyhal_ethernet_et_init(etc->et, ET_INIT_INTRON);
    }
}

void
_cyhal_ethernet_etc_quota(etc_info_t *etc)
{
   /* Cap to _CYHAL_ETHERNET_DMA_QUOTA_MAX */
   etc->quota = _CYHAL_ETHERNET_DMA_QUOTA_MAX;
}

int16_t
_cyhal_ethernet_etc_set_speed(etc_info_t *etc, uint32_t speed)
{
    if (etc->phyaddr == EPHY_NOREG) {
        /* Don't configure the MII interface speed here */
        return -1;
    }
    switch (speed)
    {
    case GMAC_FORCE_SPEED_1000FULL:
        etc->speed = 1000;
        etc->duplex = 1;
        break;
    case GMAC_FORCE_SPEED_1000HALF:
        etc->speed = 1000;
        etc->duplex = 0;
        break;
    case GMAC_FORCE_SPEED_100FULL:
        etc->speed = 100;
        etc->duplex = 1;
        break;
    case GMAC_FORCE_SPEED_100HALF:
        etc->speed = 100;
        etc->duplex = 0;
        break;
    case GMAC_FORCE_SPEED_10FULL:
        etc->speed = 10;
        etc->duplex = 1;
        break;
    case GMAC_FORCE_SPEED_10HALF:
        etc->speed = 10;
        etc->duplex = 0;
        break;
    case GMAC_FORCE_SPEED_AUTO:
        ;
        break;
    default:
        return -1;
    }

    etc->forcespeed = speed;

    /* explicitly reset the phy */
    (_cyhal_gmac_chip_phyreset)(etc->ch, etc->phyaddr);

    /* request restart autonegotiation if we're reverting to adv mode */
    if (etc->forcespeed == GMAC_FORCE_SPEED_AUTO)
    {
        etc->advertise = (EPHY_ADV_100FULL | EPHY_ADV_100HALF |
                          EPHY_ADV_10FULL | EPHY_ADV_10HALF);
        etc->advertise2 = EPHY_ADV_1000HALF | EPHY_ADV_1000FULL;
        etc->needautoneg = TRUE;
    }
    else
    {
        etc->advertise = etc->advertise2 = 0;
        etc->needautoneg = FALSE;
    }

    _cyhal_ethernet_et_init(etc->et, ET_INIT_INTRON);
    return 0;
}

void
_cyhal_ethernet_etc_rxlazy(etc_info_t *etc, uint16_t microsecs, uint16_t framecnt)
{
    CY_ASSERT(framecnt >= 1U);
    CY_ASSERT(etc->bp_ticks_usec != 0U);

    etc->rxlazy_timeout = microsecs * etc->bp_ticks_usec;
    etc->rxlazy_framecnt = framecnt;
}

uint32_t _cyhal_ethernet_etc_phyread(etc_info_t *etc, uint8_t reg)
{
    uint32_t value = (_cyhal_gmac_chip_phyrd)(etc->ch, etc->phyaddr, reg);
    _CYHAL_ETHERNET_LOG_INFO(("_cyhal_ethernet_etc_phyread: reg 0x%x => 0x%x\n", reg, value));
    return value;
}

int16_t _cyhal_ethernet_etc_phywrite(etc_info_t *etc, uint8_t reg, uint32_t value)
{
    (_cyhal_gmac_chip_phywr)(etc->ch, etc->phyaddr, reg, value);
    _CYHAL_ETHERNET_LOG_INFO(("_cyhal_ethernet_etc_phywrite of reg 0x%x <= 0x%x\n", reg, value));
    return 0;
}



#ifdef _CYHAL_ETHERNET_LOG_ENABLE
uint16_t
_cyhal_ethernet_etc_dump(etc_info_t *etc)
{
    _cyhal_ethernet_etc_dumpetc(etc, b);
    (_cyhal_gmac_chip_dump)(etc->ch, b);

    (_cyhal_gmac_chip_dumpmib)(etc->ch, b, FALSE);

    return b->size;
}
#endif /* _CYHAL_ETHERNET_LOG_ENABLE */

void
_cyhal_ethernet_etc_dumpetc(etc_info_t *etc)
{
    uint16_t i;

    _CYHAL_ETHERNET_LOG_INFO(("etc 0x%x et 0x%x unit %d msglevel %d speed/duplex %d %s\n",
        (ulong)etc, (ulong)etc->et, etc->unit, cy_ethernet_log_level,
        etc->speed, (etc->duplex ? "full": "half")));
    _CYHAL_ETHERNET_LOG_INFO(("up %d promisc %d loopbk 0x%x forcespeed %d advertise 0x%x "
                   "advertise2 0x%x needautoneg %d\n",
                   etc->up, etc->promisc, etc->loopbk, etc->forcespeed,
                   etc->advertise, etc->advertise2, etc->needautoneg));
    _CYHAL_ETHERNET_LOG_INFO(("piomode %d pioactive 0x%x qos %d\n",
        etc->piomode, (ulong)etc->pioactive, etc->qos));
    _CYHAL_ETHERNET_LOG_INFO(("vendor 0x%x device 0x%x rev %d unit %d phyaddr %d mdcport %d\n",
        etc->vendorid, etc->deviceid, etc->chiprev,
        etc->unit, etc->phyaddr, etc->mdcport));

    _CYHAL_ETHERNET_LOG_INFO(("perm_etheraddr %s cur_etheraddr %s\n",
        _cyhal_ether_to_ascii(&etc->perm_etheraddr, perm),
        _cyhal_ether_to_ascii(&etc->cur_etheraddr, cur)));

    cyhal_gmac_filter_info_t config;
    _CYHAL_ETHERNET_LOG_INFO(("GMAC Multicast filters\n"));
    _CYHAL_ETHERNET_LOG_INFO(("index   ignore  type   addr\n"));
    for (i = 0; i < CYHAL_ETHERNET_NUM_FILTERS; i++)
    {
        if (_cyhal_gmac_broadcast_filter_lookup_by_index(i, &config) == CY_RSLT_SUCCESS)
        {
            _CYHAL_ETHERNET_LOG_INFO(("%d  %d  %s   0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n", i, config.ignore_bytes,
                    ( (config.filter_type == CYHAL_GMAC_FILTER_TYPE_DESTINATION) ? "dest" : "src "),
                    config.mac_addr[0], config.mac_addr[1], config.mac_addr[2],
                    config.mac_addr[3], config.mac_addr[4], config.mac_addr[5]));
        }
    }

    _CYHAL_ETHERNET_LOG_INFO(("linkstate %d\n", etc->linkstate));
    _CYHAL_ETHERNET_LOG_INFO(("\n"));

    /* refresh stat counters */
    _cyhal_gmac_chip_statsupd(etc->ch);

    /* summary stat counter line */
    /* use sw frame and byte counters -- hw mib counters wrap too quickly */
    _CYHAL_ETHERNET_LOG_INFO(("txframe %d txbyte %d txerror %d\n",
        etc->txframe, etc->txbyte, etc->txerror ));
    for (i=0; i< _CYHAL_ETHERNET_DMA_TX_NUM_QUEUES; i++)
    {
        _CYHAL_ETHERNET_LOG_INFO(("txframes[%d] %d ", i, etc->txframes[i]));
    }
    _CYHAL_ETHERNET_LOG_INFO(("\n\n"));

    /* transmit & receive stat counters */
    /* hardware mib pkt and octet counters wrap too quickly to be useful */
    _cyhal_gmac_chip_dumpmib(etc->ch, FALSE);

    _CYHAL_ETHERNET_LOG_INFO(("txnobuf %d reset %d dmade %d dmada %d dmape %d\n\n",
                   etc->txnobuf, etc->reset, etc->dmade, etc->dmada, etc->dmape));

    /* hardware mib pkt and octet counters wrap too quickly to be useful */
    _CYHAL_ETHERNET_LOG_INFO(("rxframe %d rxbyte %d rxerror %d \n", etc->rxframe, etc->rxbyte, etc->rxerror));
    _CYHAL_ETHERNET_LOG_INFO(("rxnobuf %d rxdmauflo %d rxoflo %d rxbadlen %d "
                   "rxgiants %d rxoflodiscards %d\n",
                   etc->rxnobuf, etc->rxdmauflo, etc->rxoflo, etc->rxbadlen,
                   etc->rxgiants, etc->rxoflodiscards));

    _CYHAL_ETHERNET_LOG_INFO(("chained %d unchained %d maxchainsz %d currchainsz %d\n",
                   etc->chained, etc->unchained, etc->maxchainsz, etc->currchainsz));

#if defined(CY_ETHERNET_LOG_ENABLE)

    _CYHAL_ETHERNET_LOG_INFO(("\nrx processed :%d", etc->rxprocessed));

#if 0       // Too much output if you don't need it
    for (i = 0; i < _CYHAL_ETHERNET_DMA_QUOTA_MAX; i++) {
        if (etc->quota_stats[i]) {
            _CYHAL_ETHERNET_LOG_INFO(("  %d %d\n", i, etc->quota_stats[i]));
        }
    }

    bzero(etc->quota_stats, sizeof(etc->quota_stats) );
#endif

#endif /* _CYHAL_ETHERNET_LOG_ENABLE */

    _CYHAL_ETHERNET_LOG_INFO(("\n"));
    _CYHAL_ETHERNET_LOG_INFO(("et: %s return size: %d\n", __func__, b->size));
}

#ifdef _CYHAL_ETHERNET_LOG_ENABLE
void
etc_prhdr(char *msg, struct cyhal_ether_header *eh, uint16_t len, int unit)
{
    char da[32], sa[32];

    if (msg && (msg[0] != '\0'))
        _CYHAL_ETHERNET_LOG_INFO(("et%d: %s: ", unit, msg));
    else
        _CYHAL_ETHERNET_LOG_INFO(("et%d: ", unit));

    _CYHAL_ETHERNET_LOG_WARNING(("dst %s src %s type 0x%x len %d\n",
        _cyhal_ether_to_ascii((cyhal_ether_addr_t *)eh->ether_dhost, da),
        _cyhal_ether_to_ascii((cyhal_ether_addr_t *)eh->ether_shost, sa),
        ntoh16(eh->ether_type),
        len));
}
void
etc_prhex(char *msg, uchar *buf, uint16_t nbytes, int unit)
{
    if (msg && (msg[0] != '\0'))
        _CYHAL_ETHERNET_LOG_WARNING(("et%d: %s:\n", unit, msg));
    else
        _CYHAL_ETHERNET_LOG_WARNING(("et%d:\n", unit));

    prhex(NULL, buf, nbytes);
}
#endif /* _CYHAL_ETHERNET_LOG_ENABLE */

