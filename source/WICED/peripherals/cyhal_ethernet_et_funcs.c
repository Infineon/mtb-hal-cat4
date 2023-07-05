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
 * $Id: etc.c 479508 2014-05-21 10:21:25Z amitsi $
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <typedefs.h>

#include "bcmendian.h"
#include "cyhal_ethernet.h"
#include "cyhal_ethernet_phy.h"
#include "cyhal_ethernet_etc.h"
#include "bcmutils.h"
#include "cyhal_pin_package.h"

/* may not be needed after moving some code into appropriate files (platform_chipcontrol(),     cyhal_system_delay_us( 2 ) ) */
#include "cyhal_system.h"
#include "cyhal_pwm.h"

/*******************************************************************************
 *
 * Defines / MACROS
 *
 *******************************************************************************/

/*******************************************************************************
 *
 * Variables / Data
 *
 *******************************************************************************/

static cyhal_pwm_t pwm_phy_reset_obj;

/*******************************************************************************
 *
 * et functions
 *
 *******************************************************************************/

void _cyhal_ethernet_et_intrson( void *et_arg )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t *etc = (etc_info_t *)et->etc;
    _cyhal_gmac_chip_intrson( etc->ch );
}

void _cyhal_ethernet_et_intrsoff( void *et_arg )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t *etc = (etc_info_t *)et->etc;
    _cyhal_gmac_chip_intrsoff( etc->ch );
}

uint16_t _cyhal_ethernet_et_getintrevents(void *et_arg, bool in_isr)
{
    uint16_t interrupt_events = 0;
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;

    if ( et->started )
    {
        etc_info_t *etc = (etc_info_t *)et->etc;

        interrupt_events =  _cyhal_gmac_chip_getintrevents( etc->ch, in_isr ) | et->watchdog_events;
        et->watchdog_events = 0;

        _cyhal_ethernet_et_intrsoff( et); /* disable and ack interrupts retrieved via getintrevents */

        if (interrupt_events & GMAC_INTR_NEW)
        {
            if ( interrupt_events & GMAC_INTR_TX )
            {
                _cyhal_gmac_chip_txreclaim(etc->ch, FALSE);
            }
            if ( interrupt_events & GMAC_INTR_RX )
            {
                /* We intercept packets here to tell if they are going to pass filters.
                 */
                void *p = _cyhal_gmac_chip_rxpeek_and_drop(etc->ch);
                if (p == NULL)
                {
                    /* The packet was dropped. Do not send a new event. */
                    interrupt_events &= ~GMAC_INTR_RX;
                }
            }
        }

        _cyhal_ethernet_et_intrson( et );
    }
    return interrupt_events;
}

/* Note: Changes etc->linkstate = TRUE before calling this function */
void _cyhal_ethernet_et_link_up( void *et_arg )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t *etc = et->etc;
    etc->ethernet_link_up = true;
    _CYHAL_ETHERNET_LOG_INFO(( "ethernet: link up\n" ));
}

/* Note: Changes etc->linkstate = FALSE before calling this function */
void _cyhal_ethernet_et_link_down( void *et_arg )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t *etc = et->etc;
    etc->ethernet_link_up = false;
    _CYHAL_ETHERNET_LOG_INFO(( "ethernet: link down\n" ));
}

static bool _cyhal_ethernet_check_and_clear_adv( uint32_t *speed_adv, uint32_t mode )
{
    bool ret = ( *speed_adv & mode ) != 0;

    *speed_adv &= ~mode;

    return ret;
}
static void _cyhal_ethernet_config_adv_speed( void *et_arg  )
{
    _cyhal_ethernet_info_t  *et = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t *etc = et->etc;
    uint32_t    adv = etc->forcespeed;

    if (adv & GMAC_FORCE_SPEED_AUTO)
    {
        return;
    }

    etc->advertise = 0;
    etc->advertise2 = 0;

    if ( _cyhal_ethernet_check_and_clear_adv( &adv, GMAC_FORCE_SPEED_10HALF) )
    {
        etc->advertise |= EPHY_ADV_10HALF;
    }
    if ( _cyhal_ethernet_check_and_clear_adv( &adv, GMAC_FORCE_SPEED_10FULL) )
    {
        etc->advertise |= EPHY_ADV_10FULL;
    }
    if ( _cyhal_ethernet_check_and_clear_adv( &adv, GMAC_FORCE_SPEED_100HALF) )
    {
        etc->advertise |= EPHY_ADV_100HALF;
    }
    if ( _cyhal_ethernet_check_and_clear_adv( &adv, GMAC_FORCE_SPEED_100FULL) )
    {
        etc->advertise |= EPHY_ADV_100FULL;
    }
    if ( _cyhal_ethernet_check_and_clear_adv( &adv, GMAC_FORCE_SPEED_1000HALF) )
    {
        etc->advertise2 |= EPHY_ADV_1000HALF;
    }
    if ( _cyhal_ethernet_check_and_clear_adv( &adv, GMAC_FORCE_SPEED_1000FULL) )
    {
        etc->advertise2 |= EPHY_ADV_1000FULL;
    }

    CY_ASSERT( adv == 0 );
}

static void _cyhal_et_reclaim_packets( _cyhal_ethernet_info_t* et )
{
    etc_info_t*   etc   = et->etc;
//    struct chops* chops = etc->chops;
    void*         ch    = etc->ch;

    _cyhal_gmac_chip_txreclaim( ch, true );
    _cyhal_gmac_chip_rxreclaim( ch );
}

void _cyhal_ethernet_et_reset( void *et_arg )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;

    etc_info_t* etc = et->etc;
    _cyhal_ethernet_etc_reset( etc );
}

void _cyhal_ethernet_et_init( void *et_arg, uint options )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;

    etc_info_t* etc = et->etc;

    _cyhal_ethernet_config_adv_speed( et );

    _cyhal_ethernet_et_reset( et );

    _cyhal_ethernet_etc_init( etc, options );
}


int _cyhal_ethernet_et_up( void *et_arg )
{
    _cyhal_ethernet_info_t *et = (_cyhal_ethernet_info_t *)et_arg;
    CY_UNUSED_PARAMETER(et);

    etc_info_t* etc = et->etc;

    _cyhal_ethernet_etc_up( etc );

    _cyhal_ethernet_etc_watchdog( etc );    /* NOTE: _cyhal_ethernet_etc_watchdog() fails the first time it is called. cyhal_ethernet_watchdog_timer_isr() calls 1x per secc */

    return 0;
}

void _cyhal_ethernet_et_watchdog(void *et_arg)
{
    _cyhal_ethernet_info_t   *et  = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t  *etc = et->etc;

    _cyhal_ethernet_etc_watchdog(etc);
}

int _cyhal_ethernet_et_down( void *et_arg, int reset )
{
    _cyhal_ethernet_info_t   *et  = (_cyhal_ethernet_info_t *)et_arg;
    etc_info_t  *etc = et->etc;

    etc->linkstate = false;
    _cyhal_ethernet_etc_down( etc, reset );

    _cyhal_et_reclaim_packets( et );

    return 0;
}

int _cyhal_ethernet_et_phy_reset( void *et_arg )
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    CY_UNUSED_PARAMETER(et_arg);

    // From WICED: OUTPUT_PUSH_PULL
    // For MTB:    CYHAL_GPIO_DRIVE_STRONG
    // FROM WICED "/43xxx_Wi-Fi/platforms/CYW943907AEVAL1F/platform.h"
    //+--------------------------------------------------------------------------------------------------------+--------------+--------------------------------------------------+
    //| Enum ID       |Pin  |   Pin Name   | Mod |  Module Pin  Name       |  Board Net Name          | Header | Function     | Board Connection              | WICED Peripheral |
    //|               | #   |   on 43907   | Pin#|                         |                          | Conn   |              |                               | Alias            |
    //|---------------+-----+--------------+-----+-------------------------+--------------------------+--------+--------------+-------------------------------|------------------|
    //| WICED_GPIO_15 | 190 | PWM_2        | A27 | PWM_2                   | PWM_2                    | N/C    |  PWM, GPIO   | Ethernet PHY Reset            |                  |

    /* Init the PWM_2 pin for PHY chip reset */
    status = cyhal_pwm_init_adv(&pwm_phy_reset_obj, PIN_PWM_2, NC, CYHAL_PWM_LEFT_ALIGN, true, 0, false, NULL);
    if (status == CY_RSLT_SUCCESS)
    {
        /* start high, let chip settle */
        status = cyhal_pwm_set_duty_cycle(&pwm_phy_reset_obj, 100.0f, 10000);
        status = cyhal_pwm_start(&pwm_phy_reset_obj);
        cyhal_system_delay_ms( 100 );

        /* Keep RESET low for 2 us */
        status = cyhal_pwm_set_duty_cycle(&pwm_phy_reset_obj, 0.0f, 10000);
        cyhal_system_delay_us( 2 ); // 2 us

        /* Leave High */
        status = cyhal_pwm_set_duty_cycle(&pwm_phy_reset_obj, 100.0f, 10000);
        cyhal_system_delay_ms( 100 );
    }
    if (status != CY_RSLT_SUCCESS)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("ERROR: Failed to reset the PHY in _cyhal_ethernet_et_phy_reset()!\r\n"));
        return 0; // failure
    }
    return 1; // success
}

int _cyhal_ethernet_et_phy_free( void *et_arg )
{
    CY_UNUSED_PARAMETER(et_arg);

    cyhal_pwm_free(&pwm_phy_reset_obj);
    return 1;
}

int _cyhal_ethernet_et_set_addrs( void *etc_arg )
{
    etc_info_t  *etc = (etc_info_t  *)etc_arg;
    _cyhal_ethernet_info_t   *et  = etc->et;

    cyhal_ether_addr_t *mac = &et->mac_address;

    etc->phyaddr = 0;   // et->phy_addr; // in WICED, this is set to 0x00 in config and never changed

    memcpy( &etc->cur_etheraddr.octet[0], &mac->octet[0], CYHAL_ETHER_ADDR_LEN );

    memcpy( &etc->perm_etheraddr.octet[0], &mac->octet[0], CYHAL_ETHER_ADDR_LEN );

    _CYHAL_ETHERNET_LOG_NOTICE(( "Set Ethernet PHY Address : 0x%08X\r\n", etc->phyaddr));
    _CYHAL_ETHERNET_LOG_NOTICE(( "Set Ethernet MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                            mac->octet[0], mac->octet[1], mac->octet[2], mac->octet[3], mac->octet[4], mac->octet[5]));

    return 1;
}

void _cyhal_ethernet_et_clear_gmac_interrupts(void *et_arg, uint32_t int_bits_to_clear)
{
//    extern void chipclearinterrupts(ch_t *ch, uint32_t int_bits_to_clear);  // TODO: Move to appropriate header file

    _cyhal_ethernet_info_t   *et  = (_cyhal_ethernet_info_t   *)et_arg;
    etc_info_t  *etc = (etc_info_t  *)et->etc;

    _cyhal_gmac_chip_clearinterrupts(etc->ch, int_bits_to_clear);
}
