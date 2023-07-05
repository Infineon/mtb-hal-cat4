/***************************************************************************//**
* \file cyhal_ethernet.c
*
* \brief
* Provides a high level interface for interacting with the Infineon Ethernet sub system.
* This interface abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
*
********************************************************************************
* \copyright
* Copyright 2023 Cypress Semiconductor Corporation (an Infineon company) or
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "cy_result.h"
#include "cyhal_dma.h"
#include "cyhal_hw_types.h"
#include "cyhal_ethernet.h"
#include "cyhal_gpio.h"
#include "cyhal_utils.h"
#include "cyhal_clock.h"
#include "cyhal_timer.h"

#include "cyhal_system.h"
#include "cyhal_ethernet_gmac_filter.h"

#include "cyhal_ethernet_etc.h"
#include "cyhal_ethernet_gmac_buffer_interface.h"
#include "cyhal_hw_resources.h"
#include "cyhal_hwmgr.h"

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
 *
 * The frame buffer is a complete Ethernet Header, with the start of the buffer containing the cyhal_ether_header_t structure
 * followed by the data, and room for the CRC (4 bytes) at the end of the buffer.
 * Calculate the buffer size:
 *  size = sizeof(cyhal_ether_header_t) + sizeof(uint32_t) + actual data size
 *
 * Minimum data filled in by Caller:
 * Header:
 *   - ether_dhost  (destination mac address)
 *
 * cyhal_ethernet_transmit_frame() will fill in these parts of the packet:
 * Header:
 *   - preamble     (802.3 standard)
 *   - ether_shost  (source mac address, this device)
 *   - ether_type   (type of Ethernet packet)
 * Footer:
 *   - CRC
 *
 * ++++++++++++++++++++++++++++++++++++++++++++ --
 * | Ethernet  |                              |    \
 * | Header    | cyhal_ether_header_t         |     \
 * +-----------+------------------------------+      \
 * |           |                              |       --- size of frame_data
 * |  Data     |  CYHAL_ETHER_MAX_DATA bytes  |      /
 * |           |                              |     /
 * +-----------+------------------------------+    /
 * |  Footer   |  4-byte CRC                  |   /
 * ++++++++++++++++++++++++++++++++++++++++++++ --
 *
 * ref: https://www.intel.com/content/www/us/en/docs/programmable/683402/22-1-20-0-0/pause-frame-format.html
 *
 * Pause Frame packet:
 * Header:
 *   - preamble     7 octets (802.3 standard)
 *   - SFD          1 octet  end of preamble
 *   - ether_dhost  6 octets (destination mac address)
 *   - ether_shost  6 octets (source mac address, this device)
 *   - ether_type   2 octets (type of packet - 0x8808 Ethernet flow control packet)
 *   - OPCODE       2 octets opcode (0x0001)
 *   - Pause Quanta 2 octets PAUSE QUANTA (p1 - MSB, p2 - LSB)
 *   - PAD          42 octets of 0x00
 * Footer:
 *   - CRC
 *******************************************************************************/

/* Pause Frame Control Registers
 *
 *  FlowCntl_th     0x18005104
 *  bit 26:16   regtorxq_flow_cntl_off_th   This field is flow control off threshold (reset value 614)
 *  bit 10:0    regtorxq_flow_cntl_on_th    This field is flow control on threshold. (reset value 714)
 *
 *  PAUSE_QUANT     0x18005818
 *  bit 15:0    pause_quant         Receive Pause Quanta (RW). 16-Bit value, sets, in increment of
 *                                  512 Ethernet bit times, the pause quanta used in each Pause Frame
 *                                  sent to the remote Ethernet device (reset value 0x0000ffff)
 *
 *  PAUSE_CONTROL   0x18005B30
 *  bit 17      pause_control_en    Repetitive pause frame send enable. When enabled and Back Pressured asserted,
 *                                  pauses are transmitted when PAUSE_TIMER value expires. (res3et value 0x00)
 *  bit 16:0    pause_timer         Pause timer value for repetitive pause frames. (reset value 0x0000ffff)
 *
 *
 * Pause Frame Sent Status can be checked with PHY read
 *  DevStatus       0x18005004      PHY_register_01
 *  bit 6       [RO] pause_on       When this bit = 1, MAC is sending a pause on frame. And re-transmit
 *                                  the pause on frame once the pause time slot counts down to zero.
 *                                  When this bit transitions from 1 to zero, MAC will send a pause off frame.
 *                                  (reset value 0)
 * value = (_cyhal_gmac_chip_phyrd)(etc->ch, 0, 0);
 */

/*******************************************************************************
 *
 * Macros / Defines
 *
 *******************************************************************************/

    /* DMA events that we register for to get callbacks
     * NOTE: These are unsupported DMA events CYHAL_DMA_SRC_MISAL | CYHAL_DMA_DST_MISAL | CYHAL_DMA_CURR_PTR_NULL | CYHAL_DMA_ACTIVE_CH_DISABLED
     */
#define  CYHAL_ETHERNET_DMA_EVENTS   (CYHAL_DMA_TRANSFER_COMPLETE | CYHAL_DMA_DESCRIPTOR_COMPLETE |                     \
                                      CYHAL_DMA_SRC_BUS_ERROR |  CYHAL_DMA_DST_BUS_ERROR | CYHAL_DMA_DESCR_BUS_ERROR)

/*******************************************************************************
 *
 * Variables / Data
 *
 *******************************************************************************/
static const cyhal_resource_inst_t ethernet_hw_instance = { CYHAL_RSC_ETHERNET, 0, 0 };

static cyhal_ethernet_t   *caller_obj;
static etc_info_t etc_local;

/* Timer object used for Ethernet maintenance */
static cyhal_timer_t ethernet_maintenance_timer;

/* Timer period for the maintenance timer ms */
#define CYHAL_ETHER_MAINTENAN_TIMER_PERIOD_MS       ( 100)
#define MILLISECONDS_PER_SECOND                     (1000)

/*******************************************************************************
 *
 * Prototypes
 *
 *******************************************************************************/

/*******************************************************************************
 *
 * Internal Functions
 *
 *******************************************************************************/
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
static int _cyhal_ethernet_hex_dump_print(const void* data_ptr, uint16_t length)
{
    uint8_t*  data = (uint8_t*)data_ptr;
    int i, count;


    if ((data == NULL) || (length == 0))
    {
        return -1;
    }

    count = 0;
    while (length > 0)
    {

        i = 0;
        while ((length > 0) && (i < 16))
        {
            _CYHAL_ETHERNET_LOG_NOTICE((" %02x", *data));
            i++;
            data++;
            length--;
            count++;
        }
        _CYHAL_ETHERNET_LOG_NOTICE(("\r\n"));
    }

    return count;
}
#endif  /* _CYHAL_ETHERNET_LOG_ENABLE */

cyhal_ethernet_event_t _get_enabled_events( void )
{
    return caller_obj->event_req;
}

/* Interrupt for Ethernet HAL */
PLATFORM_DEFINE_ISR( _cyhal_ethernet_isr )
{
    _CYHAL_ETHERNET_LOG_DEBUG(("Interrupt Ethernet HAL ISR: Ethernet: OBJ: %p\n", caller_obj));
    if ( (caller_obj != NULL) && (caller_obj->tag == CYHAL_ETHERNET_OBJ_TAG) )
    {
        cyhal_ethernet_event_t new_events = CYHAL_ETHERNET_EVENT_NONE;
        _cyhal_ethernet_info_t *et = &caller_obj->et_info;
        uint16_t interrupts = _cyhal_ethernet_et_getintrevents(et, true);

        if (interrupts & GMAC_INTR_NEW)
        {
            if (interrupts & GMAC_INTR_MDIO)
            {
            }
            if (interrupts & GMAC_INTR_RX)
            {
                new_events |= CYHAL_ETHERNET_RX_FRAME_EVENT;
            }
            if (interrupts & GMAC_INTR_TX)
            {
                new_events |= CYHAL_ETHERNET_TX_COMPLETE_EVENT;
            }
            if (interrupts & GMAC_INTR_ERROR)
            {
                new_events |= CYHAL_ETHERNET_TX_ERR_OCCURED_EVENT;
            }
#if 0   /* TODO: JIRA 4583 :: 43907 Ethernet 1EEE 1588 TSU Support */
            // CYHAL_ETHERNET_TSU_SECOND_INC_EVENT The TSU (Time Stamp Unit) second counter was incremented
            new_events |= CYHAL_ETHERNET_TSU_SECOND_INC_EVENT;
#endif

            if (new_events != CYHAL_ETHERNET_EVENT_NONE)
            {
                // We use the Ethernet HAL maintenance timer interrupt to call back to the Application.
                // Save the events in the caller's ethernet object.
                caller_obj->events_from_GMAC_irq |= new_events;
            }
        }
    }
}


/* Timer set to 10 per second (100ms) */
PLATFORM_MAP_ISR( _cyhal_ethernet_isr, GMAC_ISR )
static void _cyhal_ethernet_maintenance_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    CY_UNUSED_PARAMETER(event);
    cyhal_ethernet_t *obj = (cyhal_ethernet_t *)callback_arg;

    if ( obj->tag == CYHAL_ETHERNET_OBJ_TAG)
    {
        /* call routine to update the ethernet counters */
        /* refresh stat counters */

        _cyhal_ethernet_info_t* et = &obj->et_info;

        if ( et->started )
        {
            static int8_t once_per_second = MILLISECONDS_PER_SECOND / CYHAL_ETHER_MAINTENAN_TIMER_PERIOD_MS;
            if (--once_per_second <= 0)
            {
                _cyhal_ethernet_etc_watchdog( et->etc );
                once_per_second = MILLISECONDS_PER_SECOND / CYHAL_ETHER_MAINTENAN_TIMER_PERIOD_MS;
            }
        }

        if ( (obj->user_callback != NULL) && (obj->event_req != 0) )
        {
            /* TODO: JIRA 4583 :: 43907 Ethernet 1EEE 1588 TSU Support */

            cyhal_ethernet_event_t events = obj->events_from_GMAC_irq;
            obj->events_from_GMAC_irq = 0;

            events = events & _get_enabled_events();

            if (events != 0)
            {
                cyhal_ethernet_event_callback_t func = (cyhal_ethernet_event_callback_t)obj->user_callback;
                func(obj->user_data, events);
            }
        }
    }

}

static cy_rslt_t _cyhal_ethernet_maintenance_timer_init(cyhal_ethernet_t *obj)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;;

    const cyhal_timer_cfg_t ethernet_maintenance_timer_cfg =
    {
        .compare_value = (CYHAL_ETHER_MAINTENAN_TIMER_PERIOD_MS/2),  /* Timer compare value, less than 'period' and more than 'value' */
        .period = CYHAL_ETHER_MAINTENAN_TIMER_PERIOD_MS, /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = true,                 /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&ethernet_maintenance_timer, NC, NULL);

    /* Configure the timer */
    if(result == CY_RSLT_SUCCESS)
    {
        result= cyhal_timer_configure(&ethernet_maintenance_timer, &ethernet_maintenance_timer_cfg);
    }

    /* Set the frequency of timer's clock source */
    if(result == CY_RSLT_SUCCESS)
    {
        result =cyhal_timer_set_frequency(&ethernet_maintenance_timer, CYHAL_ETHER_MAINTENAN_TIMER_PERIOD_MS);
    }

    /* Assign the ISR to execute on timer interrupt */
    if(result == CY_RSLT_SUCCESS)
    {
        cyhal_timer_register_callback(&ethernet_maintenance_timer, _cyhal_ethernet_maintenance_timer_isr, obj);
    }

    /* Set the event on which timer interrupt occurs and enable it */
    if(result == CY_RSLT_SUCCESS)
    {
        cyhal_timer_enable_event(&ethernet_maintenance_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);
    }

    /* Start the timer with the configured settings */
    if(result == CY_RSLT_SUCCESS)
    {
        result=cyhal_timer_start(&ethernet_maintenance_timer);
    }

    if(result != CY_RSLT_SUCCESS)
    {
        /* Timer setup failed, free the timer. */
        cyhal_timer_free(&ethernet_maintenance_timer);
    }

    return result;
}

/* Driver functions */
static cy_rslt_t _cyhal_ethernet_config_phy_interface( cyhal_ethernet_t *obj, const cyhal_ethernet_config_t *eth_config )
{
    CY_UNUSED_PARAMETER(obj);
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t interface_val = GCI_CHIPCONTROL_GMAC_INTERFACE_MII;

    switch ( eth_config->drive_mode )
    {
    default:
    case CYHAL_ETHERNET_MII_10:
    case CYHAL_ETHERNET_MII_100:
    case CYHAL_ETHERNET_GMII_1000:
        interface_val = GCI_CHIPCONTROL_GMAC_INTERFACE_MII;
        break;

    case CYHAL_ETHERNET_RMII_10:
    case CYHAL_ETHERNET_RMII_100:
    case CYHAL_ETHERNET_RGMII_10:
    case CYHAL_ETHERNET_RGMII_100:
    case CYHAL_ETHERNET_RGMII_1000:
        interface_val = GCI_CHIPCONTROL_GMAC_INTERFACE_RMII;
        break;
    }

    _cyhal_system_gci_gpiocontrol( GCI_CHIPCONTROL_GMAC_INTERFACE_REG,
                              GCI_CHIPCONTROL_GMAC_INTERFACE_MASK,
                              interface_val);

    return result;
}

static cy_rslt_t _cyhal_ethernet_config_force_speed( _cyhal_ethernet_info_t *et, cyhal_ethernet_mac_drive_mode_t mode )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int       force_mode   = GMAC_FORCE_SPEED_10FULL;

    switch(mode)
    {
    case CYHAL_ETHERNET_MII_10:     /* MII type with link speed up to 10 Mbps */
        force_mode = GMAC_FORCE_SPEED_10FULL;
        break;
    case CYHAL_ETHERNET_MII_100:    /* MII type with link speed up to 100 Mbps */
        force_mode = GMAC_FORCE_SPEED_100FULL;
        break;
    case CYHAL_ETHERNET_GMII_1000:  /* GMII type with link speed up to 1000 Mbps */
        force_mode = GMAC_FORCE_SPEED_1000FULL;
        break;
    case CYHAL_ETHERNET_RGMII_10:   /* RGMII type with link speed up to 10 Mbps */
        force_mode = GMAC_FORCE_SPEED_10HALF;
        break;
    case CYHAL_ETHERNET_RGMII_100:  /* RGMII type with link speed up to 100 Mbps */
        force_mode = GMAC_FORCE_SPEED_100HALF;
        break;
    case CYHAL_ETHERNET_RGMII_1000: /* RGMII type with link speed up to 1000 Mbps */
        force_mode = GMAC_FORCE_SPEED_1000HALF;
        break;
    case CYHAL_ETHERNET_RMII_10:    /* RMII type with link speed up to 10 Mbps */
        force_mode = GMAC_FORCE_SPEED_10HALF;
        break;
    case CYHAL_ETHERNET_RMII_100:   /* RMII type with link speed up to 100 Mbps */
        force_mode = GMAC_FORCE_SPEED_100HALF;
        break;
    default:
        result = CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
        break;
    }

    if (result == CY_RSLT_SUCCESS)
    {
        if (_cyhal_ethernet_etc_set_speed(et->etc, force_mode) < 0)
        {
            result = CYHAL_ETHERNET_RSLT_ERR_FAILED;
        }

    }
    return result;
}

static cy_rslt_t _cyhal_ethernet_read_gmac_addr_from_otp( cyhal_ethernet_t *obj )
{
    cy_rslt_t   result = CY_RSLT_SUCCESS;

    CY_ASSERT(obj != NULL);

    /* The mac is read from the OTP (One time programming) region.  */
    result = _cyhal_ethernet_etc_read_mac_addr_from_OTP( &obj->et_info.mac_address );
    if (result != CY_RSLT_SUCCESS)
    {
#ifndef ETHERNET_MAC_ADDRESS_DEFAULT
        _CYHAL_ETHERNET_LOG_WARNING(("WARNING: Unable to get MAC address from OTP, please define '#define ETHERNET_MAC_ADDRESS_DEFAULT 0x00A0504a4fc8ull' for development purposes\n"));
#else
        et->mac_address.octet[0]= (uint8_t)((ETHERNET_MAC_ADDRESS_DEFAULT >> 40) & 0xFF);
        et->mac_address.octet[1]= (uint8_t)((ETHERNET_MAC_ADDRESS_DEFAULT >> 32) & 0xFF);
        et->mac_address.octet[2]= (uint8_t)((ETHERNET_MAC_ADDRESS_DEFAULT >> 24) & 0xFF);
        et->mac_address.octet[3]= (uint8_t)((ETHERNET_MAC_ADDRESS_DEFAULT >> 16) & 0xFF);
        et->mac_address.octet[4]= (uint8_t)((ETHERNET_MAC_ADDRESS_DEFAULT >>  8) & 0xFF);
        et->mac_address.octet[5]= (uint8_t)((ETHERNET_MAC_ADDRESS_DEFAULT >>  0) & 0xFF);
#endif
    }

    return result;

}

static cy_rslt_t _cyhal_ethernet_init_clock( cyhal_ethernet_t *obj, const cyhal_clock_t *clk  )
{
    cy_rslt_t   result = CY_RSLT_SUCCESS;

    if (obj == NULL)
    {
        result = CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    if (result == CY_RSLT_SUCCESS)
    {
        /* reserve the clock */
        if (clk != NULL)
        {
            if ( (clk->block != _CYHAL_ETHERNET_CLOCK_BLOCK) || (clk->channel != _CYHAL_ETHERNET_CLOCK_CHANNEL) )
            {
                result = CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
            }
            else
            {
                obj->clk.block   = clk->block;
                obj->clk.channel = clk->channel;
            }
        }
        else
        {
            /* defaults here */
            obj->clk.block   = _CYHAL_ETHERNET_CLOCK_BLOCK;
            obj->clk.channel = _CYHAL_ETHERNET_CLOCK_CHANNEL;
        }

        if(result == CY_RSLT_SUCCESS)
        {
            result = cyhal_clock_reserve(&obj->clk, &obj->clk);
        }

    }

    return result;
}


static cy_rslt_t _cyhal_ethernet_init_1588_timer(void)
{
    static int init_1588_tsu_done = 0;

    if (init_1588_tsu_done == 0)
    {
        /*
         * ascu_control  0x18000200
         *
         * 8 _cyhal_gmac_timestamp_enable use GMAC tx/rx triggers to timestamp audio timers instead of MAC (default is MAC). RW 0
         *
         * 7 fw_timer_sample firmware forces the audio timer to sample (dedicated set of register). Writes of 1 to
         *   this field generate a single-cycle pulse. Writing a 0 to this field has no effect. RWT 0
         * 6 force_i2s1_ts_sync_start force i2s1 to start draining. RW 0
         * 5 force_i2s0_ts_sync_start force i2s0 to start draining. RW 0
         *
         * 4 network_timer_freeze freeze the network timer. RW 0
         * 3 network_timer_disable disable the network timer. RW 0
         *
         * 2 audio_timer_freeze freeze the audio timer. RW 0
         * 1 audio_timer_disable disable the audio timer. RW 0
         *
         * 0 enable_update write a 0->1 transition to enable the offset adjustment (not self-clearing). RW 0
         *
         */
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
        _CYHAL_ETHERNET_LOG_NOTICE(("\nREAD 1588 Timer\n"));

        uint32_t ascu_control = _CYHAL_GMAC_R_REG((volatile uint32_t *)0x18000200);
        _CYHAL_ETHERNET_LOG_NOTICE(("read ascu_control                   0x18000200     : 0x%08lx\n", ascu_control));
        ascu_control |= (1 << 8); // enable GMAC tx/Rx to trigger timestamp
        ascu_control |= (1 << 7); // fw_timer_sample trigger timestamp
        _CYHAL_ETHERNET_LOG_NOTICE(("write ascu_control                  0x18000200     : 0x%08lx\n", ascu_control));
        _CYHAL_GMAC_W_REG((volatile uint32_t *)0x18000200, ascu_control);
        ascu_control = _CYHAL_GMAC_R_REG((volatile uint32_t *)0x18000200);
        _CYHAL_ETHERNET_LOG_NOTICE(("read back ascu_control              0x18000200     : 0x%08lx\n\n", ascu_control));


        /*
         * DevControl register ( 0x18005000 )
         * Field   Name    Description R/W Reset Value
         * 9   avb_enable  This field is set to 1 to enable the AVB timestamp hardware. When this field contains 0, the AVBTimeStamp register will always contain 0.   R/W 1'b0
         */
        uint32_t DevControl = _CYHAL_GMAC_R_REG((volatile uint32_t *)0x18005000);
        _CYHAL_ETHERNET_LOG_NOTICE(("read DevControl                   0x18005000     : 0x%08lx\n", DevControl));
        DevControl |= (1 << 9); // enable GMAC tx/Rx to trigger timestamp
        _CYHAL_ETHERNET_LOG_NOTICE(("write DevControl                  0x18005000     : 0x%08lx\n", DevControl));
        _CYHAL_GMAC_W_REG((volatile uint32_t *)0x18005000, DevControl);
        DevControl = _CYHAL_GMAC_R_REG((volatile uint32_t *)0x18005000);
        _CYHAL_ETHERNET_LOG_NOTICE(("read back DevControl              0x18005000     : 0x%08lx\n\n", DevControl));
#endif  /* _CYHAL_ETHERNET_LOG_ENABLE */

        init_1588_tsu_done=1;
    }
    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
 *
 * External Functions
 *
 *******************************************************************************/

/* \addtogroup group_hal_ethernet_impl Ethernet
 * \ingroup group_hal_impl
 * \{
 *   \section section_hal_impl_ethernet_pins CAT4 Ethernet Pins
 *   The CAT4 MCU has exclusive GPIOs to connect the GMAC (Ethernet Giga-bit MAC) module
 *   directly to an external Ethernet PHY device. Please pass NULL for the 'eth_pins' argument
 *   to \ref cyhal_ethernet_init().
 *  \} group_hal_ethernet_impl
 *
 */
cy_rslt_t cyhal_ethernet_init(cyhal_ethernet_t *obj, const cyhal_ethernet_config_t *eth_config, const cyhal_ethernet_pins_t *eth_pins, const cyhal_clock_t *clk)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* check for valid args */
    if ( (obj == NULL) || (eth_config == NULL) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }
    if ( eth_pins != NULL )
    {
        _CYHAL_ETHERNET_LOG_INFO(("Ethernet pins on 43907 are fixed, 'eth_pins' argument will be ignored.\n"));
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    /* determine if already inited */
    result = cyhal_hwmgr_reserve(&ethernet_hw_instance);
    if (result != CY_RSLT_SUCCESS)
    {
        return CYHAL_ETHERNET_RSLT_ERR_ALREADY_INITED;
    }

    /* Check clock configuration */
    if ( (eth_config->clk_source == CYHAL_ETHERNET_CLK_INT) && (eth_config->ext_clk_freq != 0) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }
    if ( (eth_config->clk_source == CYHAL_ETHERNET_CLK_EXT) && (eth_config->ext_clk_freq == 0) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    /* Clear the structure */
    memset(obj, 0x00, sizeof(cyhal_ethernet_t));

    if (result == CY_RSLT_SUCCESS)
    {
        /* JIRA 4685 :: 43907 Ethernet powersave support */
        //    result = platform_mcu_powersave_request_clock( PLATFORM_MCU_POWERSAVE_CLOCK_BACKPLANE_ON_HT );
    }

    /* Configure necessary resources */
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_ethernet_init_clock( obj, clk );
    }

    /* Configure PHY interface */
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_ethernet_config_phy_interface( obj, eth_config );
    }

    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_ethernet_read_gmac_addr_from_otp( obj );
    }

    /* Initialize DMA reading buffers*/
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_gmac_host_read_buffer_init();
    }

    /* Initialize GMAC sub system */
    if (result == CY_RSLT_SUCCESS)
    {
        obj->et_info.etc = &etc_local;

        _cyhal_ethernet_etc_attach( &obj->et_info,
                _CYHAL_VENDOR_BROADCOM,
                _CYHAL_BCM47XX_GMAC_ID,
                NULL /* regsva */ );
        if ( obj->et_info.etc == NULL)
        {
            result = CYHAL_ETHERNET_RSLT_ERR_INIT_FAILED;
        }
    }

    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_ethernet_config_force_speed( &obj->et_info, eth_config->drive_mode );
    }

    /* Initialize Ethernet Maintenance timer - we read/update GMAC counters in the maintenance ISR and perform user callbacks */
    if (result == CY_RSLT_SUCCESS)
    {
        result = _cyhal_ethernet_maintenance_timer_init(obj);
    }

    if (result == CY_RSLT_SUCCESS)
    {
        /* Enable the IRQs for GMAC */
        platform_irq_enable_irq(GMAC_ExtIRQn);

        /* bring Ethernet up */
        _cyhal_ethernet_et_up( &obj->et_info );

        /* Initialize TSU registers */
        result = _cyhal_ethernet_init_1588_timer();
    }

    if (result == CY_RSLT_SUCCESS)
    {
        caller_obj = obj;                       /* save for interrupt use */
        obj->tag = CYHAL_ETHERNET_OBJ_TAG;
        obj->et_info.started = true;
    }

    if(CY_RSLT_SUCCESS != result)
    {
        cyhal_ethernet_free(obj);
    }

    return result;
}

void cyhal_ethernet_free(cyhal_ethernet_t *obj)
{
    /* Check for an initialized object */
    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return; // CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    obj->tag = 0xDEADBEEF;
    caller_obj = NULL;

    /* Disable the IRQs for GMAC */
    platform_irq_disable_irq(GMAC_ExtIRQn);

    /* Stop and free the Ethernet maintenance timer */
    cyhal_timer_free(&ethernet_maintenance_timer);

    /* Bring down the Ethernet support and free low-level resources */
    _cyhal_ethernet_et_down( &obj->et_info, TRUE );

    obj->et_info.started = false;

    _cyhal_ethernet_etc_detach( (etc_info_t *)obj->et_info.etc );

    /* Clean up DMA reading buffers */
    _cyhal_gmac_host_read_buffer_deinit();

    /* free the clock */
    if (obj->clk.reserved)
    {
        cyhal_clock_free(&obj->clk);
    }

    /* Free the PWM (PHY reset gpio) */
    _cyhal_ethernet_et_phy_free(&obj->et_info);


    /* JIRA 4685 :: 43907 Ethernet powersave support */
    //    platform_mcu_powersave_release_clock( PLATFORM_MCU_POWERSAVE_CLOCK_BACKPLANE_ON_HT );

    cyhal_hwmgr_free(&ethernet_hw_instance);

}

/* For most platforms, this function would init using the configurator output.
 * However, CAT4 MCUs are not supported by the configurator at this time.
 */
cy_rslt_t cyhal_ethernet_init_cfg(cyhal_uart_t *obj, const cyhal_ethernet_configurator_t *cfg)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);
    return CYHAL_ETHERNET_RSLT_NOT_SUPPORTED;
}

void cyhal_ethernet_register_callback(cyhal_ethernet_t *obj, cyhal_ethernet_event_callback_t callback, void *callback_arg)
{
    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return;     // CYHAL_ETHERNET_RSLT_NOT_INITIALIZED
    }

    /*
     * Add callback and arg to object
     */
    obj->events_from_GMAC_irq = 0;
    obj->user_callback = callback;
    obj->user_data     = callback_arg;
}

void cyhal_ethernet_enable_event(cyhal_ethernet_t *obj, cyhal_ethernet_event_t event, uint8_t intr_priority, bool enable)
{
    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return;     // CYHAL_ETHERNET_RSLT_NOT_INITIALIZED
    }

    obj->intr_priority = intr_priority;;

    /* Are just adding this event ? */
    if ( (obj->event_req & event) == 0)
    {
        /* first time adding this event, clear any previous events received */
        obj->events_from_GMAC_irq &= ~event;
    }

    if(enable)
    {
        obj->event_req |= event;
    }
    else
    {
        obj->event_req &= ~(event);
    }
}

cy_rslt_t cyhal_ethernet_transmit_frame(cyhal_ethernet_t *obj, uint8_t *frame_data, uint16_t size)
{
    etc_info_t *etc = NULL;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }
    if ( (frame_data == NULL) || (size == 0) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }
    if (size < CYHAL_ETHER_MIN_LEN)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    /* finish setting up the packet header */
    cyhal_ether_header_t *header = (cyhal_ether_header_t *)frame_data;
    memcpy (header->ether_shost, &obj->et_info.mac_address.octet, CYHAL_ETHER_ADDR_LEN);

    if (header->ether_type == 0)
    {
        if (size < CYHAL_ETHER_TYPE_MIN)
        {
            header->ether_type = size;
        }
    }

#ifdef _CYHAL_ETHERNET_LOG_ENABLE
    _CYHAL_ETHERNET_LOG_DEBUG(("Packet to transmit: buffer: %p    sz: %d\n", frame_data, size));
    _cyhal_ethernet_hex_dump_print( (const void*)frame_data, size);
#endif

    etc = (etc_info_t *)obj->et_info.etc;
    if (_cyhal_ethernet_etc_transmit( etc, frame_data, size ) == false )
    {
        result = CYHAL_ETHERNET_RSLT_TX_FAILED;
    }
    else
    {
        etc->txframe++;
        etc->txbyte += size;
    }

    return result;
}


cy_rslt_t cyhal_ethernet_read_frame(cyhal_ethernet_t *obj, uint8_t *frame_data, uint16_t *size)
{
    uint16_t    avail_buffer_size = 0;
    _cyhal_ethernet_info_t   *et  = NULL;
    etc_info_t  *etc = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }
    if ( (frame_data == NULL) || (size == NULL) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }
    if (*size < CYHAL_ETHER_MIN_LEN)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    et  = &obj->et_info;
    etc = (etc_info_t *)et->etc;


    /* get available size for passing to lower level code */
    avail_buffer_size = *size;

    /* Clear size to reflect actual result of copy operation. */
    *size = 0;

    /* Clear out the buffer before reading data */
    memset(frame_data, 0x00, avail_buffer_size);

    /* Read data */
    uint32_t data_in_size = _cyhal_ethernet_etc_receive_and_copy(etc, frame_data, avail_buffer_size);

    if (data_in_size > 0)
    {
#ifdef _CYHAL_ETHERNET_LOG_ENABLE
        _CYHAL_ETHERNET_LOG_DEBUG(("%s() Received packet: %p   0x%lx bytes\n", __func__, frame_data, data_in_size));
        _cyhal_ethernet_hex_dump_print( (const void*)frame_data, data_in_size);
#endif

        /* return actual copied count to caller */
        *size = data_in_size;
        return CY_RSLT_SUCCESS;
    }

    return CYHAL_ETHERNET_RSLT_RX_FAILED;
}

/* \addtogroup group_hal_ethernet_impl Ethernet
 * \ingroup group_hal_impl
 * \{
 *   \section section_hal_impl_ethernet_tsu CAT4 Ethernet TSU Support
 *   This feature is currently not available.
 *  \} group_hal_ethernet_impl
 */
cy_rslt_t cyhal_ethernet_get_1588_timer_value(cyhal_ethernet_t *obj, cyhal_ethernet_1588_timer_val_t *timer_val)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(timer_val);

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    return CYHAL_ETHERNET_RSLT_NOT_SUPPORTED;
}

cy_rslt_t cyhal_ethernet_set_1588_timer_value(cyhal_ethernet_t *obj, const cyhal_ethernet_1588_timer_val_t *timer_val)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(timer_val);

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    return CYHAL_ETHERNET_RSLT_NOT_SUPPORTED;
}

/*
 * \addtogroup group_hal_ethernet_impl Ethernet
 * \ingroup group_hal_impl
 * \{
 *   \section section_hal_impl_ethernet_pause_frame CAT4 Ethernet Pause Frame
 *   This feature is currently not available on CAT4 MCUs.
 *  \} group_hal_ethernet_impl
 */
cy_rslt_t cyhal_ethernet_config_pause(cyhal_ethernet_t *obj, const uint16_t pause_quanta)
{
    CY_UNUSED_PARAMETER(pause_quanta);

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    return CYHAL_ETHERNET_RSLT_NOT_SUPPORTED;
}

cy_rslt_t cyhal_ethernet_transmit_pause_frame(cyhal_ethernet_t *obj, const bool start_pause)
{
    CY_UNUSED_PARAMETER(start_pause);

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    return CYHAL_ETHERNET_RSLT_NOT_SUPPORTED;
}

uint8_t cyhal_ethernet_get_num_filter(cyhal_ethernet_t *obj)
{
    uint8_t num_filters = 0;

    if ( (obj != NULL) && (obj->tag == CYHAL_ETHERNET_OBJ_TAG) )
    {
        num_filters = CYHAL_ETHERNET_NUM_FILTERS;
    }

    return num_filters;
}

cy_rslt_t cyhal_ethernet_set_filter_address(cyhal_ethernet_t *obj, const uint8_t filter_num, const cyhal_ethernet_filter_config_t *config)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    if (cyhal_ethernet_get_num_filter(obj) == 0)
    {
        return CYHAL_ETHERNET_RSLT_NOT_SUPPORTED;
    }

    if (filter_num >= cyhal_ethernet_get_num_filter(obj) )
    {
        /* Filter index > Number of filters available */
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    if (CYHAL_ETHER_IS_NULL_ADDR(&config->mac_addr) )
    {
        /* Caller wants to remove the addr from the GMAC white list */
        result = _cyhal_gmac_broadcast_filter_remove(filter_num);
    }
    else
    {
        result = _cyhal_gmac_broadcast_filter_add(filter_num, config);
    }

    return result;
}


cy_rslt_t cyhal_ethernet_set_promiscuous_mode(cyhal_ethernet_t *obj, const bool enable_promiscuous)
{
    cy_rslt_t  result = CY_RSLT_SUCCESS;
    etc_info_t *etc   = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    etc = obj->et_info.etc;
    _cyhal_ethernet_etc_promisc(etc, (uint16_t)enable_promiscuous);

    return result;
}

cy_rslt_t cyhal_ethernet_enable_broadcast_receive(cyhal_ethernet_t *obj, const bool enable_broadcast)
{
    cy_rslt_t  result = CY_RSLT_SUCCESS;
    etc_info_t *etc   = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }

    etc = obj->et_info.etc;
    if (_cyhal_ethernet_etc_broadcast_receive_enable(etc, enable_broadcast) < 0)
    {
        result = CYHAL_ETHERNET_RSLT_ERR_FAILED;
    }

    if (result == CY_RSLT_SUCCESS)
    {
        etc->broadcast_enable = enable_broadcast;
    }

    return result;
}


/* 4390x GMAC registers are 4-byte values, 4-byte aligned */
cy_rslt_t cyhal_ethernet_phy_reg_write(cyhal_ethernet_t *obj, const uint8_t reg_number_in, const uint32_t value)
{
    cy_rslt_t  result = CY_RSLT_SUCCESS;
    etc_info_t  *etc  = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }
    if ( reg_number_in >= MAXPHYREG)
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    etc = obj->et_info.etc;
    if( _cyhal_ethernet_etc_phywrite(etc, reg_number_in, value) < 0)
    {
        result = CYHAL_ETHERNET_RSLT_ERR_FAILED;
    }
    return result;
}

/* 4390x GMAC registers are 4-byte values, 4-byte aligned */
cy_rslt_t cyhal_ethernet_phy_reg_read(cyhal_ethernet_t *obj, const uint8_t reg_number_in, uint32_t *value)
{
    uint32_t   return_value = 0;
    etc_info_t  *etc        = NULL;

    if ( (obj == NULL) || (obj->tag != CYHAL_ETHERNET_OBJ_TAG) )
    {
        return CYHAL_ETHERNET_RSLT_NOT_INITIALIZED;
    }
    if ( (reg_number_in >= MAXPHYREG) || (value == NULL) )
    {
        return CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG;
    }

    etc = obj->et_info.etc;
    return_value = _cyhal_ethernet_etc_phyread(etc, reg_number_in);

    *value = return_value;

    return CY_RSLT_SUCCESS;
}

#if defined(__cplusplus)
}
#endif

