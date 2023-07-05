/***************************************************************************//**
* \file cyhal_ethernet.h
*
* \brief
* Provides a high level interface for interacting with the Infineon Ethernet sub system.
* This interface abstracts out the chip specific details. If any chip specific
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

/**
* \addtogroup group_hal_ethernet Ethernet (Ethernet interface)
* \ingroup group_hal
* \{
* High level interface for interacting with the Ethernet.
*
* This driver supports initializing and configuring Infineon Ethernet MAC along with third party Ethernet PHY.
* The driver implements the functions for transmitting and receiveing Ethernet frames over the LAN network.
*
* \note Certain platforms may not support all of the Ethernet MAC to PHY interface mode MII, RMII, GMII and RGMII.
* Please refer to implementation specific documentation for details on available options.
*
* \section section_ethernet_features Features
*
* * Ethernet initialization
* * Send and receive ethernet packets
* * Ethernet flow control
* * MAC filters
* * Ethernet PHY low-level register control
* * Configurable interrupt and callback assignment from ethernet events - \ref cyhal_ethernet_event_t
*
* \section subsection_ethernet_sample_snippets Code Snippets
*
* \subsection subsection_ethernet_snippet_1 Snippet 1: Transmit Frame
* The following snippet transmits an ethernet frame
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_transmit_frame
*
* \subsection subsection_ethernet_snippet_2 Snippet 2: Receive Frame
* The following snippet receives an ethernet frame
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_receive_frame
*
* \subsection subsection_ethernet_snippet_3 Snippet 3: Flow control
* The following snippet demonstrates using pause frames for flow control
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_flow_control
*
* \subsection subsection_ethernet_snippet_4 Snippet 4: MAC filtering
* The following snippet demonstrates filtering packets by MAC address

* \snippet hal_ethernet.c snippet_cyhal_ethernet_mac_filter
*
* \subsection subsection_ethernet_snippet_5 Snippet 5: 1588 timer set
* The following snippet demonstrates setting the 1588 timer value
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_set_1588_timer_value
*
* \subsection subsection_ethernet_snippet_6 Snippet 6: 1588 timer get
* The following snippet demonstrates getting the 1588 timer value
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_get_1588_timer_value
*
* \subsection subsection_ethernet_snippet_7 Snippet 7: PHY Register Write
* The following snippet demonstrates writing a low-level PHY register
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_phy_write
*
* \subsection subsection_ethernet_snippet_8 Snippet 8: PHY Register Read
* The following snippet demonstrates reading a low-level PHY register
*
* \snippet hal_ethernet.c snippet_cyhal_ethernet_phy_read
*
*/


#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cy_result.h"
#include "cyhal_general_types.h"
#include "cyhal_gpio.h"
#include "cyhal_hw_types.h"

#if defined(__cplusplus)
extern "C" {
#endif


/** \addtogroup group_hal_results_ethernet Ethernet HAL Results
 *  Ethernet specific return codes
 *  \ingroup group_hal_results
 *  \{ *//**
 */


/** An invalid pin location was specified */
#define CYHAL_ETHERNET_RSLT_ERR_INVALID_PIN                  \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  0))
/** An argument was provided */
#define CYHAL_ETHERNET_RSLT_ERR_INVALID_ARG                  \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  1))
/** Initialization of the ethernet hardware failed*/
#define CYHAL_ETHERNET_RSLT_ERR_INIT_FAILED                  \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  2))
/** Trying to initialize an Ethernet Object that is already initialized*/
#define CYHAL_ETHERNET_RSLT_ERR_ALREADY_INITED                  \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  3))
/** Function failed*/
#define CYHAL_ETHERNET_RSLT_ERR_FAILED                  \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  4))
/** Etherent has not been initialized. */
#define CYHAL_ETHERNET_RSLT_NOT_INITIALIZED \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  5))
/** The requested clock frequency could not be achieved */
#define CYHAL_ETHERNET_RSLT_ERR_CLOCK \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  6))
/** The requested configuration is not supported. */
#define CYHAL_ETHERNET_RSLT_NOT_SUPPORTED \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  7))
/** Transmit data not successful because of MAC/PHY busy transmitting previous data */
#define CYHAL_ETHERNET_RSLT_TX_FAILED \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  8))
/** Receive data not successful  */
#define CYHAL_ETHERNET_RSLT_RX_FAILED \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET,  9))
/** Multicast Filter address already added */
#define CYHAL_ETHERNET_RSLT_FILTER_ALREADY_ADDED \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET, 10))
/** No space for more Multicast Filter s */
#define CYHAL_ETHERNET_RSLT_FILTER_NO_SPACE \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_ETHERNET, 11))


/**
 * \}
 */

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**< cyhal_ethernet_t field 'tag' set to this value when when initialized */
#define CYHAL_ETHERNET_OBJ_TAG      (0xC025520C)


/** Return true for NULL Ethernet address (all 0x00) */
#define CYHAL_ETHER_IS_NULL_ADDR(ea)        \
        ( ( ( (const uint8 *)(ea))[0] |     \
            ( (const uint8 *)(ea))[1] |     \
            ( (const uint8 *)(ea))[2] |     \
            ( (const uint8 *)(ea))[3] |     \
            ( (const uint8 *)(ea))[4] |     \
            ( (const uint8 *)(ea))[5]) == 0)

/** Return true for Unicast Ethernet address */
#define CYHAL_ETHER_IS_UNICAST(ea)   ( ( (uint8 *)(ea)[0]) & 0x01)

/** Return true for Multicast Ethernet address (also returns true for Broadcast address) */
#define CYHAL_ETHER_IS_MULTICAST(ea)       ( ( (const uint8 *)(ea) )[0] & 1)

/** Return true for Broadcast Ethernet Address  (all 0xFF) */
#define CYHAL_ETHER_IS_BROADCAST(ea)            \
        ( ( ( (const uint8 *)(ea) )[0] &        \
            ( (const uint8 *)(ea) )[1] &        \
            ( (const uint8 *)(ea) )[2] &        \
            ( (const uint8 *)(ea) )[3] &        \
            ( (const uint8 *)(ea) )[4] &        \
            ( (const uint8 *)(ea) )[5]) == 0xff)

/** Compare two Ethernet addresses - assumes the pointers can be referenced as shorts */
#define CYHAL_ETHER_ADDR_CMP(a, b) ( ( ( (const uint16 *)(a))[0] ^ ( (const uint16 *)(b) )[0]) | \
                     ( ( (const uint16 *)(a) )[1] ^ ( (const uint16 *)(b) )[1]) | \
                     ( ( (const uint16 *)(a) )[2] ^ ( (const uint16 *)(b) )[2]))

/** Copy an Ethernet address - assumes the pointers can be referenced as shorts */
#define CYHAL_ETHER_ADDR_CPY(s, d)                          \
        do {                                                \
            ((uint16 *)(d))[0] = ((const uint16 *)(s))[0];  \
            ((uint16 *)(d))[1] = ((const uint16 *)(s))[1];  \
            ((uint16 *)(d))[2] = ((const uint16 *)(s))[2];  \
        } while (0)


/** \addtogroup group_hal_ethernet_header
 *  Some Ethernet Header types (see  IEEE 802.3 for full list)
 *  \{ *//**
 */
#define CYHAL_ETHER_TYPE_MIN        0x0600      /**< Ethernet header type less than MIN is a length */
#define CYHAL_ETHER_TYPE_IP         0x0800      /**< Ethernet header type IPv4 */
#define CYHAL_ETHER_TYPE_ARP        0x0806      /**< Ethernet header type ARP */

/**
 * \}
 */

/** Reference clock input selection for the Ethernet MAC */
typedef enum
{
    CYHAL_ETHERNET_CLK_EXT,                         /**< External off chip clock source from IO Pin */
    CYHAL_ETHERNET_CLK_INT,                         /**< Internally generated clock source */
} cyhal_ethernet_clk_src_t;


/** Ethernet MAC to PHY drive modes */
typedef enum
{
    CYHAL_ETHERNET_MII_10,         /**< Media-Independent Interface (MII) type with link speed upto 10 Mbps */
    CYHAL_ETHERNET_MII_100,        /**< Media-Independent Interface (MII) type with link speed upto 100 Mbps */
    CYHAL_ETHERNET_GMII_1000,      /**< Gigabit Media-Independent Interface (GMII) type with link speed upto 1000 Mbps */
    CYHAL_ETHERNET_RGMII_10,       /**< Reduced Gigabit Media-Independent Interface (RGMII) type with link speed upto 10 Mbps */
    CYHAL_ETHERNET_RGMII_100,      /**< Reduced Gigabit Media-Independent Interface (RGMII) type with link speed upto 100 Mbps */
    CYHAL_ETHERNET_RGMII_1000,     /**< Reduced Gigabit Media-Independent Interface (RGMII) type with link speed upto 1000 Mbps */
    CYHAL_ETHERNET_RMII_10,        /**< Reduced Media-Independent Interface (RMII) type with link speed upto 10 Mbps */
    CYHAL_ETHERNET_RMII_100,       /**< Reduced Media-Independent Interface (RMII) type with link speed upto 100 Mbps */
    CYHAL_ETHERNET_MODE_UNKNOWN,   /**< If we fail to set the mode */
} cyhal_ethernet_mac_drive_mode_t;


/** Pins to use for MAC to PHY interface for MII drive mode */
typedef struct {
    cyhal_gpio_t mdc;     //!< MDC (Management Clock) pin
    cyhal_gpio_t mdio;    //!< MDIO (Management Data) pin bi-direction
    cyhal_gpio_t tx_clk;  //!< Transmit clock PHY to MAC
    cyhal_gpio_t tx_d0;   //!< Transmit data line 0
    cyhal_gpio_t tx_d1;   //!< Transmit data line 1
    cyhal_gpio_t tx_d2;   //!< Transmit data line 2
    cyhal_gpio_t tx_d3;   //!< Transmit data line 3
    cyhal_gpio_t tx_en;   //!< Transmit enable
    cyhal_gpio_t tx_err;  //!< Transmit error signal (Option), make it NULL when not used
    cyhal_gpio_t rx_clk;  //!< Receive clock from PHY to MAC
    cyhal_gpio_t rx_d0;   //!< Receive data line 0
    cyhal_gpio_t rx_d1;   //!< Receive data line 1
    cyhal_gpio_t rx_d2;   //!< Receive data line 2
    cyhal_gpio_t rx_d3;   //!< Receive data line 3
    cyhal_gpio_t rx_dv;   //!< Receive data valid line
    cyhal_gpio_t rx_err;  //!< Receive error signal
    cyhal_gpio_t rx_crs;  //!< Receive carrier sense signal
    cyhal_gpio_t rx_col;  //!< Receive collision detect signal
} cyhal_ethernet_mii_pins_t;


/** Pins to use for MAC to PHY interface for RMII drive mode */
typedef struct {
    cyhal_gpio_t mdc;     //!< MDC (Management Clock) pin
    cyhal_gpio_t mdio;    //!< MDIO (Management Data) pin bi-direction
    cyhal_gpio_t ref_clk; //!< Reference clock signal
    cyhal_gpio_t tx_d0;   //!< Transmit data line 0
    cyhal_gpio_t tx_d1;   //!< Transmit data line 1
    cyhal_gpio_t tx_en;   //!< Transmit enable
    cyhal_gpio_t rx_d0;   //!< Receive data line 0
    cyhal_gpio_t rx_d1;   //!< Receive data line 1
    cyhal_gpio_t rx_dv_crs; //!< Receive data valid and carrier sense signal
    cyhal_gpio_t rx_err;  //!< Receive error signal
} cyhal_ethernet_rmii_pins_t;


/** Pins to use for MAC to PHY interface for GMII drive mode */
typedef struct {
    cyhal_gpio_t mdc;     //!< MDC (Management Clock) pin
    cyhal_gpio_t mdio;    //!< MDIO (Management Data) pin bi-direction
    cyhal_gpio_t gtx_clk; //!< Transmit clock
    cyhal_gpio_t tx_d0;   //!< Transmit data line 0
    cyhal_gpio_t tx_d1;   //!< Transmit data line 1
    cyhal_gpio_t tx_d2;   //!< Transmit data line 2
    cyhal_gpio_t tx_d3;   //!< Transmit data line 3
    cyhal_gpio_t tx_d4;   //!< Transmit data line 4
    cyhal_gpio_t tx_d5;   //!< Transmit data line 5
    cyhal_gpio_t tx_d6;   //!< Transmit data line 6
    cyhal_gpio_t tx_d7;   //!< Transmit data line 7
    cyhal_gpio_t tx_en;   //!< Transmit enable
    cyhal_gpio_t tx_err;  //!< Transmit error signal
    cyhal_gpio_t rx_clk;  //!< Receive clock signal
    cyhal_gpio_t rx_d0;   //!< Receive data line 0
    cyhal_gpio_t rx_d1;   //!< Receive data line 1
    cyhal_gpio_t rx_d2;   //!< Receive data line 2
    cyhal_gpio_t rx_d3;   //!< Receive data line 3
    cyhal_gpio_t rx_d4;   //!< Receive data line 4
    cyhal_gpio_t rx_d5;   //!< Receive data line 5
    cyhal_gpio_t rx_d6;   //!< Receive data line 6
    cyhal_gpio_t rx_d7;   //!< Receive data line 7
    cyhal_gpio_t rx_dv;   //!< Receive data valid line
    cyhal_gpio_t rx_err;  //!< Receive error signal
    cyhal_gpio_t rx_crs;  //!< Receive carrier sense signal
    cyhal_gpio_t rx_col;  //!< Receive collision detect signal
} cyhal_ethernet_gmii_pins_t;


/** Pins to use for MAC to PHY interface for RGMII drive mode */
typedef struct {
    cyhal_gpio_t mdc;     //!< MDC (Management Clock) pin
    cyhal_gpio_t mdio;    //!< MDIO (Management Data) pin bi-direction
    cyhal_gpio_t gtx_clk; //!< Transmit clock
    cyhal_gpio_t tx_d0;   //!< Transmit data line 0
    cyhal_gpio_t tx_d1;   //!< Transmit data line 1
    cyhal_gpio_t tx_d2;   //!< Transmit data line 2
    cyhal_gpio_t tx_d3;   //!< Transmit data line 3
    cyhal_gpio_t tx_ctl;  //!< Transmit control signal used for both transmit enable and transmit error indication
    cyhal_gpio_t rx_clk;  //!< Receive clock
    cyhal_gpio_t rx_d0;   //!< Receive data line 0
    cyhal_gpio_t rx_d1;   //!< Receive data line 1
    cyhal_gpio_t rx_d2;   //!< Receive data line 2
    cyhal_gpio_t rx_d3;   //!< Receive data line 3
    cyhal_gpio_t rx_ctl;  //!< Receive control signal used for both receive enable and receive error indication
} cyhal_ethernet_rgmii_pins_t;


/** Pin selection according to MAC to PHY drive mode.
 *   This is fixed per hardware design.
 */
typedef union {
  cyhal_ethernet_mii_pins_t  eth_type_mii;      /**< Pin information when drive mode is \ref CYHAL_ETHERNET_MII_10 or \ref CYHAL_ETHERNET_MII_100 */
  cyhal_ethernet_gmii_pins_t eth_type_gmii;     /**< Pin information when drive mode is \ref CYHAL_ETHERNET_GMII_1000 */
  cyhal_ethernet_gmii_pins_t eth_type_rgmii;    /**< Pin information when drive mode is \ref CYHAL_ETHERNET_RGMII_10 or \ref CYHAL_ETHERNET_RGMII_100 or \ref CYHAL_ETHERNET_RGMII_1000  */
  cyhal_ethernet_rmii_pins_t eth_type_rmii;     /**< Pin information when drive mode is \ref CYHAL_ETHERNET_RMII_10 or \ref CYHAL_ETHERNET_RMII_100 */
} cyhal_ethernet_pins_t;


/** Configuring 1588 TSU (Time Stamp Unit) timer in the MAC as part of IEEE 1588 (Precision Timing Protocol) enablement */
typedef struct
{
    uint16_t secs_upper;                /**< Upper 16 bits of seconds value */
    uint32_t secs_lower;                /**< Lower 32 bits of seconds value */
    uint32_t nano_secs;                 /**< Nanoseconds value (30 bits) */
} cyhal_ethernet_1588_timer_val_t;



/** Ethernet configuration */
typedef struct
{
    cyhal_ethernet_mac_drive_mode_t drive_mode;     /**< Ethernet drive mode configuration */
    cyhal_ethernet_clk_src_t        clk_source;     /**< Ethernet reference clock source */
    uint32_t                        ext_clk_freq;   /**< External off chip clock source frequency information in Hz
                                                     * If `clk_source` is \ref CYHAL_ETHERNET_CLK_EXT, this must be non-zero
                                                     * If `clk_source` is \ref CYHAL_ETHERNET_CLK_INT, this must be zero */
} cyhal_ethernet_config_t;

/** Configuration to filter Ethernet packets matching MAC address */
typedef enum
{
  CYHAL_ETHERNET_FILTER_TYPE_DESTINATION,  /**< Filter ethernet packet matching Destination MAC address */
  CYHAL_ETHERNET_FILTER_TYPE_SOURCE,       /**< Filter ethernet packet matching Source MAC address */
} cyhal_ethernet_filter_type_t;

/** Ethernet MAC filter configuration */
typedef struct
{
  cyhal_ethernet_filter_type_t filter_type; /**< Ethernet MAC filter for source/destination MAC adress */
  uint8_t mac_addr[6];                      /**< Store 6byte Ethernet MAC address */
  uint8_t ignore_bytes;                     /**< Ignore number of bytes for in the received packet before comparing (filtering)
                                            *    ignore_bytes = 0x01 implies first byte received should not be compared. */
} cyhal_ethernet_filter_config_t;

/**
  * @brief Ethernet Header
  *
  * IEEE 802.3 standard Ethernet Header
  */
typedef struct cyhal_ether_header {
    uint8_t     preamble[CYHAL_ETHER_PREAMBLE_LEN];     /**< Preamble 0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA, and 0xAB */
    uint8_t     ether_dhost[CYHAL_ETHER_ADDR_LEN];      /**< Destination host ethernet MAC */
    uint8_t     ether_shost[CYHAL_ETHER_ADDR_LEN];      /**< Source host ethernet MAC */
    uint16_t    ether_type;                             /**< Ethernet Packet Type */
} cyhal_ether_header_t;

/** @brief Enum for Ethernet events  */
typedef enum
{
    CYHAL_ETHERNET_EVENT_NONE               = 0,       /**< No event */
    CYHAL_ETHERNET_TX_COMPLETE_EVENT        = 1 << 1,  /**< A message transmission was completed. */
    CYHAL_ETHERNET_TX_ERR_OCCURED_EVENT     = 1 << 2,  /**< An error occurred while transmitting a message. */
    CYHAL_ETHERNET_RX_FRAME_EVENT           = 1 << 3,  /**< An ethernet message was received. If filtering is on, or broadcast/promiscuous mode turned off
                                                            You may get an event, and when you try to read, there will be no packet. */
    CYHAL_ETHERNET_TSU_SECOND_INC_EVENT     = 1 << 4,  /**< The TSU (Time Stamp Unit) second counter was incremented. */
    CYHAL_ETHERNET_DMA_ERR_OCCURED_EVENT    = 1 << 5,  /**< DMA xfer with GMAC error. */
} cyhal_ethernet_event_t;

/** Callback handler for Ethernet events */
typedef void (*cyhal_ethernet_event_callback_t)(void *callback_arg, cyhal_ethernet_event_t event);

/** Initialize the Ethernet MAC and PHY. It sets the default parameters for Ethernet
 *  peripheral, and configures the specifies pins.
 *  There multiple drive mode configuration between Ethernet MAC and PHY so the appropriate pin information union member must be populated.
 *  The Ethernet MAC can operate either internal clock (PLL) or external clock source via HSIO.
 *
 * @param[in,out] obj           Pointer to an Ethernet object. The caller must allocate the memory
 *                                for this object but the init function will initialize its contents.
 * @param[in]  eth_config       Initial ethernet block configuration contains information regarding the drive mode, reference clock, and reference clock frequency
 * @param[in]  eth_pins         Pins for MAC to PHY interface.
 * @param[in]  clk              Clock source to use for this instance. If NULL, a dedicated clock divider will be allocated for this instance.
 *                              If `eth_config.clk_source` is \ref CYHAL_ETHERNET_CLK_EXT, this must be NULL
 * @return The status of the init request
 */
cy_rslt_t cyhal_ethernet_init(cyhal_ethernet_t *obj, const cyhal_ethernet_config_t *eth_config, const cyhal_ethernet_pins_t *eth_pins, const cyhal_clock_t *clk);

/** Deinitialize an Ethernet object
 *
 * @param[in]  obj              Pointer to an Ethernet object
 */
void cyhal_ethernet_free(cyhal_ethernet_t *obj);

/** Initialize the Ethernet peripheral using a configurator generated configuration struct.
 *
 * @param[in]  obj              The Ethernet peripheral to configure
 * @param[in]  cfg              Configuration structure generated by a configurator.
 * @return The status of the operation
 */
cy_rslt_t cyhal_ethernet_init_cfg(cyhal_uart_t *obj, const cyhal_ethernet_configurator_t *cfg);

/** Register an Ethernet callback handler
 *
 * This function will be called when one of the events enabled by \ref cyhal_ethernet_enable_event occurs.
 *
 * @param[in] obj               Pointer to an Ethernet object
 * @param[in] callback          The callback handler which will be invoked when the interrupt fires
 * @param[in] callback_arg Generic argument that will be provided to the callback when called
 */
void cyhal_ethernet_register_callback(cyhal_ethernet_t *obj, cyhal_ethernet_event_callback_t callback, void *callback_arg);

/** Configure Ethernet events.
 *
 * When an enabled event occurs, the function specified by \ref cyhal_ethernet_register_callback will be called.
 *
 * @param[in] obj               Pointer to an Ethernet object
 * @param[in] event             The Ethernet event type
 * @param[in] intr_priority     The priority for NVIC interrupt events
 * @param[in] enable            True to turn on specified events, False to turn off
 */
void cyhal_ethernet_enable_event(cyhal_ethernet_t *obj, cyhal_ethernet_event_t event, uint8_t intr_priority, bool enable);


/** Transmits ethernet frame data. This functions returns immediately after copying data to MAC internal buffer.
 * Optional step, to monitor actual transmit completion, set callback \ref cyhal_ethernet_register_callback() and event notification
 * \ref cyhal_ethernet_enable_event() \ref CYHAL_ETHERNET_TX_COMPLETE_EVENT .
 * The function return error code \ref CYHAL_ETHERNET_RSLT_TX_FAILED if the MAC internal buffer occupied and busy transmitting.
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
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] frame_data        Pointer to ethernet frame data
 * @param[in] size              Transmit data size in number of bytes (size of frame_data buffer)
 * @return The status of the ethernet frame data transmit request
 */
cy_rslt_t cyhal_ethernet_transmit_frame(cyhal_ethernet_t *obj, uint8_t *frame_data, uint16_t size);

/** Read ethernet frame data. This function return 0 for size when there is no data is available for readback.
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[out] frame_data       Pointer to frame data location
 * @param[in,out] size          [in] The size of `frame_data`.
 *                              [out] Received frame data size in number of bytes
 * @return The status of the ethernet frame data transmit request
 */
cy_rslt_t cyhal_ethernet_read_frame(cyhal_ethernet_t *obj, uint8_t *frame_data, uint16_t *size);

/** Read IEEE 1588 TSU (Time Stamp Unit) programmed timer value
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[out] timer_val        Pointer to 1588 timer value.
 * @return The status of the 1588 timer value request
 */
cy_rslt_t cyhal_ethernet_get_1588_timer_value(cyhal_ethernet_t *obj, cyhal_ethernet_1588_timer_val_t *timer_val);

/** Program IEEE 1588 TSU (Time Stamp Unit) timer programmed
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] timer_val         Pointer to 1588 timer value
 * @return The status of the 1588 timer value setup
 */
cy_rslt_t cyhal_ethernet_set_1588_timer_value(cyhal_ethernet_t *obj, const cyhal_ethernet_1588_timer_val_t *timer_val);

/** A pause frame includes the period of pause time being requested, in the form of a two-byte (16-bit), unsigned integer (0 through 65535).
 *  This number is the requested duration of the pause. The pause time is measured in units of pause "quanta", where each unit is equal to 512 bit times.
 *  This interface configures the pause frame. To transmit the pause frame once configured, use \ref cyhal_ethernet_transmit_pause_frame.
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] pause_quanta      Pause quanta
 * @return The status of the pause frame configuration request
 */
cy_rslt_t cyhal_ethernet_config_pause(cyhal_ethernet_t *obj, const uint16_t pause_quanta);

/** Transmit a pause frame. Before it can be used, \ref cyhal_ethernet_config_pause must be called
 * at least once to configure the pause frame.
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] start_pause       Start/stop transmitting pause frames
 *                              True  - The pause frames will be transmitted for pause_quanta defined
 *                                      in cyhal_ethernet_config_pause()
 *                              False - Pause frame transmission is stopped
 * @return The status of the pause frame transmit request
 */
cy_rslt_t cyhal_ethernet_transmit_pause_frame(cyhal_ethernet_t *obj, const bool start_pause);


/** Get the number of MAC filters supported by the device.
 *
 * @param[in] obj               Pointer to an Ethernet object.
 *
 * @return The number of filters available
 */
uint8_t cyhal_ethernet_get_num_filter(cyhal_ethernet_t *obj);

/** Configure a source or destination MAC filter.
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] filter_num        MAC filter number to be configured. This must be less than the value returned
 *                                by `\ref cyhal_ethernet_get_num_filter`. If this filter number was previously
 *                                configured, it will be reconfigured.
 *
 * @param[in] config            MAC filter configuration.
 * @return The status of the MAC configuration request.
 */
cy_rslt_t cyhal_ethernet_set_filter_address(cyhal_ethernet_t *obj, const uint8_t filter_num, const cyhal_ethernet_filter_config_t *config);


/** Enable or disable receiving Ethernet traffic that would normally be excluded by MAC filters.
 *
 * @param[in] obj                   Pointer to an Ethernet object.
 * @param[in] enable_promiscuous    True (default behavior) - MAC configured to capture all the network traffic
 *                                  False - Ethernet MAC captures the ethernet frames as per the MAC filter setting
 * @return The status of the promiscuous mode enable/disable request.
 */
cy_rslt_t cyhal_ethernet_set_promiscuous_mode(cyhal_ethernet_t *obj, const bool enable_promiscuous);

/** Enable/disable receiving of broadcast type ethernet frames.
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] enable_broadcast  True (default behavior) - MAC is configured to capture broadcast type frames.
 *                              False - MAC will filter broadcast frames; so no broadcast frames passed on to upper layers.
 *
 * @return The status of the broadcast enable/disable request.
 */
cy_rslt_t cyhal_ethernet_enable_broadcast_receive(cyhal_ethernet_t *obj, const bool enable_broadcast);


/** Write low level PHY register.
 * This interface is to access Ethernet PHY registers specified in IEEE 802.3 standard.
 * There is basic register set address range 00h - 0Fh and vendor specific address range 10h to 1Fh.
 * For details on the vendor specific registers, see the Ethernet PHY datasheet.
 * Ethernet HAL driver don't perform vendor specific configuration and control hence the interface provided to user level.
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] reg_number        Ethernet PHY register number.
 * @param[in] value             Value to be written into register.
 * @return The status of the Ethernet PHY register write request.
 */
cy_rslt_t cyhal_ethernet_phy_reg_write(cyhal_ethernet_t *obj, const uint8_t reg_number, const uint32_t value);

/** Read low level PHY register, \ref cyhal_ethernet_phy_reg_write() for more details on the register access.
 *
 *
 * @param[in] obj               Pointer to an Ethernet object.
 * @param[in] reg_number        Ethernet PHY register number.
 * @param[out] value            Value readback from the PHY
 * @return The status of the Ethernet PHY register read request.
 */
cy_rslt_t cyhal_ethernet_phy_reg_read(cyhal_ethernet_t *obj, const uint8_t reg_number, uint32_t *value);

/* Include implementation header if defined in cyhal_hw_types.h */
#ifdef _CYHAL_ETHERNET_IMPL_HEADER
#include _CYHAL_ETHERNET_IMPL_HEADER
#endif

#if defined(__cplusplus)
}
#endif


/** \} group_hal_ethernet */
