/***************************************************************************//**
* \file cyhal_pin_package.h
*
* Description:
* Provides definitions for the pinout for each supported device.
*
********************************************************************************
* \copyright
* Copyright 2018-2022 Cypress Semiconductor Corporation (an Infineon company) or
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
* \addtogroup group_hal_impl_pin_package Pins
* \ingroup group_hal_impl
* \{
* Definitions for the pinout for each supported device
*/

#pragma once

#include "cyhal_hw_resources.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/** Definitions for all of the pins that are bonded out on in the package for the 43907. */
typedef enum {
    NC = 0xFF, //!< No Connect/Invalid Pin

    PIN_GPIO_0       = 0x00,
    PIN_GPIO_1       = 0x01,
    PIN_GPIO_2       = 0x02,
    PIN_GPIO_3       = 0x03,
    PIN_GPIO_4       = 0x04,
    PIN_GPIO_5       = 0x05,
    PIN_GPIO_6       = 0x06,
    PIN_GPIO_7       = 0x07,
    PIN_GPIO_8       = 0x08,
    PIN_GPIO_9       = 0x09,
    PIN_GPIO_10      = 0x0A,
    PIN_GPIO_11      = 0x0B,
    PIN_GPIO_12      = 0x0C,
    PIN_GPIO_13      = 0x0D,
    PIN_GPIO_14      = 0x0E,
    PIN_GPIO_15      = 0x0F,
    PIN_GPIO_16      = 0x10,
    PIN_SDIO_CLK     = 0x11,
    PIN_SDIO_CMD     = 0x12,
    PIN_SDIO_DATA_0  = 0x13,
    PIN_SDIO_DATA_1  = 0x14,
    PIN_SDIO_DATA_2  = 0x15,
    PIN_SDIO_DATA_3  = 0x16,
    PIN_UART0_CTS    = 0x17,
    PIN_UART0_RTS    = 0x18,
    PIN_UART0_RXD    = 0x19,
    PIN_UART0_TXD    = 0x1A,
    PIN_PWM_0        = 0x1B,
    PIN_PWM_1        = 0x1C,
    PIN_PWM_2        = 0x1D,
    PIN_PWM_3        = 0x1E,
    PIN_PWM_4        = 0x1F,
    PIN_PWM_5        = 0x20,
    PIN_RF_SW_CTRL_5 = 0x21,
    PIN_RF_SW_CTRL_6 = 0x22,
    PIN_RF_SW_CTRL_7 = 0x23,
    PIN_RF_SW_CTRL_8 = 0x24,
    PIN_RF_SW_CTRL_9 = 0x25,
    PIN_SPI_0_MISO   = 0x26,
    PIN_SPI_0_CLK    = 0x27,
    PIN_SPI_0_MOSI   = 0x28,
    PIN_SPI_0_CS     = 0x29,
    PIN_I2C0_SDATA   = 0x2A,
    PIN_I2C0_CLK     = 0x2B,
    PIN_I2S_MCLK0    = 0x2C,
    PIN_I2S_SCLK0    = 0x2D,
    PIN_I2S_LRCLK0   = 0x2E,
    PIN_I2S_SDATAI0  = 0x2F,
    PIN_I2S_SDATAO0  = 0x30,
    PIN_I2S_SDATAO1  = 0x31,
    PIN_I2S_SDATAI1  = 0x32,
    PIN_I2S_MCLK1    = 0x33,
    PIN_I2S_SCLK1    = 0x34,
    PIN_I2S_LRCLK1   = 0x35,
    PIN_SPI_1_CLK    = 0x36,
    PIN_SPI_1_MISO   = 0x37,
    PIN_SPI_1_MOSI   = 0x38,
    PIN_SPI_1_CS     = 0x39,
    PIN_I2C1_CLK     = 0x3A,
    PIN_I2C1_SDATA   = 0x3B
} cyhal_gpio_t;

/** Definitions for valid pin mux selection */
typedef enum {
    PIN_MUX_SEL_0   =   0,
    PIN_MUX_SEL_1   =   1,
    PIN_MUX_SEL_2   =   2,
    PIN_MUX_SEL_3   =   3,
    PIN_MUX_SEL_4   =   4,
    PIN_MUX_SEL_5   =   5,
    PIN_MUX_SEL_6   =   6,
    PIN_MUX_SEL_7   =   7,
    PIN_MUX_SEL_8   =   8,
    PIN_MUX_SEL_9   =   9,
    PIN_MUX_SEL_10  =   10,
    PIN_MUX_SEL_11  =   11,
    PIN_MUX_SEL_NA  =   12
} cyhal_mux_sel_t;

/* Connection type definition */
/** Represents an association between a pin and a resource */
typedef struct
{
    uint8_t                     block_num;      //!< The associated resource block number
    uint8_t                     channel_num;    //!< The associated resource block's channel number
    cyhal_gpio_t                pin;            //!< The GPIO pin
    cyhal_mux_sel_t             function;       //!< The pin function index value
} cyhal_resource_pin_mapping_t;

/** Represents an association between a pin and GCI register */
typedef struct
{
    int8_t gci_chipcontrol_reg;  //!< The GCI ChipControl register
    int8_t gci_chipcontrol_pos;  //!< The GCI ChipControl register shift
} cyhal_pin_gci_mapping_t;

#define GCI_DIRECT_CONNECT       (0x7F)   /**< Direct pin connection */
#define GCI_PIN_MASK_2BITS       (0x03UL) /**< 2-bit mask for GCI CC register */
#define GCI_PIN_MASK_4BITS       (0x0FUL) /**< 4-bit mask for GCI CC register */

extern const cyhal_pin_gci_mapping_t cyhal_pin_gci_map[60]; //!< Pin to GCI map LUT

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2c_scl[7]; //!< I2C_SCL pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2c_sda[7]; //!< I2C_SDA pin map

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_mclk[2]; //!< I2S_MCLK pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_sclk[2]; //!< I2S_SCLK pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_lrclk[2]; //!< I2S_LRCLK pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_sdatai[2]; //!< I2S_SDATAI pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_sdatao[2]; //!< I2S_SDATAO pin map

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_clk[1]; //!< SDIO_CLK pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_cmd[1]; //!< SDIO_CMD pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data0[1]; //!< SDIO_DATA0 pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data1[1]; //!< SDIO_DATA1 pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data2[1]; //!< SDIO_DATA2 pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data3[1]; //!< SDIO_DATA3 pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_sep_int[2]; //!< SDIO_SEP_INT pin map

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_clk[5]; //!< SPI_CLK pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_miso[5]; //!< SPI_MISO pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_mosi[5]; //!< SPI_MOSI pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_cs[5]; //!< SPI_CS pin map

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_pwm_line[42]; //!< PWM_LINE pin map

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_cts[3]; //!< UART_CTS pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_rts[3]; //!< UART_RTS pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_rx[7]; //!< UART_RX pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_tx[7]; //!< UART_TX pin map

extern const cyhal_resource_pin_mapping_t cyhal_pin_map_usbpd_afc_tx_data[1]; //!< USBPD_AFC_TX pin map
extern const cyhal_resource_pin_mapping_t cyhal_pin_map_usbpd_afc_tx_data_en[1]; //!< USBPD_AFC_TX_DATA_EN pin map

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/** \} group_hal_impl */
