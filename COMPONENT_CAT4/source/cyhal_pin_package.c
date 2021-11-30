/***************************************************************************//**
* \file cyhal_pin_package.c
*
* \brief
* 4390x device GPIO HAL header for package
*
********************************************************************************
* \copyright
* Copyright 2021 Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cyhal_pin_package.h"
#include "platform_appscr4.h"

// Based on WICED/platform/MCU/BCM4390x/platform_mcu_peripherals.c
// Based on WICED/platform/MCU/BCM4390x/peripherals/platform_pinmux.c
/*
 * BCM43909 supports pin multiplexing and function selection
 * in accordance with the below PinMux Table specification.
 *
 * Pads           0                   1              2                   3                   4              5                   6              7           8           9                 10                11
 * GPIO_0                             GPIO_0         FAST_UART_RX        I2C1_SDATA          PWM0           SPI1_MISO           PWM2           GPIO_12     GPIO_8                        PWM4              USB20H_CTL1
 * GPIO_1                             GPIO_1         FAST_UART_TX        I2C1_CLK            PWM1           SPI1_CLK            PWM3           GPIO_13     GPIO_9                        PWM5
 * GPIO_2                             GPIO_2                                                 GCI_GPIO_0                                                                TCK
 * GPIO_3                             GPIO_3                                                 GCI_GPIO_1                                                                TMS
 * GPIO_4                             GPIO_4                                                 GCI_GPIO_2                                                                TDI
 * GPIO_5                             GPIO_5                                                 GCI_GPIO_3                                                                TDO
 * GPIO_6                             GPIO_6                                                 GCI_GPIO_4                                                                TRST_L
 * GPIO_7                             GPIO_7         FAST_UART_RTS_OUT   PWM1                PWM3           SPI1_CS             I2C1_CLK       GPIO_15     GPIO_11     PMU_TEST_O                          PWM5
 * GPIO_8                             GPIO_8         SPI1_MISO           PWM2                PWM4           FAST_UART_RX                       GPIO_16     GPIO_12     TAP_SEL_P         I2C1_SDATA        PWM0
 * GPIO_9                             GPIO_9         SPI1_CLK            PWM3                PWM5           FAST_UART_TX                       GPIO_0      GPIO_13                       I2C1_CLK          PWM1
 * GPIO_10                            GPIO_10        SPI1_MOSI           PWM4                I2C1_SDATA     FAST_UART_CTS_IN    PWM0           GPIO_1      GPIO_14     PWM2              SDIO_SEP_INT      SDIO_SEP_INT_0D
 * GPIO_11                            GPIO_11        SPI1_CS             PWM5                I2C1_CLK       FAST_UART_RTS_OUT   PWM1           GPIO_7      GPIO_15     PWM3
 * GPIO_12                            GPIO_12        I2C1_SDATA          FAST_UART_RX        SPI1_MISO      PWM2                PWM4           GPIO_8      GPIO_16     PWM0              SDIO_SEP_INT_0D   SDIO_SEP_INT
 * GPIO_13                            GPIO_13        I2C1_CLK            FAST_UART_TX        SPI1_CLK       PWM3                PWM5           GPIO_9      GPIO_0      PWM1
 * GPIO_14                            GPIO_14        PWM0                FAST_UART_CTS_IN    SPI1_MOSI      I2C1_SDATA                         GPIO_10                 PWM4                                PWM2
 * GPIO_15                            GPIO_15        PWM1                FAST_UART_RTS_OUT   SPI1_CS        I2C1_CLK                           GPIO_11     GPIO_7      PWM5                                PWM3
 * GPIO_16                            GPIO_16        FAST_UART_CTS_IN    PWM0                PWM2           SPI1_MOSI           I2C1_SDATA     GPIO_14     GPIO_10     RF_DISABLE_L      USB20H_CTL2       PWM4
 * sdio_clk       TEST_SDIO_CLK       SDIO_CLK                                                                                                                         SDIO_AOS_CLK
 * sdio_cmd       TEST_SDIO_CMD       SDIO_CMD                                                                                                                         SDIO_AOS_CMD
 * sdio_data_0    TEST_SDIO_DATA_0    SDIO_D0                                                                                                                          SDIO_AOS_D0
 * sdio_data_1    TEST_SDIO_DATA_1    SDIO_D1                                                                                                                          SDIO_AOS_D1
 * sdio_data_2    TEST_SDIO_DATA_2    SDIO_D2                                                                                                                          SDIO_AOS_D2
 * sdio_data_3    TEST_SDIO_DATA_3    SDIO_D3                                                                                                                          SDIO_AOS_D3
 * rf_sw_ctrl_5                       rf_sw_ctrl_5   GCI_GPIO_5
 * rf_sw_ctrl_6                       rf_sw_ctrl_6   UART_DBG_RX         SECI_IN
 * rf_sw_ctrl_7                       rf_sw_ctrl_7   UART_DBG_TX         SECI_OUT
 * rf_sw_ctrl_8                       rf_sw_ctrl_8   SECI_IN             UART_DBG_RX
 * rf_sw_ctrl_9                       rf_sw_ctrl_9   SECI_OUT            UART_DBG_TX
 * PWM0                               PWM0           GPIO_2              GPIO_18
 * PWM1                               PWM1           GPIO_3              GPIO_19
 * PWM2                               PWM2           GPIO_4              GPIO_20
 * PWM3                               PWM3           GPIO_5              GPIO_21
 * PWM4                               PWM4           GPIO_6              GPIO_22
 * PWM5                               PWM5           GPIO_8              GPIO_23
 * SPI0_MISO                          SPI0_MISO      GPIO_17             GPIO_24
 * SPI0_CLK                           SPI0_CLK       GPIO_18             GPIO_25
 * SPI0_MOSI                          SPI0_MOSI      GPIO_19             GPIO_26
 * SPI0_CS                            SPI0_CS        GPIO_20             GPIO_27
 * I2C0_SDATA                         I2C0_SDATA     GPIO_21             GPIO_28
 * I2C0_CLK                           I2C0_CLK       GPIO_22             GPIO_29
 * i2s_mclk0                          i2s_mclk0      GPIO_23             GPIO_0
 * i2s_sclk0                          i2s_sclk0      GPIO_24             GPIO_2
 * i2s_lrclk0                         i2s_lrclk0     GPIO_25             GPIO_3
 * i2s_sdatai0                        i2s_sdatai0    GPIO_26             GPIO_4
 * i2s_sdatao0                        i2s_sdatao0    GPIO_27             GPIO_5
 * i2s_sdatao1                        i2s_sdatao1    GPIO_28             GPIO_6
 * i2s_sdatai1                        i2s_sdatai1    GPIO_29             GPIO_8
 * i2s_mclk1                          i2s_mclk1      GPIO_30             GPIO_17
 * i2s_sclk1                          i2s_sclk1      GPIO_31             GPIO_30
 * i2s_lrclk1                         i2s_lrclk1     GPIO_0              GPIO_31
 * SPI_1_MISO
 * SPI_1_CLK
 * SPI_1_MOSI
 * SPI_1_CS
 * I2C1_SDATA
 * I2C1_CLK
 */

/* Map the GCI register information to the pin pad. It's an LUT so it must be ordered the same as cyhal_gpio_t */
const cyhal_pin_gci_mapping_t cyhal_pin_gci_map[60] = {
    {GCI_CHIPCONTROL_REG_0, 0},  /* PIN_GPIO_0 */
    {GCI_CHIPCONTROL_REG_0, 4},  /* PIN_GPIO_1 */
    {GCI_CHIPCONTROL_REG_0, 8},  /* PIN_GPIO_2 */
    {GCI_CHIPCONTROL_REG_0, 12}, /* PIN_GPIO_3 */
    {GCI_CHIPCONTROL_REG_0, 16}, /* PIN_GPIO_4 */
    {GCI_CHIPCONTROL_REG_0, 20}, /* PIN_GPIO_5 */
    {GCI_CHIPCONTROL_REG_0, 24}, /* PIN_GPIO_6 */
    {GCI_CHIPCONTROL_REG_0, 28}, /* PIN_GPIO_7 */
    {GCI_CHIPCONTROL_REG_1, 0},  /* PIN_GPIO_8 */
    {GCI_CHIPCONTROL_REG_1, 4},  /* PIN_GPIO_9 */
    {GCI_CHIPCONTROL_REG_1, 8},  /* PIN_GPIO_10 */
    {GCI_CHIPCONTROL_REG_1, 12}, /* PIN_GPIO_11 */
    {GCI_CHIPCONTROL_REG_1, 16}, /* PIN_GPIO_12 */
    {GCI_CHIPCONTROL_REG_1, 20}, /* PIN_GPIO_13 */
    {GCI_CHIPCONTROL_REG_1, 24}, /* PIN_GPIO_14 */
    {GCI_CHIPCONTROL_REG_1, 28}, /* PIN_GPIO_15 */
    {GCI_CHIPCONTROL_REG_2, 24}, /* PIN_GPIO_16 */

    {GCI_CHIPCONTROL_REG_2, 0},  /* PIN_SDIO_CLK */
    {GCI_CHIPCONTROL_REG_2, 4},  /* PIN_SDIO_CMD */
    {GCI_CHIPCONTROL_REG_2, 8},  /* PIN_SDIO_DATA_0 */
    {GCI_CHIPCONTROL_REG_2, 12}, /* PIN_SDIO_DATA_1 */
    {GCI_CHIPCONTROL_REG_2, 16}, /* PIN_SDIO_DATA_2 */
    {GCI_CHIPCONTROL_REG_2, 20}, /* PIN_SDIO_DATA_3 */

    /* Direct connections */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT}, /* PIN_UART0_CTS */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT}, /* PIN_UART0_RTS */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT}, /* PIN_UART0_RXD */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT}, /* PIN_UART0_TXD */

    {GCI_CHIPCONTROL_REG_3, 0},  /* PIN_PWM_0 */
    {GCI_CHIPCONTROL_REG_3, 18}, /* PIN_PWM_1 */
    {GCI_CHIPCONTROL_REG_3, 20}, /* PIN_PWM_2 */
    {GCI_CHIPCONTROL_REG_3, 22}, /* PIN_PWM_3 */
    {GCI_CHIPCONTROL_REG_3, 24}, /* PIN_PWM_4 */
    {GCI_CHIPCONTROL_REG_5, 7},  /* PIN_PWM_5 */

    {GCI_CHIPCONTROL_REG_11, 27}, /* PIN_RF_SW_CTRL_5 */
    {GCI_CHIPCONTROL_REG_3, 10}, /* PIN_RF_SW_CTRL_6 */
    {GCI_CHIPCONTROL_REG_3, 12}, /* PIN_RF_SW_CTRL_7 */
    {GCI_CHIPCONTROL_REG_3, 14}, /* PIN_RF_SW_CTRL_8 */
    {GCI_CHIPCONTROL_REG_3, 16}, /* PIN_RF_SW_CTRL_9 */

    {GCI_CHIPCONTROL_REG_5, 9},  /* PIN_SPI_0_MISO */
    {GCI_CHIPCONTROL_REG_5, 11}, /* PIN_SPI_0_CLK */
    {GCI_CHIPCONTROL_REG_5, 13}, /* PIN_SPI_0_MOSI */
    {GCI_CHIPCONTROL_REG_9, 0},  /* PIN_SPI_0_CS */

    {GCI_CHIPCONTROL_REG_9, 2},  /* PIN_I2C0_SDATA */
    {GCI_CHIPCONTROL_REG_9, 4},  /* PIN_I2C0_CLK */

    {GCI_CHIPCONTROL_REG_9, 6},  /* PIN_I2S_MCLK0 */
    {GCI_CHIPCONTROL_REG_9, 8},  /* PIN_I2S_SCLK0 */
    {GCI_CHIPCONTROL_REG_9, 10}, /* PIN_I2S_LRCLK0 */
    {GCI_CHIPCONTROL_REG_9, 12}, /* PIN_I2S_SDATAI0 */
    {GCI_CHIPCONTROL_REG_9, 14}, /* PIN_I2S_SDATAO0 */
    {GCI_CHIPCONTROL_REG_9, 16}, /* PIN_I2S_SDATAO1 */
    {GCI_CHIPCONTROL_REG_9, 18}, /* PIN_I2S_SDATAI1 */
    {GCI_CHIPCONTROL_REG_9, 20}, /* PIN_I2S_MCLK1 */
    {GCI_CHIPCONTROL_REG_9, 22}, /* PIN_I2S_SCLK1 */
    {GCI_CHIPCONTROL_REG_9, 24}, /* PIN_I2S_LRCLK1 */

    /* Direct connections */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT},  /* PIN_SPI_1_MISO */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT},  /* PIN_SPI_1_CLK */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT},  /* PIN_SPI_1_MOSI */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT},  /* PIN_SPI_1_CS */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT},  /* PIN_I2C1_SDATA */
    {GCI_DIRECT_CONNECT, GCI_DIRECT_CONNECT},  /* PIN_I2C1_CLK */
};

/////////////////////////////////////////////// I2C ////////////////////////////////////////////////
/* Connections for: i2c_scl */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2c_scl[7] = {
    {0, 0, PIN_I2C0_CLK, PIN_MUX_SEL_1},
    {1, 0, PIN_I2C1_CLK, PIN_MUX_SEL_NA},
    {1, 0, PIN_GPIO_13, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_1, PIN_MUX_SEL_3},
    {1, 0, PIN_GPIO_11, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_7, PIN_MUX_SEL_6},
    {1, 0, PIN_GPIO_9, PIN_MUX_SEL_10},
};

/* Connections for: i2c_sda */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2c_sda[7] = {
    {0, 0, PIN_I2C0_SDATA, PIN_MUX_SEL_1},
    {1, 0, PIN_I2C1_SDATA, PIN_MUX_SEL_NA},
    {1, 0, PIN_GPIO_12, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_0, PIN_MUX_SEL_3},
    {1, 0, PIN_GPIO_10, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_16, PIN_MUX_SEL_6},
    {1, 0, PIN_GPIO_8, PIN_MUX_SEL_10},
};

/////////////////////////////////////////////// I2S ////////////////////////////////////////////////
/* Connections for: i2s_mclk */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_mclk[2] = {
    {0, 0, PIN_I2S_MCLK0, PIN_MUX_SEL_1},
    {1, 0, PIN_I2S_MCLK1, PIN_MUX_SEL_1},
};

/* Connections for: i2s_sclk */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_sclk[2] = {
    {0, 0, PIN_I2S_SCLK0, PIN_MUX_SEL_1},
    {1, 0, PIN_I2S_SCLK1, PIN_MUX_SEL_1},
};

/* Connections for: i2s_lrclk */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_lrclk[2] = {
    {0, 0, PIN_I2S_LRCLK0, PIN_MUX_SEL_1},
    {1, 0, PIN_I2S_LRCLK1, PIN_MUX_SEL_1},
};

/* Connections for: i2s_sdatai */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_sdatai[2] = {
    {0, 0, PIN_I2S_SDATAI0, PIN_MUX_SEL_1},
    {1, 0, PIN_I2S_SDATAI1, PIN_MUX_SEL_1},
};

/* Connections for: i2s_sdatao */
const cyhal_resource_pin_mapping_t cyhal_pin_map_i2s_sdatao[2] = {
    {0, 0, PIN_I2S_SDATAO0, PIN_MUX_SEL_1},
    {1, 0, PIN_I2S_SDATAO1, PIN_MUX_SEL_1},
};

/////////////////////////////////////////////// PWM ////////////////////////////////////////////////
/* Connections for: pwm_line */
const cyhal_resource_pin_mapping_t cyhal_pin_map_pwm_line[42] = {
    {0, 0, PIN_PWM_0, PIN_MUX_SEL_1},
    {1, 0, PIN_PWM_1, PIN_MUX_SEL_1},
    {2, 0, PIN_PWM_2, PIN_MUX_SEL_1},
    {3, 0, PIN_PWM_3, PIN_MUX_SEL_1},
    {4, 0, PIN_PWM_4, PIN_MUX_SEL_1},
    {5, 0, PIN_PWM_5, PIN_MUX_SEL_1},
    {0, 0, PIN_GPIO_14, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_15, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_7, PIN_MUX_SEL_3},
    {2, 0, PIN_GPIO_8, PIN_MUX_SEL_3},
    {3, 0, PIN_GPIO_9, PIN_MUX_SEL_3},
    {4, 0, PIN_GPIO_10, PIN_MUX_SEL_3},
    {5, 0, PIN_GPIO_11, PIN_MUX_SEL_3},
    {0, 0, PIN_GPIO_16, PIN_MUX_SEL_3},
    {0, 0, PIN_GPIO_0, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_1, PIN_MUX_SEL_4},
    {2, 0, PIN_GPIO_16, PIN_MUX_SEL_4},
    {3, 0, PIN_GPIO_7, PIN_MUX_SEL_4},
    {4, 0, PIN_GPIO_8, PIN_MUX_SEL_4},
    {5, 0, PIN_GPIO_9, PIN_MUX_SEL_4},
    {2, 0, PIN_GPIO_12, PIN_MUX_SEL_5},
    {3, 0, PIN_GPIO_13, PIN_MUX_SEL_5},
    {2, 0, PIN_GPIO_0, PIN_MUX_SEL_6},
    {3, 0, PIN_GPIO_1, PIN_MUX_SEL_6},
    {0, 0, PIN_GPIO_10, PIN_MUX_SEL_6},
    {1, 0, PIN_GPIO_11, PIN_MUX_SEL_6},
    {4, 0, PIN_GPIO_12, PIN_MUX_SEL_6},
    {5, 0, PIN_GPIO_13, PIN_MUX_SEL_6},
    {2, 0, PIN_GPIO_10, PIN_MUX_SEL_9},
    {3, 0, PIN_GPIO_11, PIN_MUX_SEL_9},
    {0, 0, PIN_GPIO_12, PIN_MUX_SEL_9},
    {1, 0, PIN_GPIO_13, PIN_MUX_SEL_9},
    {4, 0, PIN_GPIO_14, PIN_MUX_SEL_9},
    {5, 0, PIN_GPIO_15, PIN_MUX_SEL_9},
    {4, 0, PIN_GPIO_0, PIN_MUX_SEL_10},
    {5, 0, PIN_GPIO_1, PIN_MUX_SEL_10},
    {5, 0, PIN_GPIO_7, PIN_MUX_SEL_11},
    {0, 0, PIN_GPIO_8, PIN_MUX_SEL_11},
    {1, 0, PIN_GPIO_9, PIN_MUX_SEL_11},
    {2, 0, PIN_GPIO_14, PIN_MUX_SEL_11},
    {3, 0, PIN_GPIO_15, PIN_MUX_SEL_11},
    {4, 0, PIN_GPIO_16, PIN_MUX_SEL_11},
};

/////////////////////////////////////////////// SDIO ///////////////////////////////////////////////
/* Connections for: sdio_clk */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_clk[1] = {
    {0, 0, PIN_SDIO_CLK, PIN_MUX_SEL_1},
};

/* Connections for: sdio_cmd */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_cmd[1] = {
    {0, 0, PIN_SDIO_CMD, PIN_MUX_SEL_1},
};

/* Connections for: sdio_data0 */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data0[1] = {
    {0, 0, PIN_SDIO_DATA_0, PIN_MUX_SEL_1},
};

/* Connections for: sdio_data1 */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data1[1] = {
    {0, 0, PIN_SDIO_DATA_1, PIN_MUX_SEL_1},
};

/* Connections for: sdio_data2 */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data2[1] = {
    {0, 0, PIN_SDIO_DATA_2, PIN_MUX_SEL_1},
};

/* Connections for: sdio_data3 */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_data3[1] = {
    {0, 0, PIN_SDIO_DATA_3, PIN_MUX_SEL_1},
};

/* Connections for: sdio_sep_int */
const cyhal_resource_pin_mapping_t cyhal_pin_map_sdio_sep_int[2] = {
    {0, 0, PIN_GPIO_10, PIN_MUX_SEL_10},
    {0, 0, PIN_GPIO_12, PIN_MUX_SEL_11},
};

/////////////////////////////////////////////// SPI ////////////////////////////////////////////////
/* Connections for: spi_clk */
const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_clk[5] = {
    {0, 0, PIN_SPI_0_CLK, PIN_MUX_SEL_1},
    {1, 0, PIN_SPI_1_CLK, PIN_MUX_SEL_NA},
    {1, 0, PIN_GPIO_9, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_13, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_1, PIN_MUX_SEL_5},
};

/* Connections for: spi_miso */
const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_miso[5] = {
    {0, 0, PIN_SPI_0_MISO, PIN_MUX_SEL_1},
    {1, 0, PIN_SPI_1_MISO, PIN_MUX_SEL_NA},
    {1, 0, PIN_GPIO_8, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_12, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_0, PIN_MUX_SEL_5},
};

/* Connections for: spi_mosi */
const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_mosi[5] = {
    {0, 0, PIN_SPI_0_MOSI, PIN_MUX_SEL_1},
    {1, 0, PIN_SPI_1_MOSI, PIN_MUX_SEL_NA},
    {1, 0, PIN_GPIO_10, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_14, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_16, PIN_MUX_SEL_5},
};

/* Connections for: spi_cs */
const cyhal_resource_pin_mapping_t cyhal_pin_map_spi_cs[5] = {
    {0, 0, PIN_SPI_0_CS, PIN_MUX_SEL_1},
    {1, 0, PIN_SPI_1_CS, PIN_MUX_SEL_NA},
    {1, 0, PIN_GPIO_11, PIN_MUX_SEL_2},
    {1, 0, PIN_GPIO_15, PIN_MUX_SEL_4},
    {1, 0, PIN_GPIO_7, PIN_MUX_SEL_5},
};

/////////////////////////////////////////////// UART ///////////////////////////////////////////////
/* Connections for: uart_cts */
const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_cts[3] = {
    {0, 0, PIN_GPIO_16, PIN_MUX_SEL_2},
    {0, 0, PIN_GPIO_14, PIN_MUX_SEL_3},
    {0, 0, PIN_GPIO_10, PIN_MUX_SEL_5},
};

/* Connections for: uart_rts */
const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_rts[3] = {
    {0, 0, PIN_GPIO_7, PIN_MUX_SEL_2},
    {0, 0, PIN_GPIO_15, PIN_MUX_SEL_3},
    {0, 0, PIN_GPIO_11, PIN_MUX_SEL_5},
};

/* Connections for: uart_rx */
const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_rx[7] = {
    {0, 0, PIN_GPIO_0, PIN_MUX_SEL_2},
    {1, 0, PIN_RF_SW_CTRL_6, PIN_MUX_SEL_2},
    {2, 0, PIN_RF_SW_CTRL_6, PIN_MUX_SEL_3},
    {0, 0, PIN_GPIO_12, PIN_MUX_SEL_3},
    {1, 0, PIN_RF_SW_CTRL_8, PIN_MUX_SEL_3},
    {2, 0, PIN_RF_SW_CTRL_8, PIN_MUX_SEL_2},
    {0, 0, PIN_GPIO_8, PIN_MUX_SEL_5},
};

/* Connections for: uart_tx */
const cyhal_resource_pin_mapping_t cyhal_pin_map_uart_tx[7] = {
    {0, 0, PIN_GPIO_1, PIN_MUX_SEL_2},
    {1, 0, PIN_RF_SW_CTRL_7, PIN_MUX_SEL_2},
    {2, 0, PIN_RF_SW_CTRL_7, PIN_MUX_SEL_3},
    {0, 0, PIN_GPIO_13, PIN_MUX_SEL_3},
    {1, 0, PIN_RF_SW_CTRL_9, PIN_MUX_SEL_3},
    {2, 0, PIN_RF_SW_CTRL_9, PIN_MUX_SEL_2},
    {0, 0, PIN_GPIO_9, PIN_MUX_SEL_5},
};

/////////////////////////////////////////////// USB ////////////////////////////////////////////////
/* Connections for: usb_ctl1 */
const cyhal_resource_pin_mapping_t cyhal_pin_map_usbpd_afc_tx_data[1] = {
    {0, 0, PIN_GPIO_0, PIN_MUX_SEL_11},
};

/* Connections for: usb_ctl2 */
const cyhal_resource_pin_mapping_t cyhal_pin_map_usbpd_afc_tx_data_en[1] = {
    {0, 0, PIN_GPIO_16, PIN_MUX_SEL_10},
};
