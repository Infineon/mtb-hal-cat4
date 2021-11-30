/*******************************************************************************
* File Name: cyhal_uart_registers.h
*
* Description:
* Provides definitions for the internals of the 43907 UART registers
*
********************************************************************************
* \copyright
* Copyright 2018-2021 Cypress Semiconductor Corporation (an Infineon company) or
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
#pragma once

#include "platform_appscr4.h"

/** Represents the register layout of the slow (debug) UART */
typedef struct
{
    /** uart0data register */
    volatile uint8_t rx_tx_dll;
    /** uart0ier register */
    volatile uint8_t ier_dlm;
    /** uart0fcr register */
    volatile uint8_t iir_fcr;
    /** uart0lcr register */
    volatile uint8_t lcr;
    /** uart0mcr register */
    volatile uint8_t mcr;
    /** uart0lsr register */
    volatile uint8_t lsr;
    /** uart0msr register */
    volatile uint8_t msr;
    /** uart0scr register */
    volatile uint8_t scr;
} _cyhal_uart_slow_t;

/** Represents the common aspects of the register layout of SECI UARTs (fast and GCI) */
typedef struct
{
    /** SeciUARTData register */
    volatile uint32_t data;
    /** SeciUARTBaudRateDivisor register */
    volatile uint32_t bauddiv;
    /** SeciUARTFCR register */
    volatile uint32_t fcr;
    /** SeciUARTLCR register */
    volatile uint32_t lcr;
    /** SeciUARTMCR register */
    volatile uint32_t mcr;
    /** SeciUARTLSR register */
    volatile uint32_t lsr;
    /** SeciUARTMSR register */
    volatile uint32_t msr;
    /** SeciUARTBaudRateAdjustment register */
    volatile uint32_t baudadj;
} _cyhal_uart_seci_t;

/* Some of the bits in slow UART status and control registers */
#define UART_SLOW_LSR_TEMT   (0x40)    /* Data-hold-register empty */
#define UART_SLOW_LSR_THRE   (0x20)    /* Transmit-hold-register empty */
#define UART_SLOW_LSR_RXRDY  (0x01)    /* Receiver ready */
#define UART_SLOW_LCR_DLAB   (0x80)    /* Divisor latch access bit */
#define UART_SLOW_LCR_WLEN8  (0x03)    /* Word length: 8 bits */

/* Some of the bits in slow UART Interrupt Enable Register */
#define UART_SLOW_IER_PTIME  (0x80)     /* Programmable THRE Interrupt Mode Enable */
#define UART_SLOW_IER_THRE   (0x02)     /* Transmit-hold-register empty */
#define UART_SLOW_IER_RXRDY  (0x01)     /* Receiver ready */

/* Some of the bits in slow UART Modem Control Register */
#define UART_SLOW_MCR_OUT2          (0x08)

/* Define maximum attempts to clean pending data kept from before driver initialized */
#define UART_SLOW_CLEAN_ATTEMPTS    (64)

/* FIFO Enable bit in slow UART FCR Register */
#define UART_SLOW_FCR_FIFO_ENABLE   (0x01)
#define UART_SLOW_FCR_RCVR_MASK     (0xC0)

/* FIFO trigger level fields */
#define UART_SLOW_FCR_RCVR_MASK     (0xC0)
#define UART_SLOW_FCR_TET_MASK      (0x30)

/* RX trigger levels, pre-shifted */
#define UART_SLOW_FCR_RCVR_1_VAL      (0x00)
#define UART_SLOW_FCR_RCVR_1_4_VAL    (0x40)
#define UART_SLOW_FCR_RCVR_1_2_VAL    (0x80)
#define UART_SLOW_FCR_RCVR_2_LEFT_VAL (0xC0)

/* TX trigger levels, pre-shifted */
#define UART_SLOW_FCR_TET_EMPTY_VAL   (0x00)
#define UART_SLOW_FCR_TET_2_VAL       (0x10)
#define UART_SLOW_FCR_TET_1_4_VAL     (0x20)
#define UART_SLOW_FCR_TET_1_2_VAL     (0x30)

/* Parity bits in slow UART LCR register */
#define UART_EPS_BIT    (4)
#define UART_PEN_BIT    (3)
#define UART_STOP_BIT    (3)

#define UART_SLOW_REGBASE               (PLATFORM_CHIPCOMMON_REGBASE(0x300))
#define UART_FAST_REGBASE               (PLATFORM_CHIPCOMMON_REGBASE(0x1C0))
#define UART_GCI_REGBASE                (PLATFORM_GCI_REGBASE(0x1DC))

/* The below register defines can be moved to structure-pointer format after
 * ChipCommon and GCI structure layouts are fully defined in platform_appscr4.h */
#define CHIPCOMMON_CORE_CAP_REG         *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x004)))
#define CHIPCOMMON_CORE_CTRL_REG        *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x008)))
#define CHIPCOMMON_INT_STATUS_REG       *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x020)))
#define CHIPCOMMON_INT_MASK_REG         *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x024)))
#define CHIPCOMMON_CORE_CLK_DIV         *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x0A4)))
#define CHIPCOMMON_CORE_CAP_EXT_REG     *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x0AC)))
#define CHIPCOMMON_SECI_CONFIG_REG      *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x130)))
#define CHIPCOMMON_SECI_STATUS_REG      *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x134)))
#define CHIPCOMMON_SECI_STATUS_MASK_REG *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x138)))
#define CHIPCOMMON_SECI_FIFO_LEVEL_REG  *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x18C)))
#define CHIPCOMMON_SECI_UART_FCR_REG    *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x1C8)))
#define CHIPCOMMON_SECI_UART_LCR_REG    *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x1CC)))
#define CHIPCOMMON_SECI_UART_MCR_REG    *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x1D0)))
#define CHIPCOMMON_CLK_CTL_STATUS_REG   *((volatile uint32_t*)(PLATFORM_CHIPCOMMON_REGBASE(0x1E0)))

#define GCI_CORE_CTRL_REG                *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x00C)))
#define GCI_CORE_STATUS_REG              *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x010)))
#define GCI_INT_STATUS_REG               *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x014)))
#define GCI_INT_MASK_REG                 *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x018)))
#define GCI_INDIRECT_ADDRESS_REG         *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x040)))
#define GCI_GPIO_CTRL_REG                *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x044)))
#define GCI_GCI_RX_FIFO_PER_IP_CTRL_REG  *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x1C8)))
#define GCI_SECI_FIFO_LEVEL_REG          *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x1D8)))
#define GCI_SECI_IN_CTRL_REG             *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x218)))
#define GCI_SECI_OUT_CTRL_REG            *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x21C)))
#define GCI_SECI_IN_AUX_FIFO_RX_ENAB_REG *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x220)))
#define GCI_SECI_OUT_TX_ENAB_TX_BRK_REG  *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x224)))
#define GCI_SECI_OUT_TX_STATUS           *((volatile uint32_t*)(PLATFORM_GCI_REGBASE(0x234)))

/* ChipCommon Capabilities Extension */
#define CC_CAP_EXT_SECI_PRESENT                  (0x00000001)    /* SECI present */
#define CC_CAP_EXT_SECI_PLAIN_UART_PRESENT       (0x00000008)    /* SECI Plain UART present */

/* ChipCommon ClockCtlStatus bits */
#define CC_CLK_CTL_ST_SECI_CLK_REQ                  (1 << 8)
#define CC_CLK_CTL_ST_SECI_CLK_AVAIL                (1 << 24)
#define CC_CLK_CTL_ST_BACKPLANE_ALP                 (1 << 18)
#define CC_CLK_CTL_ST_BACKPLANE_HT                  (1 << 19)

/* ChipCommon Slow UART IntStatus and IntMask register bit */
#define UART_SLOW_CC_INT_STATUS_MASK                (1 << 6)

/* ChipCommon FAST UART IntStatus and IntMask register bit */
#define UART_FAST_CC_INT_STATUS_MASK                (1 << 4)

/* ChipCommon GCI UART IntStatus and IntMask register bit */
#define UART_GCI_CC_INT_STATUS_MASK                 (1 << 4)

/* ChipCommon SECI Config register bits */
#define CC_SECI_CONFIG_HT_CLOCK                     (1 << 13)

/* ChipCommon SECI Status register bits */
#define CC_SECI_STATUS_RX_IDLE_TIMER_INT            (1 << 8)
#define CC_SECI_STATUS_TX_FIFO_FULL                 (1 << 9)
#define CC_SECI_STATUS_TX_FIFO_ALMOST_EMPTY         (1 << 10)
#define CC_SECI_STATUS_RX_FIFO_EMPTY                (1 << 11)
#define CC_SECI_STATUS_RX_FIFO_ALMOST_FULL          (1 << 12)

/* GCI CoreCtrl register bits */
#define GCI_CORE_CTRL_SECI_RESET                    (1 << 0)
#define GCI_CORE_CTRL_RESET_SECI_LOGIC              (1 << 1)
#define GCI_CORE_CTRL_ENABLE_SECI                   (1 << 2)
#define GCI_CORE_CTRL_FORCE_SECI_OUT_LOW            (1 << 3)
#define GCI_CORE_CTRL_UPDATE_SECI                   (1 << 7)
#define GCI_CORE_CTRL_BREAK_ON_SLEEP                (1 << 8)
#define GCI_CORE_CTRL_SECI_IN_LOW_TIMEOUT_SHIFT     (9)
#define GCI_CORE_CTRL_SECI_IN_LOW_TIMEOUT_MASK      (0x3)
#define GCI_CORE_CTRL_RESET_OFF_CHIP_COEX           (1 << 11)
#define GCI_CORE_CTRL_AUTO_BT_SIG_RESEND            (1 << 12)
#define GCI_CORE_CTRL_FORCE_GCI_CLK_REQ             (1 << 16)
#define GCI_CORE_CTRL_FORCE_HW_CLK_REQ_OFF          (1 << 17)
#define GCI_CORE_CTRL_FORCE_REG_CLK                 (1 << 18)
#define GCI_CORE_CTRL_FORCE_SECI_CLK                (1 << 19)
#define GCI_CORE_CTRL_FORCE_GCI_CLK_AVAIL           (1 << 20)
#define GCI_CORE_CTRL_FORCE_GCI_CLK_AVAIL_VALUE     (1 << 21)
#define GCI_CORE_CTRL_SECI_CLK_STRETCH_SHIFT        (24)
#define GCI_CORE_CTRL_SECI_CLK_STRETCH_MASK         (0xFF)

/* GCI CoreStatus register bits */
#define GCI_CORE_STATUS_BREAK_IN                    (1 << 0)
#define GCI_CORE_STATUS_BREAK_OUT                   (1 << 1)
#define GCI_CORE_STATUS_GCI_CLK_AVAIL               (1 << 16)
#define GCI_CORE_STATUS_GCI_CLK_AVAIL_PRE           (1 << 17)

/* GCI IntStatus and IntMask register bits */
#define GCI_INT_ST_MASK_RX_BREAK_EVENT_INT          (1 << 0)
#define GCI_INT_ST_MASK_UART_BREAK_INT              (1 << 1)
#define GCI_INT_ST_MASK_SECI_PARITY_ERROR           (1 << 2)
#define GCI_INT_ST_MASK_SECI_FRAMING_ERROR          (1 << 3)
#define GCI_INT_ST_MASK_SECI_DATA_UPDATED           (1 << 4)
#define GCI_INT_ST_MASK_SECI_AUX_DATA_UPDATED       (1 << 5)
#define GCI_INT_ST_MASK_SECI_TX_UPDATED_DONE        (1 << 6)
#define GCI_INT_ST_MASK_SECI_RX_IDLE_TIMER_INT      (1 << 9)
#define GCI_INT_ST_MASK_SECI_TX_FIFO_FULL           (1 << 10)
#define GCI_INT_ST_MASK_SECI_TX_FIFO_ALMOST_EMPTY   (1 << 11)
#define GCI_INT_ST_MASK_SECI_RX_FIFO_ALMOST_FULL    (1 << 12)
#define GCI_INT_ST_MASK_SECI_FLOW_CONTROL_EVENT     (1 << 13)
#define GCI_INT_ST_MASK_SECI_RX_FIFO_NOT_EMPTY      (1 << 14)
#define GCI_INT_ST_MASK_SECI_RX_FIFO_OVERFLOW       (1 << 15)
#define GCI_INT_ST_MASK_GCI_LEVEL_INT               (1 << 20)
#define GCI_INT_ST_MASK_GCI_EVENT_INT               (1 << 21)
#define GCI_INT_ST_MASK_GCI_WAKE_LEVEL_INT          (1 << 22)
#define GCI_INT_ST_MASK_GCI_WAKE_EVENT_INT          (1 << 23)
#define GCI_INT_ST_MASK_SEMAPHORE_INT               (1 << 24)
#define GCI_INT_ST_MASK_GCI_GPIO_INT                (1 << 25)
#define GCI_INT_ST_MASK_GCI_GPIO_WAKE               (1 << 26)
#define GCI_INT_ST_MASK_BATTERY_INT                 (1 << 27)

/* GCI GPIO Control register bits */
#define GCI_GPIO_CTRL_BITS_PER_GPIO                     (8)
#define GCI_GPIO_CTRL_REG_INDEX(gpio_num)               ((gpio_num * GCI_GPIO_CTRL_BITS_PER_GPIO) / 32)
#define GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num)             ((gpio_num * GCI_GPIO_CTRL_BITS_PER_GPIO) % 32)
#define GCI_GPIO_CTRL_GPIO_MASK(gpio_num)               (0xFF << GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num))
#define GCI_GPIO_CTRL_INPUT_ENAB(gpio_num)              (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 0))
#define GCI_GPIO_CTRL_OUTPUT_ENAB(gpio_num)             (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 1))
#define GCI_GPIO_CTRL_INVERT(gpio_num)                  (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 2))
#define GCI_GPIO_CTRL_PUP(gpio_num)                     (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 3))
#define GCI_GPIO_CTRL_PDN(gpio_num)                     (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 4))
#define GCI_GPIO_CTRL_BT_SIG_ENAB(gpio_num)             (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 5))
#define GCI_GPIO_CTRL_OPEN_DRAIN_OUTPUT_ENAB(gpio_num)  (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 6))
#define GCI_GPIO_CTRL_EXTRA_GPIO_ENAB(gpio_num)         (1 << (GCI_GPIO_CTRL_GPIO_OFFSET(gpio_num) + 7))

/* GCI TX Status register bits */
#define GCI_TX_STATUS_TX_IDLE                    (3u) /* NumOfSeciOut = 2 */

/* Default maximum wait times in ms for Slow UART TX and RX */
#define UART_SLOW_MAX_TRANSMIT_WAIT_TIME         (10)
#define UART_SLOW_MAX_READ_WAIT_TIME             (200)
#define UART_SLOW_CLKDIV_MASK                    (0x000000FF)

/* Default maximum wait times in ms for Fast UART TX and RX */
#define UART_FAST_MAX_TRANSMIT_WAIT_TIME         (10)
#define UART_FAST_MAX_READ_WAIT_TIME             (200)

/* Default maximum wait times in ms for GCI UART TX and RX */
#define UART_GCI_MAX_TRANSMIT_WAIT_TIME          (10)
#define UART_GCI_MAX_READ_WAIT_TIME              (200)

/* SECI configuration */
#define SECI_MODE_UART                           (0x0)
#define SECI_MODE_SECI                           (0x1)
#define SECI_MODE_LEGACY_3WIRE_BT                (0x2)
#define SECI_MODE_LEGACY_3WIRE_WLAN              (0x3)
#define SECI_MODE_HALF_SECI                      (0x4)

#define CC_SECI_RESET                            (1 << 0)
#define CC_SECI_RESET_BAR_UART                   (1 << 1)
#define CC_SECI_ENAB_SECI_ECI                    (1 << 2)
#define CC_SECI_ENAB_SECIOUT_DIS                 (1 << 3)
#define CC_SECI_MODE_MASK                        (0x7)
#define CC_SECI_MODE_SHIFT                       (4)
#define CC_SECI_UPD_SECI                         (1 << 7)

#define SECI_SLIP_ESC_CHAR                       (0xDB)
#define SECI_SIGNOFF_0                           SECI_SLIP_ESC_CHAR
#define SECI_SIGNOFF_1                           (0)
#define SECI_REFRESH_REQ                         (0xDA)

#define UART_SECI_MSR_CTS_STATE                  (1 << 0)
#define UART_SECI_MSR_RTS_STATE                  (1 << 1)
#define UART_SECI_SECI_IN_STATE                  (1 << 2)
#define UART_SECI_SECI_IN2_STATE                 (1 << 3)

/* ChipCommon SECI FIFO Level Register Offsets */
#define CC_SECI_RX_FIFO_LVL_MASK                 (0x3F)
#define CC_SECI_RX_FIFO_LVL_SHIFT                (0)
#define CC_SECI_TX_FIFO_LVL_MASK                 (0x3F00)
#define CC_SECI_TX_FIFO_LVL_SHIFT                (8)
#define CC_SECI_RX_FIFO_LVL_FLOW_CTL_MASK        (0x3F)
#define CC_SECI_RX_FIFO_LVL_FLOW_CTL_SHIFT       (16)

/* GCI SECI RX FIFO Level Register Offsets */
#define GCI_SECI_RX_FIFO_LVL_MASK                (0x3F)
#define GCI_SECI_RX_FIFO_LVL_SHIFT               (0)

/* GCI SECI TX FIFO Level Register Offsets */
#define GCI_SECI_TX_FIFO_LVL_MASK                (0x3F00)
#define GCI_SECI_TX_FIFO_LVL_SHIFT               (8)

/* GCI UART SECI_IN port operating mode */
#define GCI_SECI_IN_OP_MODE_MASK                 (0x7)
#define GCI_SECI_IN_OP_MODE_SHIFT                (0)

/* GCI UART SECI_OUT port operating mode */
#define GCI_SECI_OUT_OP_MODE_MASK                (0x7)
#define GCI_SECI_OUT_OP_MODE_SHIFT               (0)

/* GCI UART SECI_IN GCI_GPIO mapping */
#define GCI_SECI_IN_GPIO_NUM_MASK                (0xF)
#define GCI_SECI_IN_GPIO_NUM_SHIFT               (4)

/* GCI UART SECI_OUT GCI_GPIO mapping */
#define GCI_SECI_OUT_GPIO_NUM_MASK               (0xF)
#define GCI_SECI_OUT_GPIO_NUM_SHIFT              (4)

/* Default GCI_GPIO mappings for SECI_IN and SECI_OUT */
#define GCI_SECI_IN_GPIO_NUM_DEFAULT             (6)
#define GCI_SECI_OUT_GPIO_NUM_DEFAULT            (7)

/* GCI UART SECI RX FIFO Enable bit */
#define GCI_SECI_FIFO_RX_ENAB                    (1 << 16)

/* GCI UART SECI TX Enable bit */
#define GCI_SECI_TX_ENAB                         (1 << 0)

/* Default FIFO levels for ChipCommon SECI host flow control */
#define CC_SECI_RX_FIFO_LVL_FLOW_CTL_DEFAULT        (0x20)

/* Default FIFO levels for GCI SECI RX and TX */
#define GCI_SECI_RX_FIFO_LVL_DEFAULT                (0x8)
#define GCI_SECI_TX_FIFO_LVL_DEFAULT                (0x8)

/* SECI UART FCR bit definitions */
#define UART_SECI_FCR_RFR                        (1 << 0)
#define UART_SECI_FCR_TFR                        (1 << 1)
#define UART_SECI_FCR_SR                         (1 << 2)
#define UART_SECI_FCR_THP                        (1 << 3)
#define UART_SECI_FCR_AB                         (1 << 4)
#define UART_SECI_FCR_ATOE                       (1 << 5)
#define UART_SECI_FCR_ARTSOE                     (1 << 6)
#define UART_SECI_FCR_ABV                        (1 << 7)
#define UART_SECI_FCR_ALM                        (1 << 8)

/* SECI UART LCR bit definitions */
#define UART_SECI_LCR_STOP_BITS                  (1 << 0) /* 0 - 1bit, 1 - 2bits */
#define UART_SECI_LCR_PARITY_EN                  (1 << 1)
#define UART_SECI_LCR_PARITY                     (1 << 2) /* 0 - odd, 1 - even */
#define UART_SECI_LCR_RX_EN                      (1 << 3)
#define UART_SECI_LCR_LBRK_CTRL                  (1 << 4) /* 1 => SECI_OUT held low */
#define UART_SECI_LCR_TXO_EN                     (1 << 5)
#define UART_SECI_LCR_RTSO_EN                    (1 << 6)
#define UART_SECI_LCR_SLIPMODE_EN                (1 << 7)
#define UART_SECI_LCR_RXCRC_CHK                  (1 << 8)
#define UART_SECI_LCR_TXCRC_INV                  (1 << 9)
#define UART_SECI_LCR_TXCRC_LSBF                 (1 << 10)
#define UART_SECI_LCR_TXCRC_EN                   (1 << 11)
#define UART_SECI_LCR_RXSYNC_EN                  (1 << 12)

/* SECI UART MCR bit definitions */
#define UART_SECI_MCR_TX_EN                      (1 << 0)
#define UART_SECI_MCR_PRTS                       (1 << 1)
#define UART_SECI_MCR_SWFLCTRL_EN                (1 << 2)
#define UART_SECI_MCR_HIGHRATE_EN                (1 << 3)
#define UART_SECI_MCR_LOOPBK_EN                  (1 << 4)
#define UART_SECI_MCR_AUTO_RTS                   (1 << 5)
#define UART_SECI_MCR_AUTO_TX_DIS                (1 << 6)
#define UART_SECI_MCR_BAUD_ADJ_EN                (1 << 7)
#define UART_SECI_MCR_XONOFF_RPT                 (1 << 9)

/* SECI UART LSR bit definitions */
#define UART_SECI_LSR_RXOVR_MASK                 (1 << 0)
#define UART_SECI_LSR_RFF_MASK                   (1 << 1)
#define UART_SECI_LSR_TFNE_MASK                  (1 << 2)
#define UART_SECI_LSR_TI_MASK                    (1 << 3)
#define UART_SECI_LSR_TPR_MASK                   (1 << 4)
#define UART_SECI_LSR_TXHALT_MASK                (1 << 5)

/* SECI UART MSR bit definitions */
#define UART_SECI_MSR_CTSS_MASK                  (1 << 0)
#define UART_SECI_MSR_RTSS_MASK                  (1 << 1)
#define UART_SECI_MSR_SIS_MASK                   (1 << 2)
#define UART_SECI_MSR_SIS2_MASK                  (1 << 3)

/* SECI UART Data bit definitions */
#define UART_SECI_DATA_RF_NOT_EMPTY_BIT          (1 << 12)
#define UART_SECI_DATA_RF_FULL_BIT               (1 << 13)
#define UART_SECI_DATA_RF_OVRFLOW_BIT            (1 << 14)
#define UART_SECI_DATA_FIFO_PTR_MASK             (0xFF)
#define UART_SECI_DATA_RF_RD_PTR_SHIFT           (16)
#define UART_SECI_DATA_RF_WR_PTR_SHIFT           (24)

/* Range for High rate SeciUARTBaudRateDivisor is 0xF1 - 0xF8 */
#define UART_SECI_HIGH_RATE_THRESHOLD_LOW        (0xF1)
#define UART_SECI_HIGH_RATE_THRESHOLD_HIGH       (0xF8)

#define UART_SECI_BAUD_RATE_THRESHOLD_LOW        (9600)
#define UART_SECI_ALP_CLOCK_DEFAULT              (37400000)
#define UART_SECI_BACKPLANE_CLOCK_DEFAULT        (160000000)
#define UART_SECI_BAUD_RATE_DIVISOR_MAX          (255)
#define UART_SECI_BAUD_RATE_DIVISOR_RANGE        (UART_SECI_BAUD_RATE_DIVISOR_MAX + 1)
#define UART_SECI_BAUD_RATE_ADJUSTMENT_MAX       (15)
#define UART_SECI_BAUD_RATE_ADJUSTMENT_RANGE     (UART_SECI_BAUD_RATE_ADJUSTMENT_MAX + 1)

#define _CYHAL_UART_SLOW_BASE (( _cyhal_uart_slow_t* )( UART_SLOW_REGBASE ))
#define _CYHAL_UART_FAST_BASE (( _cyhal_uart_seci_t* )( UART_FAST_REGBASE ))
#define _CYHAL_UART_GCI_BASE  (( _cyhal_uart_seci_t* )( UART_GCI_REGBASE ))
