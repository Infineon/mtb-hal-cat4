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
 * Misc Broadcom BCM47XX MDC/MDIO enet phy definitions.
 *
 */

#pragma once

/* phy address */
#define MAXEPHY               (32)              /* mdio phy addresses are 5bit quantities */
#define EPHY_MASK           (0x1f)              /* phy mask */
#define EPHY_NONE             (31)              /* nvram: no phy present at all */
#define EPHY_NOREG            (30)              /* nvram: no local phy regs */

#define MAXPHYREG             (32)              /* max 32 registers per phy */

/* just a few phy registers */
/* register 0x00 - PHY Control */
#define EPHY_CTL_RESET        (1 << 15)              /* reset */
#define EPHY_CTL_LOOP         (1 << 14)              /* loopback */
#define EPHY_CTL_SPEED        (1 << 13)              /* speed selection lsb 0=10, 1=100 */
#define EPHY_CTL_ANENAB       (1 << 12)              /* autonegotiation enable */
#define EPHY_CTL_RESTART      (1 <<  9)              /* restart autonegotiation */
#define EPHY_CTL_DUPLEX       (1 <<  8)              /* duplex mode 0=half, 1=full */
#define EPHY_CTL_SPEED_MSB    (1 <<  6)              /* speed selection msb */
#define EPHY_CTL_SPEED_10     ((0 << 6) | (0 << 13)) /* speed selection CTL.6=0, CTL.13=0 */
#define EPHY_CTL_SPEED_100    ((0 << 6) | (1 << 13)) /* speed selection CTL.6=0, CTL.13=1 */
#define EPHY_CTL_SPEED_1000   ((1 << 6) | (0 << 13)) /* speed selection CTL.6=1, CTL.13=0 */

/* register 0x01 - PHY Status */
#define EPHY_STAT_REMFAULT       (1 << 4)            /* remote fault */
#define EPHY_STAT_LINK           (1 << 2)            /* link status */
#define EPHY_STAT_JAB            (1 << 1)            /* jabber detected */

/* register 0x02
 * PHY Identifier Register #1 - Culled from another PHY device -- need to validate
 * bits 15:0 - MSB (bits 18:3 of the OUI)
 */
#define EPHY_ID_MSB              (0)

/* register 0x03
 * PHY Identifier Register #2 - Culled from another PHY device -- need to validate
 */
#define EPHY_ID_LSB             (10)    /* bits 15:10 are the LSB (bits 24:19 of OUI) */
#define EPHY_ID_VENDOR_MODEL     (4)    /* bits   9:4  are the Vendor Model # */
#define EPHY_ID_MODEL_REVISION   (0)    /* bits   3:0 are the Model revision */


/* register 0x04 - PHY Speed Advertisement */
#define EPHY_ADV_10FULL       (1 << 6)               /* autonegotiate advertise 10full */
#define EPHY_ADV_10HALF       (1 << 5)               /* autonegotiate advertise 10half */
#define EPHY_ADV_100FULL      (1 << 8)               /* autonegotiate advertise 100full */
#define EPHY_ADV_100HALF      (1 << 7)               /* autonegotiate advertise 100half */

/* link partner ability register */
/* register 0x05 - PHY Link Partner Abilities */
#define EPHY_LPA_SLCT            (0x001f)            /* same as advertise selector */
#define EPHY_LPA_10HALF          (0x0020)            /* can do 10mbps half-duplex */
#define EPHY_LPA_10FULL          (0x0040)            /* can do 10mbps full-duplex */
#define EPHY_LPA_100HALF         (0x0080)            /* can do 100mbps half-duplex */
#define EPHY_LPA_100FULL         (0x0100)            /* can do 100mbps full-duplex */
#define EPHY_LPA_100BASE4        (0x0200)            /* can do 100mbps 4k packets */
#define EPHY_LPA_RESV            (0x1c00)            /* unused */
#define EPHY_LPA_RFAULT          (0x2000)            /* link partner faulted */
#define EPHY_LPA_LPACK           (0x4000)            /* link partner acked us */
#define EPHY_LPA_NPAGE           (0x8000)            /* next page bit */

#define EPHY_LPA_DUPLEX          (EPHY_LPA_10FULL | EPHY_LPA_100FULL)
#define EPHY_LPA_100             (EPHY_LPA_100FULL | EPHY_LPA_100HALF | EPHY_LPA_100BASE4)

/* 1000BASE-T control register */
/* register 0x09 - PHY Speed Advertisement 2 */
#define EPHY_ADV_1000HALF        (0x0100)            /* advertise 1000BASE-T half duplex */
#define EPHY_ADV_1000FULL        (0x0200)            /* advertise 1000BASE-T full duplex */

/* 1000BASE-T status register */
/* register 0x0a - PHY Link Partner Abilities */
#define EPHY_LPA_1000HALF        (0x0400)            /* link partner 1000BASE-T half duplex */
#define EPHY_LPA_1000FULL        (0x0800)            /* link partner 1000BASE-T full duplex */

/* 1000BASE-T extended status register */
/* register 0x0f - PHY Extended Status */
#define EPHY_EST_1000THALF       (0x1000)            /* 1000BASE-T half duplex capable */
#define EPHY_EST_1000TFULL       (0x2000)            /* 1000BASE-T full duplex capable */
#define EPHY_EST_1000XHALF       (0x4000)            /* 1000BASE-X half duplex capable */
#define EPHY_EST_1000XFULL       (0x8000)            /* 1000BASE-X full duplex capable */

