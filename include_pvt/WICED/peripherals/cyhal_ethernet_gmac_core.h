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
 * gmacdefs - Broadcom gmac (Unimac) specific definitions
 *
 * $Id: _cyhal_gmac_core.h 376342 2012-12-24 21:02:49Z palter $
 */

#pragma once


/* cpp contortions to concatenate w/arg prescan */
#ifndef PAD
#define _PADLINE(line)    pad ## line
#define _XSTR(line)    _PADLINE(line)
#define PAD        _XSTR(__LINE__)
#endif    /* PAD */

/* We have 4 DMA TX channels */
#define GMAC_NUM_DMA_TX             4

typedef volatile struct {
    dma64regs_t    dmaxmt;          /* dma tx */
    uint32 PAD[2];
    dma64regs_t    dmarcv;          /* dma rx */
    uint32 PAD[2];
} dma64_t;

/*
 * Host Interface Registers
 *
Place this struct address to the start of the registers and the offsets are correctly matched with the registers.
 */
typedef volatile struct _gmacregs {
    uint32    devcontrol;               /* 0x000 */
    uint32    devstatus;                /* 0x004 */
    uint32    PAD;
    uint32    biststatus;               /* 0x00c */
    uint32    PAD[4];
    uint32    intstatus;                /* 0x020 */
    uint32    intmask;                  /* 0x024 */
    uint32    gptimer;                  /* 0x028 */
    uint32    PAD[53];
    uint32    intrecvlazy;              /* 0x100 */
    uint32    flowctlthresh;            /* 0x104 */
    uint32    wrrthresh;                /* 0x108 */
    uint32    _cyhal_gmac_idle_cnt_thresh;     /* 0x10c */
    uint32    PAD[28];
    uint32    phyaccess;                /* 0x180 */
    uint32    PAD;
    uint32    phycontrol;               /* 0x188 */
    uint32    txqctl;                   /* 0x18c */
    uint32    rxqctl;                   /* 0x190 */
    uint32    gpioselect;               /* 0x194 */
    uint32    gpio_output_en;           /* 0x198 */
    uint32    PAD[17];
    uint32    clk_ctl_st;               /* 0x1e0 */
    uint32    hw_war;                   /* 0x1e4 */
    uint32    pwrctl;                   /* 0x1e8 */
    uint32    PAD[5];

    dma64_t dmaregs[GMAC_NUM_DMA_TX];

    /* GAMC MIB counters */
    gmacmib_t    mib;
    uint32    PAD[245];

    uint32    unimacversion;            /* 0x800 */
    uint32    hdbkpctl;                 /* 0x804 */
    uint32    cmdcfg;                   /* 0x808 */
    uint32    macaddrhigh;              /* 0x80c */
    uint32    macaddrlow;               /* 0x810 */
    uint32    rxmaxlength;              /* 0x814 */
    uint32    pausequanta;              /* 0x818 */
    uint32    PAD[10];
    uint32    macmode;                  /* 0x844 */
    uint32    outertag;                 /* 0x848 */
    uint32    innertag;                 /* 0x84c */
    uint32    PAD[3];
    uint32    txipg;                    /* 0x85c */
    uint32    PAD[180];
    uint32    pausectl;                 /* 0xb30 */
    uint32    txflush;                  /* 0xb34 */
    uint32    rxstatus;                 /* 0xb38 */
    uint32    txstatus;                 /* 0xb3c */
} gmacregs_t;

#define GM_MIB_BASE                  0x300
#define GM_MIB_LIMIT                 0x800

/*
 * register-specific flag definitions
 */

/* device control */
#define DC_TSM                  0x00000002
#define DC_CFCO                 0x00000004
#define DC_RLSS                 0x00000008
#define DC_MROR                 0x00000010
#define DC_FCM_MASK             0x00000060
#define DC_FCM_SHIFT                     5
#define DC_NAE                  0x00000080
#define DC_TF                   0x00000100
#define DC_RDS_MASK             0x00030000
#define DC_RDS_SHIFT                    16
#define DC_TDS_MASK             0x000c0000
#define DC_TDS_SHIFT                    18

/* device status */
#define DS_RBF                  0x00000001
#define DS_RDF                  0x00000002
#define DS_RIF                  0x00000004
#define DS_TBF                  0x00000008
#define DS_TDF                  0x00000010
#define DS_TIF                  0x00000020
#define DS_PO                   0x00000040
#define DS_MM_MASK              0x00000300
#define DS_MM_SHIFT                      8

/* bist status */
#define BS_MTF                  0x00000001
#define BS_MRF                  0x00000002
#define BS_TDB                  0x00000004
#define BS_TIB                  0x00000008
#define BS_TBF                  0x00000010
#define BS_RDB                  0x00000020
#define BS_RIB                  0x00000040
#define BS_RBF                  0x00000080
#define BS_URTF                 0x00000100
#define BS_UTF                  0x00000200
#define BS_URF                  0x00000400

/* interrupt status and mask registers  GMAC IntStatus 0x18005020*/
#define I_MRO                   0x00000001  /*  0 - Rx Overflow */
#define I_MTO                   0x00000002  /*  1 - Tx Overflow */
#define I_TFD                   0x00000004  /*  2 - Tx flush done */
#define I_LS                    0x00000008  /*  3 - Link status change */
#define I_MDIO                  0x00000010  /*  4 - mdio interrupt */
#define I_MR                    0x00000020  /*  5 - an Rx MIB counter > 50% of max */
#define I_MT                    0x00000040  /*  6 - a Tx MIB counter > 50% of max */
#define I_TO                    0x00000080  /*  7 - Timeout interrupt -- BCM_GMAC3 -- CYW943907 DOES NOT USE THIS */
#define I_PDEE                  0x00000400  /* 10 - descriptor error */
#define I_PDE                   0x00000800  /* 11 - data error */
#define I_DE                    0x00001000  /* 12 - protocol error */
#define I_RDU                   0x00002000  /* 13 - Rx descriptor underflow */
#define I_RFO                   0x00004000  /* 14 - Rx FIFO overflow */
#define I_XFU                   0x00008000  /* 15 - Tx FIFO full */
#define I_RI                    0x00010000  /* 16 - Rx receive complete */
#define I_XI0                   0x01000000  /* bit 24 - Tx dma channel 0 transmit complete interrupt when channel 0 IOC set to one. Write 1 to clear */
#define I_XI1                   0x02000000  /* bit 25 - Tx dma channel 1 transmit complete interrupt when channel 1 IOC set to one. Write 1 to clear */
#define I_XI2                   0x04000000  /* bit 26 - Tx dma channel 2 transmit complete interrupt when channel 2 IOC set to one. Write 1 to clear */
#define I_XI3                   0x08000000  /* bit 27 - Tx dma channel 3 transmit complete interrupt when channel 3 IOC set to one. Write 1 to clear */
#define I_INTMASK               0x0f01fcff  /* Interrupt Mask ( I_MRO | I_MTO | I_TFD | I_LS | I_MDIO | I_MR | I_MT | I_TO | I_PDEE | I_PDE | I_DE | I_RDU | I_RFO | I_XFU | I_RI | I_XI0 - I_XI3) */
#define I_ERRMASK               0x0000fc00  /* Error Mask     (I_PDEE | I_PDE | I_DE | I_RDU | I_RFO | I_XFU) */

/* interrupt receive lazy */
#define IRL_TO_MASK             0x00ffffff
#define IRL_FC_MASK             0xff000000
#define IRL_FC_SHIFT                    24

/* flow control thresholds */
#define FCT_TT_MASK             0x00000fff
#define FCT_RT_MASK             0x0fff0000
#define FCT_RT_SHIFT                    16

/* txq aribter wrr thresholds */
#define WRRT_Q0T_MASK           0x000000ff
#define WRRT_Q1T_MASK           0x0000ff00
#define WRRT_Q1T_SHIFT                   8
#define WRRT_Q2T_MASK           0x00ff0000
#define WRRT_Q2T_SHIFT                  16
#define WRRT_Q3T_MASK           0xff000000
#define WRRT_Q3T_SHIFT                  24

/* phy access */
#define EPHY_PA_DATA_MASK       0x0000ffff
#define EPHY_PA_ADDR_MASK       0x001f0000
#define EPHY_PA_ADDR_SHIFT              16
#define EPHY_PA_REG_MASK        0x1f000000
#define EPHY_PA_REG_SHIFT               24
#define EPHY_PA_WRITE           0x20000000
#define EPHY_PA_START           0x40000000

/* phy control */
#define EPHY_PC_EPA_MASK        0x0000001f
#define EPHY_PC_MCT_MASK        0x007f0000
#define EPHY_PC_MCT_SHIFT               16
#define EPHY_PC_MTE             0x00800000

/* rxq control */
#define RC_DBT_MASK             0x00000fff
#define RC_DBT_SHIFT                     0
#define RC_PTE                  0x00001000
#define RC_MDP_MASK             0x3f000000
#define RC_MDP_SHIFT                    24

#define RC_MAC_DATA_PERIOD               9

/* txq control */
#define TC_DBT_MASK             0x00000fff
#define TC_DBT_SHIFT                     0

/* gpio select */
#define GS_GSC_MASK             0x0000000f
#define GS_GSC_SHIFT                     0

/* gpio output enable */
#define GS_GOE_MASK             0x0000ffff
#define GS_GOE_SHIFT                     0

/* clk control status */
#define CS_FA                   0x00000001
#define CS_FH                   0x00000002
#define CS_FI                   0x00000004
#define CS_AQ                   0x00000008
#define CS_HQ                   0x00000010
#define CS_FC                   0x00000020
#define CS_ER                   0x00000100
#define CS_AA                   0x00010000
#define CS_HA                   0x00020000
#define CS_BA                   0x00040000
#define CS_BH                   0x00080000
#define CS_ES                   0x01000000

/* command config */
#define CC_TE                   0x00000001          /* bit   0 Tx Enable */
#define CC_RE                   0x00000002          /* bit   1 Rx Enable */
#define CC_ES_MASK              0x0000000c          /* bit 2:3 Ethernet Speed 0=10Mbs, 1=100Mbs, 2=1000Mbs, 3=2500Mbs */
#define CC_ES_SHIFT                      2
#define CC_PROM                 0x00000010          /* bit   4 Promiscuous 1=enable */
#define CC_PAD_EN               0x00000020          /* bit   5 Frame Padding enable 1=Remove frame padding and CRC from Rx frame */
#define CC_CF                   0x00000040          /* bit   6 CRC forwarding  1=Pass CRC to Application */
#define CC_PF                   0x00000080          /* bit   7 Pause Forwarding  1=Pass pause frame to App, 0=GMAC drops pause frames */
#define CC_RPI                  0x00000100          /* bit   8 Rx Pause Ignore  1=Ignore Pause Quanta from peer */
#define CC_TAI                  0x00000200          /* bit   9 Tx addr 1, override source addr with value in GMAC registers MAC_0 and MA_1 */
#define CC_HD                   0x00000400          /* bit  10 Half Duplex  1=enable Half Duplex */
#define CC_HD_SHIFT                     10
#define CC_SR(corerev)     ((corerev >= 4) ? 0x00002000 : 0x00000800)
#define CC_ML                   0x00008000          /* bit 15 GMAC looback enable  */
#define CC_MLCON                0x00010000          /* bit 16 mac_loop_con When set to '1', transmit packets to PHY while in
                                                     *        MAC local loopback(LCL_LOOP_ENA is set), otherwise transmit to PHY is disabled
                                                     *        (normal operation) when set to '0' Can be Modified dynamically.
                                                     *        May corrupt the packet on fly during change.
                                                     *        Alternatively change under sw_reset or Tx disable
                                                     */
#define CC_AE                   0x00400000          /* bit 22 UNIMAC auto-config 1=enable auto-config */
#define CC_CFE                  0x00800000          /* bit 23 1=MAC control frames with opcode other than 0x01 are forwarded to Client interface */
#define CC_NLC                  0x01000000          /* bit 24 1=No length check */
#define CC_RL                   0x02000000          /* bit 25 1=Enable remote loopback in the FIFO system side */
#define CC_RED                  0x04000000          /* bit 26 Currently not used */
#define CC_PE                   0x08000000          /* bit 27 */
#define CC_TPI                  0x10000000          /* bit 28 1 = Ignore Tx Pause Frame transmit request */
#define CC_AT                   0x20000000          /* bit 29 1=out-of-band egress flow control is enabled */

/* mac addr high */
#define MH_HI_MASK                  0xffff
#define MH_HI_SHIFT                     16
#define MH_MID_MASK                 0xffff
#define MH_MID_SHIFT                    0

/* mac addr low */
#define ML_LO_MASK                  0xffff
#define ML_LO_SHIFT                      0

/* Core specific control flags */
#define SICF_SWCLKE                 0x0004
#define SICF_SWRST                  0x0008

/* Core specific status flags */
#define SISF_SW_ATTACHED            0x0800

/* 4707 has 4 GMAC and need to be reset before start access */
#define MAX_GMAC_CORES_4707              4

