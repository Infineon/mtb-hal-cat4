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
 */

/** @file
 * Port of siutils.c and aiutils.c. Simplified functionality.
 */

#include "typedefs.h"
#include "bcmutils.h"
#include "cyhal_ethernet_gmac_siutils.h"
#include "sbchipc.h"
#include "hndsoc.h"
#include "cyhal_ethernet_gmac_pmu.h"
#include "cyhal_ethernet_devices.h"
#include "cyhal_ethernet_gmac_dma.h"

#include "platform_map.h"
#include "platform_mcu_peripheral.h"

/** contains resource bit positions for a specific chip */
struct _cyhal_gmac_rsc_per_chip_s {
    uint8 otp_pu;
};

typedef struct _cyhal_gmac_rsc_per_chip_s _cyhal_gmac_rsc_per_chip_t;

#define GMAC_SI_INFO(sih) ((_cyhal_gmac_si_info_t*)(void*)(sih))

#define NOREV        ((uint)-1)

#define PMU_CORE_REV            26
#if ( defined(BCMCHIPREV) && (BCMCHIPREV == 0) )
#define CC_CORE_REV             50
#elif ( defined(BCMCHIPREV) && ((BCMCHIPREV == 1) || (BCMCHIPREV == 2)) )
#define CC_CORE_REV             55
#else
#error CC_CORE_REV not defined!
#endif
#define I2S_CORE_REV             5
#define DDR_CONTROLLER_CORE_REV 0
#define M2MDMA_CORE_REV         0
#define CRYPTO_CORE_REV         0
#define GCI_CORE_REV            4
#define USB20H_CORE_REV         5
#define USB20D_CORE_REV         23
#define SDIOH_CORE_REV          3

#define PMU1_XTALTAB0_960_37400K        14

#define PMUREGS_ILP_SENSITIVE(regoff) \
        ((regoff) == OFFSETOF(pmuregs_t, pmutimer) || \
         (regoff) == OFFSETOF(pmuregs_t, pmuwatchdog) || \
         (regoff) == OFFSETOF(pmuregs_t, res_req_timer))

#define CHIPCREGS_ILP_SENSITIVE(regoff) \
        ((regoff) == OFFSETOF(chipcregs_t, pmutimer) || \
         (regoff) == OFFSETOF(chipcregs_t, pmuwatchdog) || \
         (regoff) == OFFSETOF(chipcregs_t, res_req_timer))

typedef struct _cyhal_gmac_si_private
{
    uint coreid;
    uint corerev;
    void *curmap;
    void *curwrap;
} _cyhal_gmac_si_private_t;

typedef struct _cyhal_gmac_si_info
{
    struct _cyhal_gmac_si_pub pub;
    struct _cyhal_gmac_si_private priv;
    uint curidx;
} _cyhal_gmac_si_info_t;

/* Consider reading this info from chip */
static _cyhal_gmac_si_private_t core_info[] =
{
    {
        /*
         * There are TWO constants on all HND chips: SI_ENUM_BASE
         * and chipcommon being the first core.
         */
        .curmap  = (void*)PLATFORM_CHIPCOMMON_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_CHIPCOMMON_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = CC_CORE_ID,
        .corerev = CC_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_I2S0_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_I2S0_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = I2S_CORE_ID,
        .corerev = I2S_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_I2S1_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_I2S1_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = I2S_CORE_ID,
        .corerev = I2S_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_GMAC_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_GMAC_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = GMAC_CORE_ID,
        .corerev = GMAC_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_I2S0_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_I2S0_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = I2S_CORE_ID,
        .corerev = I2S_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_PMU_REGBASE(0x0),
        .curwrap = NULL,
        .coreid  = PMU_CORE_ID,
        .corerev = PMU_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_DDR_CONTROLLER_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_DDR_CONTROLLER_SLAVE_WRAPPER_REGBASE(0x0),
        .coreid  = DMEMC_CORE_ID, /* may be not what constants used in EPROM */
        .corerev = DDR_CONTROLLER_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_M2M_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_M2M_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = M2MDMA_CORE_ID,
        .corerev = M2MDMA_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_CRYPTO_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_CRYPTO_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = CRYPTO_CORE_ID,
        .corerev = CRYPTO_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_GCI_REGBASE(0x0),
        .curwrap = NULL,
        .coreid  = GCI_CORE_ID,
        .corerev = GCI_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_EHCI_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_USB20H_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = USB20H_CORE_ID,
        .corerev = USB20H_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_USB20D_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_USB20D_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = USB20D_CORE_ID,
        .corerev = USB20D_CORE_REV,
    },
    {
        .curmap  = (void*)PLATFORM_SDIOH_REGBASE(0x0),
        .curwrap = (void*)PLATFORM_SDIOH_MASTER_WRAPPER_REGBASE(0x0),
        .coreid  = SDIOH_CORE_ID,
        .corerev = SDIOH_CORE_REV,
    },
    {
        .curmap  = NULL,
        .curwrap = NULL,
        .coreid  = NODEV_CORE_ID,
        .corerev = NOREV,
    },
};

/* setup pll and query clock speed */
typedef struct {
    uint16  fref;   /* x-tal frequency in [hz] */
    uint8   xf;     /* x-tal index as contained in PMU control reg, see PMU programmers guide */
    uint8   p1div;
    uint8   p2div;
    uint8   ndiv_int;
    uint32  ndiv_frac;
} _cyhal_gmac_pmu1_xtaltab0_t;

static const _cyhal_gmac_pmu1_xtaltab0_t BCMINITDATA(pmu1_xtaltab0_960)[] = {
/*      fref      xf       p1div   p2div  ndiv_int  ndiv_frac */
        {12000,   1,       1,      1,     0x50,   0x0     }, /* array index 0 */
        {13000,   2,       1,      1,     0x49,   0xD89D89},
        {14400,   3,       1,      1,     0x42,   0xAAAAAA},
        {15360,   4,       1,      1,     0x3E,   0x800000},
        {16200,   5,       1,      1,     0x3B,   0x425ED0},
        {16800,   6,       1,      1,     0x39,   0x249249},
        {19200,   7,       1,      1,     0x32,   0x0     },
        {19800,   8,       1,      1,     0x30,   0x7C1F07},
        {20000,   9,       1,      1,     0x30,   0x0     },
        {24000,   10,      1,      1,     0x28,   0x0     },
        {25000,   11,      1,      1,     0x26,   0x666666}, /* array index 10 */
        {26000,   12,      1,      1,     0x24,   0xEC4EC4},
        {30000,   13,      1,      1,     0x20,   0x0     },
        {33600,   14,      1,      1,     0x1C,   0x924924},
        {37400,   15,      2,      1,     0x33,   0x563EF9},
        {38400,   16,      2,      1,     0x32,   0x0     },
        {40000,   17,      2,      1,     0x30,   0x0     },
        {48000,   18,      2,      1,     0x28,   0x0     },
        {52000,   19,      2,      1,     0x24,   0xEC4EC4}, /* array index 18 */
        {0,       0,       0,      0,     0,      0       }
};

/* Private SI Handle */
static _cyhal_gmac_si_info_t ksii;

static _cyhal_gmac_si_info_t*
_cyhal_gmac_si_doattach(_cyhal_gmac_si_info_t *sii)
{
    chipcregs_t *cc = (void*)PLATFORM_CHIPCOMMON_REGBASE(0x0);

    if (sii == NULL)
    {
        return NULL;
    }

    memset(sii, 0, sizeof(*sii));

    sii->pub.chip = BCMCHIPID;  // BCM43909_CHIP_ID for CYW943907AEVAL1F
    sii->pub.bustype = SI_BUS;

    sii->pub.ccrev = CC_CORE_REV;
    sii->pub.chipst = _CYHAL_GMAC_R_REG(&cc->chipstatus);
    sii->pub.cccaps = _CYHAL_GMAC_R_REG(&cc->capabilities);
    sii->pub.cccaps_ext = _CYHAL_GMAC_R_REG(&cc->capabilities_ext);

    if (sii->pub.cccaps & CC_CAP_PMU)
    {
        if (AOB_ENAB(&sii->pub))
        {
            pmuregs_t *pmu = (void*)PLATFORM_PMU_REGBASE(0x0);
            sii->pub.pmucaps = _CYHAL_GMAC_R_REG(&pmu->pmucapabilities);
        }
        else
        {
            sii->pub.pmucaps = _CYHAL_GMAC_R_REG(&cc->pmucapabilities);
        }
        sii->pub.pmurev = sii->pub.pmucaps & PCAP_REV_MASK;
    }

    sii->priv = core_info[ARRAYSIZE(core_info) - 1];
    sii->curidx = BADIDX;

    return sii;
}

_cyhal_gmac_si_t*
_cyhal_gmac_si_attach(uint pcidev, void *regs, uint bustype, void *sdh, char **vars, uint *varsz)
{
    CY_UNUSED_PARAMETER(pcidev);
    CY_UNUSED_PARAMETER(regs);
    CY_UNUSED_PARAMETER(bustype);
    CY_UNUSED_PARAMETER(sdh);
    CY_UNUSED_PARAMETER(vars);
    CY_UNUSED_PARAMETER(varsz);
    _cyhal_gmac_si_info_t *sii = _CYHAL_GMAC_MALLOC(sizeof (_cyhal_gmac_si_info_t));

    _cyhal_gmac_si_doattach(sii);

    return (_cyhal_gmac_si_t*)sii;
}

_cyhal_gmac_si_t*
_cyhal_gmac_si_kattach(void)
{
    static bool ksii_attached = FALSE;

    if (!ksii_attached)
    {
        _cyhal_gmac_si_doattach(&ksii);
        ksii_attached = TRUE;
    }

    return (_cyhal_gmac_si_t*)&ksii;
}

void
_cyhal_gmac_si_detach(_cyhal_gmac_si_t *sih)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);

    if (sii == NULL)
    {
        return;
    }

    _CYHAL_GMAC_MFREE(sii, 0);
}

uint
_cyhal_gmac_si_corerev(_cyhal_gmac_si_t *sih)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    return sii->priv.corerev;
}

uint
_cyhal_gmac_si_coreid(_cyhal_gmac_si_t *sih)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    return sii->priv.coreid;
}

uint
_cyhal_gmac_si_coreidx(_cyhal_gmac_si_t *sih)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    return sii->curidx;
}

void
_cyhal_gmac_si_core_disable(_cyhal_gmac_si_t *sih, uint32 bits)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    aidmp_t *ai = sii->priv.curwrap;
    volatile uint32 dummy;
    uint32 status;

    CY_ASSERT(ai);

    /* if core is already in reset, just return */
    if (_CYHAL_GMAC_R_REG(&ai->resetctrl) & AIRC_RESET)
    {
        return;
    }

    /*
     * Ensure there are no pending backplane operations.
     * 300usecs was sufficient to allow backplane ops to clear for big hammer
     * during driver load we may need more time.
     * If still pending ops, continue on and try disable anyway.
     */
    SPINWAIT(((status = _CYHAL_GMAC_R_REG(&ai->resetstatus)) != 0), 10300);
    UNUSED_PARAMETER(status);

    _CYHAL_GMAC_W_REG(&ai->resetctrl, AIRC_RESET);
    dummy = _CYHAL_GMAC_R_REG(&ai->resetctrl);
    UNUSED_PARAMETER(dummy);
    _CYHAL_GMAC_OSL_DELAY(1);

    _CYHAL_GMAC_W_REG(&ai->ioctrl, bits);
    dummy = _CYHAL_GMAC_R_REG(&ai->ioctrl);
    UNUSED_PARAMETER(dummy);
    _CYHAL_GMAC_OSL_DELAY(10);
}

uint32
_cyhal_gmac_si_core_cflags(_cyhal_gmac_si_t *sih, uint32 mask, uint32 val)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    aidmp_t *ai = sii->priv.curwrap;

    CY_ASSERT(ai);
    CY_ASSERT((val & ~mask) == 0);

    if (mask || val)
    {
        uint32_t w = (_CYHAL_GMAC_R_REG(&ai->ioctrl) & ~mask) | val;
        _CYHAL_GMAC_W_REG(&ai->ioctrl, w);
    }

    return _CYHAL_GMAC_R_REG(&ai->ioctrl);
}

uint32
_cyhal_gmac_si_core_sflags(_cyhal_gmac_si_t *sih, uint32 mask, uint32 val)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    aidmp_t *ai = sii->priv.curwrap;

    CY_ASSERT(ai);
    CY_ASSERT((val & ~mask) == 0);
    CY_ASSERT((mask & ~SISF_CORE_BITS) == 0);

    if (mask || val)
    {
        uint32_t w = (_CYHAL_GMAC_R_REG(&ai->iostatus) & ~mask) | val;
        _CYHAL_GMAC_W_REG(&ai->iostatus, w);
    }

    return _CYHAL_GMAC_R_REG(&ai->iostatus);
}

bool
_cyhal_gmac_si_iscoreup_wrapper(void *wrapper)
{
    aidmp_t *ai = wrapper;

    CY_ASSERT(ai);

    return (((_CYHAL_GMAC_R_REG(&ai->ioctrl) & (SICF_FGC | SICF_CLOCK_EN)) == SICF_CLOCK_EN) &&
           ((_CYHAL_GMAC_R_REG(&ai->resetctrl) & AIRC_RESET) == 0));
}

bool
_cyhal_gmac_si_iscoreup(_cyhal_gmac_si_t *sih)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    return _cyhal_gmac_si_iscoreup_wrapper(sii->priv.curwrap);
}

uint32
_cyhal_gmac_si_clock(_cyhal_gmac_si_t *sih)
{
    chipcregs_t *cc = (void*)PLATFORM_CHIPCOMMON_REGBASE(0x0);
    uint32 n, m;
    uint32 pll_type, rate;

    if (PMUCTL_ENAB(sih))
    {
        rate = _cyhal_gmac_si_pmu_si_clock(sih);
        goto exit;
    }

    n = _CYHAL_GMAC_R_REG(&cc->clockcontrol_n);
    pll_type = sih->cccaps & CC_CAP_PLL_MASK;
    if (pll_type == PLL_TYPE6)
    {
        m = _CYHAL_GMAC_R_REG(&cc->clockcontrol_m3);
    }
    else if (pll_type == PLL_TYPE3)
    {
        m = _CYHAL_GMAC_R_REG(&cc->clockcontrol_m2);
    }
    else
    {
        m = _CYHAL_GMAC_R_REG(&cc->clockcontrol_sb);
    }

    /* calculate rate */
    rate = _cyhal_gmac_si_clock_rate(pll_type, n, m);

    if (pll_type == PLL_TYPE3)
    {
        rate = rate / 2;
    }

exit:
    return rate;
}

static uint32
_cyhal_gmac_factor6(uint32 x)
{
    switch (x)
    {
        case CC_F6_2: return 2;
        case CC_F6_3: return 3;
        case CC_F6_4: return 4;
        case CC_F6_5: return 5;
        case CC_F6_6: return 6;
        case CC_F6_7: return 7;
        default:      return 0;
    }
}

uint32
_cyhal_gmac_si_clock_rate(uint32 pll_type, uint32 n, uint32 m)
{
    uint32 n1, n2, clock, m1, m2, m3, mc;
    bool valid_clock_divisor = true;

    n1 = n & CN_N1_MASK;
    n2 = (n & CN_N2_MASK) >> CN_N2_SHIFT;

    if (pll_type == PLL_TYPE6)
    {
        if (m & CC_T6_MMASK)
        {
            return CC_T6_M1;
        }
        else
        {
            return CC_T6_M0;
        }
    }
    else if ((pll_type == PLL_TYPE1) || (pll_type == PLL_TYPE3) || (pll_type == PLL_TYPE4) || (pll_type == PLL_TYPE7))
    {
        n1 = _cyhal_gmac_factor6(n1);
        n2 += CC_F5_BIAS;
    }
    else if (pll_type == PLL_TYPE2)
    {
        n1 += CC_T2_BIAS;
        n2 += CC_T2_BIAS;
        CY_ASSERT((n1 >= 2) && (n1 <= 7));
        CY_ASSERT((n2 >= 5) && (n2 <= 23));
    }
    else if (pll_type == PLL_TYPE5)
    {
        /* XXX: 5365 */
        return (100000000);
    }
    else
    {
        CY_ASSERT(0);
    }

    /* PLL types 3 and 7 use BASE2 (25Mhz) */
    if ((pll_type == PLL_TYPE3) || (pll_type == PLL_TYPE7))
    {
        clock = CC_CLOCK_BASE2 * n1 * n2;
    }
    else
    {
        clock = CC_CLOCK_BASE1 * n1 * n2;
    }
    if (clock == 0)
    {
        return 0;
    }

    m1 = m & CC_M1_MASK;
    m2 = (m & CC_M2_MASK) >> CC_M2_SHIFT;
    m3 = (m & CC_M3_MASK) >> CC_M3_SHIFT;
    mc = (m & CC_MC_MASK) >> CC_MC_SHIFT;

    if ((pll_type == PLL_TYPE1) || (pll_type == PLL_TYPE3) || (pll_type == PLL_TYPE4) || (pll_type == PLL_TYPE7))
    {
        m1 = _cyhal_gmac_factor6(m1);
        if ((pll_type == PLL_TYPE1) || (pll_type == PLL_TYPE3))
        {
            m2 += CC_F5_BIAS;
        }
        else
        {
            m2 = _cyhal_gmac_factor6(m2);
        }
        m3 = _cyhal_gmac_factor6(m3);

        /* need to error check due to possible misconfiguration or corruption */
        switch (mc)
        {
            case CC_MC_M1:
                valid_clock_divisor = ( m1 != 0 );
                break;
            case CC_MC_M1M2:
                valid_clock_divisor = ( m1 != 0 && m2 != 0 );
                break;
            case CC_MC_M1M2M3:
                valid_clock_divisor = ( m1 != 0 && m2 != 0 && m3 != 0 );
                break;
            case CC_MC_M1M3:
                valid_clock_divisor = ( m1 != 0 && m3 != 0 );
                break;
            case CC_MC_BYPASS: /* FALLSTHROUGH */
            default:
                break; /* No reliance on clock divisors here */
        }

        /* catch bad data here */
        if ( false == valid_clock_divisor )
        {
            // invalid clock divisor
            CY_ASSERT(false);
            //WPRINT_WICED_ERROR(( "Invalid clock divisor pll=%d, m1 = %d m2 = %d m3 = %d\n", mc, m1, m2, m3 ));

            return 0;
        }

        /* use clock divisor */
        switch (mc)
        {
            case CC_MC_BYPASS: return (clock);
            case CC_MC_M1:     return (clock / m1);
            case CC_MC_M1M2:   return (clock / (m1 * m2));
            case CC_MC_M1M2M3: return (clock / (m1 * m2 * m3));
            case CC_MC_M1M3:   return (clock / (m1 * m3));
            default:           return (0);
        }
    }
    else
    {
        CY_ASSERT(pll_type == PLL_TYPE2);

        m1 += CC_T2_BIAS;
        m2 += CC_T2M2_BIAS;
        m3 += CC_T2_BIAS;
        CY_ASSERT((m1 >= 2) && (m1 <= 7));
        CY_ASSERT((m2 >= 3) && (m2 <= 10));
        CY_ASSERT((m3 >= 2) && (m3 <= 7));

        if ((mc & CC_T2MC_M1BYP) == 0)
        {
            clock /= m1;
        }
        if ((mc & CC_T2MC_M2BYP) == 0)
        {
            clock /= m2;
        }
        if ((mc & CC_T2MC_M3BYP) == 0)
        {
            clock /= m3;
        }

        return clock;
    }
}

void
_cyhal_gmac_si_core_reset_set_wrapper(void *wrapper, uint32 bits, uint32 resetbits)
{
    aidmp_t *ai = wrapper;
    volatile uint32 dummy;

    CY_ASSERT(ai);

    /* ensure there are no pending backplane operations */
    SPINWAIT(((dummy = _CYHAL_GMAC_R_REG(&ai->resetstatus)) != 0), 300);

    /* put core into reset state */
    _CYHAL_GMAC_W_REG(&ai->resetctrl, AIRC_RESET);
    _CYHAL_GMAC_OSL_DELAY(10);

    /* ensure there are no pending backplane operations */
    SPINWAIT((_CYHAL_GMAC_R_REG(&ai->resetstatus) != 0), 300);

    _CYHAL_GMAC_W_REG(&ai->ioctrl, (bits | resetbits | SICF_FGC | SICF_CLOCK_EN));
    dummy = _CYHAL_GMAC_R_REG(&ai->ioctrl);
    UNUSED_PARAMETER(dummy);
}

void
_cyhal_gmac_si_core_reset_clear_wrapper(void *wrapper, uint32 bits)
{
    aidmp_t *ai = wrapper;
    volatile uint32 dummy;
    uint loop_counter;

    CY_ASSERT(ai);

    /* ensure there are no pending backplane operations */
    SPINWAIT(((dummy = _CYHAL_GMAC_R_REG(&ai->resetstatus)) != 0), 300);

    loop_counter = 10;
    while (_CYHAL_GMAC_R_REG(&ai->resetctrl) != 0 && --loop_counter != 0)
    {
        /* ensure there are no pending backplane operations */
        SPINWAIT(((dummy = _CYHAL_GMAC_R_REG(&ai->resetstatus)) != 0), 300);

        /* take core out of reset */
        _CYHAL_GMAC_W_REG(&ai->resetctrl, 0);

        /* ensure there are no pending backplane operations */
        SPINWAIT((_CYHAL_GMAC_R_REG(&ai->resetstatus) != 0), 300);
    }

    _CYHAL_GMAC_W_REG(&ai->ioctrl, (bits | SICF_CLOCK_EN));
    dummy = _CYHAL_GMAC_R_REG(&ai->ioctrl);
    UNUSED_PARAMETER(dummy);
    _CYHAL_GMAC_OSL_DELAY(1);
}

void
_cyhal_gmac_si_core_reset_wrapper(void *wrapper, uint32 bits, uint32 resetbits)
{
    _cyhal_gmac_si_core_reset_set_wrapper(wrapper, bits, resetbits);
    _cyhal_gmac_si_core_reset_clear_wrapper(wrapper, bits);
}

void
_cyhal_gmac_si_core_reset(_cyhal_gmac_si_t *sih, uint32 bits, uint32 resetbits)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    void *ai = sii->priv.curwrap;

    _cyhal_gmac_si_core_reset_wrapper(ai, bits, resetbits);
}

uint
_cyhal_gmac_si_findcoreidx(_cyhal_gmac_si_t *sih, uint coreid, uint coreunit)
{
    CY_UNUSED_PARAMETER(sih);
    uint found = 0;
    uint i;

    for (i = 0; i < ARRAYSIZE(core_info); ++i)
    {
        if (core_info[i].coreid == coreid)
        {
            if (found == coreunit)
            {
                return i;
            }
            found++;
        }
    }

    return BADIDX;
}

void*
_cyhal_gmac_si_setcoreidx(_cyhal_gmac_si_t *sih, uint coreindex)
{
    _cyhal_gmac_si_info_t *sii = GMAC_SI_INFO(sih);
    void *curmap = NULL;

    if (coreindex < ARRAYSIZE(core_info))
    {
        sii->priv = core_info[coreindex];
        sii->curidx = coreindex;
        curmap = sii->priv.curmap;
    }

    return curmap;
}

void*
_cyhal_gmac_si_setcore(_cyhal_gmac_si_t *sih, uint coreid, uint coreunit)
{
    uint coreindex = _cyhal_gmac_si_findcoreidx(sih, coreid, coreunit);
    return _cyhal_gmac_si_setcoreidx(sih, coreindex);
}

uint
_cyhal_gmac_si_corereg(_cyhal_gmac_si_t *sih, uint coreindex, uint regoff, uint mask, uint val)
{
    CY_UNUSED_PARAMETER(sih);
    uint32 *r;
    uint w;

    CY_ASSERT(coreindex < SI_MAXCORES);
    CY_ASSERT(regoff < SI_CORE_SIZE);
    CY_ASSERT((val & ~mask) == 0);

    if (coreindex >= ARRAYSIZE(core_info))
    {
        return 0;
    }

    r = (uint32 *)((uchar *)core_info[coreindex].curmap + regoff);
    CY_ASSERT(r != NULL);

    /* mask and set */
    if (mask || val)
    {
        w = (_CYHAL_GMAC_R_REG(r) & ~mask) | val;
        _CYHAL_GMAC_W_REG(r, w);
    }

    /* readback */
    w = _CYHAL_GMAC_R_REG(r);

    return w;
}

uint32*
_cyhal_gmac_si_corereg_addr(_cyhal_gmac_si_t *sih, uint coreindex, uint regoff)
{
    CY_UNUSED_PARAMETER(sih);
    uint32 *r = NULL;

    CY_ASSERT(coreindex < SI_MAXCORES);
    CY_ASSERT(regoff < SI_CORE_SIZE);

    if (coreindex >= ARRAYSIZE(core_info))
    {
        return 0;
    }

    r = (uint32 *)((uchar *)core_info[coreindex].curmap + regoff);
    CY_ASSERT(r != NULL);

    return r;
}

void
_cyhal_gmac_si_pci_setup(_cyhal_gmac_si_t *sih, uint coremask)
{
    CY_UNUSED_PARAMETER(sih);
    CY_UNUSED_PARAMETER(coremask);
}

bool
_cyhal_gmac_si_is_otp_disabled(_cyhal_gmac_si_t *sih)
{
    CY_UNUSED_PARAMETER(sih);
    switch (CHIPID(sih->chip))
    {
        /* chip-specific behavior specification */
        case BCM43909_CHIP_ID:
            return FALSE;
        default:
            return FALSE;
    }
}


/* Returns current value of PMUTimer.
 */
uint32
_cyhal_gmac_si_pmu_get_pmutimer(_cyhal_gmac_si_t *sih, chipcregs_t *cc)
{
    CY_UNUSED_PARAMETER(cc);
    uint32 start;
    start = _CYHAL_GMAC_R_REG(GMAC_PMUREG(sih, pmutimer));
    if (start != _CYHAL_GMAC_R_REG(GMAC_PMUREG(sih, pmutimer)))
        start = _CYHAL_GMAC_R_REG(GMAC_PMUREG(sih, pmutimer));
    return (start);
}

/* Returns
 * a) diff between a 'prev' value of pmu timer and current value
 * b) the current pmutime value in 'prev'
 * So, 'prev' is an IO parameter.
 */
uint32
_cyhal_gmac_si_pmu_get_pmutime_diff(_cyhal_gmac_si_t *sih, chipcregs_t *cc, uint32 *prev)
{
    uint32 pmutime_diff = 0, pmutime_val = 0;
    uint32 prev_val = *prev;

    /* read current value */
    pmutime_val = _cyhal_gmac_si_pmu_get_pmutimer(sih, cc);
    /* diff btween prev and current value, take on wraparound case as well. */
    pmutime_diff = pmutime_val - prev_val;

    *prev = pmutime_val;
    return pmutime_diff;
}


/** Wait for usec for the res_pending register to change. */
/*
 *  NOTE: usec SHOULD be > 32uS
 *  if cond = TRUE, res_pending will be read until it becomes == 0;
 *  If cond = FALSE, res_pending will be read until it becomes != 0;
 *  returns TRUE if timedout.
 *  returns elapsed time in this loop in elapsed_time
 */
bool
_cyhal_gmac_si_pmu_wait_for_res_pending(_cyhal_gmac_si_t *sih, chipcregs_t *cc, uint usec,
    bool cond, uint32 *elapsed_time)
{
    uint countdown = usec;
    uint32 pmutime_prev = 0, pmutime_elapsed = 0, res_pend;
    bool pending = FALSE;

    /* PMU timer is driven by ILP clock */
    uint pmu_us_steps = (uint)(1000000 / osl_ilp_clock()) + 1;

    /* store current time */
    pmutime_prev = _cyhal_gmac_si_pmu_get_pmutimer(sih, cc);
    while (1)
    {
        res_pend = _CYHAL_GMAC_R_REG( GMAC_PMUREG(sih, res_pending));

        /* based on the condition, check */
        if (cond == TRUE)
        {
            if (res_pend == 0) break;
        }
        else
        {
            if (res_pend != 0) break;
        }

        /* if required time over */
        if ((pmutime_elapsed * pmu_us_steps) >= countdown)
        {
            /* timeout. so return as still pending */
            pending = TRUE;
            break;
        }

        /* get elapsed time after adding diff between prev and current
        * pmutimer value
        */
        pmutime_elapsed += _cyhal_gmac_si_pmu_get_pmutime_diff(sih, cc, &pmutime_prev);
    }

    *elapsed_time = pmutime_elapsed * pmu_us_steps;
    return pending;
} /* _cyhal_gmac_si_pmu_wait_for_res_pending */

/**
 *  The algorithm for pending check is that,
 *  step1:  wait till (res_pending !=0) OR pmu_max_trans_timeout.
 *          if max_trans_timeout, flag error and exit.
 *          wait for 1 ILP clk [64uS] based on pmu timer,
 *          polling to see if res_pending again goes high.
 *          if res_pending again goes high, go back to step1.
 *  Note: res_pending is checked repeatedly because, in between switching
 *  of dependent
 *  resources, res_pending resets to 0 for a short duration of time before
 *  it becomes 1 again.
 *  Note: return 0 is GOOD, 1 is BAD [mainly timeout].
 */
int _cyhal_gmac_si_pmu_wait_for_steady_state(_cyhal_gmac_si_t *sih, chipcregs_t *cc)
{
    int stat = 0;
    bool timedout = FALSE;
    uint32 elapsed = 0, pmutime_total_elapsed = 0;

    while (1)
    {
        /* wait until all resources are settled down [till res_pending becomes 0] */
        timedout = _cyhal_gmac_si_pmu_wait_for_res_pending(sih, cc,
            PMU_MAX_TRANSITION_DLY, TRUE, &elapsed);

        if (timedout)
        {
            stat = 1;
            break;
        }

        pmutime_total_elapsed += elapsed;
        /* wait to check if any resource comes back to non-zero indicating
        * that it pends again. The res_pending goes 0 for 1 ILP clock before
        * getting set for next resource in the sequence , so if res_pending
        * is 0 for more than 1 ILP clk it means nothing is pending
        * to indicate some pending dependency.
        */
        timedout = _cyhal_gmac_si_pmu_wait_for_res_pending(sih, cc, 64, FALSE, &elapsed);

        pmutime_total_elapsed += elapsed;
        /* Here, we can also check timedout, but we make sure that,
        * we read the res_pending again.
        */
        if (timedout)
        {
            stat = 0;
            break;
        }

        /* Total wait time for all the waits above added should be
        * less than  PMU_MAX_TRANSITION_DLY
        */
        if (pmutime_total_elapsed >= PMU_MAX_TRANSITION_DLY)
        {
            /* timeout. so return as still pending */
            stat = 1;
            break;
        }
    }
    return stat;
} /* _cyhal_gmac_si_pmu_wait_for_steady_state */


static _cyhal_gmac_rsc_per_chip_t rsc_43909 =  {RES43909_OTP_PU};

/**
* For each chip, location of resource bits (e.g., ht bit) in resource mask registers may differ.
* This function abstracts the bit position of commonly used resources, thus making the rest of the
* code in hndpmu.c cleaner.
*/
static _cyhal_gmac_rsc_per_chip_t* _cyhal_gmac_si_pmu_get_rsc_positions(_cyhal_gmac_si_t *sih)
{
    CY_UNUSED_PARAMETER(sih);
    _cyhal_gmac_rsc_per_chip_t *rsc = NULL;

    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            rsc = &rsc_43909;
            break;
        default:
            CY_ASSERT(0);
            break;
    }

    return rsc;
}; /* _cyhal_gmac_si_pmu_get_rsc_positions */


bool
_cyhal_gmac_si_pmu_is_otp_powered(_cyhal_gmac_si_t *sih)
{
    uint idx;
    chipcregs_t *cc;
    bool st;
    _cyhal_gmac_rsc_per_chip_t *rsc;        /* chip specific resource bit positions */

    /* Remember original core before switch to chipc */
    idx = _cyhal_gmac_si_coreidx(sih);
    cc = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    CY_ASSERT(cc != NULL);

    _cyhal_gmac_si_pmu_wait_for_steady_state(sih, cc);

    switch (CHIPID(sih->chip))
    {
    case BCM43909_CHIP_ID:
        rsc = _cyhal_gmac_si_pmu_get_rsc_positions(sih);
        st = (_CYHAL_GMAC_R_REG(GMAC_PMUREG(sih, res_state)) & PMURES_BIT(rsc->otp_pu)) != 0;
        break;
    default:
        st = TRUE;
        break;
    }

    /* Return to original core */
    _cyhal_gmac_si_setcoreidx(sih, idx);

    return st;
} /* _cyhal_gmac_si_pmu_is_otp_powered */

bool
_cyhal_gmac_si_is_otp_powered(_cyhal_gmac_si_t *sih)
{
    if (PMUCTL_ENAB(sih))
    {
        return _cyhal_gmac_si_pmu_is_otp_powered(sih);
    }

    return TRUE;
}

uint32
_cyhal_gmac_si_pmu_pllcontrol(_cyhal_gmac_si_t *sih, uint reg, uint32 mask, uint32 val)
{
    pmu_corereg(sih, SI_CC_IDX, pllcontrol_addr, ~0, reg);
    return pmu_corereg(sih, SI_CC_IDX, pllcontrol_data, mask, val);
}

uint32
_cyhal_gmac_si_pmu_cal_fvco(_cyhal_gmac_si_t *sih)
{
    uint32 xf, ndiv_int, ndiv_frac, fvco, pll_reg, p1_div_scale;
    uint32 r_high, r_low, int_part, frac_part, rounding_const;
    uint8 p1_div;

    xf = _cyhal_gmac_si_pmu_alp_clock(sih)/1000;

    pll_reg = _cyhal_gmac_si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL2, 0, 0);

    p1_div = (pll_reg & PMU4335_PLL0_PC2_P1DIV_MASK) >> PMU4335_PLL0_PC2_P1DIV_SHIFT;
    if (p1_div == 0)
    {
        CY_ASSERT(p1_div != 0);
        return 0;
    }

    ndiv_int = (pll_reg & PMU4335_PLL0_PC2_NDIV_INT_MASK) >> PMU4335_PLL0_PC2_NDIV_INT_SHIFT;

    pll_reg = _cyhal_gmac_si_pmu_pllcontrol(sih, PMU1_PLL0_PLLCTL3, 0, 0);

    ndiv_frac = (pll_reg & PMU1_PLL0_PC3_NDIV_FRAC_MASK) >> PMU1_PLL0_PC3_NDIV_FRAC_SHIFT;

    /* Actual expression is as below */
    /* fvco1 = (100 * (xf * 1/p1_div) * (ndiv_int + (ndiv_frac * 1/(1 << 24)))) */
    /* * 1/(1000 * 100); */

    /* Representing 1/p1_div as a 12 bit number */
    /* Reason for the choice of 12: */
    /* ndiv_int is represented by 9 bits */
    /* so (ndiv_int << 24) needs 33 bits */
    /* xf needs 16 bits for the worst case of 52MHz clock */
    /* So (xf * (ndiv << 24)) needs atleast 49 bits */
    /* So remaining bits for uint64 : 64 - 49 = 15 bits */
    /* So, choosing 12 bits, with 3 bits of headroom */
    int_part = xf * ndiv_int;

    rounding_const = 1 << (BBPLL_NDIV_FRAC_BITS - 1);
    bcm4390x_uint64_multiple_add(&r_high, &r_low, ndiv_frac, xf, rounding_const);
    bcm4390x_uint64_right_shift(&frac_part, r_high, r_low, BBPLL_NDIV_FRAC_BITS);

    p1_div_scale = (1 << P1_DIV_SCALE_BITS) / p1_div;
    rounding_const = 1 << (P1_DIV_SCALE_BITS - 1);

    bcm4390x_uint64_multiple_add(&r_high, &r_low, (int_part + frac_part), p1_div_scale, rounding_const);
    bcm4390x_uint64_right_shift(&fvco, r_high, r_low, P1_DIV_SCALE_BITS);

    return fvco;
}


static uint32
_cyhal_gmac_si_pmu1_pllfvco0(_cyhal_gmac_si_t *sih)
{
    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            return _cyhal_gmac_si_pmu_cal_fvco(sih);
    }

    return 0;
}

static uint32
_cyhal_gmac_si_pmu1_cpuclk0(_cyhal_gmac_si_t *sih, chipcregs_t *cc)
{
    CY_UNUSED_PARAMETER(cc);

    uint32 tmp, mdiv = 1;
    uint32 FVCO = _cyhal_gmac_si_pmu1_pllfvco0(sih); /* in [hz] units */

    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            /* Read m3div from pllcontrol[1] */
            W_REG(NULL, GMAC_PMUREG(sih, pllcontrol_addr), PMU1_PLL0_PLLCTL1);
            tmp = R_REG(NULL, GMAC_PMUREG(sih, pllcontrol_data));
            mdiv = (tmp & PMU1_PLL0_PC1_M3DIV_MASK) >> PMU1_PLL0_PC1_M3DIV_SHIFT;
            break;
    }

    return FVCO / mdiv * 1000;
}

uint32
_cyhal_gmac_si_pmu_si_clock(_cyhal_gmac_si_t *sih)
{
    chipcregs_t *cc = (void*)PLATFORM_CHIPCOMMON_REGBASE(0x0);
    uint32 clock = HT_CLOCK; /* in [hz] units */

    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            clock = _cyhal_gmac_si_pmu1_cpuclk0(sih, cc);
            break;
    }

    return clock;
}

static const _cyhal_gmac_pmu1_xtaltab0_t *
_cyhal_gmac_si_pmu1_xtaltab0(_cyhal_gmac_si_t *sih)
{
    CY_UNUSED_PARAMETER(sih);
    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            return pmu1_xtaltab0_960;
    }

    return NULL;
}


static const _cyhal_gmac_pmu1_xtaltab0_t *
_cyhal_gmac_si_pmu1_xtaldef0(_cyhal_gmac_si_t *sih)
{
    CY_UNUSED_PARAMETER(sih);
    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            return &pmu1_xtaltab0_960[PMU1_XTALTAB0_960_37400K];
    }

    return NULL;
}


static uint32
_cyhal_gmac_si_pmu1_alpclk0(_cyhal_gmac_si_t *sih, chipcregs_t *cc)
{
    CY_UNUSED_PARAMETER(cc);
    const _cyhal_gmac_pmu1_xtaltab0_t *xt;
    uint32 xf;

    /* Find the frequency in the table */
    xf = (_CYHAL_GMAC_R_REG(GMAC_PMUREG(sih, pmucontrol)) & PCTL_XTALFREQ_MASK) >> PCTL_XTALFREQ_SHIFT;
    for (xt = _cyhal_gmac_si_pmu1_xtaltab0(sih); xt != NULL && xt->fref != 0; xt ++)
    {
        if (xt->xf == xf)
        {
            break;
        }
    }

    /* Could not find it so assign a default value */
    if (xt == NULL || xt->fref == 0)
    {
        xt = _cyhal_gmac_si_pmu1_xtaldef0(sih);
    }
    CY_ASSERT(xt != NULL && xt->fref != 0);

    return xt->fref * 1000;
}


uint32
_cyhal_gmac_si_pmu_alp_clock(_cyhal_gmac_si_t *sih)
{
    chipcregs_t *cc = (void*)PLATFORM_CHIPCOMMON_REGBASE(0x0);
    uint32 clock = ALP_CLOCK;

    switch (CHIPID(sih->chip))
    {
        case BCM43909_CHIP_ID:
            clock = _cyhal_gmac_si_pmu1_alpclk0(sih, cc);
            break;
    }

    return clock;
}

bool
_cyhal_gmac_si_pmu_is_ilp_sensitive(uint32 idx, uint regoff)
{
    if (idx == SI_CC_IDX)
    {
        return CHIPCREGS_ILP_SENSITIVE(regoff);
    }

    return PMUREGS_ILP_SENSITIVE(regoff);
}

uint
_cyhal_gmac_si_pmu_corereg(_cyhal_gmac_si_t *sih, uint32 idx, uint regoff, uint mask, uint val)
{
    int pmustatus_offset;

    /* prevent backplane stall on double write to 'ILP domain' registers in the PMU */
    if (mask != 0 && sih->pmurev >= 22 && _cyhal_gmac_si_pmu_is_ilp_sensitive(idx, regoff))
    {
        pmustatus_offset = AOB_ENAB(sih) ? OFFSETOF(pmuregs_t, pmustatus) : OFFSETOF(chipcregs_t, pmustatus);
        while (_cyhal_gmac_si_corereg(sih, idx, pmustatus_offset, 0, 0) & PST_SLOW_WR_PENDING);
    }

    return _cyhal_gmac_si_corereg(sih, idx, regoff, mask, val);
}

