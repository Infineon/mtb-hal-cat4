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
 *
 * 43909 WICED OTP Driver
 */

#include <typedefs.h>
//#include <osl.h>
#include "cyhal_ethernet_etc.h"        // for debug logging elements
#include "bcmdefs.h"
#include "cyhal_ethernet_devices.h"
#include "bcmutils.h"
#include "bcmendian.h"
#include "hndsoc.h"
#include "sbchipc.h"
#include "cyhal_ethernet_gmac_siutils.h"
#include <sbpcmcia.h>
#include "cyhal_ethernet_gmac_dma.h"
#include "platform_otp.h"

/* If you enable this, it requires an RTOS! */
//#define CYW43907_OTP_MUTHITHREADED


/*
 * The driver supports the following:
 * 1. New IPX OTP controller
 * 2. ChipCommon revision >= 49
 * 3. 40nm wrapper type layout
 */

/* What device ID should be passed to _cyhal_gmac_si_attach? */
#define OTP_DEV_ID  0

#define OTPTYPE_IPX(ccrev)  ((ccrev) == 21 || (ccrev) >= 23)
#define OTPPROWMASK(ccrev)  ((ccrev >= 49) ? OTPP_ROW_MASK9 : OTPP_ROW_MASK)

#define _CYHAL_PLATFORM_OTP_BASE   (PLATFORM_GCI_COREBASE(0x1000))

#define _CYHAL_ETHERNET_LOG_DEBUG(args)

#define OTPWSIZE        16  /* word size */

/* OTP layout */
/* Subregion word offsets in General Use region */
#define OTPGU_HSB_OFF       0
#define OTPGU_SFB_OFF       1
#define OTPGU_CI_OFF        2
#define OTPGU_P_OFF         3
#define OTPGU_SROM_OFF      4

/* Flag bit offsets in General Use region  */
#define OTPGU_NEWCISFORMAT_OFF  59
#define OTPGU_HWP_OFF           60
#define OTPGU_SWP_OFF           61
#define OTPGU_CIP_OFF           62
#define OTPGU_FUSEP_OFF         63
#define OTPGU_CIP_MSK           0x4000
#define OTPGU_P_MSK             0xf000
#define OTPGU_P_SHIFT           (OTPGU_HWP_OFF % 16)

/* OTP Size */
#define OTP_SZ_FU_608       ((ROUNDUP(608, 16))/8)
#define OTP_SZ_CHECKSUM     (16/8)      /* 16 bits */

/* Fixed size subregions sizes in words */
#define OTPGU_CI_SZ     2

/* OTP BT shared region (pre-allocated) */
#define OTP_BT_BASE_43909    (0)    /* 43909 does not need OTP BT region */
#define OTP_BT_END_43909     (0)

#define MAXNUMRDES          9    /* Maximum OTP redundancy entries */

#define OTP_HW_REGION_SDIO_HW_HEADER_SIZE    12

#define _CYHAL_PLATFORM_OTP_SECURE_BIT_OFFSET 387

/* OTP common registers */
typedef struct {
    volatile uint32_t *otpstatus;
    volatile uint32_t *otpcontrol;
    volatile uint32_t *otpprog;
    volatile uint32_t *otplayout;
    volatile uint32_t *otpcontrol1;
    volatile uint32_t *otplayoutextension;
} otpregs_t;

/* OTP common function type */
typedef void*    (*otp_init_t)(_cyhal_gmac_si_t *sih);
typedef uint32_t (*otp_status_t)(void *oh);
typedef int      (*otp_size_t)(void *oh);
typedef uint16_t (*otp_read_bit_t)(void *oh, unsigned int off);
typedef int      (*otp_read_word_t)(void *oh, unsigned int wn, uint16_t *data);
typedef int      (*otp_read_region_t)(_cyhal_gmac_si_t *sih, _cyhal_platform_otp_region_t region, uint16_t *data, unsigned int *wlen);
typedef int      (*otp_get_region_t)(void *oh, _cyhal_platform_otp_region_t region, unsigned int *wbase, unsigned int *wlen);
typedef bool     (*otp_isunified_t)(void *oh);
typedef uint16_t (*otp_avsbitslen_t)(void *oh);
typedef void     (*otp_dump_t)(void *oh);
typedef int      (*otp_lock_t)(_cyhal_gmac_si_t *sih);
typedef int      (*otp_nvread_t)(void *oh, char *data, unsigned int *len);
typedef int      (*otp_nvwrite_t)(void *oh, uint16_t *data, unsigned int wlen);
typedef int      (*otp_write_bits_t)(void *oh, int bn, int bits, uint8_t* data);
typedef int      (*otp_write_word_t)(void *oh, unsigned int wn, uint16_t data);
typedef int      (*otp_write_region_t)(void *oh, int region, uint16_t *data, unsigned int wlen, unsigned int flags);
typedef int      (*otp_cis_append_region_t)(_cyhal_gmac_si_t *sih, int region, char *vars, int count);

/* OTP function struct */
typedef struct otp_fn_s {
    otp_size_t              size;
    otp_read_bit_t          read_bit;
    otp_dump_t              dump;
    otp_status_t            status;
    otp_init_t              init;
    otp_read_region_t       read_region;
    otp_get_region_t        get_region;
    otp_nvread_t            nvread;
    otp_write_region_t      write_region;
    otp_cis_append_region_t cis_append_region;
    otp_lock_t              lock;
    otp_nvwrite_t           nvwrite;
    otp_write_word_t        write_word;
    otp_read_word_t         read_word;
    otp_write_bits_t        write_bits;
    otp_isunified_t         isunified;
    otp_avsbitslen_t        avsbitslen;
} otp_fn_t;

typedef struct {
    unsigned int  ccrev;            /* chipc revision */
    otp_fn_t      *fn;              /* OTP functions */
    _cyhal_gmac_si_t          *sih;             /* Saved sb handle */

    /* IPX OTP section */
    uint16_t      wsize;              /* Size of otp in words */
    uint16_t      rows;               /* Geometry */
    uint16_t      cols;               /* Geometry */
    uint32_t      status;             /* Flag bits (lock/prog/rv).
                                       * (Reflected only when OTP is power cycled)
                                       */
    uint16_t      hwbase;             /* hardware subregion offset */
    uint16_t      hwlim;              /* hardware subregion boundary */
    uint16_t      swbase;             /* software subregion offset */
    uint16_t      swlim;              /* software subregion boundary */
    uint16_t      fbase;              /* fuse subregion offset */
    uint16_t      flim;               /* fuse subregion boundary */
    int           otpgu_base;         /* offset to General Use Region */
    uint16_t      fusebits;           /* num of fusebits */
    bool          buotp;              /* Unified OTP flag */
    unsigned int  usbmanfid_offset;   /* Offset of the usb manfid inside the sdio CIS */
    struct {
        uint8_t width;                /* entry width in bits */
        uint8_t val_shift;            /* value bit offset in the entry */
        uint8_t offsets;              /* # entries */
        uint8_t stat_shift;           /* valid bit in otpstatus */
        uint16_t offset[MAXNUMRDES];  /* entry offset in OTP */
    } rde_cb;                         /* OTP redundancy control blocks */
    uint16_t      rde_idx;

    volatile uint16_t *otpbase;       /* Cache OTP Base address */
    uint16_t      avsbitslen;         /* Number of bits used for AVS in sw region */

    uint16_t arr_cache_word;
    uint16_t arr_cache_word_number;

} otpinfo_t;

typedef struct {
    uint8_t tag;
    void* data;
    uint16_t byte_len;
} otp_read_tag_context_t;

static otpinfo_t otpinfo;

static otpinfo_t *
get_otpinfo(void)
{
    return (otpinfo_t *)&otpinfo;
}

static void
otp_initregs(_cyhal_gmac_si_t *sih, void *coreregs, otpregs_t *otpregs)
{
    (void)coreregs;
    if (AOB_ENAB(sih))
    {
        otpregs->otpstatus = (volatile uint32_t *)(PLATFORM_GCI_REGBASE(0x310));
        otpregs->otpcontrol = (volatile uint32_t *)(PLATFORM_GCI_REGBASE(0x314));
        otpregs->otpprog = (volatile uint32_t *)(PLATFORM_GCI_REGBASE(0x318));
        otpregs->otplayout = (volatile uint32_t *)(PLATFORM_GCI_REGBASE(0x31C));
        otpregs->otpcontrol1 = (volatile uint32_t *)(PLATFORM_GCI_REGBASE(0x324));
        otpregs->otplayoutextension = (volatile uint32_t *)(PLATFORM_GCI_REGBASE(0x320));
    }
    else
    {
        otpregs->otpstatus = (volatile uint32_t *)(&PLATFORM_CHIPCOMMON->otp.otp_status);
        otpregs->otpcontrol = (volatile uint32_t *)(&PLATFORM_CHIPCOMMON->otp.otp_ctrl);
        otpregs->otpprog = (volatile uint32_t *)(&PLATFORM_CHIPCOMMON->otp.otp_prog);
        otpregs->otplayout = (volatile uint32_t *)(&PLATFORM_CHIPCOMMON->otp.otp_layout);
        otpregs->otpcontrol1 = (volatile uint32_t *)(&PLATFORM_CHIPCOMMON->otp.otp_ctrl_1);
        otpregs->otplayoutextension = (volatile uint32_t *)(&PLATFORM_CHIPCOMMON->otp.otp_layout_extensions);
    }
}

/*
 * IPX OTP Code
 *
 *   Exported functions:
 *  ipxotp_status()
 *  ipxotp_size()
 *  ipxotp_init()
 *  ipxotp_read_bit()
 *  ipxotp_read_region()
 *  ipxotp_read_word()
 *  ipxotp_dump()
 *  ipxotp_isunified()
 *  ipxotp_avsbitslen()
 *
 */

static otp_fn_t* get_ipxotp_fn(void);

static uint32_t
ipxotp_status(void *oh)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    return oi->status;
}

/** Returns size in bytes */
static int
ipxotp_size(void *oh)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    return (int)oi->wsize * 2;
}

/** Returns if otp is unified */
static bool
ipxotp_isunified(void *oh)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    return oi->buotp;
}

/** Returns number of bits used for avs at the end of sw region */
static uint16_t
ipxotp_avsbitslen(void *oh)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    return oi->avsbitslen;
}

static uint16_t
ipxotp_read_word_direct(void *oh, otpregs_t *otpregs, unsigned int wn)
{
    otpinfo_t *oi;
    uint16_t val;

    (void) otpregs; // Unused

    oi = (otpinfo_t *)oh;

    CY_ASSERT(wn < oi->wsize);

    val = _CYHAL_GMAC_R_REG(&oi->otpbase[wn]);

    return val;
}

static uint16_t
ipxotp_read_bit_direct(void *oh, otpregs_t *otpregs, unsigned int off)
{
    unsigned int word_num, bit_off;
    uint16_t val = 0;
    uint16_t bit = 0;

    word_num = off / OTPWSIZE;
    bit_off = off % OTPWSIZE;

    val = ipxotp_read_word_direct(oh, otpregs, word_num);

    bit = (val >> bit_off) & 0x1;

    return bit;
}

static uint16_t
ipxotp_read_bit_common(void *oh, otpregs_t *otpregs, unsigned int off)
{
    uint16_t bit;
    bit = ipxotp_read_bit_direct(oh, otpregs, off);
    return bit;
}

static uint16_t
ipxotp_read_bit(void *oh, unsigned int off)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    _cyhal_gmac_si_t *sih = oi->sih;
    unsigned int idx;
    void *regs;
    otpregs_t otpregs;
    uint16_t val16;

    idx = _cyhal_gmac_si_coreidx(sih);
    if (AOB_ENAB(sih)) {
        regs = _cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
    } else {
        regs = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    }
    CY_ASSERT(regs != NULL);
    otp_initregs(sih, regs, &otpregs);

    val16 = ipxotp_read_bit_common(oh, &otpregs, off);

    _cyhal_gmac_si_setcoreidx(sih, idx);
    return (val16);
}

/** OTP BT region size */
static void
ipxotp_bt_region_get(otpinfo_t *oi, uint16_t *start, uint16_t *end)
{
    (void)oi;
    *start = *end = 0;
    switch (CHIPID(oi->sih->chip)) {
    case BCM43909_CHIP_ID:
        *start = OTP_BT_BASE_43909;
        *end = OTP_BT_END_43909;
        break;
    }
}

/**
 * Calculate max HW/SW region byte size by subtracting fuse region and checksum size,
 * osizew is oi->wsize (OTP size - GU size) in words.
 */
static int
ipxotp_max_rgnsz(otpinfo_t *oi)
{
    int osizew = oi->wsize;
    int ret = 0;
    uint16_t checksum;
    _cyhal_gmac_si_t *sih = oi->sih;
    unsigned int idx;
    void *regs;
    otpregs_t otpregs;

    idx = _cyhal_gmac_si_coreidx(sih);
    if (AOB_ENAB(sih)) {
        regs = _cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
    } else {
        regs = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    }
    CY_ASSERT(regs != NULL);
    otp_initregs(sih, regs, &otpregs);

    checksum = OTP_SZ_CHECKSUM;

    /* for new chips, fusebit is available from cc register */
    oi->fusebits = _CYHAL_GMAC_R_REG(otpregs.otplayoutextension)
                    & OTPLAYOUTEXT_FUSE_MASK;
    oi->fusebits = ROUNDUP(oi->fusebits, 8);
    oi->fusebits >>= 3; /* bytes */

    _cyhal_gmac_si_setcoreidx(sih, idx);

    switch (CHIPID(sih->chip)) {
    case BCM43909_CHIP_ID:
        oi->fusebits = OTP_SZ_FU_608;
        break;
    default:
        if (oi->fusebits == 0)
            CY_ASSERT(0);  /* Don't know about this chip */
    }

    ret = osizew*2 - oi->fusebits - checksum;

    _CYHAL_ETHERNET_LOG_INFO(("max region size %d bytes\n", ret));
    return ret;
}

/**  OTP sizes for 40nm */
static int
ipxotp_otpsize_set_40nm(otpinfo_t *oi, unsigned int otpsz)
{
    /* Check for otp size */
    switch (otpsz) {
    case 1: /* 64x32: 2048 bits */
        oi->rows = 64;
        oi->cols = 32;
        break;
    case 2: /* 96x32: 3072 bits */
        oi->rows = 96;
        oi->cols = 32;
        break;
    case 3: /* 128x32: 4096 bits */
        oi->rows = 128;
        oi->cols = 32;
        break;
    case 4: /* 160x32: 5120 bits */
        oi->rows = 160;
        oi->cols = 32;
        break;
    case 5: /* 192x32: 6144 bits */
        oi->rows = 192;
        oi->cols = 32;
        break;
    case 7: /* 256x32: 8192 bits */
        oi->rows = 256;
        oi->cols = 32;
        break;
    case 11: /* 384x32: 12288 bits */
        oi->rows = 384;
        oi->cols = 32;
        break;
    default:
        /* Don't know the geometry */
        _CYHAL_ETHERNET_LOG_ERROR(("%s: unknown OTP geometry\n", __FUNCTION__));
    }

    oi->wsize = (oi->cols * oi->rows)/OTPWSIZE;
    return 0;
}

static uint16_t
ipxotp_read_word_common(void *oh, otpregs_t *otpregs, unsigned int wn)
{
    uint16_t word;
    word = ipxotp_read_word_direct(oh, otpregs, wn);
    return word;
}

static int
ipxotp_read_word(void *oh, unsigned int wn, uint16_t *data)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    unsigned int idx;
    void *regs;
    otpregs_t otpregs;
    _cyhal_gmac_si_t *sih = oi->sih;

    idx = _cyhal_gmac_si_coreidx(sih);
    if (AOB_ENAB(sih)) {
        regs = _cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
    } else {
        regs = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    }
    CY_ASSERT(regs != NULL);
    otp_initregs(sih, regs, &otpregs);

    /* Read the data */
    *data = ipxotp_read_word_common(oh, &otpregs, wn);

    _cyhal_gmac_si_setcoreidx(sih, idx);
    return 0;
}

static int
ipxotp_get_region(void *oh, _cyhal_platform_otp_region_t region, unsigned int *wbase, unsigned int *wlen)
{
    otpinfo_t *oi = (otpinfo_t *)oh;

    /* Validate region selection */
    switch (region) {
    case _CYHAL_PLATFORM_OTP_HW_RGN:
        /* OTP unification: For unified OTP sz=flim-hwbase */
        if (oi->buotp)
            *wlen = (uint)oi->flim - oi->hwbase;
        else
            *wlen = (uint)oi->hwlim - oi->hwbase;
        if (!(oi->status & OTPS_GUP_HW)) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: h/w region not programmed\n", __FUNCTION__));
            return BCME_NOTFOUND;
        }
        *wbase = oi->hwbase;
        break;
    case _CYHAL_PLATFORM_OTP_SW_RGN:
        /* OTP unification: For unified OTP sz=flim-swbase */
        if (oi->buotp)
            *wlen = ((uint)oi->flim - oi->swbase);
        else
            *wlen = ((uint)oi->swlim - oi->swbase);
        if (!(oi->status & OTPS_GUP_SW)) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: s/w region not programmed\n", __FUNCTION__));
            return BCME_NOTFOUND;
        }
        *wbase = oi->swbase;
        break;
    case _CYHAL_PLATFORM_OTP_CI_RGN:
        *wlen = OTPGU_CI_SZ;
        if (!(oi->status & OTPS_GUP_CI)) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: chipid region not programmed\n", __FUNCTION__));
            return BCME_NOTFOUND;
        }
        *wbase = oi->otpgu_base + OTPGU_CI_OFF;
        break;
    case _CYHAL_PLATFORM_OTP_FUSE_RGN:
        *wlen = (uint)oi->flim - oi->fbase;
        if (!(oi->status & OTPS_GUP_FUSE)) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: fuse region not programmed\n", __FUNCTION__));
            return BCME_NOTFOUND;
        }
        *wbase = oi->fbase;
        break;
    case _CYHAL_PLATFORM_OTP_ALL_RGN:
        *wlen = ((uint)oi->flim - oi->hwbase);
        if (!(oi->status & (OTPS_GUP_HW | OTPS_GUP_SW))) {
            _CYHAL_ETHERNET_LOG_ERROR(("%s: h/w & s/w region not programmed\n", __FUNCTION__));
            return BCME_NOTFOUND;
        }
        *wbase = oi->hwbase;
        break;
    default:
        _CYHAL_ETHERNET_LOG_ERROR(("%s: reading region %d is not supported\n", __FUNCTION__, region));
        return BCME_BADARG;
    }

    return 0;
}

static int
ipxotp_read_region(void *oh, _cyhal_platform_otp_region_t region, uint16_t *data, unsigned int *wlen)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    unsigned int idx;
    unsigned int wbase = 0, i, sz = 0;
    _cyhal_gmac_si_t *sih = oi->sih;
    void *regs;
    otpregs_t otpregs;
    int res;

    res = ipxotp_get_region(oh, region, &wbase, &sz);
    if (res != 0) {
        *wlen = sz;
        return res;
    }
    if (*wlen < sz) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: buffer too small, should be at least %u\n",
                 __FUNCTION__, sz));
        *wlen = sz;
        return BCME_BUFTOOSHORT;
    }
    *wlen = sz;

    idx = _cyhal_gmac_si_coreidx(sih);
    if (AOB_ENAB(sih)) {
        regs = _cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
    } else {
        regs = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    }
    CY_ASSERT(regs != NULL);
    otp_initregs(sih, regs, &otpregs);

    /* Read the data */
    for (i = 0; i < sz; i ++)
        data[i] = ipxotp_read_word_common(oh, &otpregs, wbase + i);

    _cyhal_gmac_si_setcoreidx(sih, idx);
    *wlen = sz;
    return 0;
}

static void
_ipxotp_init(otpinfo_t *oi, otpregs_t *otpregs)
{
    uint16_t btsz, btbase = 0, btend = 0;

    /* record word offset of General Use Region */
    oi->otpgu_base = (_CYHAL_GMAC_R_REG(otpregs->otplayout) & OTPL_HWRGN_OFF_MASK)
                    >> OTPL_HWRGN_OFF_SHIFT;
    CY_ASSERT((oi->otpgu_base - (OTPGU_SROM_OFF * OTPWSIZE)) > 0);
    oi->otpgu_base >>= 4; /* words */
    oi->otpgu_base -= OTPGU_SROM_OFF;

    /* Read OTP lock bits and subregion programmed indication bits */
    oi->status = _CYHAL_GMAC_R_REG(otpregs->otpstatus);

    /* WAR for PR 65487:
     * OTP status is not updated before power-cycle, so we need
     * to read the subregion programmed bit from OTP directly
     */
    if ((CHIPID(oi->sih->chip) == BCM43909_CHIP_ID) || 0) {
        uint32_t p_bits;
        p_bits = (ipxotp_read_word_common(oi, otpregs, oi->otpgu_base + OTPGU_P_OFF) & OTPGU_P_MSK)
            >> OTPGU_P_SHIFT;
        oi->status |= (p_bits << OTPS_GUP_SHIFT);
    }
    _CYHAL_ETHERNET_LOG_DEBUG(("%s: status 0x%x\n", __FUNCTION__, (unsigned int)oi->status));

    /* OTP unification */
    oi->buotp = FALSE; /* Initialize it to false, until its explicitely set true. */
    if ((oi->status & (OTPS_GUP_HW | OTPS_GUP_SW)) == (OTPS_GUP_HW | OTPS_GUP_SW)) {
        switch (CHIPID(oi->sih->chip)) {
            /* Add cases for supporting chips */
            default:
                _CYHAL_ETHERNET_LOG_ERROR(("chip=0x%x does not support Unified OTP.\n",
                    CHIPID(oi->sih->chip)));
                break;
        }
    }

    /* if AVS is part of s/w region, update how many bits are used for AVS */
    switch (CHIPID(oi->sih->chip)) {
        case BCM43909_CHIP_ID:
            oi->avsbitslen = 0;
            break;
        default:
            oi->avsbitslen = 0;
            break;
    }

    /*
     * h/w region base and fuse region limit are fixed to the top and
     * the bottom of the general use region. Everything else can be flexible.
     */
    oi->hwbase = oi->otpgu_base + OTPGU_SROM_OFF;
    oi->hwlim = oi->wsize;
    oi->flim = oi->wsize;

    ipxotp_bt_region_get(oi, &btbase, &btend);
    btsz = btend - btbase;
    if (btsz > 0) {
        /* default to not exceed BT base */
        oi->hwlim = btbase;

        /* With BT shared region, swlim and fbase are fixed */
        oi->swlim = btbase;
        oi->fbase = btend;
        /* if avs bits are part of swregion, subtract that from the sw/hw limit */
        oi->hwlim -= oi->avsbitslen;
        oi->swlim -= oi->avsbitslen;
    }

    /* Update hwlim and swbase */
    if (oi->status & OTPS_GUP_HW) {
        uint16_t swbase;
        _CYHAL_ETHERNET_LOG_DEBUG(("%s: hw region programmed\n", __FUNCTION__));
        swbase = ipxotp_read_word_common(oi, otpregs, oi->otpgu_base + OTPGU_HSB_OFF) / 16;
        if (swbase) {
            oi->hwlim =  swbase;
        }
        oi->swbase = oi->hwlim;
    } else
        oi->swbase = oi->hwbase;

    /* Update swlim and fbase only if no BT region */
    if (btsz == 0) {
        /* subtract fuse and checksum from beginning */
        oi->swlim = ipxotp_max_rgnsz(oi) / 2;

        if (oi->status & OTPS_GUP_SW) {
            _CYHAL_ETHERNET_LOG_DEBUG(("%s: sw region programmed\n", __FUNCTION__));
            oi->swlim = ipxotp_read_word_common(oi, otpregs, oi->otpgu_base + OTPGU_SFB_OFF) / 16;
            oi->fbase = oi->swlim;
        }
        else
            oi->fbase = oi->swbase;
        /* if avs bits are part of swregion, subtract that from the sw limit */
        oi->swlim -= oi->avsbitslen;
    }

    _CYHAL_ETHERNET_LOG_DEBUG(("%s: OTP limits---\n"
        "hwbase %d/%d hwlim %d/%d\n"
        "swbase %d/%d swlim %d/%d\n"
        "fbase %d/%d flim %d/%d\n", __FUNCTION__,
        oi->hwbase, oi->hwbase * 16, oi->hwlim, oi->hwlim * 16,
        oi->swbase, oi->swbase * 16, oi->swlim, oi->swlim * 16,
        oi->fbase, oi->fbase * 16, oi->flim, oi->flim * 16));
}

static void *
ipxotp_init(_cyhal_gmac_si_t *sih)
{
    unsigned int idx, otpsz, otpwt;
    void *regs;
    otpregs_t otpregs;
    otpinfo_t *oi = NULL;

    _CYHAL_ETHERNET_LOG_INFO(("%s: Use IPX OTP controller\n", __FUNCTION__));

    /* Make sure we're running IPX OTP */
    CY_ASSERT(OTPTYPE_IPX(sih->ccrev));
    if (!OTPTYPE_IPX(sih->ccrev))
        return NULL;

    /* Make sure chipcommon rev >= 49 */
    if (sih->ccrev < 49) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: unsupported OTP revision\n", __FUNCTION__));
        return NULL;
    }

    /* Make sure OTP is not disabled */
    if (_cyhal_gmac_si_is_otp_disabled(sih)) {
        _CYHAL_ETHERNET_LOG_INFO(("%s: OTP is disabled\n", __FUNCTION__));
        return NULL;
    }

    /* Make sure OTP is powered up */
    if (!_cyhal_gmac_si_is_otp_powered(sih)) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP is powered down\n", __FUNCTION__));
        return NULL;
    }

    /* Retrieve OTP region info */
    idx = _cyhal_gmac_si_coreidx(sih);
    if (AOB_ENAB(sih)) {
        regs = _cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
    } else {
        regs = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    }
    CY_ASSERT(regs != NULL);
    otp_initregs(sih, regs, &otpregs);

    oi = get_otpinfo();

    otpsz = (_CYHAL_GMAC_R_REG(otpregs.otplayout) & OTPL_ROW_SIZE_MASK) >> OTPL_ROW_SIZE_SHIFT;

    if (otpsz == 0) {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP\n", __FUNCTION__));
        oi = NULL;
        goto exit;
    }

    otpwt = (_CYHAL_GMAC_R_REG(otpregs.otplayout) & OTPL_WRAP_TYPE_MASK) >> OTPL_WRAP_TYPE_SHIFT;

    if (otpwt == OTPL_WRAP_TYPE_40NM) {
        ipxotp_otpsize_set_40nm(oi, otpsz);
    } else {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported wrap type: %d\n", __FUNCTION__, otpwt));
        oi = NULL;
        goto exit;
    }

    _CYHAL_ETHERNET_LOG_INFO(("%s: rows %u cols %u wsize %u\n", __FUNCTION__, oi->rows, oi->cols, oi->wsize));

    if (AOB_ENAB(sih)) {
        uint32_t otpaddr;
        otpaddr = _CYHAL_PLATFORM_OTP_BASE;
        oi->otpbase = (uint16_t *)REG_MAP(otpaddr, SI_CORE_SIZE);
        _CYHAL_ETHERNET_LOG_INFO(("%s: mapping otpbase at 0x%08x to 0x%p\n", __FUNCTION__, (unsigned int)otpaddr, oi->otpbase));
    } else {
        unsigned int idx2;
        /* Take offset of OTP Base address from GCI CORE */
        idx2 = _cyhal_gmac_si_coreidx(sih);
        oi->otpbase = (uint16_t *)_cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
        /* Reset si handler curidx to CC */
        _cyhal_gmac_si_setcoreidx(sih, idx2);
    }

    _ipxotp_init(oi, &otpregs);

exit:
    _cyhal_gmac_si_setcoreidx(sih, idx);

    return (void *)oi;
}

#ifdef OTP_DEBUG
static void
ipxotp_dump(void *oh)
{
    otpinfo_t *oi = (otpinfo_t *)oh;
    _cyhal_gmac_si_t *sih = oi->sih;
    void *regs;
    otpregs_t otpregs;
    unsigned int idx, i, count;
    uint16_t val;

    idx = _cyhal_gmac_si_coreidx(sih);
    if (AOB_ENAB(sih)) {
        regs = _cyhal_gmac_si_setcore(sih, GCI_CORE_ID, 0);
    } else {
        regs = _cyhal_gmac_si_setcoreidx(sih, SI_CC_IDX);
    }
    otp_initregs(sih, regs, &otpregs);

    count = ipxotp_size(oh);

    for (i = 0; i < count / 2; i++) {
        if (!(i % 4))
            _CYHAL_ETHERNET_LOG_ERROR(("\n0x%04x:", 2 * i));
        val = ipxotp_read_word_common(oi, &otpregs, i);
        _CYHAL_ETHERNET_LOG_ERROR((" 0x%04x", val));
    }
    _CYHAL_ETHERNET_LOG_ERROR(("\n"));

    _cyhal_gmac_si_setcoreidx(oi->sih, idx);
}
#endif /* OTP_DEBUG */

static otp_fn_t ipxotp_fn = {
    (otp_size_t)ipxotp_size,
    (otp_read_bit_t)ipxotp_read_bit,
    (otp_dump_t)NULL,
    (otp_status_t)ipxotp_status,
    (otp_init_t)ipxotp_init,
    (otp_read_region_t)ipxotp_read_region,
    (otp_get_region_t)ipxotp_get_region,
    (otp_nvread_t)NULL,
    (otp_write_region_t)NULL,
    (otp_cis_append_region_t)NULL,
    (otp_lock_t)NULL,
    (otp_nvwrite_t)NULL,
    (otp_write_word_t)NULL,
    (otp_read_word_t)ipxotp_read_word,
    (otp_write_bits_t)NULL,
    (otp_isunified_t)ipxotp_isunified,
    (otp_avsbitslen_t)ipxotp_avsbitslen
};

static otp_fn_t*
get_ipxotp_fn(void)
{
    return &ipxotp_fn;
}

static cy_rslt_t
_cyhal_platform_otp_init_internal(void)
{
    _cyhal_gmac_si_t * sih = NULL;
    otpinfo_t *oi = NULL;
    void *ret = NULL;

    oi = get_otpinfo();

    bzero(oi, sizeof(otpinfo_t));

    /* Get SI handle */
    sih = _cyhal_gmac_si_kattach();
    if (sih == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Cannot get SI handle\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    oi->ccrev = sih->ccrev;

    if (OTPTYPE_IPX(oi->ccrev))
    {
#ifdef OTP_DEBUG
        get_ipxotp_fn()->dump = ipxotp_dump;
#endif /* OTP_DEBUG */
        oi->fn = get_ipxotp_fn();
    }

    if (oi->fn == NULL)
    {
        bzero(oi, sizeof(otpinfo_t));

        _CYHAL_ETHERNET_LOG_ERROR(("%s: unsupported OTP type\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    oi->sih = sih;

    /* 43909 OTP is always powered up */
    if (!_cyhal_gmac_si_is_otp_powered(sih))
    {
        bzero(oi, sizeof(otpinfo_t));

        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not powered on\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    ret = (oi->fn->init)(sih);

    if (ret == NULL)
    {
        bzero(oi, sizeof(otpinfo_t));

        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP initialization error\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t
_cyhal_platform_otp_read_word_internal(uint16_t word_number, uint16_t *read_word)
{
    otpinfo_t *oi;
    int err = 0;

    oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not initialized\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->read_word == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP operation\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    err = (oi->fn->read_word)((void *)oi, word_number, read_word);

    if (err != 0)
    {
        return CYHAL_OTP_RSLT_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t
_cyhal_platform_otp_read_array_internal(uint16_t byte_number, void* data, uint16_t byte_len)
{
    otpinfo_t *oi = get_otpinfo();
    uint8_t *p = data;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    while (byte_len)
    {
        uint16_t word_number = byte_number / 2;
        uint16_t word = oi->arr_cache_word;

        if (oi->arr_cache_word_number != word_number)
        {
            result = _cyhal_platform_otp_read_word_internal(word_number, &word);
            if (result != CY_RSLT_SUCCESS)
            {
                break;
            }

            oi->arr_cache_word = word;
            oi->arr_cache_word_number = word_number;
        }

        if ((byte_number & 0x1) == 0)
        {
            *p++ = (uint8_t)(word & 0xFF);
            byte_number += 1;
            byte_len -= 1;
        }

        if (byte_len)
        {
            *p++ = (uint8_t)((word >> 8) & 0xFF);
            byte_number += 1;
            byte_len -= 1;
        }
    }

    return result;
}

static cy_rslt_t
_cyhal_platform_otp_get_region_internal(_cyhal_platform_otp_region_t region, uint16_t *word_number, uint16_t *word_len)
{
    otpinfo_t *oi;
    unsigned int wnumber = 0;
    unsigned int wlen = 0;
    int err = 0;

    oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not initialized\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->get_region == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP operation\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    err = (oi->fn->get_region)((void *)oi, region, &wnumber, &wlen);
    if (err != 0)
    {
        return CYHAL_OTP_RSLT_ERROR;
    }

    *word_number = wnumber;
    *word_len = wlen;

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t
_cyhal_platform_otp_read_region_internal(_cyhal_platform_otp_region_t region, uint16_t *data, uint16_t *word_len)
{
    otpinfo_t *oi;
    int err = 0;
    unsigned int wlen = *word_len;

    oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not initialized\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->read_region == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP operation\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    err = (oi->fn->read_region)((void *)oi, region, data, &wlen);

    *word_len = wlen;

    if (err == BCME_BUFTOOSHORT)
    {
        return CYHAL_OTP_RSLT_INVALID_ARG;
    }
    else if (err != 0)
    {
        return CYHAL_OTP_RSLT_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

#if 0 // Currently unused & untested
static cy_rslt_t
_cyhal_platform_otp_newcis_internal(uint16_t *newcis_bit)
{
    cy_rslt_t ret = CYHAL_OTP_RSLT_ERROR;

    otpinfo_t *oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not initialized\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->read_bit == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP operation\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    int otpgu_bit_base = oi->otpgu_base * 16;

    ret = _cyhal_platform_otp_read_bit(otpgu_bit_base + OTPGU_NEWCISFORMAT_OFF, newcis_bit);

    _CYHAL_ETHERNET_LOG_INFO(("New Cis format bit %d value: %x\n", otpgu_bit_base + OTPGU_NEWCISFORMAT_OFF, *newcis_bit));

    return ret;
}

static cy_rslt_t
_cyhal_platform_otp_isunified_internal(bool *is_unified)
{
    otpinfo_t *oi = get_otpinfo();
    bool otp_is_unified = FALSE;

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not initialized\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->isunified == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP operation\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    otp_is_unified = oi->fn->isunified((void *)oi);

    if (otp_is_unified == true)
    {
        *is_unified = true;
    }
    else
    {
        *is_unified = false;
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t
_cyhal_platform_otp_avsbitslen_internal(uint16_t *avsbitslen)
{
    otpinfo_t *oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: OTP not initialized\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->avsbitslen == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("%s: Unsupported OTP operation\n", __FUNCTION__));
        return CYHAL_OTP_RSLT_ERROR;
    }

    *avsbitslen = oi->fn->avsbitslen((void *)oi);

    return CY_RSLT_SUCCESS;
}
#endif // unused

static cy_rslt_t
_cyhal_platform_otp_init_unsafe(void)
{
    static bool done = false;
    cy_rslt_t ret = CY_RSLT_SUCCESS;

    if (done != true)
    {
        ret = _cyhal_platform_otp_init_internal();
        if (ret == CY_RSLT_SUCCESS)
        {
            done = true;
        }
    }

    get_otpinfo()->arr_cache_word_number = 1; /* any odd number */

    return ret;
}

/*
 * Common Code:
 *  _cyhal_platform_otp_init()
 *  _cyhal_platform_otp_status()
 *  _cyhal_platform_otp_size()
 *  _cyhal_platform_otp_read_bit()
 *  _cyhal_platform_otp_read_word()
 *  _cyhal_platform_otp_read_region()
 *  _cyhal_platform_otp_newcis()
 *  _cyhal_platform_otp_isunified()
 *  _cyhal_platform_otp_avsbitslen()
 *
 *  _cyhal_platform_otp_dump()
 *  _cyhal_platform_otp_dumpstats()
 */

#ifdef CYW43907_OTP_MUTHITHREADED
static host_semaphore_type_t*
_cyhal_platform_otp_init_sem( void )
{
    static bool done = false;
    static host_semaphore_type_t sem;
    uint32_t flags;

    WICED_SAVE_INTERRUPTS(flags);

    if (done != true)
    {
        host_rtos_init_semaphore(&sem);
        host_rtos_set_semaphore(&sem, false);
        done = true;
    }

    WICED_RESTORE_INTERRUPTS(flags);

    return &sem;
}

#endif

cy_rslt_t
_cyhal_platform_otp_init(void)
{
#ifdef CYW43907_OTP_MUTHITHREADED
    host_semaphore_type_t* sem = _cyhal_platform_otp_init_sem();
#endif  /*CYW43907_OTP_MUTHITHREADED */
    cy_rslt_t ret = CY_RSLT_SUCCESS;

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_get_semaphore(sem, _CYHAL_TIMER_MAX_HW_TICKS, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    ret = _cyhal_platform_otp_init_unsafe();

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_set_semaphore(sem, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    return ret;
}


cy_rslt_t
_cyhal_platform_otp_read_array(uint16_t byte_number, void* data, uint16_t byte_len)
{
#ifdef CYW43907_OTP_MUTHITHREADED
    host_semaphore_type_t* sem = _cyhal_platform_otp_init_sem();
#endif  /*CYW43907_OTP_MUTHITHREADED */
    cy_rslt_t ret;

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_get_semaphore(sem, _CYHAL_TIMER_MAX_HW_TICKS, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    ret = _cyhal_platform_otp_read_array_internal(byte_number, data, byte_len);

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_set_semaphore(sem, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    return ret;
}

cy_rslt_t
_cyhal_platform_otp_get_region(_cyhal_platform_otp_region_t region, uint16_t *word_number, uint16_t *word_len)
{
#ifdef CYW43907_OTP_MUTHITHREADED
    host_semaphore_type_t* sem = _cyhal_platform_otp_init_sem();
#endif  /*CYW43907_OTP_MUTHITHREADED */
    cy_rslt_t ret;

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_get_semaphore(sem, _CYHAL_TIMER_MAX_HW_TICKS, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    ret = _cyhal_platform_otp_get_region_internal(region, word_number, word_len);

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_set_semaphore(sem, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    return ret;
}

cy_rslt_t
_cyhal_platform_otp_read_region(_cyhal_platform_otp_region_t region, uint16_t *data, uint16_t *word_len)
{
#ifdef CYW43907_OTP_MUTHITHREADED
    host_semaphore_type_t* sem = _cyhal_platform_otp_init_sem();
#endif  /*CYW43907_OTP_MUTHITHREADED */
    cy_rslt_t ret;

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_get_semaphore(sem, _CYHAL_TIMER_MAX_HW_TICKS, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    ret = _cyhal_platform_otp_read_region_internal(region, data, word_len);

#ifdef CYW43907_OTP_MUTHITHREADED
    host_rtos_set_semaphore(sem, false);
#endif  /*CYW43907_OTP_MUTHITHREADED */

    return ret;
}

cy_rslt_t
_cyhal_platform_otp_cis_parse(_cyhal_platform_otp_region_t region, _cyhal_platform_otp_cis_parse_callback_func callback, void *context)
{
    uint16_t word_number;
    uint16_t word_len;
    uint16_t byte_number;
    uint16_t byte_len;
    cy_rslt_t result;

    result = _cyhal_platform_otp_get_region(region, &word_number, &word_len);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    byte_number = 2 * word_number;
    byte_len = 2 * word_len;

    if (region == _CYHAL_PLATFORM_OTP_HW_RGN)
    {
        if (byte_len < OTP_HW_REGION_SDIO_HW_HEADER_SIZE)
        {
            return CYHAL_OTP_RSLT_ERROR;
        }
        byte_number += OTP_HW_REGION_SDIO_HW_HEADER_SIZE;
        byte_len -= OTP_HW_REGION_SDIO_HW_HEADER_SIZE;
    }

    while (byte_len)
    {
        uint8_t tag;
        uint8_t tag_len;
        uint8_t brcm_tag = 0;

        result = _cyhal_platform_otp_read_array(byte_number, &tag, 1);
        if (result != CY_RSLT_SUCCESS)
        {
            return result;
        }
        byte_number++;
        byte_len--;

        if (tag == CISTPL_NULL)
        {
            continue;
        }

        if (tag == CISTPL_END)
        {
            break;
        }

        if (byte_len == 0)
        {
            return CYHAL_OTP_RSLT_ERROR;
        }
        result = _cyhal_platform_otp_read_array(byte_number, &tag_len, 1);
        if (result != CY_RSLT_SUCCESS)
        {
            return result;
        }
        byte_number++;
        byte_len--;

        if (tag == CISTPL_BRCM_HNBU)
        {
            if ((byte_len == 0) || (tag_len == 0))
            {
                return CYHAL_OTP_RSLT_ERROR;
            }
            result = _cyhal_platform_otp_read_array(byte_number, &brcm_tag, 1);
            if (result != CY_RSLT_SUCCESS)
            {
                return result;
            }
            byte_number++;
            byte_len--;
            tag_len--;
        }

        if (tag_len > byte_len)
        {
            return CYHAL_OTP_RSLT_ERROR;
        }

        result = (*callback)(context, tag, brcm_tag, byte_number, tag_len);
        if (result != CYHAL_OTP_RSLT_PARTIAL_RESULTS)
        {
            return result;
        }

        byte_number += tag_len;
        byte_len -= tag_len;
    }

    return CYHAL_OTP_RSLT_PARTIAL_RESULTS;
}

static cy_rslt_t
_cyhal_platform_otp_read_tag_callback( void* context, uint8_t tag, uint8_t brcm_tag, uint16_t offset, uint8_t size )
{
    otp_read_tag_context_t *parse_context = context;
    uint16_t byte_len = parse_context->byte_len;

    if ( ( tag == CISTPL_BRCM_HNBU ) && ( brcm_tag == parse_context->tag ) )
    {
        parse_context->byte_len = size;

        if ( size > byte_len )
        {
            return CYHAL_OTP_RSLT_INVALID_ARG;
        }

        if ( _cyhal_platform_otp_read_array( offset, parse_context->data, size ) != CY_RSLT_SUCCESS )
        {
            return CYHAL_OTP_RSLT_ERROR;
        }

        return CY_RSLT_SUCCESS;
    }

    return CYHAL_OTP_RSLT_PARTIAL_RESULTS;
}

cy_rslt_t
_cyhal_platform_otp_read_tag(_cyhal_platform_otp_region_t region, uint8_t tag, void *data, uint16_t *byte_len)
{
    otp_read_tag_context_t context = { .tag = tag, .data = data, .byte_len = *byte_len };
    cy_rslt_t result;

    result = _cyhal_platform_otp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    result = _cyhal_platform_otp_cis_parse(region, _cyhal_platform_otp_read_tag_callback, &context);
    if (result == CY_RSLT_SUCCESS)
    {
        *byte_len = context.byte_len;
    }
    else
    {
        *byte_len = 0;
    }
    return result;
}

#if 0 // Currently unused & untested
cy_rslt_t
_cyhal_platform_otp_package_options( uint32_t *package_options )
{
    uint16_t secure_bit = 0;

    _cyhal_platform_otp_read_bit_unprotected(_CYHAL_PLATFORM_OTP_SECURE_BIT_OFFSET, &secure_bit);

    /* Extract package options bits[2:0] from ChipCommon ChipID register */
    *package_options = (uint32_t)(PLATFORM_CHIPCOMMON->core_ctrl_status.chip_id.bits.package_option) & 0x7;

    *package_options |= (uint32_t)(secure_bit << 3);

    return CY_RSLT_SUCCESS;
}

/* Small Memory-Footprint function for OTP Read */
cy_rslt_t _cyhal_platform_otp_read_word_unprotected(uint32_t word_number, uint16_t *read_word)
{
    volatile uint16_t *otpbase;

    otpbase = (uint16_t*)_CYHAL_PLATFORM_OTP_BASE;

    *read_word = _CYHAL_GMAC_R_REG(&otpbase[word_number]);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t _cyhal_platform_otp_read_bit_unprotected(uint16_t bit_num, uint16_t *read_bit)
{
    unsigned int word_num, bit_off;
    uint16_t word_val = 0;

    word_num = bit_num / OTPWSIZE;
    bit_off  = bit_num % OTPWSIZE;

    _cyhal_platform_otp_read_word_unprotected(word_num, &word_val);

    *read_bit = (word_val >> bit_off) & 0x1;

    return CY_RSLT_SUCCESS;
}
#endif

#ifdef OTP_DEBUG
cy_rslt_t
_cyhal_platform_otp_dump(void)
{
    otpinfo_t *oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("_cyhal_platform_otp_dump: OTP not initialized\n"));
        return CYHAL_OTP_RSLT_ERROR;
    }

    if (oi->fn->dump == NULL)
    {
        _CYHAL_ETHERNET_LOG_ERROR(("_cyhal_platform_otp_dump: Unsupported OTP operation\n"));
        return CYHAL_OTP_RSLT_ERROR;
    }

    oi->fn->dump((void *)oi);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t
_cyhal_platform_otp_dumpstats(void)
{
    otpinfo_t *oi = get_otpinfo();

    if ((oi->sih == NULL) || (oi->fn == NULL))
    {
        _CYHAL_ETHERNET_LOG_ERROR(("_cyhal_platform_otp_dumpstats: OTP not initialized\n"));
        return CYHAL_OTP_RSLT_ERROR;
    }

    _CYHAL_ETHERNET_LOG_ERROR(("\nOTP, ccrev 0x%04x\n", oi->ccrev));
    _CYHAL_ETHERNET_LOG_ERROR(("wsize %d rows %d cols %d\n", oi->wsize, oi->rows, oi->cols));
    _CYHAL_ETHERNET_LOG_ERROR(("hwbase %d hwlim %d swbase %d swlim %d fusebase %d fuselim %d fusebits %d\n",
        oi->hwbase, oi->hwlim, oi->swbase, oi->swlim, oi->fbase, oi->flim, oi->fusebits));
    _CYHAL_ETHERNET_LOG_ERROR(("otpgu_base %d status %lu\n", oi->otpgu_base, oi->status));
    _CYHAL_ETHERNET_LOG_ERROR(("\n"));

    return CY_RSLT_SUCCESS;
}
#endif /* OTP_DEBUG */
