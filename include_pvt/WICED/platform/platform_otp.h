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
 * Defines BCM43909 One Time Programming access
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/* OTP regions */
typedef enum
{
    _CYHAL_PLATFORM_OTP_HW_RGN   = 1,
    _CYHAL_PLATFORM_OTP_SW_RGN   = 2,
    _CYHAL_PLATFORM_OTP_CI_RGN   = 4,
    _CYHAL_PLATFORM_OTP_FUSE_RGN = 8,
    _CYHAL_PLATFORM_OTP_ALL_RGN  = 0xf /* From h/w region to end of OTP including checksum */
} _cyhal_platform_otp_region_t;

#define CYHAL_OTP_RSLT_ERROR \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_OTP,  0))
#define CYHAL_OTP_RSLT_INVALID_ARG \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_OTP,  1))
#define CYHAL_OTP_RSLT_PARTIAL_RESULTS \
    (CY_RSLT_CREATE_EX(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ABSTRACTION_HAL, CYHAL_RSLT_MODULE_OTP,  2))

typedef cy_rslt_t (*_cyhal_platform_otp_cis_parse_callback_func)( void* context, uint8_t tag, uint8_t brcm_tag, uint16_t offset, uint8_t size );

/* Functions related to platform OTP driver */
cy_rslt_t _cyhal_platform_otp_init             ( void );
cy_rslt_t _cyhal_platform_otp_status           ( uint32_t *status );
cy_rslt_t _cyhal_platform_otp_size             ( uint16_t *size );
cy_rslt_t _cyhal_platform_otp_read_bit         ( uint16_t bit_number, uint16_t *read_bit );
cy_rslt_t _cyhal_platform_otp_read_word        ( uint16_t word_number, uint16_t *read_word );
cy_rslt_t _cyhal_platform_otp_read_array       ( uint16_t byte_number, void* data, uint16_t byte_len );
cy_rslt_t _cyhal_platform_otp_get_region       ( _cyhal_platform_otp_region_t region, uint16_t *word_number, uint16_t *word_len );
cy_rslt_t _cyhal_platform_otp_read_region      ( _cyhal_platform_otp_region_t region, uint16_t *data, uint16_t *word_len );
cy_rslt_t _cyhal_platform_otp_newcis           ( uint16_t *newcis_bit );
cy_rslt_t _cyhal_platform_otp_isunified        ( bool *is_unified );
cy_rslt_t _cyhal_platform_otp_avsbitslen       ( uint16_t *avsbitslen );
cy_rslt_t _cyhal_platform_otp_cis_parse        ( _cyhal_platform_otp_region_t region, _cyhal_platform_otp_cis_parse_callback_func callback, void *context );
cy_rslt_t _cyhal_platform_otp_read_tag         ( _cyhal_platform_otp_region_t region, uint8_t tag, void *data, uint16_t *byte_len );
cy_rslt_t _cyhal_platform_otp_package_options  ( uint32_t *package_options );
cy_rslt_t _cyhal_platform_otp_read_word_unprotected   (uint32_t word_number, uint16_t *read_word);
cy_rslt_t _cyhal_platform_otp_read_bit_unprotected    (uint16_t bit_num, uint16_t *read_bit);

#ifdef OTP_DEBUG
cy_rslt_t _cyhal_platform_otp_dump             ( void );
cy_rslt_t _cyhal_platform_otp_dumpstats        ( void );
#endif


#ifdef __cplusplus
} /* extern "C" */
#endif
