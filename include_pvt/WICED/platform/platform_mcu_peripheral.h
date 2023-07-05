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
 * Defines BCM43909 common peripheral structures, macros, constants and declares BCM43909 peripheral API
 */
#pragma once

#include "platform_appscr4.h"
#include "platform_toolchain.h"
#include "platform_config.h"

#include "ring_buffer.h"
#include "cy_utils.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define PLATFORM_DDR_FUNCNAME(type)                     platform_ddr_init_##type
#define PLATFORM_DDR_FUNCDECL(type)                     platform_result_t PLATFORM_DDR_FUNCNAME(type)( void )
#define PLATFORM_DDR_STR_EXPAND( name )                 #name
#if PLATFORM_NO_DDR
    #define PLATFORM_DDR_FUNCCALL(type)
    //
    #define PLATFORM_DDR_BSS_SIZE                       (0x0)
    #define PLATFORM_DDR_BSS_SECTION(var)               var
    //
    #define PLATFORM_DDR_FREE_OFFSET                    (0x0)
#else
    #define PLATFORM_DDR_FUNCCALL(type)                 do {PLATFORM_DDR_FUNCDECL(type); PLATFORM_DDR_FUNCNAME(type)();} while (0)
    //
    #define PLATFORM_DDR_BSS_SIZE                       ( (unsigned long)&link_ddr_bss_end - (unsigned long)&link_ddr_bss_location )
    #define PLATFORM_DDR_BSS_SECTION_NAME( name )       ".ddr_bss."PLATFORM_DDR_STR_EXPAND( name )
    #define PLATFORM_DDR_BSS_SECTION( var )             SECTION( PLATFORM_DDR_BSS_SECTION_NAME( var ) ) var
    //
    #if PLATFORM_DDR_HEAP_SIZE_CONFIG
        #define PLATFORM_DDR_HEAP_SIZE                  ( (unsigned long)&link_ddr_heap_end - (unsigned long)&link_ddr_heap_location )
        #define PLATFORM_DDR_HEAP_SECTION_NAME( name )  ".ddr_heap."PLATFORM_DDR_STR_EXPAND( name )
        #define PLATFORM_DDR_HEAP_SECTION( var )        SECTION( PLATFORM_DDR_HEAP_SECTION_NAME( var ) ) var
    #endif
    //
    #if PLATFORM_DDR_CODE_AND_DATA_ENABLE
        #define PLATFORM_DDR_TEXT_SECTION_NAME( name )  ".ddr_text."PLATFORM_DDR_STR_EXPAND( name )
        #define PLATFORM_DDR_TEXT_SECTION( var )        SECTION( PLATFORM_DDR_TEXT_SECTION_NAME( var ) ) var
        #define PLATFORM_DDR_DATA_SECTION_NAME( name )  ".ddr_data."PLATFORM_DDR_STR_EXPAND( name )
        #define PLATFORM_DDR_DATA_SECTION( var )        SECTION( PLATFORM_DDR_DATA_SECTION_NAME( var ) ) var
    #endif
    #define PLATFORM_DDR_FREE_OFFSET                    ( (unsigned long)&link_ddr_free_location - PLATFORM_DDR_BASE(0x0) )
#endif
#ifndef PLATFORM_DDR_HEAP_SIZE
#define PLATFORM_DDR_HEAP_SIZE                          (0x0)
#endif
#ifndef PLATFORM_DDR_HEAP_SECTION
#define PLATFORM_DDR_HEAP_SECTION( var )
#endif
#ifndef PLATFORM_DDR_TEXT_SECTION
#define PLATFORM_DDR_TEXT_SECTION( var )                var
#endif
#ifndef PLATFORM_DDR_DATA_SECTION
#define PLATFORM_DDR_DATA_SECTION( var )                var
#endif


#define PLATFORM_DMA_DESCRIPTORS_STR_EXPAND( name )     #name
#define PLATFORM_DMA_DESCRIPTORS_SECTION_NAME( name )   ".dma."PLATFORM_DMA_DESCRIPTORS_STR_EXPAND( name )
#define PLATFORM_DMA_DESCRIPTORS_SECTION( var )         SECTION( PLATFORM_DMA_DESCRIPTORS_SECTION_NAME( var ) ) var

#define PLATFORM_CAPABILITY_ENAB(capability)            ((platform_capabilities_word & capability) != 0)
#define PLATFORM_FEATURE_ENAB(_FEATURE_)                (PLATFORM_CAPABILITY_ENAB(PLATFORM_CAPS_##_FEATURE_) && !(PLATFORM_NO_##_FEATURE_))

#if PLATFORM_WLAN_POWERSAVE
#define PLATFORM_WLAN_POWERSAVE_SET_DELAYED_RELEASE_MS(ms)     platform_wlan_powersave_set_delayed_release_milliseconds(ms)
#define PLATFORM_WLAN_POWERSAVE_RES_UP()                       platform_wlan_powersave_res_up()
#define PLATFORM_WLAN_POWERSAVE_RES_DOWN( check_ready, force ) platform_wlan_powersave_res_down( check_ready, force )
#define PLATFORM_WLAN_POWERSAVE_IS_RES_UP()                    platform_wlan_powersave_is_res_up()
#else
#define PLATFORM_WLAN_POWERSAVE_SET_DELAYED_RELEASE_MS(ms)
#define PLATFORM_WLAN_POWERSAVE_RES_UP()
#define PLATFORM_WLAN_POWERSAVE_RES_DOWN( check_ready, force )
#define PLATFORM_WLAN_POWERSAVE_IS_RES_UP()                    1
#endif

#if PLATFORM_WLAN_POWERSAVE && PLATFORM_WLAN_POWERSAVE_STATS
#define PLATFORM_WLAN_POWERSAVE_GET_STATS( counter)            platform_wlan_powersave_get_stats( counter )
#else
#define PLATFORM_WLAN_POWERSAVE_GET_STATS( counter)            ( (uint32_t)0 )
#endif

#if PLATFORM_ALP_CLOCK_RES_FIXUP
#define PLATFORM_ALP_CLOCK_RES_UP()                            PLATFORM_WLAN_POWERSAVE_RES_UP()
#define PLATFORM_ALP_CLOCK_RES_DOWN( check_ready, force )      PLATFORM_WLAN_POWERSAVE_RES_DOWN( check_ready, force )
#else
#define PLATFORM_ALP_CLOCK_RES_UP()
#define PLATFORM_ALP_CLOCK_RES_DOWN( check_ready, force )
#endif

#if PLATFORM_USB_ALP_CLOCK_RES_FIXUP
#define PLATFORM_USB_ALP_CLOCK_RES_UP()                        PLATFORM_WLAN_POWERSAVE_RES_UP()
#define PLATFORM_USB_ALP_CLOCK_RES_DOWN( check_ready, force )  PLATFORM_WLAN_POWERSAVE_RES_DOWN( check_ready, force )
#else
#define PLATFORM_USB_ALP_CLOCK_RES_UP()
#define PLATFORM_USB_ALP_CLOCK_RES_DOWN( check_ready, force )
#endif

#ifdef DEBUG
#define PLATFORM_TIMEOUT_BEGIN( start_var_name ) \
    const uint32_t start_var_name = platform_tick_get_time( PLATFORM_TICK_GET_SLOW_TIME_STAMP ); REFERENCE_DEBUG_ONLY_VARIABLE( start_var_name );
#define PLATFORM_TIMEOUT_SEC_ASSERT( assert_string, start_var_name, good_cond, seconds ) \
    CY_ASSERT( ( platform_tick_get_time( PLATFORM_TICK_GET_SLOW_TIME_STAMP ) - ( start_var_name ) < platform_reference_clock_get_freq( PLATFORM_REFERENCE_CLOCK_ILP ) * ( seconds ) ) || ( good_cond ) );
#else
#define PLATFORM_TIMEOUT_BEGIN( start_var_name)
#define PLATFORM_TIMEOUT_SEC_ASSERT( assert_string, start_var_name, good_cond, seconds )
#endif

#ifdef DEBUG
#define PLATFORM_EXTERNAL_HEAP_FILENAME                 __FILE__
#define PLATFORM_EXTERNAL_HEAP_LINE                     __LINE__
#else
#define PLATFORM_EXTERNAL_HEAP_FILENAME                 NULL
#define PLATFORM_EXTERNAL_HEAP_LINE                     0
#endif

#define  platform_heap_malloc( heap, bytes )            platform_heap_malloc_record_caller( heap, bytes, PLATFORM_EXTERNAL_HEAP_FILENAME, PLATFORM_EXTERNAL_HEAP_LINE )
#define  platform_heap_memalign( heap, align, bytes )   platform_heap_memalign_record_caller( heap, align, bytes, PLATFORM_EXTERNAL_HEAP_FILENAME, PLATFORM_EXTERNAL_HEAP_LINE )
#define  platform_heap_realloc( heap, ptr, size )       platform_heap_realloc_record_caller( heap, ptr, size, PLATFORM_EXTERNAL_HEAP_FILENAME, PLATFORM_EXTERNAL_HEAP_LINE )
#define  platform_heap_free( heap, ptr )                platform_heap_free_record_caller( heap, ptr, PLATFORM_EXTERNAL_HEAP_FILENAME, PLATFORM_EXTERNAL_HEAP_LINE )

/******************************************************
 *                    Constants
 ******************************************************/

/* Default STDIO buffer size */
#ifndef STDIO_BUFFER_SIZE
#define STDIO_BUFFER_SIZE    (64)
#endif

#define UART_CONSOLE_MASK    (0x01)

/* BCM4390x Platform Common Capabilities */
#define PLATFORM_CAPS_COMMON (PLATFORM_CAPS_I2C | PLATFORM_CAPS_UART | PLATFORM_CAPS_SPI)

#define GPIO_TOTAL_PIN_NUMBERS          (32)
#define PIN_FUNCTION_MAX_COUNT          (12)
#define PIN_FUNCTION_UNSUPPORTED        (-1)

extern uint32_t platform_capabilities_word;

extern void*    link_ddr_bss_location;
extern void*    link_ddr_bss_end;
extern void*    link_ddr_heap_location;
extern void*    link_ddr_heap_end;
extern void*    link_ddr_free_location;

/* TODO: Revisit if external DDR dupport is needed */
// extern const wiced_block_device_driver_t ddr_block_device_driver;

#ifdef __cplusplus
} /* extern "C" */
#endif
