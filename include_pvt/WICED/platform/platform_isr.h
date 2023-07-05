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
 * Defines macros for defining and mapping interrupt handlers to ARM-Cortex-R4 CPU interrupts
 */
#pragma once

#include "platform_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/* Section where IRQ handlers are placed */
#define IRQ_SECTION ".text.irq"

/* Macro for defining an interrupt handler (non-RTOS-aware)
 *
 * @warning:
 * Do NOT call any RTOS primitive functions from here. If you need to call them,
 * define your interrupt handler using WWD_RTOS_DEFINE_ISR()
 *
 * @usage:
 * PLATFORM_DEFINE_ISR( my_irq )
 * {
 *     // Do something here
 * }
 *
 */
#if defined ( __GNUC__ )
/* GCC */
#define PLATFORM_DEFINE_ISR( name ) \
        void name( void ); \
        __attribute__(( section( IRQ_SECTION ) )) void name( void )

#elif defined ( __IAR_SYSTEMS_ICC__ )
/* IAR Systems */
#define PLATFORM_DEFINE_ISR( name ) \
        __root void name( void ); \
        __root void name( void )

#else

#define PLATFORM_DEFINE_ISR( name )

#endif


/* Macro for mapping a defined function to an interrupt handler
 *
 * @usage:
 * PLATFORM_MAP_ISR( my_irq, USART1_irq )
 */
#if defined( __GNUC__ )

#define PLATFORM_MAP_ISR( function, irq_handler ) \
        extern void irq_handler( void ); \
        __attribute__(( alias( #function ))) void irq_handler ( void );

#elif defined ( __IAR_SYSTEMS_ICC__ )

#define PLATFORM_MAP_ISR( function, irq_handler ) \
        extern void irq_handler( void ); \
        _Pragma( TO_STRING( weak irq_handler=function ) )

#else

#define PLATFORM_MAP_ISR( function, irq_handler )

#endif


/* Macro for declaring a default handler for an unhandled interrupt
 *
 * @usage:
 * PLATFORM_SET_DEFAULT_ISR( USART1_irq, default_handler )
 */
#if defined( __GNUC__ )

#define PLATFORM_SET_DEFAULT_ISR( irq_handler, default_handler ) \
        __attribute__(( weak, alias( #default_handler ))) void irq_handler ( void );

#elif defined ( __IAR_SYSTEMS_ICC__ )

#define PLATFORM_SET_DEFAULT_ISR( irq_handler, default_handler ) \
        _Pragma( TO_STRING( weak irq_handler=default_handler ) )

#else

#define PLATFORM_SET_DEFAULT_ISR( irq_handler, default_handler )

#endif


/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               platform_isr_interface.h
 ******************************************************/

#define IRQN2MASK(n) ((uint32_t)1 << (n))

extern void ChipCommon_ISR      ( void ); // ChipCommon core
extern void Timer_ISR           ( void ); // Timer (system ticks)
extern void Sw0_ISR             ( void ); // software triggered (1)
extern void Sw1_ISR             ( void ); // software triggered (2)
extern void GMAC_ISR            ( void ); // GMAC (ethernet)
extern void Serror_ISR          ( void ); // bus error
extern void M2M_ISR             ( void ); // Memory to Memory DMA
extern void I2S0_ISR            ( void ); // I2S0
extern void I2S1_ISR            ( void ); // I2S1
extern void USB_HOST_ISR        ( void ); // USB HOST
extern void SDIO_HOST_ISR       ( void ); // SDIO HOST
extern void GSPI_SLAVE_ISR      ( void ); // GSPI slave

extern void Unhandled_ISR       ( void ); // Stub called when no handler defined


/******************************************************
 *               wwd_rtos_isr.h
 ******************************************************/

// Function to demux the irq_vector_external_interrupt into specific interrupts below
extern void platform_irq_demuxer (void);

#if defined( __GNUC__ )

#define PLATFORM_SET_DEFAULT_ISR_DEMUXER( function ) \
        void function( void ); \
        __attribute__(( weak, interrupt, used, section(IRQ_SECTION) )) void function( void )

#elif defined ( __IAR_SYSTEMS_ICC__ )

#define PLATFORM_SET_DEFAULT_ISR_DEMUXER( function ) \
        __weak __irq __root void function( void ); \
        __weak __irq __root void function( void )
#else

#define PLATFORM_SET_DEFAULT_ISR_DEMUXER( function ) \
        void function( void )

#endif

#ifdef __cplusplus
} /*extern "C" */
#endif
