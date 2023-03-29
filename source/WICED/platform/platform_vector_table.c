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
 * BCM43909 vector table
 */

#include "cr4.h"
#include "platform_toolchain.h"
#include "platform_isr.h"
#include "platform_appscr4.h"
#include "platform_mcu_peripheral.h"

#include "typedefs.h"
#include "sbchipc.h"
#include "aidmp.h"


/******************************************************
 *               platform_unhandled_isr.c
 ******************************************************/

PLATFORM_DEFINE_ISR( Unhandled_ISR )
{
#ifdef DEBUG
    __BKPT(0);
    while( 1 ){}
#endif /* DEBUG */
}

PLATFORM_SET_DEFAULT_ISR( ChipCommon_ISR   , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( Timer_ISR        , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( Sw0_ISR          , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( Sw1_ISR          , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( GMAC_ISR         , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( Serror_ISR       , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( I2S0_ISR         , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( I2S1_ISR         , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( USB_HOST_ISR     , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( SDIO_HOST_ISR    , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( M2M_ISR          , Unhandled_ISR )
PLATFORM_SET_DEFAULT_ISR( GSPI_SLAVE_ISR   , Unhandled_ISR )


/******************************************************
 *                      Macros
 ******************************************************/

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

typedef struct
{
    ExtIRQn_Type irqn;
    void (*isr)( void );
} interrupt_vector_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

interrupt_vector_t interrupt_vector_table[] =
{
    {
        .irqn  = SW0_ExtIRQn,
        .isr   = Sw0_ISR,
    },
    {
        .irqn  = SW1_ExtIRQn,
        .isr   = Sw1_ISR,
    },
    {
        .irqn  = Timer_ExtIRQn,
        .isr   = Timer_ISR,
    },
    {
        .irqn  = SDIO_REMAPPED_ExtIRQn,
#ifdef GSPI_SLAVE_ENABLE
        .isr   = GSPI_SLAVE_ISR,
#else
        .isr   = SDIO_HOST_ISR,
#endif
    },
    {
        .irqn  = USB_REMAPPED_ExtIRQn,
        .isr   = USB_HOST_ISR,
    },
    {
        .irqn  = GMAC_ExtIRQn,
        .isr   = GMAC_ISR,
    },
    {
        .irqn  = Serror_ExtIRQn,
        .isr   = Serror_ISR,
    },
    {
        .irqn  = M2M_ExtIRQn,
        .isr   = M2M_ISR,
    },
    {
        .irqn  = I2S0_ExtIRQn,
        .isr   = I2S0_ISR,
    },
    {
        .irqn  = I2S1_ExtIRQn,
        .isr   = I2S1_ISR,
    },
    {
        .irqn  = ChipCommon_ExtIRQn,
        .isr   = ChipCommon_ISR,
    },
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_irq_demuxer (void)
{
    uint32_t mask = PLATFORM_APPSCR4->irq_mask;
    uint32_t status = mask & PLATFORM_APPSCR4->fiqirq_status;

    if ( status )
    {
        unsigned i;

        PLATFORM_APPSCR4->fiqirq_status = status;

        for ( i = 0; status && (i < ARRAYSIZE(interrupt_vector_table)); ++i )
        {
            uint32_t irqn_mask = IRQN2MASK(interrupt_vector_table[i].irqn);

            if ( status & irqn_mask )
            {
                interrupt_vector_table[i].isr();
                status &= ~irqn_mask;
            }
        }
    }

    if ( status )
    {
        Unhandled_ISR();
    }

    cpu_data_synchronisation_barrier( );
}

/* 
 * Note: In bare-metal mode, the "interrupt" attribute is needed to tell the compiler to
 * generate the proper entry and exit sequences for handling interrupts. In RTOS, this is
 * handled differently and the "interrupt" attribute must be removed. The trampoline
 * performed here is to allow the RTOS to use the platform_irq_demuxer() function as is.
 * The RTOS is expected to provide the strong implementations for, 
 *  1. irq_vector_external_interrupt()
 *  2. platform_irq_demuxer_default()
 * The RTOS is also expected to provide the ISR calling the platform_irq_demuxer() function:
 *  1. PLATFORM_DEFINE_ISR(platform_irq_demuxer_wrapper);
 *  2. PLATFORM_MAP_ISR(platform_irq_demuxer_wrapper, vApplicationIRQHandler);
 */
PLATFORM_SET_DEFAULT_ISR_DEMUXER( platform_irq_demuxer_default )
{
    platform_irq_demuxer();
}

PLATFORM_SET_DEFAULT_ISR(irq_vector_external_interrupt, platform_irq_demuxer_default);

