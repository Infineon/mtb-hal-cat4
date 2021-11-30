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
#include <stdint.h>
#include "cr4.h"
#include "platform_isr.h"
#include "platform_toolchain.h"
#include "cy_utils.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define CR4_FAULT_STATUS_MASK  (0x140f)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    CR4_FAULT_STATUS_ALIGNMENT                             = 0x001,   /* Highest Priority */
    CR4_FAULT_STATUS_MPU_BACKGROUND                        = 0x000,
    CR4_FAULT_STATUS_MPU_PERMISSION                        = 0x00D,
    CR4_FAULT_STATUS_SYNC_EXTERNAL_ABORT_AXI_DECODE_ERROR  = 0x008,
    CR4_FAULT_STATUS_ASYNC_EXTERNAL_ABORT_AXI_DECODE_ERROR = 0x406,
    CR4_FAULT_STATUS_SYNC_EXTERNAL_ABORT_AXI_SLAVE_ERROR   = 0x1008,
    CR4_FAULT_STATUS_ASYNC_EXTERNAL_ABORT_AXI_SLAVE_ERROR  = 0x1406,
    CR4_FAULT_STATUS_SYNC_PARITY_ECC                       = 0x409,
    CR4_FAULT_STATUS_ASYNC_PARITY_ECC                      = 0x408,
    CR4_FAULT_STATUS_DEBUG_EVENT                           = 0x002,   /* Lowest Priority */
} cr4_fault_status_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#ifndef DEBUG

static void UnhandledException( void );

PLATFORM_SET_DEFAULT_ISR( irq_vector_undefined_instruction, UnhandledException )
PLATFORM_SET_DEFAULT_ISR( irq_vector_prefetch_abort,        UnhandledException )
PLATFORM_SET_DEFAULT_ISR( irq_vector_data_abort,            UnhandledException )
PLATFORM_SET_DEFAULT_ISR( irq_vector_fast_interrupt,        UnhandledException )
PLATFORM_SET_DEFAULT_ISR( irq_vector_software_interrupt,    UnhandledException )
PLATFORM_SET_DEFAULT_ISR( irq_vector_reserved,              UnhandledException )

static void UnhandledException( void )
{
    while(1){};
}

#else /* DEBUG */

static volatile bool breakpoint_inside_exception = false;

static void default_fast_interrupt_handler( void );
static void default_software_interrupt_handler( void );
static void default_reserved_interrupt_handler( void );
static void prefetch_abort_handler( void );
static void data_abort_handler( void );
static void undefined_instruction_handler( void );

PLATFORM_SET_DEFAULT_ISR( irq_vector_undefined_instruction, undefined_instruction_handler )
PLATFORM_SET_DEFAULT_ISR( irq_vector_prefetch_abort,        prefetch_abort_handler )
PLATFORM_SET_DEFAULT_ISR( irq_vector_data_abort,            data_abort_handler )
PLATFORM_SET_DEFAULT_ISR( irq_vector_fast_interrupt,        default_fast_interrupt_handler )
PLATFORM_SET_DEFAULT_ISR( irq_vector_software_interrupt,    default_software_interrupt_handler )
PLATFORM_SET_DEFAULT_ISR( irq_vector_reserved,              default_reserved_interrupt_handler )

#define EXCEPTION_HANDLER_DUMP( str )

#define EXCEPTION_HANDLER_BREAKPOINT() \
    do \
    {  \
        EXCEPTION_HANDLER_DUMP( __func__ );   \
        breakpoint_inside_exception = true; \
        __BKPT(0);               \
    }  \
    while ( 0 )

void __WEAK NEVER_INLINE platform_exception_debug( void )
{
}

__USED static void default_fast_interrupt_handler( void )
{
    uint32_t lr = get_LR(); /* <----- CHECK THIS FOR WHERE EXCEPTION HAPPENED */

    CY_UNUSED_PARAMETER( lr );

    /* Fast interrupt triggered without any configured handler */
    EXCEPTION_HANDLER_BREAKPOINT();
    while (1)
    {
        /* Loop forever */
    }
}

__USED static void default_software_interrupt_handler( void )
{
    uint32_t lr = get_LR(); /* <----- CHECK THIS FOR WHERE EXCEPTION HAPPENED */

    CY_UNUSED_PARAMETER( lr );

    /* Software interrupt triggered without any configured handler */
    EXCEPTION_HANDLER_BREAKPOINT();
    while (1)
    {
        /* Loop forever */
    }
}

__USED static void default_reserved_interrupt_handler( void )
{
    uint32_t lr = get_LR(); /* <----- CHECK THIS FOR WHERE EXCEPTION HAPPENED */

    CY_UNUSED_PARAMETER( lr );

    /* Reserved interrupt triggered - This should never happen! */
    EXCEPTION_HANDLER_BREAKPOINT();
    while (1)
    {
        /* Loop forever */
    }
}

__USED static void prefetch_abort_handler( void )
{
    uint32_t lr               = get_LR(); /* <----- CHECK THIS FOR WHERE EXCEPTION HAPPENED */
    uint32_t ifar             = get_IFAR(); /* <----- CHECK THIS FOR ADDRESS OF INSTRUCTION */
    cr4_fault_status_t status = (cr4_fault_status_t) (get_IFSR( ) & CR4_FAULT_STATUS_MASK);  /* <----- CHECK THIS FOR STATUS */

    CY_UNUSED_PARAMETER( lr );
    CY_UNUSED_PARAMETER( ifar );

    /* Prefetch abort occurred */

    /* This means debug event, like breakpoint instruction and no JTAG debugger attached. */
    if ( status == CR4_FAULT_STATUS_DEBUG_EVENT )
    {
        if ( !breakpoint_inside_exception )
        {
            EXCEPTION_HANDLER_DUMP( "Debug event (e.g. breakpoint)" );
        }
        while ( 1 )
        {
            platform_exception_debug();
        }
    }

    /* This means that the processor attempted to execute an instruction
     * which was marked invalid due to a memory access failure (i.e. an abort)
     *
     *  - permission fault indicated by the MPU
     *  - an error response to a transaction on the AXI memory bus
     *  - an error detected in the data by the ECC checking logic.
     *
     */
    EXCEPTION_HANDLER_BREAKPOINT();

    __ASM volatile ( "MOV LR, %0; SUBS PC, LR, #4" : : "r"(lr) );  /* Step here to return to the instruction which caused the prefetch abort */
}

__USED static void data_abort_handler( void )
{
    uint32_t lr               = get_LR(); /* <----- CHECK THIS FOR WHERE EXCEPTION HAPPENED */
    uint32_t dfar             = get_DFAR(); /* <----- CHECK THIS FOR ADDRESS OF DATA ACCESS */
    cr4_fault_status_t status = (cr4_fault_status_t) (get_DFSR( ) & CR4_FAULT_STATUS_MASK);  /* <----- CHECK THIS FOR STATUS */

    CY_UNUSED_PARAMETER( lr );
    CY_UNUSED_PARAMETER( dfar );
    CY_UNUSED_PARAMETER( status );

    /* Data abort occurred */

    /* This means that the processor attempted a data access which
     * caused a memory access failure (i.e. an abort)
     *
     *  - permission fault indicated by the MPU
     *  - an error response to a transaction on the AXI memory bus
     *  - an error detected in the data by the ECC checking logic.
     *
     */
    EXCEPTION_HANDLER_BREAKPOINT();

    __ASM volatile ( "MOV LR, %0; SUBS PC, LR, #8" : : "r"(lr) );  /* If synchronous abort, Step here to return to the instruction which caused the data abort */
}

__USED static void undefined_instruction_handler( void )
{
    uint32_t lr = get_LR(); /* <----- CHECK THIS FOR WHERE EXCEPTION HAPPENED */

    CY_UNUSED_PARAMETER( lr );

    /* Undefined instruction exception occurred */
    EXCEPTION_HANDLER_BREAKPOINT();

    __ASM volatile ( "MOV LR, %0; MOVS PC, LR" : : "r"(lr) );  /* Step here to return to the instruction which caused the undefined instruction exception */
}

#endif /* DEBUG */
