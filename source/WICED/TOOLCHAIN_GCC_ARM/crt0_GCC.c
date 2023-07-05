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

/* For information on duties of crt0, see http://www.fhi-berlin.mpg.de/th/locserv/alphas/progs/doc/cygnus_doc-99r1/html/1_GS/int04.html#crt0,_the_main_startup_file
 * For details of various segments, see http://www.acsu.buffalo.edu/~charngda/elf.html
 */

#include <string.h>
#include "platform_toolchain.h"
#include "cr4.h"

extern void * link_bss_location;
extern void * link_bss_end;
#define link_bss_size   ((unsigned long)&link_bss_end  -  (unsigned long)&link_bss_location )

extern void * link_dma_location;
extern void * link_dma_end;
#define link_dma_size   ((unsigned long)&link_dma_end  -  (unsigned long)&link_dma_location )

typedef void  (*constructor_ptr_t)( void );
extern constructor_ptr_t link_constructors_location[];
extern constructor_ptr_t link_constructors_end;

__WEAK void _start( void )      __NO_RETURN;
__WEAK void _exit( int status ) __NO_RETURN;
int main( void );

#define CRT_START_FUNCTION_TYPE static inline ALWAYS_INLINE
CRT_START_FUNCTION_TYPE void _start_low( void );
CRT_START_FUNCTION_TYPE void _start_platform_init( void );
CRT_START_FUNCTION_TYPE void _start_done( void ) __NO_RETURN;

#define link_constructors_size   ((unsigned long)&link_constructors_end  -  (unsigned long)&link_constructors_location )

CRT_START_FUNCTION_TYPE void _start_low( void )
{
    cr4_init_cycle_counter( );

    /* BSS segment is for zero initialised elements, so memset it to zero */
    memset( &link_bss_location, 0, (size_t) link_bss_size );

    /* Initialize DMA descriptors */
    memset( &link_dma_location, 0, (size_t) link_dma_size );
}

CRT_START_FUNCTION_TYPE void _start_platform_init( void )
{
    unsigned long ctor_num;

    /*
     * Run global C++ constructors if any
     */
    for ( ctor_num = 0; ctor_num < link_constructors_size/sizeof(constructor_ptr_t); ctor_num++ )
    {
        link_constructors_location[ctor_num]();
    }
}

CRT_START_FUNCTION_TYPE void _start_done( void )
{
    /* the main loop has returned - there is now nothing to do */
    while ( 1 ); /* Loop forever */
}

__WEAK void cy_toolchain_init(void)
{
    /* Strong definition in CLIB-support */
}

void _start( void )
{
    _start_low( );
    _start_platform_init( );
    /* OS-specific low-level initialization */
    cy_toolchain_init();
    main( );
    _start_done( );
}

void _exit( int status )
{
    /* the main loop has returned - there is now nothing to do */
    (void) status; /* unused parameter */
    while(1){}
}
