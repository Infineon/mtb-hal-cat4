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
 *  Defines the Buffer Interface.
 *
 *  Provides prototypes for functions that allow functions to use packet
 *  buffers in an abstract way.
 *
 */

#pragma once

#include "typedefs.h"
#include "cy_result.h"

#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************
 * @cond       Constants
 ******************************************************/

/******************************************************
 *             Function declarations
 ******************************************************/

/**
 * Initialize the packet buffer interface
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Some implementations of the packet buffer interface may need additional
 * information for initialization, especially the location of packet buffer
 * pool(s). These can be passed via the 'native_arg' parameter.
 *
 * @return CY_RSLT_SUCCESS = Success, Error code = Failure
 */

extern cy_rslt_t _cyhal_gmac_host_read_buffer_init(void);

/**
 * Deinitialize the packet buffer interface
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 *
 * @return CY_RSLT_SUCCESS = Success, Error code = Failure
 */

extern cy_rslt_t _cyhal_gmac_host_read_buffer_deinit( void );

/**
 * @brief Allocates a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Attempts to allocate a packet buffer of the size requested. It can do this
 * by allocating a pre-existing packet from a pool, using a static buffer,
 * or by dynamically allocating memory. The method of allocation does not
 * concern WICED, however it must match the way the network stack expects packet
 * buffers to be allocated.
 *
 * @param buffer     A pointer which receives the allocated packet buffer handle
 * @param size      : The number of bytes to allocate.
 *
 * @return CY_RSLT_SUCCESS = Success, Error code = Failure
 *
 */

extern cy_rslt_t _cyhal_gmac_host_read_buffer_get( uint8_t **buffer, unsigned short size);

/**
 * Releases a packet buffer
 *
 * Implemented in the Wiced buffer interface, which will be specific to the
 * buffering scheme in use.
 * This function is used by WICED to indicate that it no longer requires
 * a packet buffer. The buffer can then be released back into a pool for
 * reuse, or the dynamically allocated memory can be freed, according to
 * how the packet was allocated.
 * Returns void since WICED cannot do anything about failures
 *
 * @param buffer    : the handle of the packet buffer to be released
 *
 */
extern void _cyhal_gmac_host_read_buffer_release( uint8_t *buffer );

/**
 * Retrieves the current pointer of a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Since packet buffers usually need to be created with space at the
 * front for additional headers, this function allows WICED to get
 * the current 'front' location pointer.
 *
 * @param buffer : The handle of the packet buffer whose pointer is to be retrieved
 *
 * @return The packet buffer's current pointer.
 */
extern uint8_t* _cyhal_gmac_host_read_buffer_get_current_piece_data_pointer( uint8_t *buffer );

/**
 * Retrieves the size of a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Since packet buffers usually need to be created with space at the
 * front for additional headers, the memory block used to contain a packet buffer
 * will often be larger than the current size of the packet buffer data.
 * This function allows WICED to retrieve the current size of a packet buffer's data.
 *
 * @param buffer : The handle of the packet buffer whose size is to be retrieved
 *
 * @return The size of the packet buffer.
 */
extern uint16_t _cyhal_gmac_host_read_buffer_get_current_piece_size( uint8_t *buffer );

/**
 * Sets the current size of a Wiced packet
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * This function sets the current length of a WICED packet buffer
 *
 * @param buffer : The packet to be modified
 * @param size   : The new size of the packet buffer
 *
 * @return CY_RSLT_SUCCESS = Success, Error code = Failure
 */
extern cy_rslt_t _cyhal_gmac_host_read_buffer_set_size( uint8_t *buffer, unsigned short size );

/**
 * Retrieves the next piece of a set of daisy chained packet buffers
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Some buffering schemes allow buffers to be daisy chained into linked lists.
 * This allows more flexibility with packet buffers and avoids memory copies.
 * It does however require scatter-gather DMA for the hardware bus.
 * This function retrieves the next buffer in a daisy chain of packet buffers.
 *
 * @param buffer : The handle of the packet buffer whose next buffer is to be retrieved
 *
 * @return The handle of the next buffer, or NULL if there is none.
 */
extern uint8_t *_cyhal_gmac_host_read_buffer_get_next_piece( uint8_t *buffer );

/**
 * Moves the current pointer of a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Since packet buffers usually need to be created with space at the
 * front for additional headers, this function allows WICED to move
 * the current 'front' location pointer so that it has space to add headers
 * to transmit packets, and so that the network stack does not see the
 * internal WICED headers on received packets.
 *
 * @param buffer    : A pointer to the handle of the current packet buffer
 *                    for which the current pointer will be moved. On return
 *                    this may contain a pointer to a newly allocated packet
 *                    buffer which has been daisy chained to the front of the
 *                    given one. This would be the case if the given packet buffer
 *                    didn't have enough space at the front.
 * @param add_remove_amount : This is the number of bytes to move the current pointer
 *                            of the packet buffer - a negative value increases the space
 *                            for headers at the front of the packet, a positive value
 *                            decreases the space.
 * @return CY_RSLT_SUCCESS = Success, Error code = Failure
 */
extern cy_rslt_t _cyhal_gmac_host_read_buffer_add_remove_at_front( uint8_t **buffer, int32_t add_remove_amount );


extern cy_rslt_t _cyhal_gmac_host_read_buffer_set_next_piece(uint8_t *buffer, uint8_t *next_buffer );


/** @} */

#ifdef __cplusplus
} /*extern "C" */
#endif

