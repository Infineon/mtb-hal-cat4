/***************************************************************************//**
* \file cyhal_dma_impl.h
*
* \brief
* Implementation details of Infineon DMA.
*
********************************************************************************
* \copyright
* Copyright 2021 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#pragma once

#include "cyhal_m2m_impl.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** 
 * \addtogroup group_hal_impl_dma DMA (Direct Memory Access)
 * \ingroup group_hal_impl
 * \{
 * DMA allows transferring data in SRAM without CPU intervention.
 * It can be used to transfer data from one memory location to another.
 * The highest priority channels #CYHAL_DMA_PRIORITY_HIGH are used by the WWD driver. For
 * application-specific needs, use the #CYHAL_DMA_PRIORITY_MEDIUM or #CYHAL_DMA_PRIORITY_LOW
 * channels.
 *
 */

/** Default DMA channel priority */
#define CYHAL_DMA_PRIORITY_DEFAULT    CYHAL_DMA_PRIORITY_LOW
/** High DMA channel priority */
#define CYHAL_DMA_PRIORITY_HIGH       0u
/** Medium DMA channel priority */
#define CYHAL_DMA_PRIORITY_MEDIUM     1u
/** Low DMA channel priority */
#define CYHAL_DMA_PRIORITY_LOW        2u

/** \} group_hal_impl_dma */

#if defined(__cplusplus)
}
#endif
