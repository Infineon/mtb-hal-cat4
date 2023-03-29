/***************************************************************************//**
* \file cyhal_m2m.c
*
* \brief
* Implements a high level interface for interacting with the Infineon M2M DMA.
* This implementation abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
*
********************************************************************************
* \copyright
* Copyright 2021-2022 Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_utils.h"
#include "cyhal_m2m_impl.h"
#include "cyhal_m2m.h"
#include "cyhal_system.h"
#include "cyhal_hwmgr.h"
#include "platform_appscr4.h"
#include "cr4.h"
#include "platform_cache.h"

#include "wiced_osl.h"

#include "m2m_hnddma.h"
#include "m2mdma_core.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
*       Internal - M2M DMA globals
*******************************************************************************/

/* Globals defined in common m2m driver */
extern volatile sbm2mregs_t* _cyhal_m2m_m2mreg;
extern m2m_hnddma_t*         _cyhal_m2m_dma_handle[];
extern osl_t*                _cyhal_m2m_osh[];
extern cyhal_dma_t*          _cyhal_m2m_cb[];

#define ACPU_DMA_RX_CHANNEL_IRQ_NUM         (1)

/*******************************************************************************
*       Internal - Helper
*******************************************************************************/

const cyhal_resource_inst_t rscObj = {CYHAL_RSC_DMA, _CYHAL_M2M_GRP_WWD, 0u};
cyhal_dma_t _cyhal_m2m_obj = 
{
    .resource.type = CYHAL_RSC_DMA,
    .resource.block_num = _CYHAL_M2M_GRP_WWD,
    .resource.channel_num = 0u,
    .tx_ch = ACPU_DMA_TX_CHANNEL,
    .rx_ch = ACPU_DMA_RX_CHANNEL,
    .dma_ch = MEMCPY_M2M_DMA_CHANNEL,
    .group = _CYHAL_M2M_GRP_WWD,
    .direction = 0,
    .tx_started = false,
    .is_enabled = false,
    .callback_data.callback = NULL,
    .callback_data.callback_arg = NULL
};

static void _cyhal_m2m_wlan_dma_deinit(void)
{
    /* Stop WLAN side M2M dma */
    m2m_dma_txreset_inner(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], (void *)&_cyhal_m2m_m2mreg->dmaregs[ACPU_DMA_RX_CHANNEL].tx);
    m2m_dma_rxreset_inner(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], (void *)&_cyhal_m2m_m2mreg->dmaregs[ACPU_DMA_TX_CHANNEL].rx);

    /* Reset wlan m2m tx/rx pointers */
    W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->dmaregs[ACPU_DMA_RX_CHANNEL].tx.ptr, 0);
    W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->dmaregs[ACPU_DMA_TX_CHANNEL].rx.ptr, 0);

    /* Mask interrupt & clear all interrupt status */
    W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ACPU_DMA_TX_CHANNEL].intmask, 0);
    W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ACPU_DMA_RX_CHANNEL].intstatus, 0xffffffff);
    W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ACPU_DMA_TX_CHANNEL].intstatus, 0xffffffff);
}

static void _cyhal_m2m_dma_set_irq_line_number(uint8_t channel, uint8_t irq_number)
{
    const uint32_t shift = channel * 2;
    const uint32_t mask  = 0x3 << shift;
    uint32_t intcontrol  = _cyhal_m2m_m2mreg->intcontrol;

    intcontrol &= ~mask;
    intcontrol |= ((uint32_t)irq_number << shift) & mask;

    _cyhal_m2m_m2mreg->intcontrol = intcontrol;
}

static void _cyhal_m2m_dma_enable_interrupts(bool enable, _cyhal_m2m_group_t group, uint8_t rx_ch)
{
    uint32_t rx_mask = enable ? I_DMA : 0x0;
    W_REG(_cyhal_m2m_osh[group], &_cyhal_m2m_m2mreg->intregs[rx_ch].intmask, rx_mask);
}

/*******************************************************************************
*       HAL implementation
*******************************************************************************/

cy_rslt_t cyhal_m2m_init(cyhal_m2m_t *obj, uint32_t rx_buffer_size)
{
    cy_rslt_t rslt = cyhal_hwmgr_reserve(&rscObj);

    if (rslt == (CY_RSLT_SUCCESS))
    {
        // The underlying resource is DMA so initialize it the same way
        _cyhal_m2m_init_dma(_CYHAL_M2M_GRP_WWD, ACPU_DMA_TX_CHANNEL, ACPU_DMA_RX_CHANNEL, rx_buffer_size);
        
        /* Configure all m2m channels except ACPU_DMA_TX_CHANNEL to use irq1 and remap the m2m irq */
        int channel_count = (_cyhal_m2m_m2mreg->capabilities & M2M_CAPABILITIES_CHANNEL_COUNT_MASK) + 1;
        for (int i = 0; i < channel_count ; i++)
        {
            if (i != ACPU_DMA_TX_CHANNEL)
            {
                _cyhal_m2m_dma_set_irq_line_number(i, ACPU_DMA_RX_CHANNEL_IRQ_NUM);
            }
        }
        _cyhal_system_irq_remap_sink(OOB_AOUT_M2M_INTR1, M2M_ExtIRQn);
        _cyhal_m2m_dma_enable_interrupts(true, _CYHAL_M2M_GRP_WWD, ACPU_DMA_RX_CHANNEL);

        /* Last step of initialization. Register settings inside tells WLAN that RX is ready. */
        m2m_dma_txinit(_cyhal_m2m_dma_handle[_CYHAL_M2M_GRP_WWD]);
    
        _cyhal_m2m_cb[_CYHAL_M2M_GRP_WWD] = &_cyhal_m2m_obj;
        obj->dma_obj = &_cyhal_m2m_obj;
    }

    return rslt;
}

void cyhal_m2m_free(cyhal_m2m_t *obj)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(!cyhal_m2m_is_busy(obj));

    _cyhal_m2m_disable_dma(MEMCPY_M2M_DMA_CHANNEL);
    _cyhal_m2m_dma_enable_interrupts(false, _CYHAL_M2M_GRP_WWD, ACPU_DMA_RX_CHANNEL);
    _cyhal_m2m_deinit_dma(_CYHAL_M2M_GRP_WWD);
    _cyhal_m2m_wlan_dma_deinit();

    cyhal_hwmgr_free(&obj->dma_obj->resource);
    _cyhal_m2m_cb[_CYHAL_M2M_GRP_WWD] = NULL;
    obj = NULL;
}

bool cyhal_m2m_is_busy(cyhal_m2m_t *obj)
{
    CY_ASSERT(NULL != obj);
    return !_cyhal_m2m_tx_is_idle(_CYHAL_M2M_GRP_WWD);
}

void cyhal_m2m_register_callback(cyhal_m2m_t *obj, cyhal_m2m_event_callback_t callback, void *callback_arg)
{
    CY_ASSERT(NULL != obj);
    uint32_t saved_intr_status = cyhal_system_critical_section_enter();
    obj->dma_obj->callback_data.callback = (cy_israddress)callback;
    obj->dma_obj->callback_data.callback_arg = callback_arg;
    cyhal_system_critical_section_exit(saved_intr_status);
}

void cyhal_m2m_enable_event(cyhal_m2m_t *obj, cyhal_m2m_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(intr_priority);
    CY_ASSERT(NULL != obj);

    uint32_t intmask = 0UL;

    if (event & CYHAL_M2M_DESCRIPTOR_READ_ERROR)
        intmask |= I_DE;
    if (event & CYHAL_M2M_DATA_TRANSFER_ERROR)
        intmask |= I_DA;
    if (event & CYHAL_M2M_DESCRIPTOR_PROGRAM_ERROR)
        intmask |= I_DP;
    if (event & CYHAL_M2M_NO_DESCRIPTOR_ERROR)
        intmask |= I_RU;
    if (event & CYHAL_M2M_RX_CHANNEL_INTERRUPT)
        intmask |= I_RI;
    if (event & CYHAL_M2M_TX_CHANNEL_INTERRUPT)
        intmask |= I_XI; // Note: WWD repurposes "TX error" for "TX interrupt"

    // Check if event didn't match any known events
    if (event != CYHAL_M2M_NO_INTR && intmask == 0)
        CY_ASSERT(false);

    if (event == CYHAL_M2M_NO_INTR)
    {
        _cyhal_m2m_m2mreg->intregs[MEMCPY_M2M_DMA_CHANNEL].intmask = 0UL;
    }
    else
    {
        if (enable)
        {
            _cyhal_m2m_m2mreg->intregs[MEMCPY_M2M_DMA_CHANNEL].intmask |= intmask;
        }
        else
        {
            _cyhal_m2m_m2mreg->intregs[MEMCPY_M2M_DMA_CHANNEL].intmask &= ~intmask;
        }
    }
}

cy_rslt_t cyhal_m2m_tx_send(cyhal_m2m_t *obj, void* buffer)
{
    CY_ASSERT(NULL != obj);
    /* function release packet inside if failed */
    return (cy_rslt_t)m2m_dma_txfast(_cyhal_m2m_dma_handle[_CYHAL_M2M_GRP_WWD], buffer, TRUE);
}

void cyhal_m2m_tx_release(cyhal_m2m_t *obj)
{
    CY_ASSERT(NULL != obj);
    void* txd;

    while ((txd = m2m_dma_getnexttxp(_cyhal_m2m_dma_handle[_CYHAL_M2M_GRP_WWD], FALSE)) != NULL)
    {
        PKTFREE(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], txd, TRUE);
    }
}

void cyhal_m2m_rx_receive(cyhal_m2m_t *obj, void** rxd_handle, uint16_t** hwtag)
{
    CY_UNUSED_PARAMETER(rxd_handle);
    CY_ASSERT(NULL != obj);
    void* packet = NULL;
    uint32_t timeout = 0xFFFFUL; // Upstream needs to handle the error condition

    while(timeout > 0UL)
    {
        timeout--;

        packet = m2m_dma_rx(_cyhal_m2m_dma_handle[_CYHAL_M2M_GRP_WWD]);
        if (packet == NULL)
        {
            break;
        }

        PKTPULL(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], packet, (uint16)SDPCMD_RXOFFSET);

        *hwtag = (uint16_t*) PKTDATA(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], packet);

        if (*hwtag == NULL)
        {
            PKTFREE(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], packet, FALSE);
            continue;
        }

        if (((*hwtag)[0] == 0) &&
            ((*hwtag)[1] == 0) &&
            (PKTLEN(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], packet) == 504))
        {
            // Dummy frame to keep M2M happy
            PKTFREE(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], packet, FALSE);
            continue;
        }

        if ((((*hwtag)[0] | (*hwtag)[1]) == 0               ) ||
            (((*hwtag)[0] ^ (*hwtag)[1]) != (uint16_t) 0xFFFF))
        {
            PKTFREE(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], packet, FALSE);
            continue;
        }

        break;
    }

    *rxd_handle = packet;
}

bool cyhal_m2m_rx_prepare(cyhal_m2m_t *obj)
{
    CY_ASSERT(NULL != obj);
    return m2m_dma_rxfill(_cyhal_m2m_dma_handle[_CYHAL_M2M_GRP_WWD]);
}

cyhal_m2m_rx_status_t cyhal_m2m_rx_status(cyhal_m2m_t *obj)
{
    CY_ASSERT(NULL != obj);
    return (cyhal_m2m_rx_status_t)m2m_dma_rxactive(_cyhal_m2m_dma_handle[_CYHAL_M2M_GRP_WWD]);
}

cyhal_m2m_event_t cyhal_m2m_intr_status(cyhal_m2m_t *obj, bool* signal_txdone)
{
    CY_ASSERT(NULL != obj);
    uint32_t intstatus = 0;
    uint32_t txint;
    uint32_t rxint;

    txint = R_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ ACPU_DMA_TX_CHANNEL ].intstatus);
    if (txint & I_XI)
    {
        W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ ACPU_DMA_TX_CHANNEL ].intstatus, I_XI);
        intstatus |= I_XI;
    }

    rxint = R_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ ACPU_DMA_RX_CHANNEL ].intstatus);
    if (rxint & I_RI)
    {
        W_REG(_cyhal_m2m_osh[_CYHAL_M2M_GRP_WWD], &_cyhal_m2m_m2mreg->intregs[ ACPU_DMA_RX_CHANNEL ].intstatus, I_RI);
        intstatus |= I_RI;
    }

    intstatus |= (txint & I_ERRORS) | (rxint & I_ERRORS);

    if (signal_txdone)
    {
        *signal_txdone = (rxint & I_XI) ? true : false;
    }

    /* Translate to M2M event type */
    cyhal_m2m_event_t event = CYHAL_M2M_NO_INTR;

    if (intstatus & I_DE)
        event |= CYHAL_M2M_DESCRIPTOR_READ_ERROR;
    if (intstatus & I_DA)
        event |= CYHAL_M2M_DATA_TRANSFER_ERROR;
    if (intstatus & I_DP)
        event |= CYHAL_M2M_DESCRIPTOR_PROGRAM_ERROR;
    if (intstatus & I_RU)
        event |= CYHAL_M2M_NO_DESCRIPTOR_ERROR;
    if (intstatus & I_RI)
        event |= CYHAL_M2M_RX_CHANNEL_INTERRUPT;
    if (intstatus & I_XI)
        event |= CYHAL_M2M_TX_CHANNEL_INTERRUPT; // Note: WWD repurposes "TX error" for "TX interrupt"
        
    return event;
}

#if defined(__cplusplus)
}
#endif /* __cplusplus */
