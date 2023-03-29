/***************************************************************************//**
* \file cyhal_dma.c
*
* \brief
* Implements a high level interface for interacting with the Infineon DMA.
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
#include "cyhal_dma.h"
#include "cyhal_system.h"
#include "cyhal_syspm.h"
#include "cyhal_hwmgr.h"

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
extern cyhal_dma_t*          _cyhal_m2m_cb[];


/*******************************************************************************
*       Internal - Helper
*******************************************************************************/

#define M2M_DMA_RX_BUFFER_SIZE      (256)

#define _CYHAL_DMA_SRAM_START       (0x004A0000UL)
#define _CYHAL_DMA_SRAM_SIZE        (0x200000UL)
#define _CYHAL_DMA_SRAM_END         (_CYHAL_DMA_SRAM_START + _CYHAL_DMA_SRAM_SIZE)


/*******************************************************************************
*       Internal - LPM
*******************************************************************************/

static bool _cyhal_dma_pm_has_enabled(void)
{
    for (int group = 0; group < _CYHAL_M2M_GRPS; group++)
    {
        if (_cyhal_m2m_cb[group] != NULL)
            return true;
    }
    return false;
}

static bool _cyhal_dma_pm_transition_pending_value = false;

static bool _cyhal_dma_pm_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);

    switch(mode)
    {
        case CYHAL_SYSPM_CHECK_FAIL:
        case CYHAL_SYSPM_AFTER_TRANSITION:
            _cyhal_dma_pm_transition_pending_value = false;
            break;
        case CYHAL_SYSPM_CHECK_READY:
            for (uint8_t group = 0; group < _CYHAL_M2M_GRPS; group++)
            {
                if (_cyhal_m2m_cb[group] != NULL)
                {
                    if (cyhal_dma_is_busy(_cyhal_m2m_cb[group]))
                        return false;
                }
            }
            _cyhal_dma_pm_transition_pending_value = true;
            break;
        default:
            break;
    }

    return true;
}

static cyhal_syspm_callback_data_t _cyhal_dma_syspm_callback_data =
{
    .callback = &_cyhal_dma_pm_callback,
    .states = (cyhal_syspm_callback_state_t)(CYHAL_SYSPM_CB_CPU_DEEPSLEEP | CYHAL_SYSPM_CB_SYSTEM_HIBERNATE),
    .next = NULL,
    .args = NULL,
    .ignore_modes = CYHAL_SYSPM_BEFORE_TRANSITION,
};


/*******************************************************************************
*       Internal - Interrrupt
*******************************************************************************/

#define _CYHAL_M2M_GROUP_TRIGGERED(group) ((_cyhal_m2m_cb[group] != NULL) && (_cyhal_m2m_cb[group]->tx_started || !_cyhal_m2m_tx_is_idle(group)))

static void _cyhal_m2m_irq_handler (void)
{
    uint32_t intstatus;
    uint32_t events;
    uint8_t dma_ch;

    // WHD M2M-specific DMA handling
    // Note: Interrupt clearing happens downstream in WHD
    if (((_cyhal_m2m_m2mreg->intregs[ ACPU_DMA_TX_CHANNEL ].intstatus != 0)
        || (_cyhal_m2m_m2mreg->intregs[ ACPU_DMA_RX_CHANNEL ].intstatus) != 0)
        && (NULL != _cyhal_m2m_cb[_CYHAL_M2M_GRP_WWD]->callback_data.callback))
    {
        cyhal_dma_event_callback_t callback = (cyhal_dma_event_callback_t) _cyhal_m2m_cb[_CYHAL_M2M_GRP_WWD]->callback_data.callback;
        (callback)(_cyhal_m2m_cb[_CYHAL_M2M_GRP_WWD]->callback_data.callback_arg, CYHAL_DMA_TRANSFER_COMPLETE);
    }

    // Regular DMA handling
    for (uint8_t group = 0; group < _CYHAL_M2M_GRPS; group++)
    {
        if (NULL != _cyhal_m2m_cb[group])
        {
            intstatus = 0UL;
            events = 0UL;
            dma_ch = _cyhal_m2m_cb[group]->dma_ch;

            if (_cyhal_m2m_cb[group]->tx_started && _cyhal_m2m_tx_is_idle(group))
            {
                _cyhal_m2m_cb[group]->tx_started = false;
                intstatus = _cyhal_m2m_m2mreg->intregs[dma_ch].intstatus;
            }

            /*
                Note: USR 1 and USR 2 share interrupt status and hence should only
                be cleared when it is safe to do so. There are five scenarios to consider,
                1. USR 1 was triggered.
                2. USR 2 was triggered.
                3. USR 1 and USR 2 were both triggered
                4. USR 1 was triggered and USR 2 triggered while processing USR 1
                5. USR 2 was triggered and USR 1 triggered while processing USR 2
            */
            if (group == _CYHAL_M2M_GRP_WWD ||
                (group == _CYHAL_M2M_GRP_USR_1 && !_CYHAL_M2M_GROUP_TRIGGERED(_CYHAL_M2M_GRP_USR_2)) ||
                (group == _CYHAL_M2M_GRP_USR_2 && !_CYHAL_M2M_GROUP_TRIGGERED(_CYHAL_M2M_GRP_USR_1)))
            {
                _cyhal_m2m_m2mreg->intregs[dma_ch].intstatus = ~0x0;
            }

            if ((intstatus & I_RI) != 0UL)
                events |= (CYHAL_DMA_TRANSFER_COMPLETE | CYHAL_DMA_DESCRIPTOR_COMPLETE);
            if ((intstatus & (I_DA | I_XI)) != 0UL)
                events |= (CYHAL_DMA_SRC_BUS_ERROR | CYHAL_DMA_DST_BUS_ERROR);
            if ((intstatus & I_RU) != 0UL)
                events |= CYHAL_DMA_CURR_PTR_NULL;
            if ((intstatus & (I_DE | I_DP)) != 0UL)
                events |= CYHAL_DMA_DESCR_BUS_ERROR;

            if ((NULL != _cyhal_m2m_cb[group]->callback_data.callback) && (events != 0UL))
            {
                cyhal_dma_event_callback_t callback = (cyhal_dma_event_callback_t) _cyhal_m2m_cb[group]->callback_data.callback;
                (callback)(_cyhal_m2m_cb[group]->callback_data.callback_arg, (cyhal_dma_event_t)events);
            }
        }
    }
}

/* M2M DMA:
 * APPS: TX/RX : M2M channel 1/channel 0
 * WLAN: TX/RX : M2M channel 0/channel 1
 * APPS and WLAN core only enable RX interrupt. Regarding TX DONE interrupt,
 * APPS core uses SW0 interrupt to signal WLAN core once there is TX DONE interrupt happening.
 * WLAN core also uses SW0 interrupt to signal APPS core the TX DONE interrupt.
 * */
PLATFORM_DEFINE_ISR(_cyhal_m2m_irq_handler);
PLATFORM_MAP_ISR(_cyhal_m2m_irq_handler, M2M_ISR);
PLATFORM_MAP_ISR(_cyhal_m2m_irq_handler, Sw0_ISR);


/*******************************************************************************
*       HAL implementation
*******************************************************************************/

cy_rslt_t cyhal_dma_init_adv(cyhal_dma_t *obj, cyhal_dma_src_t *src, cyhal_dma_dest_t *dest,
                            cyhal_source_t *dest_source, uint8_t priority, cyhal_dma_direction_t direction)
{
    CY_ASSERT(NULL != obj);

    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    if ((src == NULL) && (dest == NULL) && (dest_source == NULL)
        && (direction == CYHAL_DMA_DIRECTION_MEM2MEM)
        && (priority < _CYHAL_M2M_GRPS))
    {
        // Do not allow WHD M2M for general use
        uint8_t group = (priority == _CYHAL_M2M_GRP_WWD) ? ++priority : priority;
        cyhal_resource_inst_t rscObj = {CYHAL_RSC_DMA, group, 0u};

        do
        {
            rscObj.block_num = group;
            rslt =  cyhal_hwmgr_reserve(&rscObj);
            group++;
        } while (rslt != CY_RSLT_SUCCESS && group < _CYHAL_M2M_GRPS);

        if (rslt == (CY_RSLT_SUCCESS))
        {
            group--;
            uint8_t tx_ch, rx_ch, dma_ch;
            _cyhal_m2m_get_channels(group, &tx_ch, &rx_ch, &dma_ch);

            obj->resource.type = rscObj.type;
            obj->resource.block_num = rscObj.block_num;
            obj->resource.channel_num = rscObj.channel_num;
            obj->tx_ch = tx_ch;
            obj->rx_ch = rx_ch;
            obj->dma_ch = dma_ch;
            obj->group = group;
            obj->direction = direction;
            obj->tx_started = false;
            obj->is_enabled = false;
            obj->callback_data.callback = NULL;
            obj->callback_data.callback_arg = NULL;
            _cyhal_m2m_init_dma((_cyhal_m2m_group_t)(obj->group), obj->tx_ch, obj->rx_ch, M2M_DMA_RX_BUFFER_SIZE);

            if (!_cyhal_dma_pm_has_enabled())
            {
                _cyhal_syspm_register_peripheral_callback(&_cyhal_dma_syspm_callback_data);
            }
            _cyhal_m2m_cb[group] = obj;
        }
    }
    else
    {
        rslt = CYHAL_DMA_RSLT_ERR_INVALID_PARAMETER;
    }

    return rslt;
}

cy_rslt_t cyhal_dma_init_cfg(cyhal_dma_t *obj, const cyhal_dma_configurator_t *cfg)
{
    /* No configurators supported on this architecture */
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(cfg);
    return CYHAL_DMA_RSLT_FATAL_UNSUPPORTED_HARDWARE;
}

void cyhal_dma_free(cyhal_dma_t *obj)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(!cyhal_dma_is_busy(obj));

    _cyhal_m2m_cb[obj->group] = NULL;
    _cyhal_m2m_disable_dma(obj->dma_ch);
    _cyhal_m2m_deinit_dma((_cyhal_m2m_group_t)(obj->group));

    if (!_cyhal_dma_pm_has_enabled())
    {
        _cyhal_syspm_unregister_peripheral_callback(&_cyhal_dma_syspm_callback_data);
    }

    cyhal_hwmgr_free(&obj->resource);
}

cy_rslt_t cyhal_dma_configure(cyhal_dma_t *obj, const cyhal_dma_cfg_t *cfg)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(obj->resource.type == CYHAL_RSC_DMA);

    cy_rslt_t status = CY_RSLT_SUCCESS;

    if (cfg->burst_size != 0 || cfg->src_increment != 1 || cfg->dst_increment != 1 || cfg->action != CYHAL_DMA_TRANSFER_FULL)
        status = CYHAL_DMA_RSLT_ERR_INVALID_PARAMETER;

    if (status == CY_RSLT_SUCCESS && obj->direction == CYHAL_DMA_DIRECTION_MEM2MEM)
    {
        if ((cfg->src_addr < _CYHAL_DMA_SRAM_START) && (cfg->src_addr > _CYHAL_DMA_SRAM_END)
            && (cfg->dst_addr < _CYHAL_DMA_SRAM_START) && (cfg->dst_addr > _CYHAL_DMA_SRAM_END))
        {
            status = CYHAL_DMA_RSLT_ERR_INVALID_PARAMETER;
        }
    }

    if (status == CY_RSLT_SUCCESS)
    {
        if (!((cfg->transfer_width == 8u) || (cfg->transfer_width == 16u) || (cfg->transfer_width == 32u)))
            status = CYHAL_DMA_RSLT_ERR_INVALID_PARAMETER;
    }

    if (status == CY_RSLT_SUCCESS)
    {
        obj->src_addr = cfg->src_addr;
        obj->dst_addr = cfg->dst_addr;
        obj->bytes = cfg->length * ((cfg->transfer_width) / 8u);
    }

    return status;
}

cy_rslt_t cyhal_dma_start_transfer(cyhal_dma_t *obj)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(obj->resource.type == CYHAL_RSC_DMA);

    if (_cyhal_dma_pm_transition_pending_value)
    {
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }

    if (obj->is_enabled)
    {
        if (cyhal_dma_is_busy(obj))
            return CYHAL_DMA_RSLT_WARN_TRANSFER_ALREADY_STARTED;
        
        // DMA channel is shared so only transmit if neither are transmitting
        if ((obj->group == _CYHAL_M2M_GRP_WWD)
            || ((_cyhal_m2m_cb[_CYHAL_M2M_GRP_USR_1] == NULL || !_cyhal_m2m_cb[_CYHAL_M2M_GRP_USR_1]->tx_started)
                && (_cyhal_m2m_cb[_CYHAL_M2M_GRP_USR_2] == NULL || !_cyhal_m2m_cb[_CYHAL_M2M_GRP_USR_2]->tx_started)))
        {
            obj->tx_started = (_cyhal_m2m_m2mreg->intregs[obj->dma_ch].intmask != 0UL);
            _cyhal_m2m_unprotected_dma_memcpy(obj->dma_ch, (uint32_t *)(obj->dst_addr), (uint32_t *)(obj->src_addr), obj->bytes);
            return CY_RSLT_SUCCESS;
        }
        else
        {
            return CYHAL_DMA_RSLT_ERR_CHANNEL_BUSY;
        }
    }
    
    return CY_RSLT_SUCCESS; // Note: Intentional to match PSoC behavior
}

cy_rslt_t cyhal_dma_enable(cyhal_dma_t *obj)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(obj->resource.type == CYHAL_RSC_DMA);

    _cyhal_system_m2m_enable_irq();
    obj->is_enabled = true;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cyhal_dma_disable(cyhal_dma_t *obj)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(obj->resource.type == CYHAL_RSC_DMA);

    _cyhal_m2m_disable_dma(obj->dma_ch);
    obj->is_enabled = false;

    /* Turn off the m2m interrupt when no dmas are used */
    bool intrUsed = false;
    for (int group = 0; group < _CYHAL_M2M_GRPS; group++)
    {
        if (_cyhal_m2m_cb[group] != NULL)
        {
            if (_cyhal_m2m_cb[group]->is_enabled)
            {
                intrUsed = true;
                break;
            }
        }
    }

    if (!intrUsed)
        _cyhal_system_m2m_disable_irq();

    return CY_RSLT_SUCCESS;
}

bool cyhal_dma_is_busy(cyhal_dma_t *obj)
{
    CY_ASSERT(NULL != obj);
    CY_ASSERT(obj->resource.type == CYHAL_RSC_DMA);

    return !_cyhal_m2m_tx_is_idle((_cyhal_m2m_group_t)(obj->group));
}

void cyhal_dma_register_callback(cyhal_dma_t *obj, cyhal_dma_event_callback_t callback, void *callback_arg)
{
    CY_ASSERT(NULL != obj);

    uint32_t saved_intr_status = cyhal_system_critical_section_enter();
    obj->callback_data.callback = (cy_israddress)callback;
    obj->callback_data.callback_arg = callback_arg;
    cyhal_system_critical_section_exit(saved_intr_status);
}

void cyhal_dma_enable_event(cyhal_dma_t *obj, cyhal_dma_event_t event, uint8_t intr_priority, bool enable)
{
    CY_UNUSED_PARAMETER(intr_priority);
    CY_ASSERT(NULL != obj);
    CY_ASSERT(obj->resource.type == CYHAL_RSC_DMA);

    uint32_t intmask = 0UL;

    if (event & (CYHAL_DMA_TRANSFER_COMPLETE | CYHAL_DMA_DESCRIPTOR_COMPLETE))
        intmask |= I_RI;
    if (event & (CYHAL_DMA_SRC_BUS_ERROR | CYHAL_DMA_DST_BUS_ERROR))
        intmask |= I_DA | I_XI;
    if (event & CYHAL_DMA_CURR_PTR_NULL)
        intmask |= I_RU;
    if (event & CYHAL_DMA_DESCR_BUS_ERROR)
        intmask |= I_DE | I_DP;
    if (event & (CYHAL_DMA_SRC_MISAL | CYHAL_DMA_DST_MISAL | CYHAL_DMA_ACTIVE_CH_DISABLED))
        CY_ASSERT(false); // Unsupported events

    // Check if event didn't match any known events
    if (event != CYHAL_DMA_NO_INTR && intmask == 0)
        CY_ASSERT(false);

    if (event == CYHAL_DMA_NO_INTR)
    {
        _cyhal_m2m_m2mreg->intregs[obj->dma_ch].intmask = 0UL;
    }
    else
    {
        if (enable)
        {
            _cyhal_m2m_m2mreg->intregs[obj->dma_ch].intmask |= intmask;
        }
        else
        {
            _cyhal_m2m_m2mreg->intregs[obj->dma_ch].intmask &= ~intmask;
        }
    }
}

cy_rslt_t cyhal_dma_connect_digital(cyhal_dma_t *obj, cyhal_source_t source, cyhal_dma_input_t input)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(input);
    return CYHAL_DMA_RSLT_FATAL_UNSUPPORTED_HARDWARE;
}

cy_rslt_t cyhal_dma_enable_output(cyhal_dma_t *obj, cyhal_dma_output_t output, cyhal_source_t *source)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);
    CY_UNUSED_PARAMETER(source);
    return CYHAL_DMA_RSLT_FATAL_UNSUPPORTED_HARDWARE;
}

cy_rslt_t cyhal_dma_disconnect_digital(cyhal_dma_t *obj, cyhal_source_t source, cyhal_dma_input_t input)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(source);
    CY_UNUSED_PARAMETER(input);
    return CYHAL_DMA_RSLT_FATAL_UNSUPPORTED_HARDWARE;
}

cy_rslt_t cyhal_dma_disable_output(cyhal_dma_t *obj, cyhal_dma_output_t output)
{
    CY_UNUSED_PARAMETER(obj);
    CY_UNUSED_PARAMETER(output);
    return CYHAL_DMA_RSLT_FATAL_UNSUPPORTED_HARDWARE;
}

#if defined(__cplusplus)
}
#endif /* __cplusplus */
