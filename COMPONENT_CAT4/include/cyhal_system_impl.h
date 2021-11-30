/***************************************************************************//**
* \file cyhal_system_impl.h
*
* \brief
* Provides a 43907 Specific interface for interacting with the power
* management and system configuration. This interface abstracts out the
* chip specific details. If any chip specific functionality is necessary, or
* performance is critical the low level functions can be used directly.
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

#include "cyhal_system.h"
#include "platform_appscr4.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
* \addtogroup group_hal_impl_system System
* \ingroup group_hal_impl
* \{
* Routines for accessing global resources shared across multiple drivers.
*/

/** Write to a chip control register
 *
 * This is used to write to an indirect register not directly accessible by the CPU.
 * 
 * @param[in] addr_reg   Base address
 * @param[in] ctrl_reg   Control register
 * @param[in] reg_offset Offset value to access the register
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
uint32_t _cyhal_system_chipcontrol(volatile uint32_t* addr_reg, volatile uint32_t* ctrl_reg,
                     uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask);

/** Write to a common chip control register
 *
 * This is used to directly write to a control register.
 * 
 * @param[in] reg        Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
uint32_t _cyhal_system_common_chipcontrol(volatile uint32_t* reg, uint32_t clear_mask, uint32_t set_mask);

/** Read from a chip status register
 *
 * This is used to read from an indirect status register not directly accessible by the CPU.
 * 
 * @param[in] addr_reg   Base address
 * @param[in] status_reg Status register
 * @param[in] reg_offset Offset value to access the register
 * @return Value read
 */
uint32_t _cyhal_system_chipstatus(volatile uint32_t* addr_reg, volatile uint32_t* status_reg,
                    uint8_t reg_offset);

/** Write to GCI chip control register
 *
 * This is used to write to an indirect GCI CC register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_gci_chipcontrol(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(GCI_INDIRECT_ADDR_REG, GCI_CHIPCONTROL_REG,
                                reg_offset, clear_mask, set_mask);
}

/** Read from GCI chip status register
 *
 * This is used to read from an indirect status register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Offset value to access the register
 * @return Value read
 */
static inline uint32_t _cyhal_system_gci_chipstatus(uint8_t reg_offset)
{
    return _cyhal_system_chipstatus(GCI_INDIRECT_ADDR_REG, GCI_CHIPSTATUS_REG,
                               reg_offset);
}

/** Write to GCI GPIO control register
 *
 * This is used to write to an indirect GCI GPIO register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_gci_gpiocontrol(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(GCI_INDIRECT_ADDR_REG, GCI_GPIOCONTROL_REG,
                                reg_offset, clear_mask, set_mask);
}

/** Write to GCI GPIO status register
 *
 * This is used to write to an indirect GCI GPIO status register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_gci_gpiostatus(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(GCI_INDIRECT_ADDR_REG, GCI_GPIOSTATUS_REG,
                                reg_offset, clear_mask, set_mask);
}

/** Write to GCI GPIO wake make register
 *
 * This is used to write to an indirect GCI GPIO wake mask register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_gci_gpiowakemask(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(GCI_INDIRECT_ADDR_REG, GCI_GPIOWAKEMASK_REG,
                                reg_offset, clear_mask, set_mask);
}

/** Write to PMU chip control register
 *
 * This is used to write to an indirect PMU CC register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_pmu_chipcontrol(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(&PLATFORM_PMU->chipcontrol_addr, &PLATFORM_PMU->chipcontrol_data,
                                reg_offset, clear_mask, set_mask);
}

/** Write to PMU resource up/down register
 *
 * This is used to write to an indirect PMU resource updown register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_pmu_res_updown_time(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(&PLATFORM_PMU->res_table_sel, &PLATFORM_PMU->res_updn_timer,
                                reg_offset, clear_mask, set_mask);
}

/** Write to PMU resource dependency mask register
 *
 * This is used to write to an indirect PMU resource dependency register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_pmu_res_dep_mask(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(&PLATFORM_PMU->res_table_sel, &PLATFORM_PMU->res_dep_mask,
                                reg_offset, clear_mask, set_mask);
}

/** Write to PMU regulator control register
 *
 * This is used to write to an indirect PMU regulator control register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_pmu_regulatorcontrol(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(&PLATFORM_PMU->regcontrol_addr, &PLATFORM_PMU->regcontrol_data,
                                reg_offset, clear_mask, set_mask);
}

/** Write to PMU PLL control register
 *
 * This is used to write to an indirect PMU PLL control register not directly accessible by the CPU.
 * 
 * @param[in] reg_offset Register to write
 * @param[in] clear_mask Clear mask value
 * @param[in] set_mask   Set mask value
 * @return Value written
 */
static inline uint32_t _cyhal_system_pmu_pllcontrol(uint8_t reg_offset, uint32_t clear_mask, uint32_t set_mask)
{
    return _cyhal_system_chipcontrol(&PLATFORM_PMU->pllcontrol_addr, &PLATFORM_PMU->pllcontrol_data,
                                reg_offset, clear_mask, set_mask);
}

#if defined(PLATFORM_4390X_OVERCLOCK)

/** Get PMU PLL control divider
 * 
 * @param[in] channel    BB PLL channel number
 * @return Divider value
 */
uint8_t _cyhal_system_pmu_pllcontrol_mdiv_get(uint8_t channel);

/** Set PMU PLL control divider
 * 
 * @param[in] channel    BB PLL channel number
 * @param[in] divider    Divider value
 */
void _cyhal_system_pmu_pllcontrol_mdiv_set(uint8_t channel, uint8_t divider);

#endif

/** Connect pin multiplexer
 *
 * This function allows connecting/multiplexing peripherals to pins. 
 * Do not use this for CPU (SW) driven GPIO. Use the GPIO driver instead.
 * Note that pin states can always be read by the CPU using the GPIO driver.
 * 
 * @param[in] pin                 Pin pad
 * @param[in] gci_chipcontrol_mux Mux selection value
 */
void _cyhal_system_pinmux_connect(cyhal_gpio_t pin, int gci_chipcontrol_mux);

/** Route an interrupt bus line to an external interrupt
 *
 * This function should be used with the _cyhal_system_irq_remap_source() function.
 * 
 * @param[in] bus_line_num        Bus line number
 * @param[in] sink_num            External interrupt number
 * @return Success (true), Failed (false)
 */
bool _cyhal_system_irq_remap_sink( uint8_t bus_line_num, uint8_t sink_num );

/** Route an interrupt source to an interrupt bus line
 *
 * This function should be used with the _cyhal_system_irq_remap_sink() function.
 * 
 * @param[in] wrapper_addr        Wrapper base address
 * @param[in] source_num          Interrupt source number
 * @param[in] bus_line_num        Bus line number
 * @return Success (true), Failed (false)
 */
bool _cyhal_system_irq_remap_source( uint32_t wrapper_addr, uint8_t source_num, uint8_t bus_line_num );

/** Register an internal peripheral callback
 * 
 * @param[in] callback_data       Pointer to callback data
 */
void _cyhal_syspm_register_peripheral_callback(cyhal_syspm_callback_data_t *callback_data);

/** Unregister an internal peripheral callback
 * 
 * @param[in] callback_data       Pointer to callback data
 */
void _cyhal_syspm_unregister_peripheral_callback(cyhal_syspm_callback_data_t *callback_data);

/** Enable external interrupts from Timer to APPS Core */
void _cyhal_system_timer_enable_irq(void);

/** Disable external interrupts from Timer to APPS Core */
static inline void _cyhal_system_timer_disable_irq(void)
{
    platform_irq_disable_irq(Timer_ExtIRQn);
}

/** Disable external interrupts from ChipCommon to APPS Core */
static inline void _cyhal_system_chipcommon_disable_irq(void)
{
    platform_irq_disable_irq(ChipCommon_ExtIRQn);
}

/** Enable external interrupts from ChipCommon to APPS Core */
static inline void _cyhal_system_chipcommon_enable_irq(void)
{
    platform_irq_enable_irq(ChipCommon_ExtIRQn);
}

/** Disable external interrupts from M2M_ExtIRQn to APPS Core */
static inline void _cyhal_system_m2m_disable_irq(void)
{
    platform_irq_disable_irq(M2M_ExtIRQn);
}

/** Enable external interrupts from M2M_ExtIRQn to APPS Core */
static inline void _cyhal_system_m2m_enable_irq(void)
{
    platform_irq_enable_irq(M2M_ExtIRQn);
}

/** Disable external interrupts from SW0_ExtIRQn to APPS Core */
static inline void _cyhal_system_sw0_disable_irq(void)
{
    platform_irq_disable_irq(SW0_ExtIRQn);
}

/** Enable external interrupts from SW0_ExtIRQn to APPS Core */
static inline void _cyhal_system_sw0_enable_irq(void)
{
    platform_irq_enable_irq(SW0_ExtIRQn);
}

/** \} group_hal_impl_system */

#if defined(__cplusplus)
}
#endif
