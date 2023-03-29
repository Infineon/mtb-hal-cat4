/*******************************************************************************
* File Name: cmsis_compiler.h
*
* Description:
* Unfortunately, no CMSIS port exists for the ARM Cortex-R devices. This file
* provides some standard macros that would generally be made available by the
* CMSIS library.
*
********************************************************************************
* \copyright
* Copyright 2021-2022 Cypress Semiconductor Corporation
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

#ifndef __CMSIS_COMPILER_H
#define __CMSIS_COMPILER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ARMCC_VERSION) /* Arm Compiler */

    #ifndef   __ALIGNED
        #define __ALIGNED(x)                           __attribute__((aligned(x)))
    #endif
    #ifndef   __ASM
        #define __ASM                                  __asm
    #endif
    #ifndef   __INLINE
        #define __INLINE                               __inline
    #endif
    #ifndef   __STATIC_INLINE
        #define __STATIC_INLINE                        static __inline
    #endif
    #ifndef   __STATIC_FORCEINLINE
        #define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static __inline
    #endif
    #ifndef   __NO_RETURN
        #define __NO_RETURN                            __attribute__((__noreturn__))
    #endif
    #ifndef   __USED
        #define __USED                                 __attribute__((used))
    #endif
    #ifndef   __WEAK
        #define __WEAK                                 __attribute__((weak))
    #endif
    #ifndef   __PACKED
        #define __PACKED                               __attribute__((packed, aligned(1)))
    #endif
    #ifndef   __PACKED_STRUCT
        #define __PACKED_STRUCT                        struct __attribute__((packed, aligned(1)))
    #endif
    #ifndef   __PACKED_UNION
        #define __PACKED_UNION                         union __attribute__((packed, aligned(1)))
    #endif

    /** \brief  Get CPSR Register
        \return               CPSR Register value
    */
    __STATIC_FORCEINLINE uint32_t __get_CPSR(void)
    {
        uint32_t result;
        __ASM volatile("MRS %0, cpsr" : "=r" (result) );
        return(result);
    }

    /** \brief  Get Mode
        \return                Processor Mode
    */
    __STATIC_FORCEINLINE uint32_t __get_mode(void)
    {
        return (__get_CPSR() & 0x1FU);
    }

    /**
     \brief   Breakpoint
    \details Causes the processor to enter Debug state.
            Debug tools can use this to investigate system state when the instruction at a particular address is reached.
    \param [in]    value  is ignored by the processor.
                    If required, a debugger can use it to store additional information about the breakpoint.
    */
    #define __BKPT(value)     __ASM volatile ("bkpt "#value)

#elif defined ( __GNUC__ )      /* GNU Compiler */

    #ifndef   __ALIGNED
        #define __ALIGNED(x)                           __attribute__((aligned(x)))
    #endif
    #ifndef   __ASM
        #define __ASM                                  __asm
    #endif
    #ifndef   __INLINE
        #define __INLINE                               inline
    #endif
    #ifndef   __STATIC_INLINE
        #define __STATIC_INLINE                        static inline
    #endif
    #ifndef   __STATIC_FORCEINLINE
        #define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
    #endif
    #ifndef   __NO_RETURN
        #define __NO_RETURN                            __attribute__((__noreturn__))
    #endif
    #ifndef   __USED
        #define __USED                                 __attribute__((used))
    #endif
    #ifndef   __WEAK
        #define __WEAK                                 __attribute__((weak))
    #endif
    #ifndef   __PACKED
        #define __PACKED                               __attribute__((packed, aligned(1)))
    #endif
    #ifndef   __PACKED_STRUCT
        #define __PACKED_STRUCT                        struct __attribute__((packed, aligned(1)))
    #endif
    #ifndef   __PACKED_UNION
        #define __PACKED_UNION                         union __attribute__((packed, aligned(1)))
    #endif

    /** \brief  Get CPSR Register
        \return               CPSR Register value
    */
    __STATIC_FORCEINLINE uint32_t __get_CPSR(void)
    {
        uint32_t result;
        __ASM volatile("MRS %0, cpsr" : "=r" (result) );
        return(result);
    }

    /** \brief  Get Mode
        \return                Processor Mode
    */
    __STATIC_FORCEINLINE uint32_t __get_mode(void)
    {
        return (__get_CPSR() & 0x1FU);
    }

    /**
     \brief   Breakpoint
    \details Causes the processor to enter Debug state.
            Debug tools can use this to investigate system state when the instruction at a particular address is reached.
    \param [in]    value  is ignored by the processor.
                    If required, a debugger can use it to store additional information about the breakpoint.
    */
    #define __BKPT(value)                       __ASM volatile ("bkpt "#value)

#elif defined ( __ICCARM__ )    /* IAR Compiler */

    #ifndef __ALIGNED
        #if __ICCARM_V8
            #define __ALIGNED(x) __attribute__((aligned(x)))
        #elif (__VER__ >= 7080000)
            /* Needs IAR language extensions */
            #define __ALIGNED(x) __attribute__((aligned(x)))
        #else
            #warning No compiler specific solution for __ALIGNED.__ALIGNED is ignored.
            #define __ALIGNED(x)
        #endif
    #endif

    #ifndef __ASM
        #define __ASM __asm
    #endif

    #ifndef __INLINE
        #define __INLINE inline
    #endif

    #ifndef   __STATIC_INLINE
        #define __STATIC_INLINE       static inline
    #endif

    #ifndef   __STATIC_FORCEINLINE
        #define __STATIC_FORCEINLINE  __FORCEINLINE __STATIC_INLINE
    #endif

    #ifndef   __NO_RETURN
        #if __ICCARM_V8
            #define __NO_RETURN __attribute__((__noreturn__))
        #else
            #define __NO_RETURN _Pragma("object_attribute=__noreturn")
        #endif
    #endif

    #ifndef   __USED
        #if __ICCARM_V8
            #define __USED          __attribute__((used))
        #else
            #define __USED          _Pragma("__root")
        #endif
    #endif

    #ifndef   __WEAK
        #if __ICCARM_V8
            #define __WEAK          __attribute__((weak))
        #else
            #define __WEAK          _Pragma("__weak")
        #endif
    #endif

    #ifndef   __PACKED
        #if __ICCARM_V8
            #define __PACKED __attribute__((packed, aligned(1)))
        #else
            /* Needs IAR language extensions */
            #define __PACKED __packed
        #endif
    #endif

    #ifndef   __PACKED_STRUCT
        #if __ICCARM_V8
            #define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
        #else
            /* Needs IAR language extensions */
            #define __PACKED_STRUCT __packed struct
        #endif
    #endif

    #ifndef   __PACKED_UNION
        #if __ICCARM_V8
            #define __PACKED_UNION union __attribute__((packed, aligned(1)))
        #else
            /* Needs IAR language extensions */
            #define __PACKED_UNION __packed union
        #endif
    #endif

    #define __get_CPSR()                (__arm_rsr("CPSR"))
    #define __get_mode()                (__get_CPSR() & 0x1FU)
    #define __BKPT(value)               __asm volatile ("BKPT     %0" : : "i"(value))

#else
  #error Unknown compiler.
#endif

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __CMSIS_COMPILER_H */
