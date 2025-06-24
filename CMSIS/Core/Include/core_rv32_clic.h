/*
 * NVIC API:  Copyright (c) 2009-2024 Arm Limited. All rights reserved.
 * CLIC Code: Copyright (c) 2024 Codasip s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * CMSIS Codasip RISC-V RV32 with CLIC Core Peripheral Access Layer Header File
 */

/* ToDo TEST THIS & COVERT INIT FN (codasip_clic_init) TO NVIC? */

#if defined (__clang__)
  #pragma clang system_header                   /* treat file as system include file */
#elif defined ( __GNUC__ )
  #pragma GCC diagnostic ignored "-Wpedantic"   /* disable pedantic warning due to unnamed structs/unions */
#endif

#ifndef __CORE_RV32_CLIC_H_GENERIC
#define __CORE_RV32_CLIC_H_GENERIC

#include <stdint.h>
#include "codasip_clic.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  \page CMSIS_MISRA_Exceptions  MISRA-C:2004 Compliance Exceptions
  CMSIS violates the following MISRA-C:2004 rules:

   \li Required Rule 8.5, object/function definition in header file.<br>
     Function definitions in header files are used to allow 'inlining'.

   \li Required Rule 18.4, declaration of union type or object of union type: '{...}'.<br>
     Unions are used for effective representation of core registers.

   \li Advisory Rule 19.7, Function-like macro defined.<br>
     Function-like macros are used to allow more efficient code.
 */


/*******************************************************************************
 *                 CMSIS definitions
 ******************************************************************************/
/**
  \ingroup Codasip_RV32_CLIC
  @{
 */

#include "cmsis_version.h"
#include "platform.h"

/* CMSIS RV32_CLIC definitions */

#ifndef __CODASIP_RV32_CLIC
#define __CODASIP_RV32_CLIC         L110_EAGLE    /*!< Codasip RV32_CLIC Default Core version */
#endif

#define __CHECK_DEVICE_DEFINES
#define __Vendor_SysTickConfig    1U


/** __FPU_USED indicates whether an FPU is used or not.
    This core does not support an FPU
*/
#define __FPU_USED       0U

#include "cmsis_compiler.h"               /* CMSIS compiler specific defines */


#ifdef __cplusplus
}
#endif

#endif /* __CORE_RV32_CLIC_H_GENERIC */

#ifndef __CMSIS_GENERIC

#ifndef __CORE_RV32_CLIC_H_DEPENDANT
#define __CORE_RV32_CLIC_H_DEPENDANT

#ifdef __cplusplus
 extern "C" {
#endif

/* check device defines and use defaults */
#if defined __CHECK_DEVICE_DEFINES
//  #ifndef __CM0_REV
//    #define __CM0_REV               0x0000U
//    #warning "__CM0_REV not defined in device header file; using default!"
//  #endif

//  #ifndef __NVIC_PRIO_BITS
//    #define __NVIC_PRIO_BITS          2U
//    #warning "__NVIC_PRIO_BITS not defined in device header file; using default!"
//  #endif

  #ifndef CONFIG_CORE_CLICINTCTLBITS
    #ifdef CLICINTCTLBITS
      #define CONFIG_CORE_CLICINTCTLBITS (CLICINTCTLBITS)
    #else
      #define CONFIG_CORE_CLICINTCTLBITS          3U
      #warning "CONFIG_CORE_CLICINTCTLBITS not defined in device header file; using default!"
    #endif
  #endif

  #ifndef __Vendor_SysTickConfig
    #define __Vendor_SysTickConfig    0U
    #warning "__Vendor_SysTickConfig not defined in device header file; using default!"
  #endif

  /* Set the CLIC configuration from above info */
  #ifndef CLIC_ADDR
    #error Please set the CLIC_ADDR
  #endif

  #ifndef CONFIG_CORE_CLIC_NUM_INTERRUPT
    #ifdef CLIC_NUM_INTERRUPT
      #define CONFIG_CORE_CLIC_NUM_INTERRUPT (CLIC_NUM_INTERRUPT)
    #else
      #define CONFIG_CORE_CLIC_NUM_INTERRUPT 16
      #warning "CONFIG_CORE_CLIC_NUM_INTERRUPT not defined in device header file; using default!"
    #endif
  #endif
#endif


#ifndef CLIC_TRIG_DEFAULT
#define CLIC_TRIG_DEFAULT   CLIC_TRIG_EDGE_POS /* Set to one of: CLIC_TRIG_LEVEL_HI, CLIC_TRIG_EDGE_POS, CLIC_TRIG_LEVEL_LOW or CLIC_TRIG_EDGE_NEG */
#endif

/* Convert CONFIG_CORE_CLICINTCTLBITS in to __NVIC_PRIO_BITS */
#define __NVIC_PRIO_BITS    (CONFIG_CORE_CLICINTCTLBITS)


typedef enum IRQn
{
/* -------------------  Processor Exceptions Numbers  ----------------------------- */
// ARM CM0:
//NonMaskableInt_IRQn           = -14,     /*  2 Non Maskable Interrupt */
//HardFault_IRQn                = -13,     /*  3 HardFault Interrupt */
//MemoryManagement_IRQn         = -12,     /*  4 Memory Management Interrupt */
//BusFault_IRQn                 = -11,     /*  5 Bus Fault Interrupt */
//UsageFault_IRQn               = -10,     /*  6 Usage Fault Interrupt */
//SecureFault_IRQn              =  -9,     /*  7 Secure Fault Interrupt */
//SVCall_IRQn                   =  -5,     /* 11 SVC Interrupt */
//DebugMonitor_IRQn             =  -4,     /* 12 Debug Monitor Interrupt */
//PendSV_IRQn                   =  -2,     /* 14 Pend SV Interrupt */
//SysTick_IRQn                  =  -1,     /* 15 System Tick Interrupt */

/* -------------------  Processor Interrupt Numbers  ------------------------------ */
//Interrupt0_IRQn               =   0,      /* RISC-V ACLINT MSWI */
  SysTick_IRQn                  =  CLINT_MTIMER_IRQ,                    /* ACLINT MTimer SysTick_IRQn */
  PendSV_IRQn                   =  CONFIG_CORE_CLIC_NUM_INTERRUPT - 1,  /* Software trigger lowest priority deferred interrupt, use the last (hopefully) unused IRQ line */

  Interrupt2_IRQn               =   2,
  Interrupt3_IRQn               =   3,
  Interrupt4_IRQn               =   4,
  Interrupt5_IRQn               =   5,
  Interrupt6_IRQn               =   6,
  Interrupt7_IRQn               =   7,
  Interrupt8_IRQn               =   8,
  Interrupt9_IRQn               =   9
  /* Interrupts 10 .. (CONFIG_CORE_CLIC_NUM_INTERRUPT - 1) are left out */
} IRQn_Type;

/* ##########################   NVIC functions  #################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_NVICFunctions NVIC Functions
  \brief    Functions that manage interrupts and exceptions via the NVIC.
  @{
 */

#ifdef CMSIS_NVIC_VIRTUAL
  #ifndef CMSIS_NVIC_VIRTUAL_HEADER_FILE
    #define CMSIS_NVIC_VIRTUAL_HEADER_FILE "cmsis_nvic_virtual.h"
  #endif
  #include CMSIS_NVIC_VIRTUAL_HEADER_FILE
#else
  #define NVIC_SetPriorityGrouping    __NVIC_SetPriorityGrouping
  #define NVIC_GetPriorityGrouping    __NVIC_GetPriorityGrouping
  #define NVIC_EnableIRQ              __NVIC_EnableIRQ
  #define NVIC_GetEnableIRQ           __NVIC_GetEnableIRQ
  #define NVIC_DisableIRQ             __NVIC_DisableIRQ
  #define NVIC_GetPendingIRQ          __NVIC_GetPendingIRQ
  #define NVIC_SetPendingIRQ          __NVIC_SetPendingIRQ
  #define NVIC_ClearPendingIRQ        __NVIC_ClearPendingIRQ
/*        NVIC_GetActive              not available for RISC-V CLIC */
  #define NVIC_SetPriority            __NVIC_SetPriority
  #define NVIC_GetPriority            __NVIC_GetPriority
/*        NVIC_SystemReset            not available for RISC-V CLIC */

  /* Aditional Functions for CLIC */
  #define NVIC_SetPriorityTrig        __NVIC_SetPriorityTrig
#endif /* CMSIS_NVIC_VIRTUAL */

#ifdef CMSIS_VECTAB_VIRTUAL
  #ifndef CMSIS_VECTAB_VIRTUAL_HEADER_FILE
    #define CMSIS_VECTAB_VIRTUAL_HEADER_FILE "cmsis_vectab_virtual.h"
  #endif
  #include CMSIS_VECTAB_VIRTUAL_HEADER_FILE
#else
  #define NVIC_SetVector              __NVIC_SetVector
  #define NVIC_GetVector              __NVIC_GetVector
#endif  /* (CMSIS_VECTAB_VIRTUAL) */

/* NVIC programming is confusing, see:
 * https://community.arm.com/arm-community-blogs/b/embedded-blog/posts/cutting-through-the-confusion-with-arm-cortex-m-interrupt-priorities
 *
 * For ARM Priority Grouping see:
 * https://developer.arm.com/documentation/dui0646/c/The-Cortex-M7-Processor/Exception-model/Interrupt-priority-grouping#:~:text=To%20increase%20priority%20control%20in,a%20subpriority%20within%20the%20group.
 *
 * ARM Interrupt priority grouping
 * ===============================
 * To increase priority control in systems with interrupts, the NVIC supports priority grouping.
 * This divides each interrupt priority register entry into two fields:
 *
 * An upper field that defines the group priority.
 *
 * A lower field that defines a subpriority within the group.
 *
 * Only the group priority determines preemption of interrupt exceptions.
 * When the processor is executing an interrupt exception handler, another interrupt with the same
 * group priority as the interrupt being handled does not preempt the handler,
 *
 * If multiple pending interrupts have the same group priority, the subpriority field determines
 * the order in which they are processed. If multiple pending interrupts have the same group
 * priority and subpriority, the interrupt with the lowest IRQ number is processed first.
 *
 * From: https://developer.arm.com/documentation/dui0646/c/Cortex-M7-Peripherals/System-control-block/Application-Interrupt-and-Reset-Control-Register?lang=en
 *
 * Binary point
 * ============
 * The PRIGROUP field indicates the position of the binary point that splits the PRI_n fields in the
 * Interrupt Priority Registers into separate group priority and subpriority fields. The following Table
 * shows how the PRIGROUP value controls this split. Implementations having PRI_n fields of less
 * than 8 bits treat the least-significant bits as zero.
 *
 * PRIGROUP  Binary point[a]  Group priority bits  Subpriority bits  Group priorities[b]  Subpriorities[b]
 * 0b000[c]  bxxxxxxx.y       [7:1]                  [0]             128                    2
 * 0b001[c]  bxxxxxx.yy       [7:2]                [1:0]              64                    4
 * 0b010[c]  bxxxxx.yyy       [7:3]                [2:0]              32                    8
 * 0b011[c]  bxxxx.yyyy       [7:4]                [3:0]              16                   16
 * 0b100     bxxx.yyyyy       [7:5]                [4:0]               8                   32
 * 0b101     bxx.yyyyyy       [7:6]                [5:0]               4                   64
 * 0b110     bx.yyyyyyy       [7]                  [6:0]               2                  128
 * 0b111     b.yyyyyyyy       None                 [7:0]               1                  256
 *
 * [a] PRI_n[7:0] field showing the binary point. x denotes a group priority field bit,
 * and y denotes a subpriority field bit.
 *
 * [b] Implementation defined.
 *
 * [c] Optional.
 */




/**
  \brief   Set Priority Grouping
  \details Sets the priority grouping field.
           The parameter PriorityGroup is assigned to the field xcliccfg->mnlbits field.
           Only values from 0..7 are used. See Binary Point (above).
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup);


/**
  \brief   Get Priority Grouping
  \details Reads the priority grouping field from the CLIC Interrupt Controller.
  \return                Priority grouping field (7 - xcliccfg->mnlbits).
 */
uint32_t __NVIC_GetPriorityGrouping(void);


/**
  \brief   Enable Interrupt
  \details Enables a device specific interrupt in the CLIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        codasip_clic_irq_enable(IRQn);
    }
}


/**
  \brief   Get Interrupt Enable status
  \details Returns a device specific interrupt enable status from the CLIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt is not enabled.
  \return             1  Interrupt is enabled.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        return codasip_clic_irq_is_enabled(IRQn);
    }
    else
    {
        return(0U);
    }
}


/**
  \brief   Disable Interrupt
  \details Disables a device specific interrupt in the CLIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        codasip_clic_irq_disable(IRQn);
    }
}


/**
  \brief   Get Pending Interrupt
  \details Reads the CLIC pending register and returns the pending bit for the specified device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        return (uint32_t) codasip_clic_irq_get_pending(IRQn);
    }
    else
    {
        return(0U);
    }
}


/**
  \brief   Set Pending Interrupt
  \details Sets the pending bit of a device specific interrupt in the CLIC pending register.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        codasip_clic_irq_set_pending(IRQn);
    }
}


/**
  \brief   Clear Pending Interrupt
  \details Clears the pending bit of a device specific interrupt in the CLIC pending register.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        codasip_clic_irq_clear_pending(IRQn);
    }
}


/**
  \brief   Set Interrupt Priority
  \details Sets the priority of a device specific interrupt or a processor exception.
           The interrupt number must be positive to specify a device specific interrupt.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set, encode using NVIC_EncodePriority() first.
  \note    The priority cannot be set for any processor exception.
 */
__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        /* Set CLIC interrupt level/priority */
        (*clicint)[IRQn].clicintctl.reg = priority;
#ifdef CLIC_TRIG_DEFAULT
        (*clicint)[IRQn].clicintattr.trig = CLIC_TRIG_DEFAULT;
#endif
    }
    else
    {
        /* RISC-V Exceptions don't have priorities, so ignore */
    }
}

/**
  \brief   Set Interrupt Priority
  \details Sets the priority of a device specific interrupt or a processor exception.
           The interrupt number must be positive to specify a device specific interrupt.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set, encode using NVIC_EncodePriority() first.
  \param [in]  CLIC Trigger level or edge and 0/1 level or -ve/+ve edge.
  \note    This function has been added because the CLIC required the user to specify level or edge
           triggered and whether it's 0/1 level or -ve/+ve edge.
 */
__STATIC_INLINE void __NVIC_SetPriorityTrig(IRQn_Type IRQn, uint32_t priority, codasip_clic_trig_t trig)
{
    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        /* Set CLIC interrupt level/priority */
        (*clicint)[IRQn].clicintctl.reg = priority;
        (*clicint)[IRQn].clicintattr.trig = trig;
    }
    else
    {
        /* RISC-V Exceptions don't have priorities, so ignore */
    }
}

/**
  \brief   Get Interrupt Priority
  \details Reads the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn  Interrupt number.
  \return             Interrupt Priority.
                      Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

    if ((int32_t)(IRQn) >= 0 && (int32_t)(IRQn) < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        /* Get CLIC interrupt level/priority */
        return (uint32_t) (*clicint)[IRQn].clicintctl.reg;
    }
    else
    {
        return (uint32_t) 0;
    }
}


/**
  \brief   Encode Priority
  \details Encodes the priority for an interrupt with the given priority group,
           preemptive priority value, and subpriority value.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]     PriorityGroup  Used priority group. (Binary point position, see above, from 1 LSbit)
  \param [in]   PreemptPriority  Preemptive priority value (starting from 0 == highest priority).
  \param [in]       SubPriority  Subpriority value (starting from 0 == highest sub-priority).
  \return                        Encoded priority. Value can be used in the function \ref NVIC_SetPriority().
 */
uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority);

/**
  \brief   Decode Priority
  \details Decodes an interrupt priority value with a given priority group to
           preemptive priority value and subpriority value.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS) the smallest possible priority group is set.
  \param [in]         Priority   Priority value, which can be retrieved with the function \ref NVIC_GetPriority().
  \param [in]     PriorityGroup  Used priority group.
  \param [out] pPreemptPriority  Preemptive priority value (starting from 0 == highest priority).
  \param [out]     pSubPriority  Subpriority value (starting from 0 == highest sub-priority).
 */
void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority);


/**
  \brief   Set Interrupt Vector
  \details Sets an interrupt vector in CLIC interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
           VTOR must been relocated to SRAM before.
  \param [in]   IRQn      Interrupt number
  \param [in]   vector    Address of interrupt handler function, NULL = set IRQn to non-vectored
 */
__STATIC_INLINE void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
    if (IRQn < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        if (IRQn >= 0)
        {
            if (vector == (uint32_t) NULL)
            {
                /* Set the interrupt non-vectored mode */
                codasip_clic_irq_set_vectored(IRQn, false, (void (*)(void)) NULL);

                (*clicint)[IRQn].clicintattr.shv = 0;
            }
            else
            {
                /* If FIXED_IRQ_HANDLERS is set, then you are using a fixed vector table.
                 * Note: codasip_clic_irq_set_vectored() (and __NVIC_SetVector()) will not change
                 * the vector address, instead see interrupt_vector.S.
                 */

                /* Save the interrupt handler function pointer in the interrupt vector table */
                codasip_clic_irq_set_vectored(IRQn, true, (void (*)(void)) vector);
            }
        }
        else
        {
            /* ToDo exception */
        }
    }
}


/**
  \brief   Get Interrupt Vector
  \details Reads an interrupt vector from interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn      Interrupt number.
  \return                 Address of interrupt handler function
 */
__STATIC_INLINE uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
    if (IRQn < CONFIG_CORE_CLIC_NUM_INTERRUPT)
    {
        if (IRQn >= 0)
        {
            /* Get the interrupt handler function pointer in the interrupt vector table */
            return (uint32_t) mtvt_table[IRQn];
        }
        else
        {
            /* ToDo exception */
            return (uint32_t) 0;
        }
    }
}


/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
#if 0
#error __NVIC_SystemReset() - not supported
__NO_RETURN __STATIC_INLINE void __NVIC_SystemReset(void)
{
  for(;;)                                                           /* wait until reset */
  {
    /* __NOP(); */
  }
}
#endif

/*@} end of CMSIS_Core_NVICFunctions */

/* ##########################  FPU functions  #################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_FpuFunctions FPU Functions
  \brief    Function that provides FPU type.
  @{
 */

/**
  \brief   get FPU type
  \details returns the FPU type
  \returns
   - \b  0: No FPU
   - \b  1: Single precision FPU
   - \b  2: Double + Single precision FPU
 */
__STATIC_INLINE uint32_t SCB_GetFPUType(void)
{
    return 0U;           /* No FPU */
}


/*@} end of CMSIS_Core_FpuFunctions */




/* ##################################    SysTick function  ############################################ */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_SysTickFunctions SysTick Functions
  \brief    Functions that configure the System.
  @{
 */

#if defined (__Vendor_SysTickConfig) && (__Vendor_SysTickConfig == 0U)

/* OS_Tick_Setup() is used instead of SysTick_Config() by RTX */
#error OS_Tick_Setup() is used instead of SysTick_Config() by RTX (so #define __Vendor_SysTickConfig 1)

/**
  \brief   System Tick Configuration
  \details Initializes the System Timer and its interrupt, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.
 */
__STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks)
{
    (void) ticks;

  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

#endif

/*@} end of CMSIS_Core_SysTickFunctions */


#ifdef __cplusplus
}
#endif

#endif /* __CORE_RV32_CLIC_H_DEPENDANT */

#endif /* __CMSIS_GENERIC */
