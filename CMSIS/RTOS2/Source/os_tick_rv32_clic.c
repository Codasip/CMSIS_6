/**************************************************************************//**
 * @file     os_tick_rv32_clic.c
 * @brief    CMSIS OS Tick Codasip CLIC RISC-V Port
 * @version  V1.0.0
 * @date     29. Jan 2025
 ******************************************************************************/
/*
 * Copyright (c) 2017-2021 ARM Limited. All rights reserved. (Function prototypes)
 * Copyright (c) 2025 Codasip s.r.o. All rights reserved.      (Function code)
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

#include <stdint.h>
#include "rtx_os.h"
#include "rtx_lib.h"
#include "os_tick.h"
#include "platform.h"

static uint32_t clocks_per_tick = 0;
static uint32_t tick_freq = 0;

static uint64_t timecmp_prev; /* Previous timecmp value for OS_Tick_GetCount() */

#define mtimer    ((volatile uint32_t *) ACLINT_MTIME_ADDR)
#define mtimercmp ((volatile uint32_t *) ACLINT_MTIMECMP_ADDR) /* Hart 0 */

/// See: https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS__TickAPI.html

/// Setup OS Tick timer to generate periodic RTOS Kernel Ticks
/// \param[in]     freq         tick frequency in Hz
/// \param[in]     handler      tick IRQ handler
/// \return 0 on success, -1 on error.
///
/// The timer should be configured to generate periodic interrupts at frequency specified by freq.
/// The parameter handler defines the interrupt handler function that is called.
///
/// The timer should only be initialized and configured but must not be started to create interrupts.
/// The RTOS kernel calls the function OS_Tick_Enable to start the timer interrupts.
int32_t  OS_Tick_Setup (uint32_t freq, IRQHandler_t handler)
{
    (void) handler; /* This is ignored here as it is hard coded in irq_codasip_l110.S to SysTick_Handler */

    if (freq == 0)
    {
        return -1;
    }

    tick_freq = freq;
    clocks_per_tick = TARGET_PLATFORM_FREQ / tick_freq;

    if (clocks_per_tick == 0)
    {
        return -1;
    }

    /* Get the current time */
    uint64_t timenow, timecmp;
    uint32_t timenow_l, timenow_h;

    do
    {
        timenow_h = mtimer[1];
        timenow_l = mtimer[0];
    } while(timenow_h != mtimer[1]);

    /* Add on the tick interval and save to the compare timer */
    timenow = (uint64_t) timenow_h << 32 | timenow_l;
    timecmp = timenow + clocks_per_tick;

    /* Save to compare timer, need to set low word to 0xFFFFFFFF first to prevent false triggers */
    mtimercmp[0] = 0xFFFFFFFFUL;
    mtimercmp[1] = (uint32_t) (timecmp >> 32);
    mtimercmp[0] = (uint32_t)  timecmp;

    /* CLIC trap (interrupt and exception) handler should already be initialised by SVC_Setup() which is
     * called by RTX in svcRtxKernelStart() before this function */

    /* Setup the SysTick_IRQn with CLIC level=0, priority=minimum 1, +ve edge triggered, make it non-vectored
     * This is the lowest CLIC level/priority.
     *
     * clicintctl[i] will be 0b0000000.1 when CLICINTCTLBITS==7/8 bits & mnlbits==7/8, this is to simulate the ARM
     *      NVIC minimum PRIGROUP (c.f. CLICINTCTLBITS) of 0 which splits NVIC Priority/SubPriority to 7.1 bits.
     * clicintctl[i] will be 0b000.11111 when CLICINTCTLBITS==3 bits & mnlbits==3
     *
     * See: https://developer.arm.com/documentation/ddi0337/e/Nested-Vectored-Interrupt-Controller/NVIC-programmer-s-model/NVIC-register-descriptions
     */
    uint32_t PriorityGroup = NVIC_GetPriorityGrouping(); /* This is the binary point, see core_rv32_clic.h */
    uint32_t PreemptPriority = (1UL << (7 - PriorityGroup)) - 1;
    uint32_t SubPriority     = 1;
    uint32_t EncodedPriority = NVIC_EncodePriority (PriorityGroup, PreemptPriority, SubPriority);
    NVIC_SetPriorityTrig(SysTick_IRQn, EncodedPriority, CLIC_TRIG_LEVEL_HI);
    NVIC_SetVector(SysTick_IRQn, (uint32_t) NULL);

    return 0;
}

/// Enable OS Tick timer interrupt
///
/// Enable and start the OS Tick timer to generate periodic RTOS Kernel Tick interrupts.
void     OS_Tick_Enable (void)
{
    NVIC_EnableIRQ(SysTick_IRQn);
}

/// Disable OS Tick timer interrupt
///
/// Stop the OS Tick timer and disable generation of RTOS Kernel Tick interrupts.
void     OS_Tick_Disable (void)
{
    NVIC_DisableIRQ(SysTick_IRQn);
}

/// Acknowledge execution of OS Tick timer interrupt
///
/// Acknowledge the execution of the OS Tick timer interrupt function, for example clear the pending flag.
/// Called by osRtxTick_Handler()
void     OS_Tick_AcknowledgeIRQ (void)
{
    uint64_t timecmp;

    /* Update timer compare register for the next tick */

    /* Add on the tick interval and save to the compare timer */
    timecmp  = (uint64_t) mtimercmp[1] << 32 | mtimercmp[0];
    timecmp_prev = timecmp; /* Keep the previous value for OS_Tick_GetCount() */
    timecmp += clocks_per_tick;

    /* Save new value to compare timer, need to set low word to 0xFFFFFFFF first to prevent false triggers */
    mtimercmp[0] = 0xFFFFFFFFUL;
    mtimercmp[1] = (uint32_t) (timecmp >> 32);
    mtimercmp[0] = (uint32_t)  timecmp;

    /* Acknowledge (non-vectored) IRQ by clear the pending interrupt */
    NVIC_ClearPendingIRQ(SysTick_IRQn);
}

/// Get OS Tick timer IRQ number
/// \return OS Tick IRQ number
///
/// Return the numeric value that identifies the interrupt called by the OS Tick timer.
int32_t  OS_Tick_GetIRQn (void)
{
    return SysTick_IRQn;
}

/// Get OS Tick timer clock frequency
/// \return OS Tick timer clock frequency in Hz
///
/// Return the input clock frequency of the OS Tick timer.
/// This is the increment rate of the counter value returned by the function OS_Tick_GetCount.
/// This function is used to by the function osKernelGetSysTimerFreq.
uint32_t OS_Tick_GetClock (void)
{
    return TARGET_PLATFORM_FREQ;    // ARM CM0: return SystemCoreClock;
}

/// Get OS Tick timer interval reload value
/// \return OS Tick timer interval reload value
///
/// Return the number of counter ticks between to periodic OS Tick timer interrupts.
uint32_t OS_Tick_GetInterval (void)
{
    return clocks_per_tick;
}

/// Get OS Tick timer counter value
/// \return OS Tick timer counter value
///
/// Return the current value of the OS Tick counter: 0 ... (reload value -1). The reload value is returned by the
/// function OS_Tick_GetInterval. The OS Tick timer counter value is used to by the function osKernelGetSysTimerCount.
uint32_t OS_Tick_GetCount (void)
{
    uint64_t timenow, diff;
    uint32_t timenow_l, timenow_h;

    /* Get the current time */
    do
    {
        timenow_h = mtimer[1];
        timenow_l = mtimer[0];
    } while(timenow_h != mtimer[1]);

    timenow = (uint64_t) timenow_h << 32 | timenow_l;

    /* Calculate the time since the previous compare value */
    diff  = timenow - timecmp_prev;

    return (uint32_t) diff;
}

/// Get OS Tick timer overflow status
/// \return OS Tick overflow status (1 - overflow, 0 - no overflow).
///
/// Return the state of OS Tick timer interrupt pending bit that indicates timer overflows to adjust SysTimer calculations.
/// Used by svcRtxKernelGetSysTimerCount() in rtx_kernel.c
uint32_t OS_Tick_GetOverflow (void)
{
    return NVIC_GetPendingIRQ(SysTick_IRQn);
}
