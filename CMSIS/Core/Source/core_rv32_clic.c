/*
 * NVIC API:  Copyright (c) 2009-2024 Arm Limited. All rights reserved.
 * CLIC Code: Copyright (c) 2024-2025 Codasip s.r.o.
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
 * CMSIS Codasip RISC-V RV32 with CLIC Core Peripheral Access Layer
 */

#include "core_rv32_clic.h"

/**
  \brief   Set Priority Grouping
  \details Sets the priority grouping field.
           The parameter PriorityGroup is assigned to the field xcliccfg->mnlbits field.
           Only values from 0..7 are used. See Binary Point (above).
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
    uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);             /* only values 0..7 are used          */

    /* (PriorityGroup + 1) is the number of ARM Sub-Priority bits (cf CLIC Priority bits), while
     * xcliccfg->mnlbits is the number of CLIC Level bits (cf ARM Group-Priority bits). So,
     * we need to adjust the figure to count the bits from the left, not right */
    PriorityGroupTmp = 7 - PriorityGroupTmp;
    if (PriorityGroupTmp > CONFIG_CORE_CLICINTCTLBITS)
    {
        PriorityGroupTmp = CONFIG_CORE_CLICINTCTLBITS;
    }

    xcliccfg->mnlbits = PriorityGroupTmp;
}

/**
  \brief   Get Priority Grouping
  \details Reads the priority grouping field from the CLIC Interrupt Controller.
  \return                Priority grouping field (7 - xcliccfg->mnlbits).
 */
uint32_t __NVIC_GetPriorityGrouping(void)
{
    uint32_t reg_value;

    reg_value = xcliccfg->mnlbits;

    if (reg_value > 7)
    {
        reg_value = 7;
    }

    return (uint32_t)(7 - reg_value);
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
uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits; /* CLIC Level bits */
  uint32_t SubPriorityBits;     /* CLIC Priority bits */
  uint32_t PreemptPriorityMask;
  uint32_t SubPriorityMask;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = 8 - PreemptPriorityBits;

  PreemptPriorityMask = (1UL << PreemptPriorityBits) - 1UL;
  SubPriorityMask     = (1UL << SubPriorityBits    ) - 1UL;

  /* This limiting code is additional to the ARM NVIC code, which just truncates */
  if (PreemptPriority > PreemptPriorityMask)
  {
      PreemptPriority = PreemptPriorityMask;
  }

  if (SubPriority > SubPriorityMask)
  {
      SubPriority = SubPriorityMask;
  }

  /* The NVIC uses the “reversed” priority numbering scheme for interrupts and the
   * CLIC is normal (higher number == higher priority), so invert */
  PreemptPriority = PreemptPriorityMask - PreemptPriority;
  SubPriority     = SubPriorityMask     - SubPriority;

  /* When CLICINTCTLBITS == 8 and xnlbits == 8, if level == 0 then this is not an active interrupt.
   * This is not an issue when emulating the NVIC as PriorityGroup 0..7 represents binary point
   * position 1..8 for the division between PreemptPriority and SubPriority, i.e. there will
   * always be at least 1 bit of SubPriority, so level+priority will always >= 1.
   * (When xnlbits < 8, then all the levels are greater than zero since the LSBs are assumed to be 1) */

  return ( (PreemptPriority << SubPriorityBits) | SubPriority );
}

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
void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits; /* CLIC Level bits */
  uint32_t SubPriorityBits;     /* CLIC Priority bits */
  uint32_t PreemptPriorityMask;
  uint32_t SubPriorityMask;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = 8 - PreemptPriorityBits;

  PreemptPriorityMask = (1UL << PreemptPriorityBits) - 1UL;
  SubPriorityMask     = (1UL << SubPriorityBits    ) - 1UL;

  /* ~ is used as the NVIC uses the “reversed” priority numbering scheme for interrupts and the
   * CLIC is normal (higher number == higher priority), so invert */
  *pPreemptPriority = (~Priority >> SubPriorityBits) & PreemptPriorityMask;
  *pSubPriority     = (~Priority                   ) & SubPriorityMask;
}
