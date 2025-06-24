/* Copyright (c) 2024-2025 Codasip s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Codasip Core Level Interrupt Controller (CLIC) driver
 *        for RISC-V processors
 */

/* Useful info:
 * https://github.com/riscv/riscv-fast-interrupt/blob/5ef35cd40f809018a4a90b97fa0a7f5b4e150eba/clic.adoc
 * (https://github.com/riscv/riscv-fast-interrupt/blob/v0.9/src/clic.adoc)
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "codasip_clic.h"

/* CLIC Interrupt Vector Table */
#ifndef FIXED_IRQ_HANDLERS
void (*mtvt_table[CLIC_NUM_INTERRUPT])(void) __attribute__((aligned(64))) __attribute__((section(".mtvt_table"))) = {0};
#endif

static uint8_t clic_max_level;
static uint8_t clic_max_priority;

void codasip_clic_irq_clear_pending_call(uint32_t irq)
{
    (*clicint)[irq].clicintip.pending = 0;
}

/* Set interrupt non-vectored/vectored */
/* if vectored = true,  then you must have a valid irq_handler() (if FIXED_IRQ_HANDLERS is not defined)
 * if vectored = false, then set irq_handler to NULL.
 */
void codasip_clic_irq_set_vectored(uint32_t irq, bool vectored, void (*irq_handler)(void))
{
    (void) irq_handler;

    if (irq < CLIC_NUM_INTERRUPT)
    {
#ifdef FIXED_IRQ_HANDLERS
#warning FIXED_IRQ_HANDLERS is set, you are using a fixed vector table. Note: codasip_clic_irq_set_vectored() (and __NVIC_SetVector()) will not change the vector address, instead see interrupt_vector.S.

#else
        /* Save the interrupt handler function pointer in the interrupt vector table */
        mtvt_table[irq] = irq_handler;
#endif

        /* Set the interrupt vectored mode */
        (*clicint)[irq].clicintattr.shv = vectored;
    }
}


/* Set interrupt level */
void codasip_clic_irq_set_level(uint32_t irq, uint32_t level, codasip_clic_trig_t trig)
{
    clicintattr_t   l_clicintattr = (*clicint)[irq].clicintattr;

    /* Set the CLIC Interrupt Level (we are not using CLIC Priority) */
    if (level > clic_max_level)
    {
        level = clic_max_level;
    }
    (*clicint)[irq].clicintctl.level = level;

    /* Set the trigger mode as requested by trig (0 being the default) */
    l_clicintattr.trig = (uint8_t) (trig & CLICINTATTR_TRIG_MASK);

    /* Set the mode to Machine */
    /* l_clicintattr.mode = CLICINTATTR_MODE_MACHINE; Not required as we are in Machine only config,
     *                                                see "Interrupt Mode Table" (above) */

    /* Save the interrupt attributes */
    (*clicint)[irq].clicintattr = l_clicintattr;
}

/* Get interrupt level */
uint32_t codasip_clic_irq_get_level(uint32_t irq)
{
    return (*clicint)[irq].clicintctl.level;
}

/* Get interrupt trigger */
codasip_clic_trig_t codasip_clic_irq_get_trig(uint32_t irq)
{
    return (*clicint)[irq].clicintattr.trig;
}

/* Initialise the CLIC and Vector Table base address
 * trap_handler = non-vectored interrupt and exception handler address, needs to be at least 64-byte aligned.
 *                or NULL to leave it unchanged */
int codasip_clic_init(void (*trap_handler)(void))
{
    int i;

    /* Setup the Trap (Interrupt) Vector Table base address */
    /* csr_write(0x307, mtvt_table); */   /* 0x307 = mtvt */
    __asm__ volatile ("csrw 0x307, %0" : : "r" (mtvt_table) : "memory");

    /* Reset the CLIC */
    for (i = 0; i < CLIC_NUM_INTERRUPT; i++)
    {
        (*clicint)[i].reg = 0UL;
    }

    xcliccfg->reg = 0UL;

    /* Set all privilege modes to operate in CLIC mode:
     *
     * CLIC mode xtvec register layout
     *
     *  Bits          Field
     *  XLEN-1:6      base (WARL)
     *  5:2           submode (WARL)
     *  1:0           mode (WARL)
     *
     * (xtvec[5:0])
     * submode mode  Action on Interrupt
     *    aaaa 00    pc := OBASE                       (CLINT non-vectored basic mode)
     *    aaaa 01    pc := OBASE + 4 * exccode         (CLINT vectored basic mode)
     *
     *    0000 11                                      (CLIC mode)
     *               (non-vectored)
     *               pc := NBASE
     *               (vectored)
     *               vec := TBASE + 4 * irq
     *               pc  := load_address(vec)
     *
     *    0000 10                                      Reserved
     *    xxxx 1?    (xxxx!=0000)                      Reserved
     *
     * OBASE = xtvec[XLEN-1:2]<<2   # CLINT mode vector base is at least 4-byte aligned.           NOT USED WITH CLIC
     * NBASE = xtvec[XLEN-1:6]<<6   # CLIC mode vector base is at least 64-byte aligned.           EXCEPTION AND NON-VECTORED HANDLER ADDRESS
     * TBASE = xtvt[XLEN-1:6]<<6    # Software trap vector table base is at least 64-byte aligned. INTERRUPT VECTOR TABLE BASE ADDRESS
     */

    if (trap_handler == NULL)
    {
        /* trap_handler = (void (*)(void)) csr_read(mtvec); */
        __asm__ volatile ("csrr %0, mtvec" : "=r" (trap_handler));
    }

    trap_handler = (void (*)(void)) (((uint32_t) trap_handler & 0xFFFFFFC0) | MTVEC_CLIC_MODE);

    /* csr_write(mtvec, trap_handler); */
    __asm__ volatile ("csrw mtvec, %0" : : "r" (trap_handler) : "memory");

    /* We are going to use only CLIC Interrupt Levels (no Priority). */
    xcliccfg->mnlbits = CLICINTCTLBITS;

    clic_max_level    = (1 << CLICINTCTLBITS) - 1;
    clic_max_priority = 0;

    return 0;
}
