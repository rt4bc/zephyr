/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 * Contributors: 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__initialize)
GTEXT(_irq_vector_table)
GTEXT(_isr_wrapper)
SECTION_FUNC(vectors, __start)
#if defined(CONFIG_RISCV_GP)
	/* Initialize global pointer */
	.option push
	.option norelax
	la gp, __global_pointer$
	.option pop
#endif

	.option norvc;

	la t0, _irq_vector_table
	addi t0, t0, 0x3
	/* MTVEC Mode1 = 0, identify as jump funciton, limited range, support no-jump code
	               = 1, Identify by absolute address, support full range, but must jump */
	/* MTVEC Mode0 = 0  : Use of the uniform entry address.
	               = 1, Address offset based on interrupt number *4.*/
	csrw mtvec, t0

	/* Jump to __reset */
	tail __reset

SECTION_FUNC(vectors, _irq_vector_table)
	.word   __start				/* 0 */
    .word   0
    .word   _isr_wrapper       	/* 2 NMI */
    .word   _isr_wrapper       	/* 3 Hard Fault */
    .word   0
    .word   _isr_wrapper       	/* 5 Ecall M Mode */
    .word   0
    .word   0
    .word   _isr_wrapper       	/* Ecall U Mode */
    .word   _isr_wrapper       	/* Break Point */
    .word   0
    .word   0
    .word   _isr_wrapper       	/* SysTick */
    .word   0
    .word   _isr_wrapper       	/* SW */
    .word   0					/* 15 */
	.rept CONFIG_NUM_IRQS
	.word _isr_wrapper
	.endr