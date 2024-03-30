/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>

#define R32_PFIC_INT_BASE 0xE000E000
#define R32_PFIC_IENRn (R32_PFIC_INT_BASE + 0x100)
#define R32_PFIC_IRERn (R32_PFIC_INT_BASE + 0x180)
#define R32_PFIC_ISRn  (R32_PFIC_INT_BASE + 0)

void arch_irq_enable(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (R32_PFIC_IENRn + int_grp * 4));
}

void arch_irq_disable(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (R32_PFIC_IRERn + int_grp * 4));
}

int arch_irq_is_enabled(unsigned int irq)
{
	int int_grp, int_off;
	unsigned int enabler;

	int_grp = irq / 32;
	int_off = irq % 32;

	enabler = sys_read32(R32_PFIC_ISRn + int_grp * 4);

	return (enabler & (1 << int_off)) != 0;
}

