/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/arch/riscv/csr.h>

static int wch_ch32v3x_soc_init(void)
{
    /* Disable Global Interrupt */
	/* Machine mode status register bit3 MIE*/
	//__asm__("csrc mstatus, %0":: "i"(MSTATUS_MIE));

	/* Vendor defined CSR */
	/* corecfgr is mainly used to configure the 
	microprocessor pipeline, instruction prediction and 
	other related features, and generally does not need 
	to be operated. The relevant MCU products are 
	configured with default values in the startup file.
	*/
	//__asm__ volatile("li t0, 0x1f"); 
	/*corecfgr Microprocessor configuration register*/
	//__asm__ volatile("csrw 0xbc0, t0"); 

	/* Vendor defined CSR intsyscr */
	/* Enable nested and hardware stack */
	/* __asm__("li t0, 0x0b"); */
	/* intsyscr Interrupt system control register */
	/* __asm__("csrw 0x804, t0"); */

	/* Enable Global Interrupt */
	//__asm__ volatile("csrs mstatus, %0":: "i"(MSTATUS_MIE));

    return 0;
}

SYS_INIT(wch_ch32v3x_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
