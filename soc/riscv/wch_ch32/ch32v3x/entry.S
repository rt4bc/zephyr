/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/arch/riscv/csr.h>

GTEXT(__ch32_entry)
SECTION_FUNC(entry, __ch32_entry)
	/* Disable Global Interrupt */
	csrc mstatus, MSTATUS_MIE

	/* Vendor defined CSR */
	/* corecfgr is mainly used to configure the 
	microprocessor pipeline, instruction prediction and 
	other related features, and generally does not need 
	to be operated. The relevant MCU products are 
	configured with default values in the startup file.
	*/
	li t0, 0x1f
	csrw 0xbc0, t0

	/* Vendor defined CSR intsyscr */
	/* Enable nested and hardware stack */
	li t0, 0x0b
	csrw 0x804, t0

	/* Enable Global Interrupt */
	csrs mstatus, MSTATUS_MIE

	/* Jump to common start */
	tail __start