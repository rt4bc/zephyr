# Copyright (c)
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(entry.S)
zephyr_sources(vector_table.S)
zephyr_sources(soc.c)

zephyr_linker_sources(ROM_START SORT_KEY 0x0 entry.ld)
zephyr_linker_sources(ROM_START SORT_KEY 0x1 vector_table.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
