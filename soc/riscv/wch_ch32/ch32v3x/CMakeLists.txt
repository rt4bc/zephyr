# Copyright (c)
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(vector.S)
zephyr_sources(soc_irq.S)
zephyr_sources(soc_common_irq.c)
zephyr_sources(soc.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
