# Copyright (c)
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_HAS_CH32_HAL)

zephyr_library_named(hal_wch)

set(ch32_soc_dir     ${ZEPHYR_HAL_WCH_MODULE_DIR}/${CONFIG_SOC_SERIES})
zephyr_include_directories(${ch32_soc_dir}/inc)
if(CONFIG_RISCV)
zephyr_include_directories(${ch32_soc_dir}/riscv/)
if( "${CONFIG_SOC}" EQUAL "ch32v303")
target_compile_definitions(hal_wch PRIVATE CH32V30x_D8=1)
else()
target_compile_definitions(hal_wch PRIVATE CH32V30x_D8C=1)
endif()
endif()

if(CONFIG_RISCV)
zephyr_library_sources(${ch32_soc_dir}/riscv/core_riscv.c)
endif()
zephyr_library_sources_ifdef(CONFIG_DT_HAS_WCH_CH32_RCC_ENABLED ${ch32_soc_dir}/src/ch32v30x_rcc.c)
zephyr_library_sources_ifdef(CONFIG_DT_HAS_WCH_CH32_GPIO_ENABLED ${ch32_soc_dir}/src/ch32v30x_gpio.c)
zephyr_library_sources_ifdef(CONFIG_DT_HAS_WCH_CH32_USART_ENABLED ${ch32_soc_dir}/src/ch32v30x_usart.c)
endif()