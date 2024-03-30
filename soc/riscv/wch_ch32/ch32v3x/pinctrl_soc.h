/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * WCH CH32 SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_RISCV_WCH_CH32_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_RISCV_WCH_CH32_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <stdint.h>

#include <zephyr/dt-bindings/pinctrl/ch32-pinctrl.h>

/** Type for CH32 pin. */
typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function). */
	uint32_t pinmux;
	/** Pin configuration (bias, drive and slew rate). */
	uint32_t pincfg;
} pinctrl_soc_pin_t;


#endif /* ZEPHYR_SOC_RISCV_WCH_CH32_COMMON_PINCTRL_SOC_H_ */
