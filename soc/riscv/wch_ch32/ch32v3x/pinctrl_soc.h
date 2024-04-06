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

#include <dt-bindings/pinctrl/ch32-afio.h>
#include <zephyr/dt-bindings/pinctrl/ch32-pinctrl.h>

typedef uint32_t pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)			       \
	(DT_PROP_BY_IDX(node_id, prop, idx) |				       \
	 ((DT_ENUM_IDX(node_id, slew_rate) +1) << CH32_SPD_POS) | \
	 (DT_PROP(node_id, remapping) << CH32_RMP_POS)       \
	 ),

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),		       \
				DT_FOREACH_PROP_ELEM, pinmux,		       \
				Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_RISCV_WCH_CH32_COMMON_PINCTRL_SOC_H_ */
