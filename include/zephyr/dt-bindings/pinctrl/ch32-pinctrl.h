/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CH32_PINCTRL_H_
#define CH32_PINCTRL_H_

/**
 * @brief numerical IDs for IO ports
 */

#define	CH32_PORTA 0	/* IO port A */
#define	CH32_PORTB 1	/* .. */
#define	CH32_PORTC 2
#define	CH32_PORTD 3
#define	CH32_PORTE 4	/* IO port E */

/**
 * @brief helper macro to encode an IO port pin in a numerical format
 */
#define CH32PIN(_port, _pin) \
	(_port << 4 | _pin)

/**
 * @brief Macro to generate pinmux int using port, pin number and mode arguments
 */

#define CH32_MODE_SHIFT  0U
#define CH32_MODE_MASK   0xFU
#define CH32_LINE_SHIFT  4U
#define CH32_LINE_MASK   0xFU
#define CH32_PORT_SHIFT  8U
#define CH32_PORT_MASK   0x7U
#define CH32_REMAP_SHIFT 11U
#define CH32_REMAP_MASK  0x3FFU

/**
 * @brief Pin configuration configuration bit field.
 *
 * Fields:
 *
 * - mode  [ 0 : 3 ]
 * - line  [ 4 : 7 ]
 * - port  [ 8 : 11 ]
 * - remap [ 12 : 19 ]
 *
 * @param port Port ('A'..'E')
 * @param line Pin (0..15)
 * @param mode Pin mode (ANALOG, GPIO_IN, ALTERNATE).
 * @param remap Pin remapping configuration (NO_REMAP, REMAP_1, ...)
 */
#define CH32_PINMUX(port, line, mode, remap)				       \
		(((((port) - 'A') & CH32_PORT_MASK) << CH32_PORT_SHIFT) |    \
		(((line) & CH32_LINE_MASK) << CH32_LINE_SHIFT) |	       \
		(((mode) & CH32_MODE_MASK) << CH32_MODE_SHIFT) |	       \
		(((remap) & CH32_REMAP_MASK) << CH32_REMAP_SHIFT))

/**
 * @brief Pin modes
 */

#define ALTERNATE	0x0  /* Alternate function output */
#define GPIO_IN		0x1  /* Input */
#define ANALOG		0x2  /* Analog */
#define GPIO_OUT	0x3  /* Output */

/**
 * @brief PIN configuration bitfield
 *
 * Pin configuration is coded with the following
 * fields
 *    GPIO I/O Mode       [ 0 ]
 *    GPIO Input config   [ 1 : 2 ]
 *    GPIO Output speed   [ 3 : 4 ]
 *    GPIO Output PP/OD   [ 5 ]
 *    GPIO Output AF/GP   [ 6 ]
 *    GPIO PUPD Config    [ 7 : 8 ]
 *    GPIO ODR            [ 9 ]
 *
 * Applicable to WCH CH32 series
 */

/* Port Mode */
#define CH32_MODE_INPUT		(0x0 << CH32_MODE_INOUT_SHIFT)
#define CH32_MODE_OUTPUT		(0x1 << CH32_MODE_INOUT_SHIFT)
#define CH32_MODE_INOUT_MASK		0x1
#define CH32_MODE_INOUT_SHIFT		0

/* Input Port configuration */
#define CH32_CNF_IN_ANALOG		(0x0 << CH32_CNF_IN_SHIFT)
#define CH32_CNF_IN_FLOAT		(0x1 << CH32_CNF_IN_SHIFT)
#define CH32_CNF_IN_PUPD		(0x2 << CH32_CNF_IN_SHIFT)
#define CH32_CNF_IN_MASK		0x3
#define CH32_CNF_IN_SHIFT		1

/* Output Port configuration */
#define CH32_MODE_OUTPUT_MAX_10	(0x0 << CH32_MODE_OSPEED_SHIFT)
#define CH32_MODE_OUTPUT_MAX_2		(0x1 << CH32_MODE_OSPEED_SHIFT)
#define CH32_MODE_OUTPUT_MAX_50	(0x2 << CH32_MODE_OSPEED_SHIFT)
#define CH32_MODE_OSPEED_MASK		0x3
#define CH32_MODE_OSPEED_SHIFT		3

#define CH32_CNF_PUSH_PULL		(0x0 << CH32_CNF_OUT_0_SHIFT)
#define CH32_CNF_OPEN_DRAIN		(0x1 << CH32_CNF_OUT_0_SHIFT)
#define CH32_CNF_OUT_0_MASK		0x1
#define CH32_CNF_OUT_0_SHIFT		5

#define CH32_CNF_GP_OUTPUT		(0x0 << CH32_CNF_OUT_1_SHIFT)
#define CH32_CNF_ALT_FUNC		(0x1 << CH32_CNF_OUT_1_SHIFT)
#define CH32_CNF_OUT_1_MASK		0x1
#define CH32_CNF_OUT_1_SHIFT		6

/* GPIO High impedance/Pull-up/Pull-down */
#define CH32_PUPD_NO_PULL		(0x0 << CH32_PUPD_SHIFT)
#define CH32_PUPD_PULL_UP		(0x1 << CH32_PUPD_SHIFT)
#define CH32_PUPD_PULL_DOWN		(0x2 << CH32_PUPD_SHIFT)
#define CH32_PUPD_MASK			0x3
#define CH32_PUPD_SHIFT		7

/* GPIO plain output value */
#define CH32_ODR_0			(0x0 << CH32_ODR_SHIFT)
#define CH32_ODR_1			(0x1 << CH32_ODR_SHIFT)
#define CH32_ODR_MASK			0x1
#define CH32_ODR_SHIFT			9
#endif /*CH32_PINCTRL_H_*/
