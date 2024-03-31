/*
 * Copyright (c) 2022 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_CH32_RESET_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_CH32_RESET_H_

#define CH32_RESET_BUS_APB1 0x10
#define CH32_RESET_BUS_APB2 0x0C

/**
 * Pack RCC register offset and bit in one 32-bit value.
 *
 * 5 LSBs are used to keep bit number in 32-bit RCC register.
 * Next 12 bits are used to keep RCC register offset.
 * Remaining bits are unused.
 *
 * @param bus STM32 bus name (expands to CH32_RESET_BUS_{bus})
 * @param bit Reset bit
 */
#define CH32_RESET(bus, bit) (((CH32_RESET_BUS_##bus) << 5U) | (bit))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RESET_CH32_RESET_H_ */
