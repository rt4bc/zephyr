/*
 * Copyright (c) 2022 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_rctl

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>

/** RCC Reset Register offset (from id field) */
#define CH32_RESET_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
/** RCC Reset configuration bit (from id field) */
#define GD32_RESET_ID_BIT(id)	 ((id) & 0x3FU)

struct reset_ch32_config {
	uint32_t base;
};

static int reset_ch32_status(const struct device *dev, uint32_t id,
			      uint8_t *status)
{
	const struct reset_ch32_config *config = dev->config;
	*status = !!sys_test_bit(config->base + CH32_RESET_ID_OFFSET(id),
				 GD32_RESET_ID_BIT(id));
	return 0;
}

static int reset_ch32_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_ch32_config *config = dev->config;
	sys_set_bit(config->base + CH32_RESET_ID_OFFSET(id),
		    GD32_RESET_ID_BIT(id));
	return 0;
}

static int reset_ch32_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_ch32_config *config = dev->config;
	sys_clear_bit(config->base + CH32_RESET_ID_OFFSET(id),
		    GD32_RESET_ID_BIT(id));
	return 0;
}

static int reset_ch32_line_toggle(const struct device *dev, uint32_t id)
{
	(void)reset_ch32_line_assert(dev, id);
	(void)reset_ch32_line_deassert(dev, id);
	return 0;
}

static const struct reset_driver_api reset_ch32_driver_api = {
	.status = reset_ch32_status,
	.line_assert = reset_ch32_line_assert,
	.line_deassert = reset_ch32_line_deassert,
	.line_toggle = reset_ch32_line_toggle,
};

static const struct reset_ch32_config reset_ch32_config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &reset_ch32_config, PRE_KERNEL_1,
		      CONFIG_RESET_INIT_PRIORITY, &reset_ch32_driver_api);
