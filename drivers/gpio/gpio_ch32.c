/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_gpio

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/dt-bindings/pinctrl/ch32-pinctrl.h>
#include <zephyr/dt-bindings/clock/ch32_clock.h>
#include <zephyr/drivers/clock_control/ch32_clock_control.h>

#include <ch32v30x_gpio.h>

/**
 * @brief configuration of GPIO device
 */
struct gpio_ch32_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t *base;
	/* IO port */
	int port;
	struct ch32_pclken pclken;
	struct reset_dt_spec reset;
};	

/**
 * @brief driver data
 */
struct gpio_ch32_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* device's owner of this data */
	const struct device *dev;
	/* user ISR cb */
	sys_slist_t cb;
};


static inline int gpio_ch32_configure(const struct device *port, gpio_pin_t pin,
				      gpio_flags_t flags)
{	
	const struct gpio_ch32_config *config = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)config->base;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if ((flags & GPIO_OUTPUT) != 0) {
		GPIO_InitStruct.GPIO_Mode = 0x01;
	}

	if ((flags & GPIO_INPUT) != 0) {
		GPIO_InitStruct.GPIO_Mode = 0x04;
	}

	GPIO_InitStruct.GPIO_Pin |= 1<<(pin);

	GPIO_Init(gpio, &GPIO_InitStruct);

	return 0;
}

static int gpio_ch32_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_ch32_config *config = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)config->base;

	uint16_t pinval;
	pinval = GPIO_ReadInputData(gpio);
	*value = (uint32_t)pinval;
	return 0;
}

static int gpio_ch32_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	return 0;
}

static int gpio_ch32_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_ch32_config *config = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)config->base;

	GPIO_SetBits(gpio, pins);
	return 0;
}

static int gpio_ch32_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_ch32_config *config = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)config->base;

	GPIO_ResetBits(gpio, pins);
	return 0;
}

static int gpio_ch32_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_ch32_config *config = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)config->base;

	uint8_t pinval;

	pinval = GPIO_ReadOutputDataBit(gpio, pins);
	if(pinval)
	{
		GPIO_ResetBits(gpio, pins);
	}
	else
	{
		GPIO_SetBits(gpio, pins);
	}

	return 0;
}

static int gpio_ch32_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	return 0;
}

static int gpio_ch32_manage_callback(const struct device *dev, struct gpio_callback *callback,
				     bool set)
{
	// struct gpio_ch32_data *data = dev->data;
	// return gpio_manage_callback(&data->callbacks, callback, set);
	return 0;
}

static const struct gpio_driver_api gpio_ch32_api = {
	.pin_configure = gpio_ch32_configure,
	.port_get_raw = gpio_ch32_port_get_raw,
	.port_set_masked_raw = gpio_ch32_port_set_masked_raw,
	.port_set_bits_raw = gpio_ch32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ch32_port_clear_bits_raw,
	.port_toggle_bits = gpio_ch32_port_toggle_bits,
	.pin_interrupt_configure = gpio_ch32_pin_interrupt_configure,
	.manage_callback = gpio_ch32_manage_callback,
};

static int gpio_ch32_init(const struct device *port)
{
	const struct gpio_ch32_config *config = port->config;

	(void)clock_control_on(CH32_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->pclken);

	(void)reset_line_toggle_dt(&config->reset);

	return 0;
}

#define GPIO_CH32_DEFINE(n)                                                                        \
	static const struct gpio_ch32_config gpio_ch32_config##n = {                               \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.base = (uint32_t *)DT_INST_REG_ADDR(n),                                           \
		.port = n,                                                                         \
		.pclken =                                                                          \
			{                                                                          \
				.bus = (DT_INST_CLOCKS_CELL(n, id) >> 6U) + CH32_RCC_BASE,                 \
				.enr = DT_INST_CLOCKS_CELL(n, id) & 0x3FU,                \
			},                                                                         \
		.reset = RESET_DT_SPEC_INST_GET(n),									\
	};                                                                                         \
                                                                                                   \
	static struct gpio_ch32_data gpio_ch32_data##n;                                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &gpio_ch32_init, NULL, &gpio_ch32_data##n, &gpio_ch32_config##n,  \
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_ch32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_CH32_DEFINE)