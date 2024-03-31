/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_uart

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include <zephyr/drivers/clock_control/ch32_clock_control.h>
#include <ch32v30x_usart.h>


struct uart_ch32_config {
	/* USART instance */
	USART_TypeDef *uart;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
	/* clock subsystem driving this peripheral */
	const struct ch32_pclken pclken;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_ch32_data {
	/* clock device */
	const struct device *clock;
	/* uart config */
	struct uart_config *uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_ch32_isr(const struct device *dev)
{
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_ch32_init(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;
	struct ch32_uart_data *const data = dev->data;
	uint32_t word_length;
	uint32_t parity;
	int ret;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int uart_ch32_poll_in(const struct device *dev, unsigned char *c)
{
	return 0;
}

static void uart_ch32_poll_out(const struct device *dev, unsigned char c)
{
}

static int uart_ch32_err_check(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	int errors = 0;
	return errors;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int uart_ch32_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	const struct ch32_uart_config *const cfg = dev->config;
	uint8_t num_tx = 0U;

	return num_tx;
}

int uart_ch32_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	const struct ch32_uart_config *const cfg = dev->config;
	uint8_t num_rx = 0U;

	return num_rx;
}

void uart_ch32_irq_tx_enable(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//usart_interrupt_enable(cfg->reg, USART_INT_TC);
}

void uart_ch32_irq_tx_disable(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//usart_interrupt_disable(cfg->reg, USART_INT_TC);
}

int uart_ch32_irq_tx_ready(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	// return usart_flag_get(cfg->reg, USART_FLAG_TBE) &&
	//        usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TC);
	return 0;
}

int uart_ch32_irq_tx_complete(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//return usart_flag_get(cfg->reg, USART_FLAG_TC);
	return 0;
}

void uart_ch32_irq_rx_enable(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//usart_interrupt_enable(cfg->reg, USART_INT_RBNE);
}

void uart_ch32_irq_rx_disable(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//usart_interrupt_disable(cfg->reg, USART_INT_RBNE);
}

int uart_ch32_irq_rx_ready(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//return usart_flag_get(cfg->reg, USART_FLAG_RBNE);
	return 0;
}

void uart_ch32_irq_err_enable(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//usart_interrupt_enable(cfg->reg, USART_INT_ERR);
	//usart_interrupt_enable(cfg->reg, USART_INT_PERR);
}

void uart_ch32_irq_err_disable(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	//usart_interrupt_disable(cfg->reg, USART_INT_ERR);
	//usart_interrupt_disable(cfg->reg, USART_INT_PERR);
}

int uart_ch32_irq_is_pending(const struct device *dev)
{
	const struct ch32_uart_config *const cfg = dev->config;

	// return ((usart_flag_get(cfg->reg, USART_FLAG_RBNE) &&
	// 	 usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_RBNE)) ||
	// 	(usart_flag_get(cfg->reg, USART_FLAG_TC) &&
	// 	 usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TC)));
	return 0;
}

void uart_ch32_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	struct ch32_usart_data *const data = dev->data;

	data->user_cb = cb;
	data->user_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_ch32_driver_api = {
	.poll_in = uart_ch32_poll_in,
	.poll_out = uart_ch32_poll_out,
	.err_check = uart_ch32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ch32_fifo_fill,
	.fifo_read = uart_ch32_fifo_read,
	.irq_tx_enable = uart_ch32_irq_tx_enable,
	.irq_tx_disable = uart_ch32_irq_tx_disable,
	.irq_tx_ready = uart_ch32_irq_tx_ready,
	.irq_tx_complete = uart_ch32_irq_tx_complete,
	.irq_rx_enable = uart_ch32_irq_rx_enable,
	.irq_rx_disable = uart_ch32_irq_rx_disable,
	.irq_rx_ready = uart_ch32_irq_rx_ready,
	.irq_err_enable = uart_ch32_irq_err_enable,
	.irq_err_disable = uart_ch32_irq_err_disable,
	.irq_is_pending = uart_ch32_irq_is_pending,
	.irq_callback_set = uart_ch32_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define UART_CH32_IRQ_HANDLER_DECL(n) /* Not used */
#define UART_CH32_IRQ_HANDLER(n) /* Not used */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_CH32_INIT(n) \
	static struct uart_ch32_data uart_ch32_data##n; \
	static const struct uart_ch32_config uart_ch32_config##n; \
	DEVICE_DT_INST_DEFINE(n, &uart_ch32_init, \
			      NULL,						\
			      &uart_ch32_data##n,				\
			      &uart_ch32_config##n, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &uart_ch32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_CH32_INIT)