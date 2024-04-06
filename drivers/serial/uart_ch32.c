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

#include <zephyr/dt-bindings/clock/ch32_clock.h>
#include <zephyr/drivers/clock_control/ch32_clock_control.h>

#include <ch32v30x_usart.h>

#define CH32_UART_DEFAULT_BAUDRATE 115200
#define CH32_UART_DEFAULT_PARITY UART_CFG_PARITY_NONE
#define CH32_UART_DEFAULT_STOP_BITS UART_CFG_STOP_BITS_1
#define CH32_UART_DEFAULT_DATA_BITS UART_CFG_DATA_BITS_8

struct uart_ch32_config {
	/* USART instance */
	USART_TypeDef *uart;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
	/* clock subsystem driving this peripheral */
	const struct ch32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
	/* uart config */
	struct uart_config *uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_ch32_data {
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
	const struct uart_ch32_config *const cfg = dev->config;
	int ret;
	USART_InitTypeDef USART_InitStructure;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	(void)clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), 
				(clock_control_subsys_t)&cfg->pclken);
	(void)reset_line_toggle_dt(&cfg->reset);
	
	switch (cfg->uart_cfg->parity)
	{
	case UART_CFG_PARITY_NONE:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	case UART_CFG_PARITY_ODD:
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		break;
	case UART_CFG_PARITY_EVEN:
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		break;
	default:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	}

	switch (cfg->uart_cfg->stop_bits)
	{
	case UART_CFG_STOP_BITS_0_5:
		USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
		break;
	case UART_CFG_STOP_BITS_1:
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		break;
	case UART_CFG_STOP_BITS_1_5:
		USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
		break;
	case UART_CFG_STOP_BITS_2:
		USART_InitStructure.USART_StopBits = USART_StopBits_2;
		break;
	default:
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		break;
	}

	switch (cfg->uart_cfg->data_bits)
	{
	case UART_CFG_DATA_BITS_8:
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		break;
	case UART_CFG_DATA_BITS_9:
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		break;
	default:
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		break;
	}

	switch (cfg->uart_cfg->flow_ctrl)
	{
	case UART_CFG_FLOW_CTRL_NONE:
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
		break;
	default:
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		break;
	}

	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_BaudRate = cfg->uart_cfg->baudrate;

	USART_Init((USART_TypeDef *)cfg->uart, &USART_InitStructure);
    USART_Cmd((USART_TypeDef *)cfg->uart, ENABLE);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int uart_ch32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_ch32_config *const cfg = dev->config;
	uint32_t status;

	status = USART_GetFlagStatus((USART_TypeDef *)cfg->uart, USART_FLAG_RXNE);
	if (!status) {
		return -EPERM;
	}
	*c = (unsigned char)(USART_ReceiveData((USART_TypeDef *)cfg->uart));
	return 0;
}

static void uart_ch32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_ch32_config *const cfg = dev->config;
	
	USART_SendData((USART_TypeDef *)cfg->uart, (uint16_t)(c));
	do
	{
	} while(!USART_GetFlagStatus((USART_TypeDef *)cfg->uart, USART_FLAG_TXE));
	
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
#define UART_CH32_IRQ_HANDLER_DECL(idx) /* Not used */
#define UART_CH32_IRQ_HANDLER(idx) /* Not used */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_CH32_INIT(idx) \
	PINCTRL_DT_INST_DEFINE(idx);					\
	static struct uart_ch32_data uart_ch32_data##idx; \
	static struct uart_config uart_cfg_##idx = {			\
	.baudrate  = DT_INST_PROP_OR(idx, current_speed,		\
				     CH32_UART_DEFAULT_BAUDRATE),	\
	.parity    = DT_INST_ENUM_IDX_OR(idx, parity,			\
					 CH32_UART_DEFAULT_PARITY),	\
	.stop_bits = DT_INST_ENUM_IDX_OR(idx, stop_bits,		\
					 CH32_UART_DEFAULT_STOP_BITS),	\
	.data_bits = DT_INST_ENUM_IDX_OR(idx, data_bits,		\
					 CH32_UART_DEFAULT_DATA_BITS),	\
	.flow_ctrl = DT_INST_PROP(idx, hw_flow_control)		\
					? USART_HardwareFlowControl_RTS_CTS	\
					: USART_HardwareFlowControl_None,	\
	};									\
	static const struct uart_ch32_config uart_ch32_config##idx = { \
		.uart = (USART_TypeDef *)DT_INST_REG_ADDR(idx),				\
		.reset = RESET_DT_SPEC_INST_GET(idx), 		\
		.pclken =                                   \
			{                                      	\
				.bus = (DT_INST_CLOCKS_CELL(idx, id) >> 6U) + CH32_RCC_BASE,\
				.enr = DT_INST_CLOCKS_CELL(idx, id) & 0x3FU,                \
			},  										\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx), \
		.uart_cfg = &uart_cfg_##idx, \
		};	\
	DEVICE_DT_INST_DEFINE(idx, &uart_ch32_init, \
			      NULL,						\
			      &uart_ch32_data##idx,				\
			      &uart_ch32_config##idx, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &uart_ch32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_CH32_INIT)