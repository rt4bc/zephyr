/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/irq.h>
#include <zephyr/dt-bindings/clock/ch32_clock.h>
#include <zephyr/drivers/clock_control/ch32_clock_control.h>

#include <ch32v30x_spi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ch32, LOG_LEVEL_DBG);

#include "spi_context.h"

struct spi_ch32_config {
	SPI_TypeDef *spi;
	const struct ch32_pclken pclken;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	uint32_t frequency;
	uint16_t operation;
#ifdef CONFIG_SPI_CH32_INTERRUPT
	/* routine for configuring SPIP ISR */
	void (*irq_cfg_func)(const struct device *dev);
#endif

#ifdef CONFIG_SPI_CH32_DMA
	const struct spi_ch32_dma_config dma[NUM_OF_DIRECTION];
#endif
};

struct spi_ch32_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_CH32_DMA
	struct spi_gd32_dma_data dma[NUM_OF_DIRECTION];
#endif
};

/* SPI error status mask. */
#define SPI_CH32_ERR_MASK	(0x70)

static int spi_ch32_get_err(const struct spi_ch32_config *cfg)
{
	uint32_t stat = cfg->spi->STATR;

	if (stat & SPI_CH32_ERR_MASK) {
		LOG_ERR("spi%p error status detected, err = %u",
			cfg->spi, stat & (uint32_t)SPI_CH32_ERR_MASK);
		return -EIO;
	}

	return 0;
}

static bool spi_ch32_transfer_ongoing(struct spi_ch32_data *data)
{
	return spi_context_tx_on(&data->ctx) ||
	       spi_context_rx_on(&data->ctx);
}

static int spi_ch32_frame_exchange(const struct device *dev)
{
	struct spi_ch32_data *data = dev->data;
	const struct spi_ch32_config *ch32_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint16_t tx_frame = 0U, rx_frame = 0U;
	SPI_TypeDef *SPIx;

	SPIx = ch32_cfg->spi;
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == 0) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(ctx->config->operation) == 8) {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
		/* For 8 bits mode, spi will forced SPI_DATA[15:8] to 0. */
		SPI_I2S_SendData(SPIx, tx_frame);

		spi_context_update_tx(ctx, 1, 1);
	} else {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
		SPI_I2S_SendData(SPIx, tx_frame);

		spi_context_update_tx(ctx, 2, 1);
	}

	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) ==0) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		/* For 8 bits mode, spi will forced SPI_DATA[15:8] to 0. */
		rx_frame = SPI_I2S_ReceiveData(SPIx);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
		}

		spi_context_update_rx(ctx, 1, 1);
	} else {
		rx_frame = SPI_I2S_ReceiveData(SPIx);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
		}

		spi_context_update_rx(ctx, 2, 1);
	}

	return spi_ch32_get_err(ch32_cfg);
}

static int spi_ch32_configure(const struct device *dev, const struct spi_config *config)
{
	struct spi_ch32_data *data = dev->data;
	const struct spi_ch32_config *ch32_cfg = dev->config;
	uint32_t pclk_freq;
	SPI_TypeDef *SPIx;
	SPI_InitTypeDef SPI_InitStructure = {0};

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	SPIx = ch32_cfg->spi;

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	if (SPI_WORD_SIZE_GET(config->operation) == 8) {
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	} else if (SPI_WORD_SIZE_GET(config->operation) == 16) {
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	} else {
		LOG_ERR("SPI DataSize is not supported");
		return -ENOTSUP;
	}

	/*
	 * Set CPOL and CPHA.
	 * The following is how to map CH32 control register to CPOL and CPHA
	 *   CPOL    CPHA  |  CPOL    CPHA
	 *   -----------------------------
	 *    0       0    |    0       0
	 *    0       1    |    0       1
	 *    1       0    |    1       0
	 *    1       1    |    1       1
	 */
	if (config->operation & SPI_MODE_CPOL) {
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	} else {
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	}
	if (config->operation & SPI_MODE_CPHA) {
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	} else {
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	}

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	(void)clock_control_get_rate(CH32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t)&ch32_cfg->pclken, &pclk_freq);

	if ((pclk_freq / config->frequency) <= 2) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	} else if ((pclk_freq / config->frequency) <= 4) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	} else if ((pclk_freq / config->frequency) <= 8) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	} else if ((pclk_freq / config->frequency) <= 16) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	} else if ((pclk_freq / config->frequency) <= 32) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	} else if ((pclk_freq / config->frequency) <= 64) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	} else if ((pclk_freq / config->frequency) <= 128) {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	} else {
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	} else {
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	}

	// SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStructure);

	return 0;
}

static int spi_ch32_transceive_impl(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				    void *userdata)
{
	struct spi_ch32_data *data = dev->data;
	const struct spi_ch32_config *ch32_cfg = dev->config;
	int ret;
	SPI_TypeDef *SPIx;

	spi_context_lock(&data->ctx, (cb != NULL), cb, userdata, config);

	ret = spi_ch32_configure(dev, config);
	if (ret < 0) {
		goto error;
	}
	SPIx = ch32_cfg->spi;
	SPI_Cmd(SPIx, ENABLE);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

#if CONFIG_SPI_CH32_INTERRUPT
#ifdef CONFIG_SPI_CH32_DMA
#endif
#else
	do {
		ret = spi_ch32_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_ch32_transfer_ongoing(data));

#ifdef CONFIG_SPI_ASYNC
	spi_context_complete(&data->ctx, dev, ret);
#endif
#endif
error:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_ch32_transceive(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	return spi_ch32_transceive_impl(dev, config, tx_bufs, rx_bufs, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_ch32_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				     void *userdata)
{
	return spi_ch32_transceive_impl(dev, config, tx_bufs, rx_bufs, cb, userdata);
}
#endif

static int spi_ch32_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_ch32_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_ch32_driver_api = {.transceive = spi_ch32_transceive,
#ifdef CONFIG_SPI_ASYNC
							  .transceive_async =
								  spi_ch32_transceive_async,
#endif
							  .release = spi_ch32_release};

int spi_ch32_init(const struct device *dev)
{
	struct spi_ch32_data *data = dev->data;
	const struct spi_ch32_config *cfg = dev->config;
	int ret;

#ifdef CONFIG_SPI_CH32_DMA
	uint32_t ch_filter;
#endif

#ifdef CONFIG_SPI_CH32_INTERRUPT
	cfg->irq_cfg_func(dev);
#endif

	(void)clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&cfg->pclken);

	(void)reset_line_toggle_dt(&cfg->reset);
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

#ifdef CONFIG_SPI_CH32_DMA

#endif

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SPI_CH32_INTERRUPT
	cfg->irq_cfg_func(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_CH32_INTERRUPT
#define CH32_SPI_IRQ_HANDLER(idx)                                                                  \
	static void spi_ch32_config_func_##idx(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), spi_ch32_isr,           \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}
#define CH32_SPI_IRQ_HANDLER_FUNC_INIT(idx) .irq_cfg_func = spi_ch32_config_func_##idx
#else
#define CH32_SPI_IRQ_HANDLER(idx)
#define CH32_SPI_IRQ_HANDLER_FUNC_INIT(idx)
#endif

#define CH32_SPI_INIT(idx)                                                                         \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	CH32_SPI_IRQ_HANDLER(idx)                                                                  \
	static struct spi_ch32_data spi_ch32_data_##idx = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_ch32_data_##idx, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_ch32_data_##idx, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx)};                           \
	static struct spi_ch32_config spi_ch32_config_##idx = {                                    \
		.spi = (SPI_TypeDef *)DT_INST_REG_ADDR(idx),                                       \
		.reset = RESET_DT_SPEC_INST_GET(idx),                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		.pclken =                                                                          \
			{                                                                          \
				.bus = (DT_INST_CLOCKS_CELL(idx, id) >> 6U) + CH32_RCC_BASE,       \
				.enr = DT_INST_CLOCKS_CELL(idx, id) & 0x3FU,                       \
			},                                                                         \
		.operation = 0,                                                                    \
		.frequency = 0,                                                                    \
		CH32_SPI_IRQ_HANDLER_FUNC_INIT(idx)};                                              \
	DEVICE_DT_INST_DEFINE(idx, &spi_ch32_init, NULL, &spi_ch32_data_##idx,                     \
			      &spi_ch32_config_##idx, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
			      &spi_ch32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CH32_SPI_INIT)