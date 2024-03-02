/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32_CLOCKS_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32_CLOCKS_H_

/**
 * @name Register offsets
 * @{
 */
#define CH32_AHB_EN             0x14U
#define CH32_APB1_EN            0x1CU
#define CH32_APB2_EN            0x18U

#define CH32_APB2_RST           0x0CU
#define CH32_APB1_RST           0x10U

/**
 * @name Clock enable/disable definitions for peripherals
 * @{
 */

/* AHB peripherals */
#define CH32_CLOCK_DMA1         (0U)
#define CH32_CLOCK_DMA2         (1U)
#define CH32_CLOCK_SRAM         (2U)
#define CH32_CLOCK_CRC          (6U)
#define CH32_CLOCK_FSMC         (8U)
#define CH32_CLOCK_RNG          (9U)
#define CH32_CLOCK_SDIO         (10U)
#define CH32_CLOCK_USBHS        (11U)
#define CH32_CLOCK_USBFS        (12U)
#define CH32_CLOCK_DVP          (13U)
#define CH32_CLOCK_ETHMAC       (14U)
#define CH32_CLOCK_ETHMACTX     (15U)
#define CH32_CLOCK_ETHMACRX     (16U)
#define CH32_CLOCK_BLEC         (16U)
#define CH32_CLOCK_BLES         (17U)

/* APB1 peripherals */
#define CH32_CLOCK_TIM2         (0U)
#define CH32_CLOCK_TIM3         (1U)
#define CH32_CLOCK_TIM4         (2U)
#define CH32_CLOCK_TIM5         (3U)
#define CH32_CLOCK_TIM6         (4U)
#define CH32_CLOCK_TIM7         (5U)
#define CH32_CLOCK_USART6       (6U)
#define CH32_CLOCK_USART7       (7U)
#define CH32_CLOCK_USART8       (8U)
#define CH32_CLOCK_WWDG         (11U)
#define CH32_CLOCK_SPI2         (14U)
#define CH32_CLOCK_SPI3         (15U)
#define CH32_CLOCK_USART2       (17U)
#define CH32_CLOCK_USART3       (18U)
#define CH32_CLOCK_USART4       (19U)
#define CH32_CLOCK_USART5       (20U)
#define CH32_CLOCK_I2C1         (21U)
#define CH32_CLOCK_I2C2         (22U)
#define CH32_CLOCK_USBD         (23U)
#define CH32_CLOCK_CAN1         (25U)
#define CH32_CLOCK_CAN2         (26U)
#define CH32_CLOCK_BKP          (27U)
#define CH32_CLOCK_PWR          (28U)
#define CH32_CLOCK_DAC          (29U)


/* APB2 peripherals */
#define CH32_CLOCK_AFIO         (0U)
#define CH32_CLOCK_GPIOA        (2U)
#define CH32_CLOCK_GPIOB        (3U)
#define CH32_CLOCK_GPIOC        (4U)
#define CH32_CLOCK_GPIOD        (5U)
#define CH32_CLOCK_GPIOE        (6U)
#define CH32_CLOCK_ADC1         (9U)
#define CH32_CLOCK_ADC2         (10U)
#define CH32_CLOCK_TIM1         (11U)
#define CH32_CLOCK_SPI1         (12U)
#define CH32_CLOCK_TIM8         (13U)
#define CH32_CLOCK_USART1       (14U)
#define CH32_CLOCK_TIM9         (19U)
#define CH32_CLOCK_TIM10        (20U)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32_CLOCKS_H_ */
