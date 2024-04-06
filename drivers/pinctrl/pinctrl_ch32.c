/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ch32_clock.h>
#include <zephyr/drivers/clock_control/ch32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <dt-bindings/pinctrl/ch32-afio.h>
#include <ch32v30x_gpio.h>

/** Maximum 10MHz */
#define CH32_OSPEED_10MHZ 1U
/** Maximum 2MHz */
#define CH32_OSPEED_2MHZ 2U
/** Maximum 50MHz */
#define CH32_OSPEED_50MHZ 3U
/** Maximum speed */
#define CH32_OSPEED_MAX 3U

/** GPIO mode: input floating (CTL bits) */
#define GPIO_MODE_INP_FLOAT 0x4U
/** GPIO mode: input with pull-up/down (CTL bits) */
#define GPIO_MODE_INP_PUPD 0x8U
/** GPIO mode: output push-pull (CTL bits) */
#define GPIO_MODE_ALT_PP 0x8U
/** GPIO mode: output open-drain (CTL bits) */
#define GPIO_MODE_ALT_OD 0xCU

/** Utility macro that expands to the GPIO port address if it exists */
#define CH32_PORT_ADDR_OR_NONE(nodelabel)				       \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),		       \
		   (DT_REG_ADDR(DT_NODELABEL(nodelabel)),), ())

/** Utility macro that expands to the GPIO clock id if it exists */
#define CH32_PORT_CLOCK_ID_OR_NONE(nodelabel)				       \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),		       \
		   (DT_CLOCKS_CELL(DT_NODELABEL(nodelabel), id),), ())

/** CH32 port addresses */
static const uint32_t ch32_port_addrs[] = {
	CH32_PORT_ADDR_OR_NONE(gpioa)
	CH32_PORT_ADDR_OR_NONE(gpiob)
	CH32_PORT_ADDR_OR_NONE(gpioc)
	CH32_PORT_ADDR_OR_NONE(gpiod)
	CH32_PORT_ADDR_OR_NONE(gpioe)
};

/** CH32 port clock identifiers */
static const uint16_t ch32_port_clkids[] = {
	CH32_PORT_CLOCK_ID_OR_NONE(gpioa)
	CH32_PORT_CLOCK_ID_OR_NONE(gpiob)
	CH32_PORT_CLOCK_ID_OR_NONE(gpioc)
	CH32_PORT_CLOCK_ID_OR_NONE(gpiod)
	CH32_PORT_CLOCK_ID_OR_NONE(gpioe)
};

/**
 * @brief Initialize AFIO
 *
 * This function enables AFIO clock and configures the I/O compensation if
 * available and enabled in Devicetree.
 *
 * @retval 0 Always
 */
static int ch32_pinctrl_init(void)
{
	struct ch32_pclken pclken;
	pclken.bus = (CH32_CLOCK_AFIO>>6) + CH32_RCC_BASE;
	pclken.enr = CH32_CLOCK_AFIO & 0x3F;
	(void)clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       &pclken);
	return 0;
}
SYS_INIT(ch32_pinctrl_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


/**
 * @brief Configure a pin.
 *
 * @param pin The pin to configure.
 */
static void configure_pin(pinctrl_soc_pin_t pin)
{
	uint32_t port_idx, port;
	uint16_t clkid;
	struct ch32_pclken ch32_pclk;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_TypeDef *GPIOx;

	port_idx = CH32_PORT_GET(pin);
	__ASSERT_NO_MSG(port_idx < ARRAY_SIZE(ch32_port_addrs));

	clkid = ch32_port_clkids[port_idx];
	ch32_pclk.bus = (clkid >> 6U) + CH32_RCC_BASE;
	ch32_pclk.enr = clkid & 0x3FU;
	/* get GPIOx port pclk base addr and bits*/

	port = ch32_port_addrs[port_idx];

	GPIOx = (GPIO_TypeDef*)(port);
	GPIO_InitStructure.GPIO_Pin = 1<<CH32_PIN_GET(pin);
	GPIO_InitStructure.GPIO_Speed = CH32_SPD_GET(pin);
	GPIO_InitStructure.GPIO_Mode = CH32_MODE_GET(pin);
	
	(void)clock_control_on(CH32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&ch32_pclk);
	
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief Configure remap.
 *
 * @param remap Remap bit field as encoded by #CH32_REMAP.
 */
static void configure_remap(uint16_t remap)
{
	uint32_t gpio_remap;
	uint32_t remap_off;
	uint32_t remap_bits;
	uint32_t remap_value;

	if (remap & 0x1)
	{
		gpio_remap = 0;
		remap_off = remap >> (CH32_RMP_OFF_POS - CH32_RMP_POS);
		if(remap_off)
			gpio_remap |= 0x80000000;
		remap_bits = remap >> (CH32_RMP_BITS_POS - CH32_RMP_POS);
		remap_value = remap >> (CH32_RMP_RM_POS - CH32_RMP_POS);
		gpio_remap |= remap_value << remap_bits;
		GPIO_PinRemapConfig(gpio_remap, ENABLE);
	}

}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);
	if (pin_cnt == 0U) {
		return -EINVAL;
	}

	/* same remap is encoded in all pins, so just pick the first */
	configure_remap(CH32_RMP_GET(pins[0]));

	/* configure all pins */
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		configure_pin(pins[i]);
	}
	
	return 0;
}