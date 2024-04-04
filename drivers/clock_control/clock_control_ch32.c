/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_rcc

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/dt-bindings/clock/ch32_clock.h>
#include <zephyr/drivers/clock_control/ch32_clock_control.h>

#ifdef CONFIG_RISCV
#include "ch32v30x_rcc.h"

#ifdef CH32V30x_D8C
#define RCC_PLL1_MUL _CONCAT(_CONCAT(RCC_PLLMul_, PLL1_MULT), _EXTEN)
#else
#define RCC_PLL1_MUL _CONCAT(RCC_PLLMul_, PLL1_MULT)
#endif

#define RCC_SYSCLK_DIV _CONCAT(RCC_SYSCLK_Div, RCC_AHB_PRESCALER)
#define RCC_HCLK_DIV _CONCAT(RCC_HCLK_Div, RCC_APB1_PRESCALER)
#define RCC_PCLK2_DIV _CONCAT(RCC_HCLK_Div, RCC_APB2_PRESCALER)

#endif

static int clock_control_ch32_init(const struct device *dev)
{
	ErrorStatus ret;

	if (DT_NODE_HAS_STATUS(DT_PATH(clocks, pll1), okay)) {
		/*RCC_PLLSource_PREDIV1 defaut is HSE*/
		RCC_HSEConfig(RCC_HSE_ON);
		do {
			ret = RCC_WaitForHSEStartUp();
		} while (ret != READY);

		RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLL1_MUL);
		RCC_PLLCmd(ENABLE);
	}

	// configure AHB APB1 and APB2
	RCC_HCLKConfig(RCC_SYSCLK_DIV);
	RCC_PCLK1Config(RCC_HCLK_DIV);
	RCC_PCLK2Config(RCC_PCLK2_DIV);

	// switch SYSCLK clock
	if (DT_SAME_NODE(RCC_CLOCKS_SRC, PLL1_NODE)) {
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	} else if (DT_SAME_NODE(RCC_CLOCKS_SRC, CLK_HSE_NODE)) {
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
	} else {
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	}

	return 0;
}

static int clock_control_ch32_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct ch32_pclken *pclken = (struct ch32_pclken *)(sys);
	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, CH32_PERIPH_BUS_MIN, CH32_PERIPH_BUS_MAX) == 0) {
		/* Attemp to change a wrong periph clock bit */
		return -ENOTSUP;
	}
	sys_set_bit(pclken->bus, pclken->enr);
	
	return 0;
}

static int clock_control_ch32_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct ch32_pclken *pclken = (struct ch32_pclken *)(sys);
	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, CH32_PERIPH_BUS_MIN, CH32_PERIPH_BUS_MAX) == 0) {
		/* Attemp to change a wrong periph clock bit */
		return -ENOTSUP;
	}
	sys_clear_bit(pclken->bus, pclken->enr);

	return 0;
}

static enum clock_control_status clock_control_ch32_get_status(const struct device *dev,
							       clock_control_subsys_t sys)
{
	struct ch32_pclken *pclken = (struct ch32_pclken *)(sys);
	ARG_UNUSED(dev);

	if (sys_test_bit(pclken->bus, pclken->enr) != 0) {
		return CLOCK_CONTROL_STATUS_ON;
	}
	return CLOCK_CONTROL_STATUS_OFF;
}

static int clock_control_ch32_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *rate)
{
	struct ch32_pclken *pclken = (struct ch32_pclken *)(sys);
	ARG_UNUSED(dev);

	/*
	 * Get AHB Clock (= SystemCoreClock = SYSCLK/prescaler)
	 * SystemCoreClock is preferred to CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
	 * since it will be updated after clock configuration and hence
	 * more likely to contain actual clock speed
	 */
	uint32_t ahb_clock = CPU_FREQ / RCC_AHB_PRESCALER;
	uint32_t apb1_clock = ahb_clock / RCC_APB1_PRESCALER;
	uint32_t apb2_clock = ahb_clock / RCC_APB2_PRESCALER;
	switch (pclken->bus)
	{
	case (CH32_RCC_BASE + CH32_RCC_AHBEN_OFFSET):
		*rate = ahb_clock;
		break;
		
		case (CH32_RCC_BASE + CH32_RCC_APB1EN_OFFSET):
		*rate = apb1_clock;
		break;

		case (CH32_RCC_BASE + CH32_RCC_APB2EN_OFFSET):
		*rate = apb2_clock;
		break;
	
	default:
		return -ENOTSUP; 
	}

	return 0;
}

static const struct clock_control_driver_api clock_control_ch32_api = {
	.on = clock_control_ch32_on,
	.off = clock_control_ch32_off,
	.get_rate = clock_control_ch32_get_rate,
	.get_status = clock_control_ch32_get_status,
};

// BUILD_ASSERT((VCO_FREQ(pll_sys) >= PLL_VCO_FREQ_MIN) && (VCO_FREQ(pll_sys) <= PLL_VCO_FREQ_MAX)
// && 		     (VCO_FREQ(pll_sys) >= (CLOCK_FREQ_xosc / REF_DIV(pll_sys) * 16)), "pll_sys:
// vco_freq is out of range");

DEVICE_DT_INST_DEFINE(0, &clock_control_ch32_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_ch32_api);