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

static int clock_control_ch32_init(const struct device *dev)
{
    #if 0
#ifdef CONFIG_DT_HAS_WCH_CH32V3X_PLL_CLOCK_ENABLED
	uint32_t prediv, pllmul;
	uint32_t ahb_div, apb1_div, apb2_div;

	RCC_HSEConfig(RCC_HSE_ON);
	do{
	}while(RCC_WaitForHSEStartUp()!=READY);

	BUILD_ASSERT(CPU_FREQ == ((HSE_FREQ) * (PLL1_MUL1) / (PLL1_PREDIV1)) , 
		"Unexpected CPU_FREQ and PLL Configuration, Please Check DTS");
	
	switch (PLL1_MUL1)
	{
		case 3: pllmul = RCC_PLLMul_3_EXTEN; break;
		case 4: pllmul = RCC_PLLMul_4_EXTEN; break;
		case 5: pllmul = RCC_PLLMul_5_EXTEN; break;
		case 6: pllmul = RCC_PLLMul_6_EXTEN; break;
		case 7: pllmul = RCC_PLLMul_7_EXTEN; break;
		case 8: pllmul = RCC_PLLMul_8_EXTEN; break;
		case 9: pllmul = RCC_PLLMul_9_EXTEN; break;
		case 10: pllmul = RCC_PLLMul_10_EXTEN; break;
		case 11: pllmul = RCC_PLLMul_11_EXTEN; break;
		case 12: pllmul = RCC_PLLMul_12_EXTEN; break;
		case 13: pllmul = RCC_PLLMul_13_EXTEN; break;
		case 14: pllmul = RCC_PLLMul_14_EXTEN; break;
		case 15: pllmul = RCC_PLLMul_15_EXTEN; break;
		case 16: pllmul = RCC_PLLMul_16_EXTEN; break;
		// case 17: pllmul = RCC_PLLMul_6_5_EXTEN; break;
		case 18: pllmul = RCC_PLLMul_18_EXTEN; break;
		default: pllmul = 0; break;
	}

	switch (PLL1_PREDIV1)
	{
	case 1: prediv = RCC_PREDIV1_Div1; break;
	case 2: prediv = RCC_PREDIV1_Div2; break;
	case 3: prediv = RCC_PREDIV1_Div3; break;
	case 4: prediv = RCC_PREDIV1_Div4; break;
	case 5: prediv = RCC_PREDIV1_Div5; break;
	case 6: prediv = RCC_PREDIV1_Div6; break;
	case 7: prediv = RCC_PREDIV1_Div7; break;
	case 8: prediv = RCC_PREDIV1_Div8; break;
	case 9: prediv = RCC_PREDIV1_Div9; break;
	case 10: prediv = RCC_PREDIV1_Div10; break;
	case 11: prediv = RCC_PREDIV1_Div11; break;
	case 12: prediv = RCC_PREDIV1_Div12; break;
	case 13: prediv = RCC_PREDIV1_Div13; break;
	case 14: prediv = RCC_PREDIV1_Div14; break;
	case 15: prediv = RCC_PREDIV1_Div15; break;
	case 16: prediv = RCC_PREDIV1_Div16; break;
	default: prediv = 0;
		break;
	}
	
	RCC_PLLConfig(prediv, pllmul);
	RCC_PLLCmd(ENABLE);

	switch (AHB_DIV)
	{
	case 1: ahb_div = RCC_SYSCLK_Div1; break;
	case 2: ahb_div = RCC_SYSCLK_Div2; break;
	case 4: ahb_div = RCC_SYSCLK_Div4; break;
	case 8: ahb_div = RCC_SYSCLK_Div8; break;
	case 16: ahb_div = RCC_SYSCLK_Div16; break;
	case 64: ahb_div = RCC_SYSCLK_Div64; break;
	case 128: ahb_div = RCC_SYSCLK_Div128; break;
	case 256: ahb_div = RCC_SYSCLK_Div256; break;
	case 512: ahb_div = RCC_SYSCLK_Div512; break;
	default: ahb_div = 0; break;
	}

	switch (APB1_DIV)
	{
	case 1: apb1_div = RCC_HCLK_Div1;	break;
	case 2: apb1_div = RCC_HCLK_Div2;	break;
	case 4: apb1_div = RCC_HCLK_Div4;	break;
	case 8: apb1_div = RCC_HCLK_Div8;	break;
	case 16: apb1_div = RCC_HCLK_Div16;	break;
	default:
		break;
	}

	switch (APB2_DIV)
	{
	case 1: apb2_div = RCC_HCLK_Div1;	break;
	case 2: apb2_div = RCC_HCLK_Div2;	break;
	case 4: apb2_div = RCC_HCLK_Div4;	break;
	case 8: apb2_div = RCC_HCLK_Div8;	break;
	case 16: apb2_div = RCC_HCLK_Div16;	break;
	default:
		break;
	}

	RCC_HCLKConfig(ahb_div);
	RCC_PCLK1Config(apb1_div);
	RCC_PCLK2Config(apb2_div);

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	/* config AHB & APB1,2 */

#endif
#endif
	return 0;
}

static int clock_control_ch32_on(const struct device *dev,
				  clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	
	return 0;
}

static int clock_control_ch32_off(const struct device *dev,
				   clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	return 0;
}

static enum clock_control_status clock_control_ch32_get_status(const struct device *dev,
								clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	// uint32_t clk_en_reg = periph_ll_get_clk_en_reg((periph_module_t)sys);
	// uint32_t clk_en_mask =  periph_ll_get_clk_en_mask((periph_module_t)sys);

	// if (DPORT_GET_PERI_REG_MASK(clk_en_reg, clk_en_mask)) {
	// 	return CLOCK_CONTROL_STATUS_ON;
	// }
	return CLOCK_CONTROL_STATUS_OFF;
}

static int clock_control_ch32_get_rate(const struct device *dev,
					clock_control_subsys_t sub_system,
					uint32_t *rate)
{
	ARG_UNUSED(sub_system);

	// rtc_cpu_freq_config_t config;

	// rtc_clk_cpu_freq_get_config(&config);

	// *rate = config.freq_mhz;

	return 0;
}

static const struct clock_control_driver_api clock_control_ch32_api = {
	.on = clock_control_ch32_on,
	.off = clock_control_ch32_off,
	.get_rate = clock_control_ch32_get_rate,
	.get_status = clock_control_ch32_get_status,
};


DEVICE_DT_INST_DEFINE(0,
		      &clock_control_ch32_init,
		      NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &clock_control_ch32_api);