
/*
 * Copyright
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CH32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CH32_CLOCK_CONTROL_H_

#define CPU_FREQ  DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#define HSE_FREQ  DT_PROP(DT_PATH(clocks, clk_hse), clock_frequency)
#define HSI_FREQ  DT_PROP(DT_PATH(clocks, clk_hsi), clock_frequency)
#define LSI_FREQ  DT_PROP(DT_PATH(clocks, clk_lsi), clock_frequency)
#define LSE_FREQ  DT_PROP(DT_PATH(clocks, clk_lse), clock_frequency)
#define PLL1_MULT DT_PROP(DT_PATH(clocks, pll1), clock_mult)

#define RCC_AHB_PRESCALER  DT_PROP(DT_PATH(soc, reset_clock_controller_40021000), ahb_prescaler)
#define RCC_APB1_PRESCALER DT_PROP(DT_NODELABEL(rcc), apb1_prescaler)
#define RCC_APB2_PRESCALER DT_PROP(DT_NODELABEL(rcc), apb2_prescaler)

#define RCC_CLOCKS_OUTPUT DT_PROP(DT_PATH(soc, rcc), clock_frequency)
#define RCC_CLOCKS_SRC    DT_PHANDLE(DT_NODELABEL(rcc), clocks)
#define CLK_HSE_NODE      DT_NODELABEL(clk_hse)
#define CLK_HSI_NODE      DT_NODELABEL(clk_hsi)
#define PLL1_NODE         DT_NODELABEL(pll1)


#define CH32_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(rcc))

/** Driver structure definition */
struct ch32_pclken {
	uint32_t bus;
	uint32_t enr;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CH32_CLOCK_CONTROL_H_ */