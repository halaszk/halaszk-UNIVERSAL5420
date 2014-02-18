/* linux/arch/arm/mach-exynos/setup-fimc-is.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * FIMC-IS gpio and clock configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu.h>
#include <mach/exynos-fimc-is.h>
#include <mach/exynos-clock.h>
#include <mach/regs-gpio.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#undef USE_UART_DEBUG

#if defined(CONFIG_SOC_EXYNOS5260)
#include <mach/regs-clock-exynos5260.h>
/* Clock Gate func for Exynos5260 */
int exynos5260_fimc_is_clk_gate(u32 clk_gate_id, bool is_on)
{
	int cfg = 0;
	u32 value = 0;

	if (clk_gate_id == 0)
		return 0;

	/* ISP 1 */
	value = 0;
	if (clk_gate_id & (1 << FIMC_IS_GATE_ISP_IP))
		value |= (1 << 4);
	if (clk_gate_id & (1 << FIMC_IS_GATE_DRC_IP))
		value |= (1 << 2);
	if (clk_gate_id & (1 << FIMC_IS_GATE_SCC_IP))
		value |= (1 << 5);
	if (clk_gate_id & (1 << FIMC_IS_GATE_SCP_IP))
		value |= (1 << 6);
	if (clk_gate_id & (1 << FIMC_IS_GATE_FD_IP))
		value |= (1 << 3);

	if (value > 0) {
		cfg = readl(EXYNOS5260_CLKGATE_IP_ISP1);
		if (is_on)
			writel(cfg | value, EXYNOS5260_CLKGATE_IP_ISP1);
		else
			writel(cfg & ~(value), EXYNOS5260_CLKGATE_IP_ISP1);
		pr_debug("%s :2 [%s] gate(%d) (0x%x) * (0x%x)\n", __func__,
				is_on ? "ON" : "OFF",
				clk_gate_id,
				cfg,
				value);
	}

	return 0;
}

int exynos5260_fimc_is_set_user_clk_gate(u32 group_id,
		bool is_on,
		u32 user_scenario_id,
		unsigned long msk_state,
		struct exynos_fimc_is_clk_gate_info *gate_info) {
	/* if you want to skip clock on/off, let this func return -1 */
	int ret = -1;

	switch (user_scenario_id) {
	default:
		ret = 0;
		break;
	}

	return ret;
}

/* Exynos5260 */
int exynos5260_fimc_is_cfg_clk(struct platform_device *pdev)
{
	/* top */
	struct clk *sclk_bustop_pll = NULL;
	struct clk *mout_isp1_media_266 = NULL;
	struct clk *sclk_memtop_pll = NULL;

	/* isp */
	struct clk *aclk_isp_266 = NULL;
	struct clk *aclk_isp_266_user = NULL;
	struct clk *pclk_isp_133 = NULL;
	struct clk *pclk_isp_66 = NULL;
	struct clk *sclk_mpwm_isp = NULL;

	/* mcuctl */
	struct clk *aclk_isp_400 = NULL;
	struct clk *aclk_ca5_clkin = NULL;
	struct clk *aclk_ca5_atclkin = NULL;
	struct clk *pclk_ca5_pclkdbg = NULL;
#ifdef USE_UART_DEBUG
	struct clk *sclk_uart_isp = NULL;
	struct clk *sclk_uart_isp_div = NULL;
	unsigned long isp_uart;
#endif
	pr_info("%s\n", __func__);

	/* 0. TOP */
	sclk_memtop_pll = clk_get(&pdev->dev, "sclk_memtop_pll");
	if (IS_ERR(sclk_memtop_pll))
		return PTR_ERR(sclk_memtop_pll);

	mout_isp1_media_266 = clk_get(&pdev->dev, "mout_isp1_media_266");
	if (IS_ERR(mout_isp1_media_266)) {
		clk_put(sclk_memtop_pll);
		return PTR_ERR(mout_isp1_media_266);
	}

	aclk_isp_266 = clk_get(&pdev->dev, "aclk_isp_266");
	if (IS_ERR(aclk_isp_266)) {
		clk_put(sclk_memtop_pll);
		clk_put(mout_isp1_media_266);
		return PTR_ERR(aclk_isp_266);
	}

	clk_set_parent(mout_isp1_media_266, sclk_memtop_pll);
	clk_set_parent(aclk_isp_266, mout_isp1_media_266);
	clk_set_rate(aclk_isp_266, 300 * 1000000);

	clk_put(sclk_memtop_pll);
	clk_put(mout_isp1_media_266);

	sclk_bustop_pll = clk_get(&pdev->dev, "sclk_bustop_pll");
	if (IS_ERR(sclk_bustop_pll))
		return PTR_ERR(sclk_bustop_pll);

	aclk_isp_400 = clk_get(&pdev->dev, "aclk_isp_400");
	if (IS_ERR(aclk_isp_400)) {
		clk_put(sclk_bustop_pll);
		return PTR_ERR(aclk_isp_400);
	}

	clk_set_parent(aclk_isp_400, sclk_bustop_pll);
	clk_set_rate(aclk_isp_400, 400 * 1000000);

	/* 1. MCUISP: aclk_isp_400 */
	aclk_ca5_clkin = clk_get(&pdev->dev, "aclk_ca5_clkin");
	if (IS_ERR(aclk_ca5_clkin)) {
		clk_put(aclk_isp_400);
		return PTR_ERR(aclk_ca5_clkin);
	}

	aclk_ca5_atclkin = clk_get(&pdev->dev, "aclk_ca5_atclkin");
	if (IS_ERR(aclk_ca5_atclkin)) {
		clk_put(aclk_isp_400);
		clk_put(aclk_ca5_clkin);
		return PTR_ERR(aclk_ca5_atclkin);
	}

	pclk_ca5_pclkdbg = clk_get(&pdev->dev, "pclk_ca5_pclkdbg");
	if (IS_ERR(pclk_ca5_pclkdbg)) {
		clk_put(aclk_isp_400);
		clk_put(aclk_ca5_clkin);
		clk_put(aclk_ca5_atclkin);
		return PTR_ERR(pclk_ca5_pclkdbg);
	}

	clk_set_parent(aclk_ca5_clkin, aclk_isp_400);
	clk_set_rate(aclk_ca5_atclkin, 200 * 1000000);
	clk_set_rate(pclk_ca5_pclkdbg, 100 * 1000000);

	clk_put(aclk_isp_400);
	clk_put(aclk_ca5_clkin);
	clk_put(aclk_ca5_atclkin);
	clk_put(pclk_ca5_pclkdbg);

	/* 2. ACLK_ISP: aclk_isp_266 */
	aclk_isp_266_user = clk_get(&pdev->dev, "aclk_isp_266_user");
	if (IS_ERR(aclk_isp_266_user)) {
		clk_put(aclk_isp_266);
		return PTR_ERR(aclk_isp_266_user);
	}

	pclk_isp_133 = clk_get(&pdev->dev, "pclk_isp_133");
	if (IS_ERR(pclk_isp_133)) {
		clk_put(aclk_isp_266);
		clk_put(aclk_isp_266_user);
		return PTR_ERR(pclk_isp_133);
	}

	pclk_isp_66 = clk_get(&pdev->dev, "pclk_isp_66");
	if (IS_ERR(pclk_isp_66)) {
		clk_put(aclk_isp_266);
		clk_put(aclk_isp_266_user);
		clk_put(pclk_isp_133);
		return PTR_ERR(pclk_isp_66);
	}

	sclk_mpwm_isp = clk_get(&pdev->dev, "sclk_mpwm_isp");
	if (IS_ERR(sclk_mpwm_isp)) {
		clk_put(aclk_isp_266);
		clk_put(aclk_isp_266_user);
		clk_put(pclk_isp_133);
		clk_put(pclk_isp_66);
		return PTR_ERR(sclk_mpwm_isp);
	}

	clk_set_parent(aclk_isp_266_user, aclk_isp_266);
	clk_set_rate(pclk_isp_133 , 150 * 1000000);
	clk_set_rate(pclk_isp_66 ,  75 * 1000000);
	clk_set_rate(sclk_mpwm_isp, 75 * 1000000);

	clk_put(aclk_isp_266);
	clk_put(aclk_isp_266_user);
	clk_put(pclk_isp_133);
	clk_put(pclk_isp_66);
	clk_put(sclk_mpwm_isp);

	return 0;
}

int exynos5260_fimc_is_cfg_gpio(struct platform_device *pdev, int channel, bool flag_on)
{
	return 0;
}

int exynos5260_fimc_is_clk_on(struct platform_device *pdev)
{
	struct clk *isp_ctrl = NULL;

	pr_info("%s\n", __func__);

	isp_ctrl = clk_get(&pdev->dev, "isp1");
	if (IS_ERR(isp_ctrl)) {
		pr_err("%s : clk_get(isp1) failed\n", __func__);
		return PTR_ERR(isp_ctrl);
	}

	clk_enable(isp_ctrl);
	clk_put(isp_ctrl);

	return 0;
}

int exynos5260_fimc_is_clk_off(struct platform_device *pdev)
{
	struct clk *isp_ctrl = NULL;

	pr_info("%s\n", __func__);

	isp_ctrl = clk_get(&pdev->dev, "isp1");
	if (IS_ERR(isp_ctrl)) {
		pr_err("%s : clk_get(isp1) failed\n", __func__);
		return PTR_ERR(isp_ctrl);
	}

	clk_disable(isp_ctrl);
	clk_put(isp_ctrl);

	return 0;
}

/* utility function to get rate */
unsigned int  fimc_is_get_rate(struct platform_device *pdev,
	const char *conid)
{
	struct clk *target;
	unsigned int rate_target;

	target = clk_get(&pdev->dev, conid);
	if (IS_ERR(target)) {
		pr_err("%s: could not lookup clock : %s\n", __func__, conid);
		return -EINVAL;
	}

	rate_target = clk_get_rate(target);
	clk_put(target);
	pr_info("%s : %d\n", conid, rate_target);

	return rate_target;
}

int exynos5260_fimc_is_print_clk(struct platform_device *pdev)
{

	fimc_is_get_rate(pdev, "sclk_memtop_pll");
	fimc_is_get_rate(pdev, "mout_isp1_media_266");
	fimc_is_get_rate(pdev, "sclk_bustop_pll");

	fimc_is_get_rate(pdev, "aclk_isp_400");
	fimc_is_get_rate(pdev, "aclk_isp_266");

	fimc_is_get_rate(pdev, "aclk_ca5_clkin");
	fimc_is_get_rate(pdev, "aclk_ca5_atclkin");
	fimc_is_get_rate(pdev, "pclk_ca5_pclkdbg");

	fimc_is_get_rate(pdev, "aclk_isp_266_user");
	fimc_is_get_rate(pdev, "pclk_isp_133");
	fimc_is_get_rate(pdev, "pclk_isp_66");
	fimc_is_get_rate(pdev, "sclk_mpwm_isp");

	fimc_is_get_rate(pdev, "sclk_mediatop_pll");
	fimc_is_get_rate(pdev, "aclk_gscl_fimc");
	fimc_is_get_rate(pdev, "aclk_gscl_fimc_user");
	fimc_is_get_rate(pdev, "sclK_csis");

	fimc_is_get_rate(pdev, "ext_xtal");
	fimc_is_get_rate(pdev, "dout_isp1_sensor0_a");
	fimc_is_get_rate(pdev, "dout_isp1_sensor1_a");
	fimc_is_get_rate(pdev, "dout_isp1_sensor2_a");

	return 0;
}

int exynos5260_fimc_is_print_cfg(struct platform_device *pdev, u32 channel)
{
	return 0;
}
#elif defined(CONFIG_SOC_EXYNOS3470)
/* Clock Gate func for Exynos3470 */
int exynos3470_fimc_is_clk_gate(u32 clk_gate_id, bool is_on)
{
	int cfg = 0;
	u32 value = 0;

	if (clk_gate_id == 0)
		return 0;

	/* ISP 0 */
	value = 0;
	if (clk_gate_id & (1 << FIMC_IS_GATE_ISP_IP))
		value |= (1 << 0);
	if (clk_gate_id & (1 << FIMC_IS_GATE_DRC_IP))
		value |= (1 << 1);
	if (clk_gate_id & (1 << FIMC_IS_GATE_FD_IP))
		value |= (1 << 2);

	if (value > 0) {
		cfg = readl(EXYNOS4_CLKGATE_IP_ISP0);
		if (is_on)
			writel(cfg | value, EXYNOS4_CLKGATE_IP_ISP0);
		else
			writel(cfg & ~(value), EXYNOS4_CLKGATE_IP_ISP0);
		pr_debug("%s :0 [%s] gate(%d) (0x%x) * (0x%x)\n", __func__,
				is_on ? "ON" : "OFF",
				clk_gate_id,
				cfg,
				value);
	}

	/* ISP 1 */
	value = 0;
	if (clk_gate_id & (1 << FIMC_IS_GATE_SCC_IP))
		value |= (1 << 15);
	if (clk_gate_id & (1 << FIMC_IS_GATE_SCP_IP))
		value |= (1 << 16);

	if (value > 0) {
		cfg = readl(EXYNOS4_CLKGATE_IP_ISP1);
		if (is_on)
			writel(cfg | value, EXYNOS4_CLKGATE_IP_ISP1);
		else
			writel(cfg & ~(value), EXYNOS4_CLKGATE_IP_ISP1);
		pr_debug("%s :0 [%s] gate(%d) (0x%x) * (0x%x)\n", __func__,
				is_on ? "ON" : "OFF",
				clk_gate_id,
				cfg,
				value);
	}

	return 0;
}

int exynos3470_fimc_is_set_user_clk_gate(u32 group_id,
		bool is_on,
		u32 user_scenario_id,
		unsigned long msk_state,
		struct exynos_fimc_is_clk_gate_info *gate_info) {
	/* if you want to skip clock on/off, let this func return -1 */
	int ret = -1;

	switch (user_scenario_id) {
	default:
		ret = 0;
		break;
	}

	return ret;
}

int exynos3470_fimc_is_cfg_clk(struct platform_device *pdev)
{
	struct clk *sclk_vpll = NULL;
	struct clk *mout_mpll = NULL;
	struct clk *pre_aclk_400_mcuisp = NULL;
	struct clk *aclk_400_mcuisp = NULL;
	struct clk *aclk_400_mcuisp_div0 = NULL;
	struct clk *aclk_400_mcuisp_div1 = NULL;
	struct clk *pre_aclk_266 = NULL;
	struct clk *aclk_266 = NULL;
	struct clk *aclk_266_div0 = NULL;
	struct clk *aclk_266_div1 = NULL;
	struct clk *aclk_266_div2 = NULL;
#ifdef USE_UART_DEBUG
	struct clk *sclk_uart_isp = NULL;
#endif
	pr_debug("(%s) ++++\n", __func__);

	/* 1. MCUISP */
	mout_mpll = clk_get(&pdev->dev, "mout_mpll");
	if (IS_ERR(mout_mpll)) {
		pr_err("fail to get 'mout_mpll'\n");
		return PTR_ERR(mout_mpll);
	}

	pre_aclk_400_mcuisp = clk_get(&pdev->dev, "pre_aclk_400_mcuisp");
	if (IS_ERR(pre_aclk_400_mcuisp)) {
		clk_put(mout_mpll);
		pr_err("fail to get 'pre_aclk_400_mcuisp'\n");
		return PTR_ERR(pre_aclk_400_mcuisp);
	}
	clk_set_parent(pre_aclk_400_mcuisp, mout_mpll);
	clk_set_rate(pre_aclk_400_mcuisp, 400 * 1000000);

	aclk_400_mcuisp = clk_get(&pdev->dev, "aclk_400_mcuisp");
	if (IS_ERR(aclk_400_mcuisp)) {
		clk_put(mout_mpll);
		clk_put(pre_aclk_400_mcuisp);
		pr_err("fail to get 'aclk_400_mcuisp'\n");
		return PTR_ERR(aclk_400_mcuisp);
	}
	clk_set_parent(aclk_400_mcuisp, pre_aclk_400_mcuisp);

	aclk_400_mcuisp_div0 = clk_get(&pdev->dev, "aclk_400_mcuisp_div0");
	if (IS_ERR(aclk_400_mcuisp_div0)) {
		clk_put(mout_mpll);
		clk_put(pre_aclk_400_mcuisp);
		clk_put(aclk_400_mcuisp);
		pr_err("fail to get 'aclk_400_mcuisp_div0'\n");
		return PTR_ERR(aclk_400_mcuisp_div0);
	}
	clk_set_rate(aclk_400_mcuisp_div0, 200 * 1000000);

	aclk_400_mcuisp_div1 = clk_get(&pdev->dev, "aclk_400_mcuisp_div1");
	if (IS_ERR(aclk_400_mcuisp_div1)) {
		clk_put(mout_mpll);
		clk_put(pre_aclk_400_mcuisp);
		clk_put(aclk_400_mcuisp);
		clk_put(aclk_400_mcuisp_div0);
		pr_err("fail to get 'aclk_400_mcuisp_div1'\n");
		return PTR_ERR(aclk_400_mcuisp_div1);
	}
	clk_set_rate(aclk_400_mcuisp_div1, 100 * 1000000);

	pr_info("aclk_400_mcuisp : %ld\n", clk_get_rate(aclk_400_mcuisp));
	pr_info("aclk_400_mcuisp_div0 : %ld\n", clk_get_rate(aclk_400_mcuisp_div0));
	pr_info("aclk_400_mcuisp_div1 : %ld\n", clk_get_rate(aclk_400_mcuisp_div1));

	clk_put(mout_mpll);
	clk_put(pre_aclk_400_mcuisp);
	clk_put(aclk_400_mcuisp);
	clk_put(aclk_400_mcuisp_div0);
	clk_put(aclk_400_mcuisp_div1);

	/* 2. ACLK_ISP */
	sclk_vpll = clk_get(&pdev->dev, "sclk_vpll");
	if (IS_ERR(sclk_vpll)) {
		pr_err("fail to get 'sclk_vpll'\n");
		return PTR_ERR(sclk_vpll);
	}

	pre_aclk_266 = clk_get(&pdev->dev, "pre_aclk_266");
	if (IS_ERR(pre_aclk_266)) {
		clk_put(sclk_vpll);
		pr_err("fail to get 'pre_aclk_266'\n");
		return PTR_ERR(pre_aclk_266);
	}
	clk_set_parent(pre_aclk_266, sclk_vpll);
	clk_set_rate(pre_aclk_266, 300 * 1000000);

	aclk_266 = clk_get(&pdev->dev, "aclk_266");
	if (IS_ERR(aclk_266)) {
		clk_put(sclk_vpll);
		clk_put(pre_aclk_266);
		pr_err("fail to get 'aclk_266'\n");
		return PTR_ERR(aclk_266);
	}
	clk_set_parent(aclk_266, pre_aclk_266);

	aclk_266_div0 = clk_get(&pdev->dev, "aclk_266_div0");
	if (IS_ERR(aclk_266_div0)) {
		clk_put(sclk_vpll);
		clk_put(pre_aclk_266);
		clk_put(aclk_266);
		pr_err("fail to get 'aclk_266_div0'\n");
		return PTR_ERR(aclk_266_div0);
	}
	clk_set_rate(aclk_266_div0, 150 * 1000000);

	aclk_266_div1 = clk_get(&pdev->dev, "aclk_266_div1");
	if (IS_ERR(aclk_266_div1)) {
		clk_put(sclk_vpll);
		clk_put(pre_aclk_266);
		clk_put(aclk_266);
		clk_put(aclk_266_div0);
		pr_err("fail to get 'aclk_266_div1'\n");
		return PTR_ERR(aclk_266_div1);
	}
	clk_set_rate(aclk_266_div1, 75 * 1000000);

	aclk_266_div2 = clk_get(&pdev->dev, "aclk_266_div2");
	if (IS_ERR(aclk_266_div2)) {
		clk_put(sclk_vpll);
		clk_put(pre_aclk_266);
		clk_put(aclk_266);
		clk_put(aclk_266_div0);
		clk_put(aclk_266_div1);
		pr_err("fail to get 'aclk_266_div2'\n");
		return PTR_ERR(aclk_266_div2);
	}
	clk_set_rate(aclk_266_div2, 75 * 1000000);

	pr_info("aclk_266 : %ld\n", clk_get_rate(aclk_266));
	pr_info("aclk_266_div0 : %ld\n", clk_get_rate(aclk_266_div0));
	pr_info("aclk_266_div1 : %ld\n", clk_get_rate(aclk_266_div1));
	pr_info("aclk_266_div2 : %ld\n", clk_get_rate(aclk_266_div2));

	clk_put(sclk_vpll);
	clk_put(pre_aclk_266);
	clk_put(aclk_266);
	clk_put(aclk_266_div0);
	clk_put(aclk_266_div1);
	clk_put(aclk_266_div2);


#ifdef USE_UART_DEBUG
	/* 3. UART-ISP */
	/* change special clock source to mout_mpll_user_top */
	mout_mpll = clk_get(&pdev->dev, "mout_mpll_user_top");
	if (IS_ERR(mout_mpll)) {
		pr_err("fail to get 'mout_mpll_user_top'\n");
		return PTR_ERR(mout_mpll);
	}
	sclk_uart_isp = clk_get(&pdev->dev, "sclk_uart_isp");
	if (IS_ERR(sclk_uart_isp)) {
		clk_put(mout_mpll);
		pr_err("fail to get 'sclk_uart_isp'\n");
		return PTR_ERR(sclk_uart_isp);
	}
	clk_set_parent(sclk_uart_isp, mout_mpll);
	clk_set_rate(sclk_uart_isp, 50 * 1000000);

	pr_debug("sclk_uart_isp : %ld\n", clk_get_rate(sclk_uart_isp));
	clk_put(mout_mpll);
	clk_put(sclk_uart_isp);
#endif
	pr_debug("(%s) ----\n", __func__);

	return 0;
}

int exynos3470_fimc_is_cfg_gpio(struct platform_device *pdev, int channel, bool flag_on)
{
	return 0;
}

int exynos3470_fimc_is_clk_on(struct platform_device *pdev)
{
	struct clk *isp_ctrl = NULL;
	struct clk *pre_aclk_400_mcuisp = NULL;
	struct clk *aclk_400_mcuisp = NULL;
	struct clk *pre_aclk_266 = NULL;
	struct clk *aclk_266 = NULL;

	pr_debug("(%s) ++++\n", __func__);

	/* ISP */
	isp_ctrl = clk_get(&pdev->dev, "gate_isp0");
	if (IS_ERR(isp_ctrl)) {
		pr_err("fail to get 'gate_isp0'\n");
		return PTR_ERR(isp_ctrl);
	}
	clk_enable(isp_ctrl);
	clk_put(isp_ctrl);

	isp_ctrl = clk_get(&pdev->dev, "gate_isp1");
	if (IS_ERR(isp_ctrl)) {
		pr_err("fail to get 'gate_isp1'\n");
		return PTR_ERR(isp_ctrl);
	}
	clk_enable(isp_ctrl);
	clk_put(isp_ctrl);

#ifdef USE_UART_DEBUG
	isp_ctrl = clk_get(&pdev->dev, "uart_isp");
	if (IS_ERR(isp_ctrl)) {
		pr_err("fail to get 'uart_isp'\n");
		return PTR_ERR(isp_ctrl);
	}
	clk_enable(isp_ctrl);
	clk_put(isp_ctrl);
#endif

	/* SUB selection */
	pre_aclk_400_mcuisp = clk_get(&pdev->dev, "pre_aclk_400_mcuisp");
	if (IS_ERR(pre_aclk_400_mcuisp))
		return PTR_ERR(pre_aclk_400_mcuisp);
	aclk_400_mcuisp = clk_get(&pdev->dev, "aclk_400_mcuisp");
	if (IS_ERR(aclk_400_mcuisp)) {
		clk_put(pre_aclk_400_mcuisp);
		return PTR_ERR(aclk_400_mcuisp);
	}
	clk_set_parent(aclk_400_mcuisp, pre_aclk_400_mcuisp);
	clk_put(pre_aclk_400_mcuisp);
	clk_put(aclk_400_mcuisp);

	pre_aclk_266 = clk_get(&pdev->dev, "pre_aclk_266");
	if (IS_ERR(pre_aclk_266))
		return PTR_ERR(pre_aclk_266);
	aclk_266 = clk_get(&pdev->dev, "aclk_266");
	if (IS_ERR(aclk_266)) {
		clk_put(pre_aclk_266);
		return PTR_ERR(aclk_266);
	}
	clk_set_parent(aclk_266, pre_aclk_266);
	clk_put(pre_aclk_266);
	clk_put(aclk_266);

	pr_debug("(%s) ----\n", __func__);

	return 0;
}

int exynos3470_fimc_is_clk_off(struct platform_device *pdev)
{
	struct clk *isp_ctrl = NULL;
	struct clk *ext_xtal = NULL;
	struct clk *aclk_400_mcuisp = NULL;
	struct clk *aclk_266 = NULL;

	pr_debug("(%s) ++++\n", __func__);

	/* 3. ISP */
	isp_ctrl = clk_get(&pdev->dev, "gate_isp0");
	if (IS_ERR(isp_ctrl))
		return PTR_ERR(isp_ctrl);

	clk_disable(isp_ctrl);
	clk_put(isp_ctrl);

	isp_ctrl = clk_get(&pdev->dev, "gate_isp1");
	if (IS_ERR(isp_ctrl))
		return PTR_ERR(isp_ctrl);

	clk_disable(isp_ctrl);
	clk_put(isp_ctrl);

#ifdef USE_UART_DEBUG
	isp_ctrl = clk_get(&pdev->dev, "uart_isp");
	if (IS_ERR(isp_ctrl)) {
		pr_err("fail to get 'uart_isp'\n");
		return PTR_ERR(isp_ctrl);
	}
	clk_disable(isp_ctrl);
	clk_put(isp_ctrl);
#endif
	/* SUB selection */
	ext_xtal = clk_get(&pdev->dev, "ext_xtal");
	if (IS_ERR(ext_xtal))
		return PTR_ERR(ext_xtal);

	aclk_400_mcuisp = clk_get(&pdev->dev, "aclk_400_mcuisp");
	if (IS_ERR(aclk_400_mcuisp)) {
		clk_put(ext_xtal);
		return PTR_ERR(aclk_400_mcuisp);
	}
	clk_set_parent(aclk_400_mcuisp, ext_xtal);
	clk_put(aclk_400_mcuisp);

	aclk_266 = clk_get(&pdev->dev, "aclk_266");
	if (IS_ERR(aclk_266)) {
		clk_put(ext_xtal);
		return PTR_ERR(aclk_266);
	}
	clk_set_parent(aclk_266, ext_xtal);
	clk_put(aclk_266);

	pr_debug("(%s) ----\n", __func__);
	return 0;
}

int exynos3470_fimc_is_print_clk(struct platform_device *pdev)
{
	return 0;
}

int exynos3470_fimc_is_print_cfg(struct platform_device *pdev, u32 channel)
{
	return 0;
}
#endif
