// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq / Intel GRX500 (xRX500) PCIe Root Complex Driver
 *
 * Copyright (C) 2013 John Crispin <blogic@openwrt.org>
 * Copyright (C) 2009~2015 Lei Chuanhua <chuanhua.lei@lantiq.com>
 * Copyright (C) 2016 Intel Corporation
 * Adapted for modern Linux 6.12 LTS on Archer AX50 v1
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#include <lantiq.h>
#include <lantiq_soc.h>
#include <asm/pci.h>

#define MS(_v, _f)  (((_v) & (_f)) >> _f##_S)
#define SM(_v, _f)  (((_v) << _f##_S) & (_f))

/* Application Register Offsets */
#define PCIE_RC_CCR				0x0010
#define PCIE_RC_CCR_LTSSM_ENABLE		BIT(0)

#define PCIE_RC_DR				0x0014
#define PCIE_RC_DR_DLL_UP			BIT(0)

#define PCIE_PHY_SR				0x0018
#define PCIE_PHY_SR_PHY_LINK_UP			BIT(0)

#define PCIE_AHB_CTRL				0x0078
#define PCIE_AHB_CTRL_BUS_ERROR_SUPPRESS	BIT(0)

#define PCIE_IRNEN				0x00F4
#define PCIE_IRNCR				0x00F8

/* PHY Register Offsets (DesignWare 2.0 PDI) */
#define PCIE_PHY_CFG0				0xC004
#define PCIE_PHY_CFG1				0xC008
#define PCIE_PHY_CFG2				0xC00C
#define PCIE_PHY_CFG3				0xC010
#define PCIE_PHY_PIPE_EN			BIT(0)
#define PCIE_PHY_CM_CLK_SEL			BIT(1)
#define PCIE_PHY_CFG4				0xC014
#define PCIE_PHY_REF_CLK_PAD			BIT(0)
#define PCIE_PHY_PIPE_PD			BIT(14)
#define PCIE_PHY_PIPE_PD_O			BIT(15)

/* CGU LCPLL Register Offsets (GRX500) */
#define PCIE_LCPLL_CFG0				0x0094
#define LCPLL_CFG0_LOCKED			BIT(1)
#define PCIE_LCPLL_CFG1				0x0098
#define PCIE_LCPLL_SSC_CTRL			0x009C
#define PCIE_LCPLL_SSC_SCALE			0x00A0
#define PCIE_LCPLL_COEF_0			0x00A4
#define PCIE_LCPLL_COEF_1			0x00A8
#define PCIE_LCPLL_COEF_2			0x00AC
#define PCIE_LCPLL_COEF_3			0x00B0
#define PCIE_LCPLL_COEF_4			0x00B4
#define PCIE_LCPLL_COEF_5			0x00B8
#define PCIE_LCPLL_COEF_6			0x00BC
#define PCIE_LCPLL_COEF_7			0x00C0
#define CGU_CLK_SR2				0x0130

/* Controller Config (CSR) Register Offsets */
#define PCIE_PCICMDSTS				0x0004

#define PCIE_BNR				0x0018
#define PCIE_BNR_PRIMARY_BUS_NUM		0x000000FF
#define PCIE_BNR_PRIMARY_BUS_NUM_S		0
#define PCIE_PNR_SECONDARY_BUS_NUM		0x0000FF00
#define PCIE_PNR_SECONDARY_BUS_NUM_S		8
#define PCIE_PNR_SUB_BUS_NUM			0x00FF0000
#define PCIE_PNR_SUB_BUS_NUM_S			16
#define PCIE_IOBLSECS				0x001C
#define PCIE_IOBLSECS_IO_BASE_ADDR		0x000000F0
#define PCIE_IOBLSECS_IO_BASE_ADDR_S		4
#define PCIE_IOBLSECS_IO_LIMIT_ADDR		0x0000F000
#define PCIE_IOBLSECS_IO_LIMIT_ADDR_S		12

#define PCIE_MBML				0x0020
#define PCIE_MBML_MEM_BASE_ADDR			0x0000FFF0
#define PCIE_MBML_MEM_BASE_ADDR_S		4
#define PCIE_MBML_MEM_LIMIT_ADDR		0xFFF00000
#define PCIE_MBML_MEM_LIMIT_ADDR_S		20

#define PCIE_PMBL				0x0024

#define PCIE_INTRBCTRL				0x003C
#define PCIE_INTRBCTRL_PARITY_ERR_RESP_ENABLE	BIT(16)
#define PCIE_INTRBCTRL_SERR_ENABLE		BIT(17)

#define PCIE_PM_CSR				0x0044
#define PCIE_PM_CSR_SW_RST			BIT(3)
#define PCIE_PM_CSR_PME_ENABLE			BIT(8)

#define PCIE_MCAPR				0x0050
#define PCIE_MCAPR_MSI_ENABLE			BIT(16)
#define PCIE_MCAPR_MULTI_MSG_CAP		0x000E0000
#define PCIE_MCAPR_MULTI_MSG_ENABLE		0x00700000
#define PCIE_MCAPR_ADDR64_CAP			BIT(23)

#define PCIE_DCAP				0x0074
#define PCIE_DCAP_MAX_PAYLOAD_SIZE		0x00000007
#define PCIE_DCAP_MAX_PAYLOAD_SIZE_S		0
#define PCIE_DCAP_EP_L0S_LATENCY		0x000001C0
#define PCIE_DCAP_EP_L0S_LATENCY_S		6
#define PCIE_DCAP_EP_L1_LATENCY			0x00000E00
#define PCIE_DCAP_EP_L1_LATENCY_S		9
#define PCIE_DCAP_ROLE_BASE_ERR_REPORT		BIT(15)

#define PCIE_DCTLSTS				0x0078
#define PCIE_DCTLSTS_CORRECTABLE_ERR_EN		BIT(0)
#define PCIE_DCTLSTS_NONFATAL_ERR_EN		BIT(1)
#define PCIE_DCTLSTS_FATAL_ERR_EN		BIT(2)
#define PCIE_DCTLSTS_RELAXED_ORDERING_EN	BIT(4)
#define PCIE_DCTLSTS_MAX_PAYLOAD_SIZE		0x000000E0
#define PCIE_DCTLSTS_MAX_PAYLOAD_SIZE_S		5
#define PCIE_DCTLSTS_NO_SNOOP_EN		BIT(11)
#define PCIE_DCTLSTS_MAX_READ_SIZE		0x00007000
#define PCIE_DCTLSTS_MAX_READ_SIZE_S		12
#define PCIE_DCTLSTS_ERR_EN	(PCIE_DCTLSTS_CORRECTABLE_ERR_EN | \
				 PCIE_DCTLSTS_NONFATAL_ERR_EN | \
				 PCIE_DCTLSTS_FATAL_ERR_EN)

#define PCIE_LCAP				0x007C
#define PCIE_LCAP_L0S_EIXT_LATENCY		0x00007000
#define PCIE_LCAP_L0S_EIXT_LATENCY_S		12

#define PCIE_LCTLSTS				0x0080
#define PCIE_LCTLSTS_ASPM_ENABLE		0x00000003
#define PCIE_LCTLSTS_RCB128			BIT(3)
#define PCIE_LCTLSTS_LINK_DISABLE		BIT(4)
#define PCIE_LCTLSTS_COM_CLK_CFG		BIT(6)
#define PCIE_LCTLSTS_NEGOTIATED_LINK_WIDTH	0x03F00000
#define PCIE_LCTLSTS_NEGOTIATED_LINK_WIDTH_S	20
#define PCIE_LCTLSTS_SLOT_CLK_CFG		BIT(28)
#define PCIE_LCTLSTS_RETRAIN_PENDING		0x08000000
#define PCIE_LCTLSTS_DLL_ACTIVE			BIT(29)

#define PCIE_UEMR				0x0108
#define PCIE_UESR				0x010C
#define PCIE_CESR				0x0110
#define PCIE_CEMR				0x0114
#define PCIE_ALL_UNCORRECTABLE_ERR		0x001FFFFF
#define PCIE_CORRECTABLE_ERR			0x000031C1

#define PCIE_PVC2				0x0148
#define PCIE_PVC2_VC_ARB_16P_FIXED_WRR		BIT(0)
#define PCIE_PVC2_VC_ARB_WRR			0x0000000F

#define PCIE_VC0_RC				0x0150
#define PCIE_VC0_RC_REJECT_SNOOP		BIT(15)

#define PCIE_VC0_CRQCR				0x0750
#define PCIE_VC0_CRQCR_CPL_TLP_QUEUE_MODE	0x00E00000
#define PCIE_VC0_CRQCR_CPL_TLP_QUEUE_MODE_S	21
#define PCIE_VC0_TLP_QUEUE_MODE_BYPASS		4

#define PCIE_ALTRT				0x0700
#define PCIE_ALTRT_REPLAY_TIME_LIMIT		0xFFFF0000
#define PCIE_ALTRT_REPLAY_TIME_LIMIT_S		16

#define PCIE_AFR				0x070C
#define PCIE_AFR_FTS_NUM			0x0000FF00
#define PCIE_AFR_FTS_NUM_S			8
#define PCIE_AFR_COM_FTS_NUM			0x00FF0000
#define PCIE_AFR_COM_FTS_NUM_S			16
#define PCIE_AFR_L0S_ENTRY_LATENCY		0x07000000
#define PCIE_AFR_L0S_ENTRY_LATENCY_S		24
#define PCIE_AFR_L1_ENTRY_LATENCY		0x38000000
#define PCIE_AFR_L1_ENTRY_LATENCY_S		27
#define PCIE_AFR_FTS_NUM_DEFAULT		127
#define PCIE_AFR_L0S_ENTRY_LATENCY_DEFAULT	7
#define PCIE_AFR_L1_ENTRY_LATENCY_DEFAULT	5

#define PCIE_PLCR				0x0710
#define PCIE_PLCR_DLL_LINK_EN			BIT(5)

#define PCIE_LSR				0x0714
#define PCIE_LSR_FC_DISABLE			BIT(24)
#define PCIE_LSR_ACKNAK_DISABLE			BIT(25)

#define PCIE_STRFMR				0x071C
#define PCIE_STRFMR_FC_WDT_DISABLE		BIT(15)
#define PCIE_STRFMR_RX_IO_TRANS_ENABLE		BIT(30)
#define PCIE_STRFMR_RX_CFG_TRANS_ENABLE		BIT(31)

#define PCIE_FMR2				0x0720
#define PCIE_FMR2_VENDOR_MSG0_PASSED_TO_TRGT1	BIT(0)
#define PCIE_FMR2_VENDOR_MSG1_PASSED_TO_TRGT1	BIT(1)

#define PCIE_GEN2_CTRL				0x080C
#define PCIE_GEN2_CTRL_FTS			0x000000FF
#define PCIE_GEN2_CTRL_FTS_S			0
#define PCIE_GEN2_DEFAULT_FTS_NUM		127
#define PCIE_GEN2_CTRL_SPEED_CHANGE		BIT(17)

#define PCIE_LCTLSTS2				0x00A0
#define PCIE_LCTLSTS2_TGT_LINK_SPEED		0x0000000F
#define PCIE_LCTLSTS2_TGT_LINK_SPEED_S		0
#define PCIE_LCTLSTS2_TGT_LINK_SPEED_25GT	1
#define PCIE_LCTLSTS2_TGT_LINK_SPEED_5GT	2
#define PCIE_LCTLSTS2_HW_AUTO_DIS		BIT(5)

#define PCIE_MAX_PAYLOAD_128			0

struct grx500_pcie_port {
	int id;
	struct device *dev;
	void __iomem *csr_base;
	void __iomem *cfg_base;
	void __iomem *app_base;
	void __iomem *phy_base;

	struct reset_control *phy_rst;
	struct clk *core_clk;
	int reset_gpio;
	bool high_active_rst;

	u32 inbound_shift;
	u32 outbound_shift;
	u32 clkout_pin;

	bool inbound_swap;
	bool outbound_swap;
	u32 rst_interval;

	struct resource mem;
	struct resource io;
	struct pci_controller pcic;

	bool link_up;
	spinlock_t lock;
};

static bool lcpll_ssc_initialized;

static inline u32 pcie_rc_cfg_rd(struct grx500_pcie_port *lpp, u32 reg)
{
	return ltq_r32(lpp->csr_base + reg);
}

static inline void pcie_rc_cfg_wr(struct grx500_pcie_port *lpp, u32 val, u32 reg)
{
	ltq_w32(val, lpp->csr_base + reg);
}

static inline void pcie_rc_cfg_wr_mask(struct grx500_pcie_port *lpp, u32 clr, u32 set, u32 reg)
{
	ltq_w32_mask(clr, set, lpp->csr_base + reg);
}

static inline u32 pcie_app_rd(struct grx500_pcie_port *lpp, u32 reg)
{
	return ltq_r32(lpp->app_base + reg);
}

static inline void pcie_app_wr(struct grx500_pcie_port *lpp, u32 val, u32 reg)
{
	ltq_w32(val, lpp->app_base + reg);
}

static inline void pcie_app_wr_mask(struct grx500_pcie_port *lpp, u32 clr, u32 set, u32 reg)
{
	ltq_w32_mask(clr, set, lpp->app_base + reg);
}

static inline void pcie_phy_w32_mask(struct grx500_pcie_port *lpp, u32 clr, u32 set, u32 reg)
{
	ltq_w32_mask(clr, set, lpp->phy_base + reg);
}

static inline u32 pcie_phy_r32(struct grx500_pcie_port *lpp, u32 reg)
{
	return ltq_r32(lpp->phy_base + reg);
}

static void grx500_pcie_lcpll_init(struct grx500_pcie_port *lpp)
{
	int i;

	/* 1. Power down PHY pipe */
	dev_info(lpp->dev, "port %d: powering down PHY pipe\n", lpp->id);
	pcie_phy_w32_mask(lpp, 0, PCIE_PHY_PIPE_PD_O, PCIE_PHY_CFG4);
	mdelay(1);

	if (!lcpll_ssc_initialized) {
		dev_info(lpp->dev, "initializing LCPLL SSC\n");
		/* Enable SSC and LCPLL in CGU */
		ltq_cgu_w32(0x10003004, PCIE_LCPLL_CFG1);
		ltq_cgu_w32(0x10103004, PCIE_LCPLL_CFG1);

		/* Toggle bit 0 from 0 to 1 */
		ltq_cgu_w32(0x00000190, PCIE_LCPLL_CFG0);
		ltq_cgu_w32(0x00000191, PCIE_LCPLL_CFG0);

		for (i = 0; i < 1000; i++) {
			if (ltq_cgu_r32(PCIE_LCPLL_CFG0) & LCPLL_CFG0_LOCKED)
				break;
			udelay(100);
		}
		if (i >= 1000)
			dev_warn(lpp->dev, "LCPLL not locked yet\n");

		/* Fixed coefficients */
		ltq_cgu_w32(0x0000FF60, PCIE_LCPLL_COEF_0);
		ltq_cgu_w32(0x0000FF60, PCIE_LCPLL_COEF_1);
		ltq_cgu_w32(0x0000FF60, PCIE_LCPLL_COEF_2);
		ltq_cgu_w32(0x0000FF60, PCIE_LCPLL_COEF_3);
		ltq_cgu_w32(0x000000A0, PCIE_LCPLL_COEF_4);
		ltq_cgu_w32(0x000000A0, PCIE_LCPLL_COEF_5);
		ltq_cgu_w32(0x000000A0, PCIE_LCPLL_COEF_6);
		ltq_cgu_w32(0x000000A0, PCIE_LCPLL_COEF_7);

		/* Program DIV and len parameters */
		ltq_cgu_w32(0x0000FF1C, PCIE_LCPLL_SSC_CTRL);
		ltq_cgu_w32(0x0000FF1F, PCIE_LCPLL_SSC_CTRL);
		ltq_cgu_w32(0x0000FF1C, PCIE_LCPLL_SSC_CTRL);
		ltq_cgu_w32(0x0000FF1D, PCIE_LCPLL_SSC_CTRL);
		mdelay(1);

		lcpll_ssc_initialized = true;
		dev_info(lpp->dev, "PCIe LCPLL SSC Mode Enabled\n");
	}

	/* 2. Power up PHY pipe */
	dev_info(lpp->dev, "port %d: powering up PHY pipe\n", lpp->id);
	pcie_phy_w32_mask(lpp, PCIE_PHY_PIPE_PD_O, 0, PCIE_PHY_CFG4);
	mdelay(1);
}

static inline void pcie_ep_rst_assert(struct grx500_pcie_port *lpp)
{
	if (!gpio_is_valid(lpp->reset_gpio))
		return;

	if (lpp->high_active_rst)
		gpio_set_value(lpp->reset_gpio, 1);
	else
		gpio_set_value(lpp->reset_gpio, 0);
}

static inline void pcie_ep_rst_deassert(struct grx500_pcie_port *lpp)
{
	if (!gpio_is_valid(lpp->reset_gpio))
		return;

	if (lpp->high_active_rst)
		gpio_set_value(lpp->reset_gpio, 0);
	else
		gpio_set_value(lpp->reset_gpio, 1);
}

static int grx500_pcie_hw_init(struct grx500_pcie_port *lpp)
{
	int i;
	u32 reg;

	/* Step 1: Pre-initialize endpoint reset pin to deasserted state for 100ms (matches vendor pcie_ep_rst_init) */
	if (gpio_is_valid(lpp->reset_gpio)) {
		dev_info(lpp->dev, "port %d: pre-initializing reset-gpio %d (deasserted 100ms)\n",
			 lpp->id, lpp->reset_gpio);
		gpio_direction_output(lpp->reset_gpio, lpp->high_active_rst ? 0 : 1);
		msleep(100);
	}

	/* Step 2: Setup RCU endianness for PCIe bridge */
	if (lpp->outbound_swap)
		ltq_endian_set(lpp->outbound_shift);
	else
		ltq_endian_clr(lpp->outbound_shift);

	if (lpp->inbound_swap)
		ltq_endian_set(lpp->inbound_shift);
	else
		ltq_endian_clr(lpp->inbound_shift);

	/* Step 3: Initialize common LCPLL and power up PHY pipe ONCE */
	grx500_pcie_lcpll_init(lpp);

	/* Step 4: Reset PCIe PHY via RCU */
	if (lpp->phy_rst) {
		dev_info(lpp->dev, "port %d: resetting PHY via RCU\n", lpp->id);
		reset_control_assert(lpp->phy_rst);
		udelay(10);
		reset_control_deassert(lpp->phy_rst);
		udelay(20);
	}

	/* Put PCIe endpoint into reset during controller setup (matches vendor pcie_device_rst_assert) */
	pcie_ep_rst_assert(lpp);

	/* Step 5: Enable controller core clock and verify in CGU */
	if (lpp->core_clk) {
		dev_info(lpp->dev, "port %d: enabling core clock\n", lpp->id);
		clk_prepare_enable(lpp->core_clk);
	}
	dev_info(lpp->dev, "port %d: CGU_CLK_SR2 = 0x%08x\n", lpp->id, ltq_cgu_r32(CGU_CLK_SR2));

	/* Enable 100 MHz PCIe reference clock output in RCU_IFMUX_CFG (matches vendor pcie_phy_clk_out) */
	if (lpp->clkout_pin) {
		ltq_pcie_clk_out(lpp->clkout_pin);
		dev_info(lpp->dev, "port %d: enabled PCIe REFCLK output (clkout bit %u)\n",
			 lpp->id, lpp->clkout_pin);
	}

	/* Step 6: Clear status registers */
	pcie_app_wr(lpp, 0, PCIE_RC_DR);
	pcie_rc_cfg_wr(lpp, 0, PCIE_PCICMDSTS);
	pcie_rc_cfg_wr(lpp, 0, PCIE_DCTLSTS);
	pcie_rc_cfg_wr(lpp, 0, PCIE_LCTLSTS);
	pcie_rc_cfg_wr(lpp, 0, PCIE_UESR);
	pcie_rc_cfg_wr(lpp, 0, PCIE_CESR);

	/* Disable LTSSM during configuration */
	pcie_app_wr(lpp, 0, PCIE_RC_CCR);

	/* Setup Bus Numbers (Primary 0, Secondary 1, Subordinate 0xFF like vendor) */
	reg = SM(0, PCIE_BNR_PRIMARY_BUS_NUM) |
	      SM(1, PCIE_PNR_SECONDARY_BUS_NUM) |
	      SM(0xFF, PCIE_PNR_SUB_BUS_NUM);
	pcie_rc_cfg_wr(lpp, reg, PCIE_BNR);

	/* Setup Memory window in bridge */
	reg = SM((lpp->mem.end >> 20), PCIE_MBML_MEM_LIMIT_ADDR) |
	      SM((lpp->mem.start >> 20), PCIE_MBML_MEM_BASE_ADDR);
	pcie_rc_cfg_wr(lpp, reg, PCIE_MBML);
	pcie_rc_cfg_wr(lpp, reg, PCIE_PMBL);

	/* Setup IO Address Range in bridge if valid */
	if (lpp->io.end > lpp->io.start) {
		reg = SM(((lpp->io.end) >> 12), PCIE_IOBLSECS_IO_LIMIT_ADDR) |
		      SM((lpp->io.start >> 12), PCIE_IOBLSECS_IO_BASE_ADDR);
		pcie_rc_cfg_wr(lpp, reg, PCIE_IOBLSECS);
	}

	/* Setup MSI Capability (matches vendor pcie_msi_setup) */
	reg = pcie_rc_cfg_rd(lpp, PCIE_MCAPR);
	reg &= ~PCIE_MCAPR_ADDR64_CAP;
	reg |= PCIE_MCAPR_MSI_ENABLE;
	reg &= ~(PCIE_MCAPR_MULTI_MSG_CAP | PCIE_MCAPR_MULTI_MSG_ENABLE);
	pcie_rc_cfg_wr(lpp, reg, PCIE_MCAPR);

	/* Setup Power Management: Enable PME, Soft Reset enabled (matches vendor pcie_pm_setup) */
	pcie_rc_cfg_wr_mask(lpp, 0, PCIE_PM_CSR_PME_ENABLE | PCIE_PM_CSR_SW_RST, PCIE_PM_CSR);

	/* Device capabilities & control */
	reg = pcie_rc_cfg_rd(lpp, PCIE_DCAP);
	reg |= PCIE_DCAP_ROLE_BASE_ERR_REPORT;
	reg |= SM(PCIE_MAX_PAYLOAD_128, PCIE_DCAP_MAX_PAYLOAD_SIZE);
	reg &= ~(PCIE_DCAP_EP_L0S_LATENCY | PCIE_DCAP_EP_L1_LATENCY);
	pcie_rc_cfg_wr(lpp, reg, PCIE_DCAP);

	reg = pcie_rc_cfg_rd(lpp, PCIE_DCTLSTS);
	reg |= SM(PCIE_MAX_PAYLOAD_128, PCIE_DCTLSTS_MAX_READ_SIZE) |
	       SM(PCIE_MAX_PAYLOAD_128, PCIE_DCTLSTS_MAX_PAYLOAD_SIZE);
	reg |= PCIE_DCTLSTS_RELAXED_ORDERING_EN | PCIE_DCTLSTS_ERR_EN |
	       PCIE_DCTLSTS_NO_SNOOP_EN;
	pcie_rc_cfg_wr(lpp, reg, PCIE_DCTLSTS);

	/* Link capabilities & control */
	reg = pcie_rc_cfg_rd(lpp, PCIE_LCAP);
	reg &= ~PCIE_LCAP_L0S_EIXT_LATENCY;
	reg |= SM(3, PCIE_LCAP_L0S_EIXT_LATENCY);
	pcie_rc_cfg_wr(lpp, reg, PCIE_LCAP);

	reg = pcie_rc_cfg_rd(lpp, PCIE_LCTLSTS);
	reg &= ~PCIE_LCTLSTS_LINK_DISABLE;
	reg |= PCIE_LCTLSTS_SLOT_CLK_CFG | PCIE_LCTLSTS_COM_CLK_CFG;
	reg &= ~PCIE_LCTLSTS_ASPM_ENABLE;
	reg &= ~PCIE_LCTLSTS_RCB128;
	pcie_rc_cfg_wr(lpp, reg, PCIE_LCTLSTS);

	/* Gen2 speed change setup (matches vendor pcie_speed_change_setup) */
	reg = pcie_rc_cfg_rd(lpp, PCIE_GEN2_CTRL);
	reg |= PCIE_GEN2_CTRL_SPEED_CHANGE;
	reg &= ~PCIE_GEN2_CTRL_FTS;
	reg |= SM(PCIE_GEN2_DEFAULT_FTS_NUM, PCIE_GEN2_CTRL_FTS);
	pcie_rc_cfg_wr(lpp, reg, PCIE_GEN2_CTRL);

	/* Error reporting setup */
	pcie_rc_cfg_wr_mask(lpp, 0, PCIE_INTRBCTRL_SERR_ENABLE |
			    PCIE_INTRBCTRL_PARITY_ERR_RESP_ENABLE,
			    PCIE_INTRBCTRL);
	pcie_rc_cfg_wr_mask(lpp, PCIE_ALL_UNCORRECTABLE_ERR, 0, PCIE_UEMR);
	pcie_rc_cfg_wr(lpp, PCIE_ALL_UNCORRECTABLE_ERR, PCIE_UESR);
	pcie_rc_cfg_wr_mask(lpp, PCIE_CORRECTABLE_ERR, 0, PCIE_CEMR);

	/* Virtual Channel setup */
	pcie_rc_cfg_wr_mask(lpp, PCIE_PVC2_VC_ARB_WRR,
			    PCIE_PVC2_VC_ARB_16P_FIXED_WRR, PCIE_PVC2);
	pcie_rc_cfg_wr_mask(lpp, PCIE_VC0_RC_REJECT_SNOOP, 0, PCIE_VC0_RC);

	/* Port logic setup */
	reg = pcie_rc_cfg_rd(lpp, PCIE_AFR);
	reg &= ~(PCIE_AFR_FTS_NUM | PCIE_AFR_COM_FTS_NUM);
	reg |= SM(PCIE_AFR_FTS_NUM_DEFAULT, PCIE_AFR_FTS_NUM) |
	       SM(PCIE_AFR_FTS_NUM_DEFAULT, PCIE_AFR_COM_FTS_NUM);
	reg &= ~(PCIE_AFR_L0S_ENTRY_LATENCY | PCIE_AFR_L1_ENTRY_LATENCY);
	reg |= SM(PCIE_AFR_L0S_ENTRY_LATENCY_DEFAULT, PCIE_AFR_L0S_ENTRY_LATENCY) |
	       SM(PCIE_AFR_L1_ENTRY_LATENCY_DEFAULT, PCIE_AFR_L1_ENTRY_LATENCY);
	pcie_rc_cfg_wr(lpp, reg, PCIE_AFR);

	pcie_rc_cfg_wr_mask(lpp, 0, PCIE_PLCR_DLL_LINK_EN, PCIE_PLCR);
	pcie_rc_cfg_wr_mask(lpp, PCIE_LSR_ACKNAK_DISABLE | PCIE_LSR_FC_DISABLE, 0, PCIE_LSR);

	reg = pcie_rc_cfg_rd(lpp, PCIE_STRFMR);
	reg |= PCIE_STRFMR_RX_CFG_TRANS_ENABLE | PCIE_STRFMR_RX_IO_TRANS_ENABLE;
	reg &= ~PCIE_STRFMR_FC_WDT_DISABLE;
	pcie_rc_cfg_wr(lpp, reg, PCIE_STRFMR);

	pcie_rc_cfg_wr_mask(lpp, 0, PCIE_FMR2_VENDOR_MSG1_PASSED_TO_TRGT1 |
			    PCIE_FMR2_VENDOR_MSG0_PASSED_TO_TRGT1, PCIE_FMR2);

	/* VC0 Completion Receive Queue Control: bypass mode (matches vendor pcie_port_logic_setup) */
	reg = pcie_rc_cfg_rd(lpp, PCIE_VC0_CRQCR);
	reg &= ~PCIE_VC0_CRQCR_CPL_TLP_QUEUE_MODE;
	reg |= SM(PCIE_VC0_TLP_QUEUE_MODE_BYPASS, PCIE_VC0_CRQCR_CPL_TLP_QUEUE_MODE);
	pcie_rc_cfg_wr(lpp, reg, PCIE_VC0_CRQCR);

	/* Suppress AHB bus errors on invalid config accesses */
	pcie_app_wr(lpp, PCIE_AHB_CTRL_BUS_ERROR_SUPPRESS, PCIE_AHB_CTRL);

	/* Hold endpoint reset for rst_interval (200 ms) while clocks and registers are ready */
	if (gpio_is_valid(lpp->reset_gpio)) {
		dev_info(lpp->dev, "port %d: holding reset for %d ms before release\n",
			 lpp->id, lpp->rst_interval);
		msleep(lpp->rst_interval);
	}

	/* Vendor pcie_app_loigc_setup sequence: deassert PERST# and immediately enable LTSSM */
	pcie_ep_rst_deassert(lpp);
	pcie_app_wr(lpp, PCIE_RC_CCR_LTSSM_ENABLE, PCIE_RC_CCR);

	dev_info(lpp->dev, "port %d: LTSSM enabled, waiting for PHY link up\n", lpp->id);

	/* Wait for retrain pending to clear (vendor timeout 10 * 10us) */
	for (i = 0; i < 100; i++) {
		if (!(pcie_rc_cfg_rd(lpp, PCIE_LCTLSTS) & PCIE_LCTLSTS_RETRAIN_PENDING))
			break;
		udelay(10);
	}

	/* Wait for PHY link up with generous timeout (up to 2000 ms) */
	dev_info(lpp->dev, "port %d: waiting for PHY link up\n", lpp->id);
	for (i = 0; i < 2000; i++) {
		if (pcie_app_rd(lpp, PCIE_PHY_SR) & PCIE_PHY_SR_PHY_LINK_UP)
			break;
		usleep_range(1000, 1500);
	}

	if (i >= 2000) {
		u32 phy_sr = pcie_app_rd(lpp, PCIE_PHY_SR);
		u32 rc_dr = pcie_app_rd(lpp, PCIE_RC_DR);
		u32 lctlsts = pcie_rc_cfg_rd(lpp, PCIE_LCTLSTS);
		u32 lctlsts2 = pcie_rc_cfg_rd(lpp, PCIE_LCTLSTS2);
		int gval = gpio_is_valid(lpp->reset_gpio) ? gpio_get_value(lpp->reset_gpio) : -1;

		dev_info(lpp->dev,
			 "port %d: PHY link down (PHY_SR=0x%08x RC_DR=0x%08x LCTLSTS=0x%08x LCTLSTS2=0x%08x gpio=%d CFG3=0x%08x CFG4=0x%08x)\n",
			 lpp->id, phy_sr, rc_dr, lctlsts, lctlsts2, gval,
			 pcie_phy_r32(lpp, PCIE_PHY_CFG3),
			 pcie_phy_r32(lpp, PCIE_PHY_CFG4));
		return -ENODEV;
	}
	dev_info(lpp->dev, "port %d: PHY link is UP\n", lpp->id);

	/* Wait for Data Link layer up (timeout ~100ms) */
	dev_info(lpp->dev, "port %d: waiting for DLL link up\n", lpp->id);
	for (i = 0; i < 5000; i++) {
		if (pcie_app_rd(lpp, PCIE_RC_DR) & PCIE_RC_DR_DLL_UP)
			break;
		udelay(10);
	}
	if (i >= 5000) {
		dev_warn(lpp->dev, "port %d: DLL link down (RC_DR=0x%08x)\n",
			 lpp->id, pcie_app_rd(lpp, PCIE_RC_DR));
		return -ENODEV;
	}

	if (!(pcie_rc_cfg_rd(lpp, PCIE_LCTLSTS) & PCIE_LCTLSTS_DLL_ACTIVE)) {
		dev_warn(lpp->dev, "port %d: DLL is not active\n", lpp->id);
		return -ENODEV;
	}
	dev_info(lpp->dev, "port %d: DLL link is UP and active\n", lpp->id);

	/* Update Replay Time Limit */
	reg = pcie_rc_cfg_rd(lpp, PCIE_ALTRT);
	reg &= ~PCIE_ALTRT_REPLAY_TIME_LIMIT;
	reg |= SM(839, PCIE_ALTRT_REPLAY_TIME_LIMIT);
	pcie_rc_cfg_wr(lpp, reg, PCIE_ALTRT);

	lpp->link_up = true;
	dev_info(lpp->dev, "port %d: PCIe link UP\n", lpp->id);
	return 0;
}


static u32 grx500_pcie_bus_addr(u8 bus_num, u16 devfn, int where)
{
	u32 addr;

	if (!bus_num) {
		/* type 0 */
		addr = ((PCI_SLOT(devfn) & 0x1F) << 15) |
		       ((PCI_FUNC(devfn) & 0x7) << 12) |
		       ((where & 0xFFF) & ~3);
	} else {
		/* type 1 */
		addr = ((bus_num & 0x7) << 20) |
		       ((PCI_SLOT(devfn) & 0x1F) << 15) |
		       ((PCI_FUNC(devfn) & 0x7) << 12) |
		       ((where & 0xFFF) & ~3);
	}
	return addr;
}

static int grx500_pcie_root_bus_nr(struct pci_bus *bus)
{
	while (bus->parent)
		bus = bus->parent;
	return bus->number;
}

static int grx500_pcie_read_config(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 *value)
{
	struct pci_controller *hose = (struct pci_controller *)bus->sysdata;
	struct grx500_pcie_port *lpp = container_of(hose, struct grx500_pcie_port, pcic);
	unsigned long flags;
	u32 data = 0;
	int bus_offset;
	int rel_bus;

	if (unlikely(size != 1 && size != 2 && size != 4))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (unlikely(((size - 1) & where)))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	bus_offset = grx500_pcie_root_bus_nr(bus);
	rel_bus = bus->number - bus_offset;
	if (bus->parent == NULL)
		rel_bus = 0;

	/* Point-to-point PCIe link: only slot 0 exists on root bus and direct endpoint bus */
	if (PCI_SLOT(devfn) != 0) {
		*value = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (!lpp->link_up && rel_bus != 0) {
		*value = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (rel_bus > 8) {
		*value = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	spin_lock_irqsave(&lpp->lock, flags);

	if (rel_bus == 0) {
		/* Access Root Complex config space via csr_base */
		data = ltq_r32(lpp->csr_base + (where & ~3));
		if (where == PCI_PRIMARY_BUS) {
			u8 primary = (data & 0xff) + bus_offset;
			u8 secondary = ((data >> 8) & 0xff) + bus_offset;
			u8 subordinate = (data >> 16) & 0xff;

			if (subordinate != 0xff)
				subordinate += bus_offset;
			data = (data & 0xff000000) | primary |
			       (secondary << 8) | (subordinate << 16);
		}
	} else {
		/* Access Endpoint config space via cfg_base */
		u32 addr = grx500_pcie_bus_addr(rel_bus, devfn, where);
		data = ltq_r32(lpp->cfg_base + addr);
		if (lpp->outbound_swap)
			data = le32_to_cpu(data);
	}

	spin_unlock_irqrestore(&lpp->lock, flags);

	switch (size) {
	case 1:
		data = (data >> ((where & 3) * 8)) & 0xff;
		break;
	case 2:
		data = (data >> ((where & 2) * 8)) & 0xffff;
		break;
	case 4:
		break;
	}

	*value = data;
	return PCIBIOS_SUCCESSFUL;
}

static int grx500_pcie_write_config(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 value)
{
	struct pci_controller *hose = (struct pci_controller *)bus->sysdata;
	struct grx500_pcie_port *lpp = container_of(hose, struct grx500_pcie_port, pcic);
	unsigned long flags;
	u32 data;
	int s;
	int bus_offset;
	int rel_bus;

	if (unlikely(size != 1 && size != 2 && size != 4))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (unlikely(((size - 1) & where)))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	bus_offset = grx500_pcie_root_bus_nr(bus);
	rel_bus = bus->number - bus_offset;
	if (bus->parent == NULL)
		rel_bus = 0;

	if (PCI_SLOT(devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (!lpp->link_up && rel_bus != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (rel_bus > 8)
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&lpp->lock, flags);

	if (rel_bus == 0) {
		if (where == PCI_PRIMARY_BUS) {
			u8 primary = value & 0xff;
			u8 secondary = (value >> 8) & 0xff;
			u8 subordinate = (value >> 16) & 0xff;

			if (primary >= bus_offset && primary != 0xff)
				primary -= bus_offset;
			if (secondary >= bus_offset && secondary != 0xff)
				secondary -= bus_offset;
			if (subordinate >= bus_offset && subordinate != 0xff)
				subordinate -= bus_offset;
			value = (value & 0xff000000) | primary |
				(secondary << 8) | (subordinate << 16);
		} else if (where == PCI_SUBORDINATE_BUS) {
			u8 subordinate = value & 0xff;

			if (subordinate >= bus_offset && subordinate != 0xff)
				subordinate -= bus_offset;
			value = subordinate;
		}

		data = ltq_r32(lpp->csr_base + (where & ~3));
		switch (size) {
		case 1:
			s = ((where & 3) * 8);
			data &= ~(0xff << s);
			data |= ((value & 0xff) << s);
			break;
		case 2:
			s = ((where & 2) * 8);
			data &= ~(0xffff << s);
			data |= ((value & 0xffff) << s);
			break;
		case 4:
			data = value;
			break;
		}
		ltq_w32(data, lpp->csr_base + (where & ~3));
	} else {
		u32 addr = grx500_pcie_bus_addr(rel_bus, devfn, where);
		data = ltq_r32(lpp->cfg_base + addr);
		if (lpp->outbound_swap)
			data = le32_to_cpu(data);
		switch (size) {
		case 1:
			s = ((where & 3) * 8);
			data &= ~(0xff << s);
			data |= ((value & 0xff) << s);
			break;
		case 2:
			s = ((where & 2) * 8);
			data &= ~(0xffff << s);
			data |= ((value & 0xffff) << s);
			break;
		case 4:
			data = value;
			break;
		}
		if (lpp->outbound_swap)
			data = cpu_to_le32(data);
		ltq_w32(data, lpp->cfg_base + addr);
	}

	spin_unlock_irqrestore(&lpp->lock, flags);
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops grx500_pcie_ops = {
	.read = grx500_pcie_read_config,
	.write = grx500_pcie_write_config,
};

#ifndef PCI_VENDOR_ID_INFINEON
#define PCI_VENDOR_ID_INFINEON 0x15D1
#endif
#ifndef PCI_VENDOR_ID_LANTIQ
#define PCI_VENDOR_ID_LANTIQ 0x1BEF
#endif

static void grx500_pcie_fixup_bridge(struct pci_dev *dev)
{
	u16 reg;

	if ((dev->class >> 16) == PCI_BASE_CLASS_BRIDGE) {
		pci_read_config_word(dev, PCI_COMMAND, &reg);
		reg |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
		pci_write_config_word(dev, PCI_COMMAND, reg);
		pci_info(dev, "fixup host controller (%04x:%04x)\n", dev->vendor, dev->device);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INFINEON, PCI_ANY_ID, grx500_pcie_fixup_bridge);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_LANTIQ, PCI_ANY_ID, grx500_pcie_fixup_bridge);

static void grx500_pcie_rc_class_early_fixup(struct pci_dev *dev)
{
	if ((dev->bus->parent == NULL) && (dev->devfn == 0)) {
		dev->class = (PCI_CLASS_BRIDGE_PCI << 8) | (dev->class & 0xff);
		pci_info(dev, "fixed host bridge to pci-pci bridge\n");
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INFINEON, PCI_ANY_ID, grx500_pcie_rc_class_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_LANTIQ, PCI_ANY_ID, grx500_pcie_rc_class_early_fixup);

static int grx500_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct grx500_pcie_port *lpp;
	struct resource *res;
	u32 prop;
	int id;
	int ret;

	id = of_alias_get_id(np, "pcie");
	if (id < 0) {
		dev_err(dev, "failed to get alias id: %d\n", id);
		return id;
	}

	lpp = devm_kzalloc(dev, sizeof(*lpp), GFP_KERNEL);
	if (!lpp)
		return -ENOMEM;

	lpp->id = id;
	lpp->dev = dev;
	spin_lock_init(&lpp->lock);

	of_property_read_u32(np, "lantiq,inbound-shift", &lpp->inbound_shift);
	of_property_read_u32(np, "lantiq,outbound-shift", &lpp->outbound_shift);
	of_property_read_u32(np, "lantiq,clkout", &lpp->clkout_pin);

	lpp->rst_interval = 200;
	if (!of_property_read_u32(np, "lantiq,rst-interval", &prop))
		lpp->rst_interval = prop;

	lpp->high_active_rst = false;
	if (!of_property_read_u32(np, "lantiq,rst-high-active", &prop))
		lpp->high_active_rst = !!prop;

	lpp->inbound_swap = true;
	if (!of_property_read_u32(np, "lantiq,inbound-swap", &prop))
		lpp->inbound_swap = !!prop;

	lpp->outbound_swap = false;
	if (!of_property_read_u32(np, "lantiq,outbound-swap", &prop))
		lpp->outbound_swap = !!prop;

	lpp->phy_rst = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(lpp->phy_rst)) {
		dev_warn(dev, "failed to get phy reset: %ld\n", PTR_ERR(lpp->phy_rst));
		lpp->phy_rst = NULL;
	}

	lpp->core_clk = devm_clk_get_optional(dev, "ctl");
	if (IS_ERR(lpp->core_clk)) {
		dev_err(dev, "failed to get ctl clock: %ld\n", PTR_ERR(lpp->core_clk));
		return PTR_ERR(lpp->core_clk);
	}

	lpp->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(lpp->reset_gpio)) {
		ret = devm_gpio_request(dev, lpp->reset_gpio, "pcie-reset");
		if (ret) {
			dev_err(dev, "failed to request reset-gpio %d: %d\n",
				lpp->reset_gpio, ret);
			return ret;
		}
		dev_info(dev, "acquired reset-gpio %d\n", lpp->reset_gpio);
	} else if (lpp->reset_gpio == -EPROBE_DEFER) {
		dev_info(dev, "reset-gpio deferred\n");
		return -EPROBE_DEFER;
	} else {
		dev_info(dev, "no valid reset-gpio found (%d)\n", lpp->reset_gpio);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csr");
	if (!res)
		return -EINVAL;
	lpp->csr_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!lpp->csr_base)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	if (!res)
		return -EINVAL;
	lpp->cfg_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!lpp->cfg_base)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "app");
	if (!res)
		return -EINVAL;
	lpp->app_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!lpp->app_base)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	if (!res)
		return -EINVAL;
	lpp->phy_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!lpp->phy_base)
		return -ENOMEM;

	platform_set_drvdata(pdev, lpp);

	lpp->pcic.pci_ops = &grx500_pcie_ops;
	lpp->pcic.io_resource = &lpp->io;
	lpp->pcic.mem_resource = &lpp->mem;

	pci_load_of_ranges(&lpp->pcic, np);

	ret = grx500_pcie_hw_init(lpp);
	if (ret) {
		dev_info(dev, "Lantiq PCIe Port %d link not trained (%d)\n", lpp->id, ret);
		pcie_ep_rst_deassert(lpp);
		return ret;
	}

	register_pci_controller(&lpp->pcic);

	pr_info("Lantiq PCIe Root Complex Port %d Init Done\n", lpp->id);
	return 0;
}

static const struct of_device_id grx500_pcie_match[] = {
	{ .compatible = "lantiq,pcie-xrx500" },
	{},
};
MODULE_DEVICE_TABLE(of, grx500_pcie_match);

static struct platform_driver grx500_pcie_driver = {
	.probe = grx500_pcie_probe,
	.driver = {
		.name = "grx500-pcie",
		.of_match_table = grx500_pcie_match,
	},
};

static int __init grx500_pcie_init(void)
{
	return platform_driver_register(&grx500_pcie_driver);
}
arch_initcall(grx500_pcie_init);

MODULE_AUTHOR("Lantiq / Intel / OpenWrt");
MODULE_DESCRIPTION("Lantiq / Intel GRX500 PCIe Root Complex Driver");
MODULE_LICENSE("GPL v2");
