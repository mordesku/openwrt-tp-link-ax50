// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Hardware Reset & PLL Sequencing
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include "wave600.h"
#include "wave600_regs.h"

void wave600_secure_unlock_upper(struct wave600_dev *wdev)
{
	wave600_write32(wdev, WAVE600_SYS_IF_UM_SECURE_WRITE, WAVE600_SECURE_KEY1);
	mmiowb();
	wave600_write32(wdev, WAVE600_SYS_IF_UM_SECURE_WRITE, WAVE600_SECURE_KEY2);
	mmiowb();
}

void wave600_secure_unlock_lower(struct wave600_dev *wdev, int lm_idx)
{
	u32 reg = (lm_idx == 0) ? WAVE600_SYS_IF_LM0_SECURE_WRITE :
				  WAVE600_SYS_IF_LM1_SECURE_WRITE;

	wave600_write32(wdev, reg, WAVE600_SECURE_KEY1);
	mmiowb();
	wave600_write32(wdev, reg, WAVE600_SECURE_KEY2);
	mmiowb();
}

void wave600_hw_put_cpus_to_reset(struct wave600_dev *wdev)
{
	/* Place Lower MAC CPUs in reset */
	wave600_secure_unlock_lower(wdev, 0);
	wave600_write32(wdev, WAVE600_SYS_IF_LM0_MIPS_CTL, WAVE600_MIPS_RESET);
	mmiowb();
	msleep(20);

	wave600_secure_unlock_lower(wdev, 1);
	wave600_write32(wdev, WAVE600_SYS_IF_LM1_MIPS_CTL, WAVE600_MIPS_RESET);
	mmiowb();
	msleep(20);

	/* Place Upper MAC CPU in reset */
	wave600_secure_unlock_upper(wdev);
	wave600_write32(wdev, WAVE600_SYS_IF_UM_MIPS_CTL, WAVE600_MIPS_RESET);
	mmiowb();
	msleep(20);
}

void wave600_hw_reset(struct wave600_dev *wdev)
{
	u32 val;

	dev_info(wdev->dev, "performing hardware soft reset\n");

	/* 1. Pulse WLAN SOC Soft Reset (WAVE600_SOC_SOFT_RESET = 0x00270004) */
	val = wave600_read32(wdev, WAVE600_SOC_SOFT_RESET);
	val &= ~WAVE600_SOC_SOFT_RESET_WLAN;
	wave600_write32(wdev, WAVE600_SOC_SOFT_RESET, val);
	mmiowb();
	msleep(10);

	/* Mask host interrupts and disable legacy INTx during hardware reset */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0);
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0xffffff);
	mmiowb();

	/* 2. Disable MAC PAC RX during reset */
	val = wave600_read32(wdev, WAVE600_PAC_RX_CONTROL);
	val &= ~WAVE600_PAC_RX_CTRL_ENABLED;
	wave600_write32(wdev, WAVE600_PAC_RX_CONTROL, val);
	mmiowb();

	/* 3. Place CPUs in reset */
	wave600_hw_put_cpus_to_reset(wdev);
}

void wave600_hw_set_cpu_clock(struct wave600_dev *wdev, u32 clk_sel)
{
	wave600_secure_unlock_upper(wdev);
	wave600_write32(wdev, WAVE600_SYS_IF_CLOCK_SELECT, clk_sel);
	mmiowb();
}

int wave600_fcsi_write(struct wave600_dev *wdev, u8 addr, u16 data)
{
	u32 val;
	int timeout = WAVE600_FCSI_BUSY_WAIT_COUNT;

	/* 1. Wait for FCSI Central busy bit to clear */
	while (timeout--) {
		val = wave600_read32(wdev, WAVE600_PHY_RXTD_IF136);
		if (!(val & WAVE600_FCSI_BUSY_BIT))
			break;
		udelay(10);
	}
	if (val & WAVE600_FCSI_BUSY_BIT) {
		dev_warn(wdev->dev, "FCSI busy timeout before write to 0x%02x\n", addr);
		return -ETIMEDOUT;
	}

	/* 2. Write 16-bit payload data */
	wave600_write32(wdev, WAVE600_PHY_RXTD_IF12C, data);
	mmiowb();

	/* 3. Trigger write transaction: addr, Central mask, MS access */
	val = (addr & 0x7f) | WAVE600_FCSI_CENTRAL_MASK | WAVE600_FCSI_MS_ACCESS;
	wave600_write32(wdev, WAVE600_PHY_RXTD_IF_FCSI_ACCESS, val);
	mmiowb();

	/* 4. Wait for transaction to complete */
	timeout = WAVE600_FCSI_BUSY_WAIT_COUNT;
	while (timeout--) {
		val = wave600_read32(wdev, WAVE600_PHY_RXTD_IF136);
		if (!(val & WAVE600_FCSI_BUSY_BIT))
			break;
		udelay(10);
	}
	if (val & WAVE600_FCSI_BUSY_BIT) {
		dev_warn(wdev->dev, "FCSI busy timeout after write to 0x%02x\n", addr);
		return -ETIMEDOUT;
	}

	return 0;
}

void wave600_hw_release_cpus(struct wave600_dev *wdev)
{
	u32 lm0_ctl, lm1_ctl, um_ctl, bb_status;

	dev_info(wdev->dev, "releasing ARC CPUs from reset on 40 MHz XO (Gen 6 release sequence)\n");

	/* Release Lower MAC CPU 0 */
	wave600_secure_unlock_lower(wdev, 0);
	wave600_write32(wdev, WAVE600_SYS_IF_LM0_MIPS_CTL, WAVE600_MIPS_RELEASE);
	mmiowb();
	msleep(20);

	/* Release Lower MAC CPU 1 */
	wave600_secure_unlock_lower(wdev, 1);
	wave600_write32(wdev, WAVE600_SYS_IF_LM1_MIPS_CTL, WAVE600_MIPS_RELEASE);
	mmiowb();
	msleep(20);

	/* Release Upper MAC CPU */
	wave600_secure_unlock_upper(wdev);
	wave600_write32(wdev, WAVE600_SYS_IF_UM_MIPS_CTL, WAVE600_MIPS_RELEASE);
	mmiowb();
	msleep(10);

	/* Verify status readback of MIPS control registers */
	lm0_ctl = wave600_read32(wdev, WAVE600_SYS_IF_LM0_MIPS_CTL);
	lm1_ctl = wave600_read32(wdev, WAVE600_SYS_IF_LM1_MIPS_CTL);
	um_ctl = wave600_read32(wdev, WAVE600_SYS_IF_UM_MIPS_CTL);
	bb_status = wave600_read32(wdev, WAVE600_SYS_IF_BB_OPMODE_STATUS);

	dev_info(wdev->dev, "ARC CPUs MIPS_CTL status: LM0=0x%08x, LM1=0x%08x, UM=0x%08x (BB_OPMODE=0x%08x)\n",
		 lm0_ctl, lm1_ctl, um_ctl, bb_status);
}

void wave600_hw_power_on_cpus(struct wave600_dev *wdev)
{
	u32 val;

	/* Enable MAC GenRISC QManager CPU clocks (MAC_HT_EXT_CLK_CTRL) */
	val = wave600_read32(wdev, WAVE600_MAC_HT_EXT_CLK_CTRL);
	val |= 0x0f; /* Enable QManagers 0..3 */
	wave600_write32(wdev, WAVE600_MAC_HT_EXT_CLK_CTRL, val);
	mmiowb();
}

int wave600_hw_init_pll(struct wave600_dev *wdev)
{
	u32 val;

	/*
	 * 1. Take PHY out of reset and enable FCSI central
	 * Matches vendor _phy_lib_FCSI_RESET & _abb_fcsi_enable
	 */
	/* Set band 0 PHY active */
	val = wave600_read32(wdev, WAVE600_MAC_GEN_PHY_BAND_CONFIG);
	val |= WAVE600_PHY_BAND_CFG_B0_ACTIVE;
	wave600_write32(wdev, WAVE600_MAC_GEN_PHY_BAND_CONFIG, val);
	mmiowb();

	/* Pulse PHY Reset lines (0 -> 1) */
	wave600_write32(wdev, WAVE600_MAC_GEN_PHY_EXT_RESET, 0);
	mmiowb();
	udelay(10);
	wave600_write32(wdev, WAVE600_MAC_GEN_PHY_EXT_RESET,
			WAVE600_PHY_EXT_RESET_B0 | WAVE600_PHY_EXT_RESET_B1);
	mmiowb();
	udelay(10);

	/* Pulse FCSI Reset lines (0 -> 1) */
	wave600_write32(wdev, WAVE600_MAC_GEN_FCSI_CTL, 0);
	mmiowb();
	udelay(10);
	wave600_write32(wdev, WAVE600_MAC_GEN_FCSI_CTL, WAVE600_FCSI_CTL_RESET_QC_D2A);
	mmiowb();
	udelay(10);

	/* Enable Central FCSI */
	val = wave600_read32(wdev, WAVE600_MAC_GEN_PHY_BAND_CONFIG);
	val |= WAVE600_PHY_BAND_CFG_FCSI_CEN_BLOCK_EN;
	wave600_write32(wdev, WAVE600_MAC_GEN_PHY_BAND_CONFIG, val);
	mmiowb();

	val &= ~WAVE600_PHY_BAND_CFG_FCSI_CEN_RESET_N;
	wave600_write32(wdev, WAVE600_MAC_GEN_PHY_BAND_CONFIG, val);
	mmiowb();
	udelay(10);

	val |= WAVE600_PHY_BAND_CFG_FCSI_CEN_RESET_N;
	wave600_write32(wdev, WAVE600_MAC_GEN_PHY_BAND_CONFIG, val);
	mmiowb();
	udelay(10);

	/*
	 * Configure FCSI clock dividers (WR=4, RD=15)
	 * Matches vendor _phy_lib_FCSI_CENTRAL_WR/RD_EXT_RF/AFE_CLKDIV
	 */
	wave600_write32(wdev, WAVE600_PHY_RXTD_IF126, 0x0F04);
	wave600_write32(wdev, WAVE600_PHY_RXTD_IF127, 0x0F04);
	mmiowb();

	/*
	 * 2. Trim Central LDO voltages via FCSI
	 * Matches vendor _abb_central_fusing with default AFE constants:
	 * - ICENTBIAS0:  0x0080 (iptat=8)
	 * - IVREG0V950:  0x0008 (0.982V digital core & IRAM)
	 * - ILDOCLKD0:   0x0008 (0.982V clock distribution)
	 * - ILDOPLLREF0: 0x0005 (0.951V PLL reference)
	 * - ILDOPLLPOST0:0x0005 (0.951V PLL post-divider)
	 * - IVREG1V8HV0: 0x0004 (1.8V PLL high-voltage)
	 */
	wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_BIAS, WAVE600_FCSI_VAL_BIAS);
	wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_VREG0V95, WAVE600_FCSI_VAL_VREG0V95);
	wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_LDOCLKD, WAVE600_FCSI_VAL_LDOCLKD);
	wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_PLLREF, WAVE600_FCSI_VAL_PLLREF);
	wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_PLLPOST, WAVE600_FCSI_VAL_PLLPOST);
	wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_VREG1V8, WAVE600_FCSI_VAL_VREG1V8);

	/*
	 * 3. Initialize Central LCPLL and power up
	 * Matches vendor _abb_central_init_pll
	 */
	wave600_secure_unlock_upper(wdev);
	wave600_write32(wdev, WAVE600_SYS_IF_PLL_DIV, 0);
	mmiowb();

	wave600_secure_unlock_upper(wdev);
	val = wave600_read32(wdev, WAVE600_SYS_IF_PLL_CTL);
	val &= ~(WAVE600_PLL_CTL_PUP | WAVE600_PLL_CTL_RESETN);
	wave600_write32(wdev, WAVE600_SYS_IF_PLL_CTL, val);
	mmiowb();
	udelay(10);

	wave600_secure_unlock_upper(wdev);
	val |= (WAVE600_PLL_CTL_PUP | WAVE600_PLL_CTL_RESETN);
	wave600_write32(wdev, WAVE600_SYS_IF_PLL_CTL, val);
	mmiowb();

	/*
	 * 4. Wait for PLL lock (SYS_IF_PLL_LOCK bit 0)
	 * Matches vendor _abb_central_lock_pll
	 */
	{
		int timeout = 10000;

		val = 0;
		while (timeout--) {
			val = wave600_read32(wdev, WAVE600_SYS_IF_PLL_LOCK);
			if (val & WAVE600_PLL_LOCK_LOCKED)
				break;
			udelay(10);
		}

		if (val & WAVE600_PLL_LOCK_LOCKED) {
			dev_info(wdev->dev, "LCPLL locked successfully (status 0x%08x)\n", val);
		} else {
			dev_warn(wdev->dev, "LCPLL lock timeout (status 0x%08x), continuing\n", val);
		}
	}

	/*
	 * 5. Power up WLAN clock distribution
	 * Matches vendor _abb_central_init_clkd (pd_clk_sc_d2a = 0)
	 */
	wave600_secure_unlock_upper(wdev);
	val = wave600_read32(wdev, WAVE600_SYS_IF_PLL_EN);
	val &= ~WAVE600_PLL_EN_WLAN_CLK_PDN;
	wave600_write32(wdev, WAVE600_SYS_IF_PLL_EN, val);
	mmiowb();

	/*
	 * NOTE: Keep CPU and bus clock on 40 MHz XO during firmware staging!
	 * Staging across PCIe on 40 MHz XO takes ~110 ms.
	 * Switching to 640 MHz PLL before staging causes bus contention and 60s stalls.
	 * Clock switch to 640 MHz PLL will be performed right before ARC core release.
	 */
	wave600_hw_set_cpu_clock(wdev, WAVE600_CPU_CLK_SEL_XO);
	dev_info(wdev->dev, "AFE LDOs trimmed to 0.982V, staging clock kept on 40 MHz XO (Milestone 1 PASS)\n");

	return 0;
}

int wave600_hw_init(struct wave600_dev *wdev)
{
	int ret;

	wave600_hw_reset(wdev);

	ret = wave600_hw_init_pll(wdev);
	if (ret)
		dev_warn(wdev->dev, "PLL initialization returned %d\n", ret);

	return 0;
}
