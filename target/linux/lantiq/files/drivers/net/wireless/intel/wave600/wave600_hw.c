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

/*
 * Secure Read-Modify-Write helper for SYS_IF_UM registers.
 * Matches vendor _CCR_RMW_REG_FIELD_ pattern (mtlk_ccr_pcieg6.c):
 * Read -> Modify in local variable -> Secure Unlock -> Immediate Write.
 * An intervening read between unlock and write cancels the secure latch.
 */
static void wave600_secure_rmw_upper(struct wave600_dev *wdev, u32 reg, u32 mask, u32 set)
{
	u32 val = wave600_read32(wdev, reg);

	val = (val & ~mask) | set;
	wave600_secure_unlock_upper(wdev);
	wave600_write32(wdev, reg, val);
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

	dev_info(wdev->dev, "releasing ARC CPUs from reset on 640 MHz Fast LCPLL (Gen 6 release sequence)\n");

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

static void wave600_hw_efuse_timer_config(struct wave600_dev *wdev, u32 clk_mhz)
{
	u32 field1, field2;
	u32 timer1, timer2;

	/*
	 * SOC_REGS_EFUSE_TIMER_CONFIG1:
	 * field1 (bits 0..15): TPGM = 12 us * clk_mhz
	 * field2 (bits 16..31): TSU_CS_STROBE = max(1, 8 ns * clk_mhz / 1000)
	 */
	field1 = clk_mhz * 12;
	field2 = max_t(u32, 1, (clk_mhz * 8) / 1000);
	timer1 = (field2 << 16) | (field1 & 0xffff);

	/*
	 * SOC_REGS_EFUSE_TIMER_CONFIG2:
	 * field1 (bits 0..15): TRD = max(1, 200 ns * clk_mhz / 1000) + 4
	 * field2 (bits 16..31): TSU_PS_CSB = max(1, 71 ns * clk_mhz / 1000)
	 */
	field1 = max_t(u32, 1, (clk_mhz * 200) / 1000) + 4;
	field2 = max_t(u32, 1, (clk_mhz * 71) / 1000);
	timer2 = (field2 << 16) | (field1 & 0xffff);

	wave600_write32(wdev, WAVE600_SOC_EFUSE_TIMER_CONFIG1, timer1);
	wave600_write32(wdev, WAVE600_SOC_EFUSE_TIMER_CONFIG2, timer2);
	mmiowb();

	dev_info(wdev->dev, "EFUSE timers configured: timer1=0x%08x, timer2=0x%08x (clock %u MHz)\n",
		 timer1, timer2, clk_mhz);
}

int wave600_hw_read_efuse(struct wave600_dev *wdev)
{
	u32 words[12];
	int i, timeout;
	u32 status;

	/* Zero-fill: a partial read must not leak stack garbage into the
	 * AFE block consumed by UM_MAN_SET_AFE_CAL_DATA_REQ. */
	memset(words, 0, sizeof(words));

	/* 1. Ensure EFUSE timers are set for 40 MHz XO */
	wave600_hw_efuse_timer_config(wdev, 40);

	/* 2. Disable PD and override FSM */
	wave600_write32(wdev, WAVE600_SOC_EFUSE_PD, 0);
	wave600_write32(wdev, WAVE600_SOC_EFUSE_PCIE_FSM_OVR, 1);
	mmiowb();

	/* 3. Wait for busy cleared and init_done set */
	timeout = 2000;
	while (timeout--) {
		status = wave600_read32(wdev, WAVE600_SOC_EFUSE_STATUS);
		if (!(status & WAVE600_EFUSE_STATUS_BUSY) && (status & WAVE600_EFUSE_STATUS_INIT_DONE))
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		dev_warn(wdev->dev, "eFuse controller not ready (status 0x%08x)\n", status);
		wave600_write32(wdev, WAVE600_SOC_EFUSE_PCIE_FSM_OVR, 0);
		wave600_write32(wdev, WAVE600_SOC_EFUSE_PD, 1);
		mmiowb();
		return -ETIMEDOUT;
	}

	/* 4. Read 12 words (0..11) */
	for (i = 0; i < 12; i++) {
		wave600_write32(wdev, WAVE600_SOC_EFUSE_ADDRESS, i << 5);
		wave600_write32(wdev, WAVE600_SOC_EFUSE_CONTROL, 0);
		mmiowb();

		timeout = 1000;
		while (timeout--) {
			status = wave600_read32(wdev, WAVE600_SOC_EFUSE_STATUS);
			if (!(status & WAVE600_EFUSE_STATUS_BUSY))
				break;
			udelay(10);
		}
		if (timeout <= 0) {
			dev_warn(wdev->dev, "eFuse read timeout at word %d\n", i);
			break;
		}

		/* Byte swap word as vendor does: words[i] = __swab32(word) */
		words[i] = swab32(wave600_read32(wdev, WAVE600_SOC_EFUSE_DATA));
	}

	/* 5. Close eFuse */
	wave600_write32(wdev, WAVE600_SOC_EFUSE_PCIE_FSM_OVR, 0);
	wave600_write32(wdev, WAVE600_SOC_EFUSE_PD, 1);
	mmiowb();

	/* Parse AFE data from word 7 (offset 28) */
	{
		u32 w7 = words[7];
		wdev->afe.ibiastrim           = (w7 >> 28) & 0x0f;
		wdev->afe.vreftrim            = (w7 >> 24) & 0x0f;
		wdev->afe.trim_indicator      = (w7 >> 23) & 0x01;
		wdev->afe.ldo_central_pll1v8  = (w7 >> 20) & 0x07;
		wdev->afe.ldo_central_pllref  = (w7 >> 16) & 0x0f;
		wdev->afe.ldo_central_pllpost = (w7 >> 12) & 0x0f;
		wdev->afe.iptattrim           = (w7 >> 8)  & 0x0f;
		wdev->afe.ldo_central_clkd    = (w7 >> 4)  & 0x0f;
		wdev->afe.ldo_central_clkin   = (w7 >> 0)  & 0x0f;
		wdev->afe.valid               = true;

		memcpy(wdev->afe.raw, &words[7], sizeof(wdev->afe.raw));

		dev_info(wdev->dev, "eFuse AFE: trim_ind=%d, clkin=0x%x, clkd=0x%x, pllref=0x%x, pllpost=0x%x, pll1v8=0x%x, bias=0x%x, vref=0x%x\n",
			 wdev->afe.trim_indicator, wdev->afe.ldo_central_clkin,
			 wdev->afe.ldo_central_clkd, wdev->afe.ldo_central_pllref,
			 wdev->afe.ldo_central_pllpost, wdev->afe.ldo_central_pll1v8,
			 wdev->afe.ibiastrim, wdev->afe.vreftrim);
	}

	return 0;
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
	 * Matches vendor _abb_central_fusing with eFuse AFE parameters
	 */
	if (wdev->afe.valid && wdev->afe.trim_indicator) {
		u16 bias_val = (wdev->afe.ibiastrim & 0x0f) |
			       ((wdev->afe.iptattrim & 0x0f) << 4) |
			       ((wdev->afe.vreftrim & 0x0f) << 8);

		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_BIAS, bias_val);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_VREG0V95, wdev->afe.ldo_central_clkin & 0x0f);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_LDOCLKD, wdev->afe.ldo_central_clkd & 0x0f);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_PLLREF, wdev->afe.ldo_central_pllref & 0x0f);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_PLLPOST, wdev->afe.ldo_central_pllpost & 0x0f);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_VREG1V8, wdev->afe.ldo_central_pll1v8 & 0x07);
		dev_info(wdev->dev, "AFE LDOs trimmed to factory eFuse calibrated voltages (clkin=0x%x, clkd=0x%x)\n",
			 wdev->afe.ldo_central_clkin, wdev->afe.ldo_central_clkd);
	} else {
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_BIAS, WAVE600_FCSI_VAL_BIAS);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_VREG0V95, WAVE600_FCSI_VAL_VREG0V95);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_LDOCLKD, WAVE600_FCSI_VAL_LDOCLKD);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_PLLREF, WAVE600_FCSI_VAL_PLLREF);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_PLLPOST, WAVE600_FCSI_VAL_PLLPOST);
		wave600_fcsi_write(wdev, WAVE600_FCSI_CENTRAL_VREG1V8, WAVE600_FCSI_VAL_VREG1V8);
		dev_info(wdev->dev, "AFE LDOs trimmed to default voltages (0.982V)\n");
	}

	/*
	 * 3. Initialize Central LCPLL and power up
	 * Matches vendor _abb_central_init_pll
	 */
	/* 3a. Put LCPLL into reset & power-down */
	wave600_secure_rmw_upper(wdev, WAVE600_SYS_IF_PLL_CTL,
				 WAVE600_PLL_CTL_PUP | WAVE600_PLL_CTL_RESETN, 0);

	/* 3b. Power-down WLAN clock distribution during PLL acquisition (pd_clk_sc_d2a = 1) */
	wave600_secure_rmw_upper(wdev, WAVE600_SYS_IF_PLL_EN,
				 WAVE600_PLL_EN_WLAN_CLK_PDN, WAVE600_PLL_EN_WLAN_CLK_PDN);

	/* 3c. Configure WLAN clock divider for 640 MHz (RMW bits 2..3, preserve SSI/test) */
	wave600_secure_rmw_upper(wdev, WAVE600_SYS_IF_PLL_DIV,
				 WAVE600_PLL_DIV_WLAN_CLK_DIV_MASK, WAVE600_PLL_DIV_WLAN_CLK_DIV_640M);

	/* 3d. Clear spare config (spare_config_i_d2a = 0) */
	val = wave600_read32(wdev, WAVE600_MAC_GEN_SPARE_CONFIG_D2A);
	val &= ~0xff;
	wave600_write32(wdev, WAVE600_MAC_GEN_SPARE_CONFIG_D2A, val);
	mmiowb();
	udelay(10);

	/* 3e. Step 1 of analog power-up: lcpll_pup_d2a = 1 */
	wave600_secure_rmw_upper(wdev, WAVE600_SYS_IF_PLL_CTL,
				 WAVE600_PLL_CTL_RESETN, WAVE600_PLL_CTL_PUP);
	udelay(50);

	/* 3f. Step 2 of analog power-up: lcpll_resn_d2a = 1 (maintaining PUP = 1) */
	wave600_secure_rmw_upper(wdev, WAVE600_SYS_IF_PLL_CTL,
				 0, WAVE600_PLL_CTL_RESETN);

	/*
	 * 4. Wait for PLL lock (SYS_IF_PLL_LOCK bit 0)
	 * Matches vendor _abb_central_lock_pll: msleep(1) up to 100 ms
	 */
	{
		int i;
		bool locked = false;

		for (i = 0; i < 100; i++) {
			msleep(1);
			val = wave600_read32(wdev, WAVE600_SYS_IF_PLL_LOCK);
			if (val & WAVE600_PLL_LOCK_LOCKED) {
				locked = true;
				break;
			}
		}

		if (!locked) {
			dev_err(wdev->dev, "LCPLL lock timeout after 100 ms (status 0x%08x)\n", val);
			return -ETIMEDOUT;
		}

		dev_info(wdev->dev, "LCPLL locked successfully after %d ms (status 0x%08x)\n", i, val);
	}

	/*
	 * 5. Power up WLAN clock distribution
	 * Matches vendor _abb_central_init_clkd (pd_clk_sc_d2a = 0)
	 */
	wave600_secure_rmw_upper(wdev, WAVE600_SYS_IF_PLL_EN,
				 WAVE600_PLL_EN_WLAN_CLK_PDN, 0);

	/*
	 * 6. Switch to 640 MHz Fast LCPLL and configure EFUSE timers
	 * Matches vendor _mtlk_pcieg6_init_pll sequence:
	 * - _abb_cpu_clock_select(SYS_IF_UM_CLOCK_SELECT_CPU_CLOCK_SELECT_PLL)
	 * - _mtlk_pcieg6_efuse_timer_config()
	 */
	wave600_hw_set_cpu_clock(wdev, WAVE600_CPU_CLK_SEL_PLL);
	wave600_hw_efuse_timer_config(wdev, 640);
	dev_info(wdev->dev, "AFE LDOs trimmed, CPU/bus clock switched to 640 MHz Fast LCPLL (Milestone 1 PASS)\n");

	return 0;
}

int wave600_hw_init(struct wave600_dev *wdev)
{
	int ret;

	wave600_hw_reset(wdev);

	ret = wave600_hw_read_efuse(wdev);
	if (ret)
		dev_warn(wdev->dev, "eFuse read returned %d, continuing with default AFE trims\n", ret);

	ret = wave600_hw_init_pll(wdev);
	if (ret) {
		dev_err(wdev->dev, "PLL initialization failed: %d\n", ret);
		return ret;
	}

	/* Enable MAC GenRISC QManager CPU clocks (matching vendor _mtlk_mmb_power_on) */
	wave600_hw_power_on_cpus(wdev);

	return 0;
}

/*
 * UM_MAN_HW_DEPENDENT_CONFIG_REQ (vendor hw_mmb.c _mtlk_mmb_send_hw_dep_cfg):
 * tells the firmware the board's crystal configuration from the calibration
 * EEPROM before any radio activity. On gen6 the vendor CCR AFE getter is a
 * dummy returning zeros, so only the xtal value is meaningful.
 */
int wave600_hw_send_dep_cfg(struct wave600_dev *wdev)
{
	struct wave600_umi_hw_dep_cfg p;
	int ret;

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC32) */
	p.xtal = cpu_to_le32(wdev->cal.xtal);

	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_HW_DEPENDENT_CONFIG_REQ,
			      WAVE600_MC_MAN_HW_DEPENDENT_CONFIG_CFM,
			      &p, sizeof(p), NULL, 0, 5000);
	if (ret)
		return ret;

	dev_info(wdev->dev, "HW dependent config confirmed (xtal=0x%04x)\n",
		 wdev->cal.xtal);
	return 0;
}
