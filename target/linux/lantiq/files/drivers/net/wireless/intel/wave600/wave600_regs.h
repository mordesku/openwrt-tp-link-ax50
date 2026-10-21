/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Hardware Register Map
 *
 * Derived from vendor Wave600 Emerald architecture hardware specifications:
 * - HwMemoryMap.h
 * - EmeraldEnvRegs.h
 * - SocRegsRegs.h
 * - MacGeneralRegs.h
 */

#ifndef __WAVE600_REGS_H__
#define __WAVE600_REGS_H__

#include <linux/types.h>

#define WAVE600_BAR0_IO_MASK            0x00ffffff /* 16 MiB accessible MMIO window */

/*
 * Base offsets from PCIe BAR0
 */
#define WAVE600_SYS_IF_LM0_BASE         0x00200000 /* Lower MAC 0 Environment */
#define WAVE600_SYS_IF_LM1_BASE         0x00210000 /* Lower MAC 1 Environment */
#define WAVE600_SYS_IF_UM_BASE          0x00220000 /* Upper MAC Environment */
#define WAVE600_MAC_GEN_BASE            0x00238000 /* MAC General / Doorbell */
#define WAVE600_SOC_REGS_BASE           0x00270000 /* Top SOC Control */

/*
 * IRAM Offsets from BAR0 (for Firmware Staging)
 */
#define WAVE600_CPU_IRAM_START          0x00000000 /* 1.5 MiB URAM (Upper MAC) */
#define WAVE600_CPU_IRAM_SIZE           0x00180000
#define WAVE600_IFCPU_IRAM_START        0x00230000 /* Host IF GenRISC IRAM (12 KiB) */
#define WAVE600_IFCPU_IRAM_SIZE         0x00003000
#define WAVE600_B0_TXCPU_IRAM_START     0x00800000 /* Band 0 TX GenRISC IRAM (64 KiB) */
#define WAVE600_B0_TXCPU_IRAM_SIZE      0x00010000
#define WAVE600_B0_RXCPU_IRAM_START     0x00820000 /* Band 0 RX GenRISC IRAM (16 KiB) */
#define WAVE600_B0_RXCPU_IRAM_SIZE      0x00004000
#define WAVE600_B1_TXCPU_IRAM_START     0x00C00000 /* Band 1 TX GenRISC IRAM (64 KiB) */
#define WAVE600_B1_TXCPU_IRAM_SIZE      0x00010000
#define WAVE600_B1_RXCPU_IRAM_START     0x00C20000 /* Band 1 RX GenRISC IRAM (16 KiB) */
#define WAVE600_B1_RXCPU_IRAM_SIZE      0x00004000

/*
 * Shared RAM (SHRAM) - 256 KiB Host <-> ARC Handshake & Ring Descriptors
 */
#define WAVE600_SHARED_RAM_BASE         0x00300000 /* Shared RAM offset from BAR0 */
#define WAVE600_SHARED_RAM_SIZE         0x00040000 /* 256 KiB */

/*
 * MAC Extensions / GenRISC Clock Controls
 */
#define WAVE600_MAC_HT_EXT_CLK_CTRL     0x008310A4 /* QManager Clock Control */

/*
 * PHY / Baseband FCSI AFE Registers
 */
#define WAVE600_PHY_RX_TD_IF_BASE       0x00B40000
#define WAVE600_PHY_RXTD_IF126          (WAVE600_PHY_RX_TD_IF_BASE + 0x1498)
#define WAVE600_PHY_RXTD_IF127          (WAVE600_PHY_RX_TD_IF_BASE + 0x149C)
#define WAVE600_PHY_RXTD_IF_FCSI_ACCESS (WAVE600_PHY_RX_TD_IF_BASE + 0x14A4)
#define WAVE600_PHY_RXTD_IF12C          (WAVE600_PHY_RX_TD_IF_BASE + 0x14B0)
#define WAVE600_PHY_RXTD_IF131          (WAVE600_PHY_RX_TD_IF_BASE + 0x14C4)
#define WAVE600_PHY_RXTD_IF136          (WAVE600_PHY_RX_TD_IF_BASE + 0x14D8)


/*
 * PAC / MAC RX Control
 */
#define WAVE600_PAC_RX_CONTROL          0x00904000 /* Band 0 PAC RX Control */
#define WAVE600_PAC_RX_CTRL_ENABLED     BIT(0)

/*
 * Top SOC Control Registers (SOC_REGS)
 */
#define WAVE600_SOC_SOFT_RESET          (WAVE600_SOC_REGS_BASE + 0x04)
#define WAVE600_SOC_SOFT_RESET_WLAN     BIT(0)  /* WLAN software reset strobe request (active low) */
#define WAVE600_SOC_PCIE_INTR_VEC       (WAVE600_SOC_REGS_BASE + 0x48)
#define WAVE600_SOC_PCIE_MSI_ENABLE     (WAVE600_SOC_REGS_BASE + 0x4C)
#define WAVE600_SOC_PCIE_LEGACY_INT_ENA (WAVE600_SOC_REGS_BASE + 0x50)
/*
 * AMBA order manager watchdog timer interrupt (SocRegsRegs.h 0x6C):
 * RO level status, asserted while the PCIe ordering rule workaround
 * ("CPL must not pass P" disabled by the order manager WDT) is active.
 * Bit 0: pcie0, bit 16: pcie1.
 */
#define WAVE600_SOC_PCIE_AMBA_ORDR_WDT  (WAVE600_SOC_REGS_BASE + 0x6C)

/*
 * GRX500 PCIe Root Complex app registers (lantiq,pcie-xrx500).
 * Used for INTx storm diagnostics / RC-side latch handling from this driver.
 * Physical app bases come from grx500.dtsi ssx@18000000 reg "app":
 *   RC port 0 (bus 01): 0x18800000, RC port 1 (bus 03): 0x18300000.
 * Register semantics per vendor ltq_pcie (patch 3030):
 *   IRNEN  (app+0xF4): interrupt node enable, INTA..INTD = bits 13..16
 *   IRNCR  (app+0xF8): interrupt node clear, write-1-to-clear latch
 *   IRNICR (app+0xFC): interrupt node live status
 */
#define WAVE600_RC_APP_PORT0_BASE     0x18800000
#define WAVE600_RC_APP_PORT1_BASE     0x18300000
#define WAVE600_RC_APP_IRNEN          0xF4
#define WAVE600_RC_APP_IRNCR          0xF8
#define WAVE600_RC_APP_IRNICR         0xFC
#define WAVE600_RC_INTA               BIT(13)
#define WAVE600_SOC_EFUSE_DATA          (WAVE600_SOC_REGS_BASE + 0x70)
#define WAVE600_SOC_EFUSE_ADDRESS       (WAVE600_SOC_REGS_BASE + 0x74)
#define WAVE600_SOC_EFUSE_CONTROL       (WAVE600_SOC_REGS_BASE + 0x78)
#define WAVE600_SOC_EFUSE_STATUS        (WAVE600_SOC_REGS_BASE + 0x7C)
#define WAVE600_SOC_EFUSE_TIMER_CONFIG1 (WAVE600_SOC_REGS_BASE + 0x80)

/*
 * Upper MAC Environment Registers (SYS_IF_UM)
 */
#define WAVE600_SYS_IF_PLL_DIV          (WAVE600_SYS_IF_UM_BASE + 0x40)
#define WAVE600_SYS_IF_PLL_EN           (WAVE600_SYS_IF_UM_BASE + 0x44)
#define WAVE600_SYS_IF_PLL_CTL          (WAVE600_SYS_IF_UM_BASE + 0x48)
#define WAVE600_SYS_IF_PLL_LOCK         (WAVE600_SYS_IF_UM_BASE + 0x50)
#define WAVE600_SYS_IF_UM_CPU_ENABLE    (WAVE600_SYS_IF_UM_BASE + 0x54)
#define WAVE600_SYS_IF_CLOCK_SELECT     (WAVE600_SYS_IF_UM_BASE + 0x58)
#define WAVE600_SYS_IF_SYSTEM_INFO      (WAVE600_SYS_IF_UM_BASE + 0x60)
#define WAVE600_SYS_IF_UM_SECURE_WRITE  (WAVE600_SYS_IF_UM_BASE + 0x68)
#define WAVE600_SYS_IF_UM_MIPS_CTL      (WAVE600_SYS_IF_UM_BASE + 0x74)
#define WAVE600_SYS_IF_BB_OPMODE_STATUS (WAVE600_SYS_IF_UM_BASE + 0x7C) /* Reset value: 0x1234 */

/*
 * PLL Control Bits (SYS_IF_PLL_CTL)
 */
#define WAVE600_PLL_CTL_PUP             BIT(7)  /* LCPLL Power Up */
#define WAVE600_PLL_CTL_RESETN          BIT(8)  /* LCPLL Reset_N (active low reset) */
#define WAVE600_PLL_CTL_BYPASS          BIT(9)  /* LCPLL LDO Bypass */

/*
 * PLL Clock Enable / Power Down Bits (SYS_IF_PLL_EN)
 */
#define WAVE600_PLL_EN_WLAN_CLK_PDN     BIT(1)  /* WLAN Clock Power Down */

/*
 * PLL Lock Status (SYS_IF_PLL_LOCK)
 */
#define WAVE600_PLL_LOCK_LOCKED         BIT(0)

/*
 * Lower MAC Environment Registers (SYS_IF_LM0, SYS_IF_LM1)
 */
#define WAVE600_SYS_IF_LM0_CPU_ENABLE   (WAVE600_SYS_IF_LM0_BASE + 0x54)
#define WAVE600_SYS_IF_LM0_SECURE_WRITE (WAVE600_SYS_IF_LM0_BASE + 0x68)
#define WAVE600_SYS_IF_LM0_MIPS_CTL     (WAVE600_SYS_IF_LM0_BASE + 0x74)
#define WAVE600_SYS_IF_LM1_CPU_ENABLE   (WAVE600_SYS_IF_LM1_BASE + 0x54)
#define WAVE600_SYS_IF_LM1_SECURE_WRITE (WAVE600_SYS_IF_LM1_BASE + 0x68)
#define WAVE600_SYS_IF_LM1_MIPS_CTL     (WAVE600_SYS_IF_LM1_BASE + 0x74)

/*
 * MAC General / Doorbell / Interrupt Registers (MAC_GEN_REGS)
 */
#define WAVE600_MAC_GEN_PHY_EXT_RESET   (WAVE600_MAC_GEN_BASE + 0x24)
#define WAVE600_MAC_GEN_HOST_IRQ_STATUS (WAVE600_MAC_GEN_BASE + 0x40)
#define WAVE600_MAC_GEN_HOST_IRQ_MASK   (WAVE600_MAC_GEN_BASE + 0x44)
#define WAVE600_MAC_GEN_FCSI_CTL        (WAVE600_MAC_GEN_BASE + 0x9C)
#define WAVE600_MAC_GEN_ENABLE_PHI_IRQ  (WAVE600_MAC_GEN_BASE + 0xB0)
#define WAVE600_MAC_GEN_PHI_IRQ         (WAVE600_MAC_GEN_BASE + 0xB4)
#define WAVE600_MAC_GEN_PHI_IRQ_CLEAR   (WAVE600_MAC_GEN_BASE + 0xBC)
#define WAVE600_MAC_GEN_NPU2UPI_SET     (WAVE600_MAC_GEN_BASE + 0xC8) /* Doorbell to FW */
#define WAVE600_MAC_GEN_NPU2UPI_CLEAR   (WAVE600_MAC_GEN_BASE + 0xCC)
#define WAVE600_MAC_GEN_PHY_BAND_CONFIG (WAVE600_MAC_GEN_BASE + 0x1FC)

/*
 * PHY & FCSI Reset / Active Flags
 */
#define WAVE600_PHY_EXT_RESET_B0        BIT(0)
#define WAVE600_PHY_EXT_RESET_B1        BIT(1)
#define WAVE600_PHY_BAND_CFG_B0_ACTIVE  BIT(0)
#define WAVE600_PHY_BAND_CFG_B1_ACTIVE  BIT(1)
#define WAVE600_PHY_BAND_CFG_FCSI_CEN_RESET_N BIT(2)
#define WAVE600_PHY_BAND_CFG_FCSI_CEN_BLOCK_EN BIT(3)
#define WAVE600_FCSI_CTL_RESET_QC_D2A   (BIT(0) | BIT(1))

/*
 * Secure Write Magic Sequence
 * Must be written to SECURE_WRITE prior to updating MIPS_CTL, CLOCK_SELECT, or CPU_ENABLE.
 */
#define WAVE600_SECURE_KEY1             0xAAAA
#define WAVE600_SECURE_KEY2             0x5555

/*
 * ARC CPU Reset State Values (MIPS_CTL 0x74) for Wave600 Gen 6 (WAV654)
 * Bit 0: RELEASE_FROM_RESET / RUN (0x01)
 * Bit 1: MIPS_RESET (0x02)
 */
#define WAVE600_MIPS_RESET              0x02 /* Gen 6 MIPS CPU in reset */
#define WAVE600_MIPS_RELEASE            0x01 /* Gen 6 MIPS CPU released from reset / running */


/*
 * Fast Control Serial Interface (FCSI) Definitions (Central / AFE LDO Trimming)
 */
#define WAVE600_FCSI_BUSY_WAIT_COUNT    1000
#define WAVE600_FCSI_BUSY_BIT           BIT(0)
#define WAVE600_FCSI_CENTRAL_MASK       (0x10 << 7) /* Central block mask */
#define WAVE600_FCSI_MS_ACCESS          BIT(13)     /* MS access mode */

/* Central AFE LDO Registers (ABB_CENTRAL) */
#define WAVE600_FCSI_CENTRAL_BIAS       0x40 /* ICENTBIAS0 */
#define WAVE600_FCSI_CENTRAL_VREG0V95   0x45 /* IVREG0V950 (0.95V digital core / IRAM) */
#define WAVE600_FCSI_CENTRAL_LDOCLKD    0x47 /* ILDOCLKD0 (Clock distribution LDO) */
#define WAVE600_FCSI_CENTRAL_PLLREF     0x5C /* ILDOPLLREF0 (PLL reference LDO) */
#define WAVE600_FCSI_CENTRAL_PLLPOST    0x5D /* ILDOPLLPOST0 (PLL post-divider LDO) */
#define WAVE600_FCSI_CENTRAL_VREG1V8    0x5E /* IVREG1V8HV0 (1.8V PLL high-voltage) */

/* Vendor default LDO trim values from _abb_central_fusing */
#define WAVE600_FCSI_VAL_BIAS           0x0080 /* iptattrim = 8 */
#define WAVE600_FCSI_VAL_VREG0V95       0x0008 /* 0.982V */
#define WAVE600_FCSI_VAL_LDOCLKD        0x0008 /* 0.982V */
#define WAVE600_FCSI_VAL_PLLREF         0x0005 /* 0.951V */
#define WAVE600_FCSI_VAL_PLLPOST        0x0005 /* 0.951V */
#define WAVE600_FCSI_VAL_VREG1V8        0x0004 /* 1.8V */

/*
 * CPU Clock Selection Values
 */
#define WAVE600_CPU_CLK_SEL_XO          0x01 /* 40 MHz Crystal Oscillator */
#define WAVE600_CPU_CLK_SEL_PLL         0x02 /* 640 MHz Fast LCPLL */

/*
 * Shared RAM (SHRAM) Handshake Magic
 */
#define WAVE600_CHI_MAGIC               0xBEADFEED

#endif /* __WAVE600_REGS_H__ */
