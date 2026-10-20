/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Driver Core Header
 */

#ifndef __WAVE600_H__
#define __WAVE600_H__

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/types.h>

#define DRV_NAME "wave600_wifi"

struct wave600_dev {
	struct pci_dev *pdev;
	struct device *dev;
	void __iomem *bar0;
	resource_size_t bar0_len;

	u32 chip_id;
	u32 chip_rev;
	u8 radio_idx;
};

/*
 * Register MMIO accessors - use raw accessors to match vendor driver
 * (mtlk_readl/mtlk_writel) on MIPS Big-Endian PCIe BAR0.
 */
static inline u32 wave600_read32(struct wave600_dev *wdev, u32 reg)
{
	return __raw_readl(wdev->bar0 + reg);
}

static inline void wave600_write32(struct wave600_dev *wdev, u32 reg, u32 val)
{
	__raw_writel(val, wdev->bar0 + reg);
}

/*
 * Low-level hardware control operations (wave600_hw.c)
 */
int wave600_hw_init(struct wave600_dev *wdev);
void wave600_hw_reset(struct wave600_dev *wdev);
void wave600_hw_put_cpus_to_reset(struct wave600_dev *wdev);
void wave600_hw_release_cpus(struct wave600_dev *wdev);
void wave600_hw_power_on_cpus(struct wave600_dev *wdev);
void wave600_secure_unlock_upper(struct wave600_dev *wdev);
void wave600_secure_unlock_lower(struct wave600_dev *wdev, int lm_idx);
int wave600_hw_init_pll(struct wave600_dev *wdev);
int wave600_fcsi_write(struct wave600_dev *wdev, u8 addr, u16 data);
void wave600_hw_set_cpu_clock(struct wave600_dev *wdev, u32 clk_sel);

#endif /* __WAVE600_H__ */
