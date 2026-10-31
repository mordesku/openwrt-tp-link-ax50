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
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "wave600_ipc.h"
#include "wave600_dma.h"
#include "wave600_chi.h"
#include "wave600_cal.h"

#define DRV_NAME "wave600_wifi"

/*
 * FW statistics context. The vendor fetches the counter layout descriptor
 * table exactly once during card finalize (hw_mmb.c hw_mmb_card_fin step
 * HW_MHI_STATISTIC, right after HW_DEP_CFG) and keeps a host copy; at
 * runtime only the counter database (0x0460) is re-fetched. The driver
 * mirrors that split: the table is captured once at probe, the sysfs
 * dump re-reads only the counters.
 */
struct wave600_stats {
	bool table_valid;		/* TABLE fetched during init */
	u8 *table;			/* descriptor table copy (tab_size) */
	u32 tab_size;
	u32 db_size;
	void *db;			/* coherent counter database buffer */
	dma_addr_t db_dma;
	int gen_start;
	int baa_start;
	int hostif_start;
};

struct wave600_efuse_afe {
	u8 ibiastrim;
	u8 vreftrim;
	u8 trim_indicator;
	u8 ldo_central_pll1v8;
	u8 ldo_central_pllref;
	u8 ldo_central_pllpost;
	u8 iptattrim;
	u8 ldo_central_clkd;
	u8 ldo_central_clkin;
	u8 raw[20];
	bool valid;
};

struct wave600_dev {
	struct pci_dev *pdev;
	struct device *dev;
	void __iomem *bar0;
	resource_size_t bar0_len;

	u32 chip_id;
	u32 chip_rev;
	u8 radio_idx;
	bool aux;			/* parked dual-PCI AUX data endpoint */
	char fw_version[128];

	/*
	 * Snapshot of the SHRAM CHI vector area, taken once right after the
	 * boot magic check (vendor hw_mmb.c keeps the same host copy). The
	 * ProgModel download staging area reuses SHRAM offset 0x14 onwards,
	 * so a later re-read of the area returns staging data, not CHI.
	 */
	struct wave600_chi_area chi;

	struct wave600_ipc ipc;
	struct wave600_dma dma;

	/* AP bring-up (Phase 8) */
	struct wiphy *wiphy;
	struct net_device *ndev;
	struct wireless_dev *wdev_net;
	bool ap_started;
	u16 ap_channel;
	u8 antenna_mask;
	u64 mgmt_tx_cookie;
	bool sta_connected;
	u8 sta_mac[ETH_ALEN];
	u16 sta_sid;

	/* Persistent AP beacon template buffer (DMA-mapped for ARC MAC beaconing) */
	void *beacon_buf;
	dma_addr_t beacon_dma;
	size_t beacon_len;

	/* Calibration cache DMA memory */
	void *cal_buf;
	dma_addr_t cal_dma_addr;
	size_t cal_buf_size;

	/* Factory calibration (cal_wlan0.bin) and Platform Specific Data */
	struct wave600_cal cal;
	struct wave600_psdb psdb;
	struct wave600_efuse_afe afe;

	/* FW statistics (vendor HW_MHI_STATISTIC init step) */
	struct wave600_stats stats;

	/* Delayed second FW state snapshot (diagnostic, 10 s after probe) */
	struct delayed_work diag_work;
};

int wave600_cal_init(struct wave600_dev *wdev);
void wave600_cal_cleanup(struct wave600_dev *wdev);
int wave600_cfg80211_init(struct wave600_dev *wdev);
void wave600_cfg80211_cleanup(struct wave600_dev *wdev);
void wave600_diag_handlers_probe(struct wave600_dev *wdev);
int wave600_stats_init(struct wave600_dev *wdev);
void wave600_stats_cleanup(struct wave600_dev *wdev);
int wave600_stats_sysfs_create(struct wave600_dev *wdev);
void wave600_stats_sysfs_remove(struct wave600_dev *wdev);

/*
 * Register MMIO accessors - use raw accessors to match vendor driver
 * (mtlk_readl/mtlk_writel) on MIPS Big-Endian PCIe BAR0.
 * In the vendor driver (mmb_ops.h), mtlk_raw_writel(CPU_TO_LE32(x)) and
 * LE32_TO_CPU(mtlk_raw_readl()) also cancel out to plain 32-bit register
 * reads/writes (__raw_readl / __raw_writel).
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
 * PAS (BAR0 Shared RAM / IRAM) byte-stream copy helpers, matching vendor
 * _mtlk_mmb_memcpy_toio / _mtlk_mmb_memcpy_fromio (hw_mmb.c:1319-1425) and
 * mtlk_raw_writel / mtlk_raw_readl (mmb_ops.h:100-110).
 *
 * On Big-Endian MIPS with GRX500 PCIe outbound_swap=0, __raw_writel /
 * __raw_readl preserve 32-bit numeric values (bits[31:0]), which reverses
 * the byte sequence of a byte buffer every 4 bytes if copied via Linux
 * memcpy_toio / memcpy_fromio. Applying cpu_to_le32 / le32_to_cpu on each
 * 32-bit word ensures byte N in host RAM maps to byte N in ARC memory.
 */
static inline void wave600_pas_put(void __iomem *dst, const void *src, size_t count)
{
	const u8 *s = src;
	size_t aligned = count & ~3ULL;
	size_t i;

	for (i = 0; i < aligned; i += 4) {
		u32 w;

		memcpy(&w, s + i, 4);
		__raw_writel(cpu_to_le32(w), dst + i);
	}
	if (count & 3) {
		u32 trail = 0;

		memcpy(&trail, s + aligned, count & 3);
		__raw_writel(cpu_to_le32(trail), dst + aligned);
	}
}

static inline void wave600_pas_get(void *dst, const void __iomem *src, size_t count)
{
	u8 *d = dst;
	size_t aligned = count & ~3ULL;
	size_t i;

	for (i = 0; i < aligned; i += 4) {
		u32 w = le32_to_cpu(__raw_readl(src + i));

		memcpy(d + i, &w, 4);
	}
	if (count & 3) {
		u32 trail = le32_to_cpu(__raw_readl(src + aligned));

		memcpy(d + aligned, &trail, count & 3);
	}
}

/*
 * Low-level hardware control operations (wave600_hw.c)
 */
int wave600_hw_init(struct wave600_dev *wdev);
int wave600_hw_read_efuse(struct wave600_dev *wdev);
void wave600_hw_reset(struct wave600_dev *wdev);
void wave600_hw_put_cpus_to_reset(struct wave600_dev *wdev);
void wave600_hw_release_cpus(struct wave600_dev *wdev);
void wave600_hw_power_on_cpus(struct wave600_dev *wdev);
void wave600_secure_unlock_upper(struct wave600_dev *wdev);
void wave600_secure_unlock_lower(struct wave600_dev *wdev, int lm_idx);
int wave600_hw_init_pll(struct wave600_dev *wdev);
int wave600_hw_send_dep_cfg(struct wave600_dev *wdev);
int wave600_fcsi_write(struct wave600_dev *wdev, u8 addr, u16 data);
void wave600_hw_set_cpu_clock(struct wave600_dev *wdev, u32 clk_sel);

#endif /* __WAVE600_H__ */
