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
#include <linux/netdevice.h>
#include <net/cfg80211.h>
#include "wave600_ipc.h"
#include "wave600_dma.h"
#include "wave600_chi.h"
#include "wave600_cal.h"

#define DRV_NAME "wave600_wifi"

enum wave600_state {
	W600_STATE_IDLE = 0,
	W600_STATE_RUNNING,
	W600_STATE_FAULT,
	W600_STATE_RECOVERY,
};

#define WAVE600_MAX_STATIONS 32

struct wave600_station {
	bool valid;
	u8 mac[ETH_ALEN];
	u16 sid;
	u32 connect_time;
	unsigned long last_rx;
	s8 signal;
	s8 signal_avg;
	bool is_authorized;

	/* Protocol and capability flags */
	bool is_he;
	bool is_vht;
	bool is_ht;
	u8 nss;
	u8 bw;

	/* Per-station counters */
	u64 rx_bytes;
	u64 tx_bytes;
	u32 rx_packets;
	u32 tx_packets;

	/* Dynamic rate tracking */
	u32 tx_bitrate;		/* in 100 kbps units (e.g. 12010 = 1201.0 Mbps) */
	u32 rx_bitrate;		/* in 100 kbps units */
	u8 tx_mcs;
	u8 rx_mcs;
	u8 tx_nss;
	u8 rx_nss;
	u8 tx_bw;
	u8 rx_bw;
	u8 tx_flags;
	u8 rx_flags;
	u8 tx_gi;
	u8 rx_gi;
	u8 rx_dcm;
};

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
	int la_start;			/* LINK_ADAPTATION_STATS block (TX power) */

	/* Dynamic station telemetry */
	void *phy_db;			/* coherent wholePhyRxStatusDb_t buffer */
	dma_addr_t phy_db_dma;
	u32 phy_db_size;
	int la_bw_off;
	int la_mode_off;
	int la_mcs_off;
	int la_nss_off;
	int la_scp_off;
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

struct wave600_radio {
	struct wave600_dev *wdev;
	u8 radio_idx;		/* 0 = 2.4 GHz, 1 = 5 GHz */
	u8 ep;			/* 1 for Radio 0, 0 for Radio 1 */
	u8 vap_id;		/* Local VAP index on this radio (0 for primary AP) */
	u8 antenna_mask;	/* 0x05 for Radio 0, 0x0A for Radio 1 */

	struct wiphy *wiphy;
	struct net_device *ndev;
	struct wireless_dev *wdev_net;
	bool ap_started;
	bool radio_enabled;
	u16 ap_channel;
	u8 ap_width;
	u32 ap_freq;
	int tx_power_dbm;
	u64 mgmt_tx_cookie;

	/* Station tracking */
	bool sta_connected;
	bool is_secure;
	u8 sta_mac[ETH_ALEN];
	u16 sta_sid;
	u32 sta_connect_time;
	unsigned long sta_last_rx;
	s8 sta_signal;
	struct wave600_station stations[WAVE600_MAX_STATIONS];
	u16 num_stations;

	/* Persistent AP beacon template buffer (DMA-mapped for ARC MAC beaconing) */
	void *beacon_buf;
	dma_addr_t beacon_dma;
	size_t beacon_len;

	/* AP parameters preserved across recovery */
	u8 ap_low_chan;
	u8 ap_primary_idx;
	u16 beacon_interval;
	u16 dtim_period;
	struct wave600_umi_beacon_set beacon_set_params;
};

struct wave600_dev {
	struct pci_dev *pdev;
	struct device *dev;
	void __iomem *bar0;
	resource_size_t bar0_len;

	/* Device state and recovery */
	enum wave600_state state;
	struct work_struct recovery_work;
	atomic_t in_recovery;

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

	/* DBDC Radios: [0] = 2.4 GHz (wlan0), [1] = 5 GHz (wlan1) */
	struct wave600_radio radios[2];

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

	/* Regulatory domain configured from userspace via cfg80211 */
	char country[3];
	u8 regd_code;
	u8 reg_domain_region;

	/* Delayed second FW state snapshot (diagnostic, 10 s after probe) */
	struct delayed_work diag_work;

	/* Dynamic station telemetry work (1 s period while stations connected) */
	struct delayed_work telemetry_work;
};

int wave600_set_country(struct wave600_dev *wdev, const char *alpha2);
int wave600_send_reg_domain_config(struct wave600_dev *wdev, u8 region);

static inline struct wave600_radio *wave600_wiphy_to_radio(struct wiphy *wiphy)
{
	return *(struct wave600_radio **)wiphy_priv(wiphy);
}

static inline struct wave600_radio *wave600_ndev_to_radio(struct net_device *ndev)
{
	if (!ndev || !ndev->ieee80211_ptr || !ndev->ieee80211_ptr->wiphy)
		return NULL;
	return wave600_wiphy_to_radio(ndev->ieee80211_ptr->wiphy);
}

int wave600_radio_find_sid(struct wave600_radio *radio, const u8 *mac);
int wave600_cal_init(struct wave600_dev *wdev);
void wave600_cal_cleanup(struct wave600_dev *wdev);
int wave600_cfg80211_init(struct wave600_dev *wdev);
void wave600_cfg80211_cleanup(struct wave600_dev *wdev);
int wave600_cfg80211_restore_ap(struct wave600_radio *radio);
void wave600_diag_handlers_probe(struct wave600_dev *wdev);
int wave600_stats_init(struct wave600_dev *wdev);
void wave600_stats_cleanup(struct wave600_dev *wdev);
void wave600_stats_update_telemetry(struct wave600_dev *wdev);
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
struct wave600_station *wave600_find_sta(struct wave600_radio *radio, const u8 *mac);

#endif /* __WAVE600_H__ */
