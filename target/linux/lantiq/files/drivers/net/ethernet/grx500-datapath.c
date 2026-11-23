// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 CBM/TMU CPU packet path for the AX50 five-port bring-up. */
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/grx500-dcdp.h>
#include <linux/grx500-gswip.h>
#include <linux/grx500-memory.h>
#include <linux/grx500-pae.h>
#include <linux/grx500-umt.h>
#include <linux/hashtable.h>
#include <linux/if_vlan.h>
#include <linux/in.h>
#include <linux/in6.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_reserved_mem.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/tcp.h>
#include <linux/timer.h>
#include <linux/u64_stats_sync.h>
#include <linux/udp.h>
#include <net/dsa.h>
#include <net/flow_offload.h>
#include <net/pkt_cls.h>
#include <lantiq_dmax.h>

/* Offsets are from the vendor CBM/TMU generated register headers. */
#define GRX500_TMU_CTRL		0x0
#define GRX500_TMU_FPL		0x4
#define GRX500_TMU_FPCR		0x8
#define GRX500_TMU_FPTHR		0xc
#define GRX500_TMU_TIMER		0x10
#define GRX500_TMU_LFSR		0x14
#define GRX500_TMU_CPR		0x18
#define GRX500_TMU_CSR		0x1c
#define GRX500_TMU_GOCCR		0x20
#define GRX500_TMU_GOTHR		0x30
#define GRX500_TMU_GPDCR		0x40
#define GRX500_TMU_QFILL		0x60
#define GRX500_TMU_EPFR		0x80
#define GRX500_TMU_LPIC		0xa0
#define GRX500_TMU_LPIT		0xb0
#define GRX500_TMU_ERDR		0xc0
#define GRX500_TMU_IRNCR		0x100
#define GRX500_TMU_IRNICR	0x104
#define GRX500_TMU_IRNEN		0x108
#define GRX500_TMU_TBIDCR	0x110
#define GRX500_TMU_TACPER	0x560
#define GRX500_TMU_QEMT		0x200
#define GRX500_TMU_QSMT		0x210
#define GRX500_TMU_QTHT		0x220
#define GRX500_TMU_QOCT		0x240
#define GRX500_TMU_QDCT		0x250
#define GRX500_TMU_QFMT		0x260
#define GRX500_TMU_QMTC		0x270
#define GRX500_TMU_PPT		0x600
#define GRX500_TMU_PPTC		0x610
#define GRX500_TMU_PPTC_WRITE	0x00010000
#define GRX500_TMU_PPTC_VALID	0x00020000
#define GRX500_TMU_PPT_ENTRIES	6144
#define GRX500_TMU_QMTC_QTW	0x00000400
#define GRX500_TMU_QMTC_QOW	0x00000800
#define GRX500_TMU_QMTC_QDW	0x00002000
#define GRX500_TMU_QMTC_QFW	0x00004000
#define GRX500_TMU_QMTC_QEW	0x00000100
#define GRX500_TMU_QMTC_QSW	0x00000200
#define GRX500_TMU_QMTC_QEV	0x01000000
#define GRX500_TMU_QMTC_QSV	0x02000000
#define GRX500_TMU_QMTC_QTV	0x04000000
#define GRX500_TMU_QMTC_QOV	0x08000000
#define GRX500_TMU_QMTC_QDV	0x20000000
#define GRX500_TMU_QMTC_QFV	0x40000000
#define GRX500_TMU_EPMT		0x300
#define GRX500_TMU_EPOT		0x310
#define GRX500_TMU_EPTT		0x320
#define GRX500_TMU_EPDT		0x330
#define GRX500_TMU_EPMTC		0x340
#define GRX500_TMU_EPMTC_EMW	0x00000100
#define GRX500_TMU_EPMTC_EOW	0x00000200
#define GRX500_TMU_EPMTC_ETW	0x00000400
#define GRX500_TMU_EPMTC_EDW	0x00000800
#define GRX500_TMU_EPMTC_EMV	0x01000000
#define GRX500_TMU_EPMTC_EOV	0x02000000
#define GRX500_TMU_EPMTC_ETV	0x04000000
#define GRX500_TMU_EPMTC_EDV	0x08000000
#define GRX500_TMU_SBITR		0x400
#define GRX500_TMU_SBITC		0x410
#define GRX500_TMU_SBOTR		0x420
#define GRX500_TMU_SBOTC		0x430
#define GRX500_TMU_SBC_WRITE	0x00030000
#define GRX500_TMU_SBC_VALID	0x00040000
#define GRX500_TMU_TBSTC		0x550
#define GRX500_TMU_TBSTC_WRITE	0x00030000
#define GRX500_TMU_TBSTC_VALID	0x00040000
#define GRX500_TMU_TABLE_TIMEOUT_US	1000
#define GRX500_QIDT_WORDS	4096
#define GRX500_CBM_CTRL		0x210
#define GRX500_EQM_CTRL		0x0
#define GRX500_EQM_CPU_IGP0	0x10000
#define GRX500_EQM_CPU_NEW_SPTR	0x10080
#define GRX500_EQM_CPU_POCC	0x10008
#define GRX500_EQM_CPU_EQPC	0x1000c
#define GRX500_EQM_CPU_IRNEN	0x10028
#define GRX500_DQM_CTRL		0x0
#define GRX500_DQM_CPU_CFG	0x10000
#define GRX500_DQM_CPU_EGP2	0x12000
#define GRX500_DQM_CPU_PTR_RTN	0x10080
#define GRX500_DQM_CPU_DESC	0x10100
#define GRX500_EQM_CPU_DESC	0x10100
#define GRX500_EQM_DMA_CFG	0x15000
#define GRX500_DQM_DMA_CFG	0x16000
#define GRX500_CBM_IRQ_STRIDE	0x40
#define GRX500_LS_PORT_STRIDE	0x100
#define GRX500_LS_PORT_CTRL	0x10
#define GRX500_LS_PORT_STATUS	0x14
#define GRX500_LS_IRNCR		0x910
#define GRX500_LS_GLOBAL_CTRL	0x900
#define GRX500_LS_SPREAD_CTRL	0x904
#define GRX500_LS_IRNEN		0x918
#define GRX500_CBM_CPU_PORT_STRIDE	0x1000
#define GRX500_CBM_CPU_PORTS	4
/* The AX50 DQM2 hardware test fills LS2, not LS0. This hardware port
 * number is independent of the Linux CPU which services the interrupt.
 */
#define GRX500_RX_LS_PORT	2
#define GRX500_RX_CBM_LINE	(4 + GRX500_RX_LS_PORT)
#define GRX500_RX_IRQ_INDEX	(1 + GRX500_RX_LS_PORT)
#define GRX500_RX_CBM_IRQ	BIT(8 + GRX500_RX_LS_PORT)
#define GRX500_RX_LS_IRQ	(3U << (16 + 2 * GRX500_RX_LS_PORT))
#define GRX500_CBM_ALLOC_RETRIES	20
#define GRX500_CBM_SBA0		0x200
#define GRX500_CBM_SBA1		0x204
#define GRX500_CBM_JBA0		0x208
#define GRX500_CBM_JBA1		0x20c

/* FSQM register offsets from vendor cbm/reg/fsqm.h. */
#define GRX500_FSQM_CTRL	0x00000
#define GRX500_FSQM_IO_BUF_RD	0x00008
#define GRX500_FSQM_IO_BUF_WR	0x0000c
#define GRX500_FSQM_IRNEN	0x00018
#define GRX500_FSQM_OFSQ	0x00024
#define GRX500_FSQM_OFSC	0x0002c
#define GRX500_FSQM_FSQT0	0x00030
#define GRX500_FSQM_FSQT1	0x00070
#define GRX500_FSQM_FSQT2	0x000b0
#define GRX500_FSQM_FSQT3	0x000f0
#define GRX500_FSQM_FSQT4	0x00130
#define GRX500_FSQM_LSARNG	0x00180
#define GRX500_FSQM_RCNT	0x80000
#define GRX500_FSQM_RAM		0xc0000
#define GRX500_LAN_EQM_ROUTES	4
#define GRX500_LAN_DQM_ROUTES	12
#define GRX500_CBM_IRQS		5
#define GRX500_CBM_DMA_ROUTES	(GRX500_LAN_DQM_ROUTES + GRX500_LAN_EQM_ROUTES)
#define GRX500_LAN_P2P_CLASSES	4
#define GRX500_GSWIP_R_PORTS	16
#define GRX500_PORTS		5
#define GRX500_WAN_ENDPOINT	15
/* Vendor dma_tx_desc_1 (big-endian): EP occupies DW1 bits 11:8. */
#define GRX500_TX_DW1_EP_SHIFT	8
#define GRX500_CBM_FRAME_OFFSET	(CBM_DMA_DATA_OFFSET + NET_IP_ALIGN + NET_SKB_PAD)
#define CBM_DMA_DATA_OFFSET	128
#define GRX500_PMAC_SIZE	8
#define GRX500_RX_BUFFER_OFFSET	GRX500_CBM_FRAME_OFFSET

struct grx500_eqm_route {
	u32 port;
	u32 dma_controller;
	u32 dma_channel;
	u32 jumbo;
	u32 descriptors;
};

struct grx500_dqm_route {
	u32 port;
	u32 tmu_queue;
	u32 dma_controller;
	u32 dma_channel;
	u32 descriptors;
};

enum grx500_datapath_resource {
	GRX500_TMU,
	GRX500_CBM,
	GRX500_QIDT,
	GRX500_SBIM,
	GRX500_QEQCNT,
	GRX500_QDQCNT,
	GRX500_LS,
	GRX500_EQM,
	GRX500_DQM,
	GRX500_FSQM0,
	GRX500_FSQM1,
	GRX500_DMA_DESC,
	GRX500_RESOURCE_COUNT,
};

struct grx500_resource_layout {
	resource_size_t start;
	resource_size_t size;
};

/* Exact SSX0 windows from vendor xrx500_axepoint.dtsi. Never infer a
 * register position from an unrelated GRX500 reference board.
 */
static const struct grx500_resource_layout grx500_layout[] = {
	[GRX500_TMU] = { 0x1e700000, 0x1000 },
	[GRX500_CBM] = { 0x1e710000, 0x10000 },
	[GRX500_QIDT] = { 0x1e720000, 0x4000 },
	[GRX500_SBIM] = { 0x1e730000, 0x14000 },
	[GRX500_QEQCNT] = { 0x1e750000, 0x400 },
	[GRX500_QDQCNT] = { 0x1e750800, 0x400 },
	[GRX500_LS] = { 0x1e760000, 0x10000 },
	[GRX500_EQM] = { 0x1e780000, 0x20000 },
	[GRX500_DQM] = { 0x1e7c0000, 0x30000 },
	[GRX500_FSQM0] = { 0x1e800000, 0x100000 },
	[GRX500_FSQM1] = { 0x1e900000, 0x100000 },
	[GRX500_DMA_DESC] = { 0x1e500000, 0x100000 },
};

enum grx500_pool_id {
	GRX500_STANDARD_POOL,
	GRX500_JUMBO_POOL,
	GRX500_POOL_COUNT,
};

static const resource_size_t grx500_pool_size[] = {
	[GRX500_STANDARD_POOL] = 0x1200000,
	[GRX500_JUMBO_POOL] = 0x800000,
};

static const resource_size_t grx500_pool_alignment[] = {
	[GRX500_STANDARD_POOL] = 0x800,
	[GRX500_JUMBO_POOL] = 0x2000,
};

/* Hardware TX DQM routes: index 0 is WAN (GSWIP-R, DMA1TX), indices 1..4
 * correspond to GSWIP-L endpoints 2..5 (lan_dqm[1..4], DMA2TX).
 */
struct grx500_datapath {
	struct device *dev;
	struct net_device *ndev; /* Single CPU conduit (eth0) + NAPI anchor. */
	struct napi_struct napi;
	struct timer_list tx_wake_timer;
	struct u64_stats_sync rx_syncp;
	struct u64_stats_sync tx_syncp;
	u64 rx_packets;
	u64 rx_bytes;
	u64 rx_errors;
	u64 rx_dropped;
	u64 tx_packets;
	u64 tx_bytes;
	u64 tx_dropped;
	atomic_long_t tx_timeout_dropped;
	u32 rx_bad_descriptor;
	dma_addr_t pool_dma[GRX500_POOL_COUNT];
	bool pool_mapped[GRX500_POOL_COUNT];
	bool started;
	spinlock_t tx_lock;
	void __iomem *base[GRX500_RESOURCE_COUNT];
	spinlock_t tmu_lock;
	phys_addr_t pool_phys[GRX500_POOL_COUNT];
	int irq[GRX500_CBM_IRQS];
	struct grx500_eqm_route lan_eqm[GRX500_LAN_EQM_ROUTES];
	struct grx500_eqm_route gswr_eqm;
	struct grx500_dqm_route lan_dqm[GRX500_LAN_DQM_ROUTES];
	struct grx500_dqm_route wan_dqm;
	u32 cpu_dqm_port;
	u32 cpu_dqm_queue;
	u32 dma_channels[GRX500_CBM_DMA_ROUTES];
	u32 dma_channels_acquired;
	struct {
		u32 rx;
		u32 tx;
		bool rx_requested;
		bool tx_requested;
		bool desc_allocated;
		bool buffers_allocated;
	} p2p[GRX500_LAN_P2P_CLASSES];
	struct mii_bus *gswip[2];
	struct grx500_pae_ctx *pae;
	DECLARE_HASHTABLE(pae_flows, 8);
	struct mutex pae_lock;
	u32 pae_bind_cnt;
	u32 pae_replace_ok;
	u32 pae_replace_err;
	u32 pae_del_cnt;
	u32 pae_stats_cnt;
	u32 pae_hit_cnt;
	int pae_last_err;
	u8 pae_last_stage;
	struct mutex dcdp_lock;
	struct grx500_dcdp_port dcdp_ports[GRX500_DCDP_MAX_RADIOS];
};

static struct grx500_datapath *g_grx500_dp;

struct grx500_pae_flow_entry {
	struct hlist_node hnode;
	unsigned long cookie;
	u16 session_idx;
};

static struct grx500_datapath *grx500_conduit_priv(struct net_device *ndev)
{
	return *(struct grx500_datapath **)netdev_priv(ndev);
}

static const struct grx500_dqm_route *grx500_tx_route(struct grx500_datapath *dp,
						     int idx, bool *is_wan)
{
	if (idx == 0) {
		*is_wan = true;
		return &dp->wan_dqm;
	}
	*is_wan = false;
	return &dp->lan_dqm[idx];
}

static void grx500_cbm_release_switch_buses(void *data)
{
	struct grx500_datapath *dp = data;
	int i;

	for (i = 0; i < ARRAY_SIZE(dp->gswip); i++) {
		if (dp->gswip[i]) {
			put_device(&dp->gswip[i]->dev);
			dp->gswip[i] = NULL;
		}
	}
}

static int grx500_cbm_find_switch_buses(struct platform_device *pdev,
					 struct grx500_datapath *dp)
{
	static const char * const names[] = {
		"lantiq,gswip-left", "lantiq,gswip-right",
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct device_node *node;

		node = of_parse_phandle(pdev->dev.of_node, names[i], 0);
		if (!node) {
			grx500_cbm_release_switch_buses(dp);
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "GSWIP MDIO phandle is missing\n");
		}
		dp->gswip[i] = of_mdio_find_bus(node);
		of_node_put(node);
		if (!dp->gswip[i]) {
			grx500_cbm_release_switch_buses(dp);
			return -EPROBE_DEFER;
		}
	}
	return devm_add_action_or_reset(&pdev->dev,
					grx500_cbm_release_switch_buses, dp);
}

static int __maybe_unused grx500_cbm_prepare_switches(struct grx500_datapath *dp)
{
	int ret;

	ret = grx500_gswip_prepare_pce(dp->gswip[0], false);
	if (ret)
		return ret;
	ret = grx500_gswip_prepare_pce(dp->gswip[1], true);
	if (ret)
		return ret;
	ret = grx500_gswip_prepare_pmac(dp->gswip[0], false);
	if (ret)
		return ret;
	return grx500_gswip_prepare_pmac(dp->gswip[1], true);
}

static void grx500_cbm_release_dma(struct grx500_datapath *dp)
{
	while (dp->dma_channels_acquired) {
		u32 chan = dp->dma_channels[--dp->dma_channels_acquired];

		ltq_dma_chan_off(chan);
		ltq_free_dma(chan);
	}
}

static void grx500_cbm_release_p2p(struct grx500_datapath *dp)
{
	int i;

	for (i = GRX500_LAN_P2P_CLASSES - 1; i >= 0; i--) {
		if (dp->p2p[i].tx_requested)
			ltq_dma_chan_off(dp->p2p[i].tx);
		if (dp->p2p[i].rx_requested)
			ltq_dma_chan_off(dp->p2p[i].rx);
		if (dp->started && !i && dp->p2p[i].buffers_allocated)
			ltq_dma_chan_p2p_snapshot(dp->p2p[i].rx);
		if (dp->p2p[i].buffers_allocated)
			ltq_dma_chan_data_buf_free(dp->p2p[i].rx);
		if (dp->p2p[i].desc_allocated)
			ltq_dma_chan_desc_free(dp->p2p[i].rx);
		if (dp->p2p[i].rx_requested)
			ltq_free_dma(dp->p2p[i].rx);
		if (dp->p2p[i].tx_requested)
			ltq_free_dma(dp->p2p[i].tx);
		memset(&dp->p2p[i], 0, sizeof(dp->p2p[i]));
	}
}

/* Vendor cbm.c:setup_DMA_p2p()/turn_on_DMA_p2p(). DMA2RX0-3 writes
 * GSWIP-L CPU egress into shared buffers; DMA1TX0-3 injects into GSWIP-R. This is
 * the ordinary LAN receive path, not optional PPA acceleration.
 */
static int grx500_cbm_start_p2p(struct grx500_datapath *dp)
{
	int i, ret;

	for (i = 0; i < GRX500_LAN_P2P_CLASSES; i++) {
		dp->p2p[i].tx = _DMA_C(DMA1TX, 0, i);
		dp->p2p[i].rx = _DMA_C(DMA2RX, 0, i);
		ret = ltq_request_dma(dp->p2p[i].tx, "grx500-lan-p2p-tx");
		if (ret)
			goto fail;
		dp->p2p[i].tx_requested = true;
		ret = ltq_request_dma(dp->p2p[i].rx, "grx500-lan-p2p-rx");
		if (ret)
			goto fail;
		dp->p2p[i].rx_requested = true;
		ret = ltq_dma_chan_desc_alloc(dp->p2p[i].rx, 32);
		if (ret)
			goto fail;
		dp->p2p[i].desc_allocated = true;
		ret = ltq_dma_chan_p2p_shared_buffer_cfg(dp->p2p[i].rx);
		if (ret)
			goto fail;
		ret = ltq_dma_chan_data_buf_alloc(dp->p2p[i].rx);
		if (ret)
			goto fail;
		dp->p2p[i].buffers_allocated = true;
		ret = ltq_dma_p2p_cfg(dp->p2p[i].rx, dp->p2p[i].tx);
		if (ret)
			goto fail;
		ret = ltq_dma_chan_irq_disable(dp->p2p[i].rx);
		if (ret)
			goto fail;
		ret = ltq_dma_chan_irq_disable(dp->p2p[i].tx);
		if (ret)
			goto fail;
	}
	/* Vendor setup_DMA_p2p() configures all classes before
	 * turn_on_DMA_p2p() enables any channel.
	 */
	for (i = 0; i < GRX500_LAN_P2P_CLASSES; i++) {
		ret = ltq_dma_chan_on(dp->p2p[i].tx);
		if (ret)
			goto fail;
		ret = ltq_dma_chan_on(dp->p2p[i].rx);
		if (ret)
			goto fail;
	}
	return 0;

fail:
	grx500_cbm_release_p2p(dp);
	return ret;
}

static int grx500_cbm_request_dma_channel(struct grx500_datapath *dp,
					   u32 controller, u32 channel,
					   phys_addr_t descriptor_base)
{
	u32 chan = _DMA_C(controller, 0, channel);
	int ret;

	ret = ltq_request_dma(chan, "grx500-cbm");
	if (ret)
		return ret;
	ret = ltq_dma_chan_desc_cfg(chan, descriptor_base, 2);
	if (!ret)
		ret = ltq_dma_chan_irq_disable(chan);
	if (ret) {
		ltq_free_dma(chan);
		return ret;
	}
	dp->dma_channels[dp->dma_channels_acquired++] = chan;
	return 0;
}

static int __maybe_unused grx500_cbm_prepare_dma(struct grx500_datapath *dp)
{
	phys_addr_t base = grx500_layout[GRX500_DMA_DESC].start;
	int i, ret;

	/* DQM ports 6-17 use DMA2TX and EQM ports 5/6 use DMA2RX.
	 * The CBM descriptor SRAM is physically addressed by the DMA engines.
	 * Caller must initialize/own the descriptors before channels are ON.
	 */
	for (i = 0; i < GRX500_LAN_DQM_ROUTES; i++) {
		const struct grx500_dqm_route *route = &dp->lan_dqm[i];
		phys_addr_t desc = base + 0x40000 +
				   (route->port - 5) * 0x1000;

		ret = grx500_cbm_request_dma_channel(dp, DMA2TX,
						    route->dma_channel, desc);
		if (ret)
			goto release;
	}
	for (i = 0; i < GRX500_LAN_EQM_ROUTES; i++) {
		const struct grx500_eqm_route *route = &dp->lan_eqm[i];
		phys_addr_t desc = base + (route->port - 5) * 0x1000 +
				   (route->jumbo ? 0x800 : 0);

		ret = grx500_cbm_request_dma_channel(dp, DMA2RX,
						    route->dma_channel, desc);
		if (ret)
			goto release;
	}
	return 0;

release:
	grx500_cbm_release_dma(dp);
	return ret;
}

/* QEMT and QSMT are indirect tables. The vendor driver spins forever on
 * QMTC valid bits; a failed GRX500 access must instead fail the bring-up.
 */
static int grx500_tmu_table_write(struct grx500_datapath *dp, u32 data_reg,
				  u32 command, u32 valid, u32 qid, u32 value)
{
	void __iomem *base = dp->base[GRX500_TMU];
	unsigned long flags;
	u32 status;
	int ret;

	if (qid > 255)
		return -EINVAL;
	spin_lock_irqsave(&dp->tmu_lock, flags);
	__raw_writel(value, base + data_reg);
	__raw_writel(command | qid, base + GRX500_TMU_QMTC);
	ret = read_poll_timeout_atomic(__raw_readl, status, status & valid,
				       1, GRX500_TMU_TABLE_TIMEOUT_US, false,
				       base + GRX500_TMU_QMTC);
	spin_unlock_irqrestore(&dp->tmu_lock, flags);
	return ret;
}

static int __maybe_unused grx500_tmu_table_write_words(struct grx500_datapath *dp,
						 u32 data_reg, u32 control_reg,
						 u32 command, u32 valid,
						 u32 index, const u32 *words,
						 unsigned int count)
{
	void __iomem *base = dp->base[GRX500_TMU];
	unsigned long flags;
	u32 status;
	int ret;
	unsigned int i;

	if (!count || count > 5 || data_reg + count * sizeof(u32) > 0x1000)
		return -EINVAL;
	spin_lock_irqsave(&dp->tmu_lock, flags);
	for (i = 0; i < count; i++)
		__raw_writel(words[i], base + data_reg + i * sizeof(u32));
	__raw_writel(command | index, base + control_reg);
	ret = read_poll_timeout_atomic(__raw_readl, status, status & valid,
				       1, GRX500_TMU_TABLE_TIMEOUT_US, false,
				       base + control_reg);
	spin_unlock_irqrestore(&dp->tmu_lock, flags);
	return ret;
}

static int __maybe_unused grx500_tmu_prepare_packet_pointers(struct grx500_datapath *dp)
{
	u32 words[4] = { 0 };
	u32 i;
	int ret;

	/* Vendor TMU uses a circular free-PDU list of exactly 6144 entries.
	 * A queue cannot accept even one packet before this table and FPL are
	 * initialized. Call only while TMU admission and DMA are disabled.
	 */
	for (i = 0; i < GRX500_TMU_PPT_ENTRIES; i++) {
		words[0] = ((i + 1) % GRX500_TMU_PPT_ENTRIES) << 16;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_PPT,
						   GRX500_TMU_PPTC,
						   GRX500_TMU_PPTC_WRITE,
						   GRX500_TMU_PPTC_VALID,
						   i, words, ARRAY_SIZE(words));
		if (ret)
			return ret;
	}
	__raw_writel((GRX500_TMU_PPT_ENTRIES - 1) << 16,
		     dp->base[GRX500_TMU] + GRX500_TMU_FPL);
	return 0;
}

static int __maybe_unused grx500_tmu_prepare_queues(struct grx500_datapath *dp)
{
	/* Matches the GRX500 vendor TMU defaults, not a fresh policy choice.
	 * All queues remain disabled and point at reserved egress port 72 and
	 * scheduler input 1023 until their packet path has been installed.
	 */
	static const u32 thresholds[] = {
		0x00011320, 0x00120012, 0x00240024,
		0x0fff0fff, 0x00000004,
	};
	static const u32 occupancy[] = { 10 << 16, 0, 0 };
	static const u32 discarded[] = { 0, 0, 0, 0 };
	static const u32 fifo[] = { 0, 0x3fff3fff, 0 };
	u32 qid;
	int ret;

	for (qid = 0; qid < 256; qid++) {
		ret = grx500_tmu_table_write(dp, GRX500_TMU_QEMT,
					     GRX500_TMU_QMTC_QEW,
					     GRX500_TMU_QMTC_QEV, qid, 72);
		if (ret)
			return ret;
		ret = grx500_tmu_table_write(dp, GRX500_TMU_QSMT,
					     GRX500_TMU_QMTC_QSW,
					     GRX500_TMU_QMTC_QSV, qid, 1023);
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_QTHT,
						   GRX500_TMU_QMTC,
						   GRX500_TMU_QMTC_QTW,
						   GRX500_TMU_QMTC_QTV,
						   qid, thresholds,
						   ARRAY_SIZE(thresholds));
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_QOCT,
						   GRX500_TMU_QMTC,
						   GRX500_TMU_QMTC_QOW,
						   GRX500_TMU_QMTC_QOV,
						   qid, occupancy,
						   ARRAY_SIZE(occupancy));
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_QDCT,
						   GRX500_TMU_QMTC,
						   GRX500_TMU_QMTC_QDW,
						   GRX500_TMU_QMTC_QDV,
						   qid, discarded,
						   ARRAY_SIZE(discarded));
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_QFMT,
						   GRX500_TMU_QMTC,
						   GRX500_TMU_QMTC_QFW,
						   GRX500_TMU_QMTC_QFV,
						   qid, fifo,
						   ARRAY_SIZE(fifo));
		if (ret)
			return ret;
	}
	return 0;
}

static int __maybe_unused grx500_tmu_prepare_egress_ports(struct grx500_datapath *dp)
{
	static const u32 occupancy[] = { 0, 0 };
	static const u32 threshold[] = { 0x01200120, 0x01200120 };
	static const u32 discarded[] = { 0, 0, 0, 0 };
	u32 port;
	int ret;

	for (port = 0; port < 72; port++) {
		/* Disabled, attached to reserved scheduler block 127. */
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_EPMT,
						   GRX500_TMU_EPMTC,
						   GRX500_TMU_EPMTC_EMW,
						   GRX500_TMU_EPMTC_EMV,
						   port, (const u32[]){ 127 }, 1);
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_EPOT,
						   GRX500_TMU_EPMTC,
						   GRX500_TMU_EPMTC_EOW,
						   GRX500_TMU_EPMTC_EOV,
						   port, occupancy,
						   ARRAY_SIZE(occupancy));
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_EPTT,
						   GRX500_TMU_EPMTC,
						   GRX500_TMU_EPMTC_ETW,
						   GRX500_TMU_EPMTC_ETV,
						   port, threshold,
						   ARRAY_SIZE(threshold));
		if (ret)
			return ret;
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_EPDT,
						   GRX500_TMU_EPMTC,
						   GRX500_TMU_EPMTC_EDW,
						   GRX500_TMU_EPMTC_EDV,
						   port, discarded,
						   ARRAY_SIZE(discarded));
		if (ret)
			return ret;
	}
	return 0;
}

static int __maybe_unused grx500_tmu_prepare_schedulers(struct grx500_datapath *dp)
{
	static const u32 inactive_input[] = { 255, 255, 0, 0 };
	static const u32 inactive_output[] = { 72, 255 };
	u32 index;
	int ret;

	for (index = 0; index < 1024; index++) {
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_SBITR,
						   GRX500_TMU_SBITC,
						   GRX500_TMU_SBC_WRITE,
						   GRX500_TMU_SBC_VALID,
						   index, inactive_input,
						   ARRAY_SIZE(inactive_input));
		if (ret)
			return ret;
	}
	for (index = 0; index < 128; index++) {
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_SBOTR,
						   GRX500_TMU_SBOTC,
						   GRX500_TMU_SBC_WRITE,
						   GRX500_TMU_SBC_VALID,
						   index, inactive_output,
						   ARRAY_SIZE(inactive_output));
		if (ret)
			return ret;
	}
	return 0;
}

static int __maybe_unused grx500_tmu_prepare_token_buckets(struct grx500_datapath *dp)
{
	/* TBSTR3/5/7/9 start new 16-byte register banks. */
	static const u16 offsets[] = {
		0x500, 0x504, 0x508, 0x510, 0x514, 0x520,
		0x524, 0x530, 0x534, 0x540, 0x544,
	};
	static const u32 values[] = {
		1023, 0xffff, 0xffff, 0, 0, 0, 0, 0, 0, 0, 0,
	};
	void __iomem *base = dp->base[GRX500_TMU];
	unsigned long flags;
	u32 status, index;
	int ret;
	unsigned int i;

	for (index = 0; index < 256; index++) {
		spin_lock_irqsave(&dp->tmu_lock, flags);
		for (i = 0; i < ARRAY_SIZE(values); i++)
			__raw_writel(values[i], base + offsets[i]);
		__raw_writel(GRX500_TMU_TBSTC_WRITE | index,
			     base + GRX500_TMU_TBSTC);
		ret = read_poll_timeout_atomic(__raw_readl, status,
					       status & GRX500_TMU_TBSTC_VALID,
					       1, GRX500_TMU_TABLE_TIMEOUT_US,
					       false, base + GRX500_TMU_TBSTC);
		spin_unlock_irqrestore(&dp->tmu_lock, flags);
		if (ret)
			return ret;
	}
	return 0;
}

static void __maybe_unused grx500_tmu_prepare_basic(struct grx500_datapath *dp)
{
	void __iomem *base = dp->base[GRX500_TMU];
	u32 i;

	/* Keep admission off throughout table initialization. On the real
	 * board the vendor uses crawler period 10 (not emulator period 1).
	 */
	__raw_writel(0, base + GRX500_TMU_CTRL);
	__raw_writel(GRX500_TMU_PPT_ENTRIES, base + GRX500_TMU_FPCR);
	__raw_writel(0, base + GRX500_TMU_FPTHR);
	__raw_writel(0, base + GRX500_TMU_TIMER);
	__raw_writel(0x815, base + GRX500_TMU_LFSR);
	__raw_writel(10, base + GRX500_TMU_CPR);
	__raw_writel(0, base + GRX500_TMU_CSR);
	__raw_writel(0, base + GRX500_TMU_GOCCR);
	__raw_writel(0, base + GRX500_TMU_IRNCR);
	__raw_writel(0, base + GRX500_TMU_IRNICR);
	__raw_writel(0, base + GRX500_TMU_IRNEN);
	for (i = 0; i < 4; i++) {
		__raw_writel(9216 * 66 / 100, base + GRX500_TMU_GOTHR + 4 * i);
		__raw_writel(0, base + GRX500_TMU_GPDCR + 4 * i);
		__raw_writel(0, base + GRX500_TMU_LPIC + 4 * i);
		__raw_writel(0, base + GRX500_TMU_LPIT + 4 * i);
	}
	for (i = 0; i < 8; i++)
		__raw_writel(0, base + GRX500_TMU_QFILL + 4 * i);
	for (i = 0; i < 3; i++)
		__raw_writel(0, base + GRX500_TMU_EPFR + 4 * i);
	__raw_writel(0, base + GRX500_TMU_TBIDCR);
	__raw_writel(0, base + GRX500_TMU_ERDR);
	__raw_writel(0, base + GRX500_TMU_TACPER);
	/* MAXTB=255, RPS=1, DTA=0, FRZ=0, ACT=0. */
	__raw_writel((255 << 8) | BIT(4), base + GRX500_TMU_CTRL);
}

static int __maybe_unused grx500_tmu_prepare(struct grx500_datapath *dp)
{
	int ret;

	/* This takes ownership of the entire TMU, so it must never run from
	 * probe. Build the whole inactive table state first; the caller must
	 * then create queue/scheduler/port paths and finish CBM RX before ACT.
	 */
	grx500_tmu_prepare_basic(dp);
	ret = grx500_tmu_prepare_queues(dp);
	if (ret)
		return ret;
	ret = grx500_tmu_prepare_egress_ports(dp);
	if (ret)
		return ret;
	ret = grx500_tmu_prepare_schedulers(dp);
	if (ret)
		return ret;
	ret = grx500_tmu_prepare_token_buckets(dp);
	if (ret)
		return ret;
	return grx500_tmu_prepare_packet_pointers(dp);
}

static int __maybe_unused grx500_tmu_prepare_lan_queue_maps(struct grx500_datapath *dp)
{
	int i, ret;

	/* The reference board uses one flat scheduler block per LAN DQM port.
	 * These two mappings are only part of queue creation. The caller must
	 * first initialize the remaining TMU tables, then configure/enable the
	 * scheduler input, egress queue and port before enabling DMA.
	 */
	for (i = 0; i < GRX500_LAN_DQM_ROUTES; i++) {
		const struct grx500_dqm_route *route = &dp->lan_dqm[i];
		u32 sbin = (route->tmu_queue - 16) << 3;

		ret = grx500_tmu_table_write(dp, GRX500_TMU_QEMT,
					     GRX500_TMU_QMTC_QEW,
					     GRX500_TMU_QMTC_QEV,
					     route->tmu_queue, route->port);
		if (ret)
			return ret;
		ret = grx500_tmu_table_write(dp, GRX500_TMU_QSMT,
					     GRX500_TMU_QMTC_QSW,
					     GRX500_TMU_QMTC_QSV,
					     route->tmu_queue, sbin);
		if (ret)
			return ret;
	}
	return 0;
}

static int grx500_tmu_setup_flat_path(struct grx500_datapath *dp,
				      u32 port, u32 queue)
{
	u32 sbid = queue - 16;
	u32 sbin = sbid << 3;
	const u32 scheduler_output[] = {
		BIT(31) | port, 255,
	};
	const u32 scheduler_input[] = {
		BIT(31) | queue, 255, 0, 0,
	};
	const u32 threshold[] = {
		BIT(31) | 0x00011320, 0x00120012,
		0x00240024, 0x0fff0fff, 0x00000004,
	};
	int ret;

	ret = grx500_tmu_table_write_words(dp, GRX500_TMU_SBOTR,
					   GRX500_TMU_SBOTC, BIT(16),
					   GRX500_TMU_SBC_VALID,
					   sbid, scheduler_output, 1);
	if (ret)
		return ret;
	ret = grx500_tmu_table_write_words(dp, GRX500_TMU_EPMT,
					   GRX500_TMU_EPMTC,
					   GRX500_TMU_EPMTC_EMW,
					   GRX500_TMU_EPMTC_EMV,
					   port,
					   (const u32[]){ BIT(31) | sbid }, 1);
	if (ret)
		return ret;
	ret = grx500_tmu_table_write(dp, GRX500_TMU_QEMT,
				     GRX500_TMU_QMTC_QEW,
				     GRX500_TMU_QMTC_QEV,
				     queue, port);
	if (ret)
		return ret;
	ret = grx500_tmu_table_write(dp, GRX500_TMU_QSMT,
				     GRX500_TMU_QMTC_QSW,
				     GRX500_TMU_QMTC_QSV,
				     queue, sbin);
	if (ret)
		return ret;
	ret = grx500_tmu_table_write_words(dp, GRX500_TMU_SBITR,
					   GRX500_TMU_SBITC,
					   GRX500_TMU_SBC_WRITE,
					   GRX500_TMU_SBC_VALID,
					   sbin, scheduler_input,
					   ARRAY_SIZE(scheduler_input));
	if (ret)
		return ret;
	return grx500_tmu_table_write_words(dp, GRX500_TMU_QTHT,
					    GRX500_TMU_QMTC,
					    GRX500_TMU_QMTC_QTW,
					    GRX500_TMU_QMTC_QTV,
					    queue, threshold,
					    ARRAY_SIZE(threshold));
}

static int grx500_tmu_create_lan_paths(struct grx500_datapath *dp)
{
	int i, ret;

	/* One flat scheduler block and queue per CBM LAN DQM route, matching
	 * the vendor's queue 16..27 and egress port 6..17 layout. TMU ACT must
	 * stay clear until CBM dequeue, DMA descriptors and IRQs are ready.
	 */
	for (i = 0; i <= GRX500_LAN_DQM_ROUTES + 1; i++) {
		u32 port = i == GRX500_LAN_DQM_ROUTES + 1 ? dp->wan_dqm.port :
			   i == GRX500_LAN_DQM_ROUTES ? dp->cpu_dqm_port : dp->lan_dqm[i].port;
		u32 queue = i == GRX500_LAN_DQM_ROUTES + 1 ? dp->wan_dqm.tmu_queue :
			    i == GRX500_LAN_DQM_ROUTES ? dp->cpu_dqm_queue : dp->lan_dqm[i].tmu_queue;

		ret = grx500_tmu_setup_flat_path(dp, port, queue);
		if (ret)
			return ret;
	}
	return 0;
}

static void __maybe_unused grx500_cbm_prepare_cpu_qidt(struct grx500_datapath *dp)
{
	void __iomem *qidt = dp->base[GRX500_QIDT];
	u32 key, word, shift, value;

	/* All 16,384 classification slots initially drop (queue 255).
	 * Vendor DQ2 setup then maps endpoint 0, flow-ID high bit 0 and every
	 * class/MPE/ENC/DEC/flow-ID-low combination to CPU queue 35. The LAN
	 * port allocation later maps LAN endpoints 1..6 to DQM queues 16..21.
	 * Keep this out of probe: it would replace the firmware's QIDT globally.
	 */
	for (word = 0; word < GRX500_QIDT_WORDS; word++)
		__raw_writel(0xffffffff, qidt + word * sizeof(u32));
	for (key = 0; key < 0x2000; key++) {
		if (key & 0xf0)
			continue;
		word = key >> 2;
		shift = (key & 3) * 8;
		value = __raw_readl(qidt + word * sizeof(u32));
		value &= ~(0xffu << shift);
		value |= dp->cpu_dqm_queue << shift;
		__raw_writel(value, qidt + word * sizeof(u32));
	}
	/* Vendor epg_lookup_table maps LAN endpoints 1..6 to CBM DQM ports
	 * 6..11, and cbm_dp_port_alloc() maps each endpoint to that port's
	 * default queue 16..21 for every classification combination. AX50 LAN1
	 * is physical GSWIP-L port 5, hence endpoint 5 and queue 20.
	 */
	for (key = 0; key < 0x4000; key++) {
		u32 endpoint = (key >> 4) & 0xf;

		if (endpoint != GRX500_WAN_ENDPOINT && (endpoint < 2 || endpoint > 5))
			continue;
		word = key >> 2;
		shift = (key & 3) * 8;
		value = __raw_readl(qidt + word * sizeof(u32));
		value &= ~(0xffu << shift);
		value |= (endpoint == GRX500_WAN_ENDPOINT ? dp->wan_dqm.tmu_queue :
			  dp->lan_dqm[endpoint - 1].tmu_queue) << shift;
		__raw_writel(value, qidt + word * sizeof(u32));
	}
}

/* The vendor programs both the physical base and an IOCU-coherent alias.
 * The mask strips the DDR base bit from the 0x2xxxxxxx physical address.
 */
static u32 grx500_cbm_iocu_alias(phys_addr_t phys)
{
	return 0xc0000000 | (phys & 0x1fffffff);
}

static int grx500_cbm_frame_base(struct grx500_datapath *dp,
				  phys_addr_t addr, phys_addr_t *frame)
{
	int id;

	for (id = 0; id < GRX500_POOL_COUNT; id++) {
		phys_addr_t start = dp->pool_phys[id];
		resource_size_t size = grx500_pool_size[id];
		resource_size_t frame_size = grx500_pool_alignment[id];

		if (addr >= start && addr - start < size) {
			*frame = start + round_down(addr - start, frame_size);
			return id;
		}
	}
	return -ERANGE;
}

/* CPU EQM new_sptr pops one standard CBM pool buffer. The register returns
 * 0xfffff800 when empty. The caller must own the current CPU's EQM port.
 */
static int __maybe_unused grx500_cbm_cpu_alloc_standard(struct grx500_datapath *dp,
						 u32 cpu, phys_addr_t *frame,
						 u32 *observed)
{
	void __iomem *reg;
	phys_addr_t base;
	u32 addr;
	int i, id;

	if (cpu >= GRX500_CBM_CPU_PORTS || !frame)
		return -EINVAL;
	reg = dp->base[GRX500_EQM] + GRX500_EQM_CPU_NEW_SPTR +
	      cpu * GRX500_CBM_CPU_PORT_STRIDE;
	for (i = 0; i <= GRX500_CBM_ALLOC_RETRIES; i++) {
		addr = __raw_readl(reg);
		if ((addr & 0xfffff800) != 0xfffff800)
			break;
	}
	/* Do not read NEW_SPTR again for diagnostics: each read allocates. */
	if (observed)
		*observed = addr;
	if (i > GRX500_CBM_ALLOC_RETRIES)
		return -ENOBUFS;
	id = grx500_cbm_frame_base(dp, addr, &base);
	if (id != GRX500_STANDARD_POOL || base != addr)
		return -EIO;
	*frame = base;
	return 0;
}

/* Return a consumed CBM frame through the CPU DQM EGP, never via kfree().
 * The RX/TX caller must complete its DMA cache synchronization first.
 */
static int __maybe_unused grx500_cbm_cpu_return(struct grx500_datapath *dp,
						u32 cpu, phys_addr_t addr)
{
	phys_addr_t frame;
	int ret;

	if (cpu >= GRX500_CBM_CPU_PORTS)
		return -EINVAL;
	ret = grx500_cbm_frame_base(dp, addr, &frame);
	if (ret < 0)
		return ret;
	wmb();
	__raw_writel((u32)frame, dp->base[GRX500_DQM] +
		     GRX500_DQM_CPU_PTR_RTN +
		     cpu * GRX500_CBM_CPU_PORT_STRIDE);
	return 0;
}

static void __maybe_unused grx500_cbm_prepare_cpu_ports(struct grx500_datapath *dp)
{
	u32 cpu;

	/* Match vendor EQM/DQM CPU port configuration, but keep interrupts
	 * masked until the receive poller and error handlers are installed.
	 * The global EQM/DQM engines remain off at this point.
	 */
	for (cpu = 0; cpu < GRX500_CBM_CPU_PORTS; cpu++) {
		__raw_writel(0xfc7, dp->base[GRX500_EQM] +
			     GRX500_EQM_CPU_IGP0 +
			     cpu * GRX500_CBM_CPU_PORT_STRIDE);
		__raw_writel(0, dp->base[GRX500_EQM] +
			     GRX500_EQM_CPU_POCC +
			     cpu * GRX500_CBM_CPU_PORT_STRIDE);
		__raw_writel(0, dp->base[GRX500_EQM] +
			     GRX500_EQM_CPU_IRNEN +
			     cpu * GRX500_CBM_CPU_PORT_STRIDE);
		__raw_writel(0x10f | (cpu << 16), dp->base[GRX500_DQM] +
			     GRX500_DQM_CPU_CFG +
			     cpu * GRX500_CBM_CPU_PORT_STRIDE);
	}
}

static void __maybe_unused grx500_cbm_prepare_load_spreader(struct grx500_datapath *dp)
{
	void __iomem *ls = dp->base[GRX500_LS];
	u32 cpu;

	/* Vendor uses WRR with equal weight 2 on each of four CPU ports,
	 * queue threshold 7 and counter threshold 0x2000. Leave port enable,
	 * global enable and interrupt mask clear until NAPI owns RX.
	 */
	__raw_writel(0, ls + GRX500_LS_GLOBAL_CTRL);
	for (cpu = 0; cpu < GRX500_CBM_CPU_PORTS; cpu++)
		__raw_writel((0x2000 << 16) | (7 << 8) |
			     (cpu ? 0xc : 0xe),
			     ls + cpu * GRX500_LS_PORT_STRIDE +
			     GRX500_LS_PORT_CTRL);
	__raw_writel(0x00aa0000, ls + GRX500_LS_SPREAD_CTRL);
	__raw_writel(0, ls + GRX500_LS_IRNEN);
}

static int grx500_cbm_prepare_fsqm(struct grx500_datapath *dp,
				   enum grx500_pool_id id)
{
	static const u32 thresholds[] = {
		GRX500_FSQM_FSQT0, GRX500_FSQM_FSQT1,
		GRX500_FSQM_FSQT2, GRX500_FSQM_FSQT3,
		GRX500_FSQM_FSQT4,
	};
	void __iomem *fsqm = dp->base[id == GRX500_STANDARD_POOL ?
					 GRX500_FSQM0 : GRX500_FSQM1];
	u32 frame_size = id == GRX500_STANDARD_POOL ? 0x800 : 0x2000;
	u32 frames = grx500_pool_size[id] / frame_size;
	u32 last = frames - 2;
	u32 i;

	/* FSQM uses 15-bit logical segment addresses. The last segment is
	 * intentionally excluded, exactly as in the vendor pool geometry.
	 */
	if (frames < 3 || frames > 0x8000)
		return -EINVAL;
	/* The vendor initializes every LLT/RCNT entry, including the reserved
	 * final segment. OFSQ deliberately excludes that segment.
	 */
	for (i = 0; i < frames; i++) {
		__raw_writel(i == last ? 0x7fff : (i + 1) % frames,
			     fsqm + GRX500_FSQM_RAM + i * sizeof(u32));
		__raw_writel(1, fsqm + GRX500_FSQM_RCNT + i * sizeof(u32));
	}
	__raw_writel(last << 16, fsqm + GRX500_FSQM_LSARNG);
	__raw_writel(last << 16, fsqm + GRX500_FSQM_OFSQ);
	__raw_writel(last + 1, fsqm + GRX500_FSQM_OFSC);
	__raw_writel(0x111101f, fsqm + GRX500_FSQM_IRNEN);
	for (i = 0; i < ARRAY_SIZE(thresholds); i++)
		__raw_writel((frames / 6) * (5 - i), fsqm + thresholds[i]);
	__raw_writel(0, fsqm + GRX500_FSQM_IO_BUF_RD);
	__raw_writel(0, fsqm + GRX500_FSQM_IO_BUF_WR);
	wmb();
	__raw_writel(1, fsqm + GRX500_FSQM_CTRL);
	return 0;
}

static int __maybe_unused grx500_cbm_prepare_buffers(struct grx500_datapath *dp)
{
	void __iomem *cbm = dp->base[GRX500_CBM];
	u32 ctrl;
	int ret;

	/* Called after pool validation and TMU setup, before packet DMA. */
	ret = grx500_cbm_prepare_fsqm(dp, GRX500_STANDARD_POOL);
	if (ret)
		return ret;
	ret = grx500_cbm_prepare_fsqm(dp, GRX500_JUMBO_POOL);
	if (ret)
		return ret;
	__raw_writel(dp->pool_phys[GRX500_STANDARD_POOL],
		     cbm + GRX500_CBM_SBA0);
	__raw_writel(dp->pool_phys[GRX500_JUMBO_POOL],
		     cbm + GRX500_CBM_JBA0);
	__raw_writel(grx500_cbm_iocu_alias(dp->pool_phys[GRX500_STANDARD_POOL]),
		     cbm + GRX500_CBM_SBA1);
	__raw_writel(grx500_cbm_iocu_alias(dp->pool_phys[GRX500_JUMBO_POOL]),
		     cbm + GRX500_CBM_JBA1);
	if (__raw_readl(cbm + GRX500_CBM_SBA0) != dp->pool_phys[GRX500_STANDARD_POOL] ||
	    __raw_readl(cbm + GRX500_CBM_JBA0) != dp->pool_phys[GRX500_JUMBO_POOL] ||
	    __raw_readl(cbm + GRX500_CBM_SBA1) !=
		grx500_cbm_iocu_alias(dp->pool_phys[GRX500_STANDARD_POOL]) ||
	    __raw_readl(cbm + GRX500_CBM_JBA1) !=
		grx500_cbm_iocu_alias(dp->pool_phys[GRX500_JUMBO_POOL])) {
		dev_err(dp->dev, "CBM pool base readback mismatch; packet DMA remains off\n");
		return -EIO;
	}
	/* JSEL=0 selects 8 KiB jumbo frames. Preserve all unrelated bits. */
	ctrl = __raw_readl(cbm + GRX500_CBM_CTRL);
	__raw_writel(ctrl & ~BIT(17), cbm + GRX500_CBM_CTRL);
	return 0;
}

/* DMA2RX writes into CBM-owned standard frames; DMA2TX reads them back.
 * Hold a bidirectional mapping for the whole pool while the engines run.
 * This is required because CBM, not Linux, manages individual frame reuse.
 */
static int grx500_cbm_map_pools(struct grx500_datapath *dp)
{
	int i;

	for (i = 0; i < GRX500_POOL_COUNT; i++) {
		void *cpu = phys_to_virt(dp->pool_phys[i]);
		dma_addr_t dma;

		dma = dma_map_single(dp->dev, cpu, grx500_pool_size[i],
				     DMA_BIDIRECTIONAL);
		if (dma_mapping_error(dp->dev, dma))
			return -EIO;
		dp->pool_dma[i] = dma;
		dp->pool_mapped[i] = true;
		if (dma != dp->pool_phys[i]) {
			dev_err(dp->dev, "CBM requires identity DMA mapping: pool%d %pad != %pa\n",
				i, &dma, &dp->pool_phys[i]);
			return -ERANGE;
		}
	}
	return 0;
}

static void grx500_cbm_unmap_pools(struct grx500_datapath *dp)
{
	int i;

	for (i = 0; i < GRX500_POOL_COUNT; i++) {
		if (!dp->pool_mapped[i])
			continue;
		dma_unmap_single(dp->dev, dp->pool_dma[i],
				 grx500_pool_size[i], DMA_BIDIRECTIONAL);
		dp->pool_mapped[i] = false;
	}
}

static int grx500_cbm_prepare_eqm_descriptors(struct grx500_datapath *dp,
					    const struct grx500_eqm_route *eqm)
{
	void __iomem *desc = dp->base[GRX500_DMA_DESC] +
			    (eqm->port - 5) * 0x1000;
	void __iomem *cfg = dp->base[GRX500_EQM] + GRX500_EQM_DMA_CFG +
			   (eqm->port - 5) * 0x1000;
	phys_addr_t frame;
	u32 addr;
	int i, ret;

	/* Vendor setup_eqm_dma_desc(): two standard frames per enqueue port.
	 * In big-endian mode the DMA descriptor's control word is at +8.
	 */
	for (i = 0; i < eqm->descriptors; i++) {
		ret = grx500_cbm_cpu_alloc_standard(dp, 0, &frame, NULL);
		if (ret)
			return ret;
		addr = frame + GRX500_RX_BUFFER_OFFSET;
		__raw_writel(0, desc + i * 16);
		__raw_writel(0, desc + i * 16 + 4);
		__raw_writel(addr & ~7u, desc + i * 16 + 12);
		wmb();
		__raw_writel(BIT(31) | 2048 | ((addr & 7) << 23),
			     desc + i * 16 + 8);
	}
	__raw_writel(0, cfg + 8); /* Vendor init_cbm_eqm_dma_port(): POCC. */
	__raw_writel(0x701, cfg);
	dev_info(dp->dev, "EQM%u seeded standard RX buffers: %08x %08x\n",
		 eqm->port, __raw_readl(desc + 12), __raw_readl(desc + 28));
	return 0;
}

static int grx500_cbm_prepare_port_descriptors(struct grx500_datapath *dp)
{
	void __iomem *desc = dp->base[GRX500_DMA_DESC];
	int p, i, ret;

	ret = grx500_cbm_prepare_eqm_descriptors(dp, &dp->lan_eqm[0]);
	if (ret)
		return ret;
	ret = grx500_cbm_prepare_eqm_descriptors(dp, &dp->gswr_eqm);
	if (ret)
		return ret;
	for (p = 0; p < GRX500_PORTS; p++) {
		bool is_wan;
		const struct grx500_dqm_route *dqm = grx500_tx_route(dp, p, &is_wan);

		for (i = 0; i < dqm->descriptors; i++) {
			void __iomem *ring = desc + 0x40000 +
				(dqm->port - 5) * 0x1000 + i * 16;

			__raw_writel(0, ring);
			__raw_writel(0, ring + 4);
			__raw_writel(0, ring + 8);
			__raw_writel(0, ring + 12);
		}
		__raw_writel(0x101 | (dqm->port << 16),
			     dp->base[GRX500_DQM] + GRX500_DQM_DMA_CFG +
			     (dqm->port - 6) * 0x1000);
	}
	return 0;
}

static int grx500_cbm_start_port_dma(struct grx500_datapath *dp)
{
	phys_addr_t base = grx500_layout[GRX500_DMA_DESC].start;
	int p, ret;

	/* Start the shared GSWIP-R CPU sink before any producer. */
	ret = grx500_cbm_request_dma_channel(dp, DMA1RX, dp->gswr_eqm.dma_channel,
			base + (dp->gswr_eqm.port - 5) * 0x1000);
	if (ret)
		return ret;
	ret = grx500_cbm_request_dma_channel(dp, DMA2RX, dp->lan_eqm[0].dma_channel,
			base + (dp->lan_eqm[0].port - 5) * 0x1000);
	if (ret)
		return ret;
	for (p = 0; p < GRX500_PORTS; p++) {
		bool is_wan;
		const struct grx500_dqm_route *dqm = grx500_tx_route(dp, p, &is_wan);

		ret = grx500_cbm_request_dma_channel(dp,
			is_wan ? DMA1TX : DMA2TX, dqm->dma_channel,
			base + 0x40000 + (dqm->port - 5) * 0x1000);
		if (ret)
			return ret;
	}
	for (p = 0; p < dp->dma_channels_acquired; p++) {
		ret = ltq_dma_chan_on(dp->dma_channels[p]);
		if (ret)
			return ret;
	}
	return 0;
}

static void grx500_cbm_irq_mask(struct grx500_datapath *dp)
{
	__raw_writel(0, dp->base[GRX500_CBM] +
		     GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE + 8);
	__raw_writel(0, dp->base[GRX500_LS] + GRX500_LS_IRNEN);
	(void)__raw_readl(dp->base[GRX500_LS] + GRX500_LS_IRNEN);
}

static void grx500_cbm_irq_unmask(struct grx500_datapath *dp)
{
	__raw_writel(GRX500_RX_LS_IRQ, dp->base[GRX500_LS] + GRX500_LS_IRNCR);
	__raw_writel(GRX500_RX_CBM_IRQ, dp->base[GRX500_CBM] +
		     GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE);
	__raw_writel(GRX500_RX_LS_IRQ, dp->base[GRX500_LS] + GRX500_LS_IRNEN);
	__raw_writel(GRX500_RX_CBM_IRQ, dp->base[GRX500_CBM] +
		     GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE + 8);
}

static irqreturn_t grx500_irq(int irq, void *data)
{
	struct grx500_datapath *dp = data;

	if (!(__raw_readl(dp->base[GRX500_CBM] +
		 GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE) & GRX500_RX_CBM_IRQ))
		return IRQ_NONE;
	grx500_cbm_irq_mask(dp);
	napi_schedule(&dp->napi);
	return IRQ_HANDLED;
}

static int grx500_poll(struct napi_struct *napi, int budget)
{
	struct grx500_datapath *dp = container_of(napi, struct grx500_datapath, napi);
	void __iomem *ls = dp->base[GRX500_LS] +
			   GRX500_RX_LS_PORT * GRX500_LS_PORT_STRIDE;
	int done = 0;

	while (done < budget &&
	       ((__raw_readl(ls + GRX500_LS_PORT_STATUS) >> 7) & 0xf)) {
		phys_addr_t frame;
		struct sk_buff *skb;
		u32 ptr, flags, len, offset;
		u8 *data;
		int pool;

		(void)__raw_readl(ls);
		(void)__raw_readl(ls + 4);
		ptr = __raw_readl(ls + 8);
		flags = __raw_readl(ls + 12);
		len = flags & 0xffff;
		offset = (flags >> 23) & 7;
		pool = grx500_cbm_frame_base(dp, ptr, &frame);
		if (pool != GRX500_STANDARD_POOL ||
		    ptr + offset + len > frame + 2048 ||
		    len < GRX500_PMAC_SIZE + ETH_HLEN) {
			if (dp->rx_bad_descriptor < 4)
				dev_info(dp->dev, "RX descriptor rejected ptr=%08x flags=%08x pool=%d\n",
					 ptr, flags, pool);
			dp->rx_bad_descriptor++;
			if (pool >= 0)
				grx500_cbm_cpu_return(dp, 0, frame);
			done++;
			continue;
		}
		dma_sync_single_for_cpu(dp->dev, ptr & ~7u,
					len + offset, DMA_BIDIRECTIONAL);
		data = phys_to_virt(ptr + offset);
		/* Pass the full frame (8-byte PMAC prefix + Ethernet frame) to the
		 * DSA conduit; tag_grx500 strips the PMAC after eth_type_trans().
		 * napi_alloc_skb() reserves NET_SKB_PAD + NET_IP_ALIGN (2 bytes)
		 * so L3 remains 4-byte aligned after pulling ETH_HLEN + 8B PMAC.
		 */
		skb = napi_alloc_skb(napi, len);
		if (skb) {
			skb_put_data(skb, data, len);
			skb->protocol = eth_type_trans(skb, dp->ndev);
			if (unlikely(!IS_ALIGNED((unsigned long)skb->data, 4))) {
				u64_stats_update_begin(&dp->rx_syncp);
				dp->rx_errors++;
				u64_stats_update_end(&dp->rx_syncp);
				dev_kfree_skb_any(skb);
			} else {
				napi_gro_receive(napi, skb);
				u64_stats_update_begin(&dp->rx_syncp);
				dp->rx_packets++;
				dp->rx_bytes += len;
				u64_stats_update_end(&dp->rx_syncp);
			}
		} else {
			u64_stats_update_begin(&dp->rx_syncp);
			dp->rx_dropped++;
			u64_stats_update_end(&dp->rx_syncp);
		}
		dma_sync_single_for_device(dp->dev, ptr & ~7u,
					2048 - (ptr - frame), DMA_BIDIRECTIONAL);
		grx500_cbm_cpu_return(dp, 0, frame);
		done++;
	}
	if (done && netif_queue_stopped(dp->ndev) && netif_running(dp->ndev))
		netif_wake_queue(dp->ndev);
	if (done < budget && napi_complete_done(napi, done)) {
		grx500_cbm_irq_unmask(dp);
		/* A packet can arrive between the last empty check and IRQ
		 * acknowledgement in unmask(). Recheck after the MMIO writes so
		 * that a cleared edge cannot strand packets below the threshold.
		 */
		if (((__raw_readl(ls + GRX500_LS_PORT_STATUS) >> 7) & 0xf) &&
		    napi_schedule_prep(napi)) {
			grx500_cbm_irq_mask(dp);
			__napi_schedule(napi);
		}
	}
	return done;
}

static void grx500_tx_wake_timer_cb(struct timer_list *t)
{
	struct grx500_datapath *dp = from_timer(dp, t, tx_wake_timer);

	if (netif_queue_stopped(dp->ndev) && netif_running(dp->ndev))
		netif_wake_queue(dp->ndev);
}

static netdev_tx_t grx500_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct grx500_datapath *dp = grx500_conduit_priv(ndev);
	void __iomem *desc = dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC;
	unsigned long irqflags, stats_flags;
	phys_addr_t frame;
	u32 addr, len, endpoint, alloc_ptr = 0;
	u16 pmap;
	u8 *data;
	int ret;

	if (unlikely(skb->len > ndev->mtu + ETH_HLEN + VLAN_HLEN ||
		     skb->len < GRX500_PMAC_SIZE + ETH_HLEN)) {
		stats_flags = u64_stats_update_begin_irqsave(&dp->tx_syncp);
		dp->tx_dropped++;
		u64_stats_update_end_irqrestore(&dp->tx_syncp, stats_flags);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	pmap = ((u16)skb->data[6] << 8) | skb->data[7];
	if (unlikely(!pmap)) {
		stats_flags = u64_stats_update_begin_irqsave(&dp->tx_syncp);
		dp->tx_dropped++;
		u64_stats_update_end_irqrestore(&dp->tx_syncp, stats_flags);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	endpoint = __ffs(pmap);

	spin_lock_irqsave(&dp->tx_lock, irqflags);
	ret = grx500_cbm_cpu_alloc_standard(dp, 0, &frame, &alloc_ptr);
	spin_unlock_irqrestore(&dp->tx_lock, irqflags);
	if (ret) {
		netif_stop_queue(ndev);
		mod_timer(&dp->tx_wake_timer, jiffies + 1);
		return NETDEV_TX_BUSY;
	}

	addr = frame + GRX500_CBM_FRAME_OFFSET;
	/* The switch sees 8-byte PMAC plus a complete Ethernet frame. Pad
	 * undersized frames to ETH_ZLEN + GRX500_PMAC_SIZE (68 bytes).
	 */
	len = max_t(u32, skb->len, ETH_ZLEN + GRX500_PMAC_SIZE);
	data = phys_to_virt(addr);
	ret = skb_copy_bits(skb, 0, data, skb->len);
	if (unlikely(ret)) {
		spin_lock_irqsave(&dp->tx_lock, irqflags);
		grx500_cbm_cpu_return(dp, 0, frame);
		spin_unlock_irqrestore(&dp->tx_lock, irqflags);
		stats_flags = u64_stats_update_begin_irqsave(&dp->tx_syncp);
		dp->tx_dropped++;
		u64_stats_update_end_irqrestore(&dp->tx_syncp, stats_flags);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (skb->len < ETH_ZLEN + GRX500_PMAC_SIZE)
		memset(data + skb->len, 0,
		       ETH_ZLEN + GRX500_PMAC_SIZE - skb->len);
	dma_sync_single_for_device(dp->dev, frame, 2048, DMA_BIDIRECTIONAL);

	/* Big-endian CBM CPU enqueue descriptor: DW0, DW1, address, DW3.
	 * The final DW3 write hands the complete frame to CBM/TMU.
	 */
	spin_lock_irqsave(&dp->tx_lock, irqflags);
	__raw_writel(0, desc);
	__raw_writel(endpoint << GRX500_TX_DW1_EP_SHIFT, desc + 4);
	__raw_writel(addr & ~7u, desc + 8);
	wmb();
	__raw_writel(BIT(29) | BIT(28) | ((addr & 7) << 23) | len,
		     desc + 12);
	spin_unlock_irqrestore(&dp->tx_lock, irqflags);

	stats_flags = u64_stats_update_begin_irqsave(&dp->tx_syncp);
	dp->tx_packets++;
	dp->tx_bytes += skb->len;
	u64_stats_update_end_irqrestore(&dp->tx_syncp, stats_flags);
	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
}

static int grx500_start(struct grx500_datapath *dp)
{
	int i, ret;

	if (dp->started)
		return 0;
	ret = dma_set_mask_and_coherent(dp->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = grx500_cbm_map_pools(dp);
	if (ret)
		goto err_unmap;
	ret = grx500_tmu_prepare(dp);
	if (ret)
		goto err_unmap;
	ret = grx500_tmu_create_lan_paths(dp);
	if (ret)
		goto err_unmap;
	ret = grx500_cbm_prepare_buffers(dp);
	if (ret)
		goto err_unmap;
	grx500_cbm_prepare_cpu_ports(dp);
	grx500_cbm_prepare_load_spreader(dp);
	__raw_writel(0x11, dp->base[GRX500_DQM] + GRX500_DQM_CTRL);
	__raw_writel(0x11, dp->base[GRX500_EQM] + GRX500_EQM_CTRL);
	ret = grx500_cbm_prepare_port_descriptors(dp);
	if (ret)
		goto err_engines;
	grx500_cbm_prepare_cpu_qidt(dp);
	ret = grx500_cbm_prepare_switches(dp);
	if (ret)
		goto err_engines;
	if (dp->pae) {
		ret = grx500_pae_enable_pce_rules(dp->pae);
		if (ret)
			dev_warn(dp->dev, "PAE PCE rules failed: %d\n", ret);
	}
	ret = request_irq(dp->irq[GRX500_RX_IRQ_INDEX], grx500_irq, 0, "grx500-ports", dp);
	if (ret)
		goto err_engines;
	napi_enable(&dp->napi);
	ret = grx500_cbm_start_port_dma(dp);
	if (ret)
		goto err_dma;
	ret = grx500_cbm_start_p2p(dp);
	if (ret)
		goto err_dma;
	/* Keep the tested vendor gates and weights. Hardware sends this
	 * DQM2 stream to LS2 even with SPR_EN set only on port 0; NAPI and
	 * IRQ selection must therefore consume LS2 rather than assuming LS0.
	 */
	for (i = 0; i < GRX500_CBM_CPU_PORTS; i++)
		__raw_writel(0x2000070d | (i ? 0 : BIT(1)),
			     dp->base[GRX500_LS] + i * GRX500_LS_PORT_STRIDE +
			     GRX500_LS_PORT_CTRL);
	__raw_writel(1, dp->base[GRX500_LS] + GRX500_LS_GLOBAL_CTRL);
	grx500_cbm_irq_unmask(dp);
	__raw_writel(__raw_readl(dp->base[GRX500_TMU] + GRX500_TMU_CTRL) | 1,
		     dp->base[GRX500_TMU] + GRX500_TMU_CTRL);
	/* Vendor GSW_Enable() turns on SDMA/FDMA for every GSWIP-R port.
	 * The PMAC ingress map for the P2P classes includes internal ports
	 * beyond 0-3, so do not leave those receive destinations disabled.
	 */
	for (i = 0; i < GRX500_GSWIP_R_PORTS; i++) {
		ret = grx500_gswip_enable_port(dp->gswip[1], true, i);
		if (ret)
			goto err_ports;
	}
	ret = grx500_gswip_enable_port(dp->gswip[0], false, 0);
	if (ret)
		goto err_ports;
	dp->started = true;
	dev_info(dp->dev, "AX50 shared CBM/TMU active: LAN DMA2TX2-5, WAN DMA1TX15; DMA1RX0 EQM7 CPU RX\n");
	return 0;

err_ports:
	grx500_gswip_disable_port(dp->gswip[0], false, 0);
	for (i = 0; i < GRX500_GSWIP_R_PORTS; i++)
		grx500_gswip_disable_port(dp->gswip[1], true, i);
	grx500_cbm_irq_mask(dp);
err_dma:
	grx500_cbm_release_p2p(dp);
	grx500_cbm_release_dma(dp);
	napi_disable(&dp->napi);
	free_irq(dp->irq[GRX500_RX_IRQ_INDEX], dp);
err_engines:
	__raw_writel(0, dp->base[GRX500_LS] + GRX500_LS_GLOBAL_CTRL);
	__raw_writel(__raw_readl(dp->base[GRX500_TMU] + GRX500_TMU_CTRL) & ~1u,
		     dp->base[GRX500_TMU] + GRX500_TMU_CTRL);
	__raw_writel(0, dp->base[GRX500_EQM] + GRX500_EQM_CTRL);
	__raw_writel(0, dp->base[GRX500_DQM] + GRX500_DQM_CTRL);
err_unmap:
	grx500_cbm_unmap_pools(dp);
	dev_err(dp->dev, "Shared datapath start failed (%d); reboot RAM image before retry\n", ret);
	return ret;
}

static void grx500_shutdown(struct grx500_datapath *dp)
{
	int i;

	if (!dp->started)
		return;
	timer_shutdown_sync(&dp->tx_wake_timer);
	for (i = 2; i <= 5; i++)
		grx500_gswip_disable_port(dp->gswip[0], false, i);
	grx500_gswip_disable_port(dp->gswip[0], false, 0);
	for (i = 0; i < GRX500_GSWIP_R_PORTS; i++)
		grx500_gswip_disable_port(dp->gswip[1], true, i);
	grx500_cbm_irq_mask(dp);
	napi_disable(&dp->napi);
	free_irq(dp->irq[GRX500_RX_IRQ_INDEX], dp);
	grx500_cbm_release_p2p(dp);
	grx500_cbm_release_dma(dp);
	__raw_writel(0, dp->base[GRX500_LS] + GRX500_LS_GLOBAL_CTRL);
	__raw_writel(__raw_readl(dp->base[GRX500_TMU] + GRX500_TMU_CTRL) & ~1u,
		     dp->base[GRX500_TMU] + GRX500_TMU_CTRL);
	__raw_writel(0, dp->base[GRX500_EQM] + GRX500_EQM_CTRL);
	__raw_writel(0, dp->base[GRX500_DQM] + GRX500_DQM_CTRL);
	dp->started = false;
	grx500_cbm_unmap_pools(dp);
}

/* RTNL serializes port open/close and shared startup. Keep DMA pools and
 * the NAPI anchor alive across port closes; full shutdown is remove-only.
 * This permits cable-by-cable tests and later simultaneous software routing.
 */
static int grx500_open(struct net_device *ndev)
{
	struct grx500_datapath *dp = grx500_conduit_priv(ndev);
	int ret;

	ret = grx500_start(dp);
	if (ret)
		return ret;
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int grx500_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	return 0;
}

static void grx500_get_stats64(struct net_device *ndev,
			       struct rtnl_link_stats64 *stats)
{
	struct grx500_datapath *dp = grx500_conduit_priv(ndev);
	unsigned int start;

	do {
		start = u64_stats_fetch_begin(&dp->rx_syncp);
		stats->rx_packets = dp->rx_packets;
		stats->rx_bytes = dp->rx_bytes;
		stats->rx_errors = dp->rx_errors;
		stats->rx_dropped = dp->rx_dropped;
	} while (u64_stats_fetch_retry(&dp->rx_syncp, start));

	do {
		start = u64_stats_fetch_begin(&dp->tx_syncp);
		stats->tx_packets = dp->tx_packets;
		stats->tx_bytes = dp->tx_bytes;
		stats->tx_dropped = dp->tx_dropped;
	} while (u64_stats_fetch_retry(&dp->tx_syncp, start));

	stats->tx_dropped += (u64)atomic_long_read(&dp->tx_timeout_dropped);
}

static int grx500_change_mtu(struct net_device *ndev, int new_mtu)
{
	WRITE_ONCE(ndev->mtu, new_mtu);
	return 0;
}

static void grx500_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct grx500_datapath *dp = grx500_conduit_priv(ndev);

	atomic_long_inc(&dp->tx_timeout_dropped);
	netdev_warn(ndev, "TX timeout on queue %u; waking queue\n", txqueue);
	netif_wake_queue(ndev);
}

static void grx500_set_rx_mode(struct net_device *ndev)
{
	/* GSWIP PCE/PMAC forwards all unknown unicast/multicast to CPU port 0;
	 * hardware MAC filtering is managed by the DSA switch layer.
	 */
}

static int grx500_pae_port_map(struct grx500_datapath *dp,
			       struct net_device *dev, u16 *port_map)
{
	struct dsa_port *p = dsa_port_from_netdev(dev);

	if (IS_ERR(p) || dsa_port_to_conduit(p) != dp->ndev)
		return -EOPNOTSUPP;

	if (p->index == 1) {
		*port_map = BIT(GRX500_WAN_ENDPOINT);
		return 0;
	}
	if (p->index >= 2 && p->index <= 5) {
		*port_map = BIT(p->index);
		return 0;
	}
	return -EOPNOTSUPP;
}

static struct grx500_pae_flow_entry *
grx500_pae_flow_lookup(struct grx500_datapath *dp, unsigned long cookie)
{
	struct grx500_pae_flow_entry *entry;

	hash_for_each_possible(dp->pae_flows, entry, hnode, cookie) {
		if (entry->cookie == cookie)
			return entry;
	}
	return NULL;
}

static int grx500_pae_parse_mangle(const struct flow_action_entry *act,
				   struct grx500_pae_flow *flow,
				   u8 eth_hdr[12], u8 eth_mask[12],
				   bool *has_snat, bool *has_dnat)
{
	u32 off = act->mangle.offset;
	u32 val, mask, idx;

	switch (act->mangle.htype) {
	case FLOW_ACT_MANGLE_HDR_TYPE_ETH: {
		u16 val16;

		if (off > 8 || (off & 1))
			return -EOPNOTSUPP;
		memcpy(&val, &act->mangle.val, sizeof(val));
		memcpy(&mask, &act->mangle.mask, sizeof(mask));
		if (mask == 0) {
			memcpy(&eth_hdr[off], &act->mangle.val, 4);
			memset(&eth_mask[off], 0xff, 4);
		} else if (off == 4 && mask == ~0x0000ffffu) {
			val16 = (u16)val;
			memcpy(&eth_hdr[4], &val16, 2);
			memset(&eth_mask[4], 0xff, 2);
		} else if (off == 4 && mask == ~0xffff0000u) {
			val16 = (u16)(val >> 16);
			memcpy(&eth_hdr[6], &val16, 2);
			memset(&eth_mask[6], 0xff, 2);
		} else {
			return -EOPNOTSUPP;
		}
		return 0;
	}
	case FLOW_ACT_MANGLE_HDR_TYPE_IP4:
		if (flow->is_ipv6)
			return -EOPNOTSUPP;
		if (off == offsetof(struct iphdr, saddr))
			*has_snat = true;
		else if (off == offsetof(struct iphdr, daddr))
			*has_dnat = true;
		else
			return -EOPNOTSUPP;
		flow->nat_ip[0] = ntohl((__force __be32)act->mangle.val);
		return 0;
	case FLOW_ACT_MANGLE_HDR_TYPE_IP6:
		if (!flow->is_ipv6)
			return -EOPNOTSUPP;
		if (off >= offsetof(struct ipv6hdr, saddr) &&
		    off < offsetof(struct ipv6hdr, daddr)) {
			*has_snat = true;
			idx = (off - offsetof(struct ipv6hdr, saddr)) / 4;
		} else if (off >= offsetof(struct ipv6hdr, daddr) &&
			   off < sizeof(struct ipv6hdr)) {
			*has_dnat = true;
			idx = (off - offsetof(struct ipv6hdr, daddr)) / 4;
		} else {
			return -EOPNOTSUPP;
		}
		if (idx >= 4)
			return -EOPNOTSUPP;
		flow->nat_ip[idx] = ntohl((__force __be32)act->mangle.val);
		return 0;
	case FLOW_ACT_MANGLE_HDR_TYPE_TCP:
	case FLOW_ACT_MANGLE_HDR_TYPE_UDP:
		if (off != 0)
			return -EOPNOTSUPP;
		val = ntohl((__force __be32)act->mangle.val);
		mask = ntohl((__force __be32)act->mangle.mask);
		if (mask == 0x0000ffffu) {
			*has_snat = true;
			flow->nat_port = val >> 16;
		} else if (mask == 0xffff0000u) {
			*has_dnat = true;
			flow->nat_port = val & 0xffff;
		} else {
			return -EOPNOTSUPP;
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int grx500_pae_cls_replace(struct grx500_datapath *dp,
				  struct flow_cls_offload *f)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct flow_dissector *dissector = rule->match.dissector;
	struct flow_match_meta meta;
	struct flow_match_control ctrl;
	struct flow_match_basic basic;
	struct flow_match_ports ports;
	struct flow_action_entry *act;
	struct grx500_pae_flow flow = { 0 };
	struct grx500_pae_flow_entry *entry;
	struct net_device *in_dev, *out_dev = NULL;
	u8 eth_hdr[12] = { 0 }, eth_mask[12] = { 0 };
	bool has_snat = false, has_dnat = false;
	u16 in_port_map, out_port_map, sess_idx;
	int i, ret;

	if (!dp->pae) {
		dp->pae_last_stage = 1;
		dp->pae_last_err = -EOPNOTSUPP;
		dp->pae_replace_err++;
		return -EOPNOTSUPP;
	}

	if (dissector->used_keys &
	    ~(BIT_ULL(FLOW_DISSECTOR_KEY_META) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_CONTROL) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_BASIC) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_IPV4_ADDRS) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_IPV6_ADDRS) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_PORTS) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_TCP))) {
		dp->pae_last_stage = 2;
		dp->pae_last_err = -EOPNOTSUPP;
		dp->pae_replace_err++;
		return -EOPNOTSUPP;
	}

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_META) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_CONTROL) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_BASIC) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_PORTS)) {
		dp->pae_last_stage = 3;
		dp->pae_last_err = -EOPNOTSUPP;
		dp->pae_replace_err++;
		return -EOPNOTSUPP;
	}

	flow_rule_match_meta(rule, &meta);
	rcu_read_lock();
	in_dev = dev_get_by_index_rcu(dev_net(dp->ndev), meta.key->ingress_ifindex);
	ret = grx500_pae_port_map(dp, in_dev, &in_port_map);
	rcu_read_unlock();
	if (ret) {
		dp->pae_last_stage = 4;
		dp->pae_last_err = ret;
		dp->pae_replace_err++;
		return ret;
	}

	flow_rule_match_basic(rule, &basic);
	if (basic.key->ip_proto == IPPROTO_TCP)
		flow.ext_id = GRX500_PAE_RT_EXTID_TCP;
	else if (basic.key->ip_proto == IPPROTO_UDP)
		flow.ext_id = GRX500_PAE_RT_EXTID_UDP;
	else
		return -EOPNOTSUPP;

	flow_rule_match_control(rule, &ctrl);
	if (ctrl.key->addr_type == FLOW_DISSECTOR_KEY_IPV4_ADDRS) {
		struct flow_match_ipv4_addrs ip4;

		flow_rule_match_ipv4_addrs(rule, &ip4);
		flow.src_ip[0] = ntohl(ip4.key->src);
		flow.dst_ip[0] = ntohl(ip4.key->dst);
	} else if (ctrl.key->addr_type == FLOW_DISSECTOR_KEY_IPV6_ADDRS) {
		struct flow_match_ipv6_addrs ip6;

		flow_rule_match_ipv6_addrs(rule, &ip6);
		flow.is_ipv6 = true;
		for (i = 0; i < 4; i++) {
			flow.src_ip[i] = ntohl(ip6.key->src.s6_addr32[i]);
			flow.dst_ip[i] = ntohl(ip6.key->dst.s6_addr32[i]);
		}
	} else {
		return -EOPNOTSUPP;
	}

	flow_rule_match_ports(rule, &ports);
	flow.src_port = ntohs(ports.key->src);
	flow.dst_port = ntohs(ports.key->dst);

	flow_action_for_each(i, act, &rule->action) {
		switch (act->id) {
		case FLOW_ACTION_MANGLE:
			ret = grx500_pae_parse_mangle(act, &flow, eth_hdr,
						      eth_mask, &has_snat,
						      &has_dnat);
			if (ret) {
				dp->pae_last_stage = 5;
				dp->pae_last_err = ret;
				dp->pae_replace_err++;
				return ret;
			}
			break;
		case FLOW_ACTION_CSUM:
			break;
		case FLOW_ACTION_REDIRECT:
			out_dev = act->dev;
			break;
		default:
			dp->pae_last_stage = 6;
			dp->pae_last_err = -EOPNOTSUPP;
			dp->pae_replace_err++;
			return -EOPNOTSUPP;
		}
	}

	if (!out_dev || (has_snat && has_dnat)) {
		dp->pae_last_stage = 7;
		dp->pae_last_err = -EOPNOTSUPP;
		dp->pae_replace_err++;
		return -EOPNOTSUPP;
	}

	ret = grx500_pae_port_map(dp, out_dev, &out_port_map);
	if (ret) {
		dp->pae_last_stage = 8;
		dp->pae_last_err = ret;
		dp->pae_replace_err++;
		return ret;
	}

	flow.dst_port_map = out_port_map;
	flow.mtu = out_dev->mtu;
	flow.ttl_dec = true;

	if (memchr_inv(eth_mask, 0xff, ETH_ALEN) == NULL) {
		memcpy(flow.dst_mac, eth_hdr, ETH_ALEN);
		flow.use_dst_mac = true;
	}
	if (memchr_inv(eth_mask + ETH_ALEN, 0xff, ETH_ALEN) == NULL) {
		memcpy(flow.src_mac, eth_hdr + ETH_ALEN, ETH_ALEN);
		flow.use_src_mac = true;
	}

	if (has_snat || has_dnat) {
		flow.mode = flow.nat_port ? GRX500_PAE_MODE_NAPT :
					    GRX500_PAE_MODE_NAT;
		flow.dir = has_snat ? 1 : 0;
	} else {
		flow.mode = GRX500_PAE_MODE_ROUTING;
	}

	mutex_lock(&dp->pae_lock);
	entry = grx500_pae_flow_lookup(dp, f->cookie);
	if (entry) {
		grx500_pae_session_del(dp->pae, entry->session_idx);
		hash_del(&entry->hnode);
		kfree(entry);
	}

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		mutex_unlock(&dp->pae_lock);
		return -ENOMEM;
	}

	ret = grx500_pae_session_add(dp->pae, &flow, &sess_idx);
	if (ret) {
		dp->pae_last_stage = 9;
		dp->pae_last_err = ret;
		dp->pae_replace_err++;
		kfree(entry);
		mutex_unlock(&dp->pae_lock);
		return ret;
	}

	entry->cookie = f->cookie;
	entry->session_idx = sess_idx;
	hash_add(dp->pae_flows, &entry->hnode, entry->cookie);
	dp->pae_replace_ok++;
	mutex_unlock(&dp->pae_lock);
	return 0;
}

static int grx500_pae_cls_destroy(struct grx500_datapath *dp,
				  struct flow_cls_offload *f)
{
	struct grx500_pae_flow_entry *entry;
	int ret = 0;

	if (!dp->pae)
		return -EOPNOTSUPP;

	mutex_lock(&dp->pae_lock);
	entry = grx500_pae_flow_lookup(dp, f->cookie);
	if (entry) {
		ret = grx500_pae_session_del(dp->pae, entry->session_idx);
		hash_del(&entry->hnode);
		kfree(entry);
		dp->pae_del_cnt++;
	}
	mutex_unlock(&dp->pae_lock);
	return ret;
}

static int grx500_pae_cls_stats(struct grx500_datapath *dp,
				struct flow_cls_offload *f)
{
	struct grx500_pae_flow_entry *entry;
	bool hit = false;
	u32 packets = 0;
	int ret;

	if (!dp->pae)
		return -EOPNOTSUPP;

	mutex_lock(&dp->pae_lock);
	entry = grx500_pae_flow_lookup(dp, f->cookie);
	if (!entry) {
		mutex_unlock(&dp->pae_lock);
		return -ENOENT;
	}
	ret = grx500_pae_session_hit_stats(dp->pae, entry->session_idx,
					   true, &hit, &packets);
	mutex_unlock(&dp->pae_lock);
	dp->pae_stats_cnt++;
	if (ret)
		return ret;

	if (hit) {
		dp->pae_hit_cnt++;
		flow_stats_update(&f->stats, 0, packets ? packets : 1, 0,
				  jiffies, FLOW_ACTION_HW_STATS_DELAYED);
	}
	return 0;
}

static int grx500_ft_block_cb(enum tc_setup_type type, void *type_data,
			      void *cb_priv)
{
	struct grx500_datapath *dp = cb_priv;
	struct flow_cls_offload *f = type_data;

	if (type != TC_SETUP_CLSFLOWER)
		return -EOPNOTSUPP;

	switch (f->command) {
	case FLOW_CLS_REPLACE:
		return grx500_pae_cls_replace(dp, f);
	case FLOW_CLS_DESTROY:
		return grx500_pae_cls_destroy(dp, f);
	case FLOW_CLS_STATS:
		return grx500_pae_cls_stats(dp, f);
	default:
		return -EOPNOTSUPP;
	}
}

static int grx500_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data)
{
	struct grx500_datapath *dp = grx500_conduit_priv(ndev);
	struct flow_block_offload *f = type_data;
	struct flow_block_cb *block_cb;

	if (type != TC_SETUP_FT)
		return -EOPNOTSUPP;

	if (f->binder_type != FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		return -EOPNOTSUPP;

	switch (f->command) {
	case FLOW_BLOCK_BIND:
		dp->pae_bind_cnt++;
		block_cb = flow_block_cb_lookup(f->block, grx500_ft_block_cb, dp);
		if (block_cb) {
			flow_block_cb_incref(block_cb);
			return 0;
		}
		block_cb = flow_block_cb_alloc(grx500_ft_block_cb, dp, dp, NULL);
		if (IS_ERR(block_cb))
			return PTR_ERR(block_cb);
		flow_block_cb_incref(block_cb);
		flow_block_cb_add(block_cb, f);
		return 0;
	case FLOW_BLOCK_UNBIND:
		block_cb = flow_block_cb_lookup(f->block, grx500_ft_block_cb, dp);
		if (!block_cb)
			return 0;
		if (!flow_block_cb_decref(block_cb))
			flow_block_cb_remove(block_cb, f);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops grx500_ops = {
	.ndo_open = grx500_open,
	.ndo_stop = grx500_stop,
	.ndo_start_xmit = grx500_xmit,
	.ndo_tx_timeout = grx500_tx_timeout,
	.ndo_get_stats64 = grx500_get_stats64,
	.ndo_change_mtu = grx500_change_mtu,
	.ndo_set_rx_mode = grx500_set_rx_mode,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_setup_tc = grx500_setup_tc,
};

static void grx500_get_drvinfo(struct net_device *ndev,
			       struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "grx500-datapath", sizeof(info->driver));
	strscpy(info->bus_info, dev_name(ndev->dev.parent), sizeof(info->bus_info));
}

static const struct ethtool_ops grx500_ethtool_ops = {
	.get_drvinfo		= grx500_get_drvinfo,
	.get_link		= ethtool_op_get_link,
};

static int grx500_read_lan_routes(struct platform_device *pdev,
				  struct grx500_datapath *dp)
{
	struct device_node *np = pdev->dev.of_node;
	u32 cpu_route[2];
	int i, ret;

	/* Both tables are copied from the vendor GRX350 axepoint reference DTS.
	 * The AX50 connector/PHY map was verified separately on hardware.
	 */
	if (of_property_count_u32_elems(np, "lantiq,lan-eqm-routes") !=
	    ARRAY_SIZE(dp->lan_eqm) * 5 ||
	    of_property_count_u32_elems(np, "lantiq,lan-dqm-routes") !=
	    ARRAY_SIZE(dp->lan_dqm) * 5 ||
	    of_property_count_u32_elems(np, "lantiq,gswr-eqm-route") != 5 ||
	    of_property_count_u32_elems(np, "lantiq,wan-dqm-route") != 5 ||
	    of_property_count_u32_elems(np, "lantiq,cpu-dqm-port") != 2)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "CBM route table has wrong tuple count\n");
	ret = of_property_read_u32_array(np, "lantiq,lan-eqm-routes",
					 (u32 *)dp->lan_eqm,
					 ARRAY_SIZE(dp->lan_eqm) * 5);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "LAN CBM enqueue routing is missing\n");
	ret = of_property_read_u32_array(np, "lantiq,lan-dqm-routes",
					 (u32 *)dp->lan_dqm,
					 ARRAY_SIZE(dp->lan_dqm) * 5);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "LAN CBM dequeue routing is missing\n");
	ret = of_property_read_u32_array(np, "lantiq,cpu-dqm-port",
					 cpu_route, ARRAY_SIZE(cpu_route));
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "CPU CBM dequeue route is missing\n");
	ret = of_property_read_u32_array(np, "lantiq,gswr-eqm-route",
					 (u32 *)&dp->gswr_eqm, 5);
	if (ret)
		return ret;
	if (dp->gswr_eqm.port != 7 || dp->gswr_eqm.dma_controller != 1 ||
	    dp->gswr_eqm.dma_channel != 0 || dp->gswr_eqm.jumbo != 0 ||
	    dp->gswr_eqm.descriptors != 2)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "GSWIP-R CBM enqueue route is invalid\n");
	ret = of_property_read_u32_array(np, "lantiq,wan-dqm-route",
					 (u32 *)&dp->wan_dqm, 5);
	if (ret)
		return ret;
	if (dp->wan_dqm.port != 19 || dp->wan_dqm.tmu_queue != 28 ||
	    dp->wan_dqm.dma_controller != 1 || dp->wan_dqm.dma_channel != 15 ||
	    dp->wan_dqm.descriptors != 2)
		return dev_err_probe(&pdev->dev, -EINVAL, "WAN CBM dequeue route is invalid\n");
	for (i = 0; i < GRX500_LAN_EQM_ROUTES; i++) {
		const struct grx500_eqm_route *r = &dp->lan_eqm[i];

		if (r->port != 5 + i / 2 || r->dma_controller != 2 ||
		    r->jumbo != i % 2 || r->descriptors != 2 ||
		    r->dma_channel >= 32)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "LAN CBM enqueue route is invalid\n");
	}
	for (i = 0; i < GRX500_LAN_DQM_ROUTES; i++) {
		const struct grx500_dqm_route *r = &dp->lan_dqm[i];

		if (r->port != i + 6 || r->tmu_queue != i + 16 ||
		    r->dma_controller != 2 || r->dma_channel >= 32 ||
		    r->descriptors != 2)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "LAN CBM dequeue route is invalid\n");
	}
	if (cpu_route[0] != GRX500_RX_LS_PORT || cpu_route[1] != 35)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "CPU CBM dequeue route is invalid\n");
	dp->cpu_dqm_port = cpu_route[0];
	dp->cpu_dqm_queue = cpu_route[1];
	return 0;
}

static int grx500_find_pools(struct platform_device *pdev,
			     struct grx500_datapath *dp)
{
	phys_addr_t kernel_start, kernel_end;
	int i;

	grx500_kernel_phys_range(&kernel_start, &kernel_end);

	for (i = 0; i < GRX500_POOL_COUNT; i++) {
		struct device_node *node;
		struct reserved_mem *rmem;
		phys_addr_t last;

		node = of_parse_phandle(pdev->dev.of_node, "memory-region", i);
		if (!node)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "CBM buffer pool is missing\n");
		rmem = of_reserved_mem_lookup(node);
		if (!rmem) {
			dev_err(&pdev->dev, "CBM pool%d reserved-memory lookup failed for %pOF\n",
				i, node);
			of_node_put(node);
			return -EINVAL;
		}
		of_node_put(node);
		if (rmem->size != grx500_pool_size[i] ||
		    !IS_ALIGNED(rmem->base, grx500_pool_alignment[i]))
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "CBM pool%d base=%pa size=%pa expected-size=%pa alignment=%pa\n",
					     i, &rmem->base, &rmem->size,
					     &grx500_pool_size[i], &grx500_pool_alignment[i]);
		last = rmem->base + rmem->size - 1;
		/* AX50 DDR begins at 0x20000000. The CBM base registers and
		 * packet descriptors are 32-bit, so reject any out-of-range pool.
		 */
		if (rmem->base < 0x20000000 || last > U32_MAX ||
		    last < rmem->base)
			return dev_err_probe(&pdev->dev, -ERANGE,
					     "CBM buffer pool is outside 32-bit DDR\n");
		/* A prior DT allocation overlapped the kernel and RX wrote ARP
		 * bytes into setup_frame(). Reject it before mapping or DMA, even
		 * if a future boot-memory regression defeats the early reservation.
		 * The prefix below _text contains the VMB/IBL firmware window.
		 */
		if (rmem->base < kernel_end) {
			dev_err(&pdev->dev,
				"CBM pool%d %pa..%pa overlaps kernel/firmware below %pa (kernel starts %pa)\n",
				i, &rmem->base, &last, &kernel_end, &kernel_start);
			return -ERANGE;
		}
		if (i && rmem->base < dp->pool_phys[0] + grx500_pool_size[0] &&
		    last >= dp->pool_phys[0])
			return dev_err_probe(&pdev->dev, -ERANGE,
					     "CBM standard and jumbo pools overlap\n");
		dp->pool_phys[i] = rmem->base;
		dev_info(&pdev->dev, "CBM pool%d validated: %pa..%pa kernel_end=%pa\n",
			 i, &rmem->base, &last, &kernel_end);
	}
	return 0;
}

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct grx500_datapath *dp = dev_get_drvdata(dev);
	u32 tmu, cbm, eqm, dqm, igp0, egp2, eqm_pocc, ls_status, ls_irq;
	u32 eqm_packets, q20_enq, q20_deq, q35_enq, q35_deq;
	u32 desc0, desc1, desc2, desc3;
	size_t len;
	int p2p_ready = 0, i;

	/* Reading these configuration/status words has no enqueue/dequeue side
	 * effect. No CBM/TMU reset or queue ownership is attempted here.
	 */
	tmu = __raw_readl(dp->base[GRX500_TMU] + GRX500_TMU_CTRL);
	cbm = __raw_readl(dp->base[GRX500_CBM] + GRX500_CBM_CTRL);
	eqm = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CTRL);
	dqm = __raw_readl(dp->base[GRX500_DQM] + GRX500_DQM_CTRL);
	igp0 = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_IGP0);
	egp2 = __raw_readl(dp->base[GRX500_DQM] + GRX500_DQM_CPU_EGP2);
	eqm_pocc = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_POCC);
	eqm_packets = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_EQPC);
	q20_enq = __raw_readl(dp->base[GRX500_QEQCNT] + 20 * sizeof(u32));
	q20_deq = __raw_readl(dp->base[GRX500_QDQCNT] + 20 * sizeof(u32));
	q35_enq = __raw_readl(dp->base[GRX500_QEQCNT] + 35 * sizeof(u32));
	q35_deq = __raw_readl(dp->base[GRX500_QDQCNT] + 35 * sizeof(u32));
	desc0 = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC);
	desc1 = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC + 4);
	desc2 = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC + 8);
	desc3 = __raw_readl(dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC + 12);
	ls_status = __raw_readl(dp->base[GRX500_LS] +
		GRX500_RX_LS_PORT * GRX500_LS_PORT_STRIDE + GRX500_LS_PORT_STATUS);
	ls_irq = __raw_readl(dp->base[GRX500_LS] + GRX500_LS_IRNCR);
	for (i = 0; i < GRX500_LAN_P2P_CLASSES; i++)
		p2p_ready += dp->p2p[i].tx_requested &&
			     dp->p2p[i].rx_requested &&
			     dp->p2p[i].buffers_allocated;

	len = sysfs_emit(buf,
		"tmu_ctrl=%08x active=%u\n"
		"cbm_ctrl=%08x active=%u\n"
		"eqm_ctrl=%08x enabled=%u\n"
		"dqm_ctrl=%08x enabled=%u\n"
		"cpu_igp0_cfg=%08x cpu_egp2_cfg=%08x\n"
		"eqm_cpu_pocc=%08x ls_port_status=%08x ls_irq_status=%08x\n"
		"eqm_cpu_enqueued=%u q20_enq=%u q20_deq=%u q35_enq=%u q35_deq=%u\n"
		"eqm_last_desc=%08x:%08x:%08x:%08x\n"
		"p2p_dma2rx_to_dma1tx=0-3 ready=%d/4\n"
		"std_pool=%pa jumbo_pool=%pa cpu_dqm=%u:%u\n"
		"cbm_sba0=%08x sba1=%08x jba0=%08x jba1=%08x\n"
		"lan_eqm=%u/%u lan_dqm=%u-%u queues=%u-%u\n"
		"eqm_irq=%d cpu_dqm_irqs=%d,%d,%d,%d (shared after first port activation)\n",
		tmu, !!(tmu & 1), cbm, !!(cbm & 1),
		eqm, !!(eqm & 1), dqm, !!(dqm & 1), igp0, egp2,
		eqm_pocc, ls_status, ls_irq,
		eqm_packets, q20_enq, q20_deq, q35_enq, q35_deq,
		desc0, desc1, desc2, desc3, p2p_ready,
		&dp->pool_phys[GRX500_STANDARD_POOL],
		&dp->pool_phys[GRX500_JUMBO_POOL],
		dp->cpu_dqm_port, dp->cpu_dqm_queue,
		__raw_readl(dp->base[GRX500_CBM] + GRX500_CBM_SBA0),
		__raw_readl(dp->base[GRX500_CBM] + GRX500_CBM_SBA1),
		__raw_readl(dp->base[GRX500_CBM] + GRX500_CBM_JBA0),
		__raw_readl(dp->base[GRX500_CBM] + GRX500_CBM_JBA1),
		dp->lan_eqm[0].port, dp->lan_eqm[2].port,
		dp->lan_dqm[0].port,
		dp->lan_dqm[GRX500_LAN_DQM_ROUTES - 1].port,
		dp->lan_dqm[0].tmu_queue,
		dp->lan_dqm[GRX500_LAN_DQM_ROUTES - 1].tmu_queue,
			dp->irq[0], dp->irq[1], dp->irq[2], dp->irq[3], dp->irq[4]);
	len += sysfs_emit_at(buf, len,
		"rx_ls_port=%u rx_irq=%d cbm_line=%u ls_global=%08x spread=%08x irq_enable=%08x cbm_line_status=%08x enable=%08x\n",
		GRX500_RX_LS_PORT, dp->irq[GRX500_RX_IRQ_INDEX], GRX500_RX_CBM_LINE,
		__raw_readl(dp->base[GRX500_LS] + GRX500_LS_GLOBAL_CTRL),
		__raw_readl(dp->base[GRX500_LS] + GRX500_LS_SPREAD_CTRL),
		__raw_readl(dp->base[GRX500_LS] + GRX500_LS_IRNEN),
		__raw_readl(dp->base[GRX500_CBM] + GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE),
		__raw_readl(dp->base[GRX500_CBM] + GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE + 8));
	for (i = 0; i < GRX500_CBM_CPU_PORTS; i++) {
		void __iomem *ls = dp->base[GRX500_LS] + i * GRX500_LS_PORT_STRIDE;
		void __iomem *egp = dp->base[GRX500_DQM] + GRX500_DQM_CPU_CFG +
				   i * GRX500_CBM_CPU_PORT_STRIDE;

		len += sysfs_emit_at(buf, len,
			"ls%d_ctrl=%08x status=%08x dqm%d_cfg=%08x dequeued=%u irq=%08x\n",
			i, __raw_readl(ls + GRX500_LS_PORT_CTRL),
			__raw_readl(ls + GRX500_LS_PORT_STATUS), i,
			__raw_readl(egp), __raw_readl(egp + 4), __raw_readl(egp + 0x20));
	}
	/* DQM descriptors are released by a DW3 write in vendor cbm_dq_deq_list.
	 * These reads do not acknowledge ownership. Never sample LS dequeue
	 * registers here: reading their DW3 would consume a packet behind NAPI.
	 */
	for (i = 0; i < 2; i++) {
		void __iomem *desc = dp->base[GRX500_DQM] + GRX500_DQM_CPU_DESC +
				    dp->cpu_dqm_port * GRX500_CBM_CPU_PORT_STRIDE + i * 16;

		len += sysfs_emit_at(buf, len, "dqm%u_desc%d_live=%08x:%08x:%08x:%08x\n",
			dp->cpu_dqm_port, i, __raw_readl(desc), __raw_readl(desc + 4),
			__raw_readl(desc + 8), __raw_readl(desc + 12));
	}
	/* Live MMIO samples, not atomic descriptor snapshots. EQM7 activity
	 * separates a blocked GSWIP-R sink from later QIDT/Load Spreader faults.
	 */
	for (i = 0; i < 2; i++) {
		const struct grx500_eqm_route *r = i ? &dp->gswr_eqm : &dp->lan_eqm[0];
		void __iomem *cfg = dp->base[GRX500_EQM] + GRX500_EQM_DMA_CFG +
				   (r->port - 5) * 0x1000;
		void __iomem *ring = dp->base[GRX500_DMA_DESC] +
				    (r->port - 5) * 0x1000;
		int d;

		len += sysfs_emit_at(buf, len,
			"eqm%u_dma%urx%u cfg=%08x pocc=%08x enqueued=%u\n",
			r->port, r->dma_controller, r->dma_channel,
			__raw_readl(cfg), __raw_readl(cfg + 8), __raw_readl(cfg + 12));
		for (d = 0; d < r->descriptors; d++)
			len += sysfs_emit_at(buf, len,
				"eqm%u_desc%d_live=%08x:%08x:%08x:%08x\n",
				r->port, d, __raw_readl(ring + d * 16),
				__raw_readl(ring + d * 16 + 4),
				__raw_readl(ring + d * 16 + 8),
				__raw_readl(ring + d * 16 + 12));
	}
	len += sysfs_emit_at(buf, len,
		"rx_bad_descriptor=%u\n"
		"pae_bind=%u replace_ok=%u replace_err=%u del=%u stats=%u hit=%u last_stage=%u last_err=%d\n",
		READ_ONCE(dp->rx_bad_descriptor),
		READ_ONCE(dp->pae_bind_cnt),
		READ_ONCE(dp->pae_replace_ok),
		READ_ONCE(dp->pae_replace_err),
		READ_ONCE(dp->pae_del_cnt),
		READ_ONCE(dp->pae_stats_cnt),
		READ_ONCE(dp->pae_hit_cnt),
		READ_ONCE(dp->pae_last_stage),
		READ_ONCE(dp->pae_last_err));
	if (dp->pae) {
		u32 gctrl0 = 0, gctrl1 = 0, tfcr_num = 0;
		u16 k12 = 0, v9 = 0;
		bool vld = false;

		grx500_pae_debug_regs(dp->pae, &gctrl0, &gctrl1, &tfcr_num,
				      &k12, &v9, &vld);
		len += sysfs_emit_at(buf, len,
			"pae_regs gctrl0=%04x gctrl1=%04x tfcr_num=%04x tfcr0_k12=%04x tfcr0_v9=%04x tfcr0_vld=%u\n",
			gctrl0, gctrl1, tfcr_num, k12, v9, vld);
	}
	return len;
}
static DEVICE_ATTR_RO(status);

static void grx500_dcdp_set_cbm_wlan_ep(struct grx500_datapath *dp,
					u32 ep_id, u32 tmu_port)
{
	u32 val, mask, shift;

	switch (tmu_port) {
	case 4:
		mask = 0x00f00000U;
		shift = 20;
		break;
	case 24:
		mask = 0x0000f000U;
		shift = 12;
		break;
	case 25:
		mask = 0x00000f00U;
		shift = 8;
		break;
	case 26:
		mask = 0x000000f0U;
		shift = 4;
		break;
	default:
		return;
	}

	val = __raw_readl(dp->base[GRX500_CBM] + GRX500_CBM_CTRL);
	val = (val & ~mask) | ((ep_id << shift) & mask);
	__raw_writel(val, dp->base[GRX500_CBM] + GRX500_CBM_CTRL);
}

static void grx500_dcdp_set_qidt_ep(struct grx500_datapath *dp,
				    u32 ep_id, u32 queue)
{
	void __iomem *qidt = dp->base[GRX500_QIDT];
	u32 key;

	for (key = 0; key < 0x4000; key++) {
		u32 word, shift, value;

		if (((key >> 4) & 0xf) != ep_id)
			continue;
		word = key >> 2;
		shift = (key & 3) * 8;
		value = __raw_readl(qidt + word * sizeof(u32));
		value &= ~(0xffu << shift);
		value |= (queue & 0xffu) << shift;
		__raw_writel(value, qidt + word * sizeof(u32));
	}
}

int grx500_dcdp_register_radio(u32 ep_id, struct net_device *dev,
			       u32 umt_dst_phys, u32 rx_ring_entries,
			       struct grx500_dcdp_port **out_port)
{
	struct grx500_datapath *dp = READ_ONCE(g_grx500_dp);
	struct grx500_dcdp_port *port;
	u32 slot, cbm_pid, tmu_port, tmu_queue;
	size_t rx_bytes;
	int ret;

	if (!dp || !dev || !umt_dst_phys || !rx_ring_entries || !out_port)
		return -EINVAL;

	if (ep_id == GRX500_DCDP_EP_WLAN0) {
		slot = 0;
		cbm_pid = 0;
		tmu_port = 4;
		tmu_queue = 36;
	} else if (ep_id == GRX500_DCDP_EP_WLAN1) {
		slot = 1;
		cbm_pid = 1;
		tmu_port = 24;
		tmu_queue = 37;
	} else {
		return -EINVAL;
	}

	mutex_lock(&dp->dcdp_lock);
	port = &dp->dcdp_ports[slot];
	if (port->active) {
		mutex_unlock(&dp->dcdp_lock);
		return -EBUSY;
	}
	memset(port, 0, sizeof(*port));
	port->ep_id = ep_id;
	port->cbm_pid = cbm_pid;
	port->tmu_port = tmu_port;
	port->tmu_queue = tmu_queue;
	port->dev = dev;

	ret = grx500_umt_request(ep_id, cbm_pid, &port->dma_ctrl_id,
				 &port->dma_cid, &port->umt_id);
	if (ret)
		goto err_unlock;

	if (dp->gswip[1]) {
		ret = grx500_gswip_configure_wlan_port(dp->gswip[1], ep_id,
						       port->dma_cid);
		if (ret)
			goto err_umt_release;
	}

	grx500_dcdp_set_cbm_wlan_ep(dp, ep_id, tmu_port);
	__raw_writel(0x107U | ((tmu_port & 0x1fU) << 16),
		     dp->base[GRX500_DQM] + GRX500_DQM_CPU_CFG +
		     tmu_port * GRX500_CBM_CPU_PORT_STRIDE);

	ret = grx500_tmu_setup_flat_path(dp, tmu_port, tmu_queue);
	if (ret)
		goto err_cbm_clear;
	grx500_dcdp_set_qidt_ep(dp, ep_id, tmu_queue);

	rx_bytes = (size_t)rx_ring_entries * GRX500_DCDP_RING_DESC_SIZE;
	port->rx_out.virt = dma_alloc_coherent(dp->dev, rx_bytes,
					       &port->rx_out.dma, GFP_KERNEL);
	if (!port->rx_out.virt) {
		ret = -ENOMEM;
		goto err_qidt_clear;
	}
	port->rx_out.phys = (phys_addr_t)port->rx_out.dma;
	port->rx_out.entries = rx_ring_entries;
	port->rx_out.entry_size = GRX500_DCDP_RING_DESC_SIZE;

	port->tx_in.phys = grx500_layout[GRX500_DQM].start +
			   GRX500_DQM_CPU_DESC +
			   tmu_port * GRX500_CBM_CPU_PORT_STRIDE;
	port->tx_in.virt = dp->base[GRX500_DQM] + GRX500_DQM_CPU_DESC +
			   tmu_port * GRX500_CBM_CPU_PORT_STRIDE;
	port->tx_in.entries = 32;
	port->tx_in.entry_size = GRX500_DCDP_RING_DESC_SIZE;

	port->tx_out.phys = grx500_layout[GRX500_DQM].start +
			    GRX500_DQM_CPU_PTR_RTN +
			    tmu_port * GRX500_CBM_CPU_PORT_STRIDE;
	port->tx_out.virt = dp->base[GRX500_DQM] + GRX500_DQM_CPU_PTR_RTN +
			    tmu_port * GRX500_CBM_CPU_PORT_STRIDE;
	port->tx_out.entries = 32;
	port->tx_out.entry_size = GRX500_DCDP_TXOUT_DESC_SIZE;

	ret = grx500_umt_set_mode(port->umt_id, ep_id, 0, 3, umt_dst_phys,
				  GRX500_DCDP_DEFAULT_UMT_US, true);
	if (ret)
		goto err_free_rx;
	ret = grx500_umt_enable(port->umt_id, ep_id, true);
	if (ret)
		goto err_free_rx;

	port->active = true;
	*out_port = port;
	mutex_unlock(&dp->dcdp_lock);

	dev_info(dp->dev,
		 "DCDP radio registered: ep=%u cbm_pid=%u tmu_port=%u q=%u umt=%u dma_cid=%u tx_in=%pa tx_out=%pa rx_out=%pa\n",
		 port->ep_id, port->cbm_pid, port->tmu_port, port->tmu_queue,
		 port->umt_id, port->dma_cid, &port->tx_in.phys,
		 &port->tx_out.phys, &port->rx_out.phys);
	return 0;

err_free_rx:
	grx500_umt_enable(port->umt_id, ep_id, false);
	dma_free_coherent(dp->dev, rx_bytes, port->rx_out.virt,
			  port->rx_out.dma);
err_qidt_clear:
	grx500_dcdp_set_qidt_ep(dp, ep_id, 255);
err_cbm_clear:
	grx500_dcdp_set_cbm_wlan_ep(dp, 0, tmu_port);
err_umt_release:
	grx500_umt_release(port->umt_id, ep_id);
err_unlock:
	memset(port, 0, sizeof(*port));
	mutex_unlock(&dp->dcdp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(grx500_dcdp_register_radio);

void grx500_dcdp_unregister_radio(struct grx500_dcdp_port *port)
{
	struct grx500_datapath *dp = READ_ONCE(g_grx500_dp);
	size_t rx_bytes;

	if (!dp || !port)
		return;

	mutex_lock(&dp->dcdp_lock);
	if (!port->active) {
		mutex_unlock(&dp->dcdp_lock);
		return;
	}

	grx500_umt_enable(port->umt_id, port->ep_id, false);
	grx500_dcdp_set_qidt_ep(dp, port->ep_id, 255);
	grx500_dcdp_set_cbm_wlan_ep(dp, 0, port->tmu_port);
	grx500_umt_release(port->umt_id, port->ep_id);
	if (port->rx_out.virt) {
		rx_bytes = (size_t)port->rx_out.entries * port->rx_out.entry_size;
		dma_free_coherent(dp->dev, rx_bytes, port->rx_out.virt,
				  port->rx_out.dma);
	}
	memset(port, 0, sizeof(*port));
	mutex_unlock(&dp->dcdp_lock);
}
EXPORT_SYMBOL_GPL(grx500_dcdp_unregister_radio);

int grx500_dcdp_xmit(struct grx500_dcdp_port *port, u16 subif_id,
		     struct sk_buff *skb)
{
	struct grx500_datapath *dp = READ_ONCE(g_grx500_dp);
	void __iomem *desc;
	unsigned long irqflags;
	phys_addr_t frame;
	u32 addr, len, alloc_ptr = 0;
	u8 *data;
	int ret;

	if (!dp || !port || !port->active || !skb)
		return -EINVAL;
	if (!dp->started)
		return -ENETDOWN;
	if (skb->len > 1536) {
		dev_kfree_skb_any(skb);
		return -EMSGSIZE;
	}

	spin_lock_irqsave(&dp->tx_lock, irqflags);
	ret = grx500_cbm_cpu_alloc_standard(dp, 0, &frame, &alloc_ptr);
	spin_unlock_irqrestore(&dp->tx_lock, irqflags);
	if (ret)
		return -EBUSY;

	addr = frame + GRX500_CBM_FRAME_OFFSET;
	len = max_t(u32, skb->len, ETH_ZLEN);
	data = phys_to_virt(addr);
	ret = skb_copy_bits(skb, 0, data, skb->len);
	if (unlikely(ret)) {
		spin_lock_irqsave(&dp->tx_lock, irqflags);
		grx500_cbm_cpu_return(dp, 0, frame);
		spin_unlock_irqrestore(&dp->tx_lock, irqflags);
		dev_kfree_skb_any(skb);
		return ret;
	}
	if (skb->len < ETH_ZLEN)
		memset(data + skb->len, 0, ETH_ZLEN - skb->len);
	dma_sync_single_for_device(dp->dev, frame, 2048, DMA_BIDIRECTIONAL);

	desc = dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC;
	spin_lock_irqsave(&dp->tx_lock, irqflags);
	__raw_writel((u32)subif_id, desc);
	__raw_writel(port->ep_id << GRX500_TX_DW1_EP_SHIFT, desc + 4);
	__raw_writel(addr & ~7u, desc + 8);
	wmb();
	__raw_writel(BIT(29) | BIT(28) | ((addr & 7) << 23) | len,
		     desc + 12);
	spin_unlock_irqrestore(&dp->tx_lock, irqflags);

	dev_consume_skb_any(skb);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_dcdp_xmit);

static int grx500_datapath_probe(struct platform_device *pdev)
{
	struct grx500_datapath *dp;
	struct net_device *ndev;
	struct resource *res;
	u8 mac[ETH_ALEN];
	int i, ret;

	dp = devm_kzalloc(&pdev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;
	dp->dev = &pdev->dev;
	spin_lock_init(&dp->tmu_lock);
	spin_lock_init(&dp->tx_lock);
	mutex_init(&dp->pae_lock);
	mutex_init(&dp->dcdp_lock);
	hash_init(dp->pae_flows);
	u64_stats_init(&dp->rx_syncp);
	u64_stats_init(&dp->tx_syncp);
	atomic_long_set(&dp->tx_timeout_dropped, 0);
	timer_setup(&dp->tx_wake_timer, grx500_tx_wake_timer_cb, 0);
	for (i = 0; i < GRX500_RESOURCE_COUNT; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res || res->start != grx500_layout[i].start ||
		    resource_size(res) != grx500_layout[i].size)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "CBM/TMU MMIO layout differs from vendor DTS\n");
		dp->base[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(dp->base[i]))
			return PTR_ERR(dp->base[i]);
	}
	ret = grx500_find_pools(pdev, dp);
	if (ret)
		return ret;
	ret = grx500_read_lan_routes(pdev, dp);
	if (ret)
		return ret;
	ret = grx500_cbm_find_switch_buses(pdev, dp);
	if (ret)
		return ret;
	dp->pae = grx500_pae_init(dp->gswip[1]);
	if (IS_ERR(dp->pae)) {
		dev_warn(&pdev->dev, "PAE L3 offload disabled: %ld\n",
			 PTR_ERR(dp->pae));
		dp->pae = NULL;
	}
	for (i = 0; i < GRX500_CBM_IRQS; i++) {
		dp->irq[i] = platform_get_irq(pdev, i);
		if (dp->irq[i] < 0)
			return dev_err_probe(&pdev->dev, dp->irq[i],
					     "CBM IRQ is missing\n");
	}
	platform_set_drvdata(pdev, dp);
	ret = device_create_file(&pdev->dev, &dev_attr_status);
	if (ret)
		return ret;

	ndev = alloc_etherdev(sizeof(struct grx500_datapath *));
	if (!ndev) {
		ret = -ENOMEM;
		goto err_sysfs;
	}
	*(struct grx500_datapath **)netdev_priv(ndev) = dp;
	dp->ndev = ndev;
	strscpy(ndev->name, "eth0", IFNAMSIZ);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &grx500_ops;
	ndev->ethtool_ops = &grx500_ethtool_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = 1536;
	ndev->features &= ~(NETIF_F_SG | NETIF_F_HW_CSUM | NETIF_F_TSO);
	if (!of_get_mac_address(pdev->dev.of_node, mac))
		eth_hw_addr_set(ndev, mac);
	else
		eth_hw_addr_random(ndev);
	netif_carrier_off(ndev);
	netif_napi_add(ndev, &dp->napi, grx500_poll);

	rtnl_lock();
	ret = register_netdevice(ndev);
	rtnl_unlock();
	if (ret)
		goto err_register;

	WRITE_ONCE(g_grx500_dp, dp);
	dev_info(&pdev->dev, "AX50 CPU conduit eth0 registered; shared CBM/TMU starts on open\n");
	return 0;

err_register:
	netif_napi_del(&dp->napi);
	free_netdev(ndev);
err_sysfs:
	device_remove_file(&pdev->dev, &dev_attr_status);
	return ret;
}

static void grx500_datapath_remove(struct platform_device *pdev)
{
	struct grx500_datapath *dp = platform_get_drvdata(pdev);
	struct grx500_pae_flow_entry *entry;
	struct hlist_node *tmp;
	int bkt, i;

	for (i = 0; i < GRX500_DCDP_MAX_RADIOS; i++) {
		if (dp->dcdp_ports[i].active)
			grx500_dcdp_unregister_radio(&dp->dcdp_ports[i]);
	}
	WRITE_ONCE(g_grx500_dp, NULL);

	/* Stop the conduit before tearing down shared CBM/TMU and NAPI. */
	rtnl_lock();
	if (netif_running(dp->ndev))
		dev_close(dp->ndev);
	grx500_shutdown(dp);
	unregister_netdevice(dp->ndev);
	rtnl_unlock();
	mutex_lock(&dp->pae_lock);
	hash_for_each_safe(dp->pae_flows, bkt, tmp, entry, hnode) {
		hash_del(&entry->hnode);
		kfree(entry);
	}
	mutex_unlock(&dp->pae_lock);
	grx500_pae_destroy(dp->pae);
	netif_napi_del(&dp->napi);
	free_netdev(dp->ndev);
	device_remove_file(&pdev->dev, &dev_attr_status);
}

static const struct of_device_id grx500_datapath_of_match[] = {
	{ .compatible = "lantiq,grx500-datapath" },
	{ }
};
MODULE_DEVICE_TABLE(of, grx500_datapath_of_match);

static struct platform_driver grx500_datapath_driver = {
	.probe = grx500_datapath_probe,
	.remove = grx500_datapath_remove,
	.driver = {
		.name = "grx500-datapath",
		.of_match_table = grx500_datapath_of_match,
	},
};
module_platform_driver(grx500_datapath_driver);

MODULE_DESCRIPTION("GRX500 CBM/TMU packet-path bring-up");
MODULE_LICENSE("GPL");
