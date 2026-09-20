// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 CBM/TMU CPU packet path for the AX50 LAN1 bring-up. */
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/grx500-gswip.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_reserved_mem.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <asm/sections.h>
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
#define GRX500_LAN1_ENDPOINT	5
/* Vendor dma_tx_desc_1 (big-endian): EP occupies DW1 bits 11:8. */
#define GRX500_TX_DW1_EP_SHIFT	8
#define GRX500_LAN1_GSWIP_PORT	5
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

struct grx500_datapath {
	struct device *dev;
	struct net_device *ndev;
	struct napi_struct napi;
	struct phy_device *phy;
	dma_addr_t pool_dma[GRX500_POOL_COUNT];
	bool pool_mapped[GRX500_POOL_COUNT];
	bool started;
	bool poisoned;
	spinlock_t tx_lock;
	void __iomem *base[GRX500_RESOURCE_COUNT];
	spinlock_t tmu_lock;
	phys_addr_t pool_phys[GRX500_POOL_COUNT];
	int irq[GRX500_CBM_IRQS];
	struct grx500_eqm_route lan_eqm[GRX500_LAN_EQM_ROUTES];
	struct grx500_eqm_route gswr_eqm;
	struct grx500_dqm_route lan_dqm[GRX500_LAN_DQM_ROUTES];
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
};

/* netdev_priv stores a pointer to the devm-managed datapath object, not
 * the object itself. Keep every netdevice callback on the same accessor.
 */
static struct grx500_datapath *grx500_lan1_dp(struct net_device *ndev)
{
	return *(struct grx500_datapath **)netdev_priv(ndev);
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

static int grx500_tmu_create_lan_paths(struct grx500_datapath *dp)
{
	int i, ret;

	/* One flat scheduler block and queue per CBM LAN DQM route, matching
	 * the vendor's queue 16..27 and egress port 6..17 layout. TMU ACT must
	 * stay clear until CBM dequeue, DMA descriptors and IRQs are ready.
	 */
	for (i = 0; i <= GRX500_LAN_DQM_ROUTES; i++) {
		u32 port = i == GRX500_LAN_DQM_ROUTES ?
			   dp->cpu_dqm_port : dp->lan_dqm[i].port;
		u32 queue = i == GRX500_LAN_DQM_ROUTES ?
			    dp->cpu_dqm_queue : dp->lan_dqm[i].tmu_queue;
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
		ret = grx500_tmu_table_write_words(dp, GRX500_TMU_QTHT,
						   GRX500_TMU_QMTC,
						   GRX500_TMU_QMTC_QTW,
						   GRX500_TMU_QMTC_QTV,
						   queue,
						   threshold,
						   ARRAY_SIZE(threshold));
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

		if (endpoint < 1 || endpoint > 6)
			continue;
		word = key >> 2;
		shift = (key & 3) * 8;
		value = __raw_readl(qidt + word * sizeof(u32));
		value &= ~(0xffu << shift);
		value |= dp->lan_dqm[endpoint - 1].tmu_queue << shift;
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
						 u32 cpu, phys_addr_t *frame)
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
		ret = grx500_cbm_cpu_alloc_standard(dp, 0, &frame);
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

static int grx500_cbm_prepare_lan1_descriptors(struct grx500_datapath *dp)
{
	const struct grx500_dqm_route *dqm = &dp->lan_dqm[GRX500_LAN1_ENDPOINT - 1];
	void __iomem *desc = dp->base[GRX500_DMA_DESC];
	int i, ret;

	ret = grx500_cbm_prepare_eqm_descriptors(dp, &dp->lan_eqm[0]);
	if (ret)
		return ret;
	/* Hypothesis from the ARP-in-P2P hardware result: GSWIP-R cannot
	 * drain to CPU because DMA1RX0/EQM7 was absent. The vendor axepoint
	 * EQ7 tuple supplies this final receive leg, independent of PPA.
	 */
	ret = grx500_cbm_prepare_eqm_descriptors(dp, &dp->gswr_eqm);
	if (ret)
		return ret;
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
	return 0;
}

static int grx500_cbm_start_lan1_dma(struct grx500_datapath *dp)
{
	const struct grx500_dqm_route *dqm = &dp->lan_dqm[4];
	const struct grx500_eqm_route *eqm = &dp->lan_eqm[0];
	phys_addr_t base = grx500_layout[GRX500_DMA_DESC].start;
	int ret;

	ret = grx500_cbm_request_dma_channel(dp, DMA2TX, dqm->dma_channel,
			base + 0x40000 + (dqm->port - 5) * 0x1000);
	if (ret)
		return ret;
	ret = grx500_cbm_request_dma_channel(dp, DMA2RX, eqm->dma_channel,
			base + (eqm->port - 5) * 0x1000);
	if (ret)
		return ret;
	ret = grx500_cbm_request_dma_channel(dp, DMA1RX,
			dp->gswr_eqm.dma_channel,
			base + (dp->gswr_eqm.port - 5) * 0x1000);
	if (ret)
		return ret;
	/* The GSWIP-R sink must be ready before the P2P producer starts. */
	ret = ltq_dma_chan_on(dp->dma_channels[2]);
	if (ret)
		return ret;
	ret = ltq_dma_chan_on(dp->dma_channels[0]);
	if (ret)
		return ret;
	return ltq_dma_chan_on(dp->dma_channels[1]);
}

static void grx500_lan1_adjust_link(struct net_device *ndev)
{
	struct grx500_datapath *dp = grx500_lan1_dp(ndev);

	if (dp->phy->link)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);
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

static irqreturn_t grx500_lan1_irq(int irq, void *data)
{
	struct grx500_datapath *dp = data;

	if (!(__raw_readl(dp->base[GRX500_CBM] +
		 GRX500_RX_CBM_LINE * GRX500_CBM_IRQ_STRIDE) & GRX500_RX_CBM_IRQ))
		return IRQ_NONE;
	grx500_cbm_irq_mask(dp);
	napi_schedule(&dp->napi);
	return IRQ_HANDLED;
}

static int grx500_lan1_poll(struct napi_struct *napi, int budget)
{
	struct grx500_datapath *dp = container_of(napi, struct grx500_datapath, napi);
	struct net_device *ndev = dp->ndev;
	void __iomem *ls = dp->base[GRX500_LS] +
			   GRX500_RX_LS_PORT * GRX500_LS_PORT_STRIDE;
	int done = 0;

	while (done < budget &&
	       ((__raw_readl(ls + GRX500_LS_PORT_STATUS) >> 7) & 0xf)) {
		phys_addr_t frame;
		struct sk_buff *skb;
		u32 ptr, flags, len, offset;
		u8 *data;
		int pool, hdr;

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
			if (ndev->stats.rx_errors < 4)
				dev_info(dp->dev, "RX descriptor rejected ptr=%08x flags=%08x pool=%d\n",
					 ptr, flags, pool);
			ndev->stats.rx_errors++;
			if (pool >= 0)
				grx500_cbm_cpu_return(dp, 0, frame);
			done++;
			continue;
		}
		dma_sync_single_for_cpu(dp->dev, ptr & ~7u,
					len + offset, DMA_BIDIRECTIONAL);
		data = phys_to_virt(ptr + offset);
		/* Hardware may prepend 0, 8 or 48 parser bytes. Require a
		 * GSWIP-L source port 5 PMAC before accepting any LAN1 frame.
		 */
		for (hdr = 0; hdr <= 48; hdr += hdr == 0 ? 8 : 40) {
			if (len >= hdr + GRX500_PMAC_SIZE + ETH_HLEN &&
			    (data[hdr + 2] >> 4) == GRX500_LAN1_ENDPOINT)
				break;
		}
		if (hdr > 48) {
			if (ndev->stats.rx_dropped < 4)
				dev_info(dp->dev, "RX PMAC mismatch ptr=%08x flags=%08x first32=%*phN\n",
					 ptr, flags, (int)min_t(u32, len, 32), data);
			ndev->stats.rx_dropped++;
		} else {
			len -= hdr + GRX500_PMAC_SIZE;
			/* napi_alloc_skb() already reserves NET_SKB_PAD +
			 * NET_IP_ALIGN. A second reserve misaligns the IP header
			 * by two bytes and triggers MIPS address-error emulation.
			 */
			skb = napi_alloc_skb(napi, len);
			if (skb) {
				skb_put_data(skb, data + hdr + GRX500_PMAC_SIZE, len);
				skb->protocol = eth_type_trans(skb, ndev);
				/* Never hand a misaligned L3 header to the stack on
				 * this strict-alignment CPU. Check after ETH_HLEN is
				 * pulled, including frames with a stripped PMAC prefix.
				 */
				if (unlikely(!IS_ALIGNED((unsigned long)skb->data, 4))) {
					if (ndev->stats.rx_errors < 4)
						dev_err(dp->dev, "RX L3 alignment rejected headroom=%u mod4=%lu\n",
							skb_headroom(skb), (unsigned long)skb->data & 3);
					ndev->stats.rx_errors++;
					dev_kfree_skb_any(skb);
				} else {
					if (ndev->stats.rx_packets < 2)
						dev_info(dp->dev, "RX L3 aligned headroom=%u protocol=%04x len=%u\n",
							 skb_headroom(skb), ntohs(skb->protocol), len);
					napi_gro_receive(napi, skb);
					ndev->stats.rx_packets++;
					ndev->stats.rx_bytes += len;
				}
			} else {
				ndev->stats.rx_dropped++;
			}
		}
		dma_sync_single_for_device(dp->dev, ptr & ~7u,
					2048 - (ptr - frame), DMA_BIDIRECTIONAL);
		grx500_cbm_cpu_return(dp, 0, frame);
		done++;
	}
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

static netdev_tx_t grx500_lan1_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct grx500_datapath *dp = grx500_lan1_dp(ndev);
	void __iomem *desc = dp->base[GRX500_EQM] + GRX500_EQM_CPU_DESC;
	unsigned long irqflags;
	phys_addr_t frame;
	u32 addr, len;
	u8 *data;
	int ret;

	if (unlikely(skb->len > ETH_FRAME_LEN + VLAN_HLEN ||
		     skb->len < ETH_HLEN)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	spin_lock_irqsave(&dp->tx_lock, irqflags);
	ret = grx500_cbm_cpu_alloc_standard(dp, 0, &frame);
	if (ret) {
		spin_unlock_irqrestore(&dp->tx_lock, irqflags);
		return NETDEV_TX_BUSY;
	}
	addr = frame + GRX500_CBM_FRAME_OFFSET;
	/* The switch sees PMAC plus a complete Ethernet frame. In particular,
	 * ARP is only 42 bytes before padding; never enqueue an undersized frame.
	 */
	len = max_t(u32, skb->len, ETH_ZLEN) + GRX500_PMAC_SIZE;
	data = phys_to_virt(addr);
	/* Vendor LAN TX PMAC: port_map_en, class_en, destination EP5. */
	memset(data, 0, GRX500_PMAC_SIZE);
	data[3] = 0x90;
	data[7] = BIT(GRX500_LAN1_ENDPOINT);
	ret = skb_copy_bits(skb, 0, data + GRX500_PMAC_SIZE, skb->len);
	if (unlikely(ret)) {
		grx500_cbm_cpu_return(dp, 0, frame);
		spin_unlock_irqrestore(&dp->tx_lock, irqflags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (skb->len < ETH_ZLEN)
		memset(data + GRX500_PMAC_SIZE + skb->len, 0,
		       ETH_ZLEN - skb->len);
	dma_sync_single_for_device(dp->dev, frame, 2048, DMA_BIDIRECTIONAL);
	/* Big-endian CBM CPU enqueue descriptor: DW1, DW0, address, DW3.
	 * The final DW3 write hands the complete frame to CBM/TMU.
	 */
	/* Vendor cbm_cpu_enqueue_hw writes DW0 first, then DW1. The previous
	 * image put EP5 into DW0 bits 23:20 (and left DW1 EP=0), so packets
	 * counted by the netdev never selected the LAN1 CBM/TMU route.
	 */
	__raw_writel(0, desc);
	__raw_writel(GRX500_LAN1_ENDPOINT << GRX500_TX_DW1_EP_SHIFT,
		     desc + 4);
	__raw_writel(addr & ~7u, desc + 8);
	wmb();
	__raw_writel(BIT(29) | BIT(28) | ((addr & 7) << 23) | len,
		     desc + 12);
	spin_unlock_irqrestore(&dp->tx_lock, irqflags);
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
}

static int grx500_lan1_open(struct net_device *ndev)
{
	struct grx500_datapath *dp = grx500_lan1_dp(ndev);
	struct phy_device *phy;
	int i, ret;

	if (dp->poisoned)
		return -EIO;
	phy = mdiobus_get_phy(dp->gswip[0], GRX500_LAN1_GSWIP_PORT);
	if (!phy)
		return -ENODEV;
	ret = dma_set_mask_and_coherent(dp->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = grx500_cbm_map_pools(dp);
	if (ret)
		goto err_unmap;
	/* From here onward the CBM/TMU global state is modified. Reboot the
	 * RAM image after any failed start rather than reusing half-initialized
	 * hardware state.
	 */
	dp->poisoned = true;
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
	ret = grx500_cbm_prepare_lan1_descriptors(dp);
	if (ret)
		goto err_engines;
	grx500_cbm_prepare_cpu_qidt(dp);
	ret = grx500_cbm_prepare_switches(dp);
	if (ret)
		goto err_engines;
	ret = request_irq(dp->irq[GRX500_RX_IRQ_INDEX], grx500_lan1_irq, 0, "grx500-lan1", dp);
	if (ret)
		goto err_engines;
	napi_enable(&dp->napi);
	ret = grx500_cbm_start_lan1_dma(dp);
	if (ret)
		goto err_dma;
	ret = grx500_cbm_start_p2p(dp);
	if (ret)
		goto err_dma;
	ret = phy_connect_direct(ndev, phy, grx500_lan1_adjust_link,
				 PHY_INTERFACE_MODE_RGMII);
	if (ret)
		goto err_dma;
	dp->phy = phy;
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
			goto err_phy;
	}
	ret = grx500_gswip_enable_port(dp->gswip[0], false, 0);
	if (!ret)
		ret = grx500_gswip_enable_port(dp->gswip[0], false,
					GRX500_LAN1_GSWIP_PORT);
	if (ret)
		goto err_phy;
	dp->started = true;
	netif_carrier_off(ndev);
	phy_start(phy);
	netif_start_queue(ndev);
	dev_info(dp->dev, "LAN1 CBM/TMU path active: EP5 Q20 DQM10 DMA2TX5/RX14, DMA2RX0-3 to DMA1TX0-3 P2P, DMA1RX0 EQM7 CPU RX\n");
	return 0;

err_phy:
	grx500_gswip_disable_port(dp->gswip[0], false, 0);
	for (i = 0; i < GRX500_GSWIP_R_PORTS; i++)
		grx500_gswip_disable_port(dp->gswip[1], true, i);
	grx500_cbm_irq_mask(dp);
	phy_disconnect(phy);
	dp->phy = NULL;
err_dma:
	grx500_cbm_release_p2p(dp);
	grx500_cbm_release_dma(dp);
	napi_disable(&dp->napi);
	free_irq(dp->irq[GRX500_RX_IRQ_INDEX], dp);
err_engines:
	__raw_writel(0, dp->base[GRX500_EQM] + GRX500_EQM_CTRL);
	__raw_writel(0, dp->base[GRX500_DQM] + GRX500_DQM_CTRL);
err_unmap:
	grx500_cbm_unmap_pools(dp);
	dev_err(dp->dev, "LAN1 start failed (%d); reboot RAM image before retry\n", ret);
	return ret;
}

static int grx500_lan1_stop(struct net_device *ndev)
{
	struct grx500_datapath *dp = grx500_lan1_dp(ndev);
	int i;

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	phy_stop(dp->phy);
	grx500_gswip_disable_port(dp->gswip[0], false,
				 GRX500_LAN1_GSWIP_PORT);
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
	phy_disconnect(dp->phy);
	dp->phy = NULL;
	dp->started = false;
	grx500_cbm_unmap_pools(dp);
	return 0;
}

static const struct net_device_ops grx500_lan1_ops = {
	.ndo_open = grx500_lan1_open,
	.ndo_stop = grx500_lan1_stop,
	.ndo_start_xmit = grx500_lan1_xmit,
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
	phys_addr_t kernel_start = __pa_symbol(_text);
	phys_addr_t kernel_end = __pa_symbol(_end);
	int i;

	for (i = 0; i < GRX500_POOL_COUNT; i++) {
		struct device_node *node;
		struct reserved_mem *rmem;
		phys_addr_t last;

		node = of_parse_phandle(pdev->dev.of_node, "memory-region", i);
		if (!node)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "CBM buffer pool is missing\n");
		rmem = of_reserved_mem_lookup(node);
		of_node_put(node);
		if (!rmem || rmem->size != grx500_pool_size[i] ||
		    !IS_ALIGNED(rmem->base, grx500_pool_alignment[i]))
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "CBM buffer pool geometry differs from vendor config\n");
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
		"eqm_irq=%d cpu_dqm_irqs=%d,%d,%d,%d (requested only while lan1 is up)\n",
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
	return len;
}
static DEVICE_ATTR_RO(status);

static int grx500_datapath_probe(struct platform_device *pdev)
{
	struct grx500_datapath *dp;
	struct resource *res;
	int i, ret;

	dp = devm_kzalloc(&pdev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;
	dp->dev = &pdev->dev;
	spin_lock_init(&dp->tmu_lock);
	spin_lock_init(&dp->tx_lock);
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
	dp->ndev = alloc_etherdev(sizeof(struct grx500_datapath *));
	if (!dp->ndev) {
		ret = -ENOMEM;
		goto err_attr;
	}
	*(struct grx500_datapath **)netdev_priv(dp->ndev) = dp;
	strscpy(dp->ndev->name, "lan1", IFNAMSIZ);
	SET_NETDEV_DEV(dp->ndev, &pdev->dev);
	dp->ndev->netdev_ops = &grx500_lan1_ops;
	dp->ndev->min_mtu = ETH_MIN_MTU;
	dp->ndev->max_mtu = ETH_DATA_LEN;
	dp->ndev->features &= ~(NETIF_F_SG | NETIF_F_HW_CSUM | NETIF_F_TSO);
	eth_hw_addr_random(dp->ndev);
	netif_carrier_off(dp->ndev);
	netif_napi_add(dp->ndev, &dp->napi, grx500_lan1_poll);
	ret = register_netdev(dp->ndev);
	if (ret)
		goto err_netdev;
	dev_info(&pdev->dev, "LAN1 netdev registered; CBM/TMU starts when interface is brought up\n");
	return 0;

err_netdev:
	netif_napi_del(&dp->napi);
	free_netdev(dp->ndev);
err_attr:
	device_remove_file(&pdev->dev, &dev_attr_status);
	return ret;
}

static void grx500_datapath_remove(struct platform_device *pdev)
{
	struct grx500_datapath *dp = platform_get_drvdata(pdev);

	unregister_netdev(dp->ndev);
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
