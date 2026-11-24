// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 User Message Timer (UMT / mcpy@a2800000) hardware sync helper. */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/grx500-umt.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <asm/mipsregs.h>
#include <lantiq_dmax.h>

#define GRX500_MCPY_INT_MASK		0x080
#define GRX500_MCPY_INTERNAL_INT_MASK	0x084
#define GRX500_MCPY_INT_EN		0x088
#define GRX500_MCPY_INTERNAL_INT_EN	0x08c
#define GRX500_MCPY_UMT_SW_MODE		0x094
#define GRX500_MCPY_UMT_PERD		0x0dc
#define GRX500_MCPY_UMT_TRG_MUX		0x0e0
#define GRX500_MCPY_UMT_CNT_CTRL	0x0e4
#define GRX500_MCPY_GCTRL		0x200
#define GRX500_MCPY_UMT_MSG(x)		(0x220 + (x) * 4)
#define GRX500_MCPY_UMT_DEST		0x230
#define GRX500_MCPY_MIPS_CFG(x)		(0x240 + (x) * 0x10)
#define GRX500_MCPY_UMT_XBASE		0x400
#define GRX500_MCPY_UMT_XOFFSET		0x100
#define GRX500_MCPY_UMT_XPERIOD		0x020
#define GRX500_MCPY_UMT_XDEST		0x030
#define GRX500_MCPY_UMT_XSW_MODE	0x034

#define GRX500_MCPY_X_ADDR(id, off) \
	(GRX500_MCPY_UMT_XBASE + ((id) - 1) * GRX500_MCPY_UMT_XOFFSET + (off))

/* Vendor GRX500 NGI/XBAR clock default (200 MHz) if no DT clock is attached. */
#define GRX500_UMT_DEFAULT_CLK_HZ	200000000u

/* Vendor 7022 MCPY DMA3 port definitions */
#define GRX500_MCPY_PORTS_NUM		8
#define GRX500_MCPY_DMA_RX_CID		12
#define GRX500_MCPY_DMA_TX_CID		13
#define GRX500_MCPY_DBASE		0x10600u
#define GRX500_MCPY_DBASE_OFFSET	0x100u
#define GRX500_MCPY_DMA_DESC_SIZE	8u

struct grx500_umt_port {
	spinlock_t lock;
	u32 umt_pid;
	u32 ep_id;
	u32 cbm_pid;
	u32 dma_cid;
	u32 dst_phys;
	u32 period_us;
	bool enabled;
	void __iomem *dqpc_reg;
	void __iomem *dst_iomem;
	u32 dst_mapped_phys;
	u32 dq_idx;
	u32 sw_ticks;
	u32 sw_writes;
	u32 sw_last_cnt;
	u32 sw_last_dqpc;
	u32 sw_total_pkts;
	struct hrtimer sw_timer;
	bool sw_timer_init;
};

struct grx500_umt_ctx {
	struct device *dev;
	void __iomem *base;
	struct clk *toe_clk;
	struct clk *mpe_clk;
	u32 clk_mhz;
	spinlock_t lock;
	phys_addr_t phybase;
	u32 mcpy_port_map;
	u32 mcpy_rx_ch[GRX500_MCPY_PORTS_NUM];
	u32 mcpy_tx_ch[GRX500_MCPY_PORTS_NUM];
	struct grx500_umt_port ports[GRX500_UMT_PORTS_NUM];
};

static struct grx500_umt_ctx *g_umt_ctx;

static u32 grx500_umt_read(struct grx500_umt_ctx *ctx, u32 reg)
{
	return __raw_readl(ctx->base + reg);
}

static void grx500_umt_write(struct grx500_umt_ctx *ctx, u32 reg, u32 val)
{
	wmb();
	__raw_writel(val, ctx->base + reg);
}

static void grx500_umt_hw_set_mode(struct grx500_umt_ctx *ctx, u32 id,
				   enum grx500_umt_mode mode)
{
	u32 val, off;

	if (!id) {
		val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL) & ~BIT(1);
		val |= ((u32)mode & 1) << 1;
	} else {
		off = 16 + (id - 1) * 3;
		val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL) & ~BIT(off);
		val |= ((u32)mode & 1) << off;
	}
	grx500_umt_write(ctx, GRX500_MCPY_GCTRL, val);
}

static void grx500_umt_hw_set_msgmode(struct grx500_umt_ctx *ctx, u32 id,
				      enum grx500_umt_msg_mode msg_mode)
{
	u32 reg = id ? GRX500_MCPY_X_ADDR(id, GRX500_MCPY_UMT_XSW_MODE) :
		       GRX500_MCPY_UMT_SW_MODE;

	grx500_umt_write(ctx, reg, (u32)msg_mode);
}

static void grx500_umt_hw_set_period(struct grx500_umt_ctx *ctx, u32 id,
				     u32 period_us)
{
	u32 reg = id ? GRX500_MCPY_X_ADDR(id, GRX500_MCPY_UMT_XPERIOD) :
		       GRX500_MCPY_UMT_PERD;

	grx500_umt_write(ctx, reg, period_us * ctx->clk_mhz);
}

static void grx500_umt_hw_set_dst(struct grx500_umt_ctx *ctx, u32 id,
				  u32 dst_phys)
{
	u32 reg = id ? GRX500_MCPY_X_ADDR(id, GRX500_MCPY_UMT_XDEST) :
		       GRX500_MCPY_UMT_DEST;

	grx500_umt_write(ctx, reg, dst_phys);
}

static void grx500_umt_hw_set_mux(struct grx500_umt_ctx *ctx, u32 id,
				  u32 cbm_pid, u32 dma_cid)
{
	u32 val = grx500_umt_read(ctx, GRX500_MCPY_UMT_TRG_MUX);

	val &= ~(0x000f000fu << (id * 4));
	val |= ((dma_cid & 0xf) << (id * 4)) |
	       ((cbm_pid & 0xf) << (16 + id * 4));
	grx500_umt_write(ctx, GRX500_MCPY_UMT_TRG_MUX, val);
}

static void grx500_umt_hw_enable(struct grx500_umt_ctx *ctx, u32 id,
				 bool enable)
{
	u32 val, off;

	if (!id) {
		val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL) & ~BIT(2);
		if (enable)
			val |= BIT(2);
	} else {
		off = 17 + (id - 1) * 3;
		val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL) & ~BIT(off);
		if (enable)
			val |= BIT(off);
	}
	grx500_umt_write(ctx, GRX500_MCPY_GCTRL, val);
}

int grx500_umt_request(u32 ep_id, u32 cbm_pid, u32 *dma_ctrlid,
		       u32 *dma_cid, u32 *umt_id)
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;
	unsigned long flags;
	int i, pid = -1;

	if (!ctx)
		return -ENODEV;
	if (!ep_id || cbm_pid >= GRX500_UMT_PORTS_NUM ||
	    !dma_ctrlid || !dma_cid || !umt_id)
		return -EINVAL;

	spin_lock_irqsave(&ctx->lock, flags);
	for (i = 0; i < GRX500_UMT_PORTS_NUM; i++) {
		port = &ctx->ports[i];
		if (port->ep_id == ep_id && port->cbm_pid == cbm_pid) {
			pid = i;
			break;
		}
		if (!port->ep_id && pid < 0)
			pid = i;
	}
	if (pid < 0) {
		spin_unlock_irqrestore(&ctx->lock, flags);
		return -ENOSPC;
	}

	port = &ctx->ports[pid];
	port->ep_id = ep_id;
	port->cbm_pid = cbm_pid;
	grx500_umt_hw_set_mux(ctx, port->umt_pid, port->cbm_pid, port->dma_cid);
	*dma_ctrlid = 1; /* DMA1TX */
	*dma_cid = port->dma_cid;
	*umt_id = port->umt_pid;
	spin_unlock_irqrestore(&ctx->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(grx500_umt_request);

static enum hrtimer_restart grx500_umt_sw_timer_cb(struct hrtimer *t)
{
	struct grx500_umt_port *port = container_of(t, struct grx500_umt_port,
						    sw_timer);
	u32 dq_ptr, dq_cnt;

	if (!READ_ONCE(port->enabled) || !port->dqpc_reg || !port->dst_iomem)
		return HRTIMER_NORESTART;

	dq_ptr = __raw_readl(port->dqpc_reg);
	dq_cnt = dq_ptr - port->dq_idx;
	if (dq_cnt > 32)
		dq_cnt = 32;
	if (dq_cnt) {
		__raw_writel(dq_cnt, port->dst_iomem);
		wmb();
		port->dq_idx += dq_cnt;
		port->sw_writes++;
		port->sw_last_cnt = dq_cnt;
		port->sw_total_pkts += dq_cnt;
	}
	port->sw_last_dqpc = dq_ptr;
	port->sw_ticks++;

	hrtimer_forward_now(t, ns_to_ktime((u64)port->period_us * 1000ULL));
	return HRTIMER_RESTART;
}

void grx500_umt_sw_set_dqpc(u32 umt_id, void __iomem *dqpc_reg)
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;

	if (!ctx || umt_id >= GRX500_UMT_PORTS_NUM)
		return;
	port = &ctx->ports[umt_id];
	if (port->sw_timer_init)
		hrtimer_cancel(&port->sw_timer);
	port->dqpc_reg = dqpc_reg;
	port->dq_idx = dqpc_reg ? __raw_readl(dqpc_reg) : 0;
	port->sw_ticks = 0;
	port->sw_writes = 0;
	port->sw_last_cnt = 0;
	port->sw_last_dqpc = port->dq_idx;
	port->sw_total_pkts = 0;
	if (!port->sw_timer_init) {
		hrtimer_init(&port->sw_timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL);
		port->sw_timer.function = grx500_umt_sw_timer_cb;
		port->sw_timer_init = true;
	}
}
EXPORT_SYMBOL_GPL(grx500_umt_sw_set_dqpc);

int grx500_umt_set_mode(u32 umt_id, u32 ep_id, enum grx500_umt_mode umt_mode,
			enum grx500_umt_msg_mode msg_mode, u32 phy_dst,
			u32 period_us, bool enable)
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;
	void __iomem *old_iomem = NULL;
	void __iomem *new_iomem = NULL;
	u32 pa = phy_dst;
	unsigned long flags;

	if (!ctx)
		return -ENODEV;
	if (umt_id >= GRX500_UMT_PORTS_NUM || !phy_dst || !period_us)
		return -EINVAL;

	port = &ctx->ports[umt_id];
	if (port->sw_timer_init)
		hrtimer_cancel(&port->sw_timer);

	if (port->dqpc_reg && port->dst_mapped_phys != pa) {
		new_iomem = ioremap(pa, 4);
		if (!new_iomem)
			return -ENOMEM;
	}

	spin_lock_irqsave(&port->lock, flags);
	if (port->ep_id != ep_id) {
		spin_unlock_irqrestore(&port->lock, flags);
		if (new_iomem)
			iounmap(new_iomem);
		return -EINVAL;
	}

	if (new_iomem) {
		old_iomem = port->dst_iomem;
		port->dst_iomem = new_iomem;
		port->dst_mapped_phys = pa;
	}

	grx500_umt_hw_enable(ctx, umt_id, false);
	/* Toggle mode while disabled to reset internal UMT counter (vendor umt_reset_umt). */
	grx500_umt_hw_set_mode(ctx, umt_id,
			       umt_mode == GRX500_UMT_SELFCNT_MODE ?
			       GRX500_UMT_USER_MODE : GRX500_UMT_SELFCNT_MODE);
	port->dst_phys = phy_dst;
	port->period_us = period_us;
	WRITE_ONCE(port->enabled, enable);

	grx500_umt_hw_set_mode(ctx, umt_id, umt_mode);
	grx500_umt_hw_set_msgmode(ctx, umt_id, msg_mode);
	grx500_umt_hw_set_dst(ctx, umt_id, phy_dst);
	grx500_umt_hw_set_period(ctx, umt_id, period_us);
	if (!port->dqpc_reg)
		grx500_umt_hw_enable(ctx, umt_id, enable);
	dev_info(ctx->dev,
		 "UMT%u configured (%s): ep=%u cbm_pid=%u dma_cid=%u dst=0x%08x period=%uus en=%u gctrl=0x%08x mux=0x%08x\n",
		 umt_id, port->dqpc_reg ? "SW_MODE" : "HW",
		 ep_id, port->cbm_pid, port->dma_cid, phy_dst,
		 period_us, enable, grx500_umt_read(ctx, GRX500_MCPY_GCTRL),
		 grx500_umt_read(ctx, GRX500_MCPY_UMT_TRG_MUX));
	spin_unlock_irqrestore(&port->lock, flags);

	if (old_iomem)
		iounmap(old_iomem);
	if (port->dqpc_reg && enable && port->sw_timer_init)
		hrtimer_start(&port->sw_timer,
			      ns_to_ktime((u64)period_us * 1000ULL),
			      HRTIMER_MODE_REL);

	return 0;
}
EXPORT_SYMBOL_GPL(grx500_umt_set_mode);

int grx500_umt_enable(u32 umt_id, u32 ep_id, bool enable)
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;
	unsigned long flags;

	if (!ctx)
		return -ENODEV;
	if (umt_id >= GRX500_UMT_PORTS_NUM)
		return -EINVAL;

	port = &ctx->ports[umt_id];
	if (port->sw_timer_init && !enable)
		hrtimer_cancel(&port->sw_timer);

	spin_lock_irqsave(&port->lock, flags);
	if (port->ep_id != ep_id || !port->ep_id || !port->dst_phys) {
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}

	WRITE_ONCE(port->enabled, enable);
	if (!port->dqpc_reg)
		grx500_umt_hw_enable(ctx, umt_id, enable);
	spin_unlock_irqrestore(&port->lock, flags);

	if (port->dqpc_reg && enable && port->sw_timer_init)
		hrtimer_start(&port->sw_timer,
			      ns_to_ktime((u64)port->period_us * 1000ULL),
			      HRTIMER_MODE_REL);

	return 0;
}
EXPORT_SYMBOL_GPL(grx500_umt_enable);

void grx500_umt_debug_info(u32 out[8])
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;

	if (!ctx) {
		memset(out, 0, sizeof(u32) * 8);
		return;
	}
	out[0] = grx500_umt_read(ctx, GRX500_MCPY_GCTRL);
	out[1] = grx500_umt_read(ctx, GRX500_MCPY_UMT_TRG_MUX);
	out[2] = grx500_umt_read(ctx, GRX500_MCPY_UMT_DEST);
	out[3] = grx500_umt_read(ctx, GRX500_MCPY_UMT_PERD);
	out[4] = grx500_umt_read(ctx, GRX500_MCPY_UMT_SW_MODE);
	out[5] = grx500_umt_read(ctx, GRX500_MCPY_UMT_CNT_CTRL);
	out[6] = grx500_umt_read(ctx, GRX500_MCPY_UMT_MSG(0));
	out[7] = grx500_umt_read(ctx, GRX500_MCPY_UMT_MSG(1));
}
EXPORT_SYMBOL_GPL(grx500_umt_debug_info);

void grx500_umt_sw_debug(u32 umt_id, u32 out[8])
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;

	if (!ctx || umt_id >= GRX500_UMT_PORTS_NUM) {
		memset(out, 0, sizeof(u32) * 8);
		return;
	}
	port = &ctx->ports[umt_id];
	out[0] = READ_ONCE(port->enabled);
	out[1] = READ_ONCE(port->dq_idx);
	out[2] = READ_ONCE(port->sw_ticks);
	out[3] = READ_ONCE(port->sw_writes);
	out[4] = READ_ONCE(port->sw_last_cnt);
	out[5] = READ_ONCE(port->sw_last_dqpc);
	out[6] = READ_ONCE(port->sw_total_pkts);
	out[7] = READ_ONCE(port->dst_mapped_phys);
}
EXPORT_SYMBOL_GPL(grx500_umt_sw_debug);

int grx500_umt_release(u32 umt_id, u32 ep_id)
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;
	unsigned long flags;

	if (!ctx)
		return -ENODEV;
	if (umt_id >= GRX500_UMT_PORTS_NUM)
		return -EINVAL;

	port = &ctx->ports[umt_id];
	if (port->sw_timer_init)
		hrtimer_cancel(&port->sw_timer);
	spin_lock_irqsave(&port->lock, flags);
	if (port->ep_id != ep_id) {
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}
	grx500_umt_hw_enable(ctx, umt_id, false);
	if (port->dst_iomem) {
		iounmap(port->dst_iomem);
		port->dst_iomem = NULL;
		port->dst_mapped_phys = 0;
	}
	port->dqpc_reg = NULL;
	port->ep_id = 0;
	port->cbm_pid = 0;
	port->dst_phys = 0;
	port->period_us = 0;
	WRITE_ONCE(port->enabled, false);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(grx500_umt_release);

static int grx500_umt_enable_sys_clk(struct device *dev, const char *dev_id,
				     struct clk **out_clk, unsigned long *rate)
{
	struct clk *clk;
	int ret;

	clk = clk_get_sys(dev_id, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "clk_get_sys(%s) failed: %ld\n",
			dev_id, PTR_ERR(clk));
		*out_clk = NULL;
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "clk_prepare_enable(%s) failed: %d\n",
			dev_id, ret);
		clk_put(clk);
		*out_clk = NULL;
		return ret;
	}

	if (rate && !*rate)
		*rate = clk_get_rate(clk);
	*out_clk = clk;
	dev_info(dev, "enabled clock %s (rate=%lu)\n",
		 dev_id, clk_get_rate(clk));
	return 0;
}

static void grx500_umt_disable_sys_clk(struct clk **clk)
{
	if (!*clk)
		return;

	clk_disable_unprepare(*clk);
	clk_put(*clk);
	*clk = NULL;
}

static void grx500_umt_cleanup_mcpy_dma(struct grx500_umt_ctx *ctx)
{
	u32 pid;

	for (pid = 0; pid < GRX500_MCPY_PORTS_NUM; pid++) {
		if (!(ctx->mcpy_port_map & BIT(pid)))
			continue;
		ltq_dma_chan_off(ctx->mcpy_rx_ch[pid]);
		ltq_dma_chan_off(ctx->mcpy_tx_ch[pid]);
		ltq_free_dma(ctx->mcpy_rx_ch[pid]);
		ltq_free_dma(ctx->mcpy_tx_ch[pid]);
	}
	ctx->mcpy_port_map = 0;
}

static int grx500_umt_init_mcpy_dma(struct grx500_umt_ctx *ctx)
{
	u32 pid;
	int ret;

	for (pid = 0; pid < GRX500_MCPY_PORTS_NUM; pid++) {
		phys_addr_t rch_dbase = ctx->phybase + GRX500_MCPY_DBASE +
					pid * GRX500_MCPY_DBASE_OFFSET;
		phys_addr_t tch_dbase = rch_dbase +
					2u * GRX500_MCPY_DMA_DESC_SIZE;
		u32 rx_ch = _DMA_C(DMA3, DMA3_PORT,
				   GRX500_MCPY_DMA_RX_CID + pid * 2);
		u32 tx_ch = _DMA_C(DMA3, DMA3_PORT,
				   GRX500_MCPY_DMA_TX_CID + pid * 2);

		ret = ltq_request_dma(rx_ch, "mcpy-rx");
		if (ret)
			goto err_cleanup;
		ret = ltq_dma_chan_desc_cfg(rx_ch, rch_dbase, 2);
		if (ret) {
			ltq_free_dma(rx_ch);
			goto err_cleanup;
		}

		ret = ltq_request_dma(tx_ch, "mcpy-tx");
		if (ret) {
			ltq_free_dma(rx_ch);
			goto err_cleanup;
		}
		ret = ltq_dma_chan_desc_cfg(tx_ch, tch_dbase, 2);
		if (ret) {
			ltq_free_dma(tx_ch);
			ltq_free_dma(rx_ch);
			goto err_cleanup;
		}

		ltq_dma_chan_on(rx_ch);
		ltq_dma_chan_on(tx_ch);
		ctx->mcpy_rx_ch[pid] = rx_ch;
		ctx->mcpy_tx_ch[pid] = tx_ch;
		ctx->mcpy_port_map |= BIT(pid);
	}

	return 0;

err_cleanup:
	grx500_umt_cleanup_mcpy_dma(ctx);
	return ret == -ENODEV ? -EPROBE_DEFER : ret;
}

static int grx500_umt_probe(struct platform_device *pdev)
{
	struct grx500_umt_ctx *ctx;
	struct resource *res;
	unsigned long rate = 0;
	u32 val, i;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	ctx->phybase = res->start;
	ctx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);

	/* SSX7/SSX8 (TOE/MCPY/MPE) CGU clock gates are disabled by default at boot. */
	ret = grx500_umt_enable_sys_clk(&pdev->dev, "1e300000.dma",
					&ctx->toe_clk, &rate);
	if (ret)
		return ret;

	ret = grx500_umt_enable_sys_clk(&pdev->dev, "a0000000.mpe",
					&ctx->mpe_clk, &rate);
	if (ret) {
		grx500_umt_disable_sys_clk(&ctx->toe_clk);
		return ret;
	}
	ctx->clk_mhz = rate ? max_t(u32, rate / 1000000u, 1) :
			      (GRX500_UMT_DEFAULT_CLK_HZ / 1000000u);

	spin_lock_init(&ctx->lock);
	for (i = 0; i < GRX500_UMT_PORTS_NUM; i++) {
		char prop[32];
		u32 dmacid = 7 + i;

		snprintf(prop, sizeof(prop), "lantiq,umt%u-dmacid", i);
		of_property_read_u32(pdev->dev.of_node, prop, &dmacid);
		spin_lock_init(&ctx->ports[i].lock);
		ctx->ports[i].umt_pid = i;
		ctx->ports[i].dma_cid = dmacid;
	}

	/* Reset MCPY/UMT engine before programming EVA SegCtl and UMT regs (vendor mcpy_sw_reset). */
	val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL) | BIT(4);
	grx500_umt_write(ctx, GRX500_MCPY_GCTRL, val);
	for (i = 0; i < 10000; i++) {
		if (!(grx500_umt_read(ctx, GRX500_MCPY_GCTRL) & BIT(4)))
			break;
		cpu_relax();
	}
	if (grx500_umt_read(ctx, GRX500_MCPY_GCTRL) & BIT(4))
		dev_warn(&pdev->dev, "MCPY soft reset timeout\n");

	/* Copy EVA SegCtl0..2 into MCPY_MIPS_CFG(0..5) (matching vendor mcpy_eva_cfg). */
	if (IS_ENABLED(CONFIG_EVA)) {
		u32 seg0 = read_c0_segctl0();
		u32 seg1 = read_c0_segctl1();
		u32 seg2 = read_c0_segctl2();

		grx500_umt_write(ctx, GRX500_MCPY_MIPS_CFG(0), seg0 & 0xffffu);
		grx500_umt_write(ctx, GRX500_MCPY_MIPS_CFG(1), (seg0 >> 16) & 0xffffu);
		grx500_umt_write(ctx, GRX500_MCPY_MIPS_CFG(2), seg1 & 0xffffu);
		grx500_umt_write(ctx, GRX500_MCPY_MIPS_CFG(3), (seg1 >> 16) & 0xffffu);
		grx500_umt_write(ctx, GRX500_MCPY_MIPS_CFG(4), seg2 & 0xffffu);
		grx500_umt_write(ctx, GRX500_MCPY_MIPS_CFG(5), (seg2 >> 16) & 0xffffu);
	}

	/* Initialize vendor 7022 MCPY DMA3 channels (pid 0..7) before enabling MCPY/UMT. */
	ret = grx500_umt_init_mcpy_dma(ctx);
	if (ret) {
		grx500_umt_disable_sys_clk(&ctx->mpe_clk);
		grx500_umt_disable_sys_clk(&ctx->toe_clk);
		return ret;
	}

	/* Keep MCPY/DMA3 interrupts masked; UMT runs in self-counting mode without IRQ. */
	grx500_umt_write(ctx, GRX500_MCPY_INTERNAL_INT_EN, 0);
	grx500_umt_write(ctx, GRX500_MCPY_INTERNAL_INT_MASK, 0);
	grx500_umt_write(ctx, GRX500_MCPY_INT_EN, 0);
	grx500_umt_write(ctx, GRX500_MCPY_INT_MASK, 0);

	/* Enable MCPY/UMT engine (BIT(0)), MCPY ports (bits 15:8), UMT expand mode (BIT(31)) and BE DW swap (BIT(29)). */
	val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL);
	val &= ~(0xffu << 8);
	val |= (ctx->mcpy_port_map << 8) | BIT(31) | BIT(29) | BIT(0);
	if (!IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		val |= BIT(28);
	else
		val &= ~BIT(28);
	grx500_umt_write(ctx, GRX500_MCPY_GCTRL, val);

	platform_set_drvdata(pdev, ctx);
	g_umt_ctx = ctx;

	dev_info(&pdev->dev,
		 "GRX500 UMT expand mode active (clk=%u MHz, mcpy_ports=0x%02x)\n",
		 ctx->clk_mhz, ctx->mcpy_port_map);
	return 0;
}

static void grx500_umt_remove(struct platform_device *pdev)
{
	struct grx500_umt_ctx *ctx = platform_get_drvdata(pdev);
	u32 i;

	for (i = 0; i < GRX500_UMT_PORTS_NUM; i++) {
		if (ctx->ports[i].sw_timer_init)
			hrtimer_cancel(&ctx->ports[i].sw_timer);
		if (ctx->ports[i].dst_iomem) {
			iounmap(ctx->ports[i].dst_iomem);
			ctx->ports[i].dst_iomem = NULL;
		}
		if (ctx->ports[i].enabled)
			grx500_umt_hw_enable(ctx, i, false);
	}
	grx500_umt_cleanup_mcpy_dma(ctx);
	g_umt_ctx = NULL;
	grx500_umt_disable_sys_clk(&ctx->mpe_clk);
	grx500_umt_disable_sys_clk(&ctx->toe_clk);
}

static const struct of_device_id grx500_umt_of_match[] = {
	{ .compatible = "lantiq,grx500-umt" },
	{ }
};
MODULE_DEVICE_TABLE(of, grx500_umt_of_match);

static struct platform_driver grx500_umt_driver = {
	.probe = grx500_umt_probe,
	.remove = grx500_umt_remove,
	.driver = {
		.name = "grx500-umt",
		.of_match_table = grx500_umt_of_match,
	},
};
module_platform_driver(grx500_umt_driver);

MODULE_DESCRIPTION("GRX500 UMT hardware counter sync helper");
MODULE_LICENSE("GPL");
