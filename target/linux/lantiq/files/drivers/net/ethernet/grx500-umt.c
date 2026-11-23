// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 User Message Timer (UMT / mcpy@a2800000) hardware sync helper. */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/grx500-umt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define GRX500_MCPY_UMT_SW_MODE		0x094
#define GRX500_MCPY_UMT_PERD		0x0dc
#define GRX500_MCPY_UMT_TRG_MUX		0x0e0
#define GRX500_MCPY_UMT_CNT_CTRL	0x0e4
#define GRX500_MCPY_GCTRL		0x200
#define GRX500_MCPY_UMT_DEST		0x230
#define GRX500_MCPY_UMT_XBASE		0x400
#define GRX500_MCPY_UMT_XOFFSET		0x100
#define GRX500_MCPY_UMT_XPERIOD		0x020
#define GRX500_MCPY_UMT_XDEST		0x030
#define GRX500_MCPY_UMT_XSW_MODE	0x034

#define GRX500_MCPY_X_ADDR(id, off) \
	(GRX500_MCPY_UMT_XBASE + ((id) - 1) * GRX500_MCPY_UMT_XOFFSET + (off))

/* Vendor GRX500 NGI/XBAR clock default (200 MHz) if no DT clock is attached. */
#define GRX500_UMT_DEFAULT_CLK_HZ	200000000u

struct grx500_umt_port {
	spinlock_t lock;
	u32 umt_pid;
	u32 ep_id;
	u32 cbm_pid;
	u32 dma_cid;
	u32 dst_phys;
	u32 period_us;
	bool enabled;
};

struct grx500_umt_ctx {
	struct device *dev;
	void __iomem *base;
	struct clk *toe_clk;
	struct clk *mpe_clk;
	u32 clk_mhz;
	spinlock_t lock;
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

int grx500_umt_set_mode(u32 umt_id, u32 ep_id, enum grx500_umt_mode umt_mode,
			enum grx500_umt_msg_mode msg_mode, u32 phy_dst,
			u32 period_us, bool enable)
{
	struct grx500_umt_ctx *ctx = g_umt_ctx;
	struct grx500_umt_port *port;
	unsigned long flags;

	if (!ctx)
		return -ENODEV;
	if (umt_id >= GRX500_UMT_PORTS_NUM || !phy_dst || !period_us)
		return -EINVAL;

	port = &ctx->ports[umt_id];
	spin_lock_irqsave(&port->lock, flags);
	if (port->ep_id != ep_id) {
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}

	grx500_umt_hw_enable(ctx, umt_id, false);
	port->dst_phys = phy_dst;
	port->period_us = period_us;
	port->enabled = enable;

	grx500_umt_hw_set_mode(ctx, umt_id, umt_mode);
	grx500_umt_hw_set_msgmode(ctx, umt_id, msg_mode);
	grx500_umt_hw_set_dst(ctx, umt_id, phy_dst);
	grx500_umt_hw_set_period(ctx, umt_id, period_us);
	grx500_umt_hw_enable(ctx, umt_id, enable);
	spin_unlock_irqrestore(&port->lock, flags);

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
	spin_lock_irqsave(&port->lock, flags);
	if (port->ep_id != ep_id || !port->ep_id || !port->dst_phys) {
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}

	port->enabled = enable;
	grx500_umt_hw_enable(ctx, umt_id, enable);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(grx500_umt_enable);

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
	spin_lock_irqsave(&port->lock, flags);
	if (port->ep_id != ep_id) {
		spin_unlock_irqrestore(&port->lock, flags);
		return -EINVAL;
	}
	grx500_umt_hw_enable(ctx, umt_id, false);
	port->ep_id = 0;
	port->cbm_pid = 0;
	port->dst_phys = 0;
	port->period_us = 0;
	port->enabled = false;
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

static int grx500_umt_probe(struct platform_device *pdev)
{
	struct grx500_umt_ctx *ctx;
	unsigned long rate = 0;
	u32 val, i;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = &pdev->dev;
	ctx->base = devm_platform_ioremap_resource(pdev, 0);
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

	/* Enable UMT expand mode and Big-Endian DW swap (matching vendor). */
	val = grx500_umt_read(ctx, GRX500_MCPY_GCTRL);
	val |= BIT(31) | BIT(29);
	if (!IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		val |= BIT(28);
	else
		val &= ~BIT(28);
	grx500_umt_write(ctx, GRX500_MCPY_GCTRL, val);

	platform_set_drvdata(pdev, ctx);
	g_umt_ctx = ctx;

	dev_info(&pdev->dev, "GRX500 UMT expand mode active (clk=%u MHz)\n",
		 ctx->clk_mhz);
	return 0;
}

static void grx500_umt_remove(struct platform_device *pdev)
{
	struct grx500_umt_ctx *ctx = platform_get_drvdata(pdev);
	u32 i;

	for (i = 0; i < GRX500_UMT_PORTS_NUM; i++) {
		if (ctx->ports[i].enabled)
			grx500_umt_hw_enable(ctx, i, false);
	}
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
