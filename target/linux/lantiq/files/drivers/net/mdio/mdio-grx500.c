// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 GSWIP 3.0 MDIO controller and internal GPHY firmware loader. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/reset.h>
#include <linux/slab.h>

/* Vendor GSWIP 3.0 register numbers are 32-bit word offsets. */
#define GRX500_GSW_TOP                  0xf00
#define GRX500_MDCTRL                   ((GRX500_GSW_TOP + 0x004) * 4)
#define GRX500_MDREAD                   ((GRX500_GSW_TOP + 0x005) * 4)
#define GRX500_MDWRITE                  ((GRX500_GSW_TOP + 0x006) * 4)
#define GRX500_MDCTRL_BUSY              BIT(12)
#define GRX500_MDCTRL_READ              (2 << 10)
#define GRX500_MDCTRL_WRITE             (1 << 10)
#define GRX500_GPHY_FW_SIZE             65536
#define GRX500_GPHY_ALIGN               16384
#define GRX500_GSWT_MDCCFG_0            (GRX500_GSW_TOP + 0x007)
#define GRX500_GSWT_MDCCFG_1            (GRX500_GSW_TOP + 0x008)
#define GRX500_GSWT_MDCCFG_1_RES        BIT(15)
#define GRX500_GSWT_MDCCFG_1_MCEN       BIT(8)
#define GRX500_GSWT_MDCCFG_1_FREQ_97KHZ 0xff
#define GRX500_GSWT_ANEG_EEE(port)      (GRX500_GSW_TOP + 0x046 + 4 * ((port) - 1))
#define GRX500_GSWT_ANEG_EEE_CLK_STOP   GENMASK(3, 2)

static const u16 grx500_gphy_bootreg[] = {
	0x0228, 0x0238, 0x0248, 0x0258, 0x0268,
};
static bool grx500_gphy_ready;

static u32 grx500_gswip_read(void __iomem *base, u16 word)
{
	u32 val = __raw_readl(base + (u32)word * 4);

	rmb();
	return val;
}

static void grx500_gswip_write(void __iomem *base, u16 word, u32 value)
{
	wmb();
	__raw_writel(value, base + (u32)word * 4);
}

static void grx500_gswip_update_bits(void __iomem *base, u16 word,
				     u32 mask, u32 value)
{
	u32 old = grx500_gswip_read(base, word);

	grx500_gswip_write(base, word, (old & ~mask) | (value & mask));
}

struct grx500_gphy_mem {
	struct device *dev;
	void *cpu;
	dma_addr_t dma;
};

static void grx500_gphy_free(void *data)
{
	struct grx500_gphy_mem *mem = data;

	dma_unmap_single(mem->dev, mem->dma, GRX500_GPHY_FW_SIZE,
			 DMA_TO_DEVICE);
	kfree(mem->cpu);
}

static int grx500_gphy_boot(struct platform_device *pdev, void __iomem *base)
{
	struct grx500_gphy_mem *mem;
	const struct firmware *fw;
	const char *name;
	dma_addr_t entry;
	unsigned int i;
	void *fw_cpu;
	int ret;

	ret = of_property_read_string(pdev->dev.of_node,
				      "firmware-name", &name);
	if (ret)
		return 0;
	ret = request_firmware(&fw, name, &pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "GPHY firmware unavailable\n");
	if (fw->size != GRX500_GPHY_FW_SIZE) {
		dev_err(&pdev->dev, "GPHY firmware size %zu is not 65536\n",
			fw->size);
		release_firmware(fw);
		return -EINVAL;
	}
	mem = devm_kzalloc(&pdev->dev, sizeof(*mem), GFP_KERNEL);
	if (!mem) {
		release_firmware(fw);
		return -ENOMEM;
	}
	mem->dev = &pdev->dev;
	mem->cpu = kmalloc(GRX500_GPHY_FW_SIZE + GRX500_GPHY_ALIGN,
			   GFP_KERNEL);
	if (!mem->cpu) {
		release_firmware(fw);
		return -ENOMEM;
	}
	fw_cpu = PTR_ALIGN(mem->cpu, GRX500_GPHY_ALIGN);
	memcpy(fw_cpu, fw->data, fw->size);
	release_firmware(fw);
	mem->dma = dma_map_single(&pdev->dev, fw_cpu, GRX500_GPHY_FW_SIZE,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(&pdev->dev, mem->dma)) {
		kfree(mem->cpu);
		return -EIO;
	}
	ret = devm_add_action_or_reset(&pdev->dev, grx500_gphy_free, mem);
	if (ret)
		return ret;
	entry = mem->dma;
	if (entry > U32_MAX || !IS_ALIGNED(entry, GRX500_GPHY_ALIGN)) {
		dev_err(&pdev->dev, "GPHY DMA address %pad is not 16 KiB aligned below 4 GiB\n",
			&entry);
		return -ERANGE;
	}

	for (i = 0; i < ARRAY_SIZE(grx500_gphy_bootreg); i++) {
		struct reset_control *reset;
		char reset_name[8];
		u16 offset = grx500_gphy_bootreg[i];

		snprintf(reset_name, sizeof(reset_name), "phy%u", i);
		reset = devm_reset_control_get_exclusive(&pdev->dev, reset_name);
		if (IS_ERR(reset))
			return dev_err_probe(&pdev->dev, PTR_ERR(reset),
					     "missing GPHY reset\n");
		ret = reset_control_assert(reset);
		if (ret)
			return ret;
		__raw_writel(entry & 0xffff, base + 0x3c00 + offset);
		__raw_writel(entry >> 16, base + 0x3c00 + offset + 4);
		wmb();
		ret = reset_control_deassert(reset);
		if (ret)
			return ret;
		msleep(100);
		dev_info(&pdev->dev, "GPHY%u booted from DMA address %pad\n",
			 i, &entry);
	}
	WRITE_ONCE(grx500_gphy_ready, true);
	return 0;
}

static int grx500_mdio_wait(struct mii_bus *bus, const char *op, int phy, int reg)
{
	void __iomem *base = bus->priv;
	u32 val;
	int ret;

	ret = read_poll_timeout(__raw_readl, val,
				!(val & GRX500_MDCTRL_BUSY),
				20, 100000, false,
				base + GRX500_MDCTRL);
	if (ret)
		dev_err(&bus->dev,
			"MDIO %s timeout phy=%d reg=0x%02x mdctrl=0x%08x mdccfg1=0x%08x\n",
			op, phy, reg, __raw_readl(base + GRX500_MDCTRL),
			grx500_gswip_read(base, GRX500_GSWT_MDCCFG_1));
	return ret;
}

static int grx500_mdio_read(struct mii_bus *bus, int phy, int reg)
{
	void __iomem *base = bus->priv;
	int ret;

	if (phy < 0 || phy > 31 || reg < 0 || reg > 31)
		return -EINVAL;
	ret = grx500_mdio_wait(bus, "read-pre", phy, reg);
	if (ret)
		return ret;
	__raw_writel(GRX500_MDCTRL_READ | (phy << 5) | reg,
		     base + GRX500_MDCTRL);
	wmb();
	(void)__raw_readl(base + GRX500_MDCTRL);
	/* MDC runs at 97.6 kHz (10.24 us/cycle); wait >1 cycle for BUSY to latch. */
	udelay(12);
	ret = grx500_mdio_wait(bus, "read-post", phy, reg);
	if (ret)
		return ret;
	rmb();
	return __raw_readl(base + GRX500_MDREAD) & 0xffff;
}

static int grx500_mdio_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	void __iomem *base = bus->priv;
	int ret;

	if (phy < 0 || phy > 31 || reg < 0 || reg > 31)
		return -EINVAL;
	ret = grx500_mdio_wait(bus, "write-pre", phy, reg);
	if (ret)
		return ret;
	__raw_writel(val, base + GRX500_MDWRITE);
	wmb();
	__raw_writel(GRX500_MDCTRL_WRITE | (phy << 5) | reg,
		     base + GRX500_MDCTRL);
	wmb();
	(void)__raw_readl(base + GRX500_MDCTRL);
	udelay(12);
	return grx500_mdio_wait(bus, "write-post", phy, reg);
}

static void grx500_mdio_hw_init(void __iomem *base, bool right)
{
	u8 max_port = right ? 1 : 5;
	u8 port;

	/* Disable hardware MDIO auto-polling left enabled by bootloader; Linux
	 * phylib owns PHY polling and updates switch MAC link state manually.
	 */
	grx500_gswip_write(base, GRX500_GSWT_MDCCFG_0, 0);

	/* Match GSWIP 3.0 MDIO clock (97.6 kHz), enable MDC, and reset block. */
	grx500_gswip_write(base, GRX500_GSWT_MDCCFG_1,
			   GRX500_GSWT_MDCCFG_1_MCEN |
			   GRX500_GSWT_MDCCFG_1_FREQ_97KHZ);
	grx500_gswip_update_bits(base, GRX500_GSWT_MDCCFG_1,
				 GRX500_GSWT_MDCCFG_1_RES,
				 GRX500_GSWT_MDCCFG_1_RES);

	/* Override EEE clock-stop capability on all external switch ports. */
	for (port = 1; port <= max_port; port++)
		grx500_gswip_update_bits(base, GRX500_GSWT_ANEG_EEE(port),
					 GRX500_GSWT_ANEG_EEE_CLK_STOP,
					 GRX500_GSWT_ANEG_EEE_CLK_STOP);
}

static int grx500_mdio_probe(struct platform_device *pdev)
{
	struct mii_bus *bus;
	void __iomem *base;
	bool right;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	right = of_property_read_bool(pdev->dev.of_node,
				      "lantiq,wait-for-gphy");
	if (right && !READ_ONCE(grx500_gphy_ready))
		return -EPROBE_DEFER;
	ret = grx500_gphy_boot(pdev, base);
	if (ret)
		return ret;
	grx500_mdio_hw_init(base, right);
	bus = devm_mdiobus_alloc(&pdev->dev);
	if (!bus)
		return -ENOMEM;
	bus->name = "GRX500 GSWIP MDIO";
	bus->read = grx500_mdio_read;
	bus->write = grx500_mdio_write;
	bus->priv = base;
	bus->parent = &pdev->dev;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(&pdev->dev));

	ret = of_mdiobus_register(bus, pdev->dev.of_node);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to register MDIO bus\n");
	platform_set_drvdata(pdev, bus);
	dev_info(&pdev->dev, "GRX500 GSWIP MDIO bus registered\n");
	return 0;
}

static void grx500_mdio_remove(struct platform_device *pdev)
{
	mdiobus_unregister(platform_get_drvdata(pdev));
}

static const struct of_device_id grx500_mdio_of_match[] = {
	{ .compatible = "lantiq,grx500-gswip-mdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, grx500_mdio_of_match);

static struct platform_driver grx500_mdio_driver = {
	.probe = grx500_mdio_probe,
	.remove = grx500_mdio_remove,
	.driver = {
		.name = "grx500-gswip-mdio",
		.of_match_table = grx500_mdio_of_match,
	},
};
module_platform_driver(grx500_mdio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GRX500 GSWIP 3.0 MDIO controller");
