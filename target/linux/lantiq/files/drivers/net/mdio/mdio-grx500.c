// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 GSWIP 3.0 MDIO controller, isolated from the vendor datapath. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/io.h>
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
#define GRX500_GPHY_FW_SIZE              65536
#define GRX500_GPHY_ALIGN                16384

static const u16 grx500_gphy_bootreg[] = {
	0x0228, 0x0238, 0x0248, 0x0258, 0x0268,
};
static bool grx500_gphy_ready;

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
	/* The firmware is CPU-written once and then only read by the GPHYs.
	 * On GRX500 EVA, freeing a dma_alloc_coherent() buffer called vunmap()
	 * on a non-vmalloc alias during deferred probe. A streaming mapping
	 * avoids that mismatched coherent DMA free path.
	 */
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

static int grx500_mdio_wait(void __iomem *base)
{
	int tries;

	for (tries = 0; tries < 10000; tries++) {
		if (!(__raw_readl(base + GRX500_MDCTRL) & GRX500_MDCTRL_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int grx500_mdio_read(struct mii_bus *bus, int phy, int reg)
{
	void __iomem *base = bus->priv;
	int ret;

	if (phy < 0 || phy > 31 || reg < 0 || reg > 31)
		return -EINVAL;
	ret = grx500_mdio_wait(base);
	if (ret)
		return ret;
	__raw_writel(GRX500_MDCTRL_READ | (phy << 5) | reg,
		     base + GRX500_MDCTRL);
	ret = grx500_mdio_wait(base);
	if (ret)
		return ret;
	return __raw_readl(base + GRX500_MDREAD) & 0xffff;
}

static int grx500_mdio_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	void __iomem *base = bus->priv;
	int ret;

	if (phy < 0 || phy > 31 || reg < 0 || reg > 31)
		return -EINVAL;
	ret = grx500_mdio_wait(base);
	if (ret)
		return ret;
	__raw_writel(val, base + GRX500_MDWRITE);
	__raw_writel(GRX500_MDCTRL_WRITE | (phy << 5) | reg,
		     base + GRX500_MDCTRL);
	return grx500_mdio_wait(base);
}

static ssize_t phy_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mii_bus *bus = dev_get_drvdata(dev);
	bool right = of_property_read_bool(dev->of_node,
					   "lantiq,wait-for-gphy");
	int first = right ? 1 : 2;
	int last = right ? 1 : 5;
	int phy, id1, id2, bmsr;
	size_t len = 0;

	for (phy = first; phy <= last; phy++) {
		id1 = mdiobus_read(bus, phy, MII_PHYSID1);
		id2 = mdiobus_read(bus, phy, MII_PHYSID2);
		/* BMSR link is latched low; the second read reports current link. */
		bmsr = mdiobus_read(bus, phy, MII_BMSR);
		if (id1 < 0 || id2 < 0 || bmsr < 0)
			return id1 < 0 ? id1 : (id2 < 0 ? id2 : bmsr);
		bmsr = mdiobus_read(bus, phy, MII_BMSR);
		if (bmsr < 0)
			return bmsr;
		len += sysfs_emit_at(buf, len,
				     "phy=%d id=%04x:%04x bmsr=%04x link=%s\n",
				     phy, id1, id2, bmsr,
				     (id1 == 0xffff && id2 == 0xffff) ? "absent" :
				     (bmsr & BMSR_LSTATUS) ? "up" : "down");
	}
	return len;
}
static DEVICE_ATTR_RO(phy_status);

static int grx500_mdio_probe(struct platform_device *pdev)
{
	struct mii_bus *bus;
	void __iomem *base;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	if (of_property_read_bool(pdev->dev.of_node,
				  "lantiq,wait-for-gphy") &&
	    !READ_ONCE(grx500_gphy_ready))
		return -EPROBE_DEFER;
	/* GRX500 sysctrl registers both switch clocks with always_on=0.
	 * Its cgu_enable()/cgu_disable() intentionally do nothing for those
	 * gates, so no clock-framework operation is required for MDIO.
	 */
	ret = grx500_gphy_boot(pdev, base);
	if (ret)
		return ret;
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
	ret = device_create_file(&pdev->dev, &dev_attr_phy_status);
	if (ret) {
		mdiobus_unregister(bus);
		return ret;
	}
	dev_info(&pdev->dev, "MDIO bus registered; GPHY firmware and packet path are separate\n");
	return 0;
}

static void grx500_mdio_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_phy_status);
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

MODULE_DESCRIPTION("GRX500 GSWIP 3.0 MDIO controller");
MODULE_LICENSE("GPL");
