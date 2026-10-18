// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 GSWIP 3.0 MDIO controller, isolated from the vendor datapath. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/grx500-gswip.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "grx500-pce-microcode.h"

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
#define GRX500_PMAC_TBL_VAL4             0xd40
#define GRX500_PMAC_TBL_ADDR             0xd45
#define GRX500_PMAC_TBL_CTRL             0xd46
#define GRX500_PMAC_TBL_BUSY             BIT(15)
#define GRX500_PMAC_TBL_WRITE            BIT(5)
#define GRX500_PMAC_IGCFG                1
#define GRX500_PMAC_EGCFG                2
#define GRX500_PMAC_CTRL2                0xd05
#define GRX500_PMAC_CTRL4                0xd07
#define GRX500_FDMA_PASR                 0xa47
#define GRX500_FDMA_PCTRL                0xa80
#define GRX500_SDMA_PCTRL                0xbc0
#define GRX500_SDMA_PCTRL_FCSIGN         5
#define GRX500_PCE_PCTRL                 0x480
#define GRX500_PCE_PCTRL_3               0x483
#define GRX500_MAC_PCTRL(port)           (0x903 + 0x0c * (port))
#define GRX500_MAC_CTRL                  0x903
#define GRX500_GSW_CTRL                  GRX500_GSW_TOP
#define GRX500_GSW_CTRL_ENABLE           BIT(15)
#define GRX500_GSWT_MDCCFG_0             (GRX500_GSW_TOP + 0x007)
#define GRX500_GSWT_MDCCFG_1             (GRX500_GSW_TOP + 0x008)
#define GRX500_GSWT_MII_CFG_1            (GRX500_GSW_TOP + 0x009)
#define GRX500_GSWT_PHY_ADDR_1           (GRX500_GSW_TOP + 0x044)
#define GRX500_GSWT_MDIO_STAT_1          (GRX500_GSW_TOP + 0x045)
#define GRX500_PCE_GCTRL                 0x456
#define GRX500_PCE_GCTRL_MC_VALID        BIT(3)
#define GRX500_PCE_TFCR_NUM              0x46f
#define GRX500_PCE_PMAP2                 0x454
#define GRX500_PCE_PMAP3                 0x455
#define GRX500_PCE_PORT_FORWARDING       7
#define GRX500_PCE_TBL_CLEAR_FIRST       0x42a
#define GRX500_PCE_TBL_CLEAR_LAST        0x449
#define GRX500_PCE_TBL_VAL0              0x44d
#define GRX500_PCE_TBL_ADDR              0x44e
#define GRX500_PCE_TBL_CTRL              0x44f
#define GRX500_PCE_TBL_BUSY              BIT(15)
#define GRX500_PCE_TBL_WRITE             BIT(5)
#define GRX500_PCE_QUEUE_MAP_TABLE       0x11
#define GRX500_PCE_TBL_OPMODE_WRITE      (1 << 5)
#define GRX500_PCE_TBL_VALID             BIT(12)
#define GRX500_BM_PCFG                   0x080
#define GRX500_BM_RMON_CTRL              0x081
#define GRX500_BM_RAM_VAL1               0x042
#define GRX500_BM_RAM_VAL0               0x043
#define GRX500_BM_RAM_ADDR               0x044
#define GRX500_BM_RAM_CTRL               0x045
#define GRX500_BM_RAM_BUSY               BIT(15)
#define GRX500_BM_QUEUE_GCTRL            0x04a

static const u16 grx500_gphy_bootreg[] = {
	0x0228, 0x0238, 0x0248, 0x0258, 0x0268,
};
static bool grx500_gphy_ready;
static DEFINE_MUTEX(grx500_gswip_bm_lock);

static u32 grx500_gswip_read(void __iomem *base, u16 word)
{
	return __raw_readl(base + (u32)word * 4);
}

static void grx500_gswip_write(void __iomem *base, u16 word, u32 value)
{
	__raw_writel(value, base + (u32)word * 4);
}

static void grx500_gswip_update_bits(void __iomem *base, u16 word,
				      u32 mask, u32 value)
{
	u32 old = grx500_gswip_read(base, word);

	grx500_gswip_write(base, word, (old & ~mask) | (value & mask));
}

static int grx500_gswip_pmac_table_read(void __iomem *base, u16 addr,
					u8 table, u16 value[5])
{
	u32 ctrl;
	int i, ret;

	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PMAC_TBL_BUSY), 1, 1000,
				false, base, GRX500_PMAC_TBL_CTRL);
	if (ret)
		return ret;
	grx500_gswip_write(base, GRX500_PMAC_TBL_ADDR, addr);
	grx500_gswip_write(base, GRX500_PMAC_TBL_CTRL,
			   GRX500_PMAC_TBL_BUSY | table);
	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PMAC_TBL_BUSY), 1, 1000,
				false, base, GRX500_PMAC_TBL_CTRL);
	if (ret)
		return ret;
	for (i = 0; i < 5; i++)
		value[i] = grx500_gswip_read(base, GRX500_PMAC_TBL_VAL4 + 4 - i);
	return 0;
}

/* Mirror gsw_pmicro_code_init() on GSWIP 3.0. The packet path owns this
 * operation: it is done before SDMA/FDMA ports are enabled, not at MDIO probe.
 */
int grx500_gswip_prepare_pce(struct mii_bus *bus, bool right)
{
	void __iomem *base;
	u32 ctrl;
	int i, port, word, ret;

	if (!bus || !bus->priv || !bus->parent || !bus->parent->of_node ||
	    right != of_property_read_bool(bus->parent->of_node,
					   "lantiq,wait-for-gphy"))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_GSW_CTRL,
				 GRX500_GSW_CTRL_ENABLE, GRX500_GSW_CTRL_ENABLE);
	/* Vendor GSW_HW_Init() and GSW_Enable() reserve 128 common flow
	 * entries on GSWIP-R before enabling its packet path.
	 */
	if (right)
		grx500_gswip_update_bits(base, GRX500_PCE_TFCR_NUM,
					 GENMASK(7, 0), 0x80);
	grx500_gswip_update_bits(base, GRX500_PCE_GCTRL,
				 GRX500_PCE_GCTRL_MC_VALID, 0);
	for (i = 0; i < ARRAY_SIZE(grx500_pce_ucode); i++) {
		ret = read_poll_timeout(grx500_gswip_read, ctrl,
					!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
					false, base, GRX500_PCE_TBL_CTRL);
		if (ret)
			return ret;
		grx500_gswip_write(base, GRX500_PCE_TBL_ADDR, i);
		/* The vendor zeroes key, mask and unused values on each row. */
		for (word = GRX500_PCE_TBL_CLEAR_FIRST;
		     word <= GRX500_PCE_TBL_CLEAR_LAST; word++)
			grx500_gswip_write(base, word, 0);
		for (word = 0; word < 4; word++)
			grx500_gswip_write(base, GRX500_PCE_TBL_VAL0 - word,
					   grx500_pce_ucode[i][word]);
		wmb();
		grx500_gswip_write(base, GRX500_PCE_TBL_CTRL,
				   GRX500_PCE_TBL_BUSY | GRX500_PCE_TBL_WRITE);
		ret = read_poll_timeout(grx500_gswip_read, ctrl,
					!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
					false, base, GRX500_PCE_TBL_CTRL);
		if (ret)
			return ret;
		/* Vendor gsw_pce_table_write() clears the full control word
		 * after each completed access; leaving OPMOD=write selected is
		 * a real startup-sequence difference on GSWIP-R.
		 */
		grx500_gswip_write(base, GRX500_PCE_TBL_CTRL, 0);
	}
	grx500_gswip_update_bits(base, GRX500_PCE_GCTRL,
				 GRX500_PCE_GCTRL_MC_VALID,
				 GRX500_PCE_GCTRL_MC_VALID);
	/* Last step of vendor gsw_pmicro_code_init(): use the normal BM
	 * queue mode before any SDMA/FDMA port is enabled.
	 */
	grx500_gswip_update_bits(base, GRX500_BM_QUEUE_GCTRL, BIT(10), 0);
	/* Vendor gsw_pmicro_code_init() enables RMON on every switch port. */
	for (port = 0; port <= (right ? 15 : 5); port++) {
		grx500_gswip_update_bits(base, GRX500_BM_PCFG + 2 * port,
					 BIT(0), BIT(0));
		grx500_gswip_update_bits(base, GRX500_BM_RMON_CTRL + 2 * port,
					 BIT(2), BIT(2));
	}
	dev_info(bus->parent, "GSWIP PCE parser loaded (%zu rows); switch enabled\n",
		 ARRAY_SIZE(grx500_pce_ucode));
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_prepare_pce);

/* GPL gsw_flow_core.c:xwayflow_pmac_table_write(), with a bounded wait.
 * The five 16-bit data words occupy ascending word addresses 0xd40..0xd44.
 */
static int grx500_gswip_pmac_table_write(void __iomem *base, u16 addr,
					 u8 table, const u16 value[5])
{
	u32 ctrl;
	int i, ret;

	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PMAC_TBL_BUSY), 1, 1000,
				false, base, GRX500_PMAC_TBL_CTRL);
	if (ret)
		return ret;
	grx500_gswip_write(base, GRX500_PMAC_TBL_ADDR, addr);
	for (i = 0; i < 5; i++)
		grx500_gswip_write(base, GRX500_PMAC_TBL_VAL4 + i,
				   value[4 - i]);
	wmb();
	grx500_gswip_write(base, GRX500_PMAC_TBL_CTRL,
			   GRX500_PMAC_TBL_BUSY | GRX500_PMAC_TBL_WRITE | table);
	return read_poll_timeout(grx500_gswip_read, ctrl,
				 !(ctrl & GRX500_PMAC_TBL_BUSY), 1, 1000,
				 false, base, GRX500_PMAC_TBL_CTRL);
}

/* Port the unaccelerated PMAC lookup tables from vendor
 * ltq_eth_drv_xrx500.c:ltq_gsw_pmac_init(). This is not run at MDIO probe:
 * the CBM owner must activate it only as part of the complete packet path.
 */
int grx500_gswip_prepare_pmac(struct mii_bus *bus, bool right)
{
	void __iomem *base;
	u16 value[5];
	u16 addr;
	u8 port, class, flow;
	int ret;

	if (!bus || !bus->priv || !bus->parent || !bus->parent->of_node)
		return -EINVAL;
	if (right != of_property_read_bool(bus->parent->of_node,
					    "lantiq,wait-for-gphy"))
		return -EINVAL;
	base = bus->priv;
	/* No acceleration parser prefix: RX always begins with the 8-byte PMAC.
	 * Vendor GSW_CPU_PortCfgSet uses FDMA_PASR bits 7:0 for CPU/MPE modes.
	 */
	grx500_gswip_update_bits(base, GRX500_FDMA_PASR, GENMASK(7, 0), 0);
	/* The vendor table indexing below uses traffic class, not flags. */
	if (grx500_gswip_read(base, GRX500_PMAC_CTRL4) & BIT(0))
		return -EOPNOTSUPP;

	if (!right) {
		for (class = 0; class < 16; class++) {
			for (flow = 0; flow < 4; flow++) {
				addr = (flow << 8) | (class << 4);
				memset(value, 0, sizeof(value));
				value[1] = min_t(u8, class, 3);
				value[2] = BIT(0); /* PMAC header present */
				ret = grx500_gswip_pmac_table_write(base, addr,
						GRX500_PMAC_EGCFG, value);
				if (ret)
					return ret;
			}
			memset(value, 0, sizeof(value));
			value[1] = ((class & 8 ? 3 : 1) << 8) | 0x90;
			value[3] = BIT(class & 7);
			value[4] = BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) |
				   BIT(5) | BIT(6) | BIT(7);
			ret = grx500_gswip_pmac_table_write(base, class,
						GRX500_PMAC_IGCFG, value);
			if (ret)
				return ret;
		}
		/* Separate Linux interfaces: unknown traffic goes only to CPU.
		 * Disable hardware learning so software bridge/routing owns the FDB.
		 * CPU TX still uses the explicit PMAC destination port bitmap.
		 */
		grx500_gswip_write(base, GRX500_PCE_PMAP2, BIT(0));
		grx500_gswip_write(base, GRX500_PCE_PMAP3, BIT(0));
		for (port = 0; port <= 5; port++)
			grx500_gswip_update_bits(base, GRX500_PCE_PCTRL_3 + 10 * port,
						 BIT(15), BIT(15));
		/* GSWIP-L CPU port 0: ingress special tag, no egress tag. */
		grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL, BIT(1), 0);
		grx500_gswip_update_bits(base, GRX500_PCE_PCTRL, BIT(11), BIT(11));
	} else {
		for (port = 0; port < 16; port++) {
			for (class = 0; class < 16; class++) {
				for (flow = 0; flow < 4; flow++) {
					addr = port | (class << 4) | (flow << 8);
					memset(value, 0, sizeof(value));
					value[1] = port <= 13 ? 0 : 5;
					value[2] = BIT(0) |
						   (port == 13 ? BIT(1) : 0);
					ret = grx500_gswip_pmac_table_write(base,
						addr, GRX500_PMAC_EGCFG, value);
					if (ret)
						return ret;
				}
			}
			memset(value, 0, sizeof(value));
			/* GSWIP-R PMAC Ingress Configuration Table:
			 * Matches vendor ltq_gsw_pmac_init(): channels 0-3 and
			 * channel 15 enable the default port-map selectors (bits 4 and 6).
			 * For DMA1TX15 (channel 15), defPmacHdr[6..7] is 0x8000
			 * (GSWIP-R WAN logical port 15).
			 */
			value[1] = port == 15 ? 0x80 : port << 12;
			value[3] = port == 15 ? 0x8000 : 0xffff;
			value[4] = BIT(0) | BIT(7);
			if (port == 15)
				value[4] |= BIT(1);
			if (port < 4 || port == 15)
				value[4] |= BIT(4) | BIT(6);
			ret = grx500_gswip_pmac_table_write(base, port,
						GRX500_PMAC_IGCFG, value);
			if (ret)
				return ret;
		}
		/* GSWIP-R CPU port 0: ingress special tag, no egress tag, SPFDIS. */
		grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL, BIT(1), 0);
		grx500_gswip_update_bits(base, GRX500_PCE_PCTRL,
					 BIT(11) | BIT(14), BIT(11) | BIT(14));
		grx500_gswip_update_bits(base, GRX500_MAC_CTRL, BIT(7), BIT(7));

		/* GSWIP-R TOP link and PHY interface setup:
		 * Enable MDIO clock (FREQ=0xFF), enable MCEN, deassert reset (RES).
		 * Enable MDIO hardware auto-polling for PHY 1 (PEN_1).
		 * Configure Port 1 link: 1000M full duplex (0x3201).
		 * Configure Port 1 MAC_CTRL: PADEN, FCS, FDUP, GMII 1000M (0x186).
		 */
		grx500_gswip_write(base, GRX500_GSWT_MDCCFG_1, 0x81ff);
		grx500_gswip_update_bits(base, GRX500_GSWT_MDCCFG_0, GENMASK(6, 1), BIT(1));
		grx500_gswip_write(base, GRX500_GSWT_PHY_ADDR_1, 0x3201);
		grx500_gswip_write(base, GRX500_MAC_PCTRL(1), 0x186);

		for (port = 0; port < 16; port++) {
			/* No egress special tag on physical or forward ports. */
			grx500_gswip_update_bits(base,
				GRX500_FDMA_PCTRL + 6 * port, BIT(1), 0);
			/* Vendor GSW_CPU_PortCfgSet(bFcsCheck=0) explicitly
			 * disables SDMA FCS checking on every CPU-facing port.
			 */
			grx500_gswip_update_bits(base,
				GRX500_SDMA_PCTRL + 6 * port,
				BIT(GRX500_SDMA_PCTRL_FCSIGN), 0);
			/* Wire/external ports (including Port 1 WAN) must NOT
			 * expect an ingress PMAC special tag (IGSTEN=0). Only CPU
			 * port 0 has IGSTEN enabled above.
			 */
			if (port != 0)
				grx500_gswip_update_bits(base,
					GRX500_PCE_PCTRL + 10 * port,
					BIT(11), 0);
			/* Vendor GSW_PortCfgSet(bLearning=1) sets LNDIS. */
			grx500_gswip_update_bits(base,
				GRX500_PCE_PCTRL_3 + 10 * port,
				BIT(15), BIT(15));
		}
		/* Vendor ltq_gsw_pmac_init() sends unknown traffic to port 0
		 * on GSWIP-R after the LAN-switch DMA P2P handoff.
		 */
		grx500_gswip_write(base, GRX500_PCE_PMAP2, BIT(0));
		grx500_gswip_write(base, GRX500_PCE_PMAP3, BIT(0));
	}
	/* Match the vendor's disabled short-frame PMAC length check. */
	grx500_gswip_update_bits(base, GRX500_PMAC_CTRL2, GENMASK(1, 0), 0);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_prepare_pmac);

/* reset.c deliberately clears SDMA/FDMA enable on both switches. Re-enable
 * only after the CBM/DMA packet path can receive and recycle every frame.
 */
int grx500_gswip_enable_port(struct mii_bus *bus, bool right, u8 port)
{
	void __iomem *base;

	if (!bus || !bus->priv || !bus->parent || !bus->parent->of_node ||
	    right != of_property_read_bool(bus->parent->of_node,
					   "lantiq,wait-for-gphy") ||
	    port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_PCE_PCTRL + 10 * port,
				 GENMASK(2, 0), GRX500_PCE_PORT_FORWARDING);
	grx500_gswip_update_bits(base, GRX500_SDMA_PCTRL + 6 * port,
				 BIT(0), BIT(0));
	grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL + 6 * port,
				 BIT(0), BIT(0));
	if (right && port == 1) {
		grx500_gswip_write(base, GRX500_MAC_PCTRL(1), 0x186);
		grx500_gswip_write(base, GRX500_GSWT_PHY_ADDR_1, 0x3201);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_enable_port);

int grx500_gswip_disable_port(struct mii_bus *bus, bool right, u8 port)
{
	void __iomem *base;

	if (!bus || !bus->priv || !bus->parent || !bus->parent->of_node ||
	    right != of_property_read_bool(bus->parent->of_node,
					   "lantiq,wait-for-gphy") ||
	    port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL + 6 * port,
				 BIT(0), 0);
	grx500_gswip_update_bits(base, GRX500_SDMA_PCTRL + 6 * port,
				 BIT(0), 0);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_disable_port);

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
		/* Match the vendor GSWIP 3.0 read/write sequence: wait before
		 * even the first busy read. An immediate idle observation after
		 * a command may otherwise expose the previous MDIO result and
		 * report a false PHY link transition. Keep the bounded timeout.
		 */
		udelay(1);
		if (!(__raw_readl(base + GRX500_MDCTRL) & GRX500_MDCTRL_BUSY))
			return 0;
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
		if (bmsr >= 0)
			bmsr = mdiobus_read(bus, phy, MII_BMSR);

		len += sysfs_emit_at(buf, len,
				     "phy=%d id=%04x:%04x bmsr=%04x link=%s\n",
				     phy,
				     id1 >= 0 ? (u16)id1 : 0xffff,
				     id2 >= 0 ? (u16)id2 : 0xffff,
				     bmsr >= 0 ? (u16)bmsr : 0,
				     (id1 < 0 || id2 < 0 || (id1 == 0xffff && id2 == 0xffff)) ? "absent" :
				     ((bmsr >= 0) && (bmsr & BMSR_LSTATUS)) ? "up" : "down");
	}
	return len;
}
static DEVICE_ATTR_RO(phy_status);

static ssize_t switch_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mii_bus *bus = dev_get_drvdata(dev);
	void __iomem *base = bus->priv;
	u8 port = 5;
	u16 ingress[5], egress[5], wan_egress[5], wan_ingress[5];
	ssize_t len;
	int ret;

	ret = grx500_gswip_pmac_table_read(base, 0, GRX500_PMAC_IGCFG,
					      ingress);
	if (ret)
		return ret;
	ret = grx500_gswip_pmac_table_read(base, 0, GRX500_PMAC_EGCFG,
					      egress);
	if (ret)
		return ret;
	ret = grx500_gswip_pmac_table_read(base, 15, GRX500_PMAC_IGCFG,
					      wan_ingress);
	if (ret)
		return ret;
	/* Endpoint 15 is the vendor Ethernet-WAN destination.  Reading its
	 * first egress entry makes the PMAC route observable without dumping
	 * the complete 4096-entry table.
	 */
	ret = grx500_gswip_pmac_table_read(base, 15, GRX500_PMAC_EGCFG,
					      wan_egress);
	if (ret)
		return ret;

	len = sysfs_emit(buf,
		"gsw_ctrl=%08x pce_gctrl=%08x pce_table_ctrl=%08x "
		"cpu_sdma=%08x cpu_fdma=%08x port%u_sdma=%08x port%u_fdma=%08x "
		"wan_port1_sdma=%08x wan_port1_fdma=%08x\n"
		"pce_pmap2=%08x pce_pmap3=%08x pce_port0=%08x pce_port%u=%08x "
		"wan_pce_port1=%08x tfcr_num=%08x bm_queue_gctrl=%08x\n",
		grx500_gswip_read(base, GRX500_GSW_CTRL),
		grx500_gswip_read(base, GRX500_PCE_GCTRL),
		grx500_gswip_read(base, GRX500_PCE_TBL_CTRL),
		grx500_gswip_read(base, GRX500_SDMA_PCTRL),
		grx500_gswip_read(base, GRX500_FDMA_PCTRL),
		port, grx500_gswip_read(base, GRX500_SDMA_PCTRL + 6 * port),
		port, grx500_gswip_read(base, GRX500_FDMA_PCTRL + 6 * port),
		grx500_gswip_read(base, GRX500_SDMA_PCTRL + 6),
		grx500_gswip_read(base, GRX500_FDMA_PCTRL + 6),
		grx500_gswip_read(base, GRX500_PCE_PMAP2),
		grx500_gswip_read(base, GRX500_PCE_PMAP3),
		grx500_gswip_read(base, GRX500_PCE_PCTRL),
		port, grx500_gswip_read(base, GRX500_PCE_PCTRL + 10 * port),
		grx500_gswip_read(base, GRX500_PCE_PCTRL + 10),
		grx500_gswip_read(base, GRX500_PCE_TFCR_NUM),
		grx500_gswip_read(base, GRX500_BM_QUEUE_GCTRL));
	len += sysfs_emit_at(buf, len,
		"pmac_ig0=%04x:%04x:%04x:%04x:%04x pmac_eg0=%04x:%04x:%04x:%04x:%04x "
		"pmac_ig15=%04x:%04x:%04x:%04x:%04x pmac_eg15=%04x:%04x:%04x:%04x:%04x\n",
		ingress[0], ingress[1], ingress[2], ingress[3], ingress[4],
		egress[0], egress[1], egress[2], egress[3], egress[4],
		wan_ingress[0], wan_ingress[1], wan_ingress[2], wan_ingress[3],
		wan_ingress[4],
		wan_egress[0], wan_egress[1], wan_egress[2], wan_egress[3],
		wan_egress[4]);
	len += sysfs_emit_at(buf, len,
		"mdccfg_0=%08x mdccfg_1=%08x mii_cfg1=%08x phy_addr1=%08x mdio_stat1=%08x mac_ctrl0=%08x\n",
		grx500_gswip_read(base, GRX500_GSWT_MDCCFG_0),
		grx500_gswip_read(base, GRX500_GSWT_MDCCFG_1),
		grx500_gswip_read(base, GRX500_GSWT_MII_CFG_1),
		grx500_gswip_read(base, GRX500_GSWT_PHY_ADDR_1),
		grx500_gswip_read(base, GRX500_GSWT_MDIO_STAT_1),
		grx500_gswip_read(base, GRX500_MAC_CTRL));
	return len;
}
static DEVICE_ATTR_RO(switch_status);

/* BM RAM is an indirect, shared read port. Counter indices match the vendor
 * GSW_RMON_Port_Get(): 0x0c/0x1f broadcast, 0x06/0x23 unicast,
 * 0x10 TX drops, 0x18 RX tail drops and 0x19 RX filter drops.
 */
static int grx500_gswip_rmon_read(void __iomem *base, u8 port,
				  u8 index, u32 *count)
{
	u32 ctrl;
	int ret;

	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_BM_RAM_BUSY), 1, 1000,
				false, base, GRX500_BM_RAM_CTRL);
	if (ret)
		return ret;
	grx500_gswip_write(base, GRX500_BM_RAM_ADDR, index);
	grx500_gswip_write(base, GRX500_BM_RAM_CTRL,
			   GRX500_BM_RAM_BUSY | (port >= 8 ? port + 8 : port));
	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_BM_RAM_BUSY), 1, 1000,
				false, base, GRX500_BM_RAM_CTRL);
	if (ret)
		return ret;
	*count = (grx500_gswip_read(base, GRX500_BM_RAM_VAL1) << 16) |
		 grx500_gswip_read(base, GRX500_BM_RAM_VAL0);
	return 0;
}

static ssize_t switch_counters_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mii_bus *bus = dev_get_drvdata(dev);
	void __iomem *base = bus->priv;
	bool right = of_property_read_bool(dev->of_node,
					   "lantiq,wait-for-gphy");
	u8 ports[16];
	u8 indices[] = { 0x0c, 0x1f, 0x06, 0x23, 0x10, 0x18, 0x19,
			 0x21, 0x1b, 0x1e };
	/* Vendor GSW_RMON_Port_Get() size/error and good-byte indices. Read
	 * these for GSWIP-R's P2P ingress port only to distinguish a 2048-byte
	 * DMA frame from a normal ARP frame before changing switch routing.
	 */
	u8 right5_indices[] = { 0x1c, 0x12, 0x13, 0x14, 0x15, 0x16,
				 0x17, 0x24, 0x25 };
	u32 count[ARRAY_SIZE(ports)][ARRAY_SIZE(indices)];
	u32 right5_count[ARRAY_SIZE(right5_indices)];
	size_t len = 0;
	int nports = right ? ARRAY_SIZE(ports) : 6;
	int port, index, ret;

	for (port = 0; port < nports; port++)
		ports[port] = port;
	mutex_lock(&grx500_gswip_bm_lock);
	for (port = 0; port < nports; port++) {
		for (index = 0; index < ARRAY_SIZE(indices); index++) {
			ret = grx500_gswip_rmon_read(base, ports[port],
						     indices[index],
						     &count[port][index]);
			if (ret) {
				mutex_unlock(&grx500_gswip_bm_lock);
				return ret;
			}
		}
	}
	if (right) {
		for (index = 0; index < ARRAY_SIZE(right5_indices); index++) {
			ret = grx500_gswip_rmon_read(base, 5,
						     right5_indices[index],
						     &right5_count[index]);
			if (ret) {
				mutex_unlock(&grx500_gswip_bm_lock);
				return ret;
			}
		}
	}
	mutex_unlock(&grx500_gswip_bm_lock);
	for (port = 0; port < nports; port++)
		len += sysfs_emit_at(buf, len,
			"port%d tx_broadcast=%u rx_broadcast=%u tx_unicast=%u rx_unicast=%u tx_drop=%u rx_tail_drop=%u rx_filter_drop=%u rx_crc_error=%u rx_oversize_good=%u rx_undersize_bad=%u\n",
			ports[port], count[port][0], count[port][1],
			count[port][2], count[port][3], count[port][4],
			count[port][5], count[port][6], count[port][7],
			count[port][8], count[port][9]);
	if (right)
		len += sysfs_emit_at(buf, len,
			"port5_rx_framing oversize_bad=%u size64=%u size65_127=%u size128_255=%u size256_511=%u size512_1023=%u size_gt1023=%u good_bytes=%llu\n",
			right5_count[0], right5_count[1], right5_count[2],
			right5_count[3], right5_count[4], right5_count[5],
			right5_count[6],
			(unsigned long long)(((u64)right5_count[8] << 32) |
					     right5_count[7]));
	return len;
}
static DEVICE_ATTR_RO(switch_counters);

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
	ret = device_create_file(&pdev->dev, &dev_attr_switch_status);
	if (ret) {
		device_remove_file(&pdev->dev, &dev_attr_phy_status);
		mdiobus_unregister(bus);
		return ret;
	}
	ret = device_create_file(&pdev->dev, &dev_attr_switch_counters);
	if (ret) {
		device_remove_file(&pdev->dev, &dev_attr_switch_status);
		device_remove_file(&pdev->dev, &dev_attr_phy_status);
		mdiobus_unregister(bus);
		return ret;
	}
	dev_info(&pdev->dev, "MDIO bus registered; GPHY firmware and packet path are separate; vendor 1us pre-poll delay\n");
	return 0;
}

static void grx500_mdio_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_switch_counters);
	device_remove_file(&pdev->dev, &dev_attr_switch_status);
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
