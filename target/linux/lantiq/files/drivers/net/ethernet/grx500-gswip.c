// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 GSWIP 3.0 switch fabric, PMAC, PCE, MAC and RMON helpers. */
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/grx500-gswip.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy.h>

#include "../mdio/grx500-pce-microcode.h"

/* Vendor GSWIP 3.0 register numbers are 32-bit word offsets. */
#define GRX500_GSW_TOP			0xf00
#define GRX500_GSW_CTRL			GRX500_GSW_TOP
#define GRX500_GSW_CTRL_ENABLE		BIT(15)
#define GRX500_GSWT_PHY_ADDR(port)	(GRX500_GSW_TOP + 0x044 + 4 * ((port) - 1))

#define GRX500_PMAC_TBL_VAL4		0xd40
#define GRX500_PMAC_TBL_ADDR		0xd45
#define GRX500_PMAC_TBL_CTRL		0xd46
#define GRX500_PMAC_TBL_BUSY		BIT(15)
#define GRX500_PMAC_TBL_WRITE		BIT(5)
#define GRX500_PMAC_IGCFG		1
#define GRX500_PMAC_EGCFG		2
#define GRX500_PMAC_CTRL2		0xd05
#define GRX500_PMAC_CTRL4		0xd07

#define GRX500_FDMA_PASR		0xa47
#define GRX500_FDMA_PCTRL		0xa80
#define GRX500_SDMA_PCTRL		0xbc0
#define GRX500_SDMA_PCTRL_FCSIGN	5

#define GRX500_PCE_PCTRL		0x480
#define GRX500_PCE_PCTRL_3		0x483
#define GRX500_PCE_GCTRL		0x456
#define GRX500_PCE_GCTRL_MC_VALID	BIT(3)
#define GRX500_PCE_TFCR_NUM		0x46f
#define GRX500_PCE_PMAP2		0x454
#define GRX500_PCE_PMAP3		0x455
#define GRX500_PCE_PORT_FORWARDING	7
#define GRX500_PCE_TBL_CLEAR_FIRST	0x42a
#define GRX500_PCE_TBL_CLEAR_LAST	0x449
#define GRX500_PCE_TBL_VAL0		0x44d
#define GRX500_PCE_TBL_ADDR		0x44e
#define GRX500_PCE_TBL_CTRL		0x44f
#define GRX500_PCE_TBL_BUSY		BIT(15)
#define GRX500_PCE_TBL_WRITE		BIT(5)

#define GRX500_MAC_FLEN			0x8c5
#define GRX500_MAC_CTRL			0x903
#define GRX500_MAC_PCTRL(port)		(0x903 + 0x0c * ((port) - 1))
#define GRX500_MAC_CTRL2(port)		(0x905 + 0x0c * ((port) - 1))
#define GRX500_MAC_CTRL2_MLEN		BIT(3)
#define GRX500_MAC_CTRL4(port)		(0x907 + 0x0c * ((port) - 1))
#define GRX500_MAC_CTRL4_GWAIT_SHIFT	8
#define GRX500_MAC_CTRL4_LPIEN		BIT(7)
#define GRX500_MAC_CTRL4_WAIT_SHIFT	0

#define GRX500_BM_PCFG			0x080
#define GRX500_BM_RMON_CTRL		0x081
#define GRX500_BM_RAM_VAL1		0x042
#define GRX500_BM_RAM_VAL0		0x043
#define GRX500_BM_RAM_ADDR		0x044
#define GRX500_BM_RAM_CTRL		0x045
#define GRX500_BM_RAM_BUSY		BIT(15)
#define GRX500_BM_QUEUE_GCTRL		0x04a

static DEFINE_MUTEX(grx500_gswip_bm_lock);

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

static bool grx500_gswip_bus_valid(struct mii_bus *bus, bool right)
{
	return bus && bus->priv && bus->parent && bus->parent->of_node &&
	       right == of_property_read_bool(bus->parent->of_node,
					      "lantiq,wait-for-gphy");
}

int grx500_gswip_prepare_pce(struct mii_bus *bus, bool right)
{
	void __iomem *base;
	u32 ctrl;
	int i, port, word, ret;

	if (!grx500_gswip_bus_valid(bus, right))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_GSW_CTRL,
				 GRX500_GSW_CTRL_ENABLE, GRX500_GSW_CTRL_ENABLE);
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
		for (word = GRX500_PCE_TBL_CLEAR_FIRST;
		     word <= GRX500_PCE_TBL_CLEAR_LAST; word++)
			grx500_gswip_write(base, word, 0);
		for (word = 0; word < 4; word++)
			grx500_gswip_write(base, GRX500_PCE_TBL_VAL0 - word,
					   grx500_pce_ucode[i][word]);
		grx500_gswip_write(base, GRX500_PCE_TBL_CTRL,
				   GRX500_PCE_TBL_BUSY | GRX500_PCE_TBL_WRITE);
		ret = read_poll_timeout(grx500_gswip_read, ctrl,
					!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
					false, base, GRX500_PCE_TBL_CTRL);
		if (ret)
			return ret;
		grx500_gswip_write(base, GRX500_PCE_TBL_CTRL, 0);
	}
	grx500_gswip_update_bits(base, GRX500_PCE_GCTRL,
				 GRX500_PCE_GCTRL_MC_VALID,
				 GRX500_PCE_GCTRL_MC_VALID);
	grx500_gswip_update_bits(base, GRX500_BM_QUEUE_GCTRL, BIT(10), 0);
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
	grx500_gswip_write(base, GRX500_PMAC_TBL_CTRL,
			   GRX500_PMAC_TBL_BUSY | GRX500_PMAC_TBL_WRITE | table);
	return read_poll_timeout(grx500_gswip_read, ctrl,
				 !(ctrl & GRX500_PMAC_TBL_BUSY), 1, 1000,
				 false, base, GRX500_PMAC_TBL_CTRL);
}

int grx500_gswip_prepare_pmac(struct mii_bus *bus, bool right)
{
	void __iomem *base;
	u16 value[5];
	u16 addr;
	u8 port, class, flow;
	int ret;

	if (!grx500_gswip_bus_valid(bus, right))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_FDMA_PASR, GENMASK(7, 0), 0);
	if (grx500_gswip_read(base, GRX500_PMAC_CTRL4) & BIT(0))
		return -EOPNOTSUPP;

	if (!right) {
		for (class = 0; class < 16; class++) {
			for (flow = 0; flow < 4; flow++) {
				addr = (flow << 8) | (class << 4);
				memset(value, 0, sizeof(value));
				value[1] = min_t(u8, class, 3);
				value[2] = BIT(0);
				ret = grx500_gswip_pmac_table_write(base, addr,
						GRX500_PMAC_EGCFG, value);
				if (ret)
					return ret;
			}
			memset(value, 0, sizeof(value));
			value[1] = ((class & 8 ? 3 : 1) << 8) | 0x90;
			value[3] = BIT(class & 7);
			value[4] = GENMASK(7, 0);
			ret = grx500_gswip_pmac_table_write(base, class,
						GRX500_PMAC_IGCFG, value);
			if (ret)
				return ret;
		}
		grx500_gswip_write(base, GRX500_PCE_PMAP2, BIT(0));
		grx500_gswip_write(base, GRX500_PCE_PMAP3, BIT(0));
		for (port = 0; port <= 5; port++)
			grx500_gswip_update_bits(base, GRX500_PCE_PCTRL_3 + 10 * port,
						 BIT(15), BIT(15));
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
		grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL, BIT(1), 0);
		grx500_gswip_update_bits(base, GRX500_PCE_PCTRL,
					 BIT(11) | BIT(14), BIT(11) | BIT(14));
		grx500_gswip_update_bits(base, GRX500_MAC_CTRL, BIT(7), BIT(7));

		for (port = 0; port < 16; port++) {
			grx500_gswip_update_bits(base,
				GRX500_FDMA_PCTRL + 6 * port, BIT(1), 0);
			grx500_gswip_update_bits(base,
				GRX500_SDMA_PCTRL + 6 * port,
				BIT(GRX500_SDMA_PCTRL_FCSIGN), 0);
			if (port != 0)
				grx500_gswip_update_bits(base,
					GRX500_PCE_PCTRL + 10 * port,
					BIT(11), 0);
			grx500_gswip_update_bits(base,
				GRX500_PCE_PCTRL_3 + 10 * port,
				BIT(15), BIT(15));
		}
		grx500_gswip_write(base, GRX500_PCE_PMAP2, BIT(0));
		grx500_gswip_write(base, GRX500_PCE_PMAP3, BIT(0));
	}
	grx500_gswip_update_bits(base, GRX500_PMAC_CTRL2, GENMASK(1, 0), 0);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_prepare_pmac);

int grx500_gswip_enable_port(struct mii_bus *bus, bool right, u8 port)
{
	void __iomem *base;

	if (!grx500_gswip_bus_valid(bus, right) || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_PCE_PCTRL + 10 * port,
				 GENMASK(2, 0), GRX500_PCE_PORT_FORWARDING);
	grx500_gswip_update_bits(base, GRX500_SDMA_PCTRL + 6 * port,
				 BIT(0), BIT(0));
	grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL + 6 * port,
				 BIT(0), BIT(0));
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_enable_port);

int grx500_gswip_disable_port(struct mii_bus *bus, bool right, u8 port)
{
	void __iomem *base;

	if (!grx500_gswip_bus_valid(bus, right) || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;
	grx500_gswip_update_bits(base, GRX500_FDMA_PCTRL + 6 * port,
				 BIT(0), 0);
	grx500_gswip_update_bits(base, GRX500_SDMA_PCTRL + 6 * port,
				 BIT(0), 0);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_disable_port);

int grx500_gswip_set_mac_link(struct mii_bus *bus, bool right, u8 port,
			      int speed, int duplex, int pause)
{
	void __iomem *base;
	u32 mac_val = 0;
	u32 phy_addr_val = 0;
	u32 mac_ctrl4_val = (0x32 << GRX500_MAC_CTRL4_GWAIT_SHIFT) |
			    (0x15 << GRX500_MAC_CTRL4_WAIT_SHIFT);

	if (!grx500_gswip_bus_valid(bus, right) ||
	    !port || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;

	if (speed == SPEED_UNKNOWN) {
		grx500_gswip_update_bits(base, GRX500_MAC_PCTRL(port),
					 GENMASK(5, 0), 0);
		grx500_gswip_write(base, GRX500_MAC_CTRL4(port), mac_ctrl4_val);
		grx500_gswip_update_bits(base, GRX500_GSWT_PHY_ADDR(port),
					 GENMASK(14, 5), (2 << 13));
		return 0;
	}

	if (duplex == DUPLEX_FULL) {
		mac_val |= (1 << 2);
		phy_addr_val |= (1 << 9);
	} else {
		mac_val |= (3 << 2);
	}

	if (speed == SPEED_1000) {
		mac_val |= 2;
		phy_addr_val |= (2 << 11);
	} else if (speed == SPEED_100) {
		mac_val |= 1;
		phy_addr_val |= (1 << 11);
	}

	if (pause) {
		mac_val |= (1 << 4);
		phy_addr_val |= (1 << 7) | (1 << 5);
	}

	phy_addr_val |= (1 << 13);

	grx500_gswip_update_bits(base, GRX500_MAC_PCTRL(port),
				 GENMASK(5, 0), mac_val);
	grx500_gswip_write(base, GRX500_MAC_CTRL4(port), mac_ctrl4_val);
	grx500_gswip_update_bits(base, GRX500_GSWT_PHY_ADDR(port),
				 GENMASK(14, 5), phy_addr_val);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_set_mac_link);

int grx500_gswip_set_mtu(struct mii_bus *bus, bool right, u8 port, int mtu)
{
	void __iomem *base;
	u32 flen = mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;

	if (!grx500_gswip_bus_valid(bus, right) ||
	    !port || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;

	grx500_gswip_update_bits(base, GRX500_MAC_FLEN, GENMASK(13, 0),
				 max_t(u32, flen, 1518));
	grx500_gswip_update_bits(base, GRX500_MAC_CTRL2(port),
				 GRX500_MAC_CTRL2_MLEN,
				 mtu > ETH_DATA_LEN ? GRX500_MAC_CTRL2_MLEN : 0);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_set_mtu);

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

int grx500_gswip_get_rmon_stats(struct mii_bus *bus, bool right, u8 port,
				u64 stats[GRX500_GSWIP_RMON_STATS])
{
	static const u8 indices[GRX500_GSWIP_RMON_STATS] = {
		0x0c, /* tx_broadcast */
		0x1f, /* rx_broadcast */
		0x06, /* tx_unicast */
		0x23, /* rx_unicast */
		0x10, /* tx_drop */
		0x18, /* rx_tail_drop */
		0x19, /* rx_filter_drop */
		0x21, /* rx_crc_error */
		0x1b, /* rx_oversize_good */
		0x1e, /* rx_undersize_bad */
	};
	void __iomem *base;
	int i, ret;

	if (!grx500_gswip_bus_valid(bus, right) || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;

	mutex_lock(&grx500_gswip_bm_lock);
	for (i = 0; i < GRX500_GSWIP_RMON_STATS; i++) {
		u32 val = 0;

		ret = grx500_gswip_rmon_read(base, port, indices[i], &val);
		if (ret) {
			mutex_unlock(&grx500_gswip_bm_lock);
			return ret;
		}
		stats[i] = val;
	}
	mutex_unlock(&grx500_gswip_bm_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_get_rmon_stats);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GRX500 GSWIP 3.0 switch fabric helpers");
