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
#define GRX500_PCE_VCTRL(port)		(0x485 + 10 * (port))
#define GRX500_PCE_VCTRL_UVR		BIT(0)
#define GRX500_PCE_VCTRL_VIMR		BIT(3)
#define GRX500_PCE_VCTRL_VEMR		BIT(4)
#define GRX500_PCE_GCTRL		0x456
#define GRX500_PCE_GCTRL_MTFL		BIT(0)
#define GRX500_PCE_GCTRL_MC_VALID	BIT(3)
#define GRX500_PCE_GCTRL_VLAN		BIT(14)
#define GRX500_PCE_TFCR_NUM		0x46f
#define GRX500_PCE_PMAP2		0x454
#define GRX500_PCE_PMAP3		0x455
#define GRX500_PCE_PORT_FORWARDING	7
#define GRX500_PCE_TBL_CLEAR_FIRST	0x42a
#define GRX500_PCE_TBL_CLEAR_LAST	0x449
#define GRX500_PCE_TBL_KEY(x)		(0x447 - (x))
#define GRX500_PCE_TBL_MASK		0x448
#define GRX500_PCE_TBL_VAL(x)		(0x44d - (x))
#define GRX500_PCE_TBL_VAL0		0x44d
#define GRX500_PCE_TBL_ADDR		0x44e
#define GRX500_PCE_TBL_CTRL		0x44f
#define GRX500_PCE_TBL_BUSY		BIT(15)
#define GRX500_PCE_TBL_TYPE		BIT(13)
#define GRX500_PCE_TBL_VLD		BIT(12)
#define GRX500_PCE_TBL_GMAP_MASK	GENMASK(10, 7)
#define GRX500_PCE_TBL_OPMOD_MASK	GENMASK(6, 5)
#define GRX500_PCE_TBL_OPMOD_ADRD	0x00
#define GRX500_PCE_TBL_OPMOD_ADWR	0x20
#define GRX500_PCE_TBL_OPMOD_KSRD	0x40
#define GRX500_PCE_TBL_OPMOD_KSWR	0x60
#define GRX500_PCE_TBL_ADDR_MASK	GENMASK(4, 0)
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
static DEFINE_MUTEX(grx500_gswip_pce_lock);

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
				 GRX500_PCE_GCTRL_MC_VALID |
				 (right ? 0 : GRX500_PCE_GCTRL_VLAN),
				 GRX500_PCE_GCTRL_MC_VALID |
				 (right ? 0 : GRX500_PCE_GCTRL_VLAN));
	grx500_gswip_update_bits(base, GRX500_BM_QUEUE_GCTRL, BIT(10), 0);
	for (port = 0; port <= (right ? 15 : 5); port++) {
		grx500_gswip_update_bits(base, GRX500_BM_PCFG + 2 * port,
					 BIT(0), BIT(0));
		grx500_gswip_update_bits(base, GRX500_BM_RMON_CTRL + 2 * port,
					 BIT(2), BIT(2));
	}
	if (!right) {
		for (port = 2; port <= 5; port++) {
			struct grx500_gswip_pce_entry vlan = {
				.index = port + 1,
				.table = GRX500_PCE_TBL_ACTIVE_VLAN,
				.val = { port + 1 },
				.valid = true,
			};

			ret = grx500_gswip_pce_write(bus, false, &vlan);
			if (ret)
				return ret;

			memset(&vlan, 0, sizeof(vlan));
			vlan.index = port + 1;
			vlan.table = GRX500_PCE_TBL_VLAN_MAPPING;
			vlan.val[1] = BIT(port) | BIT(0);
			ret = grx500_gswip_pce_write(bus, false, &vlan);
			if (ret)
				return ret;
		}
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

int grx500_gswip_pce_read(struct mii_bus *bus, bool right,
			  struct grx500_gswip_pce_entry *entry)
{
	void __iomem *base;
	u32 ctrl;
	u16 addr_mode;
	int i, ret;

	if (!grx500_gswip_bus_valid(bus, right) || !entry)
		return -EINVAL;
	base = bus->priv;
	addr_mode = entry->key_mode ? GRX500_PCE_TBL_OPMOD_KSRD :
				      GRX500_PCE_TBL_OPMOD_ADRD;

	mutex_lock(&grx500_gswip_pce_lock);
	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, base, GRX500_PCE_TBL_CTRL);
	if (ret)
		goto out;

	grx500_gswip_write(base, GRX500_PCE_TBL_ADDR, entry->index);
	grx500_gswip_update_bits(base, GRX500_PCE_TBL_CTRL,
				 GRX500_PCE_TBL_ADDR_MASK |
				 GRX500_PCE_TBL_OPMOD_MASK |
				 GRX500_PCE_TBL_BUSY,
				 (entry->table & GRX500_PCE_TBL_ADDR_MASK) |
				 addr_mode | GRX500_PCE_TBL_BUSY);

	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, base, GRX500_PCE_TBL_CTRL);
	if (ret)
		goto out;

	for (i = 0; i < ARRAY_SIZE(entry->key); i++)
		entry->key[i] = grx500_gswip_read(base, GRX500_PCE_TBL_KEY(i));
	for (i = 0; i < ARRAY_SIZE(entry->val); i++)
		entry->val[i] = grx500_gswip_read(base, GRX500_PCE_TBL_VAL(i));
	entry->mask = grx500_gswip_read(base, GRX500_PCE_TBL_MASK);

	ctrl = grx500_gswip_read(base, GRX500_PCE_TBL_CTRL);
	entry->type = !!(ctrl & GRX500_PCE_TBL_TYPE);
	entry->valid = !!(ctrl & GRX500_PCE_TBL_VLD);
	entry->gmap = (ctrl & GRX500_PCE_TBL_GMAP_MASK) >> 7;
out:
	mutex_unlock(&grx500_gswip_pce_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(grx500_gswip_pce_read);

int grx500_gswip_pce_write(struct mii_bus *bus, bool right,
			   const struct grx500_gswip_pce_entry *entry)
{
	void __iomem *base;
	u32 ctrl;
	u16 addr_mode;
	int i, ret;

	if (!grx500_gswip_bus_valid(bus, right) || !entry)
		return -EINVAL;
	base = bus->priv;
	addr_mode = entry->key_mode ? GRX500_PCE_TBL_OPMOD_KSWR :
				      GRX500_PCE_TBL_OPMOD_ADWR;

	mutex_lock(&grx500_gswip_pce_lock);
	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, base, GRX500_PCE_TBL_CTRL);
	if (ret)
		goto out;

	grx500_gswip_write(base, GRX500_PCE_TBL_ADDR, entry->index);
	for (i = 0; i < ARRAY_SIZE(entry->key); i++)
		grx500_gswip_write(base, GRX500_PCE_TBL_KEY(i), entry->key[i]);
	for (i = 0; i < ARRAY_SIZE(entry->val); i++)
		grx500_gswip_write(base, GRX500_PCE_TBL_VAL(i), entry->val[i]);
	grx500_gswip_write(base, GRX500_PCE_TBL_MASK, entry->mask);

	ctrl = (entry->table & GRX500_PCE_TBL_ADDR_MASK) | addr_mode |
	       GRX500_PCE_TBL_BUSY | ((entry->gmap << 7) & GRX500_PCE_TBL_GMAP_MASK);
	if (entry->type)
		ctrl |= GRX500_PCE_TBL_TYPE;
	if (entry->valid)
		ctrl |= GRX500_PCE_TBL_VLD;

	grx500_gswip_write(base, GRX500_PCE_TBL_CTRL, ctrl);
	ret = read_poll_timeout(grx500_gswip_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, base, GRX500_PCE_TBL_CTRL);
out:
	mutex_unlock(&grx500_gswip_pce_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(grx500_gswip_pce_write);

int grx500_gswip_set_stp_state(struct mii_bus *bus, bool right, u8 port,
			       u8 stp_state)
{
	void __iomem *base;

	if (!grx500_gswip_bus_valid(bus, right) || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;

	if (stp_state == 0xff) {
		grx500_gswip_update_bits(base, GRX500_SDMA_PCTRL + 6 * port,
					 BIT(0), 0);
		return 0;
	}

	grx500_gswip_update_bits(base, GRX500_SDMA_PCTRL + 6 * port,
				 BIT(0), BIT(0));
	grx500_gswip_update_bits(base, GRX500_PCE_PCTRL + 10 * port,
				 GENMASK(2, 0), stp_state & GENMASK(2, 0));
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_set_stp_state);

int grx500_gswip_set_pmap(struct mii_bus *bus, bool right, u16 port_mask)
{
	void __iomem *base;

	if (!grx500_gswip_bus_valid(bus, right))
		return -EINVAL;
	base = bus->priv;

	grx500_gswip_write(base, GRX500_PCE_PMAP2, port_mask | BIT(0));
	grx500_gswip_write(base, GRX500_PCE_PMAP3, port_mask | BIT(0));
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_set_pmap);

int grx500_gswip_set_vlan_ctrl(struct mii_bus *bus, bool right, u8 port,
			       bool vlan_aware)
{
	void __iomem *base;
	u32 vctrl = vlan_aware ? (GRX500_PCE_VCTRL_UVR |
				  GRX500_PCE_VCTRL_VIMR |
				  GRX500_PCE_VCTRL_VEMR) : 0;

	if (!grx500_gswip_bus_valid(bus, right) || port > (right ? 15 : 5))
		return -EINVAL;
	base = bus->priv;

	grx500_gswip_update_bits(base, GRX500_PCE_VCTRL(port),
				 GRX500_PCE_VCTRL_UVR |
				 GRX500_PCE_VCTRL_VIMR |
				 GRX500_PCE_VCTRL_VEMR, vctrl);
	return 0;
}
EXPORT_SYMBOL_GPL(grx500_gswip_set_vlan_ctrl);

int grx500_gswip_flush_fdb(struct mii_bus *bus, bool right)
{
	void __iomem *base;
	u32 ctrl;

	if (!grx500_gswip_bus_valid(bus, right))
		return -EINVAL;
	base = bus->priv;

	grx500_gswip_update_bits(base, GRX500_PCE_GCTRL,
				 GRX500_PCE_GCTRL_MTFL,
				 GRX500_PCE_GCTRL_MTFL);
	return read_poll_timeout(grx500_gswip_read, ctrl,
				 !(ctrl & GRX500_PCE_GCTRL_MTFL), 1, 1000,
				 false, base, GRX500_PCE_GCTRL);
}
EXPORT_SYMBOL_GPL(grx500_gswip_flush_fdb);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GRX500 GSWIP 3.0 switch fabric helpers");
