// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel / Lantiq GRX500 GSWIP 3.0 DSA switch driver
 */

#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/ethtool.h>
#include <linux/grx500-gswip.h>
#include <linux/if_bridge.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <net/dsa.h>

#define GRX500_DSA_NUM_PORTS		6
#define GRX500_DSA_MAX_MTU		1528
#define GRX500_DSA_MAX_VLANS		64
#define GRX500_DSA_BR_FID		1
#define GRX500_DSA_MAC_BRIDGE_SIZE	2048
#define GRX500_DSA_MAC_FID_MASK		GENMASK(5, 0)
#define GRX500_DSA_MAC_PORT_MASK	GENMASK(7, 4)
#define GRX500_DSA_MAC_STATIC		BIT(0)

struct grx500_dsa_port_cfg {
	u8 switch_id;
	u8 switch_port;
	u8 rmon_port;
};

static const struct grx500_dsa_port_cfg grx500_dsa_ports[GRX500_DSA_NUM_PORTS] = {
	[0] = { .switch_id = 0, .switch_port = 0, .rmon_port = 0 },  /* cpu */
	[1] = { .switch_id = 1, .switch_port = 1, .rmon_port = 15 }, /* wan */
	[2] = { .switch_id = 0, .switch_port = 2, .rmon_port = 2 },  /* lan4 */
	[3] = { .switch_id = 0, .switch_port = 3, .rmon_port = 3 },  /* lan3 */
	[4] = { .switch_id = 0, .switch_port = 4, .rmon_port = 4 },  /* lan2 */
	[5] = { .switch_id = 0, .switch_port = 5, .rmon_port = 5 },  /* lan1 */
};

static const char grx500_dsa_rmon_names[GRX500_GSWIP_RMON_STATS][ETH_GSTRING_LEN] = {
	"gswip_tx_broadcast",
	"gswip_rx_broadcast",
	"gswip_tx_unicast",
	"gswip_rx_unicast",
	"gswip_tx_drop",
	"gswip_rx_tail_drop",
	"gswip_rx_filter_drop",
	"gswip_rx_crc_error",
	"gswip_rx_oversize_good",
	"gswip_rx_undersize_bad",
};

struct grx500_dsa_vlan {
	bool active;
	u16 vid;
	u16 port_mask;
	u16 untag_mask;
};

struct grx500_dsa {
	struct dsa_switch ds;
	struct device *dev;
	struct mii_bus *gswip[2];
	u16 br_port_mask;
	bool vlan_filtering;
	struct grx500_dsa_vlan vlans[GRX500_DSA_MAX_VLANS];
};

static void grx500_dsa_release_buses(void *data)
{
	struct grx500_dsa *priv = data;
	int i;

	for (i = 0; i < 2; i++) {
		if (priv->gswip[i])
			put_device(&priv->gswip[i]->dev);
	}
}

static int grx500_dsa_find_buses(struct platform_device *pdev,
				 struct grx500_dsa *priv)
{
	static const char * const names[2] = {
		"lantiq,gswip-left",
		"lantiq,gswip-right",
	};
	int i;

	for (i = 0; i < 2; i++) {
		struct device_node *np;

		np = of_parse_phandle(pdev->dev.of_node, names[i], 0);
		if (!np) {
			grx500_dsa_release_buses(priv);
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "Missing %s phandle\n", names[i]);
		}
		priv->gswip[i] = of_mdio_find_bus(np);
		of_node_put(np);
		if (!priv->gswip[i]) {
			grx500_dsa_release_buses(priv);
			return -EPROBE_DEFER;
		}
	}

	return devm_add_action_or_reset(&pdev->dev,
					grx500_dsa_release_buses, priv);
}

static enum dsa_tag_protocol grx500_dsa_get_tag_protocol(struct dsa_switch *ds,
							 int port,
							 enum dsa_tag_protocol mp)
{
	return DSA_TAG_PROTO_GRX500;
}

static int grx500_dsa_setup(struct dsa_switch *ds)
{
	ds->configure_vlan_while_not_filtering = true;
	return 0;
}

static int grx500_dsa_port_enable(struct dsa_switch *ds, int port,
				  struct phy_device *phy)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;

	if (port <= 0 || port >= GRX500_DSA_NUM_PORTS)
		return 0;

	if (phy) {
		/* Switch MAC LPI is disabled; disable EEE in phylib state and
		 * GPHY advertisement so phy_start() cannot re-advertise EEE.
		 */
		phy->eee_broken_modes = -1;
		linkmode_zero(phy->advertising_eee);
		phy->eee_cfg.eee_enabled = false;
		phy->eee_cfg.tx_lpi_enabled = false;
		if (phy_write_mmd(phy, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0) == 0)
			genphy_restart_aneg(phy);
	}

	cfg = &grx500_dsa_ports[port];
	return grx500_gswip_enable_port(priv->gswip[cfg->switch_id],
					cfg->switch_id, cfg->switch_port);
}

static void grx500_dsa_port_disable(struct dsa_switch *ds, int port)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;

	if (port <= 0 || port >= GRX500_DSA_NUM_PORTS)
		return;

	cfg = &grx500_dsa_ports[port];
	grx500_gswip_set_mac_link(priv->gswip[cfg->switch_id],
				  cfg->switch_id, cfg->switch_port,
				  SPEED_UNKNOWN, DUPLEX_UNKNOWN, 0);
	grx500_gswip_disable_port(priv->gswip[cfg->switch_id],
				  cfg->switch_id, cfg->switch_port);
}

static struct phylink_pcs *grx500_dsa_phylink_mac_select_pcs(struct dsa_switch *ds,
							     int port,
							     phy_interface_t iface)
{
	return NULL;
}

static void grx500_dsa_phylink_get_caps(struct dsa_switch *ds, int port,
					struct phylink_config *config)
{
	config->mac_capabilities = MAC_10 | MAC_100 | MAC_1000FD | MAC_1000HD |
				   MAC_SYM_PAUSE | MAC_ASYM_PAUSE;

	if (port == 0) {
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_GMII,
			  config->supported_interfaces);
	} else if (port == 1) {
		__set_bit(PHY_INTERFACE_MODE_RGMII,
			  config->supported_interfaces);
	} else if (port < GRX500_DSA_NUM_PORTS) {
		__set_bit(PHY_INTERFACE_MODE_GMII,
			  config->supported_interfaces);
	}
}

static void grx500_dsa_phylink_mac_link_up(struct dsa_switch *ds, int port,
					   unsigned int mode,
					   phy_interface_t interface,
					   struct phy_device *phydev,
					   int speed, int duplex,
					   bool tx_pause, bool rx_pause)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;

	if (port <= 0 || port >= GRX500_DSA_NUM_PORTS)
		return;

	cfg = &grx500_dsa_ports[port];
	grx500_gswip_set_mac_link(priv->gswip[cfg->switch_id],
				  cfg->switch_id, cfg->switch_port,
				  speed, duplex, tx_pause || rx_pause);
}

static void grx500_dsa_phylink_mac_link_down(struct dsa_switch *ds, int port,
					     unsigned int mode,
					     phy_interface_t interface)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;

	if (port <= 0 || port >= GRX500_DSA_NUM_PORTS)
		return;

	cfg = &grx500_dsa_ports[port];
	grx500_gswip_set_mac_link(priv->gswip[cfg->switch_id],
				  cfg->switch_id, cfg->switch_port,
				  SPEED_UNKNOWN, DUPLEX_UNKNOWN, 0);
}

static int grx500_dsa_port_max_mtu(struct dsa_switch *ds, int port)
{
	return GRX500_DSA_MAX_MTU;
}

static int grx500_dsa_port_change_mtu(struct dsa_switch *ds, int port,
				      int new_mtu)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;

	if (port <= 0 || port >= GRX500_DSA_NUM_PORTS)
		return 0;

	cfg = &grx500_dsa_ports[port];
	return grx500_gswip_set_mtu(priv->gswip[cfg->switch_id],
				    cfg->switch_id, cfg->switch_port, new_mtu);
}

static int grx500_dsa_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;
	return GRX500_GSWIP_RMON_STATS;
}

static void grx500_dsa_get_strings(struct dsa_switch *ds, int port,
				   u32 stringset, u8 *data)
{
	if (stringset == ETH_SS_STATS)
		memcpy(data, grx500_dsa_rmon_names,
		       sizeof(grx500_dsa_rmon_names));
}

static void grx500_dsa_get_ethtool_stats(struct dsa_switch *ds, int port,
					 u64 *data)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;

	if (port < 0 || port >= GRX500_DSA_NUM_PORTS) {
		memset(data, 0, sizeof(u64) * GRX500_GSWIP_RMON_STATS);
		return;
	}

	cfg = &grx500_dsa_ports[port];
	if (grx500_gswip_get_rmon_stats(priv->gswip[cfg->switch_id],
					cfg->switch_id, cfg->rmon_port, data))
		memset(data, 0, sizeof(u64) * GRX500_GSWIP_RMON_STATS);
}

static int grx500_dsa_write_vlan_slot(struct grx500_dsa *priv, u16 slot,
				      u16 vid, u16 fid, u16 port_mask,
				      u16 untag_mask, bool valid)
{
	struct grx500_gswip_pce_entry entry = {
		.index = slot,
		.table = GRX500_PCE_TBL_ACTIVE_VLAN,
		.key = { vid },
		.val = { fid },
		.valid = valid,
	};
	int ret;

	ret = grx500_gswip_pce_write(priv->gswip[0], false, &entry);
	if (ret || !valid)
		return ret;

	memset(&entry, 0, sizeof(entry));
	entry.index = slot;
	entry.table = GRX500_PCE_TBL_VLAN_MAPPING;
	entry.val[0] = vid;
	entry.val[1] = port_mask;
	entry.val[2] = untag_mask;
	return grx500_gswip_pce_write(priv->gswip[0], false, &entry);
}

static void grx500_dsa_port_fast_age(struct dsa_switch *ds, int port);

static int grx500_dsa_port_bridge_join(struct dsa_switch *ds, int port,
				       struct dsa_bridge bridge,
				       bool *tx_fwd_offload,
				       struct netlink_ext_ack *extack)
{
	struct grx500_dsa *priv = ds->priv;
	int ret;

	/* Never bridge GSWIP-R (wan, port 1) into GSWIP-L hardware switching. */
	if (port <= 1 || port >= GRX500_DSA_NUM_PORTS)
		return -EOPNOTSUPP;

	priv->br_port_mask |= BIT(port);
	if (!priv->vlan_filtering) {
		ret = grx500_dsa_write_vlan_slot(priv, port + 1, 0, port + 1,
						 0, 0, false);
		if (ret)
			return ret;
		ret = grx500_dsa_write_vlan_slot(priv, GRX500_DSA_BR_FID, 0,
						 GRX500_DSA_BR_FID,
						 priv->br_port_mask | BIT(0),
						 0, true);
		if (ret)
			return ret;
	}

	ret = grx500_gswip_set_pmap(priv->gswip[0], false, priv->br_port_mask);
	if (ret)
		return ret;

	ret = grx500_gswip_set_port_learning(priv->gswip[0], false, port, true);
	if (!ret)
		*tx_fwd_offload = true;
	return ret;
}

static void grx500_dsa_port_bridge_leave(struct dsa_switch *ds, int port,
					 struct dsa_bridge bridge)
{
	struct grx500_dsa *priv = ds->priv;

	if (port <= 1 || port >= GRX500_DSA_NUM_PORTS)
		return;

	grx500_gswip_set_port_learning(priv->gswip[0], false, port, false);
	grx500_dsa_port_fast_age(ds, port);

	priv->br_port_mask &= ~BIT(port);
	grx500_gswip_set_pmap(priv->gswip[0], false, priv->br_port_mask);

	if (!priv->vlan_filtering) {
		grx500_dsa_write_vlan_slot(priv, GRX500_DSA_BR_FID, 0,
					   GRX500_DSA_BR_FID,
					   priv->br_port_mask | BIT(0),
					   0, !!priv->br_port_mask);
		grx500_dsa_write_vlan_slot(priv, port + 1, 0, port + 1,
					   BIT(port) | BIT(0), 0, true);
	}
}

static void grx500_dsa_port_stp_state_set(struct dsa_switch *ds, int port,
					  u8 state)
{
	struct grx500_dsa *priv = ds->priv;
	const struct grx500_dsa_port_cfg *cfg;
	u8 gswip_state;

	if (port <= 0 || port >= GRX500_DSA_NUM_PORTS)
		return;

	switch (state) {
	case BR_STATE_DISABLED:
		gswip_state = 0xff;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		gswip_state = 0;
		break;
	case BR_STATE_LEARNING:
		gswip_state = 3;
		break;
	case BR_STATE_FORWARDING:
		gswip_state = 7;
		break;
	default:
		return;
	}

	cfg = &grx500_dsa_ports[port];
	grx500_gswip_set_stp_state(priv->gswip[cfg->switch_id],
				   cfg->switch_id, cfg->switch_port,
				   gswip_state);
}

static void grx500_dsa_port_fast_age(struct dsa_switch *ds, int port)
{
	struct grx500_dsa *priv = ds->priv;
	struct grx500_gswip_pce_entry mac = { 0 };
	int i;

	if (port <= 1 || port >= GRX500_DSA_NUM_PORTS)
		return;

	for (i = 0; i < GRX500_DSA_MAC_BRIDGE_SIZE; i++) {
		memset(&mac, 0, sizeof(mac));
		mac.table = GRX500_PCE_TBL_MAC_BRIDGE;
		mac.index = i;
		if (grx500_gswip_pce_read(priv->gswip[0], false, &mac))
			return;
		if (!mac.valid || (mac.val[1] & GRX500_DSA_MAC_STATIC))
			continue;
		if (FIELD_GET(GRX500_DSA_MAC_PORT_MASK, mac.val[0]) != port)
			continue;
		mac.valid = false;
		if (grx500_gswip_pce_write(priv->gswip[0], false, &mac))
			return;
	}
}

static int grx500_dsa_port_fdb(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid, bool add)
{
	struct grx500_dsa *priv = ds->priv;
	struct grx500_gswip_pce_entry mac = { 0 };
	u16 fid = GRX500_DSA_BR_FID;
	int i;

	if (port <= 1 || port >= GRX500_DSA_NUM_PORTS)
		return 0;

	if (vid && priv->vlan_filtering) {
		for (i = 8; i < GRX500_DSA_MAX_VLANS; i++) {
			if (priv->vlans[i].active && priv->vlans[i].vid == vid) {
				fid = i;
				break;
			}
		}
	}

	mac.table = GRX500_PCE_TBL_MAC_BRIDGE;
	mac.key_mode = true;
	mac.key[0] = addr[5] | (addr[4] << 8);
	mac.key[1] = addr[3] | (addr[2] << 8);
	mac.key[2] = addr[1] | (addr[0] << 8);
	mac.key[3] = FIELD_PREP(GRX500_DSA_MAC_FID_MASK, fid);
	mac.val[0] = add ? BIT(port) : 0;
	mac.val[1] = GRX500_DSA_MAC_STATIC;
	mac.valid = add;

	return grx500_gswip_pce_write(priv->gswip[0], false, &mac);
}

static int grx500_dsa_port_fdb_add(struct dsa_switch *ds, int port,
				   const unsigned char *addr, u16 vid,
				   struct dsa_db db)
{
	return grx500_dsa_port_fdb(ds, port, addr, vid, true);
}

static int grx500_dsa_port_fdb_del(struct dsa_switch *ds, int port,
				   const unsigned char *addr, u16 vid,
				   struct dsa_db db)
{
	return grx500_dsa_port_fdb(ds, port, addr, vid, false);
}

static int grx500_dsa_port_fdb_dump(struct dsa_switch *ds, int port,
				    dsa_fdb_dump_cb_t *cb, void *data)
{
	struct grx500_dsa *priv = ds->priv;
	struct grx500_gswip_pce_entry mac;
	unsigned char addr[ETH_ALEN];
	int i, err;

	if (port <= 1 || port >= GRX500_DSA_NUM_PORTS)
		return 0;

	for (i = 0; i < GRX500_DSA_MAC_BRIDGE_SIZE; i++) {
		memset(&mac, 0, sizeof(mac));
		mac.table = GRX500_PCE_TBL_MAC_BRIDGE;
		mac.index = i;
		err = grx500_gswip_pce_read(priv->gswip[0], false, &mac);
		if (err)
			return err;
		if (!mac.valid)
			continue;

		addr[5] = mac.key[0] & 0xff;
		addr[4] = (mac.key[0] >> 8) & 0xff;
		addr[3] = mac.key[1] & 0xff;
		addr[2] = (mac.key[1] >> 8) & 0xff;
		addr[1] = mac.key[2] & 0xff;
		addr[0] = (mac.key[2] >> 8) & 0xff;

		if (mac.val[1] & GRX500_DSA_MAC_STATIC) {
			if (mac.val[0] & BIT(port)) {
				err = cb(addr, 0, true, data);
				if (err)
					return err;
			}
		} else if (port == FIELD_GET(GRX500_DSA_MAC_PORT_MASK,
					     mac.val[0])) {
			err = cb(addr, 0, false, data);
			if (err)
				return err;
		}
	}

	return 0;
}

static int grx500_dsa_port_vlan_filtering(struct dsa_switch *ds, int port,
					  bool vlan_filtering,
					  struct netlink_ext_ack *extack)
{
	struct grx500_dsa *priv = ds->priv;
	int p, ret;

	if (port == 0)
		return 0;
	if (port == 1 || port >= GRX500_DSA_NUM_PORTS)
		return -EOPNOTSUPP;

	priv->vlan_filtering = vlan_filtering;
	for (p = 2; p <= 5; p++) {
		ret = grx500_gswip_set_vlan_ctrl(priv->gswip[0], false, p,
						 vlan_filtering);
		if (ret)
			return ret;
	}
	return grx500_gswip_set_vlan_ctrl(priv->gswip[0], false, 0,
					  vlan_filtering);
}

static int grx500_dsa_port_vlan_add(struct dsa_switch *ds, int port,
				    const struct switchdev_obj_port_vlan *vlan,
				    struct netlink_ext_ack *extack)
{
	struct grx500_dsa *priv = ds->priv;
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	int i, slot = -1;

	if (port == 0)
		return 0;
	if (port == 1 || port >= GRX500_DSA_NUM_PORTS)
		return -EOPNOTSUPP;

	for (i = 8; i < GRX500_DSA_MAX_VLANS; i++) {
		if (priv->vlans[i].active && priv->vlans[i].vid == vlan->vid) {
			slot = i;
			break;
		}
		if (!priv->vlans[i].active && slot < 0)
			slot = i;
	}
	if (slot < 0)
		return -ENOSPC;

	priv->vlans[slot].active = true;
	priv->vlans[slot].vid = vlan->vid;
	priv->vlans[slot].port_mask |= BIT(port) | BIT(0);
	if (untagged)
		priv->vlans[slot].untag_mask |= BIT(port);
	else
		priv->vlans[slot].untag_mask &= ~BIT(port);

	return grx500_dsa_write_vlan_slot(priv, slot, vlan->vid, slot,
					  priv->vlans[slot].port_mask,
					  priv->vlans[slot].untag_mask, true);
}

static int grx500_dsa_port_vlan_del(struct dsa_switch *ds, int port,
				    const struct switchdev_obj_port_vlan *vlan)
{
	struct grx500_dsa *priv = ds->priv;
	int i;

	if (port <= 1 || port >= GRX500_DSA_NUM_PORTS)
		return 0;

	for (i = 8; i < GRX500_DSA_MAX_VLANS; i++) {
		if (!priv->vlans[i].active || priv->vlans[i].vid != vlan->vid)
			continue;

		priv->vlans[i].port_mask &= ~BIT(port);
		priv->vlans[i].untag_mask &= ~BIT(port);
		if (!(priv->vlans[i].port_mask & ~BIT(0))) {
			priv->vlans[i].active = false;
			priv->vlans[i].port_mask = 0;
			return grx500_dsa_write_vlan_slot(priv, i, vlan->vid, i,
							  0, 0, false);
		}
		return grx500_dsa_write_vlan_slot(priv, i, vlan->vid, i,
						  priv->vlans[i].port_mask,
						  priv->vlans[i].untag_mask,
						  true);
	}
	return 0;
}

static const struct dsa_switch_ops grx500_dsa_ops = {
	.get_tag_protocol	= grx500_dsa_get_tag_protocol,
	.setup			= grx500_dsa_setup,
	.port_enable		= grx500_dsa_port_enable,
	.port_disable		= grx500_dsa_port_disable,
	.phylink_get_caps	= grx500_dsa_phylink_get_caps,
	.phylink_mac_select_pcs	= grx500_dsa_phylink_mac_select_pcs,
	.phylink_mac_link_up	= grx500_dsa_phylink_mac_link_up,
	.phylink_mac_link_down	= grx500_dsa_phylink_mac_link_down,
	.port_max_mtu		= grx500_dsa_port_max_mtu,
	.port_change_mtu	= grx500_dsa_port_change_mtu,
	.get_sset_count		= grx500_dsa_get_sset_count,
	.get_strings		= grx500_dsa_get_strings,
	.get_ethtool_stats	= grx500_dsa_get_ethtool_stats,
	.port_bridge_join	= grx500_dsa_port_bridge_join,
	.port_bridge_leave	= grx500_dsa_port_bridge_leave,
	.port_stp_state_set	= grx500_dsa_port_stp_state_set,
	.port_fast_age		= grx500_dsa_port_fast_age,
	.port_fdb_add		= grx500_dsa_port_fdb_add,
	.port_fdb_del		= grx500_dsa_port_fdb_del,
	.port_fdb_dump		= grx500_dsa_port_fdb_dump,
	.port_vlan_filtering	= grx500_dsa_port_vlan_filtering,
	.port_vlan_add		= grx500_dsa_port_vlan_add,
	.port_vlan_del		= grx500_dsa_port_vlan_del,
};

static int grx500_dsa_probe(struct platform_device *pdev)
{
	struct grx500_dsa *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	ret = grx500_dsa_find_buses(pdev, priv);
	if (ret)
		return ret;

	priv->ds.dev = &pdev->dev;
	priv->ds.num_ports = GRX500_DSA_NUM_PORTS;
	priv->ds.ops = &grx500_dsa_ops;
	priv->ds.priv = priv;

	platform_set_drvdata(pdev, priv);

	ret = dsa_register_switch(&priv->ds);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register GRX500 DSA switch\n");

	dev_info(&pdev->dev, "GRX500 GSWIP 3.0 DSA switch registered\n");
	return 0;
}

static void grx500_dsa_remove(struct platform_device *pdev)
{
	struct grx500_dsa *priv = platform_get_drvdata(pdev);

	if (priv)
		dsa_unregister_switch(&priv->ds);
}

static void grx500_dsa_shutdown(struct platform_device *pdev)
{
	struct grx500_dsa *priv = platform_get_drvdata(pdev);

	if (priv)
		dsa_switch_shutdown(&priv->ds);
}

static const struct of_device_id grx500_dsa_of_match[] = {
	{ .compatible = "lantiq,grx500-dsa" },
	{ }
};
MODULE_DEVICE_TABLE(of, grx500_dsa_of_match);

static struct platform_driver grx500_dsa_driver = {
	.probe = grx500_dsa_probe,
	.remove_new = grx500_dsa_remove,
	.shutdown = grx500_dsa_shutdown,
	.driver = {
		.name = "grx500-dsa",
		.of_match_table = grx500_dsa_of_match,
	},
};
module_platform_driver(grx500_dsa_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GRX500 GSWIP 3.0 DSA switch driver");
