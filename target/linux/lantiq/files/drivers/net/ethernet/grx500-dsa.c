// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel / Lantiq GRX500 GSWIP 3.0 DSA switch driver
 */

#include <linux/errno.h>
#include <linux/ethtool.h>
#include <linux/grx500-gswip.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <net/dsa.h>

#define GRX500_DSA_NUM_PORTS	6
#define GRX500_DSA_MAX_MTU	1528

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

struct grx500_dsa {
	struct dsa_switch ds;
	struct device *dev;
	struct mii_bus *gswip[2];
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
