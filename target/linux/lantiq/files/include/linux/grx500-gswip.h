/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GRX500_GSWIP_H
#define _LINUX_GRX500_GSWIP_H

#include <linux/types.h>

struct mii_bus;

#define GRX500_GSWIP_RMON_STATS		10

#define GRX500_PCE_TBL_ACTIVE_VLAN	0x01
#define GRX500_PCE_TBL_VLAN_MAPPING	0x02
#define GRX500_PCE_TBL_MAC_BRIDGE	0x0b

struct grx500_gswip_pce_entry {
	u16 index;
	u16 key[4];
	u16 val[5];
	u16 mask;
	u8 table;
	u8 gmap;
	bool type;
	bool valid;
	bool key_mode;
};

/* Called by the CBM packet-path owner after switch and DMA initialization. */
int grx500_gswip_prepare_pmac(struct mii_bus *bus, bool right);
int grx500_gswip_prepare_pce(struct mii_bus *bus, bool right);
int grx500_gswip_enable_port(struct mii_bus *bus, bool right, u8 port);
int grx500_gswip_disable_port(struct mii_bus *bus, bool right, u8 port);
int grx500_gswip_set_mac_link(struct mii_bus *bus, bool right, u8 port,
			      int speed, int duplex, int pause);
int grx500_gswip_set_mtu(struct mii_bus *bus, bool right, u8 port, int mtu);
int grx500_gswip_get_rmon_stats(struct mii_bus *bus, bool right, u8 port,
				u64 stats[GRX500_GSWIP_RMON_STATS]);
int grx500_gswip_pce_read(struct mii_bus *bus, bool right,
			  struct grx500_gswip_pce_entry *entry);
int grx500_gswip_pce_write(struct mii_bus *bus, bool right,
			   const struct grx500_gswip_pce_entry *entry);
int grx500_gswip_set_stp_state(struct mii_bus *bus, bool right, u8 port,
			       u8 stp_state);
int grx500_gswip_set_pmap(struct mii_bus *bus, bool right, u16 port_mask);
int grx500_gswip_set_vlan_ctrl(struct mii_bus *bus, bool right, u8 port,
			       bool vlan_aware);
int grx500_gswip_set_port_learning(struct mii_bus *bus, bool right, u8 port,
				   bool enable);
int grx500_gswip_flush_fdb(struct mii_bus *bus, bool right);
int grx500_gswip_configure_wlan_port(struct mii_bus *bus, u8 port_id,
				     u8 dma_cid);

#endif
