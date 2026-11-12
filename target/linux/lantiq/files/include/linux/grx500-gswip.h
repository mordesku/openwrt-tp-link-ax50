/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GRX500_GSWIP_H
#define _LINUX_GRX500_GSWIP_H

#include <linux/types.h>

struct mii_bus;

/* Called by the CBM packet-path owner after switch and DMA initialization. */
int grx500_gswip_prepare_pmac(struct mii_bus *bus, bool right);
int grx500_gswip_prepare_pce(struct mii_bus *bus, bool right);
int grx500_gswip_enable_port(struct mii_bus *bus, bool right, u8 port);
int grx500_gswip_disable_port(struct mii_bus *bus, bool right, u8 port);
int grx500_gswip_set_mac_link(struct mii_bus *bus, bool right, u8 port,
			      int speed, int duplex, int pause);

#endif
