/* SPDX-License-Identifier: GPL-2.0-only */
/* GRX500 DirectConnect (DC_MODE_0) 3-ring bridge API for WAV654. */
#ifndef _LINUX_GRX500_DCDP_H
#define _LINUX_GRX500_DCDP_H

#include <linux/netdevice.h>
#include <linux/types.h>

struct sk_buff;

#define GRX500_DCDP_EP_WLAN0		8
#define GRX500_DCDP_EP_WLAN1		9
#define GRX500_DCDP_MAX_RADIOS		2
#define GRX500_DCDP_MAX_NETDEVS		4

#define GRX500_DCDP_RING_DESC_SIZE	16U
#define GRX500_DCDP_TXOUT_DESC_SIZE	4U
#define GRX500_DCDP_DEFAULT_UMT_US	200U

typedef int (*grx500_dcdp_get_subif_fn)(struct net_device *dev, const u8 *mac,
					u16 *subif_id);

struct grx500_dcdp_ring {
	void *virt;
	dma_addr_t dma;
	phys_addr_t phys;
	u32 entries;
	u32 entry_size;
};

struct grx500_dcdp_netdev {
	struct net_device *ndev;
	grx500_dcdp_get_subif_fn get_subif;
};

struct grx500_dcdp_port {
	u32 ep_id;
	u32 cbm_pid;
	u32 tmu_port;
	u32 tmu_queue;
	u32 umt_id;
	u32 dma_ctrl_id;
	u32 dma_cid;
	u32 dma_ch;
	bool dma_ch_open;
	struct grx500_dcdp_ring tx_in;	/* soc2dev: CBM DQM desc0 (16B) */
	struct grx500_dcdp_ring tx_out;	/* soc2dev_ret: CBM DQM ptr_rtn (4B) */
	struct grx500_dcdp_ring rx_out;	/* dev2soc: host DMA ring (16B) */
	struct net_device *dev;
	struct grx500_dcdp_netdev netdevs[GRX500_DCDP_MAX_NETDEVS];
	bool shared_res;
	bool active;
};

int grx500_dcdp_register_radio(u32 ep_id, struct net_device *dev,
			       u32 umt_dst_phys, u32 rx_ring_entries,
			       struct grx500_dcdp_port **out_port);
void grx500_dcdp_unregister_radio(struct grx500_dcdp_port *port);
int grx500_dcdp_bind_netdev(struct grx500_dcdp_port *port,
			    struct net_device *ndev,
			    grx500_dcdp_get_subif_fn get_subif);
void grx500_dcdp_unbind_netdev(struct net_device *ndev);
int grx500_dcdp_setup_tc(enum tc_setup_type type, void *type_data);
int grx500_dcdp_xmit(struct grx500_dcdp_port *port, u16 subif_id,
		     struct sk_buff *skb);

#endif /* _LINUX_GRX500_DCDP_H */
