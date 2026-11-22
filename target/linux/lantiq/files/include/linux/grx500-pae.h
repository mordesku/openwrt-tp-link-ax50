/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GRX500_PAE_H
#define _LINUX_GRX500_PAE_H

#include <linux/if_ether.h>
#include <linux/types.h>

struct mii_bus;
struct grx500_pae_ctx;

#define GRX500_PAE_RT_EXTID_TCP		0
#define GRX500_PAE_RT_EXTID_UDP		100

enum grx500_pae_mode {
	GRX500_PAE_MODE_NULL = 0,
	GRX500_PAE_MODE_ROUTING = 1,
	GRX500_PAE_MODE_NAT = 2,
	GRX500_PAE_MODE_NAPT = 3,
};

struct grx500_pae_flow {
	bool is_ipv6;
	u32 src_ip[4];
	u32 dst_ip[4];
	u16 src_port;
	u16 dst_port;
	u8 ext_id;

	enum grx500_pae_mode mode;
	u32 nat_ip[4];
	u16 nat_port;
	u8 src_mac[ETH_ALEN];
	u8 dst_mac[ETH_ALEN];
	bool use_src_mac;
	bool use_dst_mac;
	u16 mtu;

	u16 dst_port_map;
	u16 dst_subif_id;
	u8 fid;
	u8 flow_id;
	bool ttl_dec;
	bool pppoe_en;
	u16 pppoe_sid;
	u8 dir;
};

struct grx500_pae_ctx *grx500_pae_init(struct mii_bus *gswip_r);
void grx500_pae_destroy(struct grx500_pae_ctx *ctx);
int grx500_pae_enable_pce_rules(struct grx500_pae_ctx *ctx);
int grx500_pae_session_add(struct grx500_pae_ctx *ctx,
			   const struct grx500_pae_flow *flow,
			   u16 *session_idx);
int grx500_pae_session_del(struct grx500_pae_ctx *ctx, u16 session_idx);
int grx500_pae_session_hit_stats(struct grx500_pae_ctx *ctx, u16 session_idx,
				 bool clear, bool *hit, u32 *packets);
void grx500_pae_debug_regs(struct grx500_pae_ctx *ctx, u32 *gctrl0, u32 *gctrl1,
			   u32 *tfcr_num, u16 *tfcr0_k12, u16 *tfcr0_v9,
			   bool *tfcr0_vld);

#endif
