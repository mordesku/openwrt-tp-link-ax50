// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / Lantiq GRX500 GSWIP 3.0 PMAC DSA tag support
 */

#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/dsa.h>

#include "tag.h"

#define GRX500_NAME		"grx500"
#define GRX500_PMAC_HDR_LEN	8
#define GRX500_WAN_DSA_PORT	1
#define GRX500_WAN_RX_SPPID	15

static struct sk_buff *grx500_tag_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct dsa_port *dp = dsa_user_to_port(dev);
	u8 *pmac;

	skb_push(skb, GRX500_PMAC_HDR_LEN);

	pmac = skb->data;
	memset(pmac, 0, GRX500_PMAC_HDR_LEN);
	pmac[3] = 0x80; /* port_map_en = 1 */

	if (dp->index == GRX500_WAN_DSA_PORT) {
		pmac[4] = 0x40; /* redirect = 1 (GSWIP-R) */
		pmac[6] = 0x80; /* logical port 15 */
	} else {
		pmac[7] = BIT(dp->index); /* GSWIP-L ports 2..5 */
	}

	return skb;
}

static struct sk_buff *grx500_tag_rcv(struct sk_buff *skb,
				      struct net_device *dev)
{
	u8 *pmac;
	u8 rx_id;
	int port;

	if (unlikely(!pskb_may_pull(skb, GRX500_PMAC_HDR_LEN)))
		return NULL;

	pmac = skb->data - ETH_HLEN;
	rx_id = pmac[2] >> 4;
	port = (rx_id == GRX500_WAN_RX_SPPID) ? GRX500_WAN_DSA_PORT : rx_id;

	skb->dev = dsa_conduit_find_user(dev, 0, port);
	if (!skb->dev)
		return NULL;

	skb_pull_rcsum(skb, GRX500_PMAC_HDR_LEN);
	dsa_default_offload_fwd_mark(skb);

	return skb;
}

static const struct dsa_device_ops grx500_netdev_ops = {
	.name = GRX500_NAME,
	.proto = DSA_TAG_PROTO_GRX500,
	.xmit = grx500_tag_xmit,
	.rcv = grx500_tag_rcv,
	.needed_headroom = GRX500_PMAC_HDR_LEN,
};

MODULE_DESCRIPTION("DSA tag driver for GRX500 GSWIP 3.0 switches");
MODULE_LICENSE("GPL");
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_GRX500, GRX500_NAME);

module_dsa_tag_driver(grx500_netdev_ops);
