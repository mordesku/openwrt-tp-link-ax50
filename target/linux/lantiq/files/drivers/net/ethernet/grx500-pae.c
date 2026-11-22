// SPDX-License-Identifier: GPL-2.0-only
/* GRX500 GSWIP-R Packet Acceleration Engine (PAE) hardware routing tables. */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/grx500-pae.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/slab.h>

#define GRX500_PAE_SESSION_SIZE		4096
#define GRX500_PAE_IP_SIZE		2048
#define GRX500_PAE_MAC_SIZE		512
#define GRX500_PAE_PPPOE_SIZE		16
#define GRX500_PAE_MTU_SIZE		8

#define GRX500_PAE_TBL_SESSION		0x00
#define GRX500_PAE_TBL_IP		0x01
#define GRX500_PAE_TBL_MAC		0x02
#define GRX500_PAE_TBL_PPPOE		0x03
#define GRX500_PAE_TBL_MTU		0x06

#define GRX500_PAE_OP_READ		0
#define GRX500_PAE_OP_WRITE		1
#define GRX500_PAE_OP_SESS_NEXT		2
#define GRX500_PAE_OP_SESS_HIT		3

#define GRX500_PCE_TBL_TFLOW		0x0f
#define GRX500_PCE_TBL_PARSER_FLAGS	0x18

#define GRX500_PCE_TBL_MASK_REG(x)	((x) == 0 ? 0x448 : (0x42d - (x)))
#define GRX500_PCE_TBL_KEY(x)		(0x447 - (x))
#define GRX500_PCE_TBL_MASK		0x448
#define GRX500_PCE_TBL_VAL(x)		((x) < 5 ? (0x44d - (x)) : (0x43c - (x)))
#define GRX500_PCE_TBL_ADDR		0x44e
#define GRX500_PCE_TBL_CTRL		0x44f
#define GRX500_PCE_TBL_BUSY		BIT(15)
#define GRX500_PCE_TBL_VLD		BIT(12)
#define GRX500_PCE_TBL_KEYFORM		BIT(11)
#define GRX500_PCE_TBL_OPMOD_ADWR	0x20
#define GRX500_PCE_TBL_ADDR_MASK	GENMASK(4, 0)

#define GRX500_PCE_RTBL_CTRL		0x46d
#define GRX500_PCE_RTBL_BUSY		BIT(15)
#define GRX500_PCE_RTBL_VLD		BIT(12)
#define GRX500_PCE_RTBL_OPMOD_SHIFT	5
#define GRX500_PCE_RTBL_ADDR_MASK	GENMASK(4, 0)

#define GRX500_PAE_RT_EXTID_TCP		0
#define GRX500_PAE_RT_EXTID_UDP		100

#define GRX500_PCE_MSB_IP_FRAGMT	BIT(10)
#define GRX500_PCE_MSB_IPV6_EXTN	BIT(8)
#define GRX500_PCE_MSB_IPV4_OPTNS	BIT(7)
#define GRX500_PCE_MSB_UDP_FST		BIT(5)
#define GRX500_PCE_MSB_TCP		BIT(4)
#define GRX500_PCE_MSB_RT_EXCEP		BIT(3)
#define GRX500_PCE_MSB_INNR_IPV6	BIT(2)
#define GRX500_PCE_MSB_INNR_IPV4	BIT(1)
#define GRX500_PCE_MSB_OUTR_IPV6	BIT(0)

#define GRX500_PCE_LSB_OUTR_IPV4	BIT(15)
#define GRX500_PCE_LSB_CAPWAP		BIT(2)
#define GRX500_PCE_LSB_WOL		BIT(0)

struct grx500_pae_tbl_prog {
	u16 index;
	u16 key[16];
	u16 val[16];
	u16 mask;
	u8 table;
	u8 op_mode;
	bool valid;
};

struct grx500_pae_ip_entry {
	bool valid;
	bool is_ipv6;
	u32 addr[4];
	u16 refcnt;
};

struct grx500_pae_mac_entry {
	u8 mac[ETH_ALEN];
	u16 refcnt;
	bool valid;
} __aligned(2);

struct grx500_pae_mtu_entry {
	bool valid;
	u16 mtsize;
	u16 refcnt;
};

struct grx500_pae_pppoe_entry {
	bool valid;
	u16 sid;
	u16 refcnt;
};

struct grx500_pae_sess_node {
	bool valid;
	bool free_flag;
	s16 prev;
	s16 next;
	u16 hw_next;
	u16 src_ip_idx;
	u16 dst_ip_idx;
	u16 nat_ip_idx;
	u16 smac_idx;
	u16 dmac_idx;
	u16 mtu_idx;
	u16 pppoe_idx;
	bool has_smac;
	bool has_dmac;
	bool has_pppoe;
};

struct grx500_pae_ctx {
	struct mii_bus *bus;
	void __iomem *base;
	struct mutex lock;
	struct grx500_pae_ip_entry ip_tbl[GRX500_PAE_IP_SIZE];
	struct grx500_pae_mac_entry mac_tbl[GRX500_PAE_MAC_SIZE];
	struct grx500_pae_mtu_entry mtu_tbl[GRX500_PAE_MTU_SIZE];
	struct grx500_pae_pppoe_entry pppoe_tbl[GRX500_PAE_PPPOE_SIZE];
	struct grx500_pae_sess_node sess[GRX500_PAE_SESSION_SIZE];
	s16 free_head;
	s16 free_tail;
	u16 free_count;
};

static u32 grx500_pae_read(void __iomem *base, u16 word)
{
	u32 val = __raw_readl(base + (u32)word * 4);

	rmb();
	return val;
}

static void grx500_pae_write(void __iomem *base, u16 word, u32 value)
{
	wmb();
	__raw_writel(value, base + (u32)word * 4);
}

static int grx500_pae_rtbl_read(struct grx500_pae_ctx *ctx,
				struct grx500_pae_tbl_prog *prog)
{
	u32 ctrl;
	int i, ret;

	ret = read_poll_timeout(grx500_pae_read, ctrl,
				!(ctrl & GRX500_PCE_RTBL_BUSY), 1, 1000,
				false, ctx->base, GRX500_PCE_RTBL_CTRL);
	if (ret)
		return ret;

	grx500_pae_write(ctx->base, GRX500_PCE_TBL_ADDR, prog->index);
	ctrl = (prog->table & GRX500_PCE_RTBL_ADDR_MASK) |
	       ((prog->op_mode & 3) << GRX500_PCE_RTBL_OPMOD_SHIFT) |
	       GRX500_PCE_RTBL_BUSY;
	grx500_pae_write(ctx->base, GRX500_PCE_RTBL_CTRL, ctrl);

	ret = read_poll_timeout(grx500_pae_read, ctrl,
				!(ctrl & GRX500_PCE_RTBL_BUSY), 1, 1000,
				false, ctx->base, GRX500_PCE_RTBL_CTRL);
	if (ret)
		return ret;

	for (i = 0; i < 16; i++) {
		prog->key[i] = grx500_pae_read(ctx->base, GRX500_PCE_TBL_KEY(i));
		prog->val[i] = grx500_pae_read(ctx->base, GRX500_PCE_TBL_VAL(i));
	}
	prog->mask = grx500_pae_read(ctx->base, GRX500_PCE_TBL_MASK);
	ctrl = grx500_pae_read(ctx->base, GRX500_PCE_RTBL_CTRL);
	prog->valid = !!(ctrl & GRX500_PCE_RTBL_VLD);
	return 0;
}

static int grx500_pae_rtbl_write(struct grx500_pae_ctx *ctx,
				 const struct grx500_pae_tbl_prog *prog)
{
	u32 ctrl;
	int i, ret;

	ret = read_poll_timeout(grx500_pae_read, ctrl,
				!(ctrl & GRX500_PCE_RTBL_BUSY), 1, 1000,
				false, ctx->base, GRX500_PCE_RTBL_CTRL);
	if (ret)
		return ret;

	grx500_pae_write(ctx->base, GRX500_PCE_TBL_ADDR, prog->index);
	for (i = 0; i < 16; i++) {
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_KEY(i), prog->key[i]);
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_VAL(i), prog->val[i]);
	}
	grx500_pae_write(ctx->base, GRX500_PCE_TBL_MASK, prog->mask);

	ctrl = (prog->table & GRX500_PCE_RTBL_ADDR_MASK) |
	       ((prog->op_mode & 3) << GRX500_PCE_RTBL_OPMOD_SHIFT) |
	       GRX500_PCE_RTBL_BUSY;
	if (prog->valid)
		ctrl |= GRX500_PCE_RTBL_VLD;
	grx500_pae_write(ctx->base, GRX500_PCE_RTBL_CTRL, ctrl);

	return read_poll_timeout(grx500_pae_read, ctrl,
				 !(ctrl & GRX500_PCE_RTBL_BUSY), 1, 1000,
				 false, ctx->base, GRX500_PCE_RTBL_CTRL);
}

static u16 grx500_pae_crc16(const u8 *data, size_t len)
{
	u16 crc = 0;
	size_t i;

	while (len--) {
		crc ^= (u16)(*data++) << 8;
		for (i = 0; i < 8; i++)
			crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
	}
	return crc;
}

static u16 grx500_pae_hash(const struct grx500_pae_flow *flow)
{
	u8 hdata[13];
	u32 src4, dst4;

	if (flow->is_ipv6) {
		src4 = flow->src_ip[0] ^ flow->src_ip[1] ^
		       flow->src_ip[2] ^ flow->src_ip[3];
		dst4 = flow->dst_ip[0] ^ flow->dst_ip[1] ^
		       flow->dst_ip[2] ^ flow->dst_ip[3];
	} else {
		src4 = flow->src_ip[0];
		dst4 = flow->dst_ip[0];
	}

	hdata[0] = (src4 >> 24) & 0xff;
	hdata[1] = (src4 >> 16) & 0xff;
	hdata[2] = (src4 >> 8) & 0xff;
	hdata[3] = src4 & 0xff;
	hdata[4] = (dst4 >> 24) & 0xff;
	hdata[5] = (dst4 >> 16) & 0xff;
	hdata[6] = (dst4 >> 8) & 0xff;
	hdata[7] = dst4 & 0xff;
	hdata[8] = (flow->src_port >> 8) & 0xff;
	hdata[9] = flow->src_port & 0xff;
	hdata[10] = (flow->dst_port >> 8) & 0xff;
	hdata[11] = flow->dst_port & 0xff;
	hdata[12] = flow->ext_id;

	return grx500_pae_crc16(hdata, sizeof(hdata)) & (GRX500_PAE_SESSION_SIZE - 1);
}

static int grx500_pae_ip_alloc(struct grx500_pae_ctx *ctx, bool is_ipv6,
			       const u32 addr[4], u16 *out_idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	int i, free_idx = -1, ret;

	for (i = 0; i < GRX500_PAE_IP_SIZE; i++) {
		if (ctx->ip_tbl[i].valid) {
			if (ctx->ip_tbl[i].is_ipv6 == is_ipv6 &&
			    (!is_ipv6 ? ctx->ip_tbl[i].addr[0] == addr[0] :
			     memcmp(ctx->ip_tbl[i].addr, addr, 16) == 0)) {
				ctx->ip_tbl[i].refcnt++;
				*out_idx = i;
				return 0;
			}
		} else if (free_idx < 0) {
			free_idx = i;
		}
	}
	if (free_idx < 0)
		return -ENOSPC;

	prog.table = GRX500_PAE_TBL_IP;
	prog.index = free_idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	prog.valid = true;
	if (is_ipv6) {
		for (i = 0; i < 4; i++) {
			prog.val[i * 2] = (addr[3 - i] >> 16) & 0xffff;
			prog.val[i * 2 + 1] = addr[3 - i] & 0xffff;
		}
		prog.val[8] = BIT(15);
	} else {
		prog.val[0] = addr[0] & 0xffff;
		prog.val[1] = (addr[0] >> 16) & 0xffff;
	}

	ret = grx500_pae_rtbl_write(ctx, &prog);
	if (ret)
		return ret;

	ctx->ip_tbl[free_idx].valid = true;
	ctx->ip_tbl[free_idx].is_ipv6 = is_ipv6;
	memcpy(ctx->ip_tbl[free_idx].addr, addr, is_ipv6 ? 16 : 4);
	ctx->ip_tbl[free_idx].refcnt = 1;
	*out_idx = free_idx;
	return 0;
}

static void grx500_pae_ip_put(struct grx500_pae_ctx *ctx, u16 idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };

	if (idx >= GRX500_PAE_IP_SIZE || !ctx->ip_tbl[idx].valid)
		return;
	if (--ctx->ip_tbl[idx].refcnt > 0)
		return;

	ctx->ip_tbl[idx].valid = false;
	prog.table = GRX500_PAE_TBL_IP;
	prog.index = idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	grx500_pae_rtbl_write(ctx, &prog);
}

static int grx500_pae_mac_alloc(struct grx500_pae_ctx *ctx, const u8 *mac,
				u16 *out_idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	int i, free_idx = -1, ret;

	for (i = 0; i < GRX500_PAE_MAC_SIZE; i++) {
		if (ctx->mac_tbl[i].valid) {
			if (ether_addr_equal(ctx->mac_tbl[i].mac, mac)) {
				ctx->mac_tbl[i].refcnt++;
				*out_idx = i;
				return 0;
			}
		} else if (free_idx < 0) {
			free_idx = i;
		}
	}
	if (free_idx < 0)
		return -ENOSPC;

	prog.table = GRX500_PAE_TBL_MAC;
	prog.index = free_idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	prog.valid = true;
	prog.val[0] = (mac[4] << 8) | mac[5];
	prog.val[1] = (mac[2] << 8) | mac[3];
	prog.val[2] = (mac[0] << 8) | mac[1];

	ret = grx500_pae_rtbl_write(ctx, &prog);
	if (ret)
		return ret;

	ctx->mac_tbl[free_idx].valid = true;
	ether_addr_copy(ctx->mac_tbl[free_idx].mac, mac);
	ctx->mac_tbl[free_idx].refcnt = 1;
	*out_idx = free_idx;
	return 0;
}

static void grx500_pae_mac_put(struct grx500_pae_ctx *ctx, u16 idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };

	if (idx >= GRX500_PAE_MAC_SIZE || !ctx->mac_tbl[idx].valid)
		return;
	if (--ctx->mac_tbl[idx].refcnt > 0)
		return;

	ctx->mac_tbl[idx].valid = false;
	prog.table = GRX500_PAE_TBL_MAC;
	prog.index = idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	grx500_pae_rtbl_write(ctx, &prog);
}

static int grx500_pae_mtu_alloc(struct grx500_pae_ctx *ctx, u16 mtu,
				u16 *out_idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	u16 mtsize = mtu + 1;
	int i, free_idx = -1, ret;

	for (i = 0; i < GRX500_PAE_MTU_SIZE; i++) {
		if (ctx->mtu_tbl[i].valid) {
			if (ctx->mtu_tbl[i].mtsize == mtsize) {
				ctx->mtu_tbl[i].refcnt++;
				*out_idx = i;
				return 0;
			}
		} else if (free_idx < 0) {
			free_idx = i;
		}
	}
	if (free_idx < 0)
		return -ENOSPC;

	prog.table = GRX500_PAE_TBL_MTU;
	prog.index = free_idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	prog.valid = true;
	prog.val[0] = mtsize;

	ret = grx500_pae_rtbl_write(ctx, &prog);
	if (ret)
		return ret;

	ctx->mtu_tbl[free_idx].valid = true;
	ctx->mtu_tbl[free_idx].mtsize = mtsize;
	ctx->mtu_tbl[free_idx].refcnt = 1;
	*out_idx = free_idx;
	return 0;
}

static void grx500_pae_mtu_put(struct grx500_pae_ctx *ctx, u16 idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };

	if (idx >= GRX500_PAE_MTU_SIZE || !ctx->mtu_tbl[idx].valid)
		return;
	if (--ctx->mtu_tbl[idx].refcnt > 0)
		return;

	ctx->mtu_tbl[idx].valid = false;
	prog.table = GRX500_PAE_TBL_MTU;
	prog.index = idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	grx500_pae_rtbl_write(ctx, &prog);
}

static int grx500_pae_pppoe_alloc(struct grx500_pae_ctx *ctx, u16 sid,
				  u16 *out_idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	int i, free_idx = -1, ret;

	for (i = 0; i < GRX500_PAE_PPPOE_SIZE; i++) {
		if (ctx->pppoe_tbl[i].valid) {
			if (ctx->pppoe_tbl[i].sid == sid) {
				ctx->pppoe_tbl[i].refcnt++;
				*out_idx = i;
				return 0;
			}
		} else if (free_idx < 0) {
			free_idx = i;
		}
	}
	if (free_idx < 0)
		return -ENOSPC;

	prog.table = GRX500_PAE_TBL_PPPOE;
	prog.index = free_idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	prog.valid = true;
	prog.val[0] = sid;

	ret = grx500_pae_rtbl_write(ctx, &prog);
	if (ret)
		return ret;

	ctx->pppoe_tbl[free_idx].valid = true;
	ctx->pppoe_tbl[free_idx].sid = sid;
	ctx->pppoe_tbl[free_idx].refcnt = 1;
	*out_idx = free_idx;
	return 0;
}

static void grx500_pae_pppoe_put(struct grx500_pae_ctx *ctx, u16 idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };

	if (idx >= GRX500_PAE_PPPOE_SIZE || !ctx->pppoe_tbl[idx].valid)
		return;
	if (--ctx->pppoe_tbl[idx].refcnt > 0)
		return;

	ctx->pppoe_tbl[idx].valid = false;
	prog.table = GRX500_PAE_TBL_PPPOE;
	prog.index = idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	grx500_pae_rtbl_write(ctx, &prog);
}

static void grx500_pae_unlink_free(struct grx500_pae_ctx *ctx, u16 idx)
{
	s16 prev = ctx->sess[idx].prev;
	s16 next = ctx->sess[idx].next;

	if (prev >= 0)
		ctx->sess[prev].next = next;
	else
		ctx->free_head = next;

	if (next >= 0)
		ctx->sess[next].prev = prev;
	else
		ctx->free_tail = prev;

	ctx->sess[idx].free_flag = false;
	ctx->free_count--;
}

static void grx500_pae_link_free(struct grx500_pae_ctx *ctx, u16 idx)
{
	ctx->sess[idx].valid = false;
	ctx->sess[idx].free_flag = true;
	ctx->sess[idx].prev = ctx->free_tail;
	ctx->sess[idx].next = -1;
	if (ctx->free_tail >= 0)
		ctx->sess[ctx->free_tail].next = idx;
	else
		ctx->free_head = idx;
	ctx->free_tail = idx;
	ctx->free_count++;
}

int grx500_pae_session_add(struct grx500_pae_ctx *ctx,
			   const struct grx500_pae_flow *flow,
			   u16 *session_idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	u16 src_ip_idx = 0, dst_ip_idx = 0, nat_ip_idx = 0;
	u16 smac_idx = 0, dmac_idx = 0, mtu_idx = 0, pppoe_idx = 0;
	u16 bucket, idx;
	int ret;

	if (!ctx || !flow || !session_idx)
		return -EINVAL;

	mutex_lock(&ctx->lock);

	ret = grx500_pae_ip_alloc(ctx, flow->is_ipv6, flow->dst_ip, &dst_ip_idx);
	if (ret)
		goto err_unlock;
	ret = grx500_pae_ip_alloc(ctx, flow->is_ipv6, flow->src_ip, &src_ip_idx);
	if (ret)
		goto err_dst_ip;
	ret = grx500_pae_ip_alloc(ctx, flow->is_ipv6, flow->nat_ip, &nat_ip_idx);
	if (ret)
		goto err_src_ip;
	ret = grx500_pae_mtu_alloc(ctx, flow->mtu ? flow->mtu : ETH_DATA_LEN,
				   &mtu_idx);
	if (ret)
		goto err_nat_ip;
	if (flow->use_src_mac) {
		ret = grx500_pae_mac_alloc(ctx, flow->src_mac, &smac_idx);
		if (ret)
			goto err_mtu;
	}
	if (flow->use_dst_mac) {
		ret = grx500_pae_mac_alloc(ctx, flow->dst_mac, &dmac_idx);
		if (ret)
			goto err_smac;
	}
	if (flow->pppoe_en) {
		ret = grx500_pae_pppoe_alloc(ctx, flow->pppoe_sid, &pppoe_idx);
		if (ret)
			goto err_dmac;
	}

	bucket = grx500_pae_hash(flow);
	if (!ctx->sess[bucket].valid) {
		idx = bucket;
		grx500_pae_unlink_free(ctx, idx);
		ctx->sess[idx].prev = -1;
		ctx->sess[idx].next = -1;
	} else {
		s16 tail = bucket;

		if (ctx->free_count == 0 || ctx->free_head < 0) {
			ret = -ENOSPC;
			goto err_pppoe;
		}
		while (ctx->sess[tail].next >= 0)
			tail = ctx->sess[tail].next;

		idx = ctx->free_head;
		grx500_pae_unlink_free(ctx, idx);
		ctx->sess[tail].next = idx;
		ctx->sess[idx].prev = tail;
		ctx->sess[idx].next = -1;
		ctx->sess[tail].hw_next = idx;

		prog.table = GRX500_PAE_TBL_SESSION;
		prog.index = tail;
		prog.op_mode = GRX500_PAE_OP_SESS_NEXT;
		prog.key[5] = idx;
		ret = grx500_pae_rtbl_write(ctx, &prog);
		if (ret)
			goto err_pppoe;
		memset(&prog, 0, sizeof(prog));
	}

	ctx->sess[idx].valid = true;
	ctx->sess[idx].hw_next = idx;
	ctx->sess[idx].src_ip_idx = src_ip_idx;
	ctx->sess[idx].dst_ip_idx = dst_ip_idx;
	ctx->sess[idx].nat_ip_idx = nat_ip_idx;
	ctx->sess[idx].smac_idx = smac_idx;
	ctx->sess[idx].dmac_idx = dmac_idx;
	ctx->sess[idx].mtu_idx = mtu_idx;
	ctx->sess[idx].pppoe_idx = pppoe_idx;
	ctx->sess[idx].has_smac = flow->use_src_mac;
	ctx->sess[idx].has_dmac = flow->use_dst_mac;
	ctx->sess[idx].has_pppoe = flow->pppoe_en;

	prog.table = GRX500_PAE_TBL_SESSION;
	prog.index = idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	prog.valid = true;
	prog.key[0] = dst_ip_idx & 0x7ff;
	prog.key[1] = src_ip_idx & 0x7ff;
	prog.key[2] = flow->dst_port;
	prog.key[3] = flow->src_port;
	prog.key[4] = (flow->ext_id & 0xff) | BIT(15);
	prog.key[5] = idx;

	prog.val[0] = flow->dst_port_map;
	prog.val[1] = (flow->dst_subif_id & 0x1fff) << 3;
	prog.val[2] = nat_ip_idx & 0x7ff;
	prog.val[3] = flow->nat_port;
	prog.val[4] = (mtu_idx & 0x7) | ((smac_idx & 0xff) << 8);
	prog.val[5] = dmac_idx & 0x3ff;
	prog.val[6] = pppoe_idx & 0xf;
	prog.val[7] = 0x3f << 8;
	prog.val[8] = (flow->fid & 0x3f) | ((u16)flow->flow_id << 8);
	prog.val[12] = ((flow->dir & 1) << 0) |
		       ((flow->pppoe_en ? 1 : 0) << 1) |
		       ((flow->mode & 3) << 2) |
		       ((flow->ttl_dec ? 1 : 0) << 12);

	ret = grx500_pae_rtbl_write(ctx, &prog);
	if (ret)
		goto err_sess;

	*session_idx = idx;
	mutex_unlock(&ctx->lock);
	return 0;

err_sess:
	grx500_pae_link_free(ctx, idx);
err_pppoe:
	if (flow->pppoe_en)
		grx500_pae_pppoe_put(ctx, pppoe_idx);
err_dmac:
	if (flow->use_dst_mac)
		grx500_pae_mac_put(ctx, dmac_idx);
err_smac:
	if (flow->use_src_mac)
		grx500_pae_mac_put(ctx, smac_idx);
err_mtu:
	grx500_pae_mtu_put(ctx, mtu_idx);
err_nat_ip:
	grx500_pae_ip_put(ctx, nat_ip_idx);
err_src_ip:
	grx500_pae_ip_put(ctx, src_ip_idx);
err_dst_ip:
	grx500_pae_ip_put(ctx, dst_ip_idx);
err_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(grx500_pae_session_add);

int grx500_pae_session_del(struct grx500_pae_ctx *ctx, u16 session_idx)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	struct grx500_pae_sess_node *node;
	int ret;

	if (!ctx || session_idx >= GRX500_PAE_SESSION_SIZE)
		return -EINVAL;

	mutex_lock(&ctx->lock);
	node = &ctx->sess[session_idx];
	if (!node->valid) {
		mutex_unlock(&ctx->lock);
		return -ENOENT;
	}

	if (node->prev >= 0) {
		s16 prev = node->prev;
		s16 next = node->next;

		ctx->sess[prev].next = next;
		if (next >= 0)
			ctx->sess[next].prev = prev;
		ctx->sess[prev].hw_next = (next >= 0) ? next : prev;

		prog.table = GRX500_PAE_TBL_SESSION;
		prog.index = prev;
		prog.op_mode = GRX500_PAE_OP_SESS_NEXT;
		prog.key[5] = ctx->sess[prev].hw_next;
		ret = grx500_pae_rtbl_write(ctx, &prog);
		if (ret) {
			mutex_unlock(&ctx->lock);
			return ret;
		}
		memset(&prog, 0, sizeof(prog));
	}

	prog.table = GRX500_PAE_TBL_SESSION;
	prog.index = session_idx;
	prog.op_mode = GRX500_PAE_OP_WRITE;
	prog.valid = false;
	ret = grx500_pae_rtbl_write(ctx, &prog);

	grx500_pae_ip_put(ctx, node->dst_ip_idx);
	grx500_pae_ip_put(ctx, node->src_ip_idx);
	grx500_pae_ip_put(ctx, node->nat_ip_idx);
	grx500_pae_mtu_put(ctx, node->mtu_idx);
	if (node->has_smac)
		grx500_pae_mac_put(ctx, node->smac_idx);
	if (node->has_dmac)
		grx500_pae_mac_put(ctx, node->dmac_idx);
	if (node->has_pppoe)
		grx500_pae_pppoe_put(ctx, node->pppoe_idx);

	grx500_pae_link_free(ctx, session_idx);
	mutex_unlock(&ctx->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(grx500_pae_session_del);

int grx500_pae_session_hit_stats(struct grx500_pae_ctx *ctx, u16 session_idx,
				 bool clear, bool *hit, u32 *packets)
{
	struct grx500_pae_tbl_prog prog = { 0 };
	int ret;

	if (!ctx || session_idx >= GRX500_PAE_SESSION_SIZE)
		return -EINVAL;

	mutex_lock(&ctx->lock);
	if (!ctx->sess[session_idx].valid) {
		mutex_unlock(&ctx->lock);
		return -ENOENT;
	}

	prog.table = GRX500_PAE_TBL_SESSION;
	prog.index = session_idx;
	prog.op_mode = GRX500_PAE_OP_READ;
	ret = grx500_pae_rtbl_read(ctx, &prog);
	if (ret)
		goto out;

	if (hit)
		*hit = !!(prog.val[12] & BIT(15));
	if (packets)
		*packets = prog.val[10] | ((u32)prog.val[11] << 16);

	if (clear && (prog.val[12] & BIT(15))) {
		prog.val[12] &= ~BIT(15);
		prog.val[10] = 0;
		prog.val[11] = 0;
		prog.op_mode = GRX500_PAE_OP_SESS_HIT;
		ret = grx500_pae_rtbl_write(ctx, &prog);
	}
out:
	mutex_unlock(&ctx->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(grx500_pae_session_hit_stats);

static int grx500_pae_pce_tbl_write(struct grx500_pae_ctx *ctx, u8 table,
				    u16 index, const u16 key[16],
				    const u16 mask[4], const u16 val[16],
				    bool kformat, bool valid)
{
	u32 ctrl;
	int i, ret;

	ret = read_poll_timeout(grx500_pae_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, ctx->base, GRX500_PCE_TBL_CTRL);
	if (ret)
		return ret;

	grx500_pae_write(ctx->base, GRX500_PCE_TBL_ADDR, index);
	for (i = 0; i < 16; i++) {
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_KEY(i),
				 key ? key[i] : 0);
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_VAL(i),
				 val ? val[i] : 0);
	}
	for (i = 0; i < 4; i++)
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_MASK_REG(i),
				 mask ? mask[i] : 0);

	ctrl = (table & GRX500_PCE_TBL_ADDR_MASK) | GRX500_PCE_TBL_OPMOD_ADWR |
	       GRX500_PCE_TBL_BUSY |
	       (kformat ? GRX500_PCE_TBL_KEYFORM : 0) |
	       (valid ? GRX500_PCE_TBL_VLD : 0);
	grx500_pae_write(ctx->base, GRX500_PCE_TBL_CTRL, ctrl);

	ret = read_poll_timeout(grx500_pae_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, ctx->base, GRX500_PCE_TBL_CTRL);
	grx500_pae_write(ctx->base, GRX500_PCE_TBL_CTRL, 0);
	return ret;
}

static int grx500_pae_pce_flags_write(struct grx500_pae_ctx *ctx, u16 index,
				      u16 flag_val, u16 flag_mask)
{
	u16 key[16] = { 0 };
	int i, b;

	/* Expand each 4-bit nibble into a 16-bit one-hot column mask. */
	for (i = 0; i < 4; i++) {
		u8 val_nibble = (flag_val >> (i * 4)) & 0xf;
		u8 mask_nibble = (flag_mask >> (i * 4)) & 0xf;

		for (b = 0; b < 16; b++) {
			if ((b & ~mask_nibble) == (val_nibble & ~mask_nibble))
				key[i] |= BIT(b);
		}
	}

	return grx500_pae_pce_tbl_write(ctx, GRX500_PCE_TBL_PARSER_FLAGS,
					index, key, NULL, NULL, true, true);
}

static int grx500_pae_pce_tfcr_write(struct grx500_pae_ctx *ctx, u16 index,
				     u8 parse_lsb_idx, u8 parse_msb_idx,
				     u8 rt_ext_id)
{
	u16 key[16] = {
		[0] = (0x7f << 8) | 0xff,
		[1] = 0xffff,
		[2] = 0xffff,
		[3] = 0xffff,
		[4] = 0xffff,
		[5] = 0xffff,
		[6] = (0x7f << 8) | 0x0f,
		[7] = (0x1f << 8) | 0x1f,
		[8] = (0x7f << 8) | 0x1f,
		[9] = (0x7f << 8) | 0xff,
		[10] = 0xffff,
		[11] = 0xffff,
		[12] = ((u16)parse_msb_idx << 8) | parse_lsb_idx,
		[13] = 0xffff,
		[14] = 0x1f,
		[15] = 0,
	};
	u16 val[16] = {
		[1] = 0xffff,
		[3] = 0x7f1f,
		/* Enable PAE routing lookup + 5-tuple compare (bits 6:3 = 0). */
		[9] = ((u16)rt_ext_id << 8) | 0x03,
	};

	return grx500_pae_pce_tbl_write(ctx, GRX500_PCE_TBL_TFLOW,
					index, key, NULL, val, false, true);
}

int grx500_pae_enable_pce_rules(struct grx500_pae_ctx *ctx)
{
	u16 msb_excl = GRX500_PCE_MSB_IP_FRAGMT | GRX500_PCE_MSB_IPV6_EXTN |
		       GRX500_PCE_MSB_IPV4_OPTNS | GRX500_PCE_MSB_RT_EXCEP |
		       GRX500_PCE_MSB_INNR_IPV6 | GRX500_PCE_MSB_INNR_IPV4;
	u16 lsb_excl = GRX500_PCE_LSB_CAPWAP | GRX500_PCE_LSB_WOL;
	int ret;

	/* Rule 0: TCP unicast (IPv4 + IPv6) -> RT_EXTID_TCP */
	ret = grx500_pae_pce_flags_write(ctx, 0, GRX500_PCE_MSB_TCP,
					 ~(msb_excl | GRX500_PCE_MSB_TCP));
	if (ret)
		return ret;
	ret = grx500_pae_pce_tfcr_write(ctx, 0, 0xff, 0, GRX500_PAE_RT_EXTID_TCP);
	if (ret)
		return ret;

	/* Rule 1: UDP unicast IPv4 -> RT_EXTID_UDP */
	ret = grx500_pae_pce_flags_write(ctx, 1, GRX500_PCE_MSB_UDP_FST,
					 ~(msb_excl | GRX500_PCE_MSB_UDP_FST));
	if (ret)
		return ret;
	ret = grx500_pae_pce_flags_write(ctx, 2, GRX500_PCE_LSB_OUTR_IPV4,
					 ~(lsb_excl | GRX500_PCE_LSB_OUTR_IPV4));
	if (ret)
		return ret;
	ret = grx500_pae_pce_tfcr_write(ctx, 1, 2, 1, GRX500_PAE_RT_EXTID_UDP);
	if (ret)
		return ret;

	/* Rule 2: UDP unicast IPv6 -> RT_EXTID_UDP */
	ret = grx500_pae_pce_flags_write(ctx, 3,
					 GRX500_PCE_MSB_UDP_FST |
					 GRX500_PCE_MSB_OUTR_IPV6,
					 ~(msb_excl | GRX500_PCE_MSB_UDP_FST |
					   GRX500_PCE_MSB_OUTR_IPV6));
	if (ret)
		return ret;
	ret = grx500_pae_pce_flags_write(ctx, 4, 0, ~lsb_excl);
	if (ret)
		return ret;
	return grx500_pae_pce_tfcr_write(ctx, 2, 4, 3, GRX500_PAE_RT_EXTID_UDP);
}
EXPORT_SYMBOL_GPL(grx500_pae_enable_pce_rules);

struct grx500_pae_ctx *grx500_pae_init(struct mii_bus *gswip_r)
{
	struct grx500_pae_ctx *ctx;
	int i, ret;

	if (!gswip_r || !gswip_r->priv || !gswip_r->parent ||
	    !gswip_r->parent->of_node ||
	    !of_property_read_bool(gswip_r->parent->of_node,
				   "lantiq,wait-for-gphy"))
		return ERR_PTR(-EINVAL);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->bus = gswip_r;
	ctx->base = gswip_r->priv;
	mutex_init(&ctx->lock);
	ctx->free_count = GRX500_PAE_SESSION_SIZE;
	ctx->free_head = 0;
	ctx->free_tail = GRX500_PAE_SESSION_SIZE - 1;

	for (i = 0; i < GRX500_PAE_SESSION_SIZE; i++) {
		ctx->sess[i].free_flag = true;
		ctx->sess[i].prev = (i == 0) ? -1 : (i - 1);
		ctx->sess[i].next = (i == GRX500_PAE_SESSION_SIZE - 1) ? -1 : (i + 1);
		ctx->sess[i].hw_next = i;
	}

	ret = grx500_pae_enable_pce_rules(ctx);
	if (ret) {
		kfree(ctx);
		return ERR_PTR(ret);
	}

	return ctx;
}
EXPORT_SYMBOL_GPL(grx500_pae_init);

void grx500_pae_destroy(struct grx500_pae_ctx *ctx)
{
	int i;

	if (!ctx)
		return;

	for (i = 0; i < GRX500_PAE_SESSION_SIZE; i++) {
		if (ctx->sess[i].valid)
			grx500_pae_session_del(ctx, i);
	}
	kfree(ctx);
}
EXPORT_SYMBOL_GPL(grx500_pae_destroy);

void grx500_pae_debug_regs(struct grx500_pae_ctx *ctx, u32 *gctrl0, u32 *gctrl1,
			   u32 *tfcr_num, u16 *tfcr0_k12, u16 *tfcr0_v9,
			   bool *tfcr0_vld)
{
	u32 ctrl;
	int ret;

	if (!ctx)
		return;

	mutex_lock(&ctx->lock);
	*gctrl0 = grx500_pae_read(ctx->base, 0x456);
	*gctrl1 = grx500_pae_read(ctx->base, 0x457);
	*tfcr_num = grx500_pae_read(ctx->base, 0x46f);
	ret = read_poll_timeout(grx500_pae_read, ctrl,
				!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
				false, ctx->base, GRX500_PCE_TBL_CTRL);
	if (!ret) {
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_ADDR, 0);
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_CTRL,
				 GRX500_PCE_TBL_TFLOW | GRX500_PCE_TBL_BUSY);
		ret = read_poll_timeout(grx500_pae_read, ctrl,
					!(ctrl & GRX500_PCE_TBL_BUSY), 1, 1000,
					false, ctx->base, GRX500_PCE_TBL_CTRL);
		if (!ret) {
			*tfcr0_k12 = grx500_pae_read(ctx->base,
						     GRX500_PCE_TBL_KEY(12));
			*tfcr0_v9 = grx500_pae_read(ctx->base,
						    GRX500_PCE_TBL_VAL(9));
			*tfcr0_vld = !!(ctrl & GRX500_PCE_TBL_VLD);
		}
		grx500_pae_write(ctx->base, GRX500_PCE_TBL_CTRL, 0);
	}
	mutex_unlock(&ctx->lock);
}
EXPORT_SYMBOL_GPL(grx500_pae_debug_regs);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GRX500 GSWIP-R PAE hardware L3 routing/NAT table helpers");
