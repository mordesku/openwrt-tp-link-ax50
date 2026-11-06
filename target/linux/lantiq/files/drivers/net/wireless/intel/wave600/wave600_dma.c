// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Data Path: DMA Descriptor Rings & NAPI
 *
 * Implements the vendor software data path (hw_mmb.c):
 *  - TX/RX/management HOST_DSC rings allocated in host DDR (coherent,
 *    uncached on non-coherent MIPS), ring counter registers in SHRAM,
 *  - the SHRAM RD pool (per-buffer payload address + BD index), pre-filled
 *    with skbs before the firmware is told the ring addresses,
 *  - UM_MAN_DATA_PATH_INIT_REQ (0x040E) binding all four rings to the FW,
 *  - NAPI receive loop (RX ring in-place HD updates, in-place buffer
 *    recycle via the same HD) and TX completion processing in one NAPI poll,
 *  - ndo_start_xmit building HOST_DSC per the vendor _mtlk_hw_send_data().
 *
 * Endianness (verified live + vendor source, 2026-10-31):
 *  - HOST_DSC words in host DDR are native host words (the FW is told
 *    HOST_ENDIANESS_MODE_BE and WLAN hdOwnBitValue in DATA_PATH_INIT),
 *  - UMI payload fields and the ring counter/status registers
 *    (p_IN_cntr / p_OUT_cntr / p_OUT_status) and the RD pool entry
 *    fields are LITTLE-ENDIAN device interfaces (vendor CPU_TO_LE32 /
 *    HOST_TO_MAC32 on the raw accessors),
 *  - CHI ring counter addresses and the RD pool base are FULL
 *    BAR0-relative offsets (unlike the FIFO/MAN/DBG CHI offsets which
 *    are SHRAM-relative).
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/bitfield.h>
#include <net/cfg80211.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_chi.h"
#include "wave600_ipc.h"
#include "wave600_dma.h"

#define W600_OWN_POLL_ITERS	1000

/* ------------------------------------------------------------------ */
/* Ring primitives (vendor mtlk_ccr_pcieg6.c semantics)                */
/* ------------------------------------------------------------------ */

static u32 ring_hds_to_process(struct wave600_dma_ring *r)
{
	/*
	 * p_OUT_status contains the number of completed HDs. In the vendor
	 * driver (mtlk_ccr_pcieg6.c:1832 + mmb_ops.h:106),
	 * LE32_TO_CPU(mtlk_raw_readl(p_OUT_status)) expands to
	 * le32_to_cpu(cpu_to_le32(readl(p_OUT_status))) == __raw_readl().
	 */
	return __raw_readl(r->out_status);
}

static void ring_clear_interrupt(struct wave600_dma_ring *r, u32 hds)
{
	/*
	 * Ack the firmware: out_cntr is an accumulator register the FW
	 * subtracts from its own completion counter. In the vendor driver
	 * (mtlk_ccr_pcieg6.c:1845 + mmb_ops.h:100),
	 * mtlk_raw_writel(CPU_TO_LE32(hds), p_OUT_cntr) expands to
	 * writel(le32_to_cpu(cpu_to_le32(hds)), p_OUT_cntr) == __raw_writel().
	 */
	__raw_writel(hds, r->out_cntr);
}

static void ring_doorbell(struct wave600_dma_ring *r)
{
	/*
	 * One HD was posted: increment the accumulator register. In the
	 * vendor driver (mtlk_ccr_pcieg6.c:1856 + mmb_ops.h:100),
	 * mtlk_raw_writel(CPU_TO_LE32(1), p_IN_cntr) expands to
	 * writel(le32_to_cpu(cpu_to_le32(1)), p_IN_cntr) == __raw_writel(1).
	 */
	__raw_writel(1, r->in_cntr);
}

static void wave600_dma_ring_push(struct wave600_dma *dma,
				  struct wave600_dma_ring *r,
				  const struct wave600_host_dsc *d)
{
	r->descs[r->write_ptr] = *d;
	wmb();
	r->write_ptr = (r->write_ptr + 1) % r->size;
	ring_doorbell(r);
}

static int wave600_dma_ring_alloc(struct wave600_dev *wdev,
				  struct wave600_dma_ring *r, u32 size,
				  const struct wave600_chi_ring *chi,
				  const char *name)
{
	u32 bytes = size * sizeof(struct wave600_host_dsc);

	r->size = size;
	r->read_ptr = 0;
	r->write_ptr = 0;
	r->descs = dma_alloc_coherent(wdev->dev, bytes, &r->dma, GFP_KERNEL);
	if (!r->descs)
		return -ENOMEM;
	memset(r->descs, 0, bytes);

	/*
	 * The CHI ring counter addresses are FULL BAR0-relative offsets
	 * (0x002550cc-class), unlike the FIFO/MAN/DBG offsets which are
	 * SHRAM-relative. Vendor maps them as mmb_base + value WITHOUT the
	 * SHRAM base (hw_mmb.c:4635-4637). Verified live 2026-10-31: with
	 * the extra SHRAM_BASE the doorbell never reached the firmware
	 * (real counter at BAR0+0x2550cc stayed 0).
	 */
	r->in_cntr = wdev->bar0 + chi->in_counter_addr;
	r->out_status = wdev->bar0 + chi->out_status_addr;
	r->out_cntr = wdev->bar0 + chi->out_counter_addr;

	dev_info(wdev->dev, "%s ring: %u HDs (dma %pad), CHI in=%08x in_st=%08x out=%08x out_st=%08x be_in=%08x be_out=%08x\n",
		 name, size, &r->dma, chi->in_counter_addr,
		 chi->in_status_addr, chi->out_counter_addr, chi->out_status_addr,
		 chi->in_counter_addr_be, chi->out_freed_counter_addr_be);

	/*
	 * Initialize the doorbell / completion accumulator words before the
	 * data path is started. The host manages in_counter (doorbells) and
	 * out_counter (host completion acks). Status registers (in_status and
	 * out_status) are owned by the firmware; writing zero to them would
	 * clear available firmware credits. Zero is endian-neutral.
	 */
	wave600_write32(wdev, chi->in_counter_addr, 0);
	if (chi->in_counter_addr_be)
		wave600_write32(wdev, chi->in_counter_addr_be, 0);
	wave600_write32(wdev, chi->out_counter_addr, 0);
	if (chi->out_freed_counter_addr_be)
		wave600_write32(wdev, chi->out_freed_counter_addr_be, 0);
	mmiowb();

	return 0;
}

static void wave600_dma_ring_free(struct wave600_dev *wdev,
				  struct wave600_dma_ring *r)
{
	if (r->descs)
		dma_free_coherent(wdev->dev,
				  r->size * sizeof(struct wave600_host_dsc),
				  r->descs, r->dma);
	r->descs = NULL;
}

/* ------------------------------------------------------------------ */
/* RD pool (SHRAM): initial per-buffer payload address + BD index       */
/* ------------------------------------------------------------------ */

static void rd_pool_set_entry(struct wave600_dma *dma, u32 idx,
			      dma_addr_t dma_addr, bool mgmt)
{
	struct wave600_dev *wdev = dma->wdev;
	u32 pool_off = dma->rd_pool_off;

	/* Mgmt entries follow the data entries in the same pool */
	if (mgmt)
		pool_off += dma->rd_pool_num * dma->rd_size;

	/*
	 * RD pool entry fields: in the vendor driver (hw_mmb.c:3209-3218 +
	 * mmb_ops.h:100), mtlk_raw_writel(HOST_TO_MAC32(val), addr) expands
	 * to writel(le32_to_cpu(cpu_to_le32(val)), addr) == __raw_writel(val).
	 */
	__raw_writel((u32)dma_addr,
		     wdev->bar0 + pool_off + (idx * dma->rd_size) +
		     dma->rd_payload_off);
	__raw_writel(idx,
		     wdev->bar0 + pool_off + (idx * dma->rd_size) +
		     dma->rd_idx_off);
}

/* ------------------------------------------------------------------ */
/* RX buffers                                                          */
/* ------------------------------------------------------------------ */

static int rx_bufs_alloc(struct wave600_dma *dma, struct wave600_dma_buf *bufs,
			 u32 n, u32 buf_size, bool mgmt)
{
	struct wave600_dev *wdev = dma->wdev;
	u32 i;

	for (i = 0; i < n; i++) {
		/* dev_alloc_skb (not netdev_alloc_skb): the pool is posted
		 * before the netdev exists, at the vendor pre-activation
		 * position. */
		struct sk_buff *skb = dev_alloc_skb(buf_size);

		if (!skb) {
			dev_err(wdev->dev, "RX skb alloc failed at %u/%u (%u B)\n",
				i, n, buf_size);
			return -ENOMEM;
		}
		bufs[i].skb = skb;
		bufs[i].size = buf_size;
		bufs[i].dma = dma_map_single(wdev->dev, skb->data, buf_size,
					     DMA_FROM_DEVICE);
		if (dma_mapping_error(wdev->dev, bufs[i].dma)) {
			dev_err(wdev->dev, "RX dma map failed at %u/%u\n", i, n);
			dev_kfree_skb_any(skb);
			return -ENOMEM;
		}
		rd_pool_set_entry(dma, i, bufs[i].dma, mgmt);
	}
	return 0;
}

static void rx_bufs_free(struct wave600_dma *dma, struct wave600_dma_buf *bufs,
			 u32 n)
{
	u32 i;

	for (i = 0; i < n; i++) {
		if (bufs[i].skb) {
			dma_unmap_single(dma->wdev->dev, bufs[i].dma,
					  bufs[i].size, DMA_FROM_DEVICE);
			dev_kfree_skb_any(bufs[i].skb);
			bufs[i].skb = NULL;
		}
	}
}

/* ------------------------------------------------------------------ */
/* NAPI poll: RX data + TX completions (vendor _mtlk_mmb_data_rx_process  */
/* and _mtlk_mmb_data_txout_process collapsed into one NAPI loop)      */
/* ------------------------------------------------------------------ */

static int wave600_dma_rx(struct wave600_dma *dma, int budget)
{
	struct wave600_dev *wdev = dma->wdev;
	struct wave600_dma_ring *r = &dma->rx;
	u32 processed = 0;
	u32 hds;

	hds = ring_hds_to_process(r);
	if (hds == 0)
		return 0;
	if (hds > (u32)budget)
		hds = budget;

	while (processed < hds) {
		struct wave600_host_dsc *hd = &r->descs[r->read_ptr];
		struct wave600_host_dsc recycle_hd;
		struct wave600_dma_buf *buf;
		struct sk_buff *skb;
		u32 info3, offset, len, total, bd;
		int iters = W600_OWN_POLL_ITERS;

		/* Wait for the FW to hand the descriptor back (HOST_DSC words
		 * are little-endian on the wire per vendor _mtlk_hw_send_data /
		 * _mtlk_mmb_put_hd_to_ring). */
		info3 = le32_to_cpu(hd->info3);
		while (FIELD_GET(W600_TX_OWN, info3) == W600_OWN_FW) {
			if (--iters == 0) {
				dev_err_ratelimited(wdev->dev,
					"RX HD own bit stuck (ptr %u, info3=0x%08x)\n",
					r->read_ptr, info3);
				r->read_ptr = (r->read_ptr + 1) % r->size;
				processed++;
				break;
			}
			info3 = le32_to_cpu(hd->info3);
		}
		if (iters == 0)
			continue;

		bd = FIELD_GET(W600_TX_BD_INDEX, le32_to_cpu(hd->bd_index));
		if (bd >= dma->rd_pool_num) {
			dev_err(wdev->dev, "RX HD bad bd index %u\n", bd);
			r->read_ptr = (r->read_ptr + 1) % r->size;
			processed++;
			continue;
		}

		buf = &dma->rx_bufs[bd];
		offset = FIELD_GET(W600_TX_OFFSET, info3);
		/* Gen6: real offset is 4 bits wide, MSB lives in payload addr bit 3 */
		offset += (0x08 & le32_to_cpu(hd->payload_addr));
		len = FIELD_GET(W600_TX_LENGTH, info3);
		total = offset + len;

		if (total > buf->size || len < ETH_HLEN) {
			dev_dbg(wdev->dev, "RX frame invalid (len %u, offset %u, bd %u)\n",
				len, offset, bd);
			goto recycle_reuse;
		}

		dma_sync_single_for_cpu(wdev->dev, buf->dma, buf->size,
					DMA_FROM_DEVICE);

		/*
		 * Allocate an aligned skb via napi_alloc_skb().
		 * On MIPS, napi_alloc_skb() automatically reserves NET_SKB_PAD +
		 * NET_IP_ALIGN (2 bytes). When the 14-byte Ethernet header is
		 * pulled by eth_type_trans(), the resulting L3 header is at
		 * offset 16 (4-byte aligned), preventing fatal MIPS ADEL exceptions.
		 * (Canon pattern from grx500-datapath.c:1445-1463).
		 */
		u32 info1 = le32_to_cpu(hd->info1);
		u8 ep = FIELD_GET(W600_TX_EP, info1);
		u8 radio_idx = (ep ^ 1) & 1;
		struct wave600_radio *radio = (radio_idx < 2) ? &wdev->radios[radio_idx] : NULL;
		struct net_device *ndev = (radio && radio->ndev) ? radio->ndev : dma->ndev;

		skb = napi_alloc_skb(&dma->napi, len);
		if (unlikely(!skb)) {
			if (ndev)
				ndev->stats.rx_dropped++;
			dma_sync_single_for_device(wdev->dev, buf->dma,
						   buf->size, DMA_FROM_DEVICE);
			goto recycle_reuse;
		}

		skb_put_data(skb, buf->skb->data + offset, len);
		dma_sync_single_for_device(wdev->dev, buf->dma, buf->size,
					   DMA_FROM_DEVICE);

		skb->protocol = eth_type_trans(skb, ndev);
		skb->ip_summed = CHECKSUM_NONE;

		napi_gro_receive(&dma->napi, skb);
		dma->rx_packets++;
		if (ndev) {
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
		}
		if (radio)
			radio->sta_last_rx = jiffies;

recycle_reuse:
		recycle_hd = *hd;
		recycle_hd.info3 = cpu_to_le32((le32_to_cpu(recycle_hd.info3) & ~W600_TX_OWN) |
					       W600_OWN_FW);
		wave600_dma_ring_push(dma, r, &recycle_hd);
		processed++;
		r->read_ptr = (r->read_ptr + 1) % r->size;
	}
	ring_clear_interrupt(r, processed);
	return processed;
}

static int wave600_dma_tx_clean(struct wave600_dma *dma, int budget)
{
	struct wave600_dev *wdev = dma->wdev;
	struct wave600_dma_ring *r = &dma->tx;
	u32 processed = 0;
	u32 hds;

	hds = ring_hds_to_process(r);
	if (hds == 0)
		return 0;
	if (hds > (u32)budget)
		hds = budget;

	while (processed < hds) {
		struct wave600_host_dsc *hd = &r->descs[r->read_ptr];
		u32 bd = FIELD_GET(W600_TX_BD_INDEX, le32_to_cpu(hd->bd_index));
		unsigned long flags;

		if (bd < WAVE600_DMA_RING_SIZE && dma->tx_skbs[bd]) {
			spin_lock_irqsave(&dma->tx_lock, flags);
			if (dma->tx_skbs[bd]) {
				dma_unmap_single(wdev->dev, dma->tx_dma[bd],
						 dma->tx_len[bd], DMA_TO_DEVICE);
				dev_kfree_skb_any(dma->tx_skbs[bd]);
				dma->tx_skbs[bd] = NULL;
				dma->tx_in_use--;
			}
			spin_unlock_irqrestore(&dma->tx_lock, flags);
		}
		dma->tx_done++;
		processed++;
		r->read_ptr = (r->read_ptr + 1) % r->size;
	}

	ring_clear_interrupt(r, processed);
	return processed;
}

/*
 * Management RX ring (vendor bss_rx_process + _mtlk_mmb_handle_bss_ind +
 * _mtlk_mmb_put_hd_to_ring, hw_mmb.c:2908-2954, 8337-8366, 10749-10883):
 * With WIPHY_FLAG_HAVE_AP_SME, the ARC firmware handles 802.11 management
 * frames internally, but still delivers received management frames into the
 * Mgmt RX ring (asserting WAVE600_IRQ_BSS_IND = BIT(3)). Each completed HD
 * must have its OWN bit reset to W600_OWN_FW, be pushed back to the Mgmt RX
 * ring with a doorbell, and have its completion acknowledged via out_cntr.
 */
static int wave600_dma_mgmt_rx(struct wave600_dma *dma, int budget)
{
	struct wave600_dev *wdev = dma->wdev;
	struct wave600_dma_ring *r = &dma->mgmt_rx;
	u32 processed = 0;
	u32 hds;

	hds = ring_hds_to_process(r);
	if (hds == 0)
		return 0;
	if (hds > (u32)budget)
		hds = budget;

	while (processed < hds) {
		struct wave600_host_dsc *hd = &r->descs[r->read_ptr];
		struct wave600_host_dsc recycle_hd;
		u32 info3, offset, len;
		int iters = W600_OWN_POLL_ITERS;

		info3 = le32_to_cpu(hd->info3);
		while (FIELD_GET(W600_TX_OWN, info3) == W600_OWN_FW) {
			if (--iters == 0) {
				dev_err_ratelimited(wdev->dev,
					"Mgmt RX HD own bit stuck (ptr %u, info3=0x%08x)\n",
					r->read_ptr, info3);
				r->read_ptr = (r->read_ptr + 1) % r->size;
				processed++;
				break;
			}
			info3 = le32_to_cpu(hd->info3);
		}
		if (iters == 0)
			continue;

		offset = FIELD_GET(W600_TX_OFFSET, info3);
		len = FIELD_GET(W600_TX_LENGTH, info3);

		if (hd->payload_addr && len > 0 && offset + len <= WAVE600_DMA_MGMT_BUF_SIZE) {
			u32 payload_addr = le32_to_cpu(hd->payload_addr);
			u8 *data = (u8 *)phys_to_virt(payload_addr + offset);
			struct ethhdr *eth = (struct ethhdr *)data;
			u32 info0 = le32_to_cpu(hd->info0);
			u32 info1 = le32_to_cpu(hd->info1);
			u8 frame_type = FIELD_GET(W600_TX_FRAMETYPE, info0);
			u8 type = FIELD_GET(GENMASK(25, 24), info0);
			u8 ep = FIELD_GET(W600_TX_EP, info1);
			u8 radio_idx = (ep ^ 1) & 1;
			struct wave600_radio *radio = (radio_idx < 2) ? &wdev->radios[radio_idx] : NULL;
			int freq = (radio && radio->ap_freq) ? radio->ap_freq :
				   ((radio_idx == 1) ? 5180 : 2437);
			bool is_eapol;

			dma_sync_single_for_cpu(wdev->dev, (dma_addr_t)payload_addr,
						offset + len, DMA_FROM_DEVICE);

			is_eapol = (frame_type == W600_FRAME_TYPE_EAPOL && type == 2) ||
				   (len >= ETH_HLEN && eth->h_proto == htons(ETH_P_PAE));

			if (is_eapol && radio && radio->ndev) {
				struct sk_buff *rskb = napi_alloc_skb(&dma->napi, len);

				dev_info_ratelimited(wdev->dev,
						     "Radio %u RX EAPOL frame len=%u src=%pM dst=%pM proto=0x%04x\n",
						     radio->radio_idx, len, eth->h_source, eth->h_dest,
						     ntohs(eth->h_proto));
				print_hex_dump(KERN_INFO, "rx-eapol: ", DUMP_PREFIX_OFFSET,
					       16, 1, data, min_t(u32, len, 64), false);
				if (rskb) {
					skb_put_data(rskb, data, len);
					rskb->protocol = eth_type_trans(rskb, radio->ndev);
					rskb->ip_summed = CHECKSUM_NONE;
					napi_gro_receive(&dma->napi, rskb);
					radio->ndev->stats.rx_packets++;
					radio->ndev->stats.rx_bytes += len;
					radio->sta_last_rx = jiffies;
				}
			} else if (radio && radio->ap_started && radio->wdev_net) {
				int sig = radio->sta_signal ? radio->sta_signal : -45;

				cfg80211_rx_mgmt(radio->wdev_net, freq, sig,
						 data, len, 0);
			}

			dma_sync_single_for_device(wdev->dev, (dma_addr_t)payload_addr,
						   offset + len, DMA_FROM_DEVICE);
		}

		recycle_hd = *hd;
		recycle_hd.info3 &= ~cpu_to_le32(W600_TX_OWN);
		r->read_ptr = (r->read_ptr + 1) % r->size;
		wave600_dma_ring_push(dma, r, &recycle_hd);
		processed++;
	}
	if (processed > 0)
		ring_clear_interrupt(r, processed);
	return processed;
}

static int wave600_dma_mgmt_tx_clean(struct wave600_dma *dma, int budget)
{
	struct wave600_dma_ring *r = &dma->mgmt_tx;
	u32 processed = 0;
	u32 hds;

	hds = ring_hds_to_process(r);
	if (hds == 0)
		return 0;
	if (hds > (u32)budget)
		hds = budget;

	while (processed < hds) {
		r->read_ptr = (r->read_ptr + 1) % r->size;
		processed++;
	}

	ring_clear_interrupt(r, processed);
	return processed;
}

int wave600_dma_mgmt_tx(struct wave600_radio *radio, const u8 *buf, size_t len)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_dma *dma = &wdev->dma;
	struct wave600_dma_ring *r = &dma->mgmt_tx;
	struct wave600_host_dsc hd;
	u32 slot_idx, info0_val;
	dma_addr_t slot_dma;
	void *slot_va;
	u16 fc;
	u8 type, subtype;
	bool mcf;

	if (!dma->data_path_ready || !dma->mgmt_tx_pool || !radio->ap_started)
		return -ENETDOWN;

	if (len > WAVE600_DMA_MGMT_BUF_SIZE)
		return -EMSGSIZE;

	slot_idx = r->write_ptr;
	slot_va = dma->mgmt_tx_pool + (slot_idx * WAVE600_DMA_MGMT_BUF_SIZE);
	slot_dma = dma->mgmt_tx_pool_dma + (slot_idx * WAVE600_DMA_MGMT_BUF_SIZE);

	memcpy(slot_va, buf, len);

	fc = (len >= 2) ? (buf[0] | (buf[1] << 8)) : 0;
	type = (fc & 0x0c) >> 2;
	subtype = (fc & 0xf0) >> 4;
	mcf = (len >= 10) ? is_multicast_ether_addr(buf + 4) : false;

	info0_val = FIELD_PREP(GENMASK(25, 24), type) |
		    FIELD_PREP(GENMASK(29, 26), subtype) |
		    FIELD_PREP(W600_TX_VAPID, radio->vap_id);

	if (subtype == 0x0d && len > 24)
		info0_val |= FIELD_PREP(GENMASK(23, 17), buf[24]);

	if (mcf) {
		info0_val |= W600_TX_MCF;
	} else if (len >= 10) {
		int sid = wave600_radio_find_sid(radio, buf + 4);
		if (sid >= 0)
			info0_val |= FIELD_PREP(W600_TX_STAID, (u16)sid);
		else
			info0_val |= W600_TX_UNKNOWN_SID;
	} else {
		info0_val |= W600_TX_UNKNOWN_SID;
	}

	memset(&hd, 0, sizeof(hd));
	hd.info0 = cpu_to_le32(info0_val);
	hd.info1 = cpu_to_le32(FIELD_PREP(W600_TX_CLASS, 15) |
			       FIELD_PREP(W600_TX_EP, radio->ep));
	hd.payload_addr = cpu_to_le32((u32)slot_dma);
	hd.info3 = cpu_to_le32(FIELD_PREP(W600_TX_SOP_EOP, W600_NO_FRAG) |
			       FIELD_PREP(W600_TX_OFFSET, 0) |
			       FIELD_PREP(W600_TX_LENGTH, len) |
			       W600_OWN_FW);
	hd.bd_index = cpu_to_le32(FIELD_PREP(W600_TX_BD_INDEX, slot_idx));

	wave600_dma_ring_push(dma, r, &hd);
	return 0;
}

int wave600_dma_mgmt_tx_eapol(struct wave600_radio *radio, const u8 *buf, size_t len)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_dma *dma = &wdev->dma;
	struct wave600_dma_ring *r = &dma->mgmt_tx;
	struct wave600_host_dsc hd;
	u32 slot_idx, info0_val;
	dma_addr_t slot_dma;
	void *slot_va;
	const struct ethhdr *eth = (const struct ethhdr *)buf;

	if (!dma->data_path_ready || !dma->mgmt_tx_pool || !radio->ap_started)
		return -ENETDOWN;

	if (len > WAVE600_DMA_MGMT_BUF_SIZE)
		return -EMSGSIZE;

	slot_idx = r->write_ptr;
	slot_va = dma->mgmt_tx_pool + (slot_idx * WAVE600_DMA_MGMT_BUF_SIZE);
	slot_dma = dma->mgmt_tx_pool_dma + (slot_idx * WAVE600_DMA_MGMT_BUF_SIZE);

	memcpy(slot_va, buf, len);

	/*
	 * ARC FullMAC firmware EAPOL TX over Mgmt TX ring:
	 * vendor hw_mmb.c:11011-11046 (PROCESS_EAPOL):
	 * type = 2 (IEEE80211_FTYPE_DATA >> 2)
	 * subtype = 8 (IEEE80211_STYPE_QOS_DATA >> 4)
	 * frame_type = 3 (W600_FRAME_TYPE_EAPOL)
	 * class = 15 (GEN6 max priority)
	 * ep = radio->ep
	 */
	info0_val = FIELD_PREP(GENMASK(25, 24), 2) | /* TX_BSS_EXTRA_TYPE = DATA */
		    FIELD_PREP(GENMASK(29, 26), 8) | /* TX_BSS_EXTRA_SUBTYPE = QOS_DATA */
		    FIELD_PREP(W600_TX_FRAMETYPE, W600_FRAME_TYPE_EAPOL) |
		    FIELD_PREP(W600_TX_VAPID, radio->vap_id);

	if (len >= ETH_HLEN) {
		int sid = wave600_radio_find_sid(radio, eth->h_dest);
		if (sid >= 0) {
			info0_val |= FIELD_PREP(W600_TX_STAID, (u16)sid);
		} else {
			dev_warn_ratelimited(wdev->dev, "Radio %u EAPOL TX dropped: unknown destination %pM\n",
					     radio->radio_idx, eth->h_dest);
			return -ENOENT;
		}
	} else {
		return -EINVAL;
	}

	memset(&hd, 0, sizeof(hd));
	hd.info0 = cpu_to_le32(info0_val);
	hd.info1 = cpu_to_le32(FIELD_PREP(W600_TX_CLASS, 7) |
			       FIELD_PREP(W600_TX_EP, radio->ep));
	hd.payload_addr = cpu_to_le32((u32)slot_dma);
	hd.info3 = cpu_to_le32(FIELD_PREP(W600_TX_SOP_EOP, W600_NO_FRAG) |
			       FIELD_PREP(W600_TX_OFFSET, 0) |
			       FIELD_PREP(W600_TX_LENGTH, len) |
			       W600_OWN_FW);
	hd.bd_index = cpu_to_le32(FIELD_PREP(W600_TX_BD_INDEX, slot_idx));

	wave600_dma_ring_push(dma, r, &hd);
	return 0;
}

static int wave600_dma_poll(struct napi_struct *napi, int budget)
{
	struct wave600_dma *dma = container_of(napi, struct wave600_dma, napi);
	struct wave600_dev *wdev = dma->wdev;
	int work;

	wave600_dma_tx_clean(dma, WAVE600_DMA_RING_SIZE);
	wave600_dma_mgmt_tx_clean(dma, WAVE600_DMA_MGMT_RING_SIZE);
	wave600_dma_mgmt_rx(dma, WAVE600_DMA_MGMT_RING_SIZE);
	work = wave600_dma_rx(dma, budget);

	if (work < budget) {
		if (napi_complete_done(napi, work)) {
			/* Re-enable host interrupts (vendor mask 0x2F) */
			wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK,
					WAVE600_IRQ_ALL);
			mmiowb();
		}
	}

	return work;
}

bool wave600_dma_schedule_napi(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;

	if (dma->data_path_ready && dma->napi_added) {
		napi_schedule(&dma->napi);
		return true;
	}
	return false;
}

/* ------------------------------------------------------------------ */
/* TX path (vendor _mtlk_hw_send_data)                                 */
/* ------------------------------------------------------------------ */

netdev_tx_t wave600_dma_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct wave600_radio *radio = wave600_ndev_to_radio(ndev);
	struct wave600_dev *wdev;
	struct wave600_dma *dma;
	struct wave600_host_dsc hd;
	dma_addr_t dma_addr;
	u32 bd;
	unsigned long flags;
	u32 info0;

	if (!radio)
		goto drop;

	wdev = radio->wdev;
	dma = &wdev->dma;

	if (!dma->data_path_ready) {
		dma->tx_dropped++;
		goto drop;
	}

	if (skb->len < ETH_HLEN)
		goto drop;

	/*
	 * EAPOL frames (Ethertype 0x888E / ETH_P_PAE):
	 * In WAV654 FullMAC firmware, when 802.1X filter is closed,
	 * EAPOL frames MUST be dispatched over the Mgmt TX ring (sMangTxRING)
	 * with FRAME_TYPE_EAPOL, rather than the data TX ring (vendor hw_mmb.c:11016).
	 */
	if (skb->protocol == htons(ETH_P_PAE) ||
	    ((struct ethhdr *)skb->data)->h_proto == htons(ETH_P_PAE)) {
		int ret;

		dev_info_ratelimited(wdev->dev, "Radio %u TX EAPOL frame len=%u dst=%pM\n",
				     radio->radio_idx, skb->len, ((struct ethhdr *)skb->data)->h_dest);
		ret = wave600_dma_mgmt_tx_eapol(radio, skb->data, skb->len);
		if (ret == 0) {
			dma->tx_packets++;
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;
		} else {
			dma->tx_dropped++;
			ndev->stats.tx_dropped++;
		}
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	dma_addr = dma_map_single(wdev->dev, skb->data, skb->len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(wdev->dev, dma_addr))
		goto drop;

	spin_lock_irqsave(&dma->tx_lock, flags);
	bd = dma->tx_write;
	if (dma->tx_skbs[bd]) {
		spin_unlock_irqrestore(&dma->tx_lock, flags);
		dma->tx_dropped++;
		goto drop_unmap;
	}
	dma->tx_skbs[bd] = skb;
	dma->tx_dma[bd] = dma_addr;
	dma->tx_len[bd] = skb->len;
	dma->tx_write = (dma->tx_write + 1) % WAVE600_DMA_RING_SIZE;
	dma->tx_in_use++;
	spin_unlock_irqrestore(&dma->tx_lock, flags);

	memset(&hd, 0, sizeof(hd));
	/*
	 * Vendor _mtlk_hw_send_data + _mtlk_mmb_put_hd_to_ring (hw_mmb.c:8346-8437):
	 *  - In DB_2x2 mode, radio_id 0 maps to ep_bit = 1, radio_id 1 maps to ep_bit = 0.
	 *  - VAPID in info0 is radio->vap_id (0 for primary AP on both radios).
	 *  - All 5 HOST_DSC words are converted with cpu_to_le32() on write.
	 */
	info0 = FIELD_PREP(W600_TX_VAPID, radio->vap_id) |
		FIELD_PREP(W600_TX_FRAMETYPE, W600_FRAME_TYPE_ETHERNET);
	if (is_multicast_ether_addr(skb->data)) {
		info0 |= W600_TX_MCF;
	} else {
		int sid = wave600_radio_find_sid(radio, skb->data);
		if (sid >= 0)
			info0 |= FIELD_PREP(W600_TX_STAID, (u16)sid);
		else
			info0 |= W600_TX_UNKNOWN_SID;
	}
	hd.info0 = cpu_to_le32(info0);
	hd.info1 = cpu_to_le32(FIELD_PREP(W600_TX_CLASS, skb->priority & 7) |
			       FIELD_PREP(W600_TX_EP, radio->ep));
	hd.payload_addr = cpu_to_le32((u32)(dma_addr & ~7));
	hd.info3 = cpu_to_le32(FIELD_PREP(W600_TX_SOP_EOP, W600_NO_FRAG) |
			       FIELD_PREP(W600_TX_OFFSET, dma_addr & 7) |
			       FIELD_PREP(W600_TX_LENGTH, skb->len) |
			       W600_OWN_FW);
	hd.bd_index = cpu_to_le32(FIELD_PREP(W600_TX_BD_INDEX, bd));

	wave600_dma_ring_push(dma, &dma->tx, &hd);
	dma->tx_packets++;
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	return NETDEV_TX_OK;

drop_unmap:
	dma_unmap_single(wdev->dev, dma_addr, skb->len, DMA_TO_DEVICE);
drop:
	dev_kfree_skb_any(skb);
	ndev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------ */
/* DATA_PATH_INIT (0x040E) - bind all four rings to the firmware       */
/* ------------------------------------------------------------------ */

/* Vendor DC_DP_RING_HW_MODE0 ring geometry (hw_mmb.c 7614-7616) */
#define WAVE600_DC_RING_SIZE		512
#define WAVE600_DC_HD_SIZE		16	/* GRX350_HW_HD_SIZE (4*4) */
#define WAVE600_DC_HD_TXOUT_SIZE		4	/* GRX350_HW_HD_TXOUT_SIZE (4*1) */
#define WAVE600_DATA_PATH_MODE_DC_NONE		0	/* mhi_umi.h DataPathMode_e: pure SW rings */
#define WAVE600_DATA_PATH_MODE_DC_MODE_0	1	/* mhi_umi.h DataPathMode_e */

struct wave600_umi_data_path_init {
	u32 tx_in_ring_start;
	u32 tx_in_ring_size_bytes;
	u32 tx_out_ring_start;
	u32 tx_out_ring_size_bytes;
	u32 rx_in_ring_start;
	u32 rx_in_ring_size_bytes;
	u32 rx_out_ring_start;
	u32 rx_out_ring_size_bytes;
	u32 mang_tx_ring_start;
	u32 mang_tx_ring_size_bytes;
	u32 mang_rx_ring_start;
	u32 mang_rx_ring_size_bytes;
	u32 tx_out_ready_counter_addr;
	u32 rx_out_ready_counter_addr;
	u32 tx_in_freed_counter_addr;
	u32 rx_in_freed_counter_addr;
	u16 logger_rx_sid;
	u8 data_path_mode;		/* DataPathMode_e: 0=DC_NONE, 1=DC_MODE_0 */
	u8 logger_max_stream_number;
	u8 host_endianess_mode;		/* 1 = HOST_ENDIANESS_MODE_BE */
	u8 hd_own_bit_value;		/* 1 = WLAN_HOST_OWN_BIT_VALUE */
	u8 fw_interface;
	u8 cbm_fragmentation_wa_enable;
	u8 ep3_msb;
	u8 reserved[3];
} __packed;

static int wave600_dma_data_path_init(struct wave600_dma *dma)
{
	struct wave600_dev *wdev = dma->wdev;
	struct wave600_umi_data_path_init p;
	u32 hd_sz = sizeof(struct wave600_host_dsc);

	memset(&p, 0, sizeof(p));
	/*
	 * Software data path configuration (hw_mmb.c lines 7796-7808):
	 * In DATA_PATH_MODE_DC_NONE, all 4 rings (TX, RX, Mgmt TX, Mgmt RX)
	 * are standard HOST_DSC descriptor rings allocated in host memory.
	 * Both in and out ring start addresses point to the corresponding
	 * ring DMA address, and size is count * sizeof(struct wave600_host_dsc).
	 * UMI payload scalars are little-endian on the wire (vendor
	 * HOST_TO_MAC32 = cpu_to_le32, mtlkinc.h); the firmware programs
	 * its ring engines from these exact values.
	 */
	p.tx_in_ring_start = cpu_to_le32((u32)dma->tx.dma);
	p.tx_in_ring_size_bytes = cpu_to_le32(dma->tx.size * hd_sz);
	p.tx_out_ring_start = cpu_to_le32((u32)dma->tx.dma);
	p.tx_out_ring_size_bytes = cpu_to_le32(dma->tx.size * hd_sz);
	p.rx_in_ring_start = cpu_to_le32((u32)dma->rx.dma);
	p.rx_in_ring_size_bytes = cpu_to_le32(dma->rx.size * hd_sz);
	p.rx_out_ring_start = cpu_to_le32((u32)dma->rx.dma);
	p.rx_out_ring_size_bytes = cpu_to_le32(dma->rx.size * hd_sz);
	p.mang_tx_ring_start = cpu_to_le32((u32)dma->mgmt_tx.dma);
	p.mang_tx_ring_size_bytes = cpu_to_le32(dma->mgmt_tx.size * hd_sz);
	p.mang_rx_ring_start = cpu_to_le32((u32)dma->mgmt_rx.dma);
	p.mang_rx_ring_size_bytes = cpu_to_le32(dma->mgmt_rx.size * hd_sz);
	p.host_endianess_mode = 1;	/* HOST_ENDIANESS_MODE_BE (rule 22) */
	p.hd_own_bit_value = 1;		/* WLAN_HOST_OWN_BIT_VALUE */
	p.data_path_mode = WAVE600_DATA_PATH_MODE_DC_NONE;
	p.cbm_fragmentation_wa_enable = 0;
	p.fw_interface = wdev->radio_idx;
	p.logger_max_stream_number = 1;	/* vendor rtlog default (old fixed 1) */
	p.logger_rx_sid = cpu_to_le16(127); /* vendor HOST_TO_MAC16(127) */
	p.ep3_msb = 7;			/* vendor ep3Msb = 7 (hybrid log line 77) */

	return wave600_ipc_cmd(wdev, WAVE600_UM_MAN_DATA_PATH_INIT_REQ,
			       WAVE600_MC_MAN_DATA_PATH_INIT_CFM,
			       &p, sizeof(p), NULL, 0, 5000);
}

/* ------------------------------------------------------------------ */
/* Init / cleanup                                                       */
/* ------------------------------------------------------------------ */

extern const struct net_device_ops wave600_netdev_ops;

static int wave600_netdev_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int wave600_netdev_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

const struct net_device_ops wave600_netdev_ops = {
	.ndo_open = wave600_netdev_open,
	.ndo_stop = wave600_netdev_stop,
	.ndo_start_xmit = wave600_dma_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

/*
 * Post the RX buffers into the SHRAM RD pool at the vendor pre-activation
 * position (hw_mmb_card steps HW_RX_DATA_BUFFERS / HW_RX_MGMT_BUFFERS run
 * right after the CHI magic wait, BEFORE the SW_RESET and the whole
 * calibration sequence). Vendor SW-mode _mtlk_mmb_prealloc_rx_data_buffers
 * allocates every pool buffer and writes its payload address + pool index
 * into the SHRAM RD pool entry - the same entries this driver fills.
 * The FW application initializes against a filled pool; the previous late
 * position (after HDK/ProgModel/RF cal) left the pool empty during the
 * entire calibration sequence.
 */
int wave600_dma_post_rx_pool(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;
	const struct wave600_chi_area *chi = &wdev->chi;
	int ret;

	if (dma->pool_posted)
		return 0;

	dma->wdev = wdev;
	dma->rd_pool_num = chi->basic.rd_pool.num_rds;
	dma->rd_size = chi->basic.rd_pool.rd_size;
	dma->rd_payload_off = chi->basic.rd_pool.payload_offset;
	dma->rd_idx_off = chi->basic.rd_pool.bd_index_offset;
	/* Full BAR0-relative offset, like the ring counters: the vendor
	 * maps the pool as mmb_base + u32RdPoolBaseAddress (hw_mmb.c:3204).
	 */
	dma->rd_pool_off = chi->basic.rd_pool.pool_base_addr;
	dma->rd_mgmt_num = chi->basic.rd_pool.num_mgt_rds;
	if (!dma->rd_pool_num || !dma->rd_size) {
		dev_err(wdev->dev, "invalid RD pool geometry (%u x %u)\n",
			dma->rd_pool_num, dma->rd_size);
		return -EINVAL;
	}

	dma->rx_bufs = kcalloc(dma->rd_pool_num, sizeof(*dma->rx_bufs), GFP_KERNEL);
	dma->mgmt_bufs = kcalloc(dma->rd_mgmt_num, sizeof(*dma->mgmt_bufs), GFP_KERNEL);
	if (!dma->rx_bufs || !dma->mgmt_bufs) {
		dev_err(wdev->dev,
			"buf array kcalloc failed: data %u x %zu, mgmt %u x %zu\n",
			dma->rd_pool_num, sizeof(*dma->rx_bufs),
			dma->rd_mgmt_num, sizeof(*dma->mgmt_bufs));
		ret = -ENOMEM;
		goto err_arrays;
	}

	ret = rx_bufs_alloc(dma, dma->rx_bufs, dma->rd_pool_num,
			   WAVE600_DMA_RX_BUF_SIZE, false);
	if (ret)
		goto err_bufs;
	ret = rx_bufs_alloc(dma, dma->mgmt_bufs, dma->rd_mgmt_num,
			   WAVE600_DMA_MGMT_BUF_SIZE, true);
	if (ret)
		goto err_bufs;

	dma->pool_posted = true;
	dev_info(wdev->dev,
		 "RD pool posted (pre-activation): %u data (%u B) + %u mgmt (%u B) buffers, entry %u B (payload +%u, idx +%u)\n",
		 dma->rd_pool_num, WAVE600_DMA_RX_BUF_SIZE, dma->rd_mgmt_num,
		 WAVE600_DMA_MGMT_BUF_SIZE, dma->rd_size,
		 dma->rd_payload_off, dma->rd_idx_off);
	return 0;

err_bufs:
	rx_bufs_free(dma, dma->rx_bufs, dma->rd_pool_num);
	rx_bufs_free(dma, dma->mgmt_bufs, dma->rd_mgmt_num);
err_arrays:
	kfree(dma->rx_bufs);
	kfree(dma->mgmt_bufs);
	dma->rx_bufs = NULL;
	dma->mgmt_bufs = NULL;
	return ret;
}

void wave600_dma_repost_rx_pool(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;
	u32 i;

	if (!dma->pool_posted || !dma->rx_bufs || !dma->mgmt_bufs)
		return;

	for (i = 0; i < dma->rd_pool_num; i++) {
		if (dma->rx_bufs[i].skb)
			rd_pool_set_entry(dma, i, dma->rx_bufs[i].dma, false);
	}

	for (i = 0; i < dma->rd_mgmt_num; i++) {
		if (dma->mgmt_bufs[i].skb)
			rd_pool_set_entry(dma, i, dma->mgmt_bufs[i].dma, true);
	}

	dev_info(wdev->dev, "RD pool descriptors re-posted to SHRAM after recovery\n");
}

static void wave600_dma_ring_reset(struct wave600_dev *wdev, struct wave600_dma_ring *r)
{
	if (!r || !r->descs)
		return;
	memset(r->descs, 0, r->size * sizeof(struct wave600_host_dsc));
	r->read_ptr = 0;
	r->write_ptr = 0;
	if (r->in_cntr)
		__raw_writel(0, r->in_cntr);
	if (r->out_cntr)
		__raw_writel(0, r->out_cntr);
}

void wave600_dma_reset_rings(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;

	wave600_dma_ring_reset(wdev, &dma->tx);
	wave600_dma_ring_reset(wdev, &dma->rx);
	wave600_dma_ring_reset(wdev, &dma->mgmt_tx);
	wave600_dma_ring_reset(wdev, &dma->mgmt_rx);
	dev_info(wdev->dev, "DMA host descriptor rings reset for recovery\n");
}

static void dc_rings_free(struct wave600_dev *wdev, struct wave600_dma *dma)
{
	if (dma->dc_tx_in)
		dma_free_coherent(wdev->dev,
				  dma->dc_ring_size * WAVE600_DC_HD_SIZE,
				  dma->dc_tx_in, dma->dc_tx_in_dma);
	if (dma->dc_rx_out)
		dma_free_coherent(wdev->dev,
				  dma->dc_ring_size * WAVE600_DC_HD_SIZE,
				  dma->dc_rx_out, dma->dc_rx_out_dma);
	if (dma->dc_tx_out)
		dma_free_coherent(wdev->dev,
				  dma->dc_ring_size * WAVE600_DC_HD_TXOUT_SIZE,
				  dma->dc_tx_out, dma->dc_tx_out_dma);
	dma->dc_tx_in = NULL;
	dma->dc_rx_out = NULL;
	dma->dc_tx_out = NULL;
}

__maybe_unused static int dc_rings_alloc(struct wave600_dev *wdev, struct wave600_dma *dma)
{
	dma->dc_ring_size = WAVE600_DC_RING_SIZE;

	dma->dc_tx_in = dma_alloc_coherent(wdev->dev,
					   dma->dc_ring_size * WAVE600_DC_HD_SIZE,
					   &dma->dc_tx_in_dma, GFP_KERNEL);
	dma->dc_rx_out = dma_alloc_coherent(wdev->dev,
					    dma->dc_ring_size * WAVE600_DC_HD_SIZE,
					    &dma->dc_rx_out_dma, GFP_KERNEL);
	dma->dc_tx_out = dma_alloc_coherent(wdev->dev,
					    dma->dc_ring_size * WAVE600_DC_HD_TXOUT_SIZE,
					    &dma->dc_tx_out_dma, GFP_KERNEL);

	if (!dma->dc_tx_in || !dma->dc_rx_out || !dma->dc_tx_out) {
		dc_rings_free(wdev, dma);
		return -ENOMEM;
	}

	memset(dma->dc_tx_in, 0, dma->dc_ring_size * WAVE600_DC_HD_SIZE);
	memset(dma->dc_rx_out, 0, dma->dc_ring_size * WAVE600_DC_HD_SIZE);
	memset(dma->dc_tx_out, 0, dma->dc_ring_size * WAVE600_DC_HD_TXOUT_SIZE);

	dev_info(wdev->dev,
		 "DC mode 0 rings: tx_in %u x %u B (dma %pad), rx_out %u x %u B (dma %pad), tx_out %u x %u B (dma %pad)\n",
		 dma->dc_ring_size, WAVE600_DC_HD_SIZE, &dma->dc_tx_in_dma,
		 dma->dc_ring_size, WAVE600_DC_HD_SIZE, &dma->dc_rx_out_dma,
		 dma->dc_ring_size, WAVE600_DC_HD_TXOUT_SIZE, &dma->dc_tx_out_dma);
	return 0;
}

/*
 * Vendor pre-activation steps HW_DAT_REQ_BDR / HW_DAT_IND_BDR: allocate
 * the host mirror rings and bind their SHRAM counter registers. No
 * firmware message is sent here; the rings are bound to the firmware
 * later in card finalize (wave600_dma_data_path_start, vendor step
 * HW_DATA_PATH).
 */
int wave600_dma_init(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;
	u32 rx_ring_size;
	int ret;

	spin_lock_init(&dma->tx_lock);

	dev_info(wdev->dev,
		 "CHI RD pool raw: base=0x%08x rd_size=%u payload_off=%u idx_off=%u num_rds=%u num_mgt=%u\n",
		 wdev->chi.basic.rd_pool.pool_base_addr, wdev->chi.basic.rd_pool.rd_size,
		 wdev->chi.basic.rd_pool.payload_offset,
		 wdev->chi.basic.rd_pool.bd_index_offset,
		 wdev->chi.basic.rd_pool.num_rds, wdev->chi.basic.rd_pool.num_mgt_rds);

	/*
	 * The RX ring is clamped to the RD pool size from the CHI snapshot
	 * (sRdPOOL), independent of the pool buffer posting step.
	 */
	rx_ring_size = min_t(u32, WAVE600_DMA_RING_SIZE,
			     wdev->chi.basic.rd_pool.num_rds);

	ret = wave600_dma_ring_alloc(wdev, &dma->tx, WAVE600_DMA_RING_SIZE,
				     &wdev->chi.basic.tx_ring, "TX");
	if (ret)
		return ret;
	ret = wave600_dma_ring_alloc(wdev, &dma->rx, rx_ring_size,
				     &wdev->chi.basic.rx_ring, "RX");
	if (ret)
		goto err_rx;
	ret = wave600_dma_ring_alloc(wdev, &dma->mgmt_tx, WAVE600_DMA_MGMT_RING_SIZE,
				     &wdev->chi.basic.mang_tx_ring, "Mgmt TX");
	if (ret)
		goto err_rx;
	ret = wave600_dma_ring_alloc(wdev, &dma->mgmt_rx, WAVE600_DMA_MGMT_RING_SIZE,
				     &wdev->chi.basic.mang_rx_ring, "Mgmt RX");
	if (ret)
		goto err_mtx;

	dma->mgmt_tx_pool = dma_alloc_coherent(wdev->dev,
					       WAVE600_DMA_MGMT_RING_SIZE * WAVE600_DMA_MGMT_BUF_SIZE,
					       &dma->mgmt_tx_pool_dma, GFP_KERNEL);
	if (!dma->mgmt_tx_pool) {
		ret = -ENOMEM;
		goto err_pool;
	}

	return 0;

err_pool:
	wave600_dma_ring_free(wdev, &dma->mgmt_rx);
err_mtx:
	wave600_dma_ring_free(wdev, &dma->mgmt_rx);
err_rx:
	wave600_dma_ring_free(wdev, &dma->mgmt_tx);
	wave600_dma_ring_free(wdev, &dma->rx);
	wave600_dma_ring_free(wdev, &dma->tx);
	return ret;
}

/*
 * Vendor card finalize step HW_DATA_PATH (_mtlk_mmb_data_path_init):
 * UM_MAN_DATA_PATH_INIT_REQ binds the prepared rings to the firmware.
 * Vendor finalize order: SW_RESET -> DATA_PATH -> HW_DEP_CFG.
 */
int wave600_dma_data_path_start(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;
	int ret;

	ret = wave600_dma_data_path_init(dma);
	if (ret) {
		dev_err(wdev->dev, "DATA_PATH_INIT failed: %d\n", ret);
		return ret;
	}

	dma->data_path_ready = true;
	dev_info(wdev->dev, "data path initialized (DATA_PATH_INIT confirmed)\n");
	return 0;
}

/*
 * Bind the AP netdevs (created by the cfg80211 layer, which owns their
 * lifecycle) to the DMA layer: NAPI, carrier state and TX queues.
 */
void wave600_dma_attach(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;
	int i;

	dma->ndev = wdev->radios[0].ndev;
	wave600_dma_enable_napi(wdev);

	for (i = 0; i < 2; i++) {
		struct net_device *ndev = wdev->radios[i].ndev;

		if (ndev) {
			netif_carrier_off(ndev);
			netif_tx_stop_all_queues(ndev);
			netif_start_queue(ndev);
		}
	}
}

void wave600_dma_enable_napi(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;

	netif_napi_add(dma->ndev, &dma->napi, wave600_dma_poll);
	napi_enable(&dma->napi);
	dma->napi_added = true;
}

void wave600_dma_stop_netdev(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;

	if (dma->napi_added) {
		napi_disable(&dma->napi);
		netif_napi_del(&dma->napi);
		dma->napi_added = false;
	}
	if (dma->ndev)
		netif_tx_disable(dma->ndev);
}

void wave600_dma_cleanup(struct wave600_dev *wdev)
{
	struct wave600_dma *dma = &wdev->dma;
	int i;

	dma->data_path_ready = false;
	rx_bufs_free(dma, dma->rx_bufs, dma->rd_pool_num);
	rx_bufs_free(dma, dma->mgmt_bufs, dma->rd_mgmt_num);
	kfree(dma->rx_bufs);
	kfree(dma->mgmt_bufs);
	/* Idempotent cleanup: a second call must be a no-op */
	dma->rx_bufs = NULL;
	dma->mgmt_bufs = NULL;
	dma->rd_pool_num = 0;
	dma->rd_mgmt_num = 0;
	dma->pool_posted = false;
	if (dma->mgmt_tx_pool) {
		dma_free_coherent(wdev->dev,
				  WAVE600_DMA_MGMT_RING_SIZE * WAVE600_DMA_MGMT_BUF_SIZE,
				  dma->mgmt_tx_pool, dma->mgmt_tx_pool_dma);
		dma->mgmt_tx_pool = NULL;
	}
	for (i = 0; i < WAVE600_DMA_RING_SIZE; i++)
		if (dma->tx_skbs[i])
			dev_kfree_skb_any(dma->tx_skbs[i]);
	dc_rings_free(wdev, dma);
	wave600_dma_ring_free(wdev, &dma->mgmt_rx);
	wave600_dma_ring_free(wdev, &dma->mgmt_tx);
	wave600_dma_ring_free(wdev, &dma->rx);
	wave600_dma_ring_free(wdev, &dma->tx);
}
