/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Data Path: DMA Descriptor Rings & NAPI
 */

#ifndef __WAVE600_DMA_H__
#define __WAVE600_DMA_H__

#include <linux/netdevice.h>

struct wave600_dev;
struct wave600_radio;

/*
 * Ring geometry. Vendor defaults are 4096 for TX/RX and 128 for the
 * management rings; the Wi-Fi bring-up plan uses 512 for data rings.
 * The RX ring is additionally clamped to the RD pool size reported by
 * the firmware in CHI sRdPOOL.
 */
#define WAVE600_DMA_RING_SIZE		512
#define WAVE600_DMA_MGMT_RING_SIZE	128

/* Vendor RX_DATA_FRAME_SIZE = 2048 - 128 - 192 (hw_mmb.h) */
#define WAVE600_DMA_RX_BUF_SIZE		1728
/* Vendor RX_MGMT_ALLOC_SIZE (~BSS_MGMT_FRAME_SIZE 2348 + alignment + metrics) */
#define WAVE600_DMA_MGMT_BUF_SIZE	2464

/* HOST_DSC (mhi_descriptors.h): 5 x 32-bit words, native endianness on
 * this platform (rule 22: no byte swapping, FW is told BE host mode). */
struct wave600_host_dsc {
	u32 info0;
	u32 info1;
	u32 payload_addr;
	u32 info3;
	u32 bd_index;
} __packed;

/* TX_DATA_INFO bitfields (mhi_umi.h) */
#define W600_TX_STAID			GENMASK(7, 0)
#define W600_TX_VAPID			GENMASK(11, 8)
#define W600_TX_UNKNOWN_SID		BIT(13)
#define W600_TX_MCF			BIT(14)
#define W600_TX_FRAMETYPE		GENMASK(16, 15)
#define W600_TX_CLASS			GENMASK(3, 0)
#define W600_TX_EP			GENMASK(11, 8)
#define W600_TX_LENGTH			GENMASK(15, 0)
#define W600_TX_OFFSET			GENMASK(25, 23)
#define W600_TX_SOP_EOP			GENMASK(29, 28)
#define W600_TX_OWN			BIT(31)
#define W600_TX_BD_INDEX		GENMASK(15, 0)

/* SOP/EOP fragment codes (mhi_umi.h): 2 = first, 1 = last, 0 = mid, 3 = none */
#define W600_FRAG_FIRST			2
#define W600_FRAG_LAST			1
#define W600_FRAG_MID			0
#define W600_NO_FRAG			3

/* Own bit: 0 = firmware owns the descriptor, 1 = host owns it */
#define W600_OWN_FW			0
#define W600_OWN_HOST			1

#define W600_FRAME_TYPE_ETHERNET	0
#define W600_FRAME_TYPE_EAPOL		3

struct wave600_dma_ring {
	struct wave600_host_dsc *descs;	/* coherent memory, HOST_DSC array */
	dma_addr_t dma;
	u32 size;
	u32 read_ptr;
	u32 write_ptr;
	/* SHRAM counter registers (BAR0 offsets, ioremapped) */
	void __iomem *in_cntr;		/* doorbell accumulator */
	void __iomem *out_status;	/* number of completed HDs */
	void __iomem *out_cntr;		/* host ack accumulator */
};

struct wave600_dma_buf {
	struct sk_buff *skb;
	dma_addr_t dma;
	u32 size;
};

struct wave600_dma {
	struct wave600_dev *wdev;
	struct net_device *ndev;
	struct napi_struct napi;
	bool data_path_ready;
	bool napi_added;
	bool pool_posted;	/* RD pool filled at vendor pre-activation position */

	struct wave600_dma_ring tx;
	struct wave600_dma_ring rx;
	struct wave600_dma_ring mgmt_tx;
	struct wave600_dma_ring mgmt_rx;
	void *mgmt_tx_pool;
	dma_addr_t mgmt_tx_pool_dma;

	/*
	 * DC-mode (DATA_PATH_MODE_DC_MODE_0) hardware rings, vendor
	 * DC_DP_RING_HW_MODE0 shape: soc2dev (tx_in) and dev2soc (rx_out)
	 * use 16-byte HW HDs, soc2dev_ret (tx_out) uses 4-byte HDs. On the
	 * vendor device the kernel DC-DP infrastructure feeds them; here
	 * they are allocated and left idle so the DATA_PATH_INIT message
	 * matches the vendor configuration exactly.
	 */
	void *dc_tx_in;
	dma_addr_t dc_tx_in_dma;
	void *dc_rx_out;
	dma_addr_t dc_rx_out_dma;
	void *dc_tx_out;
	dma_addr_t dc_tx_out_dma;
	u32 dc_ring_size;

	/* SHRAM RD pool geometry (CHI sRdPOOL) */
	u32 rd_pool_num;
	u32 rd_size;
	u32 rd_payload_off;
	u32 rd_idx_off;
	u32 rd_mgmt_num;
	u32 rd_pool_off;	/* BAR0 offset of the data pool */

	/* Host mirrors of the FW-side pool buffers */
	struct wave600_dma_buf *rx_bufs;
	struct wave600_dma_buf *mgmt_bufs;

	/* TX bookkeeping: one mirror per ring slot */
	spinlock_t tx_lock;
	struct sk_buff *tx_skbs[WAVE600_DMA_RING_SIZE];
	dma_addr_t tx_dma[WAVE600_DMA_RING_SIZE];
	u32 tx_len[WAVE600_DMA_RING_SIZE];
	u32 tx_write;
	u32 tx_in_use;

	/* statistics */
	u32 rx_packets;
	u32 tx_packets;
	u32 tx_done;
	u32 tx_dropped;
};

int wave600_dma_post_rx_pool(struct wave600_dev *wdev);
int wave600_dma_init(struct wave600_dev *wdev);
int wave600_dma_data_path_start(struct wave600_dev *wdev);
void wave600_dma_attach(struct wave600_dev *wdev);
void wave600_dma_cleanup(struct wave600_dev *wdev);
void wave600_dma_enable_napi(struct wave600_dev *wdev);
bool wave600_dma_schedule_napi(struct wave600_dev *wdev);
netdev_tx_t wave600_dma_start_xmit(struct sk_buff *skb, struct net_device *ndev);
int wave600_dma_mgmt_tx(struct wave600_radio *radio, const u8 *buf, size_t len);
int wave600_dma_mgmt_tx_eapol(struct wave600_radio *radio, const u8 *buf, size_t len);
extern const struct net_device_ops wave600_netdev_ops;
void wave600_dma_stop_netdev(struct wave600_dev *wdev);

#endif /* __WAVE600_DMA_H__ */
