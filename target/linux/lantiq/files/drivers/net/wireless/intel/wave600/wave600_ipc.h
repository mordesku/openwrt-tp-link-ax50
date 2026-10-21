/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Mailbox IPC & UMI Definitions
 */

#ifndef __WAVE600_IPC_H__
#define __WAVE600_IPC_H__

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

struct wave600_dev;

/*
 * UMI message directions & types (matching vendor msgid.h)
 */
#define WAVE600_MSG_REQ               0
#define WAVE600_MSG_CFM               1
#define WAVE600_MSG_RES               2
#define WAVE600_MSG_IND               3

#define WAVE600_MSG_TYPE_NULL         0
#define WAVE600_MSG_TYPE_DAT_IND      1
#define WAVE600_MSG_TYPE_DAT_REQ      2
#define WAVE600_MSG_TYPE_MAN_IND      3
#define WAVE600_MSG_TYPE_MAN_REQ      4
#define WAVE600_MSG_TYPE_DBG_IND      5
#define WAVE600_MSG_TYPE_DBG_REQ      6
#define WAVE600_MSG_TYPE_BSS_IND      7
#define WAVE600_MSG_TYPE_BSS_REQ      8

#define WAVE600_UMI_MSG(dir, type, num) (((dir) << 12) | ((type) << 8) | ((num) & 0xff))

#define WAVE600_UMI_MSG_MAN_REQ(num) WAVE600_UMI_MSG(WAVE600_MSG_REQ, WAVE600_MSG_TYPE_MAN_REQ, num)
#define WAVE600_UMI_MSG_MAN_CFM(num) WAVE600_UMI_MSG(WAVE600_MSG_CFM, WAVE600_MSG_TYPE_MAN_REQ, num)

/* Core UMI Management Message IDs */
#define WAVE600_UM_MAN_GET_MAC_VERSION_REQ   WAVE600_UMI_MSG_MAN_REQ(0x08) /* 0x0408 */
#define WAVE600_MC_MAN_GET_MAC_VERSION_CFM   WAVE600_UMI_MSG_MAN_CFM(0x08) /* 0x1408 */

#define WAVE600_UM_MAN_SW_RESET_MAC_REQ      WAVE600_UMI_MSG_MAN_REQ(0x0C) /* 0x040C */
#define WAVE600_MC_MAN_SW_RESET_MAC_CFM      WAVE600_UMI_MSG_MAN_CFM(0x0C) /* 0x140C */

#define WAVE600_UM_DOWNLOAD_PROG_MODEL_REQ           WAVE600_UMI_MSG_MAN_REQ(0x02) /* 0x0402 */
#define WAVE600_MC_DOWNLOAD_PROG_MODEL_CFM           WAVE600_UMI_MSG_MAN_CFM(0x02) /* 0x1402 */

#define WAVE600_UM_MAN_HDK_CONFIG_REQ                WAVE600_UMI_MSG_MAN_REQ(0x50) /* 0x0450 */
#define WAVE600_MC_MAN_HDK_CONFIG_CFM                WAVE600_UMI_MSG_MAN_CFM(0x50) /* 0x1450 */

#define WAVE600_UM_MAN_CALIBRATE_REQ                 WAVE600_UMI_MSG_MAN_REQ(0x53) /* 0x0453 */
#define WAVE600_MC_MAN_CALIBRATE_CFM                 WAVE600_UMI_MSG_MAN_CFM(0x53) /* 0x1453 */

#define WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_REQ WAVE600_UMI_MSG_MAN_REQ(0x80) /* 0x0480 */
#define WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_CFM WAVE600_UMI_MSG_MAN_CFM(0x80) /* 0x1480 */

#define WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_REQ WAVE600_UMI_MSG_MAN_REQ(0x81) /* 0x0481 */
#define WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_CFM WAVE600_UMI_MSG_MAN_CFM(0x81) /* 0x1481 */

/* Host IRQ bitmasks (matching vendor MacGeneralRegs.h and mtlk_ccr_defs.h) */
#define WAVE600_IRQ_TXOUT     BIT(0)
#define WAVE600_IRQ_RX        BIT(1)
#define WAVE600_IRQ_BSS_CFM   BIT(2)
#define WAVE600_IRQ_BSS_IND   BIT(3)
#define WAVE600_IRQ_MAILBOX   BIT(4) /* macHostMailboxCountNotEmptyIrq */
#define WAVE600_IRQ_CONTROL   BIT(5) /* rabPhiInterrupt */
#define WAVE600_IRQ_DMA_ERR   BIT(6)

/* Single-IRQ legacy mode mask, matching vendor MTLK_IRQ_ALL (hw_mmb.c ihd_1) */
#define WAVE600_IRQ_ALL       (WAVE600_IRQ_CONTROL | WAVE600_IRQ_BSS_IND | WAVE600_IRQ_BSS_CFM | \
			       WAVE600_IRQ_RX | WAVE600_IRQ_TXOUT) /* 0x2F matching vendor MTLK_IRQ_ALL */

/*
 * Storm guard: number of consecutive tasklet runs that consumed no message
 * before the driver gives up and leaves HOST_IRQ_MASK = 0 (system survives,
 * register snapshot is logged once). Inverse of vendor MTCFG_LEGACY_INT_RECOVERY_MON.
 */
#define WAVE600_IPC_STORM_LIMIT 1000

/* Bounce buffer for the MAN_IND slot IOM write-back (vendor _mtlk_mmb_resp_cm_ind) */
#define WAVE600_IPC_MAX_IND_BOUNCE 256

/*
 * HIM 32-bit descriptor encoding in sFifoQ
 * Bits 0..4:   Message Type (ARRAY_MAN_REQ, ARRAY_MAN_IND, etc.)
 * Bits 5..14:  Index of buffer in sMAN/sDBG/sDAT
 * Bit 15:      Reserved
 * Bits 16..17: Radio ID
 * Bits 18..23: VAP ID
 * Bits 24..31: Reserved
 */
static inline u32 wave600_msg_to_descr(u32 type, u32 index, u32 radio, u32 vap)
{
	return ((type & 0x1f) << 0) |
	       ((index & 0x3ff) << 5) |
	       ((radio & 0x3) << 16) |
	       ((vap & 0x3f) << 18);
}

static inline u32 wave600_descr_get_type(u32 descr)  { return (descr >> 0) & 0x1f; }
static inline u32 wave600_descr_get_index(u32 descr) { return (descr >> 5) & 0x3ff; }
static inline u32 wave600_descr_get_radio(u32 descr) { return (descr >> 16) & 0x3; }
static inline u32 wave600_descr_get_vap(u32 descr)   { return (descr >> 18) & 0x3f; }

/*
 * UMI Message Header in Shared RAM (16 bytes packed)
 */
struct wave600_umi_msg_hdr {
	u32 next;
	u8 pad1;
	u8 persistent;
	u16 msg_id;
	u32 pad2;
	u32 msg_ref;
} __packed;

/*
 * MAC Version response payload structure
 */
struct wave600_mac_version {
	u8 length;
	u8 reserved[3];
	char ver[36];
} __packed;

/*
 * HDK Configuration structure (UM_MAN_HDK_CONFIG_REQ)
 */
struct wave600_umi_hdk_config {
	u32 cal_buffer_base_addr;
	u8 get_set_operation;
	u8 set_channel_mode;
	u8 reserved[2];
	u8 num_tx_ants;
	u8 num_rx_ants;
	u16 eeprom_version;
	u8 num_points_5ghz;
	u8 num_points_2ghz;
	u8 band;
	u8 reserved2;
	u32 offline_cal_mask;
	u32 online_cal_mask;
} __packed;

#define WAVE600_MAX_CALIB_CHANS 32

/*
 * Calibration parameters structure (UM_MAN_CALIBRATE_REQ)
 */
struct wave600_umi_calibrate_params {
	u32 stored_calib_data;
	u32 status;
	u8 chan_nums[WAVE600_MAX_CALIB_CHANS];
	u8 num_chans;
	u8 chan_width;
	u8 reserved[2];
} __packed;

/*
 * Mailbox IPC Pending Command Tracker
 */
struct wave600_ipc_cmd {
	struct completion done;
	u16 req_id;
	u16 cfm_id;
	void *resp_buf;
	size_t resp_len;
	int status;
	bool in_use;
};

#define WAVE600_IPC_MAX_MAN_REQS 8

/*
 * Mailbox IPC Context
 */
struct wave600_ipc {
	struct wave600_dev *wdev;
	spinlock_t lock;

	/* SHRAM FIFO Queue Geometry */
	u32 req_offset;
	u32 req_num;
	u32 req_idx;

	u32 ind_offset;
	u32 ind_num;
	u32 ind_idx;

	/* SHRAM Management Buffer Geometry */
	u32 man_req_offset;
	u32 man_req_num;
	u32 man_req_slot_size;

	u32 man_ind_offset;
	u32 man_ind_num;
	u32 man_ind_slot_size;

	/* Pending commands */
	struct wave600_ipc_cmd cmds[WAVE600_IPC_MAX_MAN_REQS];

	/* Deferred interrupt processing (vendor mtlk_hw_mmb_deferred_handler) */
	struct tasklet_struct tasklet;

	/*
	 * GRX500 Root Complex app register block of the root port serving
	 * this endpoint (INTx storm diagnostics / RC-side latch clear).
	 */
	void __iomem *rc_app;
	u32 rc_port;

	/* ISR / tasklet statistics (diagnostics for IRQ storm triage) */
	u32 isr_total;
	u32 isr_spurious;
	u32 cfm_count;
	u32 ind_count;
	u32 empty_runs;
	bool storm_latched;

	bool irq_enabled;
};

int wave600_ipc_init(struct wave600_dev *wdev);
void wave600_ipc_cleanup(struct wave600_dev *wdev);
int wave600_ipc_cmd(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
		    const void *req_payload, size_t req_len,
		    void *cfm_payload, size_t cfm_len, unsigned long timeout_ms);
irqreturn_t wave600_ipc_isr(int irq, void *dev_id);

#endif /* __WAVE600_IPC_H__ */
