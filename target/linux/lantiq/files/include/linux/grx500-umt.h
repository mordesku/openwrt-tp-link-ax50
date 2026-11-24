/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GRX500_UMT_H
#define _LINUX_GRX500_UMT_H

#include <linux/types.h>

#define GRX500_UMT_PORTS_NUM		4
#define GRX500_UMT_PERIOD_DEFAULT_US	200

enum grx500_umt_mode {
	GRX500_UMT_SELFCNT_MODE = 0,
	GRX500_UMT_USER_MODE = 1,
};

enum grx500_umt_msg_mode {
	GRX500_UMT_NO_MSG = 0,
	GRX500_UMT_MSG0_ONLY = 1,
	GRX500_UMT_MSG1_ONLY = 2,
	GRX500_UMT_MSG0_MSG1 = 3,
};

int grx500_umt_request(u32 ep_id, u32 cbm_pid, u32 *dma_ctrlid,
		       u32 *dma_cid, u32 *umt_id);
int grx500_umt_set_mode(u32 umt_id, u32 ep_id, enum grx500_umt_mode umt_mode,
			enum grx500_umt_msg_mode msg_mode, u32 phy_dst,
			u32 period_us, bool enable);
int grx500_umt_enable(u32 umt_id, u32 ep_id, bool enable);
int grx500_umt_release(u32 umt_id, u32 ep_id);
void grx500_umt_debug_info(u32 out[8]);
void grx500_umt_sw_set_dqpc(u32 umt_id, void __iomem *dqpc_reg);
void grx500_umt_sw_debug(u32 umt_id, u32 out[8]);

#endif
