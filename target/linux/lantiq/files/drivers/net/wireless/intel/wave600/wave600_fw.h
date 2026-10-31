/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Firmware Staging & Boot Engine
 */

#ifndef __WAVE600_FW_H__
#define __WAVE600_FW_H__

#include "wave600.h"

#define WAVE600_FW_UPPER_MAC      "ap_ram_gen6_wrx_600_real_phy.bin"
#define WAVE600_FW_TX_SENDER      "tx_sender_gen6.bin"
/*
 * Host Interface image: standard software-mode image (host_interface_gen6.bin)
 * used with DATA_PATH_MODE_DC_NONE for host-driven descriptor rings.
 * (The dc_mode0 variant is for the vendor PPE DirectConnect hardware datapath).
 */
#define WAVE600_FW_HOST_IF        "host_interface_gen6.bin"
#define WAVE600_FW_RX_HANDLER     "rx_handler_gen6.bin"

int wave600_fw_load_and_boot(struct wave600_dev *wdev);

#endif /* __WAVE600_FW_H__ */
