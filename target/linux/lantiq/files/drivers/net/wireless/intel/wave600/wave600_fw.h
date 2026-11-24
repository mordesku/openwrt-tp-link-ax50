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
 * Host Interface image: host_interface_gen6.bin is for DATA_PATH_MODE_DC_NONE
 * (20-byte SW HOST_DSC rings), whereas host_interface_gen6_dc_mode0.bin is
 * required for DATA_PATH_MODE_DC_MODE_0 (16-byte GRX500 CBM/DCDP rings).
 */
#define WAVE600_FW_HOST_IF          "host_interface_gen6.bin"
#define WAVE600_FW_HOST_IF_DC_MODE0 "host_interface_gen6_dc_mode0.bin"
#define WAVE600_FW_RX_HANDLER     "rx_handler_gen6.bin"

int wave600_fw_load_and_boot(struct wave600_dev *wdev);

#endif /* __WAVE600_FW_H__ */
