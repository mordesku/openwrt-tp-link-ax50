/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Shared RAM (SHRAM) & CHI Definitions
 */

#ifndef __WAVE600_CHI_H__
#define __WAVE600_CHI_H__

#include <linux/types.h>

#define WAVE600_CHI_MAGIC                   0xBEADFEED
#define WAVE600_HOST_EXTENSION_MAGIC        0xBEADFEED

#define WAVE600_CHI_EXT_ID_CARD_CFG         12

/* Band configuration */
#define WAVE600_BAND_CFG_SINGLE_BAND        0
#define WAVE600_BAND_CFG_DUAL_BAND          1

/* PCIe configuration */
#define WAVE600_PCI_CFG_SINGLE_PCI          0
#define WAVE600_PCI_CFG_DUAL_PCI            1

struct wave600_chi_area_desc {
	__le32 ind_start_offset;
	__le32 ind_num_elements;
	__le32 req_start_offset;
	__le32 req_num_elements;
} __packed;

struct wave600_chi_ring {
	__le32 in_counter_addr;
	__le32 in_status_addr;
	__le32 out_counter_addr;
	__le32 out_status_addr;
	__le32 in_counter_addr_be;
	__le32 out_freed_counter_addr_be;
} __packed;

struct wave600_chi_rd_pool {
	__le32 pool_base_addr;
	__le32 rd_size;
	__le32 payload_offset;
	__le32 bd_index_offset;
	__le32 num_rds;
	__le32 num_mgt_rds;
} __packed;

struct wave600_chi_basic {
	struct wave600_chi_area_desc fifo_q;
	struct wave600_chi_area_desc dat;
	struct wave600_chi_area_desc man;
	struct wave600_chi_area_desc dbg;
	struct wave600_chi_area_desc bss;
	struct wave600_chi_ring tx_ring;
	struct wave600_chi_ring rx_ring;
	struct wave600_chi_ring mang_tx_ring;
	struct wave600_chi_ring mang_rx_ring;
	struct wave600_chi_rd_pool rd_pool;
	__le32 magic; /* Written by ARC firmware when ready (0xBEADFEED) */
} __packed;

struct wave600_chi_ext_header {
	__le32 magic;
	__le32 id;
	__le32 size;
} __packed;

struct wave600_chi_calibr_ext {
	struct wave600_chi_ext_header header;
	__le32 desc_location;
	__le32 buf_requested_size;
	__le32 psd_progmodel_addr;
} __packed;

struct wave600_chi_mips_control_ext {
	struct wave600_chi_ext_header header;
	__le32 desc_location;
} __packed;

struct wave600_chi_logger_ext {
	struct wave600_chi_ext_header header;
	__le32 buf_desc_location;
	__le32 num_buf_descs;
} __packed;

struct wave600_chi_fw_caps_stas_ext {
	struct wave600_chi_ext_header header;
	__le32 num_stations;
} __packed;

struct wave600_chi_fw_caps_vaps_ext {
	struct wave600_chi_ext_header header;
	__le32 num_vaps;
} __packed;

struct wave600_chi_logger_buf_info {
	__le32 size;
	__le32 num_buffers;
	__le32 buf_pool_addr;
} __packed;

struct wave600_chi_logger_buf_info_ext {
	struct wave600_chi_ext_header header;
	struct wave600_chi_logger_buf_info lm_buf_info;
	struct wave600_chi_logger_buf_info um_buf_info;
} __packed;

struct wave600_chi_stats_mgr_ext {
	struct wave600_chi_ext_header header;
	__le32 stats_data_size;
	__le32 stats_table_size;
	__le32 type_table_list[16];
} __packed;

struct wave600_chi_frag_ring_ext {
	struct wave600_chi_ext_header header;
	__le32 frag_ring_addr;
	__le32 frag_ring_num_entries;
} __packed;

struct wave600_chi_logger_rd_ext {
	struct wave600_chi_ext_header header;
	__le16 first_rd_index;
	__le16 num_rds;
} __packed;

struct wave600_chi_debug_ext {
	struct wave600_chi_ext_header header;
	__le32 core0_info;
	__le32 core1_info;
	__le32 core2_info;
} __packed;

struct wave600_chi_card_cfg_data {
	__le32 band_cfg;       /* 0x1 - dual band */
	__le32 pcie_cfg;       /* 0x0 - single PCIe */
	__le32 platform_type;  /* 0x0 - real ASIC */
} __packed;

struct wave600_chi_card_cfg_ext {
	struct wave600_chi_ext_header header;
	struct wave600_chi_card_cfg_data data;
} __packed;

struct wave600_chi_area {
	struct wave600_chi_basic basic;
	struct wave600_chi_calibr_ext calibr_ext;
	struct wave600_chi_mips_control_ext mips_control_ext;
	struct wave600_chi_logger_ext logger_ext;
	struct wave600_chi_fw_caps_stas_ext fw_caps_stas_ext;
	struct wave600_chi_fw_caps_vaps_ext fw_caps_vaps_ext;
	struct wave600_chi_logger_buf_info_ext logger_buf_info_ext;
	struct wave600_chi_stats_mgr_ext stats_mgr_ext;
	struct wave600_chi_frag_ring_ext frag_ring_ext;
	struct wave600_chi_logger_rd_ext logger_rd_ext;
	struct wave600_chi_debug_ext debug_ext;
	struct wave600_chi_card_cfg_ext card_cfg_ext;
} __packed;

#endif /* __WAVE600_CHI_H__ */
