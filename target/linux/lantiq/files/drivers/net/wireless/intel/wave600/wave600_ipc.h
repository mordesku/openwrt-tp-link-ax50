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
#include <linux/if_ether.h>

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
#define WAVE600_MSG_TYPE_DAT_LOGGER_IND 9  /* Logger buffer (vendor msgid.h) */

/*
 * DBG channel message IDs (vendor mhi_umi.h UMI_MSG_DBG_REQ/CFM/IND).
 * The DBG channel is a second, independent message ring pair (sDBG)
 * with its own blocking manager (vendor txdm): SW_RESET goes over MAN,
 * the FW logger severity and DUT/debug requests go over DBG.
 */
#define WAVE600_UMI_MSG_DBG_REQ(num)   WAVE600_UMI_MSG(WAVE600_MSG_REQ, WAVE600_MSG_TYPE_DBG_REQ, num)
#define WAVE600_UMI_MSG_DBG_CFM(num)   WAVE600_UMI_MSG(WAVE600_MSG_CFM, WAVE600_MSG_TYPE_DBG_REQ, num)
#define WAVE600_UMI_MSG_DBG_IND(num)   WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_DBG_IND, num)

/* UM_DBG_LOGGER_SET_SEVERITY (vendor mhi_umi.h:692) */
#define WAVE600_UM_DBG_LOGGER_SET_SEVERITY_REQ  WAVE600_UMI_MSG_DBG_REQ(0x34) /* 0x0634 */
#define WAVE600_MC_DBG_LOGGER_SET_SEVERITY_CFM  WAVE600_UMI_MSG_DBG_CFM(0x34) /* 0x1634 */

/*
 * UmiLoggerMsgSetSeverity_t (vendor mhi_umi.h:2150). Mailbox request
 * fields are MAC-endian (little-endian), matching the vendor
 * HOST_TO_MAC32 conversions.
 */
struct wave600_umi_logger_set_severity {
	u32 new_level_le;
	u32 target_cpu_le;
	u8 get_set_operation;
	u8 reserved[3];
} __packed;

/* LogAgentSeverityLevel_e (vendor mhi_umi.h:2026-2042) */
#define WAVE600_LOGGER_SEVERITY_INFORMATION9  9
/* UmiCpuId_e (vendor mhi_umi.h:98-104) */
#define WAVE600_UMI_CPU_ID_LM   0
#define WAVE600_UMI_CPU_ID_UM   1
#define WAVE600_UMI_CPU_ID_ALL  2

#define WAVE600_UMI_MSG(dir, type, num) (((dir) << 12) | ((type) << 8) | ((num) & 0xff))

#define WAVE600_UMI_MSG_MAN_REQ(num) WAVE600_UMI_MSG(WAVE600_MSG_REQ, WAVE600_MSG_TYPE_MAN_REQ, num)
#define WAVE600_UMI_MSG_MAN_CFM(num) WAVE600_UMI_MSG(WAVE600_MSG_CFM, WAVE600_MSG_TYPE_MAN_REQ, num)

/* Core UMI Management Message IDs */
#define WAVE600_UM_MAN_GET_MAC_VERSION_REQ   WAVE600_UMI_MSG_MAN_REQ(0x08) /* 0x0408 */
#define WAVE600_MC_MAN_GET_MAC_VERSION_CFM   WAVE600_UMI_MSG_MAN_CFM(0x08) /* 0x1408 */

#define WAVE600_UM_MAN_SW_RESET_MAC_REQ      WAVE600_UMI_MSG_MAN_REQ(0x0C) /* 0x040C */
#define WAVE600_MC_MAN_SW_RESET_MAC_CFM      WAVE600_UMI_MSG_MAN_CFM(0x0C) /* 0x140C */

/* Data path (Phase 7): bind the DDR rings to the firmware */
#define WAVE600_UM_MAN_DATA_PATH_INIT_REQ    WAVE600_UMI_MSG_MAN_REQ(0x0E) /* 0x040E */
#define WAVE600_MC_MAN_DATA_PATH_INIT_CFM   WAVE600_UMI_MSG_MAN_CFM(0x0E) /* 0x140E */

/* AP bring-up (Phase 8): vendor msgid.h values */
#define WAVE600_UM_MAN_SET_MIB_REQ           WAVE600_UMI_MSG_MAN_REQ(0x00) /* 0x0400 */
#define WAVE600_MC_MAN_SET_MIB_CFM          WAVE600_UMI_MSG_MAN_CFM(0x00) /* 0x1400 */
#define WAVE600_UM_MAN_GET_MIB_REQ           WAVE600_UMI_MSG_MAN_REQ(0x01) /* 0x0401 */
#define WAVE600_MC_MAN_GET_MIB_CFM          WAVE600_UMI_MSG_MAN_CFM(0x01) /* 0x1401 */
#define WAVE600_UM_MAN_ACTIVATE_REQ          WAVE600_UMI_MSG_MAN_REQ(0x03) /* 0x0403 */
#define WAVE600_MC_MAN_ACTIVATE_CFM         WAVE600_UMI_MSG_MAN_CFM(0x03) /* 0x1403 */
#define WAVE600_UM_SET_CHAN_REQ              WAVE600_UMI_MSG_MAN_REQ(0x12) /* 0x0412 */
#define WAVE600_MC_SET_CHAN_CFM              WAVE600_UMI_MSG_MAN_CFM(0x12) /* 0x1412 */
#define WAVE600_UM_MAN_VAP_DB_REQ            WAVE600_UMI_MSG_MAN_REQ(0x1E) /* 0x041E */
#define WAVE600_MC_MAN_VAP_DB_CFM            WAVE600_UMI_MSG_MAN_CFM(0x1E) /* 0x141E */
#define WAVE600_UM_MAN_STOP_VAP_TRAFFIC_REQ WAVE600_UMI_MSG_MAN_REQ(0x1D) /* 0x041D */
#define WAVE600_MC_MAN_STOP_VAP_TRAFFIC_CFM WAVE600_UMI_MSG_MAN_CFM(0x1D) /* 0x141D */
#define WAVE600_UM_MAN_SET_WMM_PARAMETERS_REQ WAVE600_UMI_MSG_MAN_REQ(0x0A) /* 0x040A */
#define WAVE600_UM_MAN_SET_WMM_PARAMETERS_CFM WAVE600_UMI_MSG_MAN_CFM(0x0A) /* 0x140A */
#define WAVE600_MC_MAN_SET_ANTENNAS_REQ      WAVE600_UMI_MSG_MAN_REQ(0x18) /* 0x0418 */
#define WAVE600_MC_MAN_SET_ANTENNAS_CFM      WAVE600_UMI_MSG_MAN_CFM(0x18) /* 0x1418 */
#define WAVE600_UM_MAN_ADD_VAP_REQ           WAVE600_UMI_MSG_MAN_REQ(0x1F) /* 0x041F */
#define WAVE600_MC_MAN_ADD_VAP_CFM           WAVE600_UMI_MSG_MAN_CFM(0x1F) /* 0x141F */
#define WAVE600_UM_MAN_MBSS_PRE_ACTIVATE_REQ WAVE600_UMI_MSG_MAN_REQ(0x52) /* 0x0452 */
#define WAVE600_MC_MAN_MBSS_PRE_ACTIVATE_CFM WAVE600_UMI_MSG_MAN_CFM(0x52) /* 0x1452 */
#define WAVE600_UM_MAN_SET_TPC_CONFIG_REQ   WAVE600_UMI_MSG_MAN_REQ(0x4F) /* 0x044F */
#define WAVE600_MC_MAN_SET_TPC_CONFIG_CFM  WAVE600_UMI_MSG_MAN_CFM(0x4F) /* 0x144F */
#define WAVE600_UM_MAN_SET_TPC_ANT_PARAMS_REQ WAVE600_UMI_MSG_MAN_REQ(0x4E) /* 0x044E */
#define WAVE600_MC_MAN_SET_TPC_ANT_PARAMS_CFM WAVE600_UMI_MSG_MAN_CFM(0x4E) /* 0x144E */
#define WAVE600_UM_MAN_ENABLE_RADIO_REQ    WAVE600_UMI_MSG_MAN_REQ(0x5C) /* 0x045C */
#define WAVE600_MC_MAN_ENABLE_RADIO_CFM    WAVE600_UMI_MSG_MAN_CFM(0x5C) /* 0x145C */
#define WAVE600_UM_MAN_SET_BEACON_INTERVAL_REQ WAVE600_UMI_MSG_MAN_REQ(0x76) /* 0x0476 */
#define WAVE600_MC_MAN_SET_BEACON_INTERVAL_CFM WAVE600_UMI_MSG_MAN_CFM(0x76) /* 0x1476 */
#define WAVE600_UM_MAN_REMOVE_VAP_REQ       WAVE600_UMI_MSG_MAN_REQ(0x20) /* 0x0420 */
#define WAVE600_MC_MAN_REMOVE_VAP_CFM       WAVE600_UMI_MSG_MAN_CFM(0x20) /* 0x1420 */
#define WAVE600_UM_BSS_SET_BSS_REQ           WAVE600_UMI_MSG_MAN_REQ(0x21) /* 0x0421 */
#define WAVE600_MC_BSS_SET_BSS_CFM           WAVE600_UMI_MSG_MAN_CFM(0x21) /* 0x1421 */
#define WAVE600_UM_MAN_STA_ADD_REQ           WAVE600_UMI_MSG_MAN_REQ(0x3B) /* 0x043B */
#define WAVE600_MC_MAN_STA_ADD_CFM           WAVE600_UMI_MSG_MAN_CFM(0x3B) /* 0x143B */
#define WAVE600_UM_MAN_STA_REMOVE_REQ        WAVE600_UMI_MSG_MAN_REQ(0x3C) /* 0x043C */
#define WAVE600_MC_MAN_STA_REMOVE_CFM        WAVE600_UMI_MSG_MAN_CFM(0x3C) /* 0x143C */
#define WAVE600_UM_MAN_STOP_TRAFFIC_REQ       WAVE600_UMI_MSG_MAN_REQ(0x44) /* 0x0444 */
#define WAVE600_MC_MAN_STOP_TRAFFIC_CFM       WAVE600_UMI_MSG_MAN_CFM(0x44) /* 0x1444 */
#define WAVE600_UM_MAN_SET_802_1X_FILTER_REQ  WAVE600_UMI_MSG_MAN_REQ(0x46) /* 0x0446 */
#define WAVE600_MC_MAN_SET_802_1X_FILTER_CFM  WAVE600_UMI_MSG_MAN_CFM(0x46) /* 0x1446 */
#define WAVE600_UM_MAN_SET_BEACON_TEMPLATE_REQ  WAVE600_UMI_MSG_MAN_REQ(0x3D) /* 0x043D */
#define WAVE600_MC_MAN_SET_BEACON_TEMPLATE_CFM  WAVE600_UMI_MSG_MAN_CFM(0x3D) /* 0x143D */

/* Firmware-initiated events (UMI_MSG_MAN_IND(type, dir=IND, num)) */
#define WAVE600_MC_MAN_NETWORK_EVENT_IND     WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x26) /* 0x3326 */
#define WAVE600_MC_MAN_MAC_EVENT_IND         WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x38) /* 0x3338 */

/*
 * UMI_MAC_EVENT_t (vendor mhi_mac_event.h:158-174): sent by firmware on
 * MC_MAN_MAC_EVENT_IND (0x3338) when a CPU exception or MTLK_ASSERT occurs.
 * All 32-bit fields are MAC-endian (LE) inside the UMI payload.
 */
struct wave600_umi_mac_event {
	__le32 event_id;
	__le32 core_num;
	__le32 timestamp;
	__le32 cause_or_line;
	__le32 status_reg;
	__le32 epc_reg;
	__le32 file_id;
	__le32 group_id;
	__le32 origin_id;
	__le32 fw_interface;
} __packed;

/* Platform Specific Data load (vendor mhi_umi.h: sent before RF calibration) */
#define WAVE600_UM_MAN_HDK_ANT_CONFIG_REQ         WAVE600_UMI_MSG_MAN_REQ(0x7E) /* 0x047E */
#define WAVE600_MC_MAN_HDK_ANT_CONFIG_CFM         WAVE600_UMI_MSG_MAN_CFM(0x7E) /* 0x147E */
#define WAVE600_UM_MAN_PLATFORM_DATA_FIELDS_REQ   WAVE600_UMI_MSG_MAN_REQ(0x57) /* 0x0457 */
#define WAVE600_MC_MAN_PLATFORM_DATA_FIELDS_CFM   WAVE600_UMI_MSG_MAN_CFM(0x57) /* 0x1457 */
#define WAVE600_UM_MAN_PLATFORM_TABLE_REQ          WAVE600_UMI_MSG_MAN_REQ(0x58) /* 0x0458 */
#define WAVE600_MC_MAN_PLATFORM_TABLE_CFM         WAVE600_UMI_MSG_MAN_CFM(0x58) /* 0x1458 */

/*
 * HW dependent configuration (vendor hw_mmb.c _mtlk_mmb_send_hw_dep_cfg,
 * sent right after DATA_PATH_INIT): UmiHwDependentConfig_t with the board
 * xtal from the calibration EEPROM; the gen6 CCR AFE getter is a dummy
 * returning zeros (mtlk_ccr_pcieg6.c). Mailbox payload: multi-byte
 * request fields are MAC-endian (little-endian) like every vendor
 * HOST_TO_MAC32/16 conversion; FW-written data is read with raw BE
 * accessors (vendor mmb_ops.h default branch on GRX500 BE).
 */
#define WAVE600_UM_MAN_HW_DEPENDENT_CONFIG_REQ    WAVE600_UMI_MSG_MAN_REQ(0x1C) /* 0x041C */
#define WAVE600_MC_MAN_HW_DEPENDENT_CONFIG_CFM    WAVE600_UMI_MSG_MAN_CFM(0x1C) /* 0x141C */

struct wave600_umi_hw_dep_cfg {
	u32 xtal;
	u32 afe[2];
} __packed;

/* UMI_SET_ANTENNAS (0x0418) - Vendor MC_MAN_SET_ANTENNAS_REQ */
struct wave600_umi_set_antennas {
	u8 tx_ants_mask;
	u8 rx_ants_mask;
	u8 status;
	u8 reserved;
} __packed;

/* UMI_AC_WMM_PARAMS and UMI_SET_WMM_PARAMETERS (0x040A) */
struct wave600_umi_ac_wmm_params {
	__le16 cw_min;
	__le16 cw_max;
	__le16 txop_limit;
	u8 aifsn;
	u8 acm_flag;
} __packed;

struct wave600_umi_set_wmm_params {
	u8 vap_id;
	u8 status;
	struct wave600_umi_ac_wmm_params ac[4];
} __packed;

/* Vendor UMI_STA_ADD (0x043B) */
struct wave600_umi_sta_add {
	__le16 sid;
	u8 vap_id;
	u8 status;
	u8 listen_interval;
	u8 bss_coex_20_40;
	u8 uapsd_queues;
	u8 max_sp;
	__le16 aid;
	u8 addr[ETH_ALEN];
	u8 rates[32];
	__le16 ht_cap_info;
	u8 ampdu_param;
	u8 rx_mcs_bitmask[10];
	u8 rates_len;
	u8 flags;
	u8 flags_ext;
	__le32 vht_cap_info;
	u8 vht_mcs_nss[8];
	u8 he_mac_phy_cap_info[18];
	u8 he_mcs_nss[8];
	u8 he_ppe_th[13];
	u8 reserved;
	__le32 tx_bf_caps;
	u8 vht_op_mode_notif;
	u8 he_op_mode_notif;
	u8 wds_client_type;
	s8 rssi;
} __packed;

/* Vendor UMI_STA_REMOVE (0x043C) */
struct wave600_umi_sta_remove {
	__le16 sid;
	u8 status;
	u8 reserved;
} __packed;

/* Vendor UMI_STOP_TRAFFIC (0x0444) */
struct wave600_umi_stop_traffic {
	__le16 sid;
	u8 status;
	u8 reserved;
} __packed;

/* Vendor UMI_802_1X_FILTER (0x0446) */
struct wave600_umi_802_1x_filter {
	__le16 status;
	__le16 sid;
	u8 is_filter_open;
	u8 reserved;
} __packed;

/*
 * FW statistics (vendor hw_mmb.c mhi_stats path, mhi_umi.h):
 * counter layout descriptors and counter values are DMA'd by the firmware
 * into a host-provided DDR buffer whose geometry comes from the CHI
 * vector area statistics manager extension.
 */
#define WAVE600_UM_MAN_GET_STATISTICS_TABLE_REQ    WAVE600_UMI_MSG_MAN_REQ(0x2E) /* 0x042E */
#define WAVE600_MC_MAN_GET_STATISTICS_TABLE_CFM   WAVE600_UMI_MSG_MAN_CFM(0x2E) /* 0x142E */
#define WAVE600_UM_MAN_GET_STATISTICS_DATA_REQ    WAVE600_UMI_MSG_MAN_REQ(0x60) /* 0x0460 */
#define WAVE600_MC_MAN_GET_STATISTICS_DATA_CFM    WAVE600_UMI_MSG_MAN_CFM(0x60) /* 0x1460 */

/*
 * UMI_GET_STATISTICS_DATA payload (vendor mhi_umi.h). Mailbox request
 * fields are MAC-endian (little-endian), matching the vendor
 * HOST_TO_MAC32/16 coding for every mailbox field.
 */
struct wave600_umi_get_statistics {
	u32 ddr_buffer_address;
	u16 length;
	u8 status;
} __packed;

/*
 * Calibration data upload (vendor hw_mmb.c mtlk_efuse_eeprom_load_to_hw,
 * sent for the EEPROM new format between the ProgModel download and the
 * HDK config / PSDB steps):
 *  - UM_MAN_SET_AFE_CALIBRATION_DATA_REQ (0x0459): the raw 19-byte
 *    wave_efuse_afe_data_v6_t block read from the device eFuse
 *    (eFuse bytes 28..46, GEN6 size), payload_size = 19.
 *  - UM_MAN_SET_RSSI_CAL_PARAMS_REQ (0x045B): the raw RSSI CIS section
 *    of the requested band from the calibration file
 *    (UMI_SET_RSSI_CAL_CONFIG = { u8 size; u8 data[131] }; the vendor
 *    sets payload_size = size, one byte short of the prepared struct -
 *    replicated byte-exact).
 *  - UM_MAN_SET_RFIC_CALIBRATION_DATA_REQ (0x045A) is v5-eFuse-only in
 *    the vendor (cis.rfic is never filled for the v6 format) and is
 *    therefore not implemented.
 */
#define WAVE600_AFE_CALIBRATION_DATA_SIZE_GEN6      19
#define WAVE600_MAX_RSSI_TABLE_SIZE                131

#define WAVE600_UM_MAN_SET_AFE_CAL_DATA_REQ        WAVE600_UMI_MSG_MAN_REQ(0x59) /* 0x0459 */
#define WAVE600_MC_MAN_SET_AFE_CAL_DATA_CFM        WAVE600_UMI_MSG_MAN_CFM(0x59) /* 0x1459 */
#define WAVE600_UM_MAN_SET_RSSI_CAL_PARAMS_REQ     WAVE600_UMI_MSG_MAN_REQ(0x5B) /* 0x045B */
#define WAVE600_MC_MAN_SET_RSSI_CAL_PARAMS_CFM     WAVE600_UMI_MSG_MAN_CFM(0x5B) /* 0x145B */

#define WAVE600_UM_DOWNLOAD_PROG_MODEL_REQ           WAVE600_UMI_MSG_MAN_REQ(0x02) /* 0x0402 */
#define WAVE600_MC_DOWNLOAD_PROG_MODEL_CFM           WAVE600_UMI_MSG_MAN_CFM(0x02) /* 0x1402 */

#define WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_REQ WAVE600_UMI_MSG_MAN_REQ(0x1B) /* 0x041B */
#define WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_CFM WAVE600_UMI_MSG_MAN_CFM(0x1B) /* 0x141B */

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

/* Bounce buffer for the MAN_IND slot IOM write-back (vendor sizeof(SHRAM_IND_MSG) == 620) */
#define WAVE600_IPC_MAX_IND_BOUNCE 620

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

	/*
	 * SHRAM Debug area geometry (CHI sDBG). Vendor NUM_DBG_REQS and
	 * NUM_DBG_INDS are both 1 (gen6 workaround, vendor shram.h:208-210),
	 * so both ring indices are always 0 and the slot stride is only
	 * used for the bounded bounce copies.
	 */
	u32 dbg_req_offset;
	u32 dbg_req_num;
	u32 dbg_req_slot_size;
	u32 dbg_ind_offset;
	u32 dbg_ind_num;
	u32 dbg_ind_slot_size;

	/* Single-slot DBG manager (vendor txdm over HW_DBG_REQ_BDR) */
	struct wave600_ipc_cmd dbg_cmd;
	u32 dbg_cfm_count;
	u32 dbg_ind_count;
	u32 logger_ind_count;	/* DAT_LOGGER_IND descriptors echoed back */

	/* Pending commands */
	struct wave600_ipc_cmd cmds[WAVE600_IPC_MAX_MAN_REQS];

	/*
	 * FW logger extension buffers (vendor _mtlk_mmb_init_fw_logger,
	 * pre-activation step HW_FW_LOGGER). The firmware DMAs log records
	 * into these host buffers and posts DAT_LOGGER_IND descriptors.
	 */
	struct wave600_fw_log_buf {
		void *virt;
		dma_addr_t dma;
		u32 size;
	} *fw_log_bufs;
	u32 fw_log_bufs_num;

	/* Deferred interrupt processing (vendor mtlk_hw_mmb_deferred_handler) */
	struct tasklet_struct tasklet;

	/* ISR / tasklet statistics (diagnostics for IRQ storm triage) */
	u32 isr_total;
	u32 isr_spurious;
	u32 cfm_count;
	u32 ind_count;
	u32 empty_runs;
	bool storm_latched;
	u32 irq_status;		/* status snapshot for the tasklet */

	bool irq_enabled;
};

int wave600_ipc_init(struct wave600_dev *wdev);
void wave600_ipc_cleanup(struct wave600_dev *wdev);
int wave600_ipc_cmd(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
		    const void *req_payload, size_t req_len,
		    void *cfm_payload, size_t cfm_len, unsigned long timeout_ms);
int wave600_ipc_cmd_vap(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
			const void *req_payload, size_t req_len,
			void *cfm_payload, size_t cfm_len, unsigned long timeout_ms,
			u32 vap);
/*
 * Blocking command over the DBG ring pair (vendor txdm manager,
 * _mtlk_mmb_send_fw_log_severity path). Same contract as
 * wave600_ipc_cmd but the slot/descriptor live on the sDBG areas.
 */
int wave600_ipc_dbg_cmd(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
			const void *req_payload, size_t req_len,
			void *cfm_payload, size_t cfm_len,
			unsigned long timeout_ms);
/*
 * UM_DBG_LOGGER_SET_SEVERITY_REQ 0x0634 (vendor finalize step
 * HW_FW_LOG_SEVERITY: LOGGER_SEVERITY_DEFAULT_LEVEL, UMI_CPU_ID_UM).
 */
int wave600_ipc_dbg_set_severity(struct wave600_dev *wdev, u32 level, u32 cpu);
/*
 * FW logger extension buffer posting (vendor pre-activation step
 * HW_FW_LOGGER): allocate the LM/UM log buffers described by the CHI
 * logger extensions and write their DMA addresses into the SHRAM
 * buffer descriptor ring. Non-fatal when the extensions are absent.
 */
int wave600_ipc_fw_logger_init(struct wave600_dev *wdev);
void wave600_ipc_fw_logger_cleanup(struct wave600_dev *wdev);
void wave600_ipc_diag_fw_state(struct wave600_dev *wdev, const char *tag);
irqreturn_t wave600_ipc_isr(int irq, void *dev_id);

#endif /* __WAVE600_IPC_H__ */
