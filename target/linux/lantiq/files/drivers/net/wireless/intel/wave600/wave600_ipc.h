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
/* UM_MAN_SET_POWER_LIMIT (0x045F): UMI_TX_POWER_LIMIT get/set powerLimitOffset */
#define WAVE600_UM_MAN_SET_POWER_LIMIT_REQ   WAVE600_UMI_MSG_MAN_REQ(0x5F) /* 0x045F */
#define WAVE600_MC_MAN_SET_POWER_LIMIT_CFM   WAVE600_UMI_MSG_MAN_CFM(0x5F) /* 0x145F */
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
#define WAVE600_UM_MAN_REQUEST_SID_REQ        WAVE600_UMI_MSG_MAN_REQ(0x42) /* 0x0442 */
#define WAVE600_MC_MAN_REQUEST_SID_CFM        WAVE600_UMI_MSG_MAN_CFM(0x42) /* 0x1442 */
#define WAVE600_UM_MAN_REMOVE_SID_REQ         WAVE600_UMI_MSG_MAN_REQ(0x43) /* 0x0443 */
#define WAVE600_MC_MAN_REMOVE_SID_CFM         WAVE600_UMI_MSG_MAN_CFM(0x43) /* 0x1443 */
#define WAVE600_UM_MAN_STOP_TRAFFIC_REQ       WAVE600_UMI_MSG_MAN_REQ(0x44) /* 0x0444 */
#define WAVE600_MC_MAN_STOP_TRAFFIC_CFM       WAVE600_UMI_MSG_MAN_CFM(0x44) /* 0x1444 */
#define WAVE600_UM_MAN_SET_DEFAULT_KEY_INDEX_REQ WAVE600_UMI_MSG_MAN_REQ(0x45) /* 0x0445 */
#define WAVE600_MC_MAN_SET_DEFAULT_KEY_INDEX_CFM WAVE600_UMI_MSG_MAN_CFM(0x45) /* 0x1445 */
#define WAVE600_UM_MAN_SET_802_1X_FILTER_REQ  WAVE600_UMI_MSG_MAN_REQ(0x46) /* 0x0446 */
#define WAVE600_MC_MAN_SET_802_1X_FILTER_CFM  WAVE600_UMI_MSG_MAN_CFM(0x46) /* 0x1446 */
#define WAVE600_UM_MAN_SET_KEY_REQ            WAVE600_UMI_MSG_MAN_REQ(0x05) /* 0x0405 */
#define WAVE600_MC_MAN_SET_KEY_CFM            WAVE600_UMI_MSG_MAN_CFM(0x05) /* 0x1405 */
#define WAVE600_UM_MAN_SET_BEACON_TEMPLATE_REQ  WAVE600_UMI_MSG_MAN_REQ(0x3D) /* 0x043D */
#define WAVE600_MC_MAN_SET_BEACON_TEMPLATE_CFM  WAVE600_UMI_MSG_MAN_CFM(0x3D) /* 0x143D */
#define WAVE600_UM_MAN_SET_REG_DOMAIN_CONFIG_REQ WAVE600_UMI_MSG_MAN_REQ(0x8B) /* 0x048B */
#define WAVE600_MC_MAN_SET_REG_DOMAIN_CONFIG_CFM WAVE600_UMI_MSG_MAN_CFM(0x8B) /* 0x148B */

/* Vendor recovery/default radio configuration messages */
#define WAVE600_UM_MAN_MU_OPERATION_CONFIG_REQ  WAVE600_UMI_MSG_MAN_REQ(0x67) /* 0x0467 */
#define WAVE600_MC_MAN_MU_OPERATION_CONFIG_CFM  WAVE600_UMI_MSG_MAN_CFM(0x67) /* 0x1467 */
#define WAVE600_UM_MAN_CCA_TH_REQ               WAVE600_UMI_MSG_MAN_REQ(0x68) /* 0x0468 */
#define WAVE600_MC_MAN_CCA_TH_CFM               WAVE600_UMI_MSG_MAN_CFM(0x68) /* 0x1468 */
#define WAVE600_UM_MAN_BF_MODE_CONFIG_REQ       WAVE600_UMI_MSG_MAN_REQ(0x6C) /* 0x046C */
#define WAVE600_MC_MAN_BF_MODE_CONFIG_CFM       WAVE600_UMI_MSG_MAN_CFM(0x6C) /* 0x146C */
#define WAVE600_UM_MAN_SET_TXOP_CONFIG_REQ      WAVE600_UMI_MSG_MAN_REQ(0x75) /* 0x0475 */
#define WAVE600_MC_MAN_SET_TXOP_CONFIG_CFM      WAVE600_UMI_MSG_MAN_CFM(0x75) /* 0x1475 */
#define WAVE600_UM_MAN_HE_MU_OPERATION_CONFIG_REQ WAVE600_UMI_MSG_MAN_REQ(0x7A) /* 0x047A */
#define WAVE600_MC_MAN_HE_MU_OPERATION_CONFIG_CFM WAVE600_UMI_MSG_MAN_CFM(0x7A) /* 0x147A */
#define WAVE600_UM_MAN_TS_VAP_CONFIGURE_REQ     WAVE600_UMI_MSG_MAN_REQ(0x65) /* 0x0465 */
#define WAVE600_MC_MAN_TS_VAP_CONFIGURE_CFM     WAVE600_UMI_MSG_MAN_CFM(0x65) /* 0x1465 */
#define WAVE600_UM_MAN_MSDU_IN_AMSDU_CONFIG_REQ WAVE600_UMI_MSG_MAN_REQ(0x66) /* 0x0466 */
#define WAVE600_MC_MAN_MSDU_IN_AMSDU_CONFIG_CFM WAVE600_UMI_MSG_MAN_CFM(0x66) /* 0x1466 */
#define WAVE600_UM_MAN_MAX_MPDU_LENGTH_REQ      WAVE600_UMI_MSG_MAN_REQ(0x6B) /* 0x046B */
#define WAVE600_MC_MAN_MAX_MPDU_LENGTH_CFM      WAVE600_UMI_MSG_MAN_CFM(0x6B) /* 0x146B */

/* Firmware-initiated events (UMI_MSG_MAN_IND(type, dir=IND, num)) */
#define WAVE600_MC_MAN_CLASS3_ERROR_IND          WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x04) /* 0x3304 */
#define WAVE600_MC_MAN_NETWORK_EVENT_IND         WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x26) /* 0x3326 */
#define WAVE600_MC_MAN_CONTINUOUS_INTERFERER_IND WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x2A) /* 0x332A */
#define WAVE600_MC_MAN_RADAR_IND                 WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x2B) /* 0x332B */
#define WAVE600_MC_MAN_MAC_EVENT_IND             WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x38) /* 0x3338 */
#define WAVE600_MC_MAN_BEACON_TEMPLATE_WAS_SET_IND WAVE600_UMI_MSG(WAVE600_MSG_IND, WAVE600_MSG_TYPE_MAN_IND, 0x3E) /* 0x333E */

/*
 * MAC_EVENT / APP_FATAL (vendor mhi_mac_event.h:163-174 and 236-256): sent
 * by firmware on MC_MAN_MAC_EVENT_IND (0x3338) when an assert or CPU exception occurs.
 * All 32-bit fields are MAC-endian (LE) inside the UMI payload.
 */
struct wave600_umi_mac_event {
	__le32 event_id;
	__le32 timestamp;
	__le32 core_num;
	__le32 fatal_timestamp;
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

/* UMI_TX_POWER_LIMIT (vendor mhi_umi.h:1652, message 0x045F) */
#define W600_API_SET_OPERATION	0
#define W600_API_GET_OPERATION	1
struct wave600_umi_tx_power_limit {
	u8 get_set_operation;	/* API_GET_OPERATION=1 / API_SET_OPERATION=0 */
	u8 power_limit_offset;
	u8 reserved[2];
} __packed;

/* UMI_CCA_TH_t (vendor mhi_umi.h:3219, message 0x0468) */
#define WAVE600_CCA_TH_PARAMS_LEN 5
struct wave600_umi_cca_th {
	s8 primary_ch_cca;
	s8 secondary_ch_cca;
	s8 mid_pkt_primary_cca;
	s8 mid_pkt_secondary_20_cca;
	s8 mid_pkt_secondary_40_cca;
	s8 reserved[3];
} __packed;

/* UMI_MU_OPERATION_CONFIG / UMI_HE_MU_OPERATION_CONFIG (vendor mhi_umi.h:2534) */
struct wave600_umi_mu_operation_config {
	u8 get_set_operation;
	u8 enable_mu_operation;
	u8 reserved[2];
} __packed;

struct wave600_umi_he_mu_operation_config {
	u8 enable_he_mu_operation;
	u8 get_set_operation;
	u8 reserved[2];
} __packed;

/* UMI_BF_MODE_CONFIG (vendor mhi_umi.h:2580) */
#define WAVE600_BF_MODE_EXPLICIT	0
#define WAVE600_BF_MODE_IMPLICIT	1
#define WAVE600_BF_MODE_STBC_1X2	2
#define WAVE600_BF_MODE_STBC_2X4	3
#define WAVE600_BF_MODE_NON_BF		4
#define WAVE600_BF_MODE_AUTO		0xFF

struct wave600_umi_bf_mode_config {
	u8 bf_mode;
	u8 reserved[3];
} __packed;

/* UMI_SET_TXOP_CONFIG_t (vendor mhi_umi.h:3311) */
#define WAVE600_TXOP_MODE_DISABLED	0
#define WAVE600_TXOP_MODE_FORCED	1
#define WAVE600_TXOP_MODE_DYNAMIC	2
#define WAVE600_TXOP_MODE_INVALID	0xFF
#define WAVE600_GEN6_ALL_STA_SID	0x1FF

struct wave600_umi_txop_config {
	__le32 sta_id;
	u8 mode;
	u8 get_set_operation;
	u8 reserved[2];
} __packed;

/* Vendor UMI_TS_VAP_CONFIGURE (0x0465) */
struct wave600_umi_ts_vap_configure {
	u8 get_set_op;
	u8 vap_id;
	u8 enable_ba;
	u8 amsdu_support;
	__le32 window_size;
} __packed;

/* Vendor UMI_MSDU_IN_AMSDU_CONFIG (0x0466) */
struct wave600_umi_msdu_in_amsdu_config {
	u8 get_set_op;
	u8 ht_msdu_in_amsdu;
	u8 vht_msdu_in_amsdu;
	u8 he_msdu_in_amsdu;
} __packed;

/* Vendor UMI_MAX_MPDU (0x046B) */
struct wave600_umi_max_mpdu {
	__le32 max_mpdu_length;
	u8 get_set_op;
	u8 reserved[3];
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

#define WAVE600_UMI_RSN_SEQ_NUM_LEN 8 /* 6 bytes seq num + 2 bytes pad */
#define WAVE600_UMI_RSN_TK_LEN      32 /* 16 bytes TK1 + 16 bytes TK2 */

/* RSN Key types (vendor umi_rsn.h) */
#define WAVE600_UMI_RSN_PAIRWISE_KEY    0
#define WAVE600_UMI_RSN_GROUP_KEY       1
#define WAVE600_UMI_RSN_MGMT_GROUP_KEY  2

/* RSN Cipher Suite selectors (vendor umi_rsn.h) */
#define WAVE600_UMI_CIPHER_SUITE_TKIP    0
#define WAVE600_UMI_CIPHER_SUITE_WEP40   1
#define WAVE600_UMI_CIPHER_SUITE_CCMP    2
#define WAVE600_UMI_CIPHER_SUITE_WAPI    3
#define WAVE600_UMI_CIPHER_SUITE_BIP     4
#define WAVE600_UMI_CIPHER_SUITE_WEP104  5
#define WAVE600_UMI_CIPHER_SUITE_GCMP128 6
#define WAVE600_UMI_CIPHER_SUITE_GCMP256 7
#define WAVE600_UMI_CIPHER_SUITE_NONE    8

#define WAVE600_BCAST_DEST_SID           0x3FFA

/* Vendor UMI_SET_KEY (0x0405) */
struct wave600_umi_set_key {
	__le16 status;
	__le16 sid;
	__le16 key_type;
	__le16 cipher_suite;
	__le16 key_index;
	u8 rx_seq_num[WAVE600_UMI_RSN_SEQ_NUM_LEN];
	u8 tx_seq_num[WAVE600_UMI_RSN_SEQ_NUM_LEN];
	u8 tk[WAVE600_UMI_RSN_TK_LEN];
} __packed;

/* Vendor UMI_DEFAULT_KEY_INDEX (0x0445) */
struct wave600_umi_default_key_index {
	__le16 status;
	__le16 sid;
	__le16 key_type;
	__le16 key_index;
} __packed;

/* Vendor UMI_REQUEST_SID (0x0442) */
struct wave600_umi_request_sid {
	u8 addr[ETH_ALEN];
	__le16 sid;
	u8 status;
	u8 reserved[3];
} __packed;

/* Vendor UMI_REMOVE_SID (0x0443) */
struct wave600_umi_remove_sid {
	__le16 sid;
	u8 status;
	u8 reserved;
} __packed;

/* Vendor UMI_FRAME_CLASS_ERROR (0x3304) */
struct wave600_umi_class3_error_entry {
	u8 vap_id;
	u8 reserved;
	u8 addr[ETH_ALEN];
} __packed;

struct wave600_umi_class3_error {
	u8 num_entries;
	struct wave600_umi_class3_error_entry entries[8];
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
 * Dynamic PHY Status (vendor hw_mmb.c hw_phy_rx_status_get, mhi_umi.h):
 * Firmware populates wholePhyRxStatusDb_t in DDR containing per-station
 * RSSI for all 4 antennas and 17-bit hardware PHY reception rate.
 */
#define WAVE600_UM_MAN_PHY_STATUS_REQ              WAVE600_UMI_MSG_MAN_REQ(0x0F) /* 0x040F */
#define WAVE600_MC_MAN_PHY_STATUS_CFM              WAVE600_UMI_MSG_MAN_CFM(0x0F) /* 0x140F */

struct wave600_umi_phy_status_req {
	u32 ddr_buffer_address;
	u32 ddr_buffer_size;
} __packed;

#define WAVE600_PHY_STATISTICS_MAX_RX_ANT 4
#define WAVE600_GEN6_NUM_OF_BANDS 2
#define WAVE600_GEN6_MAX_SID 256

struct wave600_station_phy_rx_status {
	s8 rssi[WAVE600_PHY_STATISTICS_MAX_RX_ANT];
	__le32 phy_rate;
	__le32 irad;
	__le32 last_tsf;
	__le32 per_client_rx_time_usage;
	s8 max_rssi;
	u8 reserved[3];
} __packed;

struct wave600_device_phy_rx_status {
	u8 noise[WAVE600_PHY_STATISTICS_MAX_RX_ANT];
	u8 rf_gain[WAVE600_PHY_STATISTICS_MAX_RX_ANT];
	__le32 irad;
	__le32 tsf;
	u8 channel_load;
	u8 total_channel_utilization;
	u8 rate_field_type;
	u8 reserved;
} __packed;

struct wave600_whole_phy_rx_status_db {
	struct wave600_device_phy_rx_status dev_status[WAVE600_GEN6_NUM_OF_BANDS];
	struct wave600_station_phy_rx_status sta_status[WAVE600_GEN6_MAX_SID];
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
 * Regulatory Domain Configuration (UM_MAN_SET_REG_DOMAIN_CONFIG_REQ)
 * Vendor mhi_umi.h: UMI_REG_DOMAIN_CONFIG
 */
#define WAVE600_REG_DOMAIN_FCC		0
#define WAVE600_REG_DOMAIN_ETSI		1
#define WAVE600_REG_DOMAIN_JAPAN	2
#define WAVE600_REG_DOMAIN_KOREA	3
#define WAVE600_REG_DOMAIN_CHINA	4
#define WAVE600_REG_DOMAIN_DEFAULT	0xFF

#define WAVE600_REGD_CODE_UNKNOWN	0x00
#define WAVE600_REGD_CODE_FCC		0x10
#define WAVE600_REGD_CODE_DOC		0x20
#define WAVE600_REGD_CODE_ETSI		0x30
#define WAVE600_REGD_CODE_SPAIN		0x31
#define WAVE600_REGD_CODE_FRANCE	0x32
#define WAVE600_REGD_CODE_UAE		0x33
#define WAVE600_REGD_CODE_GERMANY	0x34
#define WAVE600_REGD_CODE_MKK		0x40
#define WAVE600_REGD_CODE_ISRAEL	0x41
#define WAVE600_REGD_CODE_SINGAPORE	0x42
#define WAVE600_REGD_CODE_AUSTRALIA	0x44
#define WAVE600_REGD_CODE_BRAZIL	0x45
#define WAVE600_REGD_CODE_CHINA		0x46
#define WAVE600_REGD_CODE_KOREA		0x47
#define WAVE600_REGD_CODE_APAC		0x50
#define WAVE600_REGD_CODE_JAPAN		0x51

struct wave600_umi_reg_domain_config {
	u8 reg_domain_config;
	u8 get_set_operation; /* 0 = SET */
	u8 reserved[2];
} __packed;

/*
 * Beacon Template Configuration (UM_BSS_SET_BEACON_REQ)
 */
struct wave600_umi_beacon_set {
	u32 host_address;	/* DMA address of the template buffer */
	u16 part1_len;
	u16 part2_len;
	u16 part3_len;
	u16 part4_len;
	u16 part5_len;
	u8 vap_index;
	u8 status;
	u8 add_bss_load_ie;
	u8 reserved;
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
	u32 consecutive_timeouts;

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
int wave600_ipc_cmd_radio(struct wave600_dev *wdev, u8 radio_idx, u16 req_id, u16 cfm_id,
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
