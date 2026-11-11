// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) cfg80211 interface
 *
 * Dual-Band Concurrent (DBDC) AP implementation (Variant B):
 *  - Radio 0: 2.4 GHz band (b/g/n HT20), phy0, wlan0 (MAC 02:50:43:00:00:01), ep=1, vap_id=0
 *  - Radio 1: 5 GHz band (a/n/ac HT20/40/80/160), phy1, wlan1 (MAC 02:50:43:00:00:02), ep=0, vap_id=0
 *  - start_ap implementing the vendor sequence (core.c/beacon_manager.c):
 *      VAP_DB ADD (0x041E) -> MIB_IEEE_ADDRESS (0x0040) ->
 *      MIB_PRE_ACTIVATE (0x0109) -> MIB_SPECTRUM_MODE (0x1202) ->
 *      MBSS_PRE_ACTIVATE (0x0452) -> ADD_VAP (0x041F) ->
 *      ENABLE_RADIO (0x045C) -> SET_CHAN (0x0412) ->
 *      SET_TPC_ANT_PARAMS (0x044E) -> SET_TPC_CONFIG (0x044F) ->
 *      SET_ANTENNAS (0x0418) -> SET_BEACON_TEMPLATE (0x043D) ->
 *      SET_BSS (0x0421) -> SET_WMM_PARAMETERS (0x040A),
 *  - stop_ap via STOP_VAP_TRAFFIC + REMOVE_VAP.
 *  - add_station via REQUEST_SID (0x0442) + STA_ADD (0x043B).
 *  - del_station via STOP_TRAFFIC (0x0444) + STA_REMOVE (0x043C) + REMOVE_SID (0x0443).
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/pci.h>
#include <net/cfg80211.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_ipc.h"
#include "wave600_dma.h"
#include "wave600_cal.h"

#define W600_BEACON_INT		100
#define W600_DTIM		1

/* ------------------------------------------------------------------ */
/* UMI message payloads (mhi_umi.h layouts). Multi-byte request       */
/* fields are MAC-endian (little-endian), matching the vendor          */
/* HOST_TO_MAC32/16 conversions; FW-written SHRAM data is read raw BE.  */
/* ------------------------------------------------------------------ */

#define W600_MAX_RATES		32
#define W600_SSID_MAX_LEN	32

struct wave600_umi_add_vap {
	u8 bssid[ETH_ALEN];
	u8 vap_id;
	u8 operation_mode;	/* 0 = OPERATION_MODE_NORMAL */
	u8 rates[W600_MAX_RATES];
	u8 rates_len;
	u8 tx_mcs_bitmask[10];
	u8 vht_mcs_nss[8];
	u8 he_mcs_nss[8];
} __packed;

struct wave600_umi_set_chan {
	u8 low_chan_num;
	u8 low_chan_num2;
	u8 chan_width;		/* chanWidth_e: CW_20 = 0, CW_40 = 1, CW_80 = 2, CW_160 = 3 */
	u8 primary_chan_idx;
	u16 chan_switch_time;
	u8 switch_type;		/* ST_NORMAL = 0 */
	u8 block_tx_pre;
	u8 block_tx_post;
	u8 bg_scan;
	u8 result;
	u8 is_radar_detection_needed;
	u16 sid;
	u8 is_continuous_interferer_detection_needed;
	u8 reserved1;
	s8 rssi[4];
	u8 noise[4];
	u8 rf_gain[4];
	u32 rate;
	u8 antenna_mask;
	u8 channel_notification_mode;
	u8 regulation_type;
	u8 reserved3;
} __packed;

struct wave600_umi_set_bss {
	u16 beacon_interval;
	u16 dtim_interval;
	u8 vap_id;
	u8 protection_mode;	/* PROTECTION_MODE_NO_PROTECTION = 0 */
	u8 slot_time;		/* 1 = short slot (OFDM) */
	u8 rates[W600_MAX_RATES];
	u8 rates_len;
	u8 tx_mcs_bitmask[10];
	u8 vht_mcs_nss[8];
	u8 he_mcs_nss[8];
	u8 he_bss_color;
	u8 flags;
	u8 use_short_preamble;
	u8 reserved[3];
} __packed;

/* UMI_VAP_DB_OP: vendor sends this once per VAP at interface start */
struct wave600_umi_vap_db_op {
	u8 operation_code;	/* VAP_OPERATION_ADD = 1 */
	u8 vap_idx;
	u16 status;
} __packed;

/* UMI_MIB: vendor always sends payload_size = sizeof(UMI_MIB) with the
 * full MIB_VALUE union (largest member MIB_WEP_DEF_KEYS = 80 bytes). */
struct wave600_umi_mib {
	u16 object_id;
	u16 status;
	u8 value[80];
} __packed;

/* UMI_MBSS_PRE_ACTIVATE_HDR */
struct wave600_umi_mbss_pre_activate {
	u16 status;
	u8 coexistence_enabled;
	u8 intolerant_40mhz;
} __packed;

/* UMI_BEACON_INTERVAL_t */
struct wave600_umi_beacon_interval {
	u32 beacon_interval;
	u32 vap_id;
} __packed;

/* UMI_ENABLE_RADIO */
struct wave600_umi_enable_radio {
	u32 radio_on;
} __packed;

/* UMI tpcAntParams_t (mhi_mib_id.h): per-antenna frequency calibration curves */
struct wave600_umi_tpc_ant_params {
	u8 ant;
	u8 tpc_freq_len;
	u16 padding;
	u8 tpc_freq[8 * WAVE600_TPC_FREQ_SIZE];
} __packed;

/* UMI tpcConfig_t (mhi_mib_id.h): TX power control configuration.
 * MAXIMUM_BANDWIDTHS_GEN6 = 4. */
struct wave600_umi_tpc_config {
	s16 regulation_limit[4];
	s16 regulation_limit_mu[4];
	s16 regulation_limit_bf[4];
	s16 power_limit_11b;
	u8 cal_type;		/* 1 = TPC_CAL_TYPE_LOGARITHMIC */
	u8 tpc_loop_type;	/* 0 = closed loop, 1 = open loop */
	u8 fixed_gain;		/* 0xFF = NO_FIXED_GAIN */
	u8 get_set_operation;	/* 0 = API_SET_OPERATION */
} __packed;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* 2.4 GHz OFDM rates (6, 12, 24 basic rates - pure ERP/OFDM mode without 11b CCK) */
static const u8 wave600_rates_2g[] = {
	0x8c, 0x12, 0x98, 0x24,		/* 6 basic, 9, 12 basic, 18 */
	0xb0, 0x48, 0x60, 0x6c,		/* 24 basic, 36, 48, 54 */
};

/* 5 GHz OFDM rates (6, 12, 24 Mbps marked as basic rates with 0x80) */
static const u8 wave600_rates_5g[] = {
	0x8c, 0x12, 0x98, 0x24,		/* 6 basic, 9, 12 basic, 18 */
	0xb0, 0x48, 0x60, 0x6c,		/* 24 basic, 36, 48, 54 */
};

/* IE ids routed to the individual beacon template parts, exactly as the
 * vendor classifies them in beacon_manager.c _wave_get_beacon_data_length
 * (numeric values from driver/shared/frame.h ie_id_t). part1 = frame head. */
#define W600_IE_TIM			5
#define W600_IE_COUNTRY			7
#define W600_IE_FH_PATTRN_PARAMS	8
#define W600_IE_FH_PATTRM_TABLE	9
#define W600_IE_PWR_CONSTRAINT	32
#define W600_IE_TPC_REPORT		35
#define W600_IE_CHANNEL_SW_ANN	37
#define W600_IE_QUIET			40
#define W600_IE_IBSS_DFS		41
#define W600_IE_ERP_INFO		42
#define W600_IE_RSN			48
#define W600_IE_EXT_SUPP_RATES	50
#define W600_IE_BSS_LOAD		11
#define W600_IE_EDCA_PARAM_SET	2
#define W600_IE_HT_CAPABILITIES	45
#define W600_IE_QOS_CAPABILITY	46
#define W600_IE_AP_CHANNEL_REPORT	51
#define W600_IE_MOBILITY_DOMAIN	54
#define W600_IE_DSE_REGISTERED_LOCATION	58
#define W600_IE_SUPPORTED_OPER_CLASSES	59
#define W600_IE_EXT_CHANNEL_SW_ANN	60
#define W600_IE_HT_INFORMATION	61
#define W600_IE_BSS_AVERAGE_ACCESS	63
#define W600_IE_ANTENNA		64
#define W600_IE_MEASURE_PILOT_TRANSMISSION	66
#define W600_IE_BSS_AVAILABLE_ADMISSION_CAPACITY	67
#define W600_IE_BSS_AC_ACCESS_DELAY	68
#define W600_IE_TIME_ADV		69
#define W600_IE_RM_ENABLED_CAPABILITIES	70
#define W600_IE_MULTIPLE_BSSID	71
#define W600_IE_COEXISTENCE		72
#define W600_IE_OBSS_SCAN_PARAMETERS	74
#define W600_IE_FMS_DESCRIPTOR	86
#define W600_IE_QOS_TRAFFIC_CAPABILITY	89
#define W600_IE_INTERWORKING	107
#define W600_IE_ADV_PROTO		108
#define W600_IE_ROAMING_CONS	111
#define W600_IE_EMERGENCY_ALERT_ID	112
#define W600_IE_MESH_CONFIG	113
#define W600_IE_MESH_ID		114
#define W600_IE_MESH_CHANNEL_SW_PARAMETERS	118
#define W600_IE_MESH_AWAKE_WINDOW	119
#define W600_IE_BEACON_TIMING	120
#define W600_IE_MCCAOP_ADV		123
#define W600_IE_EXT_CAPABILITIES	127
#define W600_IE_MULTI_BAND		158
#define W600_IE_MCCAOP_ADV_OVERVIEW	174
#define W600_IE_QMF_POLICY		181
#define W600_IE_QLOAD_REPORT	186
#define W600_IE_HCCA_TXOP_UPDATE_COUNT	187
#define W600_IE_VHT_CAPABILITIES	191
#define W600_IE_VHT_OPERATION	192
#define W600_IE_TX_PWR_ENVELOPE	195

/* MIB object ids (mhi_mib_id.h) and pre-activation constants */
#define W600_UMI_OK				0
#define W600_MIB_PRE_ACTIVATE			0x0109
#define W600_MIB_SPECTRUM_MODE			0x1202
#define W600_MIB_SPECTRUM_20M			0
#define W600_MIB_SPECTRUM_40M			1
#define W600_MIB_SPECTRUM_80M			2
#define W600_MIB_SPECTRUM_160M			3
#define W600_MIB_IEEE_ADDRESS			0x0040
#define W600_NETWORK_11BGN_MIXED		5	/* NETWORK_MODES_E (2.4 GHz) */
#define W600_NETWORK_11AN_MIXED			8	/* NETWORK_MODES_E (5 GHz) */
#define W600_VAP_OPERATION_ADD			1
#define W600_VAP_OPERATION_DEL			2
#define W600_OPERATE_RATE_SET_11BGN		0xffffffffU
#define W600_BSS_BASIC_RATE_SET_11BGN_DEFAULT	0x00007800U
#define W600_OPERATE_RATE_SET_11AN		0xffff80ffU
#define W600_BSS_BASIC_RATE_SET_11AN		0x00000015U

/* SET_BSS / ADD_VAP capability flags (mhi_umi.h) */
#define W600_VAP_FLAG_HT			0x01
#define W600_VAP_FLAG_VHT			0x02
#define W600_VAP_FLAG_HE			0x04

/* STA_ADD capability flags (mhi_umi.h) */
#define W600_STA_FLAG_WME			0x02
#define W600_STA_FLAG_8021X_OPEN		0x04
#define W600_STA_FLAG_MFP			0x08
#define W600_STA_FLAG_HT			0x10
#define W600_STA_FLAG_VHT			0x20
#define W600_STA_FLAG_OMN			0x40
#define W600_STA_FLAG_EXT_HE			0x02

/* TPC (mhi_mib_id.h) */
#define W600_TPC_CAL_TYPE_LOGARITHMIC		1
#define W600_TPC_OPEN_LOOP			1
#define W600_TPC_NO_FIXED_GAIN			0xFF

/*
 * switch_type values (switchType enum, mhi_umi_propr.h):
 * ST_NORMAL = 0 — regular channel change (hot or cold)
 * ST_CSA = 2 — channel switch announcement
 */
#define W600_SWITCH_TYPE_NORMAL		0
#define W600_SWITCH_TYPE_SCAN		1
#define W600_SWITCH_TYPE_CSA		2

/* Forward declaration for wave600_set_country() */
static int wave600_send_tpc_config(struct wave600_radio *radio, u8 chan, bool is_5g);

/*
 * Regulatory domain country-to-code mapping table (vendor mtlk_psdb.c)
 */
struct wave600_country_to_domain {
	char country[3];
	u8 regd_code;
};

static const struct wave600_country_to_domain country_regdcode_table[] = {
	{"??", WAVE600_REGD_CODE_UNKNOWN},
	{"AD", WAVE600_REGD_CODE_ETSI},
	{"AE", WAVE600_REGD_CODE_UAE},
	{"AF", WAVE600_REGD_CODE_ETSI},
	{"AG", WAVE600_REGD_CODE_FCC},
	{"AI", WAVE600_REGD_CODE_FCC},
	{"AL", WAVE600_REGD_CODE_ETSI},
	{"AM", WAVE600_REGD_CODE_ETSI},
	{"AN", WAVE600_REGD_CODE_FCC},
	{"AO", WAVE600_REGD_CODE_ETSI},
	{"AQ", WAVE600_REGD_CODE_ETSI},
	{"AR", WAVE600_REGD_CODE_FCC},
	{"AS", WAVE600_REGD_CODE_ETSI},
	{"AT", WAVE600_REGD_CODE_ETSI},
	{"AU", WAVE600_REGD_CODE_AUSTRALIA},
	{"AW", WAVE600_REGD_CODE_FCC},
	{"AX", WAVE600_REGD_CODE_ETSI},
	{"AZ", WAVE600_REGD_CODE_ETSI},
	{"BA", WAVE600_REGD_CODE_ETSI},
	{"BB", WAVE600_REGD_CODE_FCC},
	{"BD", WAVE600_REGD_CODE_MKK},
	{"BE", WAVE600_REGD_CODE_ETSI},
	{"BF", WAVE600_REGD_CODE_MKK},
	{"BG", WAVE600_REGD_CODE_ETSI},
	{"BH", WAVE600_REGD_CODE_ETSI},
	{"BI", WAVE600_REGD_CODE_ETSI},
	{"BJ", WAVE600_REGD_CODE_ETSI},
	{"BL", WAVE600_REGD_CODE_FCC},
	{"BM", WAVE600_REGD_CODE_FCC},
	{"BN", WAVE600_REGD_CODE_MKK},
	{"BO", WAVE600_REGD_CODE_FCC},
	{"BR", WAVE600_REGD_CODE_BRAZIL},
	{"BS", WAVE600_REGD_CODE_FCC},
	{"BT", WAVE600_REGD_CODE_MKK},
	{"BV", WAVE600_REGD_CODE_MKK},
	{"BW", WAVE600_REGD_CODE_MKK},
	{"BY", WAVE600_REGD_CODE_ETSI},
	{"BZ", WAVE600_REGD_CODE_FCC},
	{"CA", WAVE600_REGD_CODE_DOC},
	{"CC", WAVE600_REGD_CODE_MKK},
	{"CD", WAVE600_REGD_CODE_MKK},
	{"CF", WAVE600_REGD_CODE_ETSI},
	{"CG", WAVE600_REGD_CODE_ETSI},
	{"CH", WAVE600_REGD_CODE_ETSI},
	{"CI", WAVE600_REGD_CODE_MKK},
	{"CK", WAVE600_REGD_CODE_MKK},
	{"CL", WAVE600_REGD_CODE_FCC},
	{"CM", WAVE600_REGD_CODE_ETSI},
	{"CN", WAVE600_REGD_CODE_CHINA},
	{"CO", WAVE600_REGD_CODE_FCC},
	{"CR", WAVE600_REGD_CODE_FCC},
	{"CU", WAVE600_REGD_CODE_FCC},
	{"CV", WAVE600_REGD_CODE_MKK},
	{"CX", WAVE600_REGD_CODE_MKK},
	{"CY", WAVE600_REGD_CODE_ETSI},
	{"CZ", WAVE600_REGD_CODE_ETSI},
	{"DE", WAVE600_REGD_CODE_ETSI},
	{"DJ", WAVE600_REGD_CODE_MKK},
	{"DK", WAVE600_REGD_CODE_ETSI},
	{"DM", WAVE600_REGD_CODE_FCC},
	{"DO", WAVE600_REGD_CODE_FCC},
	{"DZ", WAVE600_REGD_CODE_ETSI},
	{"EC", WAVE600_REGD_CODE_FCC},
	{"EE", WAVE600_REGD_CODE_ETSI},
	{"EG", WAVE600_REGD_CODE_ETSI},
	{"ER", WAVE600_REGD_CODE_ETSI},
	{"ES", WAVE600_REGD_CODE_SPAIN},
	{"ET", WAVE600_REGD_CODE_ETSI},
	{"FI", WAVE600_REGD_CODE_ETSI},
	{"FJ", WAVE600_REGD_CODE_MKK},
	{"FK", WAVE600_REGD_CODE_ETSI},
	{"FM", WAVE600_REGD_CODE_FCC},
	{"FO", WAVE600_REGD_CODE_ETSI},
	{"FR", WAVE600_REGD_CODE_FRANCE},
	{"GA", WAVE600_REGD_CODE_ETSI},
	{"GB", WAVE600_REGD_CODE_ETSI},
	{"GD", WAVE600_REGD_CODE_FCC},
	{"GE", WAVE600_REGD_CODE_ETSI},
	{"GF", WAVE600_REGD_CODE_ETSI},
	{"GG", WAVE600_REGD_CODE_ETSI},
	{"GH", WAVE600_REGD_CODE_MKK},
	{"GI", WAVE600_REGD_CODE_ETSI},
	{"GL", WAVE600_REGD_CODE_ETSI},
	{"GM", WAVE600_REGD_CODE_ETSI},
	{"GN", WAVE600_REGD_CODE_ETSI},
	{"GP", WAVE600_REGD_CODE_ETSI},
	{"GQ", WAVE600_REGD_CODE_ETSI},
	{"GR", WAVE600_REGD_CODE_ETSI},
	{"GS", WAVE600_REGD_CODE_ETSI},
	{"GT", WAVE600_REGD_CODE_FCC},
	{"GU", WAVE600_REGD_CODE_FCC},
	{"GW", WAVE600_REGD_CODE_ETSI},
	{"GY", WAVE600_REGD_CODE_FCC},
	{"HK", WAVE600_REGD_CODE_ETSI},
	{"HM", WAVE600_REGD_CODE_MKK},
	{"HN", WAVE600_REGD_CODE_FCC},
	{"HR", WAVE600_REGD_CODE_ETSI},
	{"HT", WAVE600_REGD_CODE_FCC},
	{"HU", WAVE600_REGD_CODE_ETSI},
	{"ID", WAVE600_REGD_CODE_MKK},
	{"IE", WAVE600_REGD_CODE_ETSI},
	{"IL", WAVE600_REGD_CODE_ISRAEL},
	{"IM", WAVE600_REGD_CODE_ETSI},
	{"IN", WAVE600_REGD_CODE_MKK},
	{"IQ", WAVE600_REGD_CODE_ETSI},
	{"IR", WAVE600_REGD_CODE_ETSI},
	{"IS", WAVE600_REGD_CODE_ETSI},
	{"IT", WAVE600_REGD_CODE_ETSI},
	{"JE", WAVE600_REGD_CODE_ETSI},
	{"JM", WAVE600_REGD_CODE_FCC},
	{"JO", WAVE600_REGD_CODE_ETSI},
	{"JP", WAVE600_REGD_CODE_JAPAN},
	{"KE", WAVE600_REGD_CODE_ETSI},
	{"KG", WAVE600_REGD_CODE_ETSI},
	{"KH", WAVE600_REGD_CODE_MKK},
	{"KI", WAVE600_REGD_CODE_MKK},
	{"KM", WAVE600_REGD_CODE_ETSI},
	{"KN", WAVE600_REGD_CODE_FCC},
	{"KP", WAVE600_REGD_CODE_MKK},
	{"KR", WAVE600_REGD_CODE_KOREA},
	{"KW", WAVE600_REGD_CODE_ETSI},
	{"KY", WAVE600_REGD_CODE_FCC},
	{"KZ", WAVE600_REGD_CODE_ETSI},
	{"LA", WAVE600_REGD_CODE_MKK},
	{"LB", WAVE600_REGD_CODE_ETSI},
	{"LC", WAVE600_REGD_CODE_FCC},
	{"LI", WAVE600_REGD_CODE_ETSI},
	{"LK", WAVE600_REGD_CODE_MKK},
	{"LR", WAVE600_REGD_CODE_ETSI},
	{"LS", WAVE600_REGD_CODE_ETSI},
	{"LT", WAVE600_REGD_CODE_ETSI},
	{"LU", WAVE600_REGD_CODE_ETSI},
	{"LV", WAVE600_REGD_CODE_ETSI},
	{"LY", WAVE600_REGD_CODE_ETSI},
	{"MA", WAVE600_REGD_CODE_ETSI},
	{"MC", WAVE600_REGD_CODE_ETSI},
	{"MD", WAVE600_REGD_CODE_ETSI},
	{"ME", WAVE600_REGD_CODE_ETSI},
	{"MF", WAVE600_REGD_CODE_FCC},
	{"MG", WAVE600_REGD_CODE_ETSI},
	{"MH", WAVE600_REGD_CODE_FCC},
	{"MK", WAVE600_REGD_CODE_ETSI},
	{"ML", WAVE600_REGD_CODE_ETSI},
	{"MM", WAVE600_REGD_CODE_MKK},
	{"MN", WAVE600_REGD_CODE_MKK},
	{"MO", WAVE600_REGD_CODE_MKK},
	{"MP", WAVE600_REGD_CODE_FCC},
	{"MQ", WAVE600_REGD_CODE_ETSI},
	{"MR", WAVE600_REGD_CODE_ETSI},
	{"MS", WAVE600_REGD_CODE_FCC},
	{"MT", WAVE600_REGD_CODE_ETSI},
	{"MU", WAVE600_REGD_CODE_ETSI},
	{"MV", WAVE600_REGD_CODE_MKK},
	{"MW", WAVE600_REGD_CODE_ETSI},
	{"MX", WAVE600_REGD_CODE_FCC},
	{"MY", WAVE600_REGD_CODE_MKK},
	{"MZ", WAVE600_REGD_CODE_ETSI},
	{"NA", WAVE600_REGD_CODE_ETSI},
	{"NC", WAVE600_REGD_CODE_ETSI},
	{"NE", WAVE600_REGD_CODE_ETSI},
	{"NF", WAVE600_REGD_CODE_MKK},
	{"NG", WAVE600_REGD_CODE_ETSI},
	{"NI", WAVE600_REGD_CODE_FCC},
	{"NL", WAVE600_REGD_CODE_ETSI},
	{"NO", WAVE600_REGD_CODE_ETSI},
	{"NP", WAVE600_REGD_CODE_MKK},
	{"NR", WAVE600_REGD_CODE_MKK},
	{"NU", WAVE600_REGD_CODE_MKK},
	{"NZ", WAVE600_REGD_CODE_AUSTRALIA},
	{"OM", WAVE600_REGD_CODE_ETSI},
	{"PA", WAVE600_REGD_CODE_FCC},
	{"PE", WAVE600_REGD_CODE_FCC},
	{"PF", WAVE600_REGD_CODE_ETSI},
	{"PG", WAVE600_REGD_CODE_MKK},
	{"PH", WAVE600_REGD_CODE_MKK},
	{"PK", WAVE600_REGD_CODE_ETSI},
	{"PL", WAVE600_REGD_CODE_ETSI},
	{"PM", WAVE600_REGD_CODE_ETSI},
	{"PR", WAVE600_REGD_CODE_FCC},
	{"PS", WAVE600_REGD_CODE_ETSI},
	{"PT", WAVE600_REGD_CODE_ETSI},
	{"PW", WAVE600_REGD_CODE_FCC},
	{"PY", WAVE600_REGD_CODE_FCC},
	{"QA", WAVE600_REGD_CODE_ETSI},
	{"RE", WAVE600_REGD_CODE_ETSI},
	{"RO", WAVE600_REGD_CODE_ETSI},
	{"RS", WAVE600_REGD_CODE_ETSI},
	{"RU", WAVE600_REGD_CODE_ETSI},
	{"RW", WAVE600_REGD_CODE_ETSI},
	{"SA", WAVE600_REGD_CODE_ETSI},
	{"SB", WAVE600_REGD_CODE_MKK},
	{"SC", WAVE600_REGD_CODE_ETSI},
	{"SE", WAVE600_REGD_CODE_ETSI},
	{"SG", WAVE600_REGD_CODE_SINGAPORE},
	{"SH", WAVE600_REGD_CODE_ETSI},
	{"SI", WAVE600_REGD_CODE_ETSI},
	{"SJ", WAVE600_REGD_CODE_ETSI},
	{"SK", WAVE600_REGD_CODE_ETSI},
	{"SL", WAVE600_REGD_CODE_ETSI},
	{"SM", WAVE600_REGD_CODE_ETSI},
	{"SN", WAVE600_REGD_CODE_ETSI},
	{"SO", WAVE600_REGD_CODE_ETSI},
	{"SR", WAVE600_REGD_CODE_FCC},
	{"ST", WAVE600_REGD_CODE_ETSI},
	{"SV", WAVE600_REGD_CODE_FCC},
	{"SY", WAVE600_REGD_CODE_ETSI},
	{"SZ", WAVE600_REGD_CODE_ETSI},
	{"TC", WAVE600_REGD_CODE_FCC},
	{"TD", WAVE600_REGD_CODE_ETSI},
	{"TF", WAVE600_REGD_CODE_MKK},
	{"TG", WAVE600_REGD_CODE_ETSI},
	{"TH", WAVE600_REGD_CODE_MKK},
	{"TJ", WAVE600_REGD_CODE_ETSI},
	{"TK", WAVE600_REGD_CODE_MKK},
	{"TL", WAVE600_REGD_CODE_MKK},
	{"TM", WAVE600_REGD_CODE_ETSI},
	{"TN", WAVE600_REGD_CODE_ETSI},
	{"TO", WAVE600_REGD_CODE_ETSI},
	{"TR", WAVE600_REGD_CODE_ETSI},
	{"TT", WAVE600_REGD_CODE_ETSI},
	{"TV", WAVE600_REGD_CODE_MKK},
	{"TW", WAVE600_REGD_CODE_MKK},
	{"TZ", WAVE600_REGD_CODE_ETSI},
	{"UA", WAVE600_REGD_CODE_ETSI},
	{"UG", WAVE600_REGD_CODE_FCC},
	{"UM", WAVE600_REGD_CODE_FCC},
	{"US", WAVE600_REGD_CODE_FCC},
	{"UY", WAVE600_REGD_CODE_FCC},
	{"UZ", WAVE600_REGD_CODE_ETSI},
	{"VA", WAVE600_REGD_CODE_ETSI},
	{"VC", WAVE600_REGD_CODE_FCC},
	{"VE", WAVE600_REGD_CODE_FCC},
	{"VG", WAVE600_REGD_CODE_FCC},
	{"VI", WAVE600_REGD_CODE_FCC},
	{"VN", WAVE600_REGD_CODE_MKK},
	{"VU", WAVE600_REGD_CODE_MKK},
	{"WF", WAVE600_REGD_CODE_ETSI},
	{"WS", WAVE600_REGD_CODE_MKK},
	{"YE", WAVE600_REGD_CODE_ETSI},
	{"YT", WAVE600_REGD_CODE_ETSI},
	{"ZA", WAVE600_REGD_CODE_ETSI},
	{"ZM", WAVE600_REGD_CODE_ETSI},
	{"ZW", WAVE600_REGD_CODE_ETSI},
};

static u8 wave600_country_to_regd_code(const char *country)
{
	int i;

	if (!country || !country[0])
		return WAVE600_REGD_CODE_ETSI;

	for (i = 0; i < ARRAY_SIZE(country_regdcode_table); i++) {
		if (country[0] == country_regdcode_table[i].country[0] &&
		    country[1] == country_regdcode_table[i].country[1])
			return country_regdcode_table[i].regd_code;
	}

	return WAVE600_REGD_CODE_ETSI;
}

static u8 wave600_regd_code_to_region(u8 regd_code)
{
	switch (regd_code) {
	case WAVE600_REGD_CODE_FCC:
	case WAVE600_REGD_CODE_DOC:
		return WAVE600_REG_DOMAIN_FCC;

	case WAVE600_REGD_CODE_ETSI:
	case WAVE600_REGD_CODE_SPAIN:
	case WAVE600_REGD_CODE_FRANCE:
	case WAVE600_REGD_CODE_UAE:
	case WAVE600_REGD_CODE_GERMANY:
		return WAVE600_REG_DOMAIN_ETSI;

	case WAVE600_REGD_CODE_MKK:
	case WAVE600_REGD_CODE_JAPAN:
	case WAVE600_REGD_CODE_ISRAEL:
	case WAVE600_REGD_CODE_SINGAPORE:
	case WAVE600_REGD_CODE_AUSTRALIA:
		return WAVE600_REG_DOMAIN_JAPAN;

	case WAVE600_REGD_CODE_CHINA:
		return WAVE600_REG_DOMAIN_CHINA;

	case WAVE600_REGD_CODE_KOREA:
		return WAVE600_REG_DOMAIN_KOREA;

	default:
		return WAVE600_REG_DOMAIN_ETSI;
	}
}

int wave600_send_reg_domain_config(struct wave600_dev *wdev, u8 region)
{
	struct wave600_umi_reg_domain_config p;
	int ret;

	memset(&p, 0, sizeof(p));
	p.reg_domain_config = region;
	p.get_set_operation = 0; /* SET */

	ret = wave600_ipc_cmd(wdev,
			      WAVE600_UM_MAN_SET_REG_DOMAIN_CONFIG_REQ,
			      WAVE600_MC_MAN_SET_REG_DOMAIN_CONFIG_CFM,
			      &p, sizeof(p), NULL, 0, 5000);
	if (ret) {
		dev_warn(wdev->dev, "SET_REG_DOMAIN_CONFIG (region=%u) failed: %d\n", region, ret);
		return ret;
	}

	dev_info(wdev->dev, "SET_REG_DOMAIN_CONFIG confirmed: region=%u (%s)\n",
		 region, (region == WAVE600_REG_DOMAIN_ETSI) ? "ETSI" :
			 (region == WAVE600_REG_DOMAIN_FCC) ? "FCC" :
			 (region == WAVE600_REG_DOMAIN_JAPAN) ? "JAPAN" :
			 (region == WAVE600_REG_DOMAIN_CHINA) ? "CHINA" :
			 (region == WAVE600_REG_DOMAIN_KOREA) ? "KOREA" : "OTHER");
	return 0;
}

/*
 * Vendor _mtlk_core_cfg_init_pw_lim_offset (core.c:3391): on master VAP init
 * the host reads the FW powerLimitOffset via UM_MAN_SET_POWER_LIMIT_REQ
 * (0x045F) with API_GET_OPERATION. A non-zero offset shifts the whole TX
 * power table down; log it so a silent clamp is visible.
 */
static int wave600_get_power_limit_offset(struct wave600_dev *wdev)
{
	struct wave600_umi_tx_power_limit req, cfm;
	int ret;

	memset(&req, 0, sizeof(req));
	req.get_set_operation = W600_API_GET_OPERATION;

	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_SET_POWER_LIMIT_REQ,
			      WAVE600_MC_MAN_SET_POWER_LIMIT_CFM,
			      &req, sizeof(req), &cfm, sizeof(cfm), 5000);
	if (ret) {
		dev_warn(wdev->dev, "GET_POWER_LIMIT (0x045F) failed: %d\n", ret);
		return ret;
	}

	dev_info(wdev->dev, "FW powerLimitOffset (0x045F GET) = %u (0x%02x)%s\n",
		 cfm.power_limit_offset, cfm.power_limit_offset,
		 cfm.power_limit_offset ? "  <-- NON-ZERO: TX power table shifted down!" : "");
	return 0;
}

int wave600_set_country(struct wave600_dev *wdev, const char *alpha2)
{
	u8 regd_code, region;
	int r;

	if (!alpha2 || !alpha2[0])
		return -EINVAL;

	wdev->country[0] = alpha2[0];
	wdev->country[1] = alpha2[1];
	wdev->country[2] = '\0';

	regd_code = wave600_country_to_regd_code(wdev->country);
	region = wave600_regd_code_to_region(regd_code);

	wdev->regd_code = regd_code;
	wdev->reg_domain_region = region;

	dev_info(wdev->dev, "userspace regulatory domain: country '%s' -> regd_code 0x%02x, region %u\n",
		 wdev->country, regd_code, region);

	wave600_get_power_limit_offset(wdev);

	wave600_send_reg_domain_config(wdev, region);

	/* If any AP is running, refresh TPC with the new country limits */
	for (r = 0; r < 2; r++) {
		struct wave600_radio *radio = &wdev->radios[r];
		if (radio->ap_started)
			wave600_send_tpc_config(radio, (u8)radio->ap_channel, (r == 1));
	}

	return 0;
}

static void wave600_reg_notifier(struct wiphy *wiphy, struct regulatory_request *request)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	char alpha2[3];

	alpha2[0] = request->alpha2[0];
	alpha2[1] = request->alpha2[1];
	alpha2[2] = '\0';

	dev_info(wdev->dev, "Radio %u reg_notifier: userspace requested country '%s' (initiator=%d)\n",
		 radio->radio_idx, alpha2, request->initiator);

	wave600_set_country(wdev, alpha2);
}

/*
 * Vendor radio_context_init/radio_pdb_def defaults: CCA thresholds come from
 * the host-only PSDB 0x330000 table; if the table is absent, use the vendor
 * initial PDB values. The firmware message is UM_MAN_CCA_TH_REQ (0x0468).
 */
static int wave600_send_cca_threshold(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	static const s8 wave600_default_cca_th[WAVE600_CCA_TH_PARAMS_LEN] = {
		-62, -62, -72, -72, -69
	};
	struct wave600_umi_cca_th p;
	int i, ret;

	memset(&p, 0, sizeof(p));
	for (i = 0; i < WAVE600_CCA_TH_PARAMS_LEN; i++) {
		s8 value;

		if (wdev->psdb.parsed && wdev->psdb.cca_valid[radio->radio_idx])
			value = wdev->psdb.cca_th[radio->radio_idx][i];
		else
			value = wave600_default_cca_th[i];

		switch (i) {
		case 0:
			p.primary_ch_cca = value;
			break;
		case 1:
			p.secondary_ch_cca = value;
			break;
		case 2:
			p.mid_pkt_primary_cca = value;
			break;
		case 3:
			p.mid_pkt_secondary_20_cca = value;
			break;
		default:
			p.mid_pkt_secondary_40_cca = value;
			break;
		}
	}

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_CCA_TH_REQ,
				    WAVE600_MC_MAN_CCA_TH_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u CCA_TH (0x0468) failed: %d\n",
			 radio->radio_idx, ret);
		return ret;
	}

	dev_info(wdev->dev,
		 "Radio %u CCA_TH (0x0468): %d %d %d %d %d\n",
		 radio->radio_idx,
		 p.primary_ch_cca, p.secondary_ch_cca,
		 p.mid_pkt_primary_cca, p.mid_pkt_secondary_20_cca,
		 p.mid_pkt_secondary_40_cca);
	return 0;
}

/* Vendor default PDB: BF mode is explicit (0), sent with 0x046C. */
static int wave600_send_bf_mode(struct wave600_radio *radio, u8 bf_mode)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_bf_mode_config p;
	int ret;

	memset(&p, 0, sizeof(p));
	p.bf_mode = bf_mode;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_BF_MODE_CONFIG_REQ,
				    WAVE600_MC_MAN_BF_MODE_CONFIG_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u BF_MODE (0x046C) mode=%u failed: %d\n",
			 radio->radio_idx, bf_mode, ret);
		return ret;
	}

	dev_info(wdev->dev, "Radio %u BF_MODE (0x046C): mode=%u\n",
		 radio->radio_idx, bf_mode);
	return 0;
}

/* Vendor default PDB: HE MU operation is enabled (1), message 0x047A. */
static int wave600_send_he_mu_operation(struct wave600_radio *radio,
					u8 enable_he_mu)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_he_mu_operation_config p;
	int ret;

	memset(&p, 0, sizeof(p));
	p.enable_he_mu_operation = enable_he_mu;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_HE_MU_OPERATION_CONFIG_REQ,
				    WAVE600_MC_MAN_HE_MU_OPERATION_CONFIG_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u HE_MU_OPERATION (0x047A) enable=%u failed: %d\n",
			 radio->radio_idx, enable_he_mu, ret);
		return ret;
	}

	dev_info(wdev->dev, "Radio %u HE_MU_OPERATION (0x047A): enable=%u\n",
		 radio->radio_idx, enable_he_mu);
	return 0;
}

/* Vendor PDB keeps TXOP mode invalid by default; if set, send 0x0475. */
static int wave600_send_txop_mode(struct wave600_radio *radio, u32 sid, u8 mode)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_txop_config p;
	int ret;

	if (mode == WAVE600_TXOP_MODE_INVALID)
		return 0;

	memset(&p, 0, sizeof(p));
	p.sta_id = cpu_to_le32(sid);
	p.mode = mode;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_TXOP_CONFIG_REQ,
				    WAVE600_MC_MAN_SET_TXOP_CONFIG_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u TXOP (0x0475) sid=%u mode=%u failed: %d\n",
			 radio->radio_idx, sid, mode, ret);
		return ret;
	}

	dev_info(wdev->dev, "Radio %u TXOP (0x0475): sid=%u mode=%u\n",
		 radio->radio_idx, sid, mode);
	return 0;
}

/*
 * Vendor sAggrConfig -> UMI_TS_VAP_CONFIGURE (0x0465):
 * Enables Block Ack and A-MSDU in firmware ARC for the VAP.
 * Window size 64 matches standard 802.11n/ac/ax BA agreement.
 */
static int wave600_send_ts_vap_configure(struct wave600_radio *radio,
					 u8 enable_ba, u8 amsdu_support,
					 u32 window_size)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_ts_vap_configure p;
	int ret;

	memset(&p, 0, sizeof(p));
	p.get_set_op = W600_API_SET_OPERATION;
	p.vap_id = radio->vap_id;
	p.enable_ba = enable_ba;
	p.amsdu_support = amsdu_support;
	p.window_size = cpu_to_le32(window_size);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_TS_VAP_CONFIGURE_REQ,
				    WAVE600_MC_MAN_TS_VAP_CONFIGURE_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u TS_VAP_CONFIGURE (0x0465) failed: %d\n",
			 radio->radio_idx, ret);
		return ret;
	}

	dev_info(wdev->dev, "Radio %u TS_VAP_CONFIGURE (0x0465): BA=%u AMSDU=%u win=%u\n",
		 radio->radio_idx, enable_ba, amsdu_support, window_size);
	return 0;
}


/*
 * Vendor _mtlk_core_send_tx_power_limit_offset (core.c:6304) is per-core.
 * The live nl80211 TX power path and the bring-up default offset therefore
 * both use the radio-specific endpoint.
 */
static int wave600_send_power_limit_offset_radio(struct wave600_radio *radio,
						 u8 offset_dbm)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_tx_power_limit p;
	int ret;

	memset(&p, 0, sizeof(p));
	p.get_set_operation = W600_API_SET_OPERATION;
	p.power_limit_offset = offset_dbm;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_POWER_LIMIT_REQ,
				    WAVE600_MC_MAN_SET_POWER_LIMIT_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u SET_POWER_LIMIT (0x045F) offset=%u dBm failed: %d\n",
			 radio->radio_idx, offset_dbm, ret);
		return ret;
	}

	dev_info(wdev->dev, "Radio %u SET_POWER_LIMIT (0x045F): offset=%u dBm\n",
		 radio->radio_idx, offset_dbm);
	return 0;
}

static int wave600_send_set_chan(struct wave600_radio *radio, u8 chan, u8 width,
				 u8 primary_idx, u8 switch_type,
				 u16 chan_switch_time)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_set_chan p;
	struct wave600_umi_set_chan cfm;
	int ret;
	bool is_5g = (radio->radio_idx == 1);
	bool is_dfs_chan = false;

	memset(&p, 0, sizeof(p));
	p.low_chan_num = chan;
	p.chan_width = width;	/* chanWidth_e */
	p.primary_chan_idx = primary_idx;
	p.switch_type = switch_type;
	p.chan_switch_time = cpu_to_le16(chan_switch_time);

	/*
	 * Regulatory domain: dynamic from userspace (OpenWrt / hostapd / cfg80211).
	 * Falls back to 0x30 (ETSI / Poland) if userspace has not set country yet.
	 */
	p.regulation_type = wdev->regd_code ? wdev->regd_code : WAVE600_REGD_CODE_ETSI;

	/*
	 * isContinuousInterfererDetectionNeeded: vendor scan_support.h
	 * is_interf_det_needed() returns 1 for all channels.
	 */
	p.is_continuous_interferer_detection_needed = 1;

	/*
	 * isRadarDetectionNeeded: true during CAC scan (ST_SCAN) or when any
	 * 20 MHz segment in the bonded channel bandwidth requires DFS radar
	 * detection according to the IEEE80211_CHAN_RADAR flag.
	 */
	if (switch_type == W600_SWITCH_TYPE_SCAN) {
		is_dfs_chan = true;
	} else if (is_5g && radio->wiphy && radio->wiphy->bands[NL80211_BAND_5GHZ]) {
		struct ieee80211_supported_band *b = radio->wiphy->bands[NL80211_BAND_5GHZ];
		int num_subchans = 1 << width; /* CW_20=1, CW_40=2, CW_80=4, CW_160=8 */
		int i, c;

		for (i = 0; i < num_subchans; i++) {
			u8 subchan = chan + (i * 4);

			for (c = 0; c < b->n_channels; c++) {
				if (b->channels[c].hw_value == subchan) {
					if (b->channels[c].flags & IEEE80211_CHAN_RADAR)
						is_dfs_chan = true;
					break;
				}
			}
			if (is_dfs_chan)
				break;
		}
	}
	p.is_radar_detection_needed = is_dfs_chan ? 1 : 0;

	/*
	 * channel_notification_mode: only set for DFS channels
	 * (matching sb_dfs.sb_dfs_bw from vendor core_cfg_set_chan).
	 * CHANNEL_NOTIFICATION_NORMAL = 0 means full-band notification.
	 */
	if (is_dfs_chan)
		p.channel_notification_mode = 0; /* CHANNEL_NOTIFICATION_NORMAL */

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_SET_CHAN_REQ,
				    WAVE600_MC_SET_CHAN_CFM,
				    &p, sizeof(p), &cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	/* Vendor checks req->result after send (setChanReport, 0 = OK) */
	if (cfm.result != W600_UMI_OK) {
		dev_err(wdev->dev, "Radio %u SET_CHAN: FW rejected (result=%u)\n",
			radio->radio_idx, cfm.result);
		return -EIO;
	}

	/* Antenna mask returned by FW in SET_CHAN CFM (byte 12) */
	if (cfm.antenna_mask)
		radio->antenna_mask = cfm.antenna_mask;
	else
		radio->antenna_mask = (radio->radio_idx == 1) ? 0x0A : 0x05;

	dev_info(wdev->dev, "Radio %u SET_CHAN: FW antenna mask 0x%02x -> local 0x%02x\n",
		 radio->radio_idx, cfm.antenna_mask, radio->antenna_mask);

	/* Vendor core_cfg_set_chan_clpb: send the current CCA threshold after SET_CHAN. */
	wave600_send_cca_threshold(radio);

	return 0;
}

int wave600_radio_find_sid(struct wave600_radio *radio, const u8 *mac)
{
	int i;

	for (i = 0; i < WAVE600_MAX_STATIONS; i++) {
		if (radio->stations[i].valid && ether_addr_equal(radio->stations[i].mac, mac))
			return (int)radio->stations[i].sid;
	}
	return -ENOENT;
}

struct wave600_station *wave600_find_sta(struct wave600_radio *radio, const u8 *mac)
{
	int i;

	for (i = 0; i < WAVE600_MAX_STATIONS; i++) {
		if (radio->stations[i].valid && ether_addr_equal(radio->stations[i].mac, mac))
			return &radio->stations[i];
	}
	return NULL;
}

static struct wave600_station *wave600_add_sta_entry(struct wave600_radio *radio,
						     const u8 *mac, u16 sid)
{
	struct wave600_station *sta = wave600_find_sta(radio, mac);
	int i;

	if (sta) {
		sta->sid = sid;
		sta->connect_time = ktime_get_seconds();
		sta->last_rx = jiffies;
		return sta;
	}

	for (i = 0; i < WAVE600_MAX_STATIONS; i++) {
		if (!radio->stations[i].valid) {
			sta = &radio->stations[i];
			memset(sta, 0, sizeof(*sta));
			sta->valid = true;
			ether_addr_copy(sta->mac, mac);
			sta->sid = sid;
			sta->connect_time = ktime_get_seconds();
			sta->last_rx = jiffies;
			if (radio->sta_signal < 0) {
				sta->signal = radio->sta_signal;
				sta->signal_avg = radio->sta_signal;
			}
			sta->is_authorized = false;
			radio->num_stations++;
			return sta;
		}
	}
	return NULL;
}

static void wave600_del_sta_entry(struct wave600_radio *radio, const u8 *mac)
{
	struct wave600_station *sta = wave600_find_sta(radio, mac);

	if (sta) {
		memset(sta, 0, sizeof(*sta));
		if (radio->num_stations > 0)
			radio->num_stations--;
	}
}

static void wave600_teardown_station(struct wave600_radio *radio, struct wave600_station *sta)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_stop_traffic sreq, scfm;
	struct wave600_umi_sta_remove req, cfm;
	struct wave600_umi_remove_sid rreq, rcfm;
	u8 del_mac[ETH_ALEN];
	u16 sid;
	int ret;

	if (!sta || !sta->valid)
		return;

	sid = sta->sid;
	ether_addr_copy(del_mac, sta->mac);

	dev_dbg(wdev->dev, "Radio %u teardown station %pM (sid %u)\n",
		radio->radio_idx, del_mac, sid);

	/* 1. Stop traffic queues before removing station (vendor core_ap_stop_traffic) */
	memset(&sreq, 0, sizeof(sreq));
	sreq.sid = cpu_to_le16(sid);
	sreq.status = 0;
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_STOP_TRAFFIC_REQ,
				    WAVE600_MC_MAN_STOP_TRAFFIC_CFM,
				    &sreq, sizeof(sreq), &scfm, sizeof(scfm), 2000,
				    radio->vap_id);
	if (ret)
		dev_warn(wdev->dev, "Radio %u STOP_TRAFFIC IPC failed: %d\n", radio->radio_idx, ret);
	else if (scfm.status)
		dev_warn(wdev->dev, "Radio %u STOP_TRAFFIC rejected by FW: status=%u\n",
			 radio->radio_idx, scfm.status);

	/* Allow Lower MAC ARC to finish draining and confirming queues */
	usleep_range(2000, 4000);

	/* 2. Remove station from Upper MAC ARC firmware (vendor core_ap_remove_sta) */
	memset(&req, 0, sizeof(req));
	req.sid = cpu_to_le16(sid);
	req.status = 0;
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_STA_REMOVE_REQ,
				    WAVE600_MC_MAN_STA_REMOVE_CFM,
				    &req, sizeof(req), &cfm, sizeof(cfm), 2000,
				    radio->vap_id);
	if (ret)
		dev_warn(wdev->dev, "Radio %u STA_REMOVE IPC failed: %d\n", radio->radio_idx, ret);
	else if (cfm.status)
		dev_warn(wdev->dev, "Radio %u STA_REMOVE rejected by FW: status=%u\n",
			 radio->radio_idx, cfm.status);

	/* Allow Upper MAC ARC to finalize STA close confirmation before releasing hardware SID */
	usleep_range(2000, 4000);

	/* 3. Release SID in ARC firmware hardware table (vendor core.c:14879 UM_MAN_REMOVE_SID_REQ) */
	memset(&rreq, 0, sizeof(rreq));
	rreq.sid = cpu_to_le16(sid);
	rreq.status = 0;
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_REMOVE_SID_REQ,
				    WAVE600_MC_MAN_REMOVE_SID_CFM,
				    &rreq, sizeof(rreq), &rcfm, sizeof(rcfm), 2000,
				    radio->vap_id);
	if (ret)
		dev_warn(wdev->dev, "Radio %u REMOVE_SID IPC failed: %d\n", radio->radio_idx, ret);
	else if (rcfm.status)
		dev_warn(wdev->dev, "Radio %u REMOVE_SID rejected by FW: status=%u\n",
			 radio->radio_idx, rcfm.status);

	/* 4. Delete entry from local driver table */
	wave600_del_sta_entry(radio, del_mac);
}

static int wave600_send_set_bss(struct wave600_radio *radio,
				struct cfg80211_ap_settings *settings,
				bool is_5g)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_set_bss p;
	const u8 *rates = is_5g ? wave600_rates_5g : wave600_rates_2g;
	u8 rates_len = is_5g ? sizeof(wave600_rates_5g) : sizeof(wave600_rates_2g);
	u16 beacon_int = (settings && settings->beacon_interval) ? settings->beacon_interval : 100;
	u16 dtim_int = (settings && settings->dtim_period) ? settings->dtim_period : 2;

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian on the wire (vendor
	 * HOST_TO_MAC16/32 = cpu_to_le16/32, mtlkinc.h; the MAN transport
	 * copies the payload verbatim). */
	p.beacon_interval = cpu_to_le16(beacon_int);
	p.dtim_interval = cpu_to_le16(dtim_int);
	p.vap_id = radio->vap_id;
	p.slot_time = 1;	/* short slot for OFDM */
	p.use_short_preamble = 1;
	memcpy(p.rates, rates, rates_len);
	p.rates_len = rates_len;

	/* Supported HT MCS: 2x2 MIMO MCS 0-15 */
	p.tx_mcs_bitmask[0] = 0xff;
	p.tx_mcs_bitmask[1] = 0xff;

	/* Capabilities flag: VAP_ADD_FLAGS_HT (0x01) */
	p.flags = W600_VAP_FLAG_HT;

	/* VHT MCS: 2x2 MIMO MCS 0-9 on 5 GHz (0xfffa: SS1=2, SS2=2, rest=3) */
	if (is_5g) {
		p.flags |= W600_VAP_FLAG_VHT;
		p.vht_mcs_nss[0] = 0xfa;	/* Rx MCS 0-9 for SS1-SS2 */
		p.vht_mcs_nss[1] = 0xff;	/* Rx SS3-SS8 unsupported */
		p.vht_mcs_nss[2] = 0x00;	/* Rx highest supported data rate */
		p.vht_mcs_nss[3] = 0x00;
		p.vht_mcs_nss[4] = 0xfa;	/* Tx MCS 0-9 for SS1-SS2 */
		p.vht_mcs_nss[5] = 0xff;	/* Tx SS3-SS8 unsupported */
		p.vht_mcs_nss[6] = 0x00;	/* Tx highest supported data rate */
		p.vht_mcs_nss[7] = 0x00;
	} else {
		memset(p.vht_mcs_nss, 0xff, sizeof(p.vht_mcs_nss));
	}

	/* HE MCS: 2x2 MIMO MCS 0-11 (0xfffa: SS1=2, SS2=2, rest=3)
	 * Vendor 0621-80211AX_D30.patch:609-616 & core.c:2474:
	 *  - Rx & Tx <=80 MHz: 0xfa, 0xff on both bands
	 *  - Rx & Tx 160 MHz: 0xfa, 0xff on 5 GHz; 0xff on 2.4 GHz
	 */
	p.flags |= W600_VAP_FLAG_HE;
	p.he_mcs_nss[0] = 0xfa;		/* Rx HE-MCS Map <= 80 MHz (SS1=2, SS2=2) */
	p.he_mcs_nss[1] = 0xff;		/* Rx SS3-SS8 unsupported */
	p.he_mcs_nss[2] = 0xfa;		/* Tx HE-MCS Map <= 80 MHz (SS1=2, SS2=2) */
	p.he_mcs_nss[3] = 0xff;		/* Tx SS3-SS8 unsupported */
	if (is_5g) {
		p.he_mcs_nss[4] = 0xfa;	/* Rx HE-MCS Map 160 MHz (SS1=2, SS2=2) */
		p.he_mcs_nss[5] = 0xff;	/* Rx SS3-SS8 unsupported */
		p.he_mcs_nss[6] = 0xfa;	/* Tx HE-MCS Map 160 MHz (SS1=2, SS2=2) */
		p.he_mcs_nss[7] = 0xff;	/* Tx SS3-SS8 unsupported */
	} else {
		memset(&p.he_mcs_nss[4], 0xff, 4);
	}

	if (settings && settings->he_oper) {
		u32 oper_params = le32_to_cpu(settings->he_oper->he_oper_params);
		p.he_bss_color = (u8)((oper_params & IEEE80211_HE_OPERATION_BSS_COLOR_MASK) >>
				      IEEE80211_HE_OPERATION_BSS_COLOR_OFFSET);
	} else {
		p.he_bss_color = 1;
	}

	return wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				     WAVE600_UM_BSS_SET_BSS_REQ,
				     WAVE600_MC_BSS_SET_BSS_CFM,
				     &p, sizeof(p), NULL, 0, 5000,
				     radio->vap_id);
}

/* Initial basic rates for ADD_VAP (vendor core_pdb_def.h / core.c:2329-2348:
 * pure OFDM 6, 12, 24 Mbps in 500 kbps units: 12, 24, 48) */
static const u8 wave600_add_vap_rates_2g[] = { 12, 24, 48 };
static const u8 wave600_add_vap_rates_5g[] = { 12, 24, 48 };

static int wave600_send_add_vap(struct wave600_radio *radio, bool is_5g)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_add_vap p;
	const u8 *rates = is_5g ? wave600_add_vap_rates_5g : wave600_add_vap_rates_2g;
	u8 rates_len = is_5g ? sizeof(wave600_add_vap_rates_5g) : sizeof(wave600_add_vap_rates_2g);

	memset(&p, 0, sizeof(p));
	memcpy(p.bssid, radio->ndev->dev_addr, ETH_ALEN);
	p.vap_id = radio->vap_id;
	p.operation_mode = 0;	/* OPERATION_MODE_NORMAL */
	memcpy(p.rates, rates, rates_len);
	p.rates_len = rates_len;

	/* Supported HT MCS: 2x2 MIMO MCS 0-15 */
	p.tx_mcs_bitmask[0] = 0xff;
	p.tx_mcs_bitmask[1] = 0xff;

	/* VHT MCS: 2x2 MIMO MCS 0-9 on 5 GHz (0xfffa) */
	if (is_5g) {
		p.vht_mcs_nss[0] = 0xfa;
		p.vht_mcs_nss[1] = 0xff;
		memset(&p.vht_mcs_nss[2], 0xff, 6);
	} else {
		memset(p.vht_mcs_nss, 0xff, sizeof(p.vht_mcs_nss));
	}

	/* HE MCS: 2x2 MIMO MCS 0-11 on both bands (0xfffa) */
	p.he_mcs_nss[0] = 0xfa;
	p.he_mcs_nss[1] = 0xff;
	memset(&p.he_mcs_nss[2], 0xff, 6);

	return wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				     WAVE600_UM_MAN_ADD_VAP_REQ,
				     WAVE600_MC_MAN_ADD_VAP_CFM,
				     &p, sizeof(p), NULL, 0, 5000,
				     radio->vap_id);
}

/* Vendor mtlk_mbss_send_vap_db_add (core.c): register the VAP index in the
 * firmware VAP DB before any activation. CFM returns u16Status at payload
 * offset 2 (UMI_VAP_DB_OP). */
static int wave600_send_vap_db_add(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_vap_db_op p;
	u8 cfm[4] = { 0 };
	u16 status;
	int ret;

	memset(&p, 0, sizeof(p));
	p.operation_code = W600_VAP_OPERATION_ADD;
	p.vap_idx = radio->vap_id;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_VAP_DB_REQ,
				    WAVE600_MC_MAN_VAP_DB_CFM,
				    &p, sizeof(p), cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	status = (u16)(cfm[2] | (cfm[3] << 8));
	if (status != W600_UMI_OK) {
		dev_err(wdev->dev, "Radio %u VAP_DB ADD: FW rejected (status=0x%04x)\n",
			radio->radio_idx, status);
		return -EIO;
	}
	return 0;
}

/* Vendor mtlk_mbss_send_vap_db_del (core.c:2962): unregister the VAP index
 * from the firmware VAP DB when tearing down an AP interface. */
static int wave600_send_vap_db_del(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_vap_db_op p;
	u8 cfm[4] = { 0 };
	u16 status;
	int ret;

	memset(&p, 0, sizeof(p));
	p.operation_code = W600_VAP_OPERATION_DEL;
	p.vap_idx = radio->vap_id;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_VAP_DB_REQ,
				    WAVE600_MC_MAN_VAP_DB_CFM,
				    &p, sizeof(p), cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	status = (u16)(cfm[2] | (cfm[3] << 8));
	if (status != W600_UMI_OK) {
		dev_warn(wdev->dev, "Radio %u VAP_DB DEL: FW rejected (status=0x%04x)\n",
			 radio->radio_idx, status);
		return -EIO;
	}
	return 0;
}

/* Vendor mtlk_set_mib_value_raw (mtlkmib.c): payload is always the full
 * UMI_MIB (object id + status + 80-byte MIB_VALUE union). */
static int wave600_send_mib(struct wave600_radio *radio, u16 object_id,
			    const void *value, size_t value_len)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_mib p;
	u8 cfm[4] = { 0 };
	u16 status;
	int ret;

	if (value_len > sizeof(p.value))
		return -EINVAL;

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC16) */
	p.object_id = cpu_to_le16(object_id);
	if (value_len)
		memcpy(p.value, value, value_len);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_MIB_REQ,
				    WAVE600_MC_MAN_SET_MIB_CFM,
				    &p, sizeof(p), cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	status = (u16)(cfm[2] | (cfm[3] << 8));
	if (status != W600_UMI_OK) {
		dev_err(wdev->dev, "Radio %u SET_MIB 0x%04x: FW rejected (status=0x%04x)\n",
			radio->radio_idx, object_id, status);
		return -EIO;
	}
	return 0;
}

/* Vendor mtlk_mib_set_pre_activate (mib_osdep.c): common physical
 * operational parameters for the pre-activation phase. Only the fields
 * the vendor sets are non-zero (PRE_ACTIVATE_MIB_TYPE layout). */
static int wave600_send_mib_pre_activate(struct wave600_radio *radio, bool is_5g,
					 u8 width, u8 primary_idx)
{
	struct {
		u8 reserved[3];
		u8 network_mode;
		u8 upper_lower_channel;
		u8 spectrum_mode;
		u8 short_slot_time_enabled_11g;
		u8 short_preamble_option_implemented;
		u32 operational_rate_set;
		u32 bss_basic_rate_set;
	} __packed v;

	memset(&v, 0, sizeof(v));
	v.network_mode = is_5g ? W600_NETWORK_11AN_MIXED : W600_NETWORK_11BGN_MIXED;
	/* Vendor core.c:2175: prog_model_spectrum_mode is CW_20 if width is 20 MHz, else CW_40 */
	v.spectrum_mode = (width == 0) ? W600_MIB_SPECTRUM_20M : W600_MIB_SPECTRUM_40M;
	/* Vendor mib_osdep.c:89: upper_lower_channel bonding mode (0 for 20MHz, else primary_idx) */
	v.upper_lower_channel = (width == 0) ? 0 : primary_idx;
	v.short_slot_time_enabled_11g = 1;
	v.short_preamble_option_implemented = 1;
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC32) */
	v.operational_rate_set = cpu_to_le32(is_5g ? W600_OPERATE_RATE_SET_11AN : W600_OPERATE_RATE_SET_11BGN);
	v.bss_basic_rate_set = cpu_to_le32(is_5g ? W600_BSS_BASIC_RATE_SET_11AN : W600_BSS_BASIC_RATE_SET_11BGN_DEFAULT);

	return wave600_send_mib(radio, W600_MIB_PRE_ACTIVATE, &v, sizeof(v));
}

static int wave600_send_mib_spectrum_mode(struct wave600_radio *radio, u8 width)
{
	/* Vendor core.c:2130: MIB_SPECTRUM_MODE uses prog_model_spectrum_mode (CW_20 or CW_40) */
	u8 v = (width == 0) ? W600_MIB_SPECTRUM_20M : W600_MIB_SPECTRUM_40M;

	return wave600_send_mib(radio, W600_MIB_SPECTRUM_MODE, &v, sizeof(v));
}

/* Vendor mtlk_set_vap_mibs (mib_osdep.c): MIB_IEEE_ADDRESS carries the
 * interface MAC before VAP activation. */
static int wave600_send_mib_ieee_addr(struct wave600_radio *radio)
{
	return wave600_send_mib(radio, W600_MIB_IEEE_ADDRESS,
				radio->ndev->dev_addr, ETH_ALEN);
}

/*
 * Pseudo-ACK discriminator (UM_MAN_GET_MIB_REQ, vendor access_mib_value
 * with direction=GET): reads back a previously SET MIB object. If the
 * firmware returns the value the host wrote, the MIB handler executes
 * for real; an all-zero value (or a failed request) means the
 * confirmations carry no real state and every "confirmed" message in the
 * AP bring-up sequence is suspect (2026-10-31 diagnostic session).
 */
static void wave600_diag_mib_readback(struct wave600_radio *radio, u16 object_id,
				      const void *written, size_t written_len,
				      const char *what)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_mib p;
	struct wave600_umi_mib cfm;
	int ret;

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC16) */
	p.object_id = cpu_to_le16(object_id);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_GET_MIB_REQ,
				    WAVE600_MC_MAN_GET_MIB_CFM,
				    &p, sizeof(p), &cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret) {
		dev_dbg(wdev->dev,
			 "Radio %u: GET_MIB 0x%04x (%s): request failed: %d\n",
			 radio->radio_idx, object_id, what, ret);
		return;
	}

	dev_dbg(wdev->dev,
		 "Radio %u: GET_MIB 0x%04x (%s): status=0x%04x value=%*ph (written: %*ph)\n",
		 radio->radio_idx, object_id, what, le16_to_cpu(cfm.status),
		 (int)min_t(size_t, written_len, 16), cfm.value,
		 (int)min_t(size_t, written_len, 16), written);
}

/* Vendor _mtlk_core_set_radio_mode_req (core.c): userspace enables the
 * radio via "sEnableRadio"; without this request the firmware keeps the
 * radio disabled and transmits nothing. */
static int wave600_send_enable_radio(struct wave600_radio *radio, u32 on)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_enable_radio p;

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC32) */
	p.radio_on = cpu_to_le32(on);

	return wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				     WAVE600_UM_MAN_ENABLE_RADIO_REQ,
				     WAVE600_MC_MAN_ENABLE_RADIO_CFM,
				     &p, sizeof(p), NULL, 0, 5000,
				     radio->vap_id);
}

/* Vendor _mtlk_reload_tpc_g5_g6 (mtlk_eeprom.c): for every antenna with
 * TSSI data in the factory calibration, collect the TPC_FREQ entries of
 * all bandwidths around the current channel (exact match, else the
 * closest lower and upper entries) and send them with
 * UM_MAN_SET_TPC_ANT_PARAMS_REQ. Entries are copied byte-exact from the
 * calibration file (little-endian, preserved by the mailbox writer). */
#define W600_TPC_ANT_MSG_SIZE		(4 + 8 * WAVE600_TPC_FREQ_SIZE)

static int wave600_send_tpc_ant_params(struct wave600_radio *radio, u8 chan, bool is_5g)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_cal *cal = &wdev->cal;
	u8 mask = radio->antenna_mask ? radio->antenna_mask : (is_5g ? 0x0A : 0x05);
	u8 target_band = is_5g ? 1 : 0;
	u8 tpc_len = (u8)(cal->tpc_data_size ? cal->tpc_data_size : 9);
	int a, ret;

	if (!cal->parsed)
		return 0;

	for (a = 0; a < WAVE600_CAL_MAX_ANT; a++) {
		struct wave600_umi_tpc_ant_params p;
		u32 count = cal->entry_count[a];
		u32 tpc_cnt = 0;
		int bw;

		if (!(mask & BIT(a)))
			continue;

		memset(&p, 0, sizeof(p));
		p.ant = (u8)a;
		p.tpc_freq_len = tpc_len;

		for (bw = 0; bw < 4 && tpc_cnt < 8; bw++) {
			const struct wave600_cal_tssi_entry *exact = NULL;
			const struct wave600_cal_tssi_entry *lower = NULL;
			const struct wave600_cal_tssi_entry *upper = NULL;
			u32 i;

			u8 target_chan = is_5g ? wave600_chan_to_subband_center_chan(chan, (u8)bw) : chan;

			for (i = 0; i < count; i++) {
				const struct wave600_cal_tssi_entry *e = &cal->entries[a][i];

				if (e->band != target_band || e->bw != bw)
					continue;

				if (e->chan == target_chan) {
					exact = e;
					break;
				} else if (e->chan < target_chan) {
					lower = e;
				} else if (e->chan > target_chan && !upper) {
					upper = e;
				}
			}

			if (exact) {
				memcpy(p.tpc_freq + tpc_cnt * WAVE600_TPC_FREQ_SIZE, exact->raw, WAVE600_TPC_FREQ_SIZE);
				tpc_cnt++;
			} else {
				if (lower && tpc_cnt < 8) {
					memcpy(p.tpc_freq + tpc_cnt * WAVE600_TPC_FREQ_SIZE, lower->raw, WAVE600_TPC_FREQ_SIZE);
					tpc_cnt++;
				}
				if (upper && tpc_cnt < 8) {
					memcpy(p.tpc_freq + tpc_cnt * WAVE600_TPC_FREQ_SIZE, upper->raw, WAVE600_TPC_FREQ_SIZE);
					tpc_cnt++;
				}
			}
		}

		if (!tpc_cnt) {
			/* Fallback: take available entries for this band */
			u32 i;
			for (i = 0; i < count && tpc_cnt < 8; i++) {
				if (cal->entries[a][i].band == target_band) {
					memcpy(p.tpc_freq + tpc_cnt * WAVE600_TPC_FREQ_SIZE,
					       cal->entries[a][i].raw, WAVE600_TPC_FREQ_SIZE);
					tpc_cnt++;
				}
			}
		}

		if (tpc_cnt > 0) {
			ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
						    WAVE600_UM_MAN_SET_TPC_ANT_PARAMS_REQ,
						    WAVE600_MC_MAN_SET_TPC_ANT_PARAMS_CFM,
						    &p, sizeof(p), NULL, 0, 5000,
						    radio->vap_id);
			if (ret) {
				dev_warn(wdev->dev, "Radio %u SET_TPC_ANT_PARAMS (ant %d) failed: %d\n",
					 radio->radio_idx, a, ret);
			} else {
				dev_info(wdev->dev, "Radio %u active antenna %u TPC calibrated v2-24b (%u freq points, stride=%d)\n",
					 radio->radio_idx, a, tpc_cnt, WAVE600_TPC_FREQ_SIZE);
			}
		}
	}

	return 0;
}

static int wave600_send_tpc_config(struct wave600_radio *radio, u8 chan, bool is_5g)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_tpc_config p;
	u16 pw[13] = { 0 };
	u16 target_pw;
	u8 regd = wdev->regd_code ? wdev->regd_code : WAVE600_REGD_CODE_ETSI;
	int ret, i;

	memset(&p, 0, sizeof(p));
	p.cal_type = W600_TPC_CAL_TYPE_LOGARITHMIC; /* 1 */
	p.tpc_loop_type = 0; /* 0 = closed loop (TPC_CLOSED_LOOP, vendor Gen6 default in radio_pdb_def.h:219) */
	p.fixed_gain = W600_TPC_NO_FIXED_GAIN; /* 0xFF */
	p.get_set_operation = 0; /* API_SET_OPERATION */

	/* Regulatory domain power limits from PSDB / PSD.bin for active country */
	ret = wave600_psdb_regd_limits(wdev, regd, chan, pw);
	if (ret || !pw[1]) {
		/* Fallback: 20 dBm (160 in 1/8 dBm) for 2.4GHz, 23 dBm (184 in 1/8 dBm) for 5GHz */
		u16 def_pw = is_5g ? (23 * 8) : (20 * 8);
		pw[0] = is_5g ? 0 : def_pw;
		for (i = 1; i < 13; i++)
			pw[i] = def_pw;
	} else {
		/* Fallback for higher bandwidths: in PSD.bin 40/80/160 MHz entries are indexed
		 * by center channel frequencies, so matching on primary channel frequency leaves
		 * pw[2..4] as 0. Fall back to 20 MHz limit so firmware does not clamp transmit
		 * power for 40/80/160 MHz frames to 0 dBm (minimum power). */
		for (i = 1; i < 4; i++) {
			if (pw[1 + i] == 0)
				pw[1 + i] = pw[1];
		}
		for (i = 0; i < 4; i++) {
			if (pw[5 + i] == 0)
				pw[5 + i] = pw[1 + i];
			if (pw[9 + i] == 0)
				pw[9 + i] = pw[1 + i];
		}
		if (is_5g)
			pw[0] = 0;
	}

	/* If a specific tx power was requested by the user, clamp target limits */
	if (radio->tx_power_dbm > 0) {
		target_pw = (u16)(radio->tx_power_dbm * 8);
		for (i = 1; i < 13; i++) {
			if (pw[i] == 0 || pw[i] > target_pw)
				pw[i] = target_pw;
		}
		if (!is_5g && (pw[0] == 0 || pw[0] > target_pw))
			pw[0] = target_pw;
	}

	p.power_limit_11b = cpu_to_le16(pw[0]);
	for (i = 0; i < 4; i++) {
		p.regulation_limit[i] = cpu_to_le16(pw[1 + i]);
		p.regulation_limit_mu[i] = cpu_to_le16(pw[5 + i]);
		p.regulation_limit_bf[i] = cpu_to_le16(pw[9 + i]);
	}

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_TPC_CONFIG_REQ,
				    WAVE600_MC_MAN_SET_TPC_CONFIG_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret) {
		dev_err(wdev->dev, "Radio %u SET_TPC_CONFIG failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	dev_info(wdev->dev, "Radio %u TPC configured v3-regd-%s (code 0x%02x, loop closed): 11b=%u, OFDM 20/40/80/160=[%u %u %u %u] (1/8 dBm, tx_pwr=%d dBm)\n",
		 radio->radio_idx, wdev->country[0] ? wdev->country : "PL", regd,
		 pw[0], pw[1], pw[2], pw[3], pw[4],
		 radio->tx_power_dbm ? radio->tx_power_dbm : (is_5g ? 23 : 20));
	return 0;
}



/* Vendor _mtlk_mbss_send_preactivate_req (core.c): issued prior to the
 * first VAP activation; coexistence / 40 MHz intolerance stay disabled.
 * CFM returns u16Status at payload offset 0 (UMI_MBSS_PRE_ACTIVATE_HDR). */
static int wave600_send_pre_activate(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_mbss_pre_activate p;
	u8 cfm[4] = { 0 };
	u16 status;
	int ret;

	memset(&p, 0, sizeof(p));

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_MBSS_PRE_ACTIVATE_REQ,
				    WAVE600_MC_MAN_MBSS_PRE_ACTIVATE_CFM,
				    &p, sizeof(p), cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	status = (u16)(cfm[0] | (cfm[1] << 8));
	if (status != W600_UMI_OK) {
		dev_err(wdev->dev, "Radio %u MBSS_PRE_ACTIVATE: FW rejected (status=0x%04x)\n",
			radio->radio_idx, status);
		return -EIO;
	}
	return 0;
}

/* Vendor UM_MAN_SET_WMM_PARAMETERS_REQ (0x040A) */
static int wave600_send_set_wmm_params(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_set_wmm_params p;
	struct wave600_umi_set_wmm_params cfm;
	int ret;

	memset(&p, 0, sizeof(p));
	p.vap_id = radio->vap_id;

	/*
	 * Default WMM / EDCA parameters for Access Categories (vendor core_pdb_def.h / core_config.c):
	 * Index 0 = BE (Best Effort):  cwmin=4 (15), cwmax=10 (1023), aifs=3, txop=0
	 * Index 1 = BK (Background):   cwmin=4 (15), cwmax=10 (1023), aifs=7, txop=0
	 * Index 2 = VI (Video):        cwmin=3 (7),  cwmax=4 (15),   aifs=2, txop=94 (3008us/32)
	 * Index 3 = VO (Voice):        cwmin=2 (3),  cwmax=3 (7),    aifs=2, txop=47 (1504us/32)
	 */
	p.ac[0].cw_min = cpu_to_le16(4);
	p.ac[0].cw_max = cpu_to_le16(10);
	p.ac[0].txop_limit = 0;
	p.ac[0].aifsn = 3;

	p.ac[1].cw_min = cpu_to_le16(4);
	p.ac[1].cw_max = cpu_to_le16(10);
	p.ac[1].txop_limit = 0;
	p.ac[1].aifsn = 7;

	p.ac[2].cw_min = cpu_to_le16(3);
	p.ac[2].cw_max = cpu_to_le16(4);
	p.ac[2].txop_limit = cpu_to_le16(94);
	p.ac[2].aifsn = 2;

	p.ac[3].cw_min = cpu_to_le16(2);
	p.ac[3].cw_max = cpu_to_le16(3);
	p.ac[3].txop_limit = cpu_to_le16(47);
	p.ac[3].aifsn = 2;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_WMM_PARAMETERS_REQ,
				    WAVE600_UM_MAN_SET_WMM_PARAMETERS_CFM,
				    &p, sizeof(p), &cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	if (cfm.status != W600_UMI_OK) {
		dev_err(wdev->dev, "Radio %u SET_WMM_PARAMS: FW rejected (status=%u)\n",
			radio->radio_idx, cfm.status);
		return -EIO;
	}
	dev_info(wdev->dev, "Radio %u WMM / EDCA parameters set (VAP transmit queues activated)\n",
		 radio->radio_idx);
	return 0;
}

static int wave600_send_beacon(struct wave600_radio *radio,
			       const struct cfg80211_beacon_data *beacon)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_beacon_set p;
	struct wave600_umi_beacon_set cfm;
	dma_addr_t dma;
	u32 part1_len, part2_len, part3_len, part4_len, part5_len;
	size_t total;
	u8 *buf;
	const u8 *ie;
	int ret;

	if (!beacon->head || !beacon->head_len)
		return -EINVAL;

	/* part1 = fixed beacon head; parts 2-5 classify the tail IEs exactly
	 * like vendor beacon_manager.c _wave_get_beacon_data_length. The
	 * template buffer stays contiguous (head followed by tail). */
	total = beacon->head_len + beacon->tail_len;
	buf = kmalloc(total, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, beacon->head, beacon->head_len);
	if (beacon->tail && beacon->tail_len)
		memcpy(buf + beacon->head_len, beacon->tail, beacon->tail_len);
	dma = dma_map_single(wdev->dev, buf, total, DMA_TO_DEVICE);
	if (dma_mapping_error(wdev->dev, dma)) {
		kfree(buf);
		return -ENOMEM;
	}

	part1_len = beacon->head_len;
	part2_len = 0;
	part3_len = 0;
	part4_len = 0;
	part5_len = 0;

	ie = buf + beacon->head_len;
	while (ie < buf + total) {
		u8 id = ie[0];
		u16 ie_len = ie[1] + 2;

		if (ie + ie_len > buf + total) {
			dev_err(wdev->dev, "Radio %u beacon template: wrong IE length (id %u len %u)\n",
				radio->radio_idx, id, ie[1]);
			dma_unmap_single(wdev->dev, dma, total, DMA_TO_DEVICE);
			kfree(buf);
			return -EINVAL;
		}
		ie += ie_len;

		switch (id) {
		case W600_IE_TIM:
		case W600_IE_COUNTRY:
		case W600_IE_FH_PATTRN_PARAMS:
		case W600_IE_FH_PATTRM_TABLE:
		case W600_IE_PWR_CONSTRAINT:
			part2_len += ie_len;
			break;
		case W600_IE_CHANNEL_SW_ANN:
		case W600_IE_QUIET:
		case W600_IE_IBSS_DFS:
		case W600_IE_TPC_REPORT:
		case W600_IE_ERP_INFO:
		case W600_IE_EXT_SUPP_RATES:
		case W600_IE_RSN:
			part3_len += ie_len;
			break;
		case W600_IE_BSS_LOAD:
		case W600_IE_EDCA_PARAM_SET:
		case W600_IE_HT_CAPABILITIES:
		case W600_IE_QOS_CAPABILITY:
		case W600_IE_AP_CHANNEL_REPORT:
		case W600_IE_MOBILITY_DOMAIN:
		case W600_IE_DSE_REGISTERED_LOCATION:
		case W600_IE_SUPPORTED_OPER_CLASSES:
		case W600_IE_EXT_CHANNEL_SW_ANN:
		case W600_IE_HT_INFORMATION:
		case W600_IE_BSS_AVERAGE_ACCESS:
		case W600_IE_ANTENNA:
		case W600_IE_MEASURE_PILOT_TRANSMISSION:
		case W600_IE_BSS_AVAILABLE_ADMISSION_CAPACITY:
		case W600_IE_BSS_AC_ACCESS_DELAY:
		case W600_IE_TIME_ADV:
		case W600_IE_RM_ENABLED_CAPABILITIES:
		case W600_IE_MULTIPLE_BSSID:
		case W600_IE_COEXISTENCE:
		case W600_IE_OBSS_SCAN_PARAMETERS:
		case W600_IE_FMS_DESCRIPTOR:
		case W600_IE_QOS_TRAFFIC_CAPABILITY:
		case W600_IE_INTERWORKING:
		case W600_IE_ADV_PROTO:
		case W600_IE_ROAMING_CONS:
		case W600_IE_EMERGENCY_ALERT_ID:
		case W600_IE_MESH_CONFIG:
		case W600_IE_MESH_ID:
		case W600_IE_MESH_CHANNEL_SW_PARAMETERS:
		case W600_IE_MESH_AWAKE_WINDOW:
		case W600_IE_BEACON_TIMING:
		case W600_IE_MCCAOP_ADV:
		case W600_IE_EXT_CAPABILITIES:
		case W600_IE_MULTI_BAND:
		case W600_IE_MCCAOP_ADV_OVERVIEW:
		case W600_IE_QMF_POLICY:
		case W600_IE_QLOAD_REPORT:
		case W600_IE_HCCA_TXOP_UPDATE_COUNT:
		case W600_IE_VHT_CAPABILITIES:
		case W600_IE_VHT_OPERATION:
		case W600_IE_TX_PWR_ENVELOPE:
			part4_len += ie_len;
			break;
		default:
			part5_len += ie_len;
			break;
		}
	}

	if (total != part1_len + part2_len + part3_len + part4_len + part5_len) {
		dev_err(wdev->dev, "Radio %u beacon template lengths mismatch: %zu != %u+%u+%u+%u+%u\n",
			radio->radio_idx, total, part1_len, part2_len, part3_len, part4_len,
			part5_len);
		dma_unmap_single(wdev->dev, dma, total, DMA_TO_DEVICE);
		kfree(buf);
		return -EINVAL;
	}

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC16/32).
	 * The firmware dereferences host_address to DMA the template on
	 * every beacon interval - a wrong byte order means it reads garbage. */
	p.host_address = cpu_to_le32((u32)dma);
	p.part1_len = cpu_to_le16((u16)part1_len);
	p.part2_len = cpu_to_le16((u16)part2_len);
	p.part3_len = cpu_to_le16((u16)part3_len);
	p.part4_len = cpu_to_le16((u16)part4_len);
	p.part5_len = cpu_to_le16((u16)part5_len);
	p.vap_index = radio->vap_id;

	dev_dbg(wdev->dev, "Radio %u beacon template: total %zu = %u/%u/%u/%u/%u (dma %pad)\n",
		radio->radio_idx, total, part1_len, part2_len, part3_len, part4_len, part5_len,
		&dma);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_BEACON_TEMPLATE_REQ,
				    WAVE600_MC_MAN_SET_BEACON_TEMPLATE_CFM,
				    &p, sizeof(p), &cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret || cfm.status != W600_UMI_OK) {
		if (!ret) {
			dev_err(wdev->dev, "Radio %u SET_BEACON_TEMPLATE: FW rejected (status=%u)\n",
				radio->radio_idx, cfm.status);
			ret = -EIO;
		}
		dma_unmap_single(wdev->dev, dma, total, DMA_TO_DEVICE);
		kfree(buf);
		return ret;
	}

	/*
	 * Keep the template buffer persistently mapped in host DDR: the FullMAC
	 * ARC firmware periodic beacon engine DMAs this buffer across PCIe
	 * on every beacon interval (~100ms). Free old buffer if reallocated.
	 */
	if (radio->beacon_buf) {
		dma_unmap_single(wdev->dev, radio->beacon_dma, radio->beacon_len,
				 DMA_TO_DEVICE);
		kfree(radio->beacon_buf);
	}
	radio->beacon_buf = buf;
	radio->beacon_dma = dma;
	radio->beacon_len = total;
	radio->beacon_set_params = p;

	return 0;
}

/* ------------------------------------------------------------------ */
/* cfg80211 operations                                                 */
/* ------------------------------------------------------------------ */

static int wave600_chandef_to_params(struct wave600_radio *radio,
				     const struct cfg80211_chan_def *chandef,
				     u8 *out_chan, u8 *out_width,
				     u8 *out_low_chan, u8 *out_primary_idx)
{
	bool is_5g = (radio->radio_idx == 1);
	u8 chan, width, low_chan, primary_idx = 0;

	if (radio->radio_idx == 0 && chandef->chan->band != NL80211_BAND_2GHZ)
		return -EOPNOTSUPP;
	if (radio->radio_idx == 1 && chandef->chan->band != NL80211_BAND_5GHZ)
		return -EOPNOTSUPP;

	chan = chandef->chan->hw_value;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_20:
	case NL80211_CHAN_WIDTH_20_NOHT:
		width = 0;	/* CW_20 */
		low_chan = chan;
		primary_idx = 0;
		break;
	case NL80211_CHAN_WIDTH_40:
		width = 1;	/* CW_40 */
		if (is_5g) {
			u32 cf1 = chandef->center_freq1;
			u32 low_freq = cf1 - 10;
			low_chan = (u8)((low_freq - 5000) / 5);
			primary_idx = (chan >= low_chan) ? (chan - low_chan) / 4 : 0;
		} else {
			low_chan = (chandef->center_freq1 > chandef->chan->center_freq) ? chan : (chan - 4);
			primary_idx = (chan - low_chan) / 4;
		}
		break;
	case NL80211_CHAN_WIDTH_80:
		width = 2;	/* CW_80 */
		if (is_5g) {
			u32 cf1 = chandef->center_freq1;
			u32 low_freq = cf1 - 30;
			low_chan = (u8)((low_freq - 5000) / 5);
			primary_idx = (chan >= low_chan) ? (chan - low_chan) / 4 : 0;
		} else {
			return -EOPNOTSUPP;
		}
		break;
	case NL80211_CHAN_WIDTH_160:
		width = 3;	/* CW_160 */
		if (is_5g) {
			u32 cf1 = chandef->center_freq1;
			u32 low_freq = cf1 - 70;
			low_chan = (u8)((low_freq - 5000) / 5);
			primary_idx = (chan >= low_chan) ? (chan - low_chan) / 4 : 0;
		} else {
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	*out_chan = chan;
	*out_width = width;
	*out_low_chan = low_chan;
	*out_primary_idx = primary_idx;
	return 0;
}

static int wave600_cfg80211_start_ap(struct wiphy *wiphy, struct net_device *ndev,
				     struct cfg80211_ap_settings *settings)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	bool is_5g;
	u8 chan, width, low_chan, primary_idx = 0;
	int ret;

	if (wdev->state != W600_STATE_RUNNING || atomic_read(&wdev->in_recovery))
		return -EBUSY;

	if (settings->ssid_len == 0 || settings->ssid_len > W600_SSID_MAX_LEN)
		return -EINVAL;

	ret = wave600_chandef_to_params(radio, &settings->chandef,
					&chan, &width, &low_chan, &primary_idx);
	if (ret)
		return ret;

	is_5g = (radio->radio_idx == 1);
	radio->ap_channel = chan;
	radio->ap_width = width;
	radio->ap_freq = settings->chandef.chan->center_freq;
	radio->ap_low_chan = low_chan;
	radio->ap_primary_idx = primary_idx;
	radio->beacon_interval = (settings && settings->beacon_interval) ? settings->beacon_interval : 100;
	radio->dtim_period = (settings && settings->dtim_period) ? settings->dtim_period : 2;

	dev_info(wdev->dev, "Radio %u start_ap: SSID '%.*s' band %s channel %d (low %u, idx %u) width %d\n",
		 radio->radio_idx, (int)settings->ssid_len, settings->ssid,
		 is_5g ? "5GHz" : "2.4GHz", chan, low_chan, primary_idx, width);

	/*
	 * Vendor AP bring-up sequence (verified against _mtlk_core_on_rcvry_configure
	 * in core.c:14677-14752 and live hybrid boot log):
	 *  1. VAP_DB_ADD -> MIB_IEEE_ADDRESS -> MIB_PRE_ACTIVATE ->
	 *     MIB_SPECTRUM_MODE -> MBSS_PRE_ACTIVATE -> ADD_VAP (only if not already added by CAC)
	 *  2. ENABLE_RADIO(1)
	 *  3. SET_CHAN -> SET_TPC_ANT_PARAMS -> SET_TPC_CONFIG
	 *  4. SET_BEACON_TEMPLATE -> SET_BEACON_INTERVAL -> SET_BSS
	 */
	if (!radio->vap_added) {
		ret = wave600_send_vap_db_add(radio);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: VAP_DB ADD failed: %d\n", radio->radio_idx, ret);
			return ret;
		}

		ret = wave600_send_mib_ieee_addr(radio);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: MIB_IEEE_ADDRESS failed: %d\n", radio->radio_idx, ret);
			return ret;
		}

		/* Pseudo-ACK discriminator: verify the FW really stored the MAC */
		wave600_diag_mib_readback(radio, W600_MIB_IEEE_ADDRESS,
					  radio->ndev->dev_addr, ETH_ALEN,
					  "IEEE_ADDRESS after SET");

		ret = wave600_send_mib_pre_activate(radio, is_5g, width, primary_idx);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: MIB_PRE_ACTIVATE failed: %d\n", radio->radio_idx, ret);
			return ret;
		}

		ret = wave600_send_mib_spectrum_mode(radio, width);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: MIB_SPECTRUM_MODE failed: %d\n", radio->radio_idx, ret);
			return ret;
		}

		ret = wave600_send_pre_activate(radio);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: MBSS_PRE_ACTIVATE failed: %d\n", radio->radio_idx, ret);
			return ret;
		}

		ret = wave600_send_add_vap(radio, is_5g);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: ADD_VAP failed: %d\n", radio->radio_idx, ret);
			return ret;
		}
		radio->vap_added = true;
	}

	if (!radio->radio_enabled) {
		ret = wave600_send_enable_radio(radio, 1);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: ENABLE_RADIO failed: %d\n", radio->radio_idx, ret);
			return ret;
		}
		radio->radio_enabled = true;
	}

	/*
	 * Vendor _core_on_rcvry_configure defaults for Gen6. These are sent
	 * before SET_CHAN.
	 */
	wave600_send_he_mu_operation(radio, 1);
	wave600_send_bf_mode(radio, WAVE600_BF_MODE_NON_BF);
	wave600_send_txop_mode(radio, WAVE600_GEN6_ALL_STA_SID,
			       WAVE600_TXOP_MODE_INVALID);

	/*
	 * Vendor core_config.c:866-869: TPC config + TPC ANT params MUST be
	 * sent BEFORE SET_CHAN so the firmware initializes RF synthesizer
	 * and Front-End Module (FEM PA/LNA) bias to calibrated output levels.
	 */
	wave600_send_tpc_config(radio, chan, is_5g);
	wave600_send_tpc_ant_params(radio, chan, is_5g);

	ret = wave600_send_set_chan(radio, low_chan, width, primary_idx,
				    W600_SWITCH_TYPE_NORMAL, 0);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_CHAN failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	ret = wave600_send_beacon(radio, &settings->beacon);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_BEACON_TEMPLATE failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	ret = wave600_send_set_bss(radio, settings, is_5g);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_BSS failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	/* Vendor core_config.c:150-205 _mtlk_core_cfg_wmm_param_send -> UM_MAN_SET_WMM_PARAMETERS_REQ */
	ret = wave600_send_set_wmm_params(radio);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_WMM_PARAMS failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	/*
	 * Enable Block Ack and AMSDU support on the VAP (UM_MAN_TS_VAP_CONFIGURE_REQ).
	 * Vendor sAggrConfig sets enableBa = 1, amsduSupport = 1, windowSize = 64.
	 * Required for firmware ARC to negotiate Block Ack (ADDBA) with associated STAs
	 * for frames >= 716 bytes.
	 */
	ret = wave600_send_ts_vap_configure(radio, 1, 1, 64);
	if (ret)
		dev_warn(wdev->dev, "Radio %u start_ap: TS_VAP_CONFIGURE failed: %d\n",
			 radio->radio_idx, ret);

	radio->chandef = settings->chandef;
	radio->ap_started = true;
	netif_carrier_on(ndev);
	netif_tx_wake_all_queues(ndev);
	dev_info(wdev->dev, "Radio %u AP started on %s (%s, channel %u)\n",
		 radio->radio_idx, ndev->name, is_5g ? "5GHz" : "2.4GHz", chan);
	return 0;
}

int wave600_cfg80211_restore_ap(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	struct cfg80211_ap_settings settings;
	struct wave600_umi_beacon_set cfm;
	bool is_5g;
	int ret;

	if (!radio->ap_started)
		return 0;

	is_5g = (radio->radio_idx == 1);
	dev_info(wdev->dev, "Radio %u restoring AP on %s (ch %u, width %u, low %u, idx %u)...\n",
		 radio->radio_idx, radio->ndev ? radio->ndev->name : "wlan",
		 radio->ap_channel, radio->ap_width, radio->ap_low_chan, radio->ap_primary_idx);

	ret = wave600_send_vap_db_add(radio);
	if (ret)
		return ret;

	ret = wave600_send_mib_ieee_addr(radio);
	if (ret)
		return ret;

	ret = wave600_send_mib_pre_activate(radio, is_5g, radio->ap_width, radio->ap_primary_idx);
	if (ret)
		return ret;

	ret = wave600_send_mib_spectrum_mode(radio, radio->ap_width);
	if (ret)
		return ret;

	ret = wave600_send_pre_activate(radio);
	if (ret)
		return ret;

	ret = wave600_send_add_vap(radio, is_5g);
	if (ret)
		return ret;

	ret = wave600_send_enable_radio(radio, 1);
	if (ret)
		return ret;
	radio->radio_enabled = true;

	wave600_send_he_mu_operation(radio, 1);
	wave600_send_bf_mode(radio, WAVE600_BF_MODE_NON_BF);
	wave600_send_txop_mode(radio, WAVE600_GEN6_ALL_STA_SID,
			       WAVE600_TXOP_MODE_INVALID);

	/*
	 * Vendor core_config.c:866-869: TPC config + TPC ANT params MUST be
	 * sent BEFORE SET_CHAN so the firmware initializes RF synthesizer
	 * and Front-End Module (FEM PA/LNA) bias to calibrated output levels.
	 */
	wave600_send_tpc_config(radio, (u8)radio->ap_channel, is_5g);
	wave600_send_tpc_ant_params(radio, (u8)radio->ap_channel, is_5g);

	ret = wave600_send_set_chan(radio, radio->ap_low_chan, radio->ap_width,
				    radio->ap_primary_idx, W600_SWITCH_TYPE_NORMAL, 0);
	if (ret)
		return ret;

	if (radio->beacon_dma && radio->beacon_set_params.part1_len) {
		ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
					    WAVE600_UM_MAN_SET_BEACON_TEMPLATE_REQ,
					    WAVE600_MC_MAN_SET_BEACON_TEMPLATE_CFM,
					    &radio->beacon_set_params, sizeof(radio->beacon_set_params),
					    &cfm, sizeof(cfm), 5000,
					    radio->vap_id);
		if (ret)
			dev_warn(wdev->dev, "Radio %u restore SET_BEACON_TEMPLATE failed: %d\n",
				 radio->radio_idx, ret);
	}

	memset(&settings, 0, sizeof(settings));
	settings.beacon_interval = radio->beacon_interval;
	settings.dtim_period = radio->dtim_period;
	ret = wave600_send_set_bss(radio, &settings, is_5g);
	if (ret)
		return ret;

	ret = wave600_send_set_wmm_params(radio);
	if (ret)
		return ret;

	wave600_send_ts_vap_configure(radio, 1, 1, 64);

	if (radio->ndev) {
		netif_carrier_on(radio->ndev);
		netif_tx_wake_all_queues(radio->ndev);
	}
	radio->vap_added = true;
	radio->radio_enabled = true;
	dev_info(wdev->dev, "Radio %u AP restored successfully after recovery\n", radio->radio_idx);
	return 0;
}

static int wave600_cfg80211_stop_ap(struct wiphy *wiphy, struct net_device *ndev,
				    unsigned int link_id)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct {
		u16 status;
		u8 vap_id;
		u8 reserved;
	} __packed p;
	int ret;

	if (radio->cac_in_progress) {
		cancel_delayed_work_sync(&radio->cac_work);
		radio->cac_in_progress = false;
		cfg80211_cac_event(ndev, &radio->cac_chandef,
				   NL80211_RADAR_CAC_ABORTED, GFP_KERNEL, 0);
	}

	if (!radio->ap_started && !radio->vap_added)
		return 0;

	if (wdev->state != W600_STATE_RUNNING || atomic_read(&wdev->in_recovery)) {
		netif_tx_stop_all_queues(ndev);
		netif_carrier_off(ndev);
		radio->vap_added = false;
		radio->radio_enabled = false;
		radio->ap_started = false;
		return 0;
	}

	netif_tx_stop_all_queues(ndev);
	netif_carrier_off(ndev);

	memset(&p, 0, sizeof(p));
	p.vap_id = radio->vap_id;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_STOP_VAP_TRAFFIC_REQ,
				    WAVE600_MC_MAN_STOP_VAP_TRAFFIC_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret)
		dev_err(wdev->dev, "Radio %u stop_ap: STOP_VAP_TRAFFIC failed: %d\n", radio->radio_idx, ret);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_REMOVE_VAP_REQ,
				    WAVE600_MC_MAN_REMOVE_VAP_CFM,
				    &p, sizeof(p), NULL, 0, 5000,
				    radio->vap_id);
	if (ret)
		dev_err(wdev->dev, "Radio %u stop_ap: REMOVE_VAP failed: %d\n", radio->radio_idx, ret);

	ret = wave600_send_vap_db_del(radio);
	if (ret)
		dev_warn(wdev->dev, "Radio %u stop_ap: VAP_DB DEL failed: %d\n", radio->radio_idx, ret);
	radio->vap_added = false;

	if (radio->radio_enabled) {
		ret = wave600_send_enable_radio(radio, 0);
		if (ret)
			dev_warn(wdev->dev, "Radio %u stop_ap: DISABLE_RADIO failed: %d\n",
				 radio->radio_idx, ret);
		radio->radio_enabled = false;
	}

	if (radio->beacon_buf) {
		dma_unmap_single(wdev->dev, radio->beacon_dma, radio->beacon_len,
				 DMA_TO_DEVICE);
		kfree(radio->beacon_buf);
		radio->beacon_buf = NULL;
		radio->beacon_dma = 0;
		radio->beacon_len = 0;
	}

	radio->ap_started = false;
	memset(&radio->chandef, 0, sizeof(radio->chandef));
	radio->is_secure = false;
	memset(radio->stations, 0, sizeof(radio->stations));
	radio->num_stations = 0;
	return 0;
}

static int wave600_cfg80211_change_beacon(struct wiphy *wiphy, struct net_device *ndev,
					  struct cfg80211_ap_update *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	if (radio->wdev->state != W600_STATE_RUNNING || atomic_read(&radio->wdev->in_recovery))
		return -EBUSY;

	return wave600_send_beacon(radio, &params->beacon);
}

static int wave600_cfg80211_set_channel(struct wiphy *wiphy,
					struct net_device *dev,
					struct cfg80211_chan_def *chandef)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	u8 chan, width, low_chan, primary_idx = 0;
	int ret;

	ret = wave600_chandef_to_params(radio, chandef, &chan, &width, &low_chan, &primary_idx);
	if (ret)
		return ret;

	radio->ap_channel = chan;
	radio->ap_width = width;
	radio->ap_freq = chandef->chan->center_freq;
	radio->chandef = *chandef;

	if (!radio->ap_started)
		return 0;

	dev_info(radio->wdev->dev, "Radio %u set_channel: band %s channel %u (low %u, idx %u, width %u)\n",
		 radio->radio_idx, (radio->radio_idx == 1) ? "5GHz" : "2.4GHz",
		 chan, low_chan, primary_idx, width);

	/* Hot channel change: vendor sends SET_CHAN with switch_type=ST_NORMAL.
	 * Do NOT send TPC commands here — vendor only reloads TPC during
	 * start_ap; sending SET_TPC on hot channel change can crash the LM0
	 * core in PowerAdaptation.c:1717. */
	ret = wave600_send_set_chan(radio, low_chan, width, primary_idx,
				    W600_SWITCH_TYPE_NORMAL, 0);
	if (ret) {
		dev_err(radio->wdev->dev, "Radio %u set_channel: SET_CHAN failed: %d\n",
			radio->radio_idx, ret);
		return ret;
	}

	return 0;
}

static int wave600_cfg80211_channel_switch(struct wiphy *wiphy,
					   struct net_device *ndev,
					   struct cfg80211_csa_settings *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	u8 chan, width, low_chan, primary_idx = 0;
	int ret;

	ret = wave600_chandef_to_params(radio, &params->chandef, &chan, &width, &low_chan, &primary_idx);
	if (ret)
		return ret;

	dev_info(radio->wdev->dev, "Radio %u channel_switch: band %s channel %d -> %d (low %u, idx %u, width %d)\n",
		 radio->radio_idx, (radio->radio_idx == 1) ? "5GHz" : "2.4GHz",
		 radio->ap_channel, chan, low_chan, primary_idx, width);

	/* CSA: vendor core_cfg_set_chan sets switch_type=ST_CSA and
	 * chan_switch_time = nb_beacons * beacon_interval_ms. */
	u16 chan_switch_time = params->count * W600_BEACON_INT;
	ret = wave600_send_set_chan(radio, low_chan, width, primary_idx,
				    W600_SWITCH_TYPE_CSA, chan_switch_time);
	if (ret) {
		dev_err(radio->wdev->dev, "Radio %u channel_switch: SET_CHAN failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	radio->ap_channel = chan;
	radio->ap_freq = params->chandef.chan->center_freq;
	radio->chandef = params->chandef;

	cfg80211_ch_switch_notify(ndev, &params->chandef, 0);
	return 0;
}

static void wave600_cac_work(struct work_struct *work)
{
	struct wave600_radio *radio = container_of(to_delayed_work(work),
						   struct wave600_radio,
						   cac_work);

	if (!radio->cac_in_progress)
		return;

	radio->cac_in_progress = false;
	dev_info(radio->wdev->dev, "Radio %u CAC finished successfully on ch %u (width %u). Notifying hostapd...\n",
		 radio->radio_idx, radio->cac_chandef.chan->hw_value,
		 radio->cac_chandef.width);

	cfg80211_cac_event(radio->ndev, &radio->cac_chandef,
			   NL80211_RADAR_CAC_FINISHED, GFP_KERNEL, 0);
}

static int wave600_cfg80211_start_radar_detection(struct wiphy *wiphy,
						  struct net_device *dev,
						  struct cfg80211_chan_def *chandef,
						  u32 cac_time_ms, int link_id)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	u8 chan, width, low_chan, primary_idx = 0;
	int ret;

	if (radio->radio_idx == 0)
		return -EOPNOTSUPP;

	ret = wave600_chandef_to_params(radio, chandef, &chan, &width, &low_chan, &primary_idx);
	if (ret)
		return ret;

	cancel_delayed_work_sync(&radio->cac_work);

	dev_info(wdev->dev, "Radio %u start_radar_detection: ch %u (low %u, idx %u, width %u, cac %u ms)\n",
		 radio->radio_idx, chan, low_chan, primary_idx, width, cac_time_ms);

	if (!radio->radio_enabled) {
		ret = wave600_send_vap_db_add(radio);
		if (ret)
			return ret;
		ret = wave600_send_mib_ieee_addr(radio);
		if (ret)
			return ret;
		ret = wave600_send_mib_pre_activate(radio, true, width, primary_idx);
		if (ret)
			return ret;
		ret = wave600_send_mib_spectrum_mode(radio, width);
		if (ret)
			return ret;
		ret = wave600_send_pre_activate(radio);
		if (ret)
			return ret;
		ret = wave600_send_add_vap(radio, true);
		if (ret)
			return ret;
		radio->vap_added = true;
		ret = wave600_send_enable_radio(radio, 1);
		if (ret)
			return ret;
		radio->radio_enabled = true;
		wave600_send_he_mu_operation(radio, 1);
		wave600_send_bf_mode(radio, WAVE600_BF_MODE_NON_BF);
		wave600_send_txop_mode(radio, WAVE600_GEN6_ALL_STA_SID, WAVE600_TXOP_MODE_INVALID);
	}

	/* Vendor core_config.c:867: Always configure TPC limits before SET_CHAN */
	wave600_send_tpc_config(radio, chan, true);
	wave600_send_tpc_ant_params(radio, chan, true);

	ret = wave600_send_set_chan(radio, low_chan, width, primary_idx,
				    W600_SWITCH_TYPE_SCAN, 0);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_radar_detection: SET_CHAN failed: %d\n",
			radio->radio_idx, ret);
		return ret;
	}

	radio->ap_channel = chan;
	radio->ap_width = width;
	radio->ap_freq = chandef->chan->center_freq;
	radio->ap_low_chan = low_chan;
	radio->ap_primary_idx = primary_idx;
	radio->cac_chandef = *chandef;
	radio->cac_time_ms = cac_time_ms;
	radio->cac_in_progress = true;

	schedule_delayed_work(&radio->cac_work, msecs_to_jiffies(cac_time_ms));

	return 0;
}

static void wave600_cfg80211_end_cac(struct wiphy *wiphy,
				     struct net_device *dev,
				     unsigned int link_id)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	if (!radio || !radio->cac_in_progress)
		return;

	dev_info(radio->wdev->dev, "Radio %u end_cac (aborted)\n", radio->radio_idx);
	cancel_delayed_work_sync(&radio->cac_work);
	radio->cac_in_progress = false;

	cfg80211_cac_event(radio->ndev, &radio->cac_chandef,
			   NL80211_RADAR_CAC_ABORTED, GFP_KERNEL, 0);
}

static int wave600_cfg80211_get_channel(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					unsigned int link_id,
					struct cfg80211_chan_def *chandef)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	if (!radio || !radio->ap_started || !radio->chandef.chan)
		return -EOPNOTSUPP;

	*chandef = radio->chandef;
	return 0;
}

static struct wireless_dev *
wave600_cfg80211_add_virt_intf(struct wiphy *wiphy, const char *name,
			       unsigned char name_assign_type,
			       enum nl80211_iftype type,
			       struct vif_params *params)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: add_virtual_intf (%s, type %d)\n",
		 name, type);

	if (type != NL80211_IFTYPE_AP)
		return ERR_PTR(-EOPNOTSUPP);

	/* Static pre-created AP interfaces (wlan0, wlan1) */
	return ERR_PTR(-ENOSPC);
}

static int wave600_cfg80211_del_virt_intf(struct wiphy *wiphy, struct wireless_dev *wdev_net)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: del_virtual_intf (ndev %s)\n",
		 wdev_net->netdev ? wdev_net->netdev->name : "?");

	/* The pre-created AP interface stays alive with the driver */
	return -EBUSY;
}

static int wave600_cfg80211_auth(struct wiphy *wiphy, struct net_device *ndev,
				 struct cfg80211_auth_request *req)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: auth (ndev %s)\n", ndev->name);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_assoc(struct wiphy *wiphy, struct net_device *ndev,
				  struct cfg80211_assoc_request *req)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: assoc (ndev %s)\n", ndev->name);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_deauth(struct wiphy *wiphy, struct net_device *ndev,
				  struct cfg80211_deauth_request *req)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: deauth (ndev %s)\n", ndev->name);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_disassoc(struct wiphy *wiphy, struct net_device *ndev,
				     struct cfg80211_disassoc_request *req)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: disassoc (ndev %s)\n", ndev->name);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_connect(struct wiphy *wiphy, struct net_device *ndev,
				    struct cfg80211_connect_params *sme)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: connect (ndev %s)\n", ndev->name);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *ndev,
				       u16 reason_code)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: disconnect (ndev %s, reason %u)\n",
		 ndev->name, reason_code);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_change_virt_intf(struct wiphy *wiphy,
					     struct net_device *ndev,
					     enum nl80211_iftype type,
					     struct vif_params *params)
{
	dev_info(wiphy_dev(wiphy), "cfg80211 op: change_virtual_intf (ndev %s -> type %d)\n",
		 ndev->name, type);

	/* Support switching between AP and STATION (e.g. during netifd setup/teardown) */
	if (type == NL80211_IFTYPE_AP || type == NL80211_IFTYPE_STATION) {
		if (ndev->ieee80211_ptr)
			ndev->ieee80211_ptr->iftype = type;
		return 0;
	}

	dev_info(wiphy_dev(wiphy), "change_virtual_intf: type %d not supported\n",
		 type);
	return -EOPNOTSUPP;
}

static int wave600_cfg80211_mgmt_tx(struct wiphy *wiphy,
				    struct wireless_dev *wdev_net,
				    struct cfg80211_mgmt_tx_params *params,
				    u64 *cookie)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	int ret;

	if (!radio || !params || !params->buf || !params->len)
		return -EINVAL;

	*cookie = ++radio->mgmt_tx_cookie;

	if (!radio->ap_started) {
		cfg80211_mgmt_tx_status(wdev_net, *cookie, params->buf, params->len, false, GFP_KERNEL);
		return 0;
	}

	ret = wave600_dma_mgmt_tx(radio, params->buf, params->len);
	if (ret)
		return ret;

	cfg80211_mgmt_tx_status(wdev_net, *cookie, params->buf, params->len, true, GFP_KERNEL);
	return 0;
}

static int wave600_cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
						struct wireless_dev *wdev_net,
						u64 cookie)
{
	return 0;
}

static int wave600_cfg80211_add_station(struct wiphy *wiphy,
					struct net_device *ndev,
					const u8 *mac,
					struct station_parameters *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_request_sid sid_req;
	struct wave600_umi_request_sid sid_cfm;
	struct wave600_umi_sta_add req;
	struct wave600_umi_sta_add cfm;
	const u8 *rates = (radio->radio_idx == 1) ? wave600_rates_5g : wave600_rates_2g;
	u8 rates_len = (radio->radio_idx == 1) ? sizeof(wave600_rates_5g) : sizeof(wave600_rates_2g);
	struct wave600_station *sta;
	u16 sid;
	int ret;

	if (params && params->link_sta_params.supported_rates && params->link_sta_params.supported_rates_len) {
		rates = params->link_sta_params.supported_rates;
		rates_len = min_t(u8, params->link_sta_params.supported_rates_len, (u8)sizeof(req.rates));
	}

	if (!radio->ap_started)
		return -ENETDOWN;

	dev_dbg(wdev->dev, "Radio %u cfg80211: add station %pM (aid %u)\n",
		radio->radio_idx, mac, params ? params->aid : 0);

	/*
	 * If this MAC already has a stale entry (e.g. reconnection without prior deauth),
	 * perform complete 3-step teardown first so ARC state machine resets cleanly.
	 */
	{
		struct wave600_station *old_sta = wave600_find_sta(radio, mac);
		if (old_sta) {
			dev_dbg(wdev->dev, "Radio %u add_station: tearing down stale station %pM (sid %u)\n",
				radio->radio_idx, mac, old_sta->sid);
			wave600_teardown_station(radio, old_sta);
		}
	}

	/*
	 * Step 1: Request a Station ID (SID) from FullMAC firmware.
	 * Vendor core_config.c:1036-1080 core_cfg_internal_request_sid sends
	 * UM_MAN_REQUEST_SID_REQ (0x0442) so the ARC hardware receiver table
	 * registers and binds sAddr to an allocated SID before any data frames arrive.
	 */
	memset(&sid_req, 0, sizeof(sid_req));
	ether_addr_copy(sid_req.addr, mac);
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_REQUEST_SID_REQ,
				    WAVE600_MC_MAN_REQUEST_SID_CFM,
				    &sid_req, sizeof(sid_req),
				    &sid_cfm, sizeof(sid_cfm), 2000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u REQUEST_SID IPC failed: %d\n", radio->radio_idx, ret);
		return ret;
	}
	if (sid_cfm.status) {
		dev_warn(wdev->dev, "Radio %u REQUEST_SID rejected by FW: status=%u\n",
			 radio->radio_idx, sid_cfm.status);
		return -EIO;
	}

	sid = le16_to_cpu(sid_cfm.sid);
	dev_dbg(wdev->dev, "Radio %u REQUEST_SID %pM confirmed: sid=%u\n",
		radio->radio_idx, mac, sid);

	/*
	 * Step 2: Configure station capabilities via UM_MAN_STA_ADD_REQ (0x043B).
	 */
	memset(&req, 0, sizeof(req));
	req.sid = sid_cfm.sid;
	req.vap_id = radio->vap_id;
	req.aid = cpu_to_le16(params && params->aid ? params->aid : (sid + 1));
	ether_addr_copy(req.addr, mac);
	req.rates_len = rates_len;
	memcpy(req.rates, rates, rates_len);

	/*
	 * Flags:
	 * In open AP mode: STA_ADD_FLAGS_IS_8021X_FILTER_OPEN (0x04) | STA_ADD_FLAGS_WME (0x02) = 0x06.
	 * In WPA/WPA2/WPA3 mode: STA_ADD_FLAGS_WME (0x02) only. Port is closed until PTK handshake completes.
	 */
	req.flags = radio->is_secure ? W600_STA_FLAG_WME : (W600_STA_FLAG_WME | W600_STA_FLAG_8021X_OPEN);

	/* If client has MFP / PMF (802.11w / WPA3), set STA_ADD_FLAGS_MFP (0x08) */
	if (params && (params->sta_flags_set & BIT(NL80211_STA_FLAG_MFP)))
		req.flags |= W600_STA_FLAG_MFP;

	/* If client supports Operating Mode Notification (OMN), set STA_ADD_FLAGS_OMN (0x40) */
	if (params && params->ext_capab && params->ext_capab_len >= 8 &&
	    (params->ext_capab[7] & 0x40))
		req.flags |= W600_STA_FLAG_OMN;

	/* If client has HT capabilities, copy HT parameters and set STA_ADD_FLAGS_IS_HT (0x10) */
	if (params && params->link_sta_params.ht_capa) {
		const struct ieee80211_ht_cap *ht = params->link_sta_params.ht_capa;

		req.ht_cap_info = ht->cap_info;
		req.ampdu_param = ht->ampdu_params_info;
		req.tx_bf_caps = 0;
		memcpy(req.rx_mcs_bitmask, ht->mcs.rx_mask, sizeof(req.rx_mcs_bitmask));
		req.flags |= W600_STA_FLAG_HT;
	}

	/* If client has VHT capabilities, copy VHT parameters and set STA_ADD_FLAGS_IS_VHT (0x20) */
	if (params && params->link_sta_params.vht_capa) {
		const struct ieee80211_vht_cap *vht = params->link_sta_params.vht_capa;

		req.vht_cap_info = vht->vht_cap_info;
		memcpy(req.vht_mcs_nss, &vht->supp_mcs,
		       min_t(size_t, sizeof(req.vht_mcs_nss), sizeof(vht->supp_mcs)));
		req.flags |= W600_STA_FLAG_VHT;
	}

	/* If client has HE capabilities, copy HE parameters and set STA_ADD_FLAGS_EXT_IS_HE (0x02) */
	if (params && params->link_sta_params.he_capa && params->link_sta_params.he_capa_len) {
		const u8 *he_raw = (const u8 *)params->link_sta_params.he_capa;
		u8 he_len = params->link_sta_params.he_capa_len;
		bool has_160, has_ppe;
		size_t ppe_offset;

		/*
		 * In WAV654 ARC firmware UMI_STA_ADD (mhi_umi.h / wave_80211ax.h):
		 * u8HE_Mac_Phy_Cap_Info[18] layout:
		 * - [0]:      Extension Element ID (35 / WLAN_EID_EXT_HE_CAPABILITIES)
		 * - [1..6]:   6 bytes of client HE MAC Cap Info
		 * - [7..17]:  11 bytes of client HE PHY Cap Info
		 */
		req.he_mac_phy_cap_info[0] = 35;
		if (he_len >= 6)
			memcpy(&req.he_mac_phy_cap_info[1], he_raw, 6);
		if (he_len > 6) {
			size_t phy_len = min_t(size_t, 11, (size_t)(he_len - 6));
			memcpy(&req.he_mac_phy_cap_info[7], he_raw + 6, phy_len);
		}

		/*
		 * In 802.11ax, the Supported HE-MCS and NSS Set field contains:
		 * - <= 80 MHz MCS map (4 bytes): always present.
		 * - 160 MHz MCS map (4 bytes): present only if channel width set in PHY cap
		 *   indicates 160 MHz support (he_raw[6] & IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G).
		 * - PPE Thresholds (variable length): present only if PPE_THRESHOLD_PRESENT
		 *   (he_raw[12] & IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT) is set.
		 */
		has_160 = (he_len > 6) && !!(he_raw[6] & IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G);
		has_ppe = (he_len > 12) && !!(he_raw[12] & IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT);

		memset(req.he_mcs_nss, 0xff, sizeof(req.he_mcs_nss));
		if (he_len > 17)
			memcpy(&req.he_mcs_nss[0], he_raw + 17, min_t(size_t, 4, (size_t)(he_len - 17)));

		if (has_160 && he_len > 21) {
			memcpy(&req.he_mcs_nss[4], he_raw + 21, min_t(size_t, 4, (size_t)(he_len - 21)));
			ppe_offset = 25;
		} else {
			ppe_offset = 21;
		}

		memset(req.he_ppe_th, 0, sizeof(req.he_ppe_th));
		if (has_ppe) {
			req.he_mac_phy_cap_info[13] |= 0x80;
			if (he_len > ppe_offset) {
				size_t ppe_len = min_t(size_t, sizeof(req.he_ppe_th), (size_t)(he_len - ppe_offset));
				memcpy(req.he_ppe_th, he_raw + ppe_offset, ppe_len);
			}
		} else {
			req.he_mac_phy_cap_info[13] &= ~0x80;
		}

		req.flags_ext |= W600_STA_FLAG_EXT_HE;
	}

	dev_dbg(wdev->dev, "Radio %u STA_ADD %pM: sid %u aid %u flags 0x%02x flags_ext 0x%02x ht 0x%04x ampdu 0x%02x vht 0x%08x\n",
		radio->radio_idx, mac, le16_to_cpu(req.sid), le16_to_cpu(req.aid),
		req.flags, req.flags_ext, le16_to_cpu(req.ht_cap_info),
		req.ampdu_param, le32_to_cpu(req.vht_cap_info));

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_STA_ADD_REQ,
				    WAVE600_MC_MAN_STA_ADD_CFM,
				    &req, sizeof(req), &cfm, sizeof(cfm), 2000,
				    radio->vap_id);
	if (ret) {
		struct wave600_umi_remove_sid rem_req;
		struct wave600_umi_remove_sid rem_cfm;

		dev_warn(wdev->dev, "Radio %u STA_ADD IPC failed: %d\n", radio->radio_idx, ret);
		/* Cleanup allocated SID on error */
		memset(&rem_req, 0, sizeof(rem_req));
		rem_req.sid = sid_cfm.sid;
		wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				      WAVE600_UM_MAN_REMOVE_SID_REQ,
				      WAVE600_MC_MAN_REMOVE_SID_CFM,
				      &rem_req, sizeof(rem_req),
				      &rem_cfm, sizeof(rem_cfm), 2000,
				      radio->vap_id);
		return ret;
	}

	sta = wave600_add_sta_entry(radio, mac, sid);
	if (sta) {
		sta->is_he = !!(req.flags_ext & W600_STA_FLAG_EXT_HE);
		sta->is_vht = !!(req.flags & W600_STA_FLAG_VHT);
		sta->is_ht = !!(req.flags & W600_STA_FLAG_HT);
		if (radio->sta_signal) {
			sta->signal = radio->sta_signal;
			sta->signal_avg = radio->sta_signal;
		}
	}
	dev_dbg(wdev->dev, "Radio %u STA_ADD %pM confirmed: status=%u sid=%u (flags=0x%02x flags_ext=0x%02x HE=%d VHT=%d HT=%d)\n",
		radio->radio_idx, mac, cfm.status, sid, req.flags, req.flags_ext,
		sta ? sta->is_he : 0, sta ? sta->is_vht : 0, sta ? sta->is_ht : 0);

	return 0;
}

static int wave600_cfg80211_change_station(struct wiphy *wiphy,
					   struct net_device *ndev,
					   const u8 *mac,
					   struct station_parameters *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_station *sta = wave600_find_sta(radio, mac);
	int sid = sta ? (int)sta->sid : -ENOENT;

	dev_dbg(wdev->dev, "Radio %u cfg80211: change station %pM (aid %u, flags_set=0x%x sid=%d)\n",
		radio->radio_idx, mac, params ? params->aid : 0, params ? params->sta_flags_set : 0, sid);

	/*
	 * When station completes 4-way handshake, hostapd authorizes the station:
	 * Open the 802.1X filter via UM_MAN_SET_802_1X_FILTER_REQ (0x0446).
	 * Vendor core.c:9674-9690 _mtlk_send_filter_req.
	 */
	if (sid >= 0 && params && (params->sta_flags_set & BIT(NL80211_STA_FLAG_AUTHORIZED)) && radio->is_secure) {
		struct wave600_umi_802_1x_filter req;
		struct wave600_umi_802_1x_filter cfm;
		int ret;

		memset(&req, 0, sizeof(req));
		req.sid = cpu_to_le16((u16)sid);
		req.is_filter_open = 1;

		ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
					    WAVE600_UM_MAN_SET_802_1X_FILTER_REQ,
					    WAVE600_MC_MAN_SET_802_1X_FILTER_CFM,
					    &req, sizeof(req),
					    &cfm, sizeof(cfm), 2000,
					    radio->vap_id);
		if (ret)
			dev_warn(wdev->dev, "Radio %u SET_802_1X_FILTER IPC failed: %d\n",
				 radio->radio_idx, ret);
		else if (cfm.status)
			dev_warn(wdev->dev, "Radio %u SET_802_1X_FILTER rejected by FW: status=%u\n",
				 radio->radio_idx, le16_to_cpu(cfm.status));
		else {
			if (sta)
				sta->is_authorized = true;
			dev_dbg(wdev->dev, "Radio %u SET_802_1X_FILTER %pM (sid %u) OPENED\n",
				radio->radio_idx, mac, sid);
		}
	}

	return 0;
}

static int wave600_cfg80211_del_station(struct wiphy *wiphy,
					struct net_device *ndev,
					struct station_del_parameters *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_station *sta = NULL;
	int i;

	if (!radio->ap_started || atomic_read(&wdev->in_recovery) ||
	    wdev->state != W600_STATE_RUNNING) {
		if (params && params->mac)
			wave600_del_sta_entry(radio, params->mac);
		return 0;
	}

	if (params && params->mac) {
		sta = wave600_find_sta(radio, params->mac);
		if (!sta)
			return 0; /* Station already removed - nothing to do */

		wave600_teardown_station(radio, sta);
		return 0;
	}

	/* Flush all stations on this radio */
	for (i = 0; i < WAVE600_MAX_STATIONS; i++) {
		if (radio->stations[i].valid)
			wave600_teardown_station(radio, &radio->stations[i]);
	}

	return 0;
}

static int wave600_cfg80211_get_station(struct wiphy *wiphy,
					struct net_device *ndev,
					const u8 *mac,
					struct station_info *sinfo)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_station *sta;

	if (!radio)
		return -ENOENT;

	sta = wave600_find_sta(radio, mac);
	if (!sta)
		return -ENOENT;

	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_INACTIVE_TIME) |
			 BIT_ULL(NL80211_STA_INFO_CONNECTED_TIME) |
			 BIT_ULL(NL80211_STA_INFO_RX_BYTES) |
			 BIT_ULL(NL80211_STA_INFO_TX_BYTES) |
			 BIT_ULL(NL80211_STA_INFO_RX_PACKETS) |
			 BIT_ULL(NL80211_STA_INFO_TX_PACKETS);

	sinfo->inactive_time = jiffies_to_msecs(jiffies - sta->last_rx);
	sinfo->connected_time = ktime_get_seconds() - sta->connect_time;
	if (sta->signal < 0 && sta->signal >= -100) {
		sinfo->signal = sta->signal;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_SIGNAL);
	}
	if (sta->signal_avg < 0 && sta->signal_avg >= -100) {
		sinfo->signal_avg = sta->signal_avg;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_SIGNAL_AVG);
	}
	sinfo->rx_bytes = sta->rx_bytes;
	sinfo->tx_bytes = sta->tx_bytes;
	sinfo->rx_packets = sta->rx_packets;
	sinfo->tx_packets = sta->tx_packets;

	/* Live TX Rate from firmware telemetry */
	if (sta->tx_bitrate > 0) {
		sinfo->txrate.legacy = sta->tx_bitrate;
		sinfo->txrate.bw = sta->tx_bw;
		sinfo->txrate.nss = sta->tx_nss;
		sinfo->txrate.mcs = sta->tx_mcs;
		sinfo->txrate.flags = sta->tx_flags;
		sinfo->txrate.he_gi = sta->tx_gi;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
	}

	/* Live RX Rate from firmware telemetry */
	if (sta->rx_bitrate > 0) {
		sinfo->rxrate.legacy = sta->rx_bitrate;
		sinfo->rxrate.bw = sta->rx_bw;
		sinfo->rxrate.nss = sta->rx_nss;
		sinfo->rxrate.mcs = sta->rx_mcs;
		sinfo->rxrate.flags = sta->rx_flags;
		sinfo->rxrate.he_gi = sta->rx_gi;
		sinfo->rxrate.he_dcm = sta->rx_dcm;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_RX_BITRATE);
	}

	return 0;
}

static int wave600_cfg80211_dump_station(struct wiphy *wiphy,
					 struct net_device *ndev,
					 int idx,
					 u8 *mac,
					 struct station_info *sinfo)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	int i, count = 0;

	if (!radio)
		return -ENOENT;

	for (i = 0; i < WAVE600_MAX_STATIONS; i++) {
		if (radio->stations[i].valid) {
			if (count == idx) {
				ether_addr_copy(mac, radio->stations[i].mac);
				return wave600_cfg80211_get_station(wiphy, ndev, mac, sinfo);
			}
			count++;
		}
	}

	return -ENOENT;
}

static int wave600_cfg80211_add_key(struct wiphy *wiphy,
				    struct net_device *ndev,
				    int link_id,
				    u8 key_index,
				    bool pairwise,
				    const u8 *mac_addr,
				    struct key_params *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_set_key req;
	struct wave600_umi_set_key cfm;
	u16 u16_cipher;
	u16 sid;
	u16 key_type;
	int ret;

	if (!params || !params->key || params->key_len == 0)
		return -EINVAL;

	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_CCMP:
		u16_cipher = WAVE600_UMI_CIPHER_SUITE_CCMP;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		u16_cipher = WAVE600_UMI_CIPHER_SUITE_TKIP;
		break;
	case WLAN_CIPHER_SUITE_GCMP:
		u16_cipher = WAVE600_UMI_CIPHER_SUITE_GCMP128;
		break;
	case WLAN_CIPHER_SUITE_GCMP_256:
		u16_cipher = WAVE600_UMI_CIPHER_SUITE_GCMP256;
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		u16_cipher = WAVE600_UMI_CIPHER_SUITE_BIP;
		break;
	default:
		dev_warn(wdev->dev, "Radio %u add_key: unsupported cipher 0x%08x\n",
			 radio->radio_idx, params->cipher);
		return -EOPNOTSUPP;
	}

	if (params->cipher == WLAN_CIPHER_SUITE_AES_CMAC) {
		sid = WAVE600_BCAST_DEST_SID;
		key_type = WAVE600_UMI_RSN_MGMT_GROUP_KEY; /* 2 = IGTK (BIP) */
		dev_dbg(wdev->dev, "Radio %u add_key (IGTK/BIP): bcast sid=0x%04x key_idx=%u len=%u\n",
			radio->radio_idx, sid, key_index, params->key_len);
	} else if (pairwise && mac_addr) {
		int sta_sid = wave600_radio_find_sid(radio, mac_addr);
		if (sta_sid < 0) {
			dev_warn(wdev->dev, "Radio %u add_key: station %pM not found\n",
				 radio->radio_idx, mac_addr);
			return -ENOENT;
		}
		sid = (u16)sta_sid;
		key_type = WAVE600_UMI_RSN_PAIRWISE_KEY;
		dev_dbg(wdev->dev, "Radio %u add_key (PAIRWISE): STA %pM sid=%u key_idx=%u cipher=0x%04x len=%u\n",
			radio->radio_idx, mac_addr, sid, key_index, u16_cipher, params->key_len);
	} else {
		sid = WAVE600_BCAST_DEST_SID;
		key_type = WAVE600_UMI_RSN_GROUP_KEY;
		radio->is_secure = true;
		dev_dbg(wdev->dev, "Radio %u add_key (GROUP): bcast sid=0x%04x key_idx=%u cipher=0x%04x len=%u\n",
			radio->radio_idx, sid, key_index, u16_cipher, params->key_len);
	}

	memset(&req, 0, sizeof(req));
	req.sid = cpu_to_le16(sid);
	req.key_type = cpu_to_le16(key_type);
	req.cipher_suite = cpu_to_le16(u16_cipher);
	req.key_index = cpu_to_le16(key_index);

	if (params->seq && params->seq_len > 0) {
		size_t seq_len = min_t(size_t, params->seq_len, sizeof(req.rx_seq_num));
		memcpy(req.rx_seq_num, params->seq, seq_len);
	}

	req.tx_seq_num[0] = 1;

	memcpy(req.tk, params->key, min_t(size_t, params->key_len, sizeof(req.tk)));

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_KEY_REQ,
				    WAVE600_MC_MAN_SET_KEY_CFM,
				    &req, sizeof(req),
				    &cfm, sizeof(cfm), 2000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u SET_KEY IPC failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	if (cfm.status) {
		dev_warn(wdev->dev, "Radio %u SET_KEY rejected by FW: status=%u\n",
			 radio->radio_idx, le16_to_cpu(cfm.status));
		return -EIO;
	}

	dev_dbg(wdev->dev, "Radio %u SET_KEY confirmed: sid=0x%04x key_type=%u key_idx=%u\n",
		radio->radio_idx, sid, key_type, key_index);

	return 0;
}

static int wave600_cfg80211_del_key(struct wiphy *wiphy,
				    struct net_device *ndev,
				    int link_id,
				    u8 key_index,
				    bool pairwise,
				    const u8 *mac_addr)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	dev_dbg(radio->wdev->dev, "Radio %u del_key: key_idx=%u pairwise=%d\n",
		radio->radio_idx, key_index, pairwise);
	return 0;
}

static int wave600_cfg80211_set_default_key(struct wiphy *wiphy,
					    struct net_device *ndev,
					    int link_id,
					    u8 key_index,
					    bool unicast,
					    bool multicast)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_default_key_index req;
	struct wave600_umi_default_key_index cfm;
	int ret;

	dev_dbg(wdev->dev, "Radio %u set_default_key: key_idx=%u unicast=%d multicast=%d\n",
		radio->radio_idx, key_index, unicast, multicast);

	memset(&req, 0, sizeof(req));
	req.sid = cpu_to_le16(WAVE600_BCAST_DEST_SID);
	req.key_type = cpu_to_le16(WAVE600_UMI_RSN_GROUP_KEY);
	req.key_index = cpu_to_le16(key_index);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_DEFAULT_KEY_INDEX_REQ,
				    WAVE600_MC_MAN_SET_DEFAULT_KEY_INDEX_CFM,
				    &req, sizeof(req),
				    &cfm, sizeof(cfm), 2000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev, "Radio %u SET_DEFAULT_KEY IPC failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	return 0;
}

static int wave600_cfg80211_get_txpower(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					int *dbm)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	if (radio->tx_power_dbm > 0)
		*dbm = radio->tx_power_dbm;
	else
		*dbm = (radio->radio_idx == 0) ? 20 : 23;

	return 0;
}

static int wave600_cfg80211_set_txpower(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					enum nl80211_tx_power_setting type,
					int mbm)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	int max_dbm = (radio->radio_idx == 0) ? 20 : 23;

	if (type == NL80211_TX_POWER_AUTOMATIC) {
		radio->tx_power_dbm = max_dbm;
	} else if (type == NL80211_TX_POWER_LIMITED || type == NL80211_TX_POWER_FIXED) {
		radio->tx_power_dbm = clamp(mbm / 100, 1, max_dbm);
	}

	if (radio->ap_started) {
		/*
		 * Live path: SET_POWER_LIMIT (0x045F) with offset in dBm —
		 * the vendor way to change TX power on an active radio.
		 * SET_TPC_CONFIG on a beaconing radio crashes the LM core,
		 * so the full TPC reload with the new clamp is deferred to
		 * the next start_ap.
		 */
		u8 off = (radio->tx_power_dbm > 0) ?
			 (u8)(max_dbm - radio->tx_power_dbm) : 0;

		wave600_send_power_limit_offset_radio(radio, off);
	}

	return 0;
}

static const struct ieee80211_txrx_stypes wave600_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_AP] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		      BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		      BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		      BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		      BIT(IEEE80211_STYPE_AUTH >> 4) |
		      BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		      BIT(IEEE80211_STYPE_ACTION >> 4),
	},
};

static const struct cfg80211_ops wave600_cfg80211_ops = {
	.add_virtual_intf = wave600_cfg80211_add_virt_intf,
	.del_virtual_intf = wave600_cfg80211_del_virt_intf,
	.change_virtual_intf = wave600_cfg80211_change_virt_intf,
	.start_ap = wave600_cfg80211_start_ap,
	.stop_ap = wave600_cfg80211_stop_ap,
	.get_channel = wave600_cfg80211_get_channel,
	.change_beacon = wave600_cfg80211_change_beacon,
	.set_monitor_channel = wave600_cfg80211_set_channel,
	.channel_switch = wave600_cfg80211_channel_switch,
	.start_radar_detection = wave600_cfg80211_start_radar_detection,
	.end_cac = wave600_cfg80211_end_cac,
	.mgmt_tx = wave600_cfg80211_mgmt_tx,
	.mgmt_tx_cancel_wait = wave600_cfg80211_mgmt_tx_cancel_wait,
	.add_station = wave600_cfg80211_add_station,
	.del_station = wave600_cfg80211_del_station,
	.change_station = wave600_cfg80211_change_station,
	.get_station = wave600_cfg80211_get_station,
	.dump_station = wave600_cfg80211_dump_station,
	.add_key = wave600_cfg80211_add_key,
	.del_key = wave600_cfg80211_del_key,
	.set_default_key = wave600_cfg80211_set_default_key,
	.auth = wave600_cfg80211_auth,
	.assoc = wave600_cfg80211_assoc,
	.deauth = wave600_cfg80211_deauth,
	.disassoc = wave600_cfg80211_disassoc,
	.connect = wave600_cfg80211_connect,
	.disconnect = wave600_cfg80211_disconnect,
	.get_tx_power = wave600_cfg80211_get_txpower,
	.set_tx_power = wave600_cfg80211_set_txpower,
};

/* ------------------------------------------------------------------ */
/* Wiphy registration & 802.11ax (HE) Capabilities                    */
/* ------------------------------------------------------------------ */

static const struct ieee80211_sband_iftype_data wave600_iftype_data_2g[] = {
	{
		.types_mask = BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_STATION),
		.he_cap = {
			.has_he = true,
			.he_cap_elem = {
				.mac_cap_info = {
					[0] = IEEE80211_HE_MAC_CAP0_HTC_HE,
					[1] = IEEE80211_HE_MAC_CAP1_MULTI_TID_AGG_RX_QOS_8,
					[2] = IEEE80211_HE_MAC_CAP2_ALL_ACK |
					      IEEE80211_HE_MAC_CAP2_BSR |
					      IEEE80211_HE_MAC_CAP2_ACK_EN,
					[3] = IEEE80211_HE_MAC_CAP3_OMI_CONTROL |
					      IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_EXT_2,
					[4] = IEEE80211_HE_MAC_CAP4_AMSDU_IN_AMPDU |
					      IEEE80211_HE_MAC_CAP4_BQR |
					      IEEE80211_HE_MAC_CAP4_MULTI_TID_AGG_TX_QOS_B39,
					[5] = IEEE80211_HE_MAC_CAP5_MULTI_TID_AGG_TX_QOS_B40 |
					      IEEE80211_HE_MAC_CAP5_OM_CTRL_UL_MU_DATA_DIS_RX,
				},
				.phy_cap_info = {
					[0] = IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_IN_2G,
					[1] = IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD |
					      IEEE80211_HE_PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US,
					[2] = IEEE80211_HE_PHY_CAP2_STBC_TX_UNDER_80MHZ |
					      IEEE80211_HE_PHY_CAP2_STBC_RX_UNDER_80MHZ,
					[6] = IEEE80211_HE_PHY_CAP6_PARTIAL_BW_EXT_RANGE,
				},
			},
			.he_mcs_nss_supp = {
				.rx_mcs_80 = cpu_to_le16(0xfffa),
				.tx_mcs_80 = cpu_to_le16(0xfffa),
				.rx_mcs_160 = cpu_to_le16(0xffff),
				.tx_mcs_160 = cpu_to_le16(0xffff),
				.rx_mcs_80p80 = cpu_to_le16(0xffff),
				.tx_mcs_80p80 = cpu_to_le16(0xffff),
			},
		},
	},
};

static const struct ieee80211_sband_iftype_data wave600_iftype_data_5g[] = {
	{
		.types_mask = BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_STATION),
		.he_cap = {
			.has_he = true,
			.he_cap_elem = {
				.mac_cap_info = {
					[0] = IEEE80211_HE_MAC_CAP0_HTC_HE,
					[1] = IEEE80211_HE_MAC_CAP1_MULTI_TID_AGG_RX_QOS_8,
					[2] = IEEE80211_HE_MAC_CAP2_ALL_ACK |
					      IEEE80211_HE_MAC_CAP2_BSR |
					      IEEE80211_HE_MAC_CAP2_ACK_EN,
					[3] = IEEE80211_HE_MAC_CAP3_OMI_CONTROL |
					      IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_EXT_2,
					[4] = IEEE80211_HE_MAC_CAP4_AMSDU_IN_AMPDU |
					      IEEE80211_HE_MAC_CAP4_BQR |
					      IEEE80211_HE_MAC_CAP4_MULTI_TID_AGG_TX_QOS_B39,
					[5] = IEEE80211_HE_MAC_CAP5_MULTI_TID_AGG_TX_QOS_B40 |
					      IEEE80211_HE_MAC_CAP5_OM_CTRL_UL_MU_DATA_DIS_RX,
				},
				.phy_cap_info = {
					[0] = IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_80MHZ_IN_5G |
					      IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G,
					[1] = IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD |
					      IEEE80211_HE_PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US,
					[2] = IEEE80211_HE_PHY_CAP2_STBC_TX_UNDER_80MHZ |
					      IEEE80211_HE_PHY_CAP2_STBC_RX_UNDER_80MHZ,
					[6] = IEEE80211_HE_PHY_CAP6_PARTIAL_BW_EXT_RANGE,
				},
			},
			.he_mcs_nss_supp = {
				.rx_mcs_80 = cpu_to_le16(0xfffa),
				.tx_mcs_80 = cpu_to_le16(0xfffa),
				.rx_mcs_160 = cpu_to_le16(0xfffa),
				.tx_mcs_160 = cpu_to_le16(0xfffa),
				.rx_mcs_80p80 = cpu_to_le16(0xffff),
				.tx_mcs_80p80 = cpu_to_le16(0xffff),
			},
		},
	},
};

static void wave600_setup_band_2g(struct ieee80211_supported_band *band)
{
	static const struct ieee80211_channel channels_2g[] = {
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12, .max_power = 20, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13, .max_power = 20, .max_antenna_gain = 0 },
	};

	memcpy(band->channels, channels_2g, sizeof(channels_2g));
	band->n_channels = ARRAY_SIZE(channels_2g);
}

static void wave600_setup_band_5g(struct ieee80211_supported_band *band)
{
	static const struct ieee80211_channel channels_5g[] = {
		/* UNII-1: Non-DFS indoor channels */
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5180, .hw_value = 36, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5200, .hw_value = 40, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5220, .hw_value = 44, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5240, .hw_value = 48, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },

		/* UNII-2A: DFS channels (Radar detection required) */
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5260, .hw_value = 52, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5280, .hw_value = 56, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5300, .hw_value = 60, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5320, .hw_value = 64, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },

		/* UNII-2C: DFS channels (Radar detection required) */
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5500, .hw_value = 100, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5520, .hw_value = 104, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5540, .hw_value = 108, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5560, .hw_value = 112, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5580, .hw_value = 116, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5600, .hw_value = 120, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5620, .hw_value = 124, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5640, .hw_value = 128, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5660, .hw_value = 132, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5680, .hw_value = 136, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5700, .hw_value = 140, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5720, .hw_value = 144, .max_power = 23, .max_antenna_gain = 0, .flags = IEEE80211_CHAN_RADAR },

		/* UNII-3: Non-DFS channels */
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5745, .hw_value = 149, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5765, .hw_value = 153, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5785, .hw_value = 157, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5805, .hw_value = 161, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5825, .hw_value = 165, .max_power = 23, .max_antenna_gain = 0, .flags = 0 },
	};

	memcpy(band->channels, channels_5g, sizeof(channels_5g));
	band->n_channels = ARRAY_SIZE(channels_5g);
}

/* ------------------------------------------------------------------ */
/* Handler-liveness diagnostic (one-shot, end of probe)               */
/* ------------------------------------------------------------------ */

void wave600_diag_handlers_probe(struct wave600_dev *wdev)
{
	struct wave600_mac_version ver;
	u8 req[sizeof(ver)];
	int ret;
	int i;
	bool nonzero = false;

	/* Positive control: GET_MAC_VERSION (read-only) */
	memset(req, 0, sizeof(req));
	memset(&ver, 0, sizeof(ver));
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_GET_MAC_VERSION_REQ,
			      WAVE600_MC_MAN_GET_MAC_VERSION_CFM,
			      req, sizeof(req), &ver, sizeof(ver), 5000);
	if (ret) {
		dev_info(wdev->dev, "diag: GET_MAC_VERSION send failed: %d\n", ret);
	} else {
		for (i = 0; i < sizeof(ver.ver); i++) {
			if (ver.ver[i])
				nonzero = true;
		}
		dev_info(wdev->dev, "diag: GET_MAC_VERSION cfm: length=%u head=%16ph\n",
			 ver.length, &ver);
		if (nonzero) {
			dev_info(wdev->dev, "diag: GET_MAC_VERSION returned real FW content: '%.*s'\n",
				 (int)sizeof(ver.ver), ver.ver);
			dev_info(wdev->dev, "diag: VERDICT: handler set is ALLOCATED (FW computes response content)\n");
		} else {
			dev_info(wdev->dev, "diag: GET_MAC_VERSION cfm payload is ALL ZERO\n");
			dev_info(wdev->dev, "diag: VERDICT: pseudo-ACK suspected (no handler content in cfm)\n");
		}
	}
}

/* ------------------------------------------------------------------ */
/* DBDC Radio Registration                                            */
/* ------------------------------------------------------------------ */

static const u8 wlan_macs[2][ETH_ALEN] = {
	{ 0x02, 0x50, 0x43, 0x00, 0x00, 0x01 }, /* wlan0: 2.4 GHz */
	{ 0x02, 0x50, 0x43, 0x00, 0x00, 0x02 }, /* wlan1: 5 GHz */
};

static const u32 wave600_cipher_suites[] = {
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_GCMP,
	WLAN_CIPHER_SUITE_GCMP_256,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

static const u8 wave600_ext_capa_ap[] = {
	[0] = WLAN_EXT_CAPA1_EXT_CHANNEL_SWITCHING,
	[7] = WLAN_EXT_CAPA8_OPMODE_NOTIF,
	[9] = WLAN_EXT_CAPA10_OBSS_NARROW_BW_RU_TOLERANCE_SUPPORT,
};

static const struct ieee80211_iface_limit wave600_if_limits[] = {
	{
		.max = 8,
		.types = BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_STATION),
	},
};

static const struct ieee80211_iface_combination wave600_if_comb_2g[] = {
	{
		.limits = wave600_if_limits,
		.n_limits = ARRAY_SIZE(wave600_if_limits),
		.max_interfaces = 8,
		.num_different_channels = 1,
	},
};

static const struct ieee80211_iface_combination wave600_if_comb_5g[] = {
	{
		.limits = wave600_if_limits,
		.n_limits = ARRAY_SIZE(wave600_if_limits),
		.max_interfaces = 8,
		.num_different_channels = 1,
		.radar_detect_widths = BIT(NL80211_CHAN_WIDTH_20_NOHT) |
				       BIT(NL80211_CHAN_WIDTH_20) |
				       BIT(NL80211_CHAN_WIDTH_40) |
				       BIT(NL80211_CHAN_WIDTH_80) |
				       BIT(NL80211_CHAN_WIDTH_160),
	},
};

static int wave600_radio_init(struct wave600_dev *wdev, u8 radio_idx)
{
	struct wave600_radio *radio = &wdev->radios[radio_idx];
	struct wiphy *wiphy;
	struct ieee80211_supported_band *band;
	struct wireless_dev *wdev_net;
	int ret;

	radio->wdev = wdev;
	radio->radio_idx = radio_idx;
	radio->ep = (radio_idx == 0) ? 1 : 0;
	radio->vap_id = 0; /* primary AP VAP index is 0 on both radios */
	radio->antenna_mask = (radio_idx == 0) ? 0x05 : 0x0A;
	INIT_DELAYED_WORK(&radio->cac_work, wave600_cac_work);

	wiphy = wiphy_new(&wave600_cfg80211_ops, sizeof(struct wave600_radio *));
	if (!wiphy)
		return -ENOMEM;

	set_wiphy_dev(wiphy, wdev->dev);

	/* priv points back at radio */
	*(struct wave600_radio **)wiphy_priv(wiphy) = radio;
	radio->wiphy = wiphy;

	wiphy->interface_modes = BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_STATION);
	wiphy->flags |= WIPHY_FLAG_REPORTS_OBSS;
	wiphy->features |= NL80211_FEATURE_AP_MODE_CHAN_WIDTH_CHANGE | NL80211_FEATURE_SAE;
	wiphy->extended_capabilities = wave600_ext_capa_ap;
	wiphy->extended_capabilities_mask = wave600_ext_capa_ap;
	wiphy->extended_capabilities_len = sizeof(wave600_ext_capa_ap);
	wiphy->cipher_suites = wave600_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(wave600_cipher_suites);
	wiphy->mgmt_stypes = wave600_mgmt_stypes;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wiphy->reg_notifier = wave600_reg_notifier;
	wiphy->max_scan_ssids = 1;
	strscpy(wiphy->fw_version, wdev->fw_version, sizeof(wiphy->fw_version));

	if (radio_idx == 0) {
		wiphy->iface_combinations = wave600_if_comb_2g;
		wiphy->n_iface_combinations = ARRAY_SIZE(wave600_if_comb_2g);

		/* 2.4 GHz Band Only */
		wiphy->bands[NL80211_BAND_2GHZ] =
			kzalloc(sizeof(*wiphy->bands[NL80211_BAND_2GHZ]), GFP_KERNEL);
		if (!wiphy->bands[NL80211_BAND_2GHZ]) {
			ret = -ENOMEM;
			goto err_wiphy;
		}

		band = wiphy->bands[NL80211_BAND_2GHZ];
		band->channels = kcalloc(13, sizeof(struct ieee80211_channel), GFP_KERNEL);
		band->bitrates = kcalloc(12, sizeof(struct ieee80211_rate), GFP_KERNEL);
		if (!band->channels || !band->bitrates) {
			ret = -ENOMEM;
			goto err_band;
		}
		wave600_setup_band_2g(band);

		{
			static const int rates[] = { 10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540 };
			static const u32 flags[] = {
				IEEE80211_RATE_SHORT_PREAMBLE | IEEE80211_RATE_MANDATORY_B,
				IEEE80211_RATE_SHORT_PREAMBLE | IEEE80211_RATE_MANDATORY_B,
				IEEE80211_RATE_SHORT_PREAMBLE | IEEE80211_RATE_MANDATORY_B,
				IEEE80211_RATE_SHORT_PREAMBLE | IEEE80211_RATE_MANDATORY_B,
				0, 0, 0, 0, 0, 0, 0, 0
			};
			int i;

			for (i = 0; i < 12; i++) {
				band->bitrates[i].bitrate = rates[i];
				band->bitrates[i].flags = flags[i];
				band->bitrates[i].hw_value = i;
			}
			band->n_bitrates = 12;
		}

		band->ht_cap.ht_supported = true;
		band->ht_cap.cap = IEEE80211_HT_CAP_SGI_20 | IEEE80211_HT_CAP_SGI_40 |
				   IEEE80211_HT_CAP_SUP_WIDTH_20_40 | BIT(8) |
				   IEEE80211_HT_CAP_LDPC_CODING |
				   IEEE80211_HT_CAP_MAX_AMSDU;
		band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
		band->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;
		memset(&band->ht_cap.mcs, 0, sizeof(band->ht_cap.mcs));
		band->ht_cap.mcs.rx_mask[0] = 0xff; /* MCS 0-7 */
		band->ht_cap.mcs.rx_mask[1] = 0xff; /* MCS 8-15 (2x2 MIMO) */

		ieee80211_set_sband_iftype_data(band, wave600_iftype_data_2g);
	} else {
		wiphy->iface_combinations = wave600_if_comb_5g;
		wiphy->n_iface_combinations = ARRAY_SIZE(wave600_if_comb_5g);

		/* 5 GHz Band Only */
		wiphy->bands[NL80211_BAND_5GHZ] =
			kzalloc(sizeof(*wiphy->bands[NL80211_BAND_5GHZ]), GFP_KERNEL);
		if (!wiphy->bands[NL80211_BAND_5GHZ]) {
			ret = -ENOMEM;
			goto err_wiphy;
		}

		band = wiphy->bands[NL80211_BAND_5GHZ];
		band->channels = kcalloc(25, sizeof(struct ieee80211_channel), GFP_KERNEL);
		band->bitrates = kcalloc(8, sizeof(struct ieee80211_rate), GFP_KERNEL);
		if (!band->channels || !band->bitrates) {
			ret = -ENOMEM;
			goto err_band;
		}
		wave600_setup_band_5g(band);

		{
			static const int rates_5g[] = { 60, 90, 120, 180, 240, 360, 480, 540 };
			static const u32 flags_5g[] = {
				IEEE80211_RATE_MANDATORY_A,
				0,
				IEEE80211_RATE_MANDATORY_A,
				0,
				IEEE80211_RATE_MANDATORY_A,
				0, 0, 0
			};
			int i;

			for (i = 0; i < 8; i++) {
				band->bitrates[i].bitrate = rates_5g[i];
				band->bitrates[i].flags = flags_5g[i];
				band->bitrates[i].hw_value = i;
			}
			band->n_bitrates = 8;
		}

		band->ht_cap.ht_supported = true;
		band->ht_cap.cap = IEEE80211_HT_CAP_SGI_20 | IEEE80211_HT_CAP_SGI_40 |
				   IEEE80211_HT_CAP_SUP_WIDTH_20_40 | BIT(8) |
				   IEEE80211_HT_CAP_LDPC_CODING |
				   IEEE80211_HT_CAP_MAX_AMSDU;
		band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
		band->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;
		memset(&band->ht_cap.mcs, 0, sizeof(band->ht_cap.mcs));
		band->ht_cap.mcs.rx_mask[0] = 0xff; /* MCS 0-7 */
		band->ht_cap.mcs.rx_mask[1] = 0xff; /* MCS 8-15 (2x2 MIMO) */

		band->vht_cap.vht_supported = true;
		band->vht_cap.cap = IEEE80211_VHT_CAP_SHORT_GI_80 |
				    IEEE80211_VHT_CAP_SHORT_GI_160 |
				    IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ |
				    IEEE80211_VHT_CAP_RXSTBC_1 |
				    IEEE80211_VHT_CAP_RXLDPC |
				    IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454 |
				    IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK;
		band->vht_cap.vht_mcs.rx_mcs_map = cpu_to_le16(0xfffa);
		band->vht_cap.vht_mcs.tx_mcs_map = cpu_to_le16(0xfffa);

		ieee80211_set_sband_iftype_data(band, wave600_iftype_data_5g);
	}

	radio->ndev = alloc_netdev(sizeof(struct wireless_dev),
				   (radio_idx == 0) ? "wlan0" : "wlan1",
				   NET_NAME_UNKNOWN, ether_setup);
	if (!radio->ndev) {
		ret = -ENOMEM;
		goto err_band;
	}
	SET_NETDEV_DEV(radio->ndev, wdev->dev);
	wdev_net = netdev_priv(radio->ndev);
	wdev_net->iftype = NL80211_IFTYPE_AP;
	wdev_net->wiphy = wiphy;
	wdev_net->netdev = radio->ndev;
	radio->ndev->ieee80211_ptr = wdev_net;
	eth_hw_addr_set(radio->ndev, wlan_macs[radio_idx]);
	radio->ndev->netdev_ops = &wave600_netdev_ops;

	ret = wiphy_register(wiphy);
	if (ret)
		goto err_ndev;

	ret = register_netdev(radio->ndev);
	if (ret)
		goto err_wiphy_reg;

	radio->wdev_net = wdev_net;
	dev_info(wdev->dev, "cfg80211 wiphy registered for Radio %u (%s, AP %s, MAC %pM)\n",
		 radio_idx, (radio_idx == 0) ? "2.4GHz" : "5GHz",
		 radio->ndev->name, radio->ndev->dev_addr);
	return 0;

err_wiphy_reg:
	wiphy_unregister(wiphy);
err_ndev:
	free_netdev(radio->ndev);
	radio->ndev = NULL;
err_band:
	if (wiphy->bands[NL80211_BAND_2GHZ]) {
		kfree(wiphy->bands[NL80211_BAND_2GHZ]->bitrates);
		kfree(wiphy->bands[NL80211_BAND_2GHZ]->channels);
		kfree(wiphy->bands[NL80211_BAND_2GHZ]);
		wiphy->bands[NL80211_BAND_2GHZ] = NULL;
	}
	if (wiphy->bands[NL80211_BAND_5GHZ]) {
		kfree(wiphy->bands[NL80211_BAND_5GHZ]->bitrates);
		kfree(wiphy->bands[NL80211_BAND_5GHZ]->channels);
		kfree(wiphy->bands[NL80211_BAND_5GHZ]);
		wiphy->bands[NL80211_BAND_5GHZ] = NULL;
	}
err_wiphy:
	wiphy_free(wiphy);
	radio->wiphy = NULL;
	return ret;
}

static void wave600_radio_cleanup(struct wave600_radio *radio)
{
	struct wave600_dev *wdev = radio->wdev;
	struct ieee80211_supported_band *band;

	cancel_delayed_work_sync(&radio->cac_work);
	radio->cac_in_progress = false;

	if (radio->beacon_buf) {
		dma_unmap_single(wdev->dev, radio->beacon_dma, radio->beacon_len,
				 DMA_TO_DEVICE);
		kfree(radio->beacon_buf);
		radio->beacon_buf = NULL;
		radio->beacon_dma = 0;
		radio->beacon_len = 0;
	}

	if (radio->ndev) {
		unregister_netdev(radio->ndev);
		free_netdev(radio->ndev);
		radio->ndev = NULL;
	}

	if (!radio->wiphy)
		return;

	wiphy_unregister(radio->wiphy);

	band = radio->wiphy->bands[NL80211_BAND_5GHZ];
	if (band) {
		kfree(band->bitrates);
		kfree(band->channels);
		kfree(band);
		radio->wiphy->bands[NL80211_BAND_5GHZ] = NULL;
	}

	band = radio->wiphy->bands[NL80211_BAND_2GHZ];
	if (band) {
		kfree(band->bitrates);
		kfree(band->channels);
		kfree(band);
		radio->wiphy->bands[NL80211_BAND_2GHZ] = NULL;
	}

	wiphy_free(radio->wiphy);
	radio->wiphy = NULL;
}

int wave600_cfg80211_init(struct wave600_dev *wdev)
{
	int ret;

	ret = wave600_radio_init(wdev, 0);
	if (ret) {
		dev_err(wdev->dev, "failed to initialize Radio 0 (2.4 GHz): %d\n", ret);
		return ret;
	}

	ret = wave600_radio_init(wdev, 1);
	if (ret) {
		dev_err(wdev->dev, "failed to initialize Radio 1 (5 GHz): %d\n", ret);
		wave600_radio_cleanup(&wdev->radios[0]);
		return ret;
	}

	dev_info(wdev->dev, "DBDC Wi-Fi ready: wlan0 (2.4 GHz) and wlan1 (5 GHz) registered\n");

	/* Initial default country: PL / ETSI (AX50 EU retail hardware) */
	wave600_set_country(wdev, "PL");

	return 0;
}

void wave600_cfg80211_cleanup(struct wave600_dev *wdev)
{
	wave600_radio_cleanup(&wdev->radios[1]);
	wave600_radio_cleanup(&wdev->radios[0]);
}
