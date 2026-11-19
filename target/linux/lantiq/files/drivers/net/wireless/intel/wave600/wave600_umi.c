// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) UMI / MIB / Regulatory / TPC Helpers
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <net/cfg80211.h>
#include "wave600.h"
#include "wave600_ipc.h"
#include "wave600_cal.h"
#include "wave600_umi.h"

/* ------------------------------------------------------------------ */
/* UMI message payloads (mhi_umi.h layouts). Multi-byte request       */
/* fields are MAC-endian (little-endian), matching the vendor          */
/* HOST_TO_MAC32/16 conversions; FW-written SHRAM data is read raw BE.  */
/* ------------------------------------------------------------------ */

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

/* 2.4 GHz OFDM rates (6, 12, 24 basic rates - pure ERP/OFDM mode without 11b CCK) */
const u8 wave600_rates_2g[8] = {
	0x8c, 0x12, 0x98, 0x24,		/* 6 basic, 9, 12 basic, 18 */
	0xb0, 0x48, 0x60, 0x6c,		/* 24 basic, 36, 48, 54 */
};

/* 5 GHz OFDM rates (6, 12, 24 Mbps marked as basic rates with 0x80) */
const u8 wave600_rates_5g[8] = {
	0x8c, 0x12, 0x98, 0x24,		/* 6 basic, 9, 12 basic, 18 */
	0xb0, 0x48, 0x60, 0x6c,		/* 24 basic, 36, 48, 54 */
};

/* IE ids routed to the individual beacon template parts, exactly as the
 * vendor classifies them in beacon_manager.c _wave_get_beacon_data_length
 * (numeric values from driver/shared/frame.h ie_id_t). part1 = frame head. */
#define W600_IE_TIM			5
#define W600_IE_COUNTRY			7
#define W600_IE_FH_PATTRN_PARAMS	8
#define W600_IE_FH_PATTRM_TABLE		9
#define W600_IE_PWR_CONSTRAINT		32
#define W600_IE_TPC_REPORT		35
#define W600_IE_CHANNEL_SW_ANN		37
#define W600_IE_QUIET			40
#define W600_IE_IBSS_DFS		41
#define W600_IE_ERP_INFO		42
#define W600_IE_RSN			48
#define W600_IE_EXT_SUPP_RATES		50
#define W600_IE_BSS_LOAD		11
#define W600_IE_EDCA_PARAM_SET		2
#define W600_IE_HT_CAPABILITIES		45
#define W600_IE_QOS_CAPABILITY		46
#define W600_IE_AP_CHANNEL_REPORT	51
#define W600_IE_MOBILITY_DOMAIN		54
#define W600_IE_DSE_REGISTERED_LOCATION	58
#define W600_IE_SUPPORTED_OPER_CLASSES	59
#define W600_IE_EXT_CHANNEL_SW_ANN	60
#define W600_IE_HT_INFORMATION		61
#define W600_IE_BSS_AVERAGE_ACCESS	63
#define W600_IE_ANTENNA			64
#define W600_IE_MEASURE_PILOT_TRANSMISSION	66
#define W600_IE_BSS_AVAILABLE_ADMISSION_CAPACITY	67
#define W600_IE_BSS_AC_ACCESS_DELAY	68
#define W600_IE_TIME_ADV		69
#define W600_IE_RM_ENABLED_CAPABILITIES	70
#define W600_IE_MULTIPLE_BSSID		71
#define W600_IE_COEXISTENCE		72
#define W600_IE_OBSS_SCAN_PARAMETERS	74
#define W600_IE_FMS_DESCRIPTOR		86
#define W600_IE_QOS_TRAFFIC_CAPABILITY	89
#define W600_IE_INTERWORKING		107
#define W600_IE_ADV_PROTO		108
#define W600_IE_ROAMING_CONS		111
#define W600_IE_EMERGENCY_ALERT_ID	112
#define W600_IE_MESH_CONFIG		113
#define W600_IE_MESH_ID			114
#define W600_IE_MESH_CHANNEL_SW_PARAMETERS	118
#define W600_IE_MESH_AWAKE_WINDOW	119
#define W600_IE_BEACON_TIMING		120
#define W600_IE_MCCAOP_ADV		123
#define W600_IE_EXT_CAPABILITIES	127
#define W600_IE_MULTI_BAND		158
#define W600_IE_MCCAOP_ADV_OVERVIEW	174
#define W600_IE_QMF_POLICY		181
#define W600_IE_QLOAD_REPORT		186
#define W600_IE_HCCA_TXOP_UPDATE_COUNT	187
#define W600_IE_VHT_CAPABILITIES	191
#define W600_IE_VHT_OPERATION		192
#define W600_IE_TX_PWR_ENVELOPE		195

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

/*
 * Vendor radio_context_init/radio_pdb_def defaults: CCA thresholds come from
 * the host-only PSDB 0x330000 table; if the table is absent, use the vendor
 * initial PDB values. The firmware message is UM_MAN_CCA_TH_REQ (0x0468).
 */
int wave600_send_cca_threshold(struct wave600_radio *radio)
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
int wave600_send_bf_mode(struct wave600_radio *radio, u8 bf_mode)
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
int wave600_send_he_mu_operation(struct wave600_radio *radio, u8 enable_he_mu)
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
int wave600_send_txop_mode(struct wave600_radio *radio, u32 sid, u8 mode)
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
int wave600_send_ts_vap_configure(struct wave600_radio *radio,
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
int wave600_send_power_limit_offset_radio(struct wave600_radio *radio,
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

int wave600_send_set_chan(struct wave600_radio *radio, u8 chan, u8 width,
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

int wave600_send_set_bss(struct wave600_radio *radio,
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

int wave600_send_add_vap(struct wave600_radio *radio, bool is_5g)
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
int wave600_send_vap_db_add(struct wave600_radio *radio)
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
int wave600_send_vap_db_del(struct wave600_radio *radio)
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
int wave600_send_mib_pre_activate(struct wave600_radio *radio, bool is_5g,
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

int wave600_send_mib_spectrum_mode(struct wave600_radio *radio, u8 width)
{
	/* Vendor core.c:2130: MIB_SPECTRUM_MODE uses prog_model_spectrum_mode (CW_20 or CW_40) */
	u8 v = (width == 0) ? W600_MIB_SPECTRUM_20M : W600_MIB_SPECTRUM_40M;

	return wave600_send_mib(radio, W600_MIB_SPECTRUM_MODE, &v, sizeof(v));
}

/* Vendor mtlk_set_vap_mibs (mib_osdep.c): MIB_IEEE_ADDRESS carries the
 * interface MAC before VAP activation. */
int wave600_send_mib_ieee_addr(struct wave600_radio *radio)
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
void wave600_diag_mib_readback(struct wave600_radio *radio, u16 object_id,
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
int wave600_send_enable_radio(struct wave600_radio *radio, u32 on)
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
int wave600_send_tpc_ant_params(struct wave600_radio *radio, u8 chan, bool is_5g)
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

int wave600_send_tpc_config(struct wave600_radio *radio, u8 chan, bool is_5g)
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
int wave600_send_pre_activate(struct wave600_radio *radio)
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
int wave600_send_set_wmm_params(struct wave600_radio *radio)
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

int wave600_send_beacon(struct wave600_radio *radio,
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

int wave600_chandef_to_params(struct wave600_radio *radio,
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
