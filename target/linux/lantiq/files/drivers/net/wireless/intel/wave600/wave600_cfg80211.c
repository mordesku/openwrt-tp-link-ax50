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
#define W600_MIB_IEEE_ADDRESS			0x0040
#define W600_NETWORK_11BGN_MIXED		5	/* NETWORK_MODES_E (2.4 GHz) */
#define W600_NETWORK_11AN_MIXED			8	/* NETWORK_MODES_E (5 GHz) */
#define W600_VAP_OPERATION_ADD			1
#define W600_VAP_OPERATION_DEL			2
#define W600_OPERATE_RATE_SET_11BGN		0xffffffffU
#define W600_BSS_BASIC_RATE_SET_11BGN_DEFAULT	0x00007800U
#define W600_OPERATE_RATE_SET_11AN		0xffff80ffU
#define W600_BSS_BASIC_RATE_SET_11AN		0x00000015U

/* TPC (mhi_mib_id.h) */
#define W600_TPC_CAL_TYPE_LOGARITHMIC		1
#define W600_TPC_OPEN_LOOP			1
#define W600_TPC_NO_FIXED_GAIN			0xFF
/* Large limit: vendor clamps with MIN(power_cfg, psd_value), so a large
 * regulation limit makes the firmware apply its own PSDB value. Vendor
 * warns that zero limits crash the FW. */
#define W600_TPC_LIMIT_MAX			0x7FFF

/*
 * switch_type values (switchType enum, mhi_umi_propr.h):
 * ST_NORMAL = 0 — regular channel change (hot or cold)
 * ST_CSA = 2 — channel switch announcement
 */
#define W600_SWITCH_TYPE_NORMAL		0
#define W600_SWITCH_TYPE_CSA		2

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
	 * Regulatory domain: vendor sends core_cfg_get_regd_code() result,
	 * defaulting to "US" (REGD_CODE_FCC = 0x10). Zero is
	 * REGD_CODE_UNKNOWN and is never sent by the vendor driver.
	 */
	p.regulation_type = 0x10;

	/*
	 * isContinuousInterfererDetectionNeeded: vendor scan_support.h
	 * is_interf_det_needed() returns 1 for all channels.
	 */
	p.is_continuous_interferer_detection_needed = 1;

	/*
	 * isRadarDetectionNeeded: true when the channel requires DFS
	 * (5 GHz DFS channels). For 2.4 GHz it is always false.
	 * We approximate by checking if the primary channel is in the
	 * 5 GHz DFS range (channels 36-64, 100-144).
	 */
	if (is_5g) {
		if ((chan >= 36 && chan <= 64) || (chan >= 100 && chan <= 144))
			is_dfs_chan = true;
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

	return 0;
}

static int wave600_send_set_bss(struct wave600_radio *radio, bool is_5g)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_set_bss p;
	const u8 *rates = is_5g ? wave600_rates_5g : wave600_rates_2g;
	u8 rates_len = is_5g ? sizeof(wave600_rates_5g) : sizeof(wave600_rates_2g);

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian on the wire (vendor
	 * HOST_TO_MAC16/32 = cpu_to_le16/32, mtlkinc.h; the MAN transport
	 * copies the payload verbatim). */
	p.beacon_interval = cpu_to_le16(W600_BEACON_INT);
	p.dtim_interval = cpu_to_le16(W600_DTIM);
	p.vap_id = radio->vap_id;
	p.slot_time = 1;	/* short slot for OFDM */
	p.use_short_preamble = 1;
	memcpy(p.rates, rates, rates_len);
	p.rates_len = rates_len;
	/* Single stream MCS 0-7 */
	memset(p.tx_mcs_bitmask, 0x00, sizeof(p.tx_mcs_bitmask));
	p.tx_mcs_bitmask[0] = 0xff;
	memset(p.vht_mcs_nss, 0xff, sizeof(p.vht_mcs_nss));
	memset(p.he_mcs_nss, 0xff, sizeof(p.he_mcs_nss));
	p.he_bss_color = 1;	/* vendor HE_BSS_COLOR = 1 (core.c:2555) */

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
	/* Vendor core.c:2342 explicitly memsets u8TX_MCS_Bitmask to 0 */
	memset(p.tx_mcs_bitmask, 0, sizeof(p.tx_mcs_bitmask));
	/* Vendor memsets the VHT/HE MCS-NSS maps to 0xff: each 2-bit subfield
	 * then reads 3 = "n spatial streams not supported" (UMI_ADD_VAP). */
	memset(p.vht_mcs_nss, 0xff, sizeof(p.vht_mcs_nss));
	memset(p.he_mcs_nss, 0xff, sizeof(p.he_mcs_nss));

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
static int wave600_send_mib_pre_activate(struct wave600_radio *radio, bool is_5g)
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
	v.upper_lower_channel = 0;	/* no channel bonding (20 MHz) */
	v.spectrum_mode = W600_MIB_SPECTRUM_20M;
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC32) */
	v.operational_rate_set = cpu_to_le32(is_5g ? W600_OPERATE_RATE_SET_11AN : W600_OPERATE_RATE_SET_11BGN);
	v.bss_basic_rate_set = cpu_to_le32(is_5g ? W600_BSS_BASIC_RATE_SET_11AN : W600_BSS_BASIC_RATE_SET_11BGN_DEFAULT);

	return wave600_send_mib(radio, W600_MIB_PRE_ACTIVATE, &v, sizeof(v));
}

static int wave600_send_mib_spectrum_mode(struct wave600_radio *radio)
{
	u8 v = W600_MIB_SPECTRUM_20M;

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
		dev_info(wdev->dev,
			 "diag Radio %u: GET_MIB 0x%04x (%s): request failed: %d\n",
			 radio->radio_idx, object_id, what, ret);
		return;
	}

	dev_info(wdev->dev,
		 "diag Radio %u: GET_MIB 0x%04x (%s): status=0x%04x value=%*ph (written: %*ph)\n",
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

/* Vendor MC_MAN_SET_ANTENNAS_REQ (0x0418) */
static int wave600_send_set_antennas(struct wave600_radio *radio, u8 mask)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_set_antennas p;
	struct wave600_umi_set_antennas cfm;
	int ret;

	memset(&p, 0, sizeof(p));
	p.tx_ants_mask = mask;
	p.rx_ants_mask = mask;

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_MC_MAN_SET_ANTENNAS_REQ,
				    WAVE600_MC_MAN_SET_ANTENNAS_CFM,
				    &p, sizeof(p), &cfm, sizeof(cfm), 5000,
				    radio->vap_id);
	if (ret)
		return ret;

	if (cfm.status != W600_UMI_OK) {
		dev_err(wdev->dev, "Radio %u SET_ANTENNAS: FW rejected (status=%u)\n",
			radio->radio_idx, cfm.status);
		return -EIO;
	}
	dev_info(wdev->dev, "Radio %u active antennas set to 0x%02x\n",
		 radio->radio_idx, mask);
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

	dev_info(wdev->dev, "Radio %u beacon template: total %zu = %u/%u/%u/%u/%u (dma %pad)\n",
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

	if (settings->ssid_len == 0 || settings->ssid_len > W600_SSID_MAX_LEN)
		return -EINVAL;

	ret = wave600_chandef_to_params(radio, &settings->chandef,
					&chan, &width, &low_chan, &primary_idx);
	if (ret)
		return ret;

	is_5g = (radio->radio_idx == 1);
	radio->ap_channel = chan;
	radio->ap_freq = settings->chandef.chan->center_freq;

	dev_info(wdev->dev, "Radio %u start_ap: SSID '%.*s' band %s channel %d (low %u, idx %u) width %d\n",
		 radio->radio_idx, (int)settings->ssid_len, settings->ssid,
		 is_5g ? "5GHz" : "2.4GHz", chan, low_chan, primary_idx, width);

	/*
	 * Vendor AP bring-up sequence (verified against _mtlk_core_on_rcvry_configure
	 * in core.c:14677-14752 and live hybrid boot log):
	 *  1. VAP_DB_ADD -> MIB_IEEE_ADDRESS -> MIB_PRE_ACTIVATE ->
	 *     MIB_SPECTRUM_MODE -> MBSS_PRE_ACTIVATE -> ADD_VAP
	 *  2. ENABLE_RADIO(1)
	 *  3. SET_CHAN -> SET_TPC_ANT_PARAMS -> SET_TPC_CONFIG
	 *  4. SET_BEACON_TEMPLATE -> SET_BEACON_INTERVAL -> SET_BSS
	 */
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

	ret = wave600_send_mib_pre_activate(radio, is_5g);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: MIB_PRE_ACTIVATE failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	ret = wave600_send_mib_spectrum_mode(radio);
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

	if (!radio->radio_enabled) {
		ret = wave600_send_enable_radio(radio, 1);
		if (ret) {
			dev_err(wdev->dev, "Radio %u start_ap: ENABLE_RADIO failed: %d\n", radio->radio_idx, ret);
			return ret;
		}
		radio->radio_enabled = true;
	}

	ret = wave600_send_set_chan(radio, low_chan, width, primary_idx,
				    W600_SWITCH_TYPE_NORMAL, 0);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_CHAN failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	/* Vendor core_config.c:646-650 mtlk_core_set_coc_actual_power_mode -> MC_MAN_SET_ANTENNAS_REQ */
	ret = wave600_send_set_antennas(radio, radio->antenna_mask ? radio->antenna_mask : (is_5g ? 0x0A : 0x05));
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_ANTENNAS failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	ret = wave600_send_beacon(radio, &settings->beacon);
	if (ret) {
		dev_err(wdev->dev, "Radio %u start_ap: SET_BEACON_TEMPLATE failed: %d\n", radio->radio_idx, ret);
		return ret;
	}

	ret = wave600_send_set_bss(radio, is_5g);
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

	radio->ap_started = true;
	netif_carrier_on(ndev);
	netif_tx_wake_all_queues(ndev);
	dev_info(wdev->dev, "Radio %u AP started on %s (%s, channel %u)\n",
		 radio->radio_idx, ndev->name, is_5g ? "5GHz" : "2.4GHz", chan);
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

	if (!radio->ap_started)
		return 0;

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
	radio->sta_connected = false;
	radio->sta_sid = 0;
	radio->is_secure = false;
	eth_zero_addr(radio->sta_mac);
	return 0;
}

static int wave600_cfg80211_change_beacon(struct wiphy *wiphy, struct net_device *ndev,
					  struct cfg80211_ap_update *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	return wave600_send_beacon(radio, &params->beacon);
}

static int wave600_cfg80211_set_channel(struct wiphy *wiphy,
					struct cfg80211_chan_def *chandef)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	u8 chan, width, low_chan, primary_idx = 0;
	int ret;

	ret = wave600_chandef_to_params(radio, chandef, &chan, &width, &low_chan, &primary_idx);
	if (ret)
		return ret;

	radio->ap_channel = chan;
	radio->ap_freq = chandef->chan->center_freq;

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

	cfg80211_ch_switch_notify(ndev, &params->chandef, 0);
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

	if (type != NL80211_IFTYPE_AP) {
		dev_info(wiphy_dev(wiphy), "change_virtual_intf: type %d not supported\n",
			 type);
		return -EOPNOTSUPP;
	}

	return 0;
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
	u16 sid;
	int ret;

	if (!radio->ap_started)
		return -ENETDOWN;

	dev_info(wdev->dev, "Radio %u cfg80211: add station %pM (aid %u)\n",
		 radio->radio_idx, mac, params ? params->aid : 0);

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
	dev_info(wdev->dev, "Radio %u REQUEST_SID %pM confirmed: sid=%u\n",
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
	 * In WPA/WPA2 mode: STA_ADD_FLAGS_WME (0x02) only. Port is closed until PTK handshake completes.
	 */
	req.flags = radio->is_secure ? 0x02 : 0x06;

	/* If client has HT capabilities, copy HT parameters and set STA_ADD_FLAGS_IS_HT (0x10) */
	if (params && params->link_sta_params.ht_capa) {
		const struct ieee80211_ht_cap *ht = params->link_sta_params.ht_capa;

		req.ht_cap_info = ht->cap_info;
		req.ampdu_param = ht->ampdu_params_info;
		memcpy(req.rx_mcs_bitmask, ht->mcs.rx_mask, sizeof(req.rx_mcs_bitmask));
		req.flags |= 0x10; /* STA_ADD_FLAGS_IS_HT */
	}

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

	radio->sta_connected = true;
	radio->sta_sid = sid;
	ether_addr_copy(radio->sta_mac, mac);
	dev_info(wdev->dev, "Radio %u STA_ADD %pM confirmed: status=%u sid=%u (flags=0x%02x)\n",
		 radio->radio_idx, mac, cfm.status, radio->sta_sid, req.flags);

	return 0;
}

static int wave600_cfg80211_change_station(struct wiphy *wiphy,
					   struct net_device *ndev,
					   const u8 *mac,
					   struct station_parameters *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;

	dev_info(wdev->dev, "Radio %u cfg80211: change station %pM (aid %u, flags_set=0x%x)\n",
		 radio->radio_idx, mac, params ? params->aid : 0, params ? params->sta_flags_set : 0);

	/*
	 * When station completes 4-way handshake, hostapd authorizes the station:
	 * Open the 802.1X filter via UM_MAN_SET_802_1X_FILTER_REQ (0x0446).
	 * Vendor core.c:9674-9690 _mtlk_send_filter_req.
	 */
	if (params && (params->sta_flags_set & BIT(NL80211_STA_FLAG_AUTHORIZED)) && radio->is_secure) {
		struct wave600_umi_802_1x_filter req;
		struct wave600_umi_802_1x_filter cfm;
		int ret;

		memset(&req, 0, sizeof(req));
		req.sid = cpu_to_le16(radio->sta_sid);
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
		else
			dev_info(wdev->dev, "Radio %u SET_802_1X_FILTER %pM (sid %u) OPENED\n",
				 radio->radio_idx, mac, radio->sta_sid);
	}

	return 0;
}

static int wave600_cfg80211_del_station(struct wiphy *wiphy,
					struct net_device *ndev,
					struct station_del_parameters *params)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_stop_traffic sreq;
	struct wave600_umi_stop_traffic scfm;
	struct wave600_umi_sta_remove req;
	struct wave600_umi_sta_remove cfm;
	struct wave600_umi_remove_sid rreq;
	struct wave600_umi_remove_sid rcfm;
	u16 sid;
	int ret;

	if (!radio->ap_started || !radio->sta_connected)
		return 0;

	if (params && params->mac && !ether_addr_equal(params->mac, radio->sta_mac))
		return 0;

	sid = radio->sta_sid;
	dev_info(wdev->dev, "Radio %u cfg80211: del station %pM (sid %u)\n",
		 radio->radio_idx, radio->sta_mac, sid);

	/* Stop traffic queues before removing station to prevent FW assertion 0x03c2 */
	memset(&sreq, 0, sizeof(sreq));
	sreq.sid = cpu_to_le16(sid);
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_STOP_TRAFFIC_REQ,
				    WAVE600_MC_MAN_STOP_TRAFFIC_CFM,
				    &sreq, sizeof(sreq), &scfm, sizeof(scfm), 2000,
				    radio->vap_id);
	if (ret)
		dev_warn(wdev->dev, "Radio %u STOP_TRAFFIC IPC failed: %d\n", radio->radio_idx, ret);

	memset(&req, 0, sizeof(req));
	req.sid = cpu_to_le16(sid);
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_STA_REMOVE_REQ,
				    WAVE600_MC_MAN_STA_REMOVE_CFM,
				    &req, sizeof(req), &cfm, sizeof(cfm), 2000,
				    radio->vap_id);
	if (ret)
		dev_warn(wdev->dev, "Radio %u STA_REMOVE IPC failed: %d\n", radio->radio_idx, ret);

	/* Release SID in ARC firmware hardware table (vendor core.c:14879 UM_MAN_REMOVE_SID_REQ) */
	memset(&rreq, 0, sizeof(rreq));
	rreq.sid = cpu_to_le16(sid);
	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_REMOVE_SID_REQ,
				    WAVE600_MC_MAN_REMOVE_SID_CFM,
				    &rreq, sizeof(rreq), &rcfm, sizeof(rcfm), 2000,
				    radio->vap_id);
	if (ret)
		dev_warn(wdev->dev, "Radio %u REMOVE_SID IPC failed: %d\n", radio->radio_idx, ret);

	radio->sta_connected = false;
	radio->sta_sid = 0;
	eth_zero_addr(radio->sta_mac);
	return 0;
}

static int wave600_cfg80211_get_station(struct wiphy *wiphy,
					struct net_device *ndev,
					const u8 *mac,
					struct station_info *sinfo)
{
	return 0;
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

	if (pairwise && mac_addr) {
		sid = radio->sta_sid;
		key_type = WAVE600_UMI_RSN_PAIRWISE_KEY;
		dev_info(wdev->dev, "Radio %u add_key (PAIRWISE): STA %pM sid=%u key_idx=%u cipher=0x%04x len=%u\n",
			 radio->radio_idx, mac_addr, sid, key_index, u16_cipher, params->key_len);
	} else {
		sid = WAVE600_BCAST_DEST_SID;
		key_type = WAVE600_UMI_RSN_GROUP_KEY;
		radio->is_secure = true;
		dev_info(wdev->dev, "Radio %u add_key (GROUP): bcast sid=0x%04x key_idx=%u cipher=0x%04x len=%u\n",
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

	dev_info(wdev->dev, "Radio %u SET_KEY confirmed: sid=0x%04x key_type=%u key_idx=%u\n",
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

	dev_info(wdev->dev, "Radio %u set_default_key: key_idx=%u unicast=%d multicast=%d\n",
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
	.change_beacon = wave600_cfg80211_change_beacon,
	.set_monitor_channel = wave600_cfg80211_set_channel,
	.channel_switch = wave600_cfg80211_channel_switch,
	.mgmt_tx = wave600_cfg80211_mgmt_tx,
	.mgmt_tx_cancel_wait = wave600_cfg80211_mgmt_tx_cancel_wait,
	.add_station = wave600_cfg80211_add_station,
	.del_station = wave600_cfg80211_del_station,
	.change_station = wave600_cfg80211_change_station,
	.get_station = wave600_cfg80211_get_station,
	.add_key = wave600_cfg80211_add_key,
	.del_key = wave600_cfg80211_del_key,
	.set_default_key = wave600_cfg80211_set_default_key,
	.auth = wave600_cfg80211_auth,
	.assoc = wave600_cfg80211_assoc,
	.deauth = wave600_cfg80211_deauth,
	.disassoc = wave600_cfg80211_disassoc,
	.connect = wave600_cfg80211_connect,
	.disconnect = wave600_cfg80211_disconnect,
};

/* ------------------------------------------------------------------ */
/* Wiphy registration                                                  */
/* ------------------------------------------------------------------ */

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
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5180, .hw_value = 36, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5200, .hw_value = 40, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5220, .hw_value = 44, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5240, .hw_value = 48, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5260, .hw_value = 52, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5280, .hw_value = 56, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5300, .hw_value = 60, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5320, .hw_value = 64, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5500, .hw_value = 100, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5520, .hw_value = 104, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5540, .hw_value = 108, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5560, .hw_value = 112, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5580, .hw_value = 116, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5600, .hw_value = 120, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5620, .hw_value = 124, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5640, .hw_value = 128, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5660, .hw_value = 132, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5680, .hw_value = 136, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5700, .hw_value = 140, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5720, .hw_value = 144, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5745, .hw_value = 149, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5765, .hw_value = 153, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5785, .hw_value = 157, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5805, .hw_value = 161, .max_power = 23, .max_antenna_gain = 0 },
		{ .band = NL80211_BAND_5GHZ, .center_freq = 5825, .hw_value = 165, .max_power = 23, .max_antenna_gain = 0 },
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

	wiphy = wiphy_new(&wave600_cfg80211_ops, sizeof(struct wave600_radio *));
	if (!wiphy)
		return -ENOMEM;

	/* priv points back at radio */
	*(struct wave600_radio **)wiphy_priv(wiphy) = radio;
	radio->wiphy = wiphy;

	wiphy->interface_modes = BIT(NL80211_IFTYPE_AP);
	wiphy->flags |= WIPHY_FLAG_REPORTS_OBSS;
	wiphy->cipher_suites = wave600_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(wave600_cipher_suites);
	wiphy->mgmt_stypes = wave600_mgmt_stypes;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_UNSPEC;
	wiphy->max_scan_ssids = 1;
	strscpy(wiphy->fw_version, wdev->fw_version, sizeof(wiphy->fw_version));

	if (radio_idx == 0) {
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
		band->ht_cap.cap = IEEE80211_HT_CAP_SGI_20 | BIT(8); /* RX STBC 1SS */
		band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_32K;
		band->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16;
		memset(&band->ht_cap.mcs, 0, sizeof(band->ht_cap.mcs));
		band->ht_cap.mcs.rx_mask[0] = 0xff;
	} else {
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
				   IEEE80211_HT_CAP_SUP_WIDTH_20_40 | BIT(8); /* RX STBC 1SS */
		band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_32K;
		band->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16;
		memset(&band->ht_cap.mcs, 0, sizeof(band->ht_cap.mcs));
		band->ht_cap.mcs.rx_mask[0] = 0xff;

		band->vht_cap.vht_supported = true;
		band->vht_cap.cap = IEEE80211_VHT_CAP_SHORT_GI_80 |
				    IEEE80211_VHT_CAP_SHORT_GI_160 |
				    IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ |
				    IEEE80211_VHT_CAP_RXSTBC_1;
		band->vht_cap.vht_mcs.rx_mcs_map = cpu_to_le16(0xfffa);
		band->vht_cap.vht_mcs.tx_mcs_map = cpu_to_le16(0xfffa);
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
	return 0;
}

void wave600_cfg80211_cleanup(struct wave600_dev *wdev)
{
	wave600_radio_cleanup(&wdev->radios[1]);
	wave600_radio_cleanup(&wdev->radios[0]);
}
