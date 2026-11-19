/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) UMI / MIB / Regulatory / TPC Helpers
 */

#ifndef __WAVE600_UMI_H__
#define __WAVE600_UMI_H__

#include <linux/types.h>

struct cfg80211_ap_settings;
struct cfg80211_beacon_data;
struct cfg80211_chan_def;
struct wave600_dev;
struct wave600_radio;

#define W600_BEACON_INT		100
#define W600_DTIM		1
#define W600_MAX_RATES		32
#define W600_SSID_MAX_LEN	32

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
 * ST_NORMAL = 0 - regular channel change (hot or cold)
 * ST_CSA = 2 - channel switch announcement
 */
#define W600_SWITCH_TYPE_NORMAL		0
#define W600_SWITCH_TYPE_SCAN		1
#define W600_SWITCH_TYPE_CSA		2

extern const u8 wave600_rates_2g[8];
extern const u8 wave600_rates_5g[8];

int wave600_send_reg_domain_config(struct wave600_dev *wdev, u8 region);
int wave600_set_country(struct wave600_dev *wdev, const char *alpha2);

int wave600_send_cca_threshold(struct wave600_radio *radio);
int wave600_send_bf_mode(struct wave600_radio *radio, u8 bf_mode);
int wave600_send_he_mu_operation(struct wave600_radio *radio, u8 enable_he_mu);
int wave600_send_txop_mode(struct wave600_radio *radio, u32 sid, u8 mode);
int wave600_send_ts_vap_configure(struct wave600_radio *radio,
				  u8 enable_ba, u8 amsdu_support,
				  u32 window_size);
int wave600_send_power_limit_offset_radio(struct wave600_radio *radio,
					  u8 offset_dbm);
int wave600_send_set_chan(struct wave600_radio *radio, u8 chan, u8 width,
			  u8 primary_idx, u8 switch_type,
			  u16 chan_switch_time);
int wave600_send_set_bss(struct wave600_radio *radio,
			 struct cfg80211_ap_settings *settings,
			 bool is_5g);
int wave600_send_add_vap(struct wave600_radio *radio, bool is_5g);
int wave600_send_vap_db_add(struct wave600_radio *radio);
int wave600_send_vap_db_del(struct wave600_radio *radio);
int wave600_send_mib_pre_activate(struct wave600_radio *radio, bool is_5g,
				  u8 width, u8 primary_idx);
int wave600_send_mib_spectrum_mode(struct wave600_radio *radio, u8 width);
int wave600_send_mib_ieee_addr(struct wave600_radio *radio);
void wave600_diag_mib_readback(struct wave600_radio *radio, u16 object_id,
			       const void *written, size_t written_len,
			       const char *what);
int wave600_send_enable_radio(struct wave600_radio *radio, u32 on);
int wave600_send_tpc_ant_params(struct wave600_radio *radio, u8 chan, bool is_5g);
int wave600_send_tpc_config(struct wave600_radio *radio, u8 chan, bool is_5g);
int wave600_send_pre_activate(struct wave600_radio *radio);
int wave600_send_set_wmm_params(struct wave600_radio *radio);
int wave600_send_beacon(struct wave600_radio *radio,
			const struct cfg80211_beacon_data *beacon);
int wave600_chandef_to_params(struct wave600_radio *radio,
			      const struct cfg80211_chan_def *chandef,
			      u8 *out_chan, u8 *out_width,
			      u8 *out_low_chan, u8 *out_primary_idx);

#endif /* __WAVE600_UMI_H__ */
