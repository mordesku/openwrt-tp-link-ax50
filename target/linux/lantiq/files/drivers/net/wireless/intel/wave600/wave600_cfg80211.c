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
#include "wave600_umi.h"
#include "wave600_sta.h"

/* ------------------------------------------------------------------ */
/* cfg80211 operations                                                 */
/* ------------------------------------------------------------------ */

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
















/* ------------------------------------------------------------------ */
/* cfg80211 operations                                                 */
/* ------------------------------------------------------------------ */



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
	} else {
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

	wave600_cfg80211_abort_scan(radio);

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

	/* Ensure all stations are removed from FW before stopping/removing the VAP */
	wave600_radio_flush_stations(radio);

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
	wave600_cfg80211_abort_scan(radio);
	cancel_delayed_work_sync(&radio->cac_work);
	radio->cac_in_progress = false;

	cfg80211_cac_event(radio->ndev, &radio->cac_chandef,
			   NL80211_RADAR_CAC_ABORTED, GFP_KERNEL, 0);
}

static int wave600_send_probe_req(struct wave600_radio *radio,
				  struct cfg80211_scan_request *req,
				  bool is_5g)
{
	static const u8 rates_2g[8] = { 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24 };
	static const u8 ext_rates_2g[4] = { 0x30, 0x48, 0x60, 0x6c };
	static const u8 rates_5g[8] = { 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c };
	int n_ssids = req->n_ssids > 0 ? req->n_ssids : 1;
	u8 *buf;
	int i, ret = 0;

	buf = kmalloc(WAVE600_DMA_MGMT_BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < n_ssids; i++) {
		struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)buf;
		u8 *pos = buf + sizeof(*hdr);
		const u8 *ssid = (req->n_ssids > 0) ? req->ssids[i].ssid : NULL;
		u8 ssid_len = (req->n_ssids > 0) ? req->ssids[i].ssid_len : 0;
		size_t needed = sizeof(*hdr) + 2 + ssid_len + 2 + 8 + (is_5g ? 0 : 6) + req->ie_len;

		if (needed > WAVE600_DMA_MGMT_BUF_SIZE)
			continue;

		memset(hdr, 0, sizeof(*hdr));
		hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_REQ);
		eth_broadcast_addr(hdr->addr1);
		ether_addr_copy(hdr->addr2, radio->ndev->dev_addr);
		eth_broadcast_addr(hdr->addr3);

		*pos++ = WLAN_EID_SSID;
		*pos++ = ssid_len;
		if (ssid_len && ssid) {
			memcpy(pos, ssid, ssid_len);
			pos += ssid_len;
		}

		*pos++ = WLAN_EID_SUPP_RATES;
		*pos++ = 8;
		memcpy(pos, is_5g ? rates_5g : rates_2g, 8);
		pos += 8;

		if (!is_5g) {
			*pos++ = WLAN_EID_EXT_SUPP_RATES;
			*pos++ = 4;
			memcpy(pos, ext_rates_2g, 4);
			pos += 4;
		}

		if (req->ie && req->ie_len > 0) {
			memcpy(pos, req->ie, req->ie_len);
			pos += req->ie_len;
		}

		ret = wave600_dma_mgmt_tx(radio, buf, pos - buf);
		if (ret)
			break;
	}

	kfree(buf);
	return ret;
}

static void wave600_scan_work(struct work_struct *work)
{
	struct wave600_radio *radio = container_of(work, struct wave600_radio, scan_work.work);
	struct wave600_dev *wdev = radio->wdev;
	struct cfg80211_scan_request *req;
	struct cfg80211_scan_info info = {};
	struct ieee80211_channel *chan;
	bool aborted = false;
	bool is_5g;
	unsigned long dwell_ms;
	int ret;

	mutex_lock(&radio->scan_lock);
	req = radio->scan_req;
	if (!radio->scan_in_progress || !req) {
		mutex_unlock(&radio->scan_lock);
		return;
	}

	if (radio->scan_aborted || wdev->state != W600_STATE_RUNNING ||
	    atomic_read(&wdev->in_recovery)) {
		aborted = true;
		goto finish_scan;
	}

	if (radio->scan_chan_idx >= req->n_channels) {
		if (radio->ap_started && radio->ap_channel) {
			bool ap_5g = (radio->ap_freq >= 5000);

			wave600_send_tpc_config(radio, radio->ap_channel, ap_5g);
			wave600_send_tpc_ant_params(radio, radio->ap_channel, ap_5g);
			wave600_send_set_chan(radio, radio->ap_low_chan, radio->ap_width,
					      radio->ap_primary_idx, W600_SWITCH_TYPE_NORMAL, 0);
		}
		goto finish_scan;
	}

	chan = req->channels[radio->scan_chan_idx];
	is_5g = (chan->band == NL80211_BAND_5GHZ);
	radio->scan_freq = chan->center_freq;

	wave600_send_tpc_config(radio, chan->hw_value, is_5g);
	wave600_send_tpc_ant_params(radio, chan->hw_value, is_5g);
	ret = wave600_send_set_chan(radio, chan->hw_value, 0, 0,
				    W600_SWITCH_TYPE_SCAN, 0);
	if (ret)
		dev_warn(wdev->dev, "Radio %u scan: SET_CHAN %u MHz failed: %d\n",
			 radio->radio_idx, chan->center_freq, ret);
	else if (!(chan->flags & (IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR)))
		wave600_send_probe_req(radio, req, is_5g);

	radio->scan_chan_idx++;
	if (req->duration)
		dwell_ms = max_t(unsigned long, ieee80211_tu_to_usec(req->duration) / 1000, 20);
	else
		dwell_ms = (chan->flags & (IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR)) ? 200 : 100;

	schedule_delayed_work(&radio->scan_work, msecs_to_jiffies(dwell_ms));
	mutex_unlock(&radio->scan_lock);
	return;

finish_scan:
	radio->scan_req = NULL;
	radio->scan_in_progress = false;
	radio->scan_aborted = false;
	radio->scan_freq = 0;
	mutex_unlock(&radio->scan_lock);

	info.aborted = aborted;
	cfg80211_scan_done(req, &info);
}

void wave600_cfg80211_abort_scan(struct wave600_radio *radio)
{
	struct cfg80211_scan_request *req = NULL;
	struct cfg80211_scan_info info = { .aborted = true };

	if (!radio)
		return;

	mutex_lock(&radio->scan_lock);
	if (!radio->scan_in_progress) {
		mutex_unlock(&radio->scan_lock);
		return;
	}
	radio->scan_aborted = true;
	mutex_unlock(&radio->scan_lock);

	cancel_delayed_work_sync(&radio->scan_work);

	mutex_lock(&radio->scan_lock);
	if (radio->scan_req) {
		req = radio->scan_req;
		radio->scan_req = NULL;
		radio->scan_in_progress = false;
		radio->scan_aborted = false;
		radio->scan_freq = 0;
	}
	mutex_unlock(&radio->scan_lock);

	if (req)
		cfg80211_scan_done(req, &info);
}

static int wave600_cfg80211_scan(struct wiphy *wiphy,
				 struct cfg80211_scan_request *request)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);
	struct wave600_dev *wdev;
	bool is_5g;
	int ret;

	if (!radio || !request || !request->n_channels)
		return -EINVAL;

	wdev = radio->wdev;
	if (!wdev || wdev->state != W600_STATE_RUNNING || atomic_read(&wdev->in_recovery))
		return -EBUSY;

	if (request->n_ssids > 16 || request->ie_len > 2048)
		return -EINVAL;

	mutex_lock(&radio->scan_lock);
	if (radio->scan_in_progress || radio->cac_in_progress) {
		mutex_unlock(&radio->scan_lock);
		return -EBUSY;
	}

	is_5g = (request->channels[0]->band == NL80211_BAND_5GHZ);

	if (!radio->vap_added) {
		ret = wave600_send_vap_db_add(radio);
		if (ret)
			goto err_unlock;
		ret = wave600_send_mib_ieee_addr(radio);
		if (ret)
			goto err_unlock;
		ret = wave600_send_mib_pre_activate(radio, is_5g, 0, 0);
		if (ret)
			goto err_unlock;
		ret = wave600_send_mib_spectrum_mode(radio, 0);
		if (ret)
			goto err_unlock;
		ret = wave600_send_pre_activate(radio);
		if (ret)
			goto err_unlock;
		ret = wave600_send_add_vap(radio, is_5g);
		if (ret)
			goto err_unlock;
		radio->vap_added = true;
	}

	if (!radio->radio_enabled) {
		ret = wave600_send_enable_radio(radio, 1);
		if (ret)
			goto err_unlock;
		radio->radio_enabled = true;
		wave600_send_he_mu_operation(radio, 1);
		wave600_send_bf_mode(radio, WAVE600_BF_MODE_NON_BF);
		wave600_send_txop_mode(radio, WAVE600_GEN6_ALL_STA_SID,
				       WAVE600_TXOP_MODE_INVALID);
	}

	radio->scan_req = request;
	radio->scan_chan_idx = 0;
	radio->scan_aborted = false;
	radio->scan_in_progress = true;
	radio->scan_freq = request->channels[0]->center_freq;

	schedule_delayed_work(&radio->scan_work, 0);
	mutex_unlock(&radio->scan_lock);
	return 0;

err_unlock:
	mutex_unlock(&radio->scan_lock);
	return ret;
}

static void wave600_cfg80211_abort_scan_op(struct wiphy *wiphy,
					   struct wireless_dev *wdev_net)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	wave600_cfg80211_abort_scan(radio);
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

	if (!params || !params->key || params->key_len == 0 ||
	    params->key_len > WAVE600_UMI_RSN_TK_LEN)
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

	/*
	 * Firmware key table holds only indices 0..3 (MIB_WEP_N_DEFAULT_KEYS = 4).
	 * IGTK keys (indices 4/5 for AES-CMAC/BIP) must not be sent to FW,
	 * otherwise FW asserts in UM_MAN_SET_KEY_REQ. Accept in cfg80211 as in
	 * vendor wave_radio_key_add() / mac80211.c.
	 */
	if (key_index >= WAVE600_MAX_HW_KEY_INDEX) {
		dev_dbg(wdev->dev, "Radio %u add_key: key_idx=%u >= %u ignored by FW\n",
			radio->radio_idx, key_index, WAVE600_MAX_HW_KEY_INDEX);
		return 0;
	}

	if (pairwise && mac_addr) {
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
		/* Vendor core.c:12912 uses UMI_RSN_GROUP_KEY for both GTK and IGTK/BIP in SET_KEY */
		sid = WAVE600_BCAST_DEST_SID;
		key_type = WAVE600_UMI_RSN_GROUP_KEY;
		if (params->cipher != WLAN_CIPHER_SUITE_AES_CMAC)
			radio->is_secure = true;
		dev_dbg(wdev->dev, "Radio %u add_key (%s): bcast sid=0x%04x key_idx=%u cipher=0x%04x len=%u\n",
			radio->radio_idx,
			params->cipher == WLAN_CIPHER_SUITE_AES_CMAC ? "IGTK/BIP" : "GROUP",
			sid, key_index, u16_cipher, params->key_len);
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

static int wave600_set_default_key_idx(struct wave600_radio *radio,
				       u16 key_type, u8 key_index)
{
	struct wave600_dev *wdev = radio->wdev;
	struct wave600_umi_default_key_index req;
	struct wave600_umi_default_key_index cfm;
	int ret;

	memset(&req, 0, sizeof(req));
	req.sid = cpu_to_le16(WAVE600_BCAST_DEST_SID);
	req.key_type = cpu_to_le16(key_type);
	req.key_index = cpu_to_le16(key_index);

	ret = wave600_ipc_cmd_radio(wdev, radio->radio_idx,
				    WAVE600_UM_MAN_SET_DEFAULT_KEY_INDEX_REQ,
				    WAVE600_MC_MAN_SET_DEFAULT_KEY_INDEX_CFM,
				    &req, sizeof(req),
				    &cfm, sizeof(cfm), 2000,
				    radio->vap_id);
	if (ret) {
		dev_warn(wdev->dev,
			 "Radio %u SET_DEFAULT_KEY (type %u idx %u) IPC failed: %d\n",
			 radio->radio_idx, key_type, key_index, ret);
		return ret;
	}

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

	dev_dbg(radio->wdev->dev, "Radio %u set_default_key: key_idx=%u unicast=%d multicast=%d\n",
		radio->radio_idx, key_index, unicast, multicast);

	return wave600_set_default_key_idx(radio, WAVE600_UMI_RSN_GROUP_KEY, key_index);
}

static int wave600_cfg80211_set_default_mgmt_key(struct wiphy *wiphy,
						 struct net_device *ndev,
						 int link_id,
						 u8 key_index)
{
	struct wave600_radio *radio = wave600_wiphy_to_radio(wiphy);

	if (key_index >= WAVE600_MAX_HW_KEY_INDEX) {
		dev_dbg(radio->wdev->dev,
			"Radio %u set_default_mgmt_key: key_idx=%u >= %u ignored by FW\n",
			radio->radio_idx, key_index, WAVE600_MAX_HW_KEY_INDEX);
		return 0;
	}

	dev_dbg(radio->wdev->dev, "Radio %u set_default_mgmt_key: key_idx=%u\n",
		radio->radio_idx, key_index);

	return wave600_set_default_key_idx(radio, WAVE600_UMI_RSN_MGMT_GROUP_KEY, key_index);
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
	.scan = wave600_cfg80211_scan,
	.abort_scan = wave600_cfg80211_abort_scan_op,
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
	.set_default_mgmt_key = wave600_cfg80211_set_default_mgmt_key,
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
	radio->vap_id = 0; /* primary AP VAP index is 0 on both radios; DC distinguishes radios by ep_id in DW1 */
	radio->antenna_mask = (radio_idx == 0) ? 0x05 : 0x0A;
	INIT_DELAYED_WORK(&radio->cac_work, wave600_cac_work);
	INIT_DELAYED_WORK(&radio->scan_work, wave600_scan_work);
	mutex_init(&radio->scan_lock);

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
	wiphy->max_scan_ssids = 16;
	wiphy->max_scan_ie_len = 2048;
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

	wave600_cfg80211_abort_scan(radio);
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
