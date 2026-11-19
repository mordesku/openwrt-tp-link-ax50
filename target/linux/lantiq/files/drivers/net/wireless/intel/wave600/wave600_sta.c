// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Station Table & Lifecycle Management
 */

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <net/cfg80211.h>
#include "wave600.h"
#include "wave600_ipc.h"
#include "wave600_umi.h"
#include "wave600_sta.h"

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

int wave600_cfg80211_add_station(struct wiphy *wiphy,
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

int wave600_cfg80211_change_station(struct wiphy *wiphy,
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

int wave600_cfg80211_del_station(struct wiphy *wiphy,
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

int wave600_cfg80211_get_station(struct wiphy *wiphy,
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

int wave600_cfg80211_dump_station(struct wiphy *wiphy,
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
