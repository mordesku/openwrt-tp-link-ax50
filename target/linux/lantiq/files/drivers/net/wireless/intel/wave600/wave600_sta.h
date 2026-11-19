/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Station Table & Lifecycle Management
 */

#ifndef __WAVE600_STA_H__
#define __WAVE600_STA_H__

#include <linux/types.h>

struct net_device;
struct station_del_parameters;
struct station_info;
struct station_parameters;
struct wiphy;
struct wave600_radio;
struct wave600_station;

int wave600_radio_find_sid(struct wave600_radio *radio, const u8 *mac);
struct wave600_station *wave600_find_sta(struct wave600_radio *radio, const u8 *mac);
void wave600_radio_flush_stations(struct wave600_radio *radio);

int wave600_cfg80211_add_station(struct wiphy *wiphy, struct net_device *ndev,
				 const u8 *mac, struct station_parameters *params);
int wave600_cfg80211_change_station(struct wiphy *wiphy, struct net_device *ndev,
				    const u8 *mac, struct station_parameters *params);
int wave600_cfg80211_del_station(struct wiphy *wiphy, struct net_device *ndev,
				 struct station_del_parameters *params);
int wave600_cfg80211_get_station(struct wiphy *wiphy, struct net_device *ndev,
				 const u8 *mac, struct station_info *sinfo);
int wave600_cfg80211_dump_station(struct wiphy *wiphy, struct net_device *ndev,
				  int idx, u8 *mac, struct station_info *sinfo);

#endif /* __WAVE600_STA_H__ */
