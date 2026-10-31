/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __WAVE600_CFG80211_H__
#define __WAVE600_CFG80211_H__

struct wave600_dev;

int wave600_cfg80211_init(struct wave600_dev *wdev);
void wave600_cfg80211_cleanup(struct wave600_dev *wdev);

#endif /* __WAVE600_CFG80211_H__ */
