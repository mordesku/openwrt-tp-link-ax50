/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Hardware Fault Recovery Header
 */

#ifndef __WAVE600_RECOVERY_H__
#define __WAVE600_RECOVERY_H__

struct wave600_dev;
struct wave600_radio;

void wave600_recovery_init(struct wave600_dev *wdev);
void wave600_recovery_cleanup(struct wave600_dev *wdev);
void wave600_recovery_trigger(struct wave600_dev *wdev, const char *reason);
int wave600_recovery_sysfs_create(struct wave600_dev *wdev);
void wave600_recovery_sysfs_remove(struct wave600_dev *wdev);

#endif /* __WAVE600_RECOVERY_H__ */
