/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Calibration & RF Profile Management
 */

#ifndef __WAVE600_CAL_H__
#define __WAVE600_CAL_H__

struct wave600_dev;

int wave600_cal_init(struct wave600_dev *wdev);
void wave600_cal_cleanup(struct wave600_dev *wdev);

#endif /* __WAVE600_CAL_H__ */
