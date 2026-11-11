/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel / MaxLinear Wave600 (WAV654) Calibration & RF Profile Management
 * Shared types: factory calibration file (EEPROM format 6.0) and the
 * Platform Specific Data (PSD.bin) parsed on the host.
 */

#ifndef __WAVE600_CAL_H__
#define __WAVE600_CAL_H__

#include <linux/types.h>
#include <linux/firmware.h>

struct wave600_dev;

/*
 * antennaParameters_t (vendor mhi_umi.h), one per radio segment
 */
struct wave600_antenna_params {
	u8 tx_ant_mask;
	u8 rx_ant_mask;
	u8 tx_ant_sel_mask;
	u8 rx_ant_sel_mask;
};

/*
 * One TPC_FREQ entry taken from the calibration file gen6 TX TSSI section.
 * Kept byte-exact (little-endian) exactly as stored in the file; the
 * firmware reads it in MAC (little) endian, the mailbox writer preserves
 * the byte order. In ARC firmware, tpcAntParams_t contains an array of
 * union _TPC_FREQ, where WAVE600_TPC_FREQ_SIZE = sizeof(TPC_FREQ) = 24.
 */
#define WAVE600_TPC_FREQ_SIZE		24
#define WAVE600_CAL_MAX_TSSI_ENTRIES	64
#define WAVE600_CAL_MAX_ANT		4

struct wave600_cal_tssi_entry {
	u8 band;	/* 0 = 2.4 GHz, 1 = 5 GHz */
	u8 chan;
	u8 bw;
	u8 raw[WAVE600_TPC_FREQ_SIZE];
};

/*
 * Factory calibration (cal_wlan0.bin, vendor EEPROM format 6.0):
 * 40-byte header, then CIS sections [tpl u8][size u16 LE][data].
 */
struct wave600_cal {
	bool parsed;			/* cal_wlan0.bin parsed (vendor HW_EEPROM_READ) */
	u16 eeprom_version;	/* MTLK_MAKE_EEPROM_VERSION(ver1, ver0) */
	u8 hw_type;		/* CIS Card ID type */
	u8 hw_rev;		/* CIS Card ID revision */
	u16 xtal;		/* CIS XTAL value in MAC (LE) form, vendor semantics */
	u16 tpc_data_size;	/* per-entry channel data size of the TSSI sections */
	struct wave600_cal_tssi_entry entries[WAVE600_CAL_MAX_ANT][WAVE600_CAL_MAX_TSSI_ENTRIES];
	u32 entry_count[WAVE600_CAL_MAX_ANT];
	/*
	 * Raw RSSI CIS sections (tpl 0x63, cis_v6_rssi_t) per band,
	 * captured byte-exact for UM_MAN_SET_RSSI_CAL_PARAMS_REQ.
	 * size = 0 means the section is absent for that band.
	 */
	struct {
		u16 size;
		u8 data[131];	/* vendor MAX_SIZE_OF_RSSI_TABLE */
	} rssi[2];		/* band 0 = 2.4 GHz, band 1 = 5 GHz */
};

/*
 * Platform Specific Data parsed from PSD.bin (kept between the host-side
 * parse and the firmware download, both inside calibration init).
 */
#define WAVE600_PSDB_MAX_TABLES		48
/* Fields message: nrows + ncols + up to 62 fblocks, little-endian words */
#define WAVE600_PSDB_FIELDS_MSG_MAX	((2 + 62 * 2) * 4)

struct wave600_psdb {
	bool parsed;
	const struct firmware *fw;	/* kept until wave600_psdb_cleanup() */
	u32 fda;			/* platform fields array byte offset */
	u8 fields_msg[WAVE600_PSDB_FIELDS_MSG_MAX];
	size_t fields_msg_len;
	struct {
		u32 id;
		u32 off;
	} tabs[WAVE600_PSDB_MAX_TABLES];
	u32 ntab;
	struct wave600_antenna_params ant[2];
	u32 regd_off;		/* REGD table byte offset in PSD.bin, 0 = none */
	s8 cca_th[2][5];
	bool cca_valid[2];
};

/*
 * Vendor init split (hw_mmb.c step order):
 *  - wave600_cal_parse_profile: HW_EEPROM_READ + HW_PSDB_READ (host
 *    parse of cal_wlan0.bin and PSD.bin; XTAL needed by HW_DEP_CFG),
 *  - wave600_cal_alloc_cache: HW_CALIBRATION_CACHE pre-activation step,
 *  - wave600_cal_init: RF bring-up (HDK config, ProgModel + PSDB
 *    download, RF calibration) after card finalize.
 */
int wave600_cal_parse_profile(struct wave600_dev *wdev);
int wave600_cal_alloc_cache(struct wave600_dev *wdev);
int wave600_cal_init(struct wave600_dev *wdev);
void wave600_cal_cleanup(struct wave600_dev *wdev);
/*
 * Vendor mtlk_efuse_eeprom_load_to_hw (EEPROM new format only): AFE block
 * from the device eFuse + the RSSI CIS section of the given band
 * (0 = 2.4 GHz). Sent between the ProgModel download and the PSDB step.
 */
int wave600_cal_send_eeprom_data(struct wave600_dev *wdev, u8 band);

/* Host-side PSDB parse (no firmware messages) and firmware download */
int wave600_psdb_parse(struct wave600_dev *wdev);
int wave600_psdb_send(struct wave600_dev *wdev, u32 pm_addr);
void wave600_psdb_cleanup(struct wave600_dev *wdev);

/*
 * Power limits of one channel from the PSDB REGD table, in vendor units,
 * antenna factor applied, MU/BF falling back to OFDM when unset.
 * pw[0] = 11B, pw[1..4] = OFDM 20/40/80/160, pw[5..8] = MU, pw[9..12] = BF.
 */
int wave600_psdb_regd_limits(struct wave600_dev *wdev, u8 regd_code, u8 chan_num,
			     u16 pw[13]);

/* Subband center channel helper for bonded bandwidths (40, 80, 160 MHz) */
static inline u8 wave600_chan_to_subband_center_chan(u8 chan, u8 width)
{
	u8 low;

	if (chan <= 14)
		return chan; /* 2.4 GHz */

	switch (width) {
	case 1: /* 40 MHz */
		if (chan >= 36 && chan <= 64)
			low = 36 + ((chan - 36) / 8) * 8;
		else if (chan >= 100 && chan <= 144)
			low = 100 + ((chan - 100) / 8) * 8;
		else if (chan >= 149 && chan <= 165)
			low = 149 + ((chan - 149) / 8) * 8;
		else
			return chan;
		return low + 2;
	case 2: /* 80 MHz */
		if (chan >= 36 && chan <= 64)
			low = 36 + ((chan - 36) / 16) * 16;
		else if (chan >= 100 && chan <= 144)
			low = 100 + ((chan - 100) / 16) * 16;
		else if (chan >= 149 && chan <= 165)
			low = 149 + ((chan - 149) / 16) * 16;
		else
			return chan;
		return low + 6;
	case 3: /* 160 MHz */
		if (chan >= 36 && chan <= 64)
			return 50; /* 5250 MHz */
		else if (chan >= 100 && chan <= 128)
			return 114; /* 5570 MHz */
		else
			return chan;
	default:
		return chan;
	}
}

static inline u16 wave600_chan_to_subband_cf(u8 chan, u8 width)
{
	u8 c_chan = wave600_chan_to_subband_center_chan(chan, width);

	if (c_chan <= 14)
		return (u16)(2407 + 5 * c_chan);

	return (u16)(5000 + 5 * c_chan);
}

#endif /* __WAVE600_CAL_H__ */
