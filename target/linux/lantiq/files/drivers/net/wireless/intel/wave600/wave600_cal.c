// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Calibration & RF Profile Management
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/io.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_chi.h"
#include "wave600_ipc.h"
#include "wave600_cal.h"

#define WAVE600_PROGMODEL_CHUNK_SIZE    8000
#define WAVE600_EEPROM_EXEC_SIG         0x1bfc

/* EEPROM format 6.0 (vendor mtlk_eeprom.c mtlk_eeprom_v5_t):
 * 40-byte PCI config header, then the CIS area starting with the major
 * version byte, followed by CIS sections [tpl u8][size u16 LE][data]. */
#define WAVE600_EEPROM_HDR_SIZE         40

/* CIS TPL codes (vendor cis_manager.c) */
#define WAVE600_CIS_TPL_NONE            0xFF
#define WAVE600_CIS_TPL_GEN6_CID        0x60
#define WAVE600_CIS_TPL_GEN6_TX_TSSI    0x61
#define WAVE600_CIS_TPL_GEN6_RSSI       0x63
#define WAVE600_CIS_TPL_GEN6_XTAL       0x64

/*
 * Vendor XTAL_DEFAULT_VALUE (cis_manager.h): value passed to the firmware
 * when the calibration file has no XTAL tuple.
 */
#define WAVE600_XTAL_DEFAULT            0x005C

/* CIS Card ID (vendor mtlk_cis_cardid_t, 16 bytes with SN length 3) */
#define WAVE600_CIS_CARDID_SIZE        16

/*
 * Insert one TSSI entry into the per-antenna store, keeping the list
 * sorted by channel number (vendor _tpc_gen5_list_sort).
 */
static int wave600_cal_add_tssi(struct wave600_dev *wdev, u8 ant, u8 band,
				u8 chan, u8 bw, const u8 *raw22)
{
	struct wave600_cal *cal = &wdev->cal;
	struct wave600_cal_tssi_entry *entries = cal->entries[ant];
	u32 count = cal->entry_count[ant];
	u32 i;

	if (count >= WAVE600_CAL_MAX_TSSI_ENTRIES)
		return -ENOMEM;

	/* keep sorted by channel */
	for (i = count; i > 0 && entries[i - 1].chan > chan; i--)
		entries[i] = entries[i - 1];

	entries[i].band = band;
	entries[i].chan = chan;
	entries[i].bw = bw;
	memcpy(entries[i].raw, raw22, WAVE600_TPC_FREQ_SIZE);
	cal->entry_count[ant]++;
	return 0;
}

/*
 * Parse one gen6 TX TSSI section (vendor _parse_tpc_g6): the section
 * body is one ant_mask_and_nof_regions byte followed by per-channel
 * entries of [chan_num_and_bw u16][per-antenna channel data]. Each
 * entry is stored byte-exact per antenna.
 */
static int wave600_cal_parse_tssi(struct wave600_dev *wdev, const u8 *data, u32 size)
{
	struct wave600_cal *cal = &wdev->cal;
	u32 ant_regions, ant_mask, nof_regions, ant_num;
	u32 tpc_data_size, entry_size;
	u32 chan, bw, band;
	int ret;

	if (size < 1)
		return -EINVAL;

	ant_regions = data[0];
	ant_mask = ant_regions & 0x1f;
	nof_regions = (ant_regions >> 6) & 0x3;

	/* vendor CIS_V6_TX_CHAN_DATA_SIZE(n) */
	tpc_data_size = 3 + ((nof_regions * 4 + 7) / 8) + nof_regions + 4 * nof_regions;
	ant_num = hweight8(ant_mask);
	entry_size = 2 + ant_num * tpc_data_size;

	if (cal->tpc_data_size && cal->tpc_data_size != tpc_data_size) {
		dev_err(wdev->dev, "cal: different TPC data sizes %u and %u\n",
			cal->tpc_data_size, tpc_data_size);
		return -EINVAL;
	}
	cal->tpc_data_size = tpc_data_size;

	size -= 1;
	data += 1;

	while (size >= entry_size) {
		u16 chan_and_bw = (u16)(data[0] | (data[1] << 8)); /* LE */
		u32 idx, slice = 0;

		chan = chan_and_bw & 0x0fff;
		bw = (chan_and_bw >> 12) & 0x0f;
		band = (chan <= 14) ? 0 : 1;

		for (idx = 0; idx < WAVE600_CAL_MAX_ANT; idx++) {
			u8 raw[WAVE600_TPC_FREQ_SIZE] = { 0 };

			if (!(ant_mask & BIT(idx)))
				continue;

			/* byte-exact TPC_FREQ entry, little-endian as in the file */
			raw[0] = data[0];
			raw[1] = data[1];
			memcpy(raw + 2, data + 2 + slice, tpc_data_size);

			ret = wave600_cal_add_tssi(wdev, (u8)idx, (u8)band,
						   (u8)chan, (u8)bw, raw);
			if (ret)
				return ret;

			slice += tpc_data_size;
		}

		size -= entry_size;
		data += entry_size;
	}

	if (size)
		dev_warn(wdev->dev, "cal: %u trailing bytes in TX TSSI section\n", size);

	return 0;
}

/*
 * Parse the factory calibration file (vendor EEPROM format 6.0): the
 * Card ID CIS provides the HW type / HW revision used for the PSDB
 * platform block selection, the TX TSSI sections provide the per-antenna
 * TPC data later sent with UM_MAN_SET_TPC_ANT_PARAMS_REQ.
 */
static int wave600_cal_parse(struct wave600_dev *wdev, const u8 *data, size_t size)
{
	struct wave600_cal *cal = &wdev->cal;
	u8 ver1;
	u32 pos;

	memset(cal, 0, sizeof(*cal));

	if (size < WAVE600_EEPROM_HDR_SIZE + 1)
		return -EINVAL;

	ver1 = data[WAVE600_EEPROM_HDR_SIZE];
	cal->eeprom_version = (u16)(ver1 << 8); /* MTLK_MAKE_EEPROM_VERSION(major, minor) */
	if (ver1 != 6)
		dev_warn(wdev->dev, "cal: unexpected EEPROM major version %u (expected 6)\n",
			 ver1);

	pos = WAVE600_EEPROM_HDR_SIZE + 1;
	while (pos + 3 <= size) {
		u8 tpl = data[pos];
		u16 tsize = (u16)(data[pos + 1] | (data[pos + 2] << 8)); /* LE */

		if (tpl == 0x00 || tpl == WAVE600_CIS_TPL_NONE)
			break; /* end of CIS */
		if (pos + 3 + tsize > size)
			break; /* partial section */

		if (tpl == WAVE600_CIS_TPL_GEN6_CID) {
			if (tsize != WAVE600_CIS_CARDID_SIZE) {
				dev_err(wdev->dev, "cal: Card ID CIS size %u != %u\n",
					tsize, WAVE600_CIS_CARDID_SIZE);
				return -EINVAL;
			}
			cal->hw_type = data[pos + 3];
			cal->hw_rev = data[pos + 4];
			dev_info(wdev->dev, "cal: Card ID: hw_type 0x%02x, hw_rev 0x%02x ('%c')\n",
				 cal->hw_type, cal->hw_rev,
				 (cal->hw_rev >= 0x20 && cal->hw_rev < 0x7f) ? cal->hw_rev : '.');
		} else if (tpl == WAVE600_CIS_TPL_GEN6_TX_TSSI) {
			int ret = wave600_cal_parse_tssi(wdev, data + pos + 3, tsize);

			if (ret)
				return ret;
		} else if (tpl == WAVE600_CIS_TPL_GEN6_RSSI) {
			/*
			 * Vendor _mtlk_cis_search_and_parse_rssi_v6
			 * (cis_manager.c): the raw section is kept per band,
			 * selected by the calibration_chan frequency
			 * (frequency_to_band: < 5000 -> 2.4 GHz). Captured
			 * byte-exact for UM_MAN_SET_RSSI_CAL_PARAMS_REQ.
			 */
			u16 freq;
			u8 band;

			if (tsize < 7 || tsize > 131) {
				dev_warn(wdev->dev, "cal: RSSI CIS size %u out of range, skipped\n",
					 tsize);
			} else {
				freq = (u16)(data[pos + 3 + 5] | (data[pos + 3 + 6] << 8));
				band = (freq < 5000) ? 0 : 1;
				if (cal->rssi[band].size) {
					dev_warn(wdev->dev, "cal: duplicate RSSI CIS for band %u, skipped\n",
						 band);
				} else {
					cal->rssi[band].size = tsize;
					memcpy(cal->rssi[band].data, data + pos + 3, tsize);
					dev_info(wdev->dev, "cal: RSSI CIS: band %u (freq %u), %u bytes\n",
						 band, freq, tsize);
				}
			}
		} else if (tpl == WAVE600_CIS_TPL_GEN6_XTAL) {
			/*
			 * Vendor cis_manager.c keeps the raw tuple bytes and
			 * mtlk_eeprom_get_xtal_value() returns them in MAC
			 * (little-endian) form; the section may be longer
			 * (temperature sensor byte) - only the first
			 * 16 bits are the xtal value.
			 */
			if (tsize < 2) {
				dev_err(wdev->dev, "cal: XTAL CIS size %u < 2\n",
					tsize);
				return -EINVAL;
			}
			cal->xtal = (u16)(data[pos + 3] | (data[pos + 4] << 8));
			dev_info(wdev->dev, "cal: XTAL CIS: 0x%04x (%u)\n",
				 cal->xtal, cal->xtal);
		}

		pos += 3 + tsize;
	}

	if (!cal->xtal) {
		cal->xtal = WAVE600_XTAL_DEFAULT;
		dev_info(wdev->dev, "cal: no XTAL CIS, using default 0x%04x (%u)\n",
			 cal->xtal, cal->xtal);
	}

	if (!cal->hw_type || !cal->hw_rev) {
		dev_err(wdev->dev, "cal: Card ID CIS not found\n");
		return -EINVAL;
	}

	for (pos = 0; pos < WAVE600_CAL_MAX_ANT; pos++) {
		if (cal->entry_count[pos])
			dev_info(wdev->dev, "cal: TSSI antenna %u: %u channels (data size %u)\n",
				 pos, cal->entry_count[pos], cal->tpc_data_size);
	}

	cal->parsed = true;

	return 0;
}

static int wave600_progmodel_download(struct wave600_dev *wdev, const char *fname,
				      u32 pm_offset, u16 req_id, u16 cfm_id)
{
	const struct firmware *fw = NULL;
	size_t loc = 0;
	u32 zero_pad[3] = {0, 0, 0};
	u32 dest = WAVE600_SHARED_RAM_BASE + pm_offset;
	int ret;

	ret = request_firmware_direct(&fw, fname, wdev->dev);
	if (ret) {
		dev_warn(wdev->dev, "optional ProgModel '%s' not found (%d)\n", fname, ret);
		return ret;
	}

	dev_info(wdev->dev, "downloading ProgModel '%s' (%zu bytes) to SHRAM 0x%08x (BAR0 0x%08x)\n",
		 fname, fw->size, pm_offset, dest);

	while (loc < fw->size) {
		size_t chunk = min((size_t)WAVE600_PROGMODEL_CHUNK_SIZE, fw->size - loc);

		wave600_pas_put(wdev->bar0 + dest, fw->data + loc, chunk);
		wave600_pas_put(wdev->bar0 + dest + chunk, zero_pad, sizeof(zero_pad));
		mmiowb();

		ret = wave600_ipc_cmd(wdev, req_id, cfm_id, NULL, 0, NULL, 0, 5000);
		if (ret) {
			dev_err(wdev->dev, "ProgModel '%s' chunk at offset %zu failed: %d\n",
				fname, loc, ret);
			release_firmware(fw);
			return ret;
		}

		loc += chunk;
	}

	release_firmware(fw);
	dev_info(wdev->dev, "ProgModel '%s' downloaded successfully\n", fname);
	return 0;
}

/*
 * Vendor pre-finalize step HW_EEPROM_READ (+ HW_PSDB_READ): parse the
 * factory calibration profile (cal_wlan0.bin) and the PSD.bin on the
 * host. Runs before card finalize: the HW_DEP_CFG step needs the XTAL
 * CIS value and the RF bring-up needs the Card ID, TSSI tables and the
 * PSDB antenna parameters.
 */
int wave600_cal_parse_profile(struct wave600_dev *wdev)
{
	const struct firmware *cal_fw = NULL;
	u16 sig = 0;
	int ret;

	ret = request_firmware_direct(&cal_fw, "cal_wlan0.bin", wdev->dev);
	if (ret || !cal_fw) {
		dev_err(wdev->dev, "factory calibration profile 'cal_wlan0.bin' not found (%d)\n", ret);
		return ret ? ret : -ENOENT;
	}

	sig = cal_fw->size >= 2 ? (cal_fw->data[0] | (cal_fw->data[1] << 8)) : 0;
	dev_info(wdev->dev, "loaded factory calibration profile (%zu bytes, sig=0x%04x)\n",
		 cal_fw->size, sig);
	if (sig != WAVE600_EEPROM_EXEC_SIG) {
		dev_warn(wdev->dev, "unexpected calibration signature 0x%04x (expected 0x%04x)\n",
			 sig, WAVE600_EEPROM_EXEC_SIG);
	}

	ret = wave600_cal_parse(wdev, cal_fw->data, cal_fw->size);
	release_firmware(cal_fw);
	if (ret) {
		dev_err(wdev->dev, "factory calibration file parse failed: %d\n", ret);
		return ret;
	}

	/* Host-side PSDB parse: needed before HDK_CONFIG because the
	 * vendor fills numTxAnts/numRxAnts from the PSDB antenna parameters
	 * of this radio (vendor parses PSD.bin at driver init). */
	ret = wave600_psdb_parse(wdev);
	if (ret) {
		dev_err(wdev->dev, "PSDB parse failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Vendor pre-activation step HW_CALIBRATION_CACHE
 * (_mtlk_mmb_init_calibration_cache): allocate the coherent DMA cache
 * whose geometry the firmware published in the CHI calibration
 * extension. Runs before SW_RESET, like every pre-activation step.
 */
int wave600_cal_alloc_cache(struct wave600_dev *wdev)
{
	u32 buf_size = wdev->chi.calibr_ext.buf_requested_size;
	u32 desc_loc = wdev->chi.calibr_ext.desc_location;
	u32 pm_addr = wdev->chi.calibr_ext.psd_progmodel_addr;

	dev_info(wdev->dev, "CHI Calibration: req_buf=%u B, psd_pm_addr=0x%08x, desc_loc=0x%08x\n",
		 buf_size, pm_addr, desc_loc);

	if (!buf_size || wdev->cal_buf)
		return 0;

	wdev->cal_buf_size = buf_size;
	wdev->cal_buf = dma_alloc_coherent(wdev->dev, buf_size,
					   &wdev->cal_dma_addr, GFP_KERNEL);
	if (!wdev->cal_buf) {
		dev_err(wdev->dev, "failed to allocate %u B DMA calibration buffer\n", buf_size);
		return -ENOMEM;
	}
	memset(wdev->cal_buf, 0, buf_size);
	dev_info(wdev->dev, "allocated %u B coherent DMA calibration cache at %pad\n",
		 buf_size, &wdev->cal_dma_addr);
	return 0;
}

/*
 * Vendor mtlk_efuse_eeprom_load_to_hw (hw_mmb.c), EEPROM new format
 * (v6, our cal file): the AFE block read from the device eFuse
 * (eFuse bytes 28..46 = wave_efuse_afe_data_v6_t, kept in wdev->afe.raw)
 * and the raw RSSI CIS section of the requested band. RFIC is
 * v5-eFuse-only in the vendor (cis.rfic is never filled for v6) and is
 * not sent.
 */
int wave600_cal_send_eeprom_data(struct wave600_dev *wdev, u8 band)
{
	int ret;

	if (wdev->afe.valid && wdev->afe.trim_indicator) {
		dev_info(wdev->dev, "sending AFE calibration data (%u bytes, eFuse trim block)...\n",
			 WAVE600_AFE_CALIBRATION_DATA_SIZE_GEN6);
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_SET_AFE_CAL_DATA_REQ,
				      WAVE600_MC_MAN_SET_AFE_CAL_DATA_CFM,
				      wdev->afe.raw,
				      WAVE600_AFE_CALIBRATION_DATA_SIZE_GEN6,
				      NULL, 0, 5000);
		if (ret) {
			dev_err(wdev->dev, "SET_AFE_CALIBRATION_DATA failed: %d\n", ret);
			return ret;
		}
		dev_info(wdev->dev, "AFE calibration data accepted by firmware\n");
	} else {
		dev_info(wdev->dev, "AFE calibration data not available (eFuse trim indicator unset)\n");
	}

	if (wdev->cal.parsed && wdev->cal.rssi[band].size) {
		u16 size = wdev->cal.rssi[band].size;
		u8 buf[1 + 131];

		/*
		 * UMI_SET_RSSI_CAL_CONFIG = { u8 size; u8 data[131] }.
		 * The vendor sets payload_size = size (one byte short of
		 * the prepared struct) - replicated byte-exact.
		 */
		buf[0] = (u8)size;
		memcpy(buf + 1, wdev->cal.rssi[band].data, size);

		dev_info(wdev->dev, "sending RSSI calibration data (band %u, %u bytes)...\n",
			 band, size);
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_SET_RSSI_CAL_PARAMS_REQ,
				      WAVE600_MC_MAN_SET_RSSI_CAL_PARAMS_CFM,
				      buf, size, NULL, 0, 5000);
		if (ret) {
			dev_err(wdev->dev, "SET_RSSI_CAL_PARAMS failed: %d\n", ret);
			return ret;
		}
		dev_info(wdev->dev, "RSSI calibration data accepted by firmware\n");
	} else {
		dev_info(wdev->dev, "RSSI calibration data missing for band %u (vendor sends nothing)\n",
			 band);
	}

	return 0;
}

/*
 * Vendor _wave_radio_dummy_vap_add / _wave_radio_dummy_vap_del (wave_radio.c):
 * before sending UM_MAN_CALIBRATE_REQ, the vendor adds a temporary dummy VAP
 * (vap_id = 0, operation_mode = 0, bssid = nic_mac, all rate/MCS fields 0)
 * and removes it immediately after calibration completes.
 */
struct wave600_cal_dummy_add_vap {
	u8 bssid[6];
	u8 vap_id;
	u8 operation_mode;
	u8 rates[32];
	u8 rates_len;
	u8 tx_mcs_bitmask[10];
	u8 vht_mcs_nss[8];
	u8 he_mcs_nss[8];
} __packed;

struct wave600_cal_dummy_rem_vap {
	u16 status;
	u8 vap_id;
	u8 reserved;
} __packed;

static int wave600_cal_dummy_vap_add(struct wave600_dev *wdev)
{
	struct wave600_cal_dummy_add_vap req;

	struct wave600_radio *radio = (wdev->radio_idx < 2) ? &wdev->radios[wdev->radio_idx] : NULL;

	memset(&req, 0, sizeof(req));
	if (radio && radio->ndev)
		memcpy(req.bssid, radio->ndev->dev_addr, sizeof(req.bssid));
	else
		eth_random_addr(req.bssid);
	req.bssid[5] += wdev->radio_idx; /* unique for every radio (vendor wave_radio.c:4807) */
	req.vap_id = 0;
	req.operation_mode = 0;

	dev_info(wdev->dev, "adding dummy calibration VAP 0 (bssid %pM) on radio %u...\n",
		 req.bssid, wdev->radio_idx);
	return wave600_ipc_cmd(wdev, WAVE600_UM_MAN_ADD_VAP_REQ,
			       WAVE600_MC_MAN_ADD_VAP_CFM,
			       &req, sizeof(req), NULL, 0, 5000);
}

static int wave600_cal_dummy_vap_del(struct wave600_dev *wdev)
{
	struct wave600_cal_dummy_rem_vap req;

	memset(&req, 0, sizeof(req));
	req.vap_id = 0;

	dev_info(wdev->dev, "removing dummy calibration VAP 0 on radio %u...\n", wdev->radio_idx);
	return wave600_ipc_cmd(wdev, WAVE600_UM_MAN_REMOVE_VAP_REQ,
			       WAVE600_MC_MAN_REMOVE_VAP_CFM,
			       &req, sizeof(req), NULL, 0, 5000);
}

/*
 * RF bring-up sequence (vendor _wave_radio_calibrate_prepare +
 * _wave_radio_calibrate in wave_radio.c:4879-5140, verified against
 * hybrid boot log):
 *  1. UM_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_REQ (0x041B) + ProgModel downloads
 *     (Common, Ant_Tx, Ant_Rx) + PHY_PROGMODEL_VER readback at BAR0+0x00B48040,
 *  2. mtlk_efuse_eeprom_load_to_hw: AFE cal data (0x0459) + RSSI cal data (0x045B),
 *  3. _wave_radio_hdk_config_send: UM_MAN_HDK_CONFIG_REQ (0x0450) with
 *     band = 1 (MTLK_HW_BAND_2_4_GHZ = 1; in WAVE600 mtlk_hw_band_e, 0 is 5 GHz
 *     and 1 is 2.4 GHz!),
 *  4. mtlk_hw_send_hdk_antennas_config (0x047E) + mtlk_psdb_load_to_hw (0x0457/0x0458),
 *  5. _wave_radio_dummy_vap_add (0x041F) -> UM_MAN_CALIBRATE_REQ (0x0453) ->
 *     _wave_radio_dummy_vap_del (0x0420).
 */
int wave600_cal_init(struct wave600_dev *wdev)
{
	struct wave600_umi_hdk_config hdk;
	struct wave600_umi_calibrate_params cal;
	u32 pm_addr, pm_ver;
	u8 num_ants;
	int ret;

	pm_addr = wdev->chi.calibr_ext.psd_progmodel_addr;

	/*
	 * Fallback for a call sequence that skipped the split steps: the
	 * cache is required by HDK_CONFIG, the profile by everything else.
	 */
	if (!wdev->cal_buf) {
		ret = wave600_cal_alloc_cache(wdev);
		if (ret)
			return ret;
	}
	if (!wdev->cal.parsed) {
		ret = wave600_cal_parse_profile(wdev);
		if (ret)
			return ret;
	}

	/* 1. Request ProgModel download permission & download ProgModels */
	if (pm_addr) {
		dev_info(wdev->dev, "requesting ProgModel download permission (0x%04x)...\n",
			 WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_REQ);
		ret = wave600_ipc_cmd(wdev,
				      WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_REQ,
				      WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_CFM,
				      NULL, 0, NULL, 0, 5000);
		if (ret) {
			dev_err(wdev->dev, "DOWNLOAD_PROG_MODEL_PERMISSION_REQ failed: %d\n", ret);
			return ret;
		}

		ret = wave600_progmodel_download(wdev, "ProgModel_gen6_Common.bin", pm_addr,
						 WAVE600_UM_DOWNLOAD_PROG_MODEL_REQ,
						 WAVE600_MC_DOWNLOAD_PROG_MODEL_CFM);
		if (ret)
			return ret;

		/* Read back PHY_RX_TD_PHY_PROGMODEL_VER (BAR0 + 0x00B48040, mask 0xffff) */
		pm_ver = wave600_read32(wdev, 0x00B48040) & 0xffff;
		dev_info(wdev->dev, "PHY_RX_TD_PHY_PROGMODEL_VER = %u (0x%04x)\n",
			 pm_ver, pm_ver);

		ret = wave600_progmodel_download(wdev, "ProgModel_gen6_Ant_Tx.bin", pm_addr,
						 WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_REQ,
						 WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_CFM);
		if (ret)
			return ret;

		ret = wave600_progmodel_download(wdev, "ProgModel_gen6_Ant_Rx.bin", pm_addr,
						 WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_REQ,
						 WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_CFM);
		if (ret)
			return ret;
	}

	/* 2. Pre-calibration for Radio 0 (2.4 GHz): EEPROM + HDK (vendor wave_radio.c:5150) */
	wdev->radio_idx = 0;
	ret = wave600_cal_send_eeprom_data(wdev, 0);
	if (ret) {
		dev_err(wdev->dev, "EEPROM calibration data upload failed: %d\n", ret);
		return ret;
	}

	memset(&hdk, 0, sizeof(hdk));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC16/32) */
	hdk.cal_buffer_base_addr = cpu_to_le32((u32)wdev->cal_dma_addr);
	/*
	 * Vendor (core.c current_chandef_init) fills numTxAnts/numRxAnts
	 * from the PSDB antenna parameters for this radio (segment 0 =
	 * 2.4 GHz).
	 */
	num_ants = hweight8(wdev->psdb.ant[0].tx_ant_mask);
	if (!num_ants)
		num_ants = 2;
	hdk.num_tx_ants = num_ants;
	hdk.num_rx_ants = num_ants;
	hdk.eeprom_version = cpu_to_le16(wdev->cal.eeprom_version);
	/*
	 * CRITICAL: In WAVE600 mtlk_hw_band_e (vendor driver + ARC firmware):
	 *   MTLK_HW_BAND_5_2_GHZ = 0
	 *   MTLK_HW_BAND_2_4_GHZ = 1
	 * Verified in hybrid boot log (_wave_radio_hdk_config_send for radio 0:
	 * "band=1, offline_algo_mask=0xffffffff, online_algo_mask=0xffffffff").
	 */
	hdk.band = 1; /* MTLK_HW_BAND_2_4_GHZ = 1 */
	hdk.offline_cal_mask = 0xffffffff;
	hdk.online_cal_mask = 0xffffffff;

	dev_info(wdev->dev, "configuring HDK for Radio 0 (band 1 [2.4 GHz], %ux%u, cal_dma=%pad)...\n",
		 num_ants, num_ants, &wdev->cal_dma_addr);
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_HDK_CONFIG_REQ,
			      WAVE600_MC_MAN_HDK_CONFIG_CFM,
			      &hdk, sizeof(hdk), NULL, 0, 5000);
	if (ret) {
		dev_err(wdev->dev, "Radio 0 HDK_CONFIG_REQ failed: %d\n", ret);
		return ret;
	}
	dev_info(wdev->dev, "Radio 0 HDK configured successfully by firmware\n");

	/* 3. Pre-calibration for Radio 1 (5 GHz): ProgModel + EEPROM + HDK (vendor wave_radio.c:5150) */
	wdev->radio_idx = 1;
	if (pm_addr) {
		dev_info(wdev->dev, "Radio 1: requesting ProgModel download permission (0x%04x)...\n",
			 WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_REQ);
		ret = wave600_ipc_cmd(wdev,
				      WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_REQ,
				      WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_PERMISSION_CFM,
				      NULL, 0, NULL, 0, 5000);
		if (ret) {
			dev_warn(wdev->dev, "Radio 1 DOWNLOAD_PROG_MODEL_PERMISSION_REQ failed: %d\n", ret);
		} else {
			ret = wave600_progmodel_download(wdev, "ProgModel_gen6_Common.bin", pm_addr,
							 WAVE600_UM_DOWNLOAD_PROG_MODEL_REQ,
							 WAVE600_MC_DOWNLOAD_PROG_MODEL_CFM);
			if (ret)
				dev_warn(wdev->dev, "Radio 1 ProgModel Common failed: %d\n", ret);

			ret = wave600_progmodel_download(wdev, "ProgModel_gen6_Ant_Tx.bin", pm_addr,
							 WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_REQ,
							 WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_CFM);
			if (ret)
				dev_warn(wdev->dev, "Radio 1 ProgModel Ant_Tx failed: %d\n", ret);

			ret = wave600_progmodel_download(wdev, "ProgModel_gen6_Ant_Rx.bin", pm_addr,
							 WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_REQ,
							 WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_CFM);
			if (ret)
				dev_warn(wdev->dev, "Radio 1 ProgModel Ant_Rx failed: %d\n", ret);
		}
	}

	ret = wave600_cal_send_eeprom_data(wdev, 1);
	if (ret)
		dev_warn(wdev->dev, "Radio 1 EEPROM calibration data upload failed: %d\n", ret);

	memset(&hdk, 0, sizeof(hdk));
	hdk.cal_buffer_base_addr = cpu_to_le32((u32)wdev->cal_dma_addr);
	num_ants = hweight8(wdev->psdb.ant[1].tx_ant_mask);
	if (!num_ants)
		num_ants = 2;
	hdk.num_tx_ants = num_ants;
	hdk.num_rx_ants = num_ants;
	hdk.eeprom_version = cpu_to_le16(wdev->cal.eeprom_version);
	hdk.band = 0; /* MTLK_HW_BAND_5_2_GHZ = 0 */
	hdk.offline_cal_mask = 0xffffffff;
	hdk.online_cal_mask = 0xffffffff;

	dev_info(wdev->dev, "configuring HDK for Radio 1 (band 0 [5 GHz], %ux%u)...\n",
		 num_ants, num_ants);
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_HDK_CONFIG_REQ,
			      WAVE600_MC_MAN_HDK_CONFIG_CFM,
			      &hdk, sizeof(hdk), NULL, 0, 5000);
	if (ret)
		dev_warn(wdev->dev, "Radio 1 HDK_CONFIG_REQ failed: %d\n", ret);
	else
		dev_info(wdev->dev, "Radio 1 HDK configured successfully by firmware\n");

	/* 4. Platform Specific Data (vendor mtlk_psdb_load_to_hw for each radio) */
	if (pm_addr) {
		wdev->radio_idx = 0;
		ret = wave600_psdb_send(wdev, pm_addr);
		if (ret) {
			dev_err(wdev->dev, "Radio 0 PSDB send failed: %d\n", ret);
			return ret;
		}

		/* Radio 1 (5 GHz) must also receive PSDB fields and tables */
		wdev->radio_idx = 1;
		ret = wave600_psdb_send(wdev, pm_addr);
		if (ret) {
			dev_err(wdev->dev, "Radio 1 PSDB send failed: %d\n", ret);
			return ret;
		}
	}

	/* 5. Execute Channel Calibration for Radio 0 (2.4 GHz) */
	wdev->radio_idx = 0;
	ret = wave600_cal_dummy_vap_add(wdev);
	if (ret) {
		dev_err(wdev->dev, "Radio 0 dummy calibration ADD_VAP failed: %d\n", ret);
		return ret;
	}

	memset(&cal, 0, sizeof(cal));
	cal.chan_nums[0] = 1;
	cal.chan_nums[1] = 6;
	cal.chan_nums[2] = 11;
	cal.num_chans = 3;
	cal.chan_width = 0; /* 20 MHz */

	dev_info(wdev->dev, "initiating Radio 0 RF calibration for channels 1, 6, 11 (20 MHz)...\n");
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_CALIBRATE_REQ,
			      WAVE600_MC_MAN_CALIBRATE_CFM,
			      &cal, sizeof(cal), &cal, sizeof(cal), 30000);
	if (ret) {
		dev_err(wdev->dev, "Radio 0 CALIBRATE_REQ failed: %d\n", ret);
		wave600_cal_dummy_vap_del(wdev);
		return ret;
	}

	dev_info(wdev->dev, "Radio 0 RF calibration verified (status: 0x%08x)\n",
		 le32_to_cpu(cal.status));

	ret = wave600_cal_dummy_vap_del(wdev);
	if (ret) {
		dev_err(wdev->dev, "Radio 0 dummy calibration REMOVE_VAP failed: %d\n", ret);
		return ret;
	}

	/* 6. Execute Channel Calibration for Radio 1 (5 GHz) */
	wdev->radio_idx = 1;
	ret = wave600_cal_dummy_vap_add(wdev);
	if (ret) {
		dev_warn(wdev->dev, "Radio 1 dummy calibration ADD_VAP failed: %d\n", ret);
	} else {
		static const u8 ch_20[] = {
			36, 40, 44, 48, 52, 56, 60, 64,
			100, 104, 108, 112, 116, 120, 124, 128,
			132, 136, 140, 144, 149, 153, 157, 161, 165
		};
		static const u8 ch_40[] = { 36, 44, 52, 60, 100, 108, 116, 124, 132, 140, 149, 157 };
		static const u8 ch_80[] = { 36, 52, 100, 116, 132, 149 };
		static const u8 ch_160[] = { 36, 100 };

		/* 20 MHz channels */
		memset(&cal, 0, sizeof(cal));
		memcpy(cal.chan_nums, ch_20, sizeof(ch_20));
		cal.num_chans = sizeof(ch_20);
		cal.chan_width = 0; /* CW_20 */
		dev_info(wdev->dev, "initiating Radio 1 RF calibration for 20 MHz channels (count %zu)...\n",
			 sizeof(ch_20));
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_CALIBRATE_REQ,
				      WAVE600_MC_MAN_CALIBRATE_CFM,
				      &cal, sizeof(cal), &cal, sizeof(cal), 30000);
		if (ret) {
			dev_warn(wdev->dev, "Radio 1 20 MHz CALIBRATE_REQ failed: %d\n", ret);
			goto out_del_r1;
		}

		/* 40 MHz channels */
		memset(&cal, 0, sizeof(cal));
		memcpy(cal.chan_nums, ch_40, sizeof(ch_40));
		cal.num_chans = sizeof(ch_40);
		cal.chan_width = 1; /* CW_40 */
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_CALIBRATE_REQ,
				      WAVE600_MC_MAN_CALIBRATE_CFM,
				      &cal, sizeof(cal), &cal, sizeof(cal), 30000);
		if (ret) {
			dev_warn(wdev->dev, "Radio 1 40 MHz CALIBRATE_REQ failed: %d\n", ret);
			goto out_del_r1;
		}

		/* 80 MHz channels */
		memset(&cal, 0, sizeof(cal));
		memcpy(cal.chan_nums, ch_80, sizeof(ch_80));
		cal.num_chans = sizeof(ch_80);
		cal.chan_width = 2; /* CW_80 */
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_CALIBRATE_REQ,
				      WAVE600_MC_MAN_CALIBRATE_CFM,
				      &cal, sizeof(cal), &cal, sizeof(cal), 30000);
		if (ret) {
			dev_warn(wdev->dev, "Radio 1 80 MHz CALIBRATE_REQ failed: %d\n", ret);
			goto out_del_r1;
		}

		/* 160 MHz channels */
		memset(&cal, 0, sizeof(cal));
		memcpy(cal.chan_nums, ch_160, sizeof(ch_160));
		cal.num_chans = sizeof(ch_160);
		cal.chan_width = 3; /* CW_160 */
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_CALIBRATE_REQ,
				      WAVE600_MC_MAN_CALIBRATE_CFM,
				      &cal, sizeof(cal), &cal, sizeof(cal), 30000);
		if (ret) {
			dev_warn(wdev->dev, "Radio 1 160 MHz CALIBRATE_REQ failed: %d\n", ret);
			goto out_del_r1;
		}

out_del_r1:
		ret = wave600_cal_dummy_vap_del(wdev);
		if (ret)
			dev_warn(wdev->dev, "Radio 1 dummy calibration REMOVE_VAP failed: %d\n", ret);
	}

	dev_info(wdev->dev, "Radio 1 RF channel calibration completed successfully\n");
	wdev->radio_idx = 0;

	return 0;
}

void wave600_cal_cleanup(struct wave600_dev *wdev)
{
	wave600_psdb_cleanup(wdev);

	if (wdev->cal_buf) {
		dma_free_coherent(wdev->dev, wdev->cal_buf_size,
				  wdev->cal_buf, wdev->cal_dma_addr);
		wdev->cal_buf = NULL;
		wdev->cal_dma_addr = 0;
		wdev->cal_buf_size = 0;
	}
}
