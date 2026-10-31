// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Platform Specific Data loader
 *
 * The vendor driver (lq-wave-300) parses the platform file PSD.bin on the
 * host and pushes it to the ARC firmware before RF calibration
 * (wave_radio.c: _wave_radio_calibrate_prepare):
 *
 *   1. UM_MAN_HDK_ANT_CONFIG_REQ (0x047E) - per-radio antenna masks
 *      (vendor mtlk_hw_send_hdk_antennas_config)
 *   2. UM_MAN_PLATFORM_DATA_FIELDS_REQ (0x0457) - single-word platform
 *      fields (vendor mtlk_hw_psdb_send_fields)
 *   3. UM_MAN_PLATFORM_TABLE_REQ (0x0458) - one per referenced data
 *      table, table body staged through the SHRAM ProgModel area
 *      (vendor mtlk_hw_psdb_send_table)
 *
 * The platform block inside PSD.bin is selected by chip ID plus the HW
 * type / HW revision taken from the calibration file Card ID CIS
 * (vendor _mtlk_hw_fill_card_info uses mtlk_eeprom_get_nic_type /
 * mtlk_eeprom_get_nic_revision). On the Archer AX50 v1 the factory
 * calibration file carries type 0x61, revision 0x46 ('F').
 *
 * PSD.bin words are little-endian (ARC "MAC endian" in the vendor code,
 * parsed there with le32_to_cpu on the Big-Endian host). Every blob is
 * copied to the firmware byte-exact, matching the vendor wordcpy()
 * without endian conversion; the mailbox payload writer (memcpy_toio)
 * preserves the byte order.
 *
 * The split into a host-side parse (wave600_psdb_parse) and a firmware
 * download (wave600_psdb_send) follows the vendor flow: the PSDB antenna
 * data is needed for HDK_CONFIG before the platform data itself is sent
 * after the ProgModel download.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/unaligned.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_ipc.h"
#include "wave600_cal.h"

#define WAVE600_PSDB_FILE_NAME             "PSD.bin"

#define WAVE600_PSDB_PLATFORM_MARKER       0xAAAA5555
#define WAVE600_PSDB_TABLE_MARKER          0x66669999

/* Field IDs of single-word fields (vendor mtlk_psdb.c) */
#define WAVE600_PSDB_FIELD_CHIP_ID         0x01
#define WAVE600_PSDB_FIELD_CHIP_REV        0x02
#define WAVE600_PSDB_FIELD_HW_TYPE         0x0B
#define WAVE600_PSDB_FIELD_HW_REV          0x0C
#define WAVE600_PSDB_FIELD_TX_ANT_MASK     0x06

/* Field IDs at or above this value are table references */
#define WAVE600_PSDB_FIELD_TABLE_ID_MIN    0x010000

/* Tables kept on the host only, never sent to firmware (vendor) */
#define WAVE600_PSDB_FIELD_HW_POWER_LIMIT  0x010000
#define WAVE600_PSDB_FIELD_CCA_TH_CFG      0x330000

/* Tables needed on the host to derive the antenna configuration */
#define WAVE600_PSDB_FIELD_CDB_CFG         0x2F0000
#define WAVE600_PSDB_FIELD_ANT_MASK_PER_CH 0x2A0000

/* CDB configuration values (vendor mtlk_psdb.h wave_cdb_cfg_e) */
#define WAVE600_CDB_CFG_INVALID            0
#define WAVE600_CDB_CFG_2G                 1
#define WAVE600_CDB_CFG_5G                 2
#define WAVE600_CDB_CFG_5G_LOW             3
#define WAVE600_CDB_CFG_5G_HIGH            4
#define WAVE600_CDB_CFG_ALTERN             5
#define WAVE600_CDB_CFG_NOT_SET            6

/* REGD table entry fields (vendor mtlk_psdb.h psdf_regd_entry_t) */
#define WAVE600_PSDB_PHY_MODE_11B          0
#define WAVE600_PSDB_PHY_MODE_OFDM         1
#define WAVE600_PSDB_PHY_MODE_MU           2
#define WAVE600_PSDB_PHY_MODE_BF           3

#define WAVE600_CDB_NUM_OF_SEGMENTS        2

/*
 * antennaParameters_t / UMI_HDK_SET_ANTENNA_REQ (vendor mhi_umi.h)
 */
struct wave600_umi_hdk_set_antenna {
	struct wave600_antenna_params ant_params[WAVE600_CDB_NUM_OF_SEGMENTS];
} __packed;

static u32 psdb_rd32(const u8 *p)
{
	return get_unaligned_le32(p);
}

static void psdb_wr32(u8 *p, u32 v)
{
	put_unaligned_le32(v, p);
}

/* vendor _wave_psdb_ant_mask_to_ant_sel_mask */
static u8 psdb_ant_mask_to_sel_mask(u8 ant_mask)
{
	u8 sel_mask = 0;
	int i;

	for (i = 0; i < 4; i++) {
		if (ant_mask & BIT(i))
			sel_mask |= BIT(2 * i);
	}

	return sel_mask;
}

/* Lower channel of a CDB segment band (vendor cdb_cfg_radio_params_table) */
static int psdb_cdb_cfg_lower_chan(u32 cdb_cfg)
{
	switch (cdb_cfg) {
	case WAVE600_CDB_CFG_2G:
		return 1;
	case WAVE600_CDB_CFG_5G:
	case WAVE600_CDB_CFG_5G_LOW:
		return 36;
	case WAVE600_CDB_CFG_5G_HIGH:
		return 100;
	default:
		return 0; /* none */
	}
}

/*
 * Antenna mask for a channel from the AntennaMaskPerChannel table
 * (vendor wave_psdb_get_ant_mask_by_channel, strict '<' comparison).
 * Returns 0 when the table is absent (caller falls back to the card mask).
 */
static u8 psdb_ant_mask_by_channel(const u8 *ampc_tab, size_t size, u32 channel)
{
	u32 nrows, ncols, i;

	if (!ampc_tab || size < 12)
		return 0;

	nrows = psdb_rd32(ampc_tab + 4);
	ncols = psdb_rd32(ampc_tab + 8);
	if (nrows == 0 || ncols != 2 || 12 + nrows * 8 > size)
		return 0;

	for (i = 0; i < nrows; i++) {
		u32 upper = psdb_rd32(ampc_tab + 12 + i * 8);
		u32 mask = psdb_rd32(ampc_tab + 12 + i * 8 + 4);

		if (channel < upper)
			return (u8)mask;
	}

	return 0;
}

/*
 * Host-side parse of PSD.bin (vendor mtlk_psdb_file_read_and_parse +
 * psdb_parse_fields_array): find the platform block for this chip and
 * the HW type / HW revision from the calibration file Card ID, build the
 * fields message, the table list, the antenna configuration and keep the
 * REGD table offset for the TX power limits.
 *
 * Missing file is non-fatal (vendor "Continue without the PSDB binary
 * file"); a malformed file is a hard error.
 */
int wave600_psdb_parse(struct wave600_dev *wdev)
{
	struct wave600_psdb *psdb = &wdev->psdb;
	const struct firmware *fw = NULL;
	const u8 *buf;
	size_t size, words;
	u32 fda = 0, nrows, ncols, nfld = 0;
	u32 card_tx_mask = 0;
	const u8 *cdb_tab = NULL, *ampc_tab = NULL;
	size_t cdb_tab_size = 0, ampc_tab_size = 0;
	u32 i, k;
	int ret;

	memset(psdb, 0, sizeof(*psdb));

	ret = request_firmware_direct(&fw, WAVE600_PSDB_FILE_NAME, wdev->dev);
	if (ret) {
		dev_warn(wdev->dev, "platform data '%s' not found (%d), continuing without PSDB\n",
			 WAVE600_PSDB_FILE_NAME, ret);
		return 0;
	}

	buf = fw->data;
	size = fw->size;
	words = size / 4;
	if (!words || size % 4) {
		dev_err(wdev->dev, "PSDB: wrong file size %zu\n", size);
		ret = -EINVAL;
		goto err;
	}

	/* Find the platform fields array for this chip and board
	 * (vendor psdb_search_fields_array) */
	for (i = 0; i + 3 < words; i++) {
		u32 id0, val0, id1;
		bool found_type = false, found_rev = false;

		if (psdb_rd32(buf + i * 4) == WAVE600_PSDB_TABLE_MARKER)
			break; /* platform data ends at the first table marker */
		if (psdb_rd32(buf + i * 4) != WAVE600_PSDB_PLATFORM_MARKER)
			continue;

		nrows = psdb_rd32(buf + i * 4 + 4);
		ncols = psdb_rd32(buf + i * 4 + 8);
		if (ncols != 2 || nrows < 2 || nrows > 62 ||
		    i * 4 + 12 + (size_t)nrows * 8 > size)
			continue;

		id0 = psdb_rd32(buf + i * 4 + 12);
		val0 = psdb_rd32(buf + i * 4 + 16);
		id1 = psdb_rd32(buf + i * 4 + 20);
		if (id0 != WAVE600_PSDB_FIELD_CHIP_ID ||
		    id1 != WAVE600_PSDB_FIELD_CHIP_REV)
			continue;
		if (val0 != wdev->chip_id)
			continue;

		for (k = 0; k < nrows; k++) {
			u32 id = psdb_rd32(buf + i * 4 + 12 + k * 8);
			u32 val = psdb_rd32(buf + i * 4 + 12 + k * 8 + 4);

			if (id == WAVE600_PSDB_FIELD_HW_TYPE &&
			    val == wdev->cal.hw_type)
				found_type = true;
			else if (id == WAVE600_PSDB_FIELD_HW_REV &&
				 val == wdev->cal.hw_rev)
				found_rev = true;
		}
		if (found_type && found_rev) {
			fda = i * 4;
			break;
		}
	}

	if (!fda) {
		dev_err(wdev->dev, "PSDB: no platform block for chip 0x%04x hw_type 0x%02x hw_rev 0x%02x\n",
			wdev->chip_id, wdev->cal.hw_type, wdev->cal.hw_rev);
		ret = -ENOENT;
		goto err;
	}

	/* Collect single-word fields and table references in file order
	 * (vendor psdb_parse_fields_array) */
	nrows = psdb_rd32(buf + fda + 4);
	nfld = 0;
	psdb->ntab = 0;
	psdb->regd_off = 0;
	for (k = 0; k < nrows; k++) {
		const u8 *fb = buf + fda + 12 + k * 8;
		u32 id = psdb_rd32(fb);
		u32 off = psdb_rd32(fb + 4);

		if (id < WAVE600_PSDB_FIELD_TABLE_ID_MIN) {
			if (nfld >= 62) {
				dev_err(wdev->dev, "PSDB: too many fields\n");
				ret = -EINVAL;
				goto err;
			}
			/* Raw little-endian block, byte-exact into the message */
			memcpy(psdb->fields_msg + 8 + nfld * 8, fb, 8);
			nfld++;

			if (id == WAVE600_PSDB_FIELD_TX_ANT_MASK)
				card_tx_mask = off;
		} else if (off) {
			if (id == WAVE600_PSDB_FIELD_HW_POWER_LIMIT) {
				psdb->regd_off = off;
				continue; /* host-only table */
			}
			if (id == WAVE600_PSDB_FIELD_CCA_TH_CFG)
				continue; /* host-only table */
			if (psdb->ntab >= WAVE600_PSDB_MAX_TABLES) {
				dev_err(wdev->dev, "PSDB: too many tables\n");
				ret = -EINVAL;
				goto err;
			}
			psdb->tabs[psdb->ntab].id = id;
			psdb->tabs[psdb->ntab].off = off;
			psdb->ntab++;

			if (off + 12 > size)
				continue; /* table header out of bounds */

			if (id == WAVE600_PSDB_FIELD_CDB_CFG) {
				cdb_tab = buf + off;
				cdb_tab_size = size - off;
			} else if (id == WAVE600_PSDB_FIELD_ANT_MASK_PER_CH) {
				ampc_tab = buf + off;
				ampc_tab_size = size - off;
			}
		}
	}

	psdb_wr32(psdb->fields_msg, nfld);
	psdb_wr32(psdb->fields_msg + 4, 2);
	psdb->fields_msg_len = 8 + nfld * 8;

	dev_info(wdev->dev, "PSDB: platform block at 0x%05x for chip 0x%04x hw_type 0x%02x hw_rev 0x%02x (%u fields, %u tables)\n",
		 fda, wdev->chip_id, wdev->cal.hw_type, wdev->cal.hw_rev,
		 nfld, psdb->ntab);

	/* Derive the per-segment antenna parameters exactly like the vendor
	 * _mtlk_psdb_update_cdb_cfg_table: the CDB config table defines the
	 * radio segments, each segment takes its antenna mask from the
	 * AntennaMaskPerChannel table at the segment's lower channel, RX
	 * mirrors TX, selection masks are derived from the antenna mask. */
	for (k = 0; k < WAVE600_CDB_NUM_OF_SEGMENTS; k++) {
		struct wave600_antenna_params *seg = &psdb->ant[k];
		u32 cdb_cfg = WAVE600_CDB_CFG_NOT_SET;
		int lower_chan;
		u8 mask;

		if (cdb_tab) {
			u32 t_nrows = psdb_rd32(cdb_tab + 4);
			u32 t_ncols = psdb_rd32(cdb_tab + 8);

			if (t_nrows == 1 && t_ncols == WAVE600_CDB_NUM_OF_SEGMENTS &&
			    12 + t_nrows * t_ncols * 4 <= cdb_tab_size)
				cdb_cfg = psdb_rd32(cdb_tab + 12 + k * 4);
		}

		lower_chan = psdb_cdb_cfg_lower_chan(cdb_cfg);
		if (lower_chan)
			mask = psdb_ant_mask_by_channel(ampc_tab, ampc_tab_size,
							lower_chan);
		else
			mask = 0;
		if (!mask)
			mask = card_tx_mask; /* vendor default mask */

		if (!mask) {
			dev_err(wdev->dev, "PSDB: segment %u antenna mask is zero\n", k);
			ret = -EINVAL;
			goto err;
		}

		seg->tx_ant_mask = mask;
		seg->rx_ant_mask = mask;
		seg->tx_ant_sel_mask = psdb_ant_mask_to_sel_mask(mask);
		seg->rx_ant_sel_mask = seg->tx_ant_sel_mask;

		dev_info(wdev->dev, "PSDB: radio segment %u (cdb_cfg=%u): tx=0x%02x rx=0x%02x sel=0x%02x\n",
			 k, cdb_cfg, seg->tx_ant_mask, seg->rx_ant_mask,
			 seg->tx_ant_sel_mask);
	}

	psdb->fw = fw;
	psdb->fda = fda;
	psdb->parsed = true;
	return 0;

err:
	release_firmware(fw);
	return ret;
}

/*
 * Download the platform data to the firmware: antenna configuration,
 * single-word fields, then every referenced table through the SHRAM
 * ProgModel area (vendor mtlk_psdb_load_to_hw after the ProgModels).
 */
int wave600_psdb_send(struct wave600_dev *wdev, u32 pm_addr)
{
	struct wave600_psdb *psdb = &wdev->psdb;
	struct wave600_umi_hdk_set_antenna ant;
	const u8 *buf;
	size_t size;
	u32 k;
	int ret;

	if (!psdb->parsed) {
		dev_warn(wdev->dev, "PSDB: nothing to send (not parsed)\n");
		return 0;
	}

	buf = psdb->fw->data;
	size = psdb->fw->size;

	/* 1. HDK antenna configuration (vendor sends it before the PSDB
	 * data). Byte fields only, no endianness involved. */
	memset(&ant, 0, sizeof(ant));
	ant.ant_params[0] = psdb->ant[0];
	ant.ant_params[1] = psdb->ant[1];

	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_HDK_ANT_CONFIG_REQ,
			      WAVE600_MC_MAN_HDK_ANT_CONFIG_CFM,
			      &ant, sizeof(ant), NULL, 0, 5000);
	if (ret) {
		dev_err(wdev->dev, "PSDB: HDK_ANT_CONFIG_REQ failed: %d\n", ret);
		goto out;
	}
	dev_info(wdev->dev, "PSDB: HDK antenna configuration confirmed by firmware\n");

	/* 2. Single-word platform fields (little-endian words, raw bytes) */
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_PLATFORM_DATA_FIELDS_REQ,
			      WAVE600_MC_MAN_PLATFORM_DATA_FIELDS_CFM,
			      psdb->fields_msg, psdb->fields_msg_len,
			      NULL, 0, 5000);
	if (ret) {
		dev_err(wdev->dev, "PSDB: PLATFORM_DATA_FIELDS_REQ failed: %d\n", ret);
		goto out;
	}
	dev_info(wdev->dev, "PSDB: platform fields (%u words) confirmed by firmware\n",
		 (unsigned)(psdb->fields_msg_len / 4));

	/* 3. Data tables: stage the table body [nrows][ncols][data] through
	 * the SHRAM ProgModel area byte-exact, then hand over the table ID. */
	for (k = 0; k < psdb->ntab; k++) {
		u32 off = psdb->tabs[k].off;
		u32 t_nrows, t_ncols, t_bytes;
		u8 tid[4];

		if (off + 12 > size || psdb_rd32(buf + off) != WAVE600_PSDB_TABLE_MARKER) {
			dev_err(wdev->dev, "PSDB: table 0x%06x at 0x%x has no table marker\n",
				psdb->tabs[k].id, off);
			ret = -EINVAL;
			goto out;
		}

		t_nrows = psdb_rd32(buf + off + 4);
		t_ncols = psdb_rd32(buf + off + 8);
		t_bytes = (t_nrows * t_ncols + 2) * 4;
		if (t_bytes > 8000 || off + 4 + t_bytes > size) {
			dev_err(wdev->dev, "PSDB: table 0x%06x bad size %u x %u\n",
				psdb->tabs[k].id, t_nrows, t_ncols);
			ret = -EINVAL;
			goto out;
		}

		wave600_pas_put(wdev->bar0 + WAVE600_SHARED_RAM_BASE + pm_addr,
				buf + off + 4, t_bytes);
		mmiowb();

		psdb_wr32(tid, psdb->tabs[k].id);
		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_PLATFORM_TABLE_REQ,
				      WAVE600_MC_MAN_PLATFORM_TABLE_CFM,
				      tid, sizeof(tid), NULL, 0, 5000);
		if (ret) {
			dev_err(wdev->dev, "PSDB: PLATFORM_TABLE_REQ 0x%06x failed: %d\n",
				psdb->tabs[k].id, ret);
			goto out;
		}

		dev_info(wdev->dev, "PSDB: table 0x%06x (%u x %u, %u bytes) confirmed by firmware\n",
			 psdb->tabs[k].id, t_nrows, t_ncols, t_bytes);
	}

	dev_info(wdev->dev, "PSDB platform data loaded to firmware (%u tables)\n",
		 psdb->ntab);
	ret = 0;

out:
	/*
	 * Keep the PSD.bin firmware buffer until driver cleanup: the REGD
	 * table is re-read by the TX power configuration on every channel
	 * set (vendor keeps its parsed PSDB structures for the same reason).
	 */
	psdb->parsed = false;
	return ret;
}

void wave600_psdb_cleanup(struct wave600_dev *wdev)
{
	struct wave600_psdb *psdb = &wdev->psdb;

	if (psdb->fw) {
		release_firmware(psdb->fw);
		psdb->fw = NULL;
	}
	psdb->parsed = false;
	psdb->regd_off = 0;
}

/*
 * Channel power limits from the REGD table (vendor psdb_get_hw_limits):
 * entries of the given regulatory domain code for the channel frequency,
 * antenna factor of the channel's radio segment added, MU/BF falling
 * back to OFDM when the table has no value.
 */
int wave600_psdb_regd_limits(struct wave600_dev *wdev, u8 regd_code, u8 chan_num,
			     u16 pw[13])
{
	struct wave600_psdb *psdb = &wdev->psdb;
	const u8 *tab;
	u32 nrows, ncols, i;
	u16 freq;
	u16 factor;
	int seg;
	u8 ant_num;
	static const int ant_factor_table[5] = {-1, 0, 24, 38, 48};

	if (!psdb->regd_off)
		return -ENOENT;

	tab = psdb->fw ? psdb->fw->data : NULL;
	if (!tab) {
		dev_err(wdev->dev, "PSDB: firmware buffer released before regd use\n");
		return -ENOENT;
	}

	/* REGD table must still be inside the kept buffer */
	if (psdb->regd_off + 12 > psdb->fw->size)
		return -ENOENT;

	if (psdb_rd32(tab + psdb->regd_off) != WAVE600_PSDB_TABLE_MARKER)
		return -ENOENT;

	nrows = psdb_rd32(tab + psdb->regd_off + 4);
	ncols = psdb_rd32(tab + psdb->regd_off + 8);
	if (ncols != 5 || psdb->regd_off + 12 + (size_t)nrows * 20 > psdb->fw->size)
		return -ENOENT;

	memset(pw, 0, 13 * sizeof(pw[0]));
	freq = (chan_num <= 14) ? (2407 + 5 * chan_num) : (5000 + 5 * chan_num);

	for (i = 0; i < nrows; i++) {
		const u8 *e = tab + psdb->regd_off + 12 + i * 20;
		u32 code = psdb_rd32(e);
		u32 e_freq = psdb_rd32(e + 4);
		u32 chanwidth = psdb_rd32(e + 8);
		u32 phy_mode = psdb_rd32(e + 12);
		u32 pwl = psdb_rd32(e + 16);
		int slot = -1;

		if (code != regd_code || e_freq != freq)
			continue;

		/* vendor _get_regd_entry_phy_cw */
		if (phy_mode == WAVE600_PSDB_PHY_MODE_11B && chanwidth == 20) {
			slot = 0;
		} else if (phy_mode == WAVE600_PSDB_PHY_MODE_OFDM ||
			   phy_mode == WAVE600_PSDB_PHY_MODE_MU ||
			   phy_mode == WAVE600_PSDB_PHY_MODE_BF) {
			int base = (phy_mode == WAVE600_PSDB_PHY_MODE_OFDM) ? 1 :
				   (phy_mode == WAVE600_PSDB_PHY_MODE_MU) ? 5 : 9;

			switch (chanwidth) {
			case 20:  slot = base + 0; break;
			case 40:  slot = base + 1; break;
			case 80:  slot = base + 2; break;
			case 160: slot = base + 3; break;
			default:  slot = -1; break;
			}
		}

		if (slot >= 0 && pwl <= 0xff)
			pw[slot] = (u16)pwl;
	}

	/* Antenna factor of the channel's radio segment
	 * (vendor mtlk_antennas_factor of the segment TX antennas number) */
	seg = (chan_num <= 14) ? 0 : 1;
	ant_num = hweight8(psdb->ant[seg].tx_ant_mask);
	if (ant_num < ARRAY_SIZE(ant_factor_table))
		factor = (u16)ant_factor_table[ant_num];
	else
		factor = 0;

	for (i = 0; i < 13; i++) {
		if (pw[i])
			pw[i] += factor;
	}

	/* If the MU/BF values are not configured, use the OFDM values */
	for (i = 0; i < 4; i++) {
		if (pw[5 + i] == 0)
			pw[5 + i] = pw[1 + i];
		if (pw[9 + i] == 0)
			pw[9 + i] = pw[1 + i];
	}

	return 0;
}
