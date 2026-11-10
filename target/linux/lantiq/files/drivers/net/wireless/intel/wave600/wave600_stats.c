// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) FW Statistics Dump (diagnostic)
 *
 * Mirrors vendor hw_mmb.c _mtlk_hw_mhi_get_table() / mtlk_hw_mhi_get_stats():
 * the firmware publishes the counter layout descriptor table through
 * UM_MAN_GET_STATISTICS_TABLE_REQ (0x042E) and the counter values through
 * UM_MAN_GET_STATISTICS_DATA_REQ (0x0460). Both carry the vendor
 * UMI_GET_STATISTICS_DATA payload with a DDR buffer that the firmware fills.
 * Buffer geometry comes from the CHI vector area extension 7
 * (statistics manager, vendor shram.h VECTOR_AREA_STATISTICS_MANAGER).
 *
 * Endianness: the mailbox request fields (DDR buffer address, length) and
 * the descriptor-table u16 fields are MAC-endian (little-endian), exactly
 * like vendor MAC_TO_HOST16/32. The FW writes the table and counter
 * database into host DDR; the table entry u16 offset/dimension fields must
 * not be byte-swapped as BE.
 *
 * The request confirmation may race the firmware DMA (the cfm can arrive
 * before the DMA write lands), so the dump sleeps briefly after each
 * confirmation before reading the buffer back.
 *
 * The dump never fails after a successful request: on parse problems the
 * collected raw evidence is returned instead of -ENOENT, so a plain
 *   cat /sys/bus/pci/devices/0000:01:00.0/fw_stats
 * always prints something useful.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/sysfs.h>
#include "wave600.h"
#include "wave600_ipc.h"

/* vendor mhi_statistics_manager_enums.h StatisticType_e (subset) */
#define WAVE600_STAT_TYPE_HOSTIF_COUNTERS	0
#define WAVE600_STAT_TYPE_BAA_COUNTERS		2
#define WAVE600_STAT_TYPE_LINK_ADAPTATION	7
#define WAVE600_STAT_TYPE_GENERAL_STATS		14

/* vendor LinkAdaptationStats_e subtypes carrying actual FW TX power.
 * unitSize=1 byte per entry, STAT_PW_PER_DBM=2 -> byte value / 2 = dBm.
 * Block layout: num_radios x PHY_STAT_RATE_CBW_NUM(4) bytes (per 20/40/80/160). */
#define W600_LA_POWER_DATA	6
#define W600_LA_POWER_MANAGEMENT 7
#define W600_LA_MIN_POWER	21
#define W600_LA_MAX_POWER	22
#define W600_STAT_PW_PER_DBM	2	/* vendor mtlk_coreui.h */

/*
 * vendor mhi_statistics_manager_defines.h StatsEntryDecoded_t (8 bytes).
 * Byte 0 bitfield (same physical layout for BE and LE bitfield code):
 *   [reserved:1][unitSize:3][type:4]
 * Byte 1 bitfield:
 *   [numOfDimensions:2][subtype:6]
 * The u16 fields (offset, dimension1) are little-endian.
 */
struct wave600_stats_entry {
	u8 flags;
	u8 dim_flags;
	u8 offset_raw[2];
	u8 dim1_raw[2];
	u8 dimension2;
	u8 next_idx_offset;
} __packed;

/* vendor mhi_statistics_manager_enums.h GeneralStats subtype names */
static const char * const stat_general_names[] = {
	"offline_cal_mask", "online_cal_mask", "drop_classifier",
	"drop_disconnect", "drop_atf", "drop_ts_flush", "drop_rekey",
	"drop_set_key", "drop_discard", "drop_disabled", "drop_agg_error",
	"defrag_start", "defrag_end", "defrag_timeout", "class_violation",
	"rxh_errors", "rx_smps", "rx_omn", "tx_mng_from_host",
	"tx_data_from_host", "cyclic_buffer_overflow",
	"beacon_transmitted", "debug_statistics",
};

/* vendor mhi_statistics_manager_enums.h BaaCounters subtype names (subset) */
static const struct {
	unsigned int subtype;
	const char *name;
} stat_baa_named[] = {
	{  0, "retry_count" },
	{ 27, "fw_mngmnt_frames_sent" },
	{ 28, "fw_mngmnt_frames_confirmed" },
	{ 29, "fw_mngmnt_frames_received" },
	{ 37, "tx_frame_error_count" },
	{ 39, "tx_sender_error" },
};

/* vendor mhi_statistics_manager_enums.h HostIfCounters subtype names (subset) */
static const char * const stat_hostif_names[] = {
	"qos_byte_count_sta", "qos_tx_sta", "qos_tx_vap",
	"tx_in_unicast_hd", "tx_in_multicast_hd", "tx_in_broadcast_hd",
};

static DEFINE_MUTEX(wave600_stats_lock);

static u16 stat_le16(const u8 *p)
{
	return (u16)(p[0] | (p[1] << 8));
}

static u8 stat_entry_type(const struct wave600_stats_entry *e)
{
	return e->flags & 0x0f;
}

static u8 stat_entry_unit(const struct wave600_stats_entry *e)
{
	return (e->flags >> 4) & 0x07;
}

static u8 stat_entry_subtype(const struct wave600_stats_entry *e)
{
	return e->dim_flags & 0x3f;
}

static u8 stat_entry_dims(const struct wave600_stats_entry *e)
{
	return (e->dim_flags >> 6) & 3;
}

static u16 stat_entry_offset(const struct wave600_stats_entry *e)
{
	return stat_le16(e->offset_raw);
}

static u16 stat_entry_dim1(const struct wave600_stats_entry *e)
{
	return stat_le16(e->dim1_raw);
}

static u32 stat_entry_bytes(const struct wave600_stats_entry *e)
{
	return (u32)stat_entry_unit(e) * stat_entry_dim1(e) * e->dimension2;
}

static bool stat_entry_fits(const struct wave600_stats_entry *e, u32 db_size)
{
	u64 end = (u64)stat_entry_offset(e) + stat_entry_bytes(e);

	return end <= db_size;
}

/*
 * Find the first table entry index of a statistics type. The CHI
 * type_table_list[] holds byte offsets from the start of the descriptor
 * table, exactly like vendor _mtlk_hw_mhi_parse_table.
 */
static int stat_find_type(const struct wave600_stats_entry *tab, u32 tab_num,
			  const struct wave600_chi_stats_mgr_ext *ext,
			  unsigned int type)
{
	u32 start;
	int i;

	if (type >= ARRAY_SIZE(ext->type_table_list))
		return -1;

	start = ext->type_table_list[type];
	if (start > tab_num * sizeof(*tab) || start % sizeof(*tab))
		return -1;

	i = start / sizeof(*tab);
	if (i >= (int)tab_num)
		return -1;

	if (stat_entry_type(&tab[i]) == type)
		return i;

	return -1;
}

/* Counter value zero-extended per little-endian unit size */
static void stat_counter_le(const u8 *db, u32 db_size,
			    const struct wave600_stats_entry *e,
			    u32 *value, u8 raw[4])
{
	u32 offset = stat_entry_offset(e);
	u32 unit = stat_entry_unit(e);
	u32 v = 0;
	int i;

	memset(raw, 0, 4);
	*value = 0;

	if (unit == 0 || unit > 4 || offset + unit > db_size)
		return;

	memcpy(raw, db + offset, unit);
	for (i = unit - 1; i >= 0; i--)
		v = (v << 8) | db[offset + i];
	*value = v;
}

static int wave600_stats_send(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
			      dma_addr_t dma, u16 len)
{
	struct wave600_umi_get_statistics req;
	struct wave600_umi_get_statistics cfm;
	int ret;

	memset(&req, 0, sizeof(req));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC16/32) */
	req.ddr_buffer_address = cpu_to_le32((u32)dma);
	req.length = cpu_to_le16((u16)len);

	ret = wave600_ipc_cmd(wdev, req_id, cfm_id, &req, sizeof(req),
			      &cfm, sizeof(cfm), 5000);
	if (ret)
		return ret;

	if (cfm.status != 0) {
		dev_err(wdev->dev, "fw_stats: request 0x%04x rejected by firmware (status=%u)\n",
			req_id, cfm.status);
		return -EIO;
	}

	return 0;
}

/* Raw hexdump of `len` bytes into the sysfs buffer, 16 bytes per line */
static int stat_hexdump(char *out, size_t out_len, int n,
			const u8 *data, u32 len, u32 skip)
{
	u32 i;

	for (i = 0; i + 16 <= len; i += 16) {
		int j;

		if (out_len - n < 96)
			break;
		n += scnprintf(out + n, out_len - n, "  %04x:", skip + i);
		for (j = 0; j < 16; j++)
			n += scnprintf(out + n, out_len - n, " %02x",
				       data[i + j]);
		n += scnprintf(out + n, out_len - n, "\n");
	}

	return n;
}

/*
 * Bitrate tables matching vendor bitrate.c (in fixed-point 10 * Mbps,
 * i.e. 100 kbps units, canonical format for sinfo->txrate/rxrate.legacy):
 */
static const u16 w600_rates_11ag[8] = {
	60, 90, 120, 180, 240, 360, 480, 540
};

static const u16 w600_rates_11b[8] = {
	0, 20, 55, 110, 10, 20, 55, 110
};

static const u16 w600_rates_11n[32 * 4] = {
	  65,   72,  135,  150,
	 130,  144,  270,  300,
	 195,  217,  405,  450,
	 260,  289,  540,  600,
	 390,  433,  810,  900,
	 520,  578, 1080, 1200,
	 585,  650, 1215, 1350,
	 650,  722, 1350, 1500,

	 130,  144,  270,  300,
	 260,  289,  540,  600,
	 390,  433,  810,  900,
	 520,  578, 1080, 1200,
	 780,  867, 1620, 1800,
	1040, 1156, 2160, 2400,
	1170, 1300, 2430, 2700,
	1300, 1444, 2700, 3000,

	 195,  217,  405,  450,
	 390,  433,  810,  900,
	 585,  650, 1215, 1350,
	 780,  867, 1620, 1800,
	1170, 1300, 2430, 2700,
	1560, 1733, 3240, 3600,
	1755, 1950, 3645, 4050,
	1950, 2167, 4050, 4500,

	 260,  289,  540,  600,
	 520,  578, 1080, 1200,
	 780,  867, 1620, 1800,
	1040, 1156, 2160, 2400,
	1560, 1733, 3240, 3600,
	2080, 2311, 4320, 4800,
	2340, 2600, 4860, 5400,
	2600, 2889, 5400, 6000,
};

static const u16 w600_rates_11ac[2][10 * 8] = {
	{ /* NSS = 1 */
	   65,   72,  135,  150,  293,  325,  585,  650,
	  130,  144,  270,  300,  585,  650, 1170, 1300,
	  195,  217,  405,  450,  878,  975, 1755, 1950,
	  260,  289,  540,  600, 1170, 1300, 2340, 2600,
	  390,  433,  810,  900, 1755, 1950, 3510, 3900,
	  520,  578, 1080, 1200, 2340, 2600, 4680, 5200,
	  585,  650, 1215, 1350, 2633, 2925, 5265, 5850,
	  650,  722, 1350, 1500, 2925, 3250, 5850, 6500,
	  780,  867, 1620, 1800, 3510, 3900, 7020, 7800,
	    0,    0, 1800, 2000, 3900, 4333, 7800, 8667,
	},
	{ /* NSS = 2 */
	  130,  144,  270,  300,  585,  650, 1170, 1300,
	  260,  289,  540,  600, 1170, 1300, 2340, 2600,
	  390,  433,  810,  900, 1755, 1950, 3510, 3900,
	  520,  578, 1080, 1200, 2340, 2600, 4680, 5200,
	  780,  867, 1620, 1800, 3510, 3900, 7020, 7800,
	 1040, 1156, 2160, 2400, 4680, 5200, 9360, 10400,
	 1170, 1303, 2430, 2700, 5265, 5850, 10530, 11700,
	 1300, 1444, 2700, 3000, 5850, 6500, 11700, 13000,
	 1560, 1733, 3240, 3600, 7020, 7800, 14040, 15600,
	    0,    0, 3600, 4000, 7800, 8667, 15600, 17333,
	}
};

static const u16 w600_rates_11ax[2][12 * 4 * 3] = {
	{ /* NSS = 1 */
	    86,    81,    73,   172,   163,   146,   360,   340,   306,   721,   681,   613,
	   172,   163,   146,   344,   325,   293,   721,   681,   613,  1441,  1361,  1225,
	   258,   244,   219,   516,   488,   439,  1081,  1021,   919,  2162,  2042,  1838,
	   344,   325,   293,   688,   650,   585,  1441,  1361,  1225,  2882,  2722,  2450,
	   516,   488,   439,  1032,   975,   878,  2162,  2042,  1838,  4324,  4083,  3675,
	   688,   650,   585,  1376,  1300,  1170,  2882,  2722,  2450,  5765,  5444,  4900,
	   774,   731,   658,  1549,  1463,  1316,  3243,  3063,  2756,  6485,  6125,  5513,
	   860,   813,   731,  1721,  1625,  1463,  3603,  3403,  3063,  7206,  6806,  6125,
	  1032,   975,   878,  2065,  1950,  1755,  4324,  4083,  3675,  8647,  8167,  7350,
	  1147,  1083,   975,  2294,  2167,  1950,  4804,  4537,  4083,  9607,  9074,  8166,
	  1290,  1219,  1097,  2581,  2438,  2194,  5404,  5104,  4594, 10809, 10208,  9188,
	  1434,  1354,  1219,  2868,  2708,  2438,  6004,  5671,  5104, 12010, 11342, 10208,
	},
	{ /* NSS = 2 */
	   172,   163,   146,   344,   325,   293,   721,   681,   613,  1441,  1361,  1225,
	   344,   325,   293,   688,   650,   585,  1441,  1361,  1225,  2882,  2722,  2450,
	   516,   488,   439,  1032,   975,   878,  2162,  2042,  1838,  4324,  4083,  3675,
	   688,   650,   585,  1376,  1300,  1170,  2882,  2722,  2450,  5765,  5444,  4900,
	  1032,   975,   878,  2065,  1950,  1755,  4324,  4083,  3675,  8647,  8167,  7350,
	  1376,  1300,  1170,  2753,  2600,  2340,  5765,  5444,  4900, 11529, 10889,  9800,
	  1549,  1463,  1316,  3097,  2925,  2633,  6485,  6125,  5513, 12971, 12250, 11025,
	  1721,  1625,  1463,  3441,  3250,  2925,  7206,  6806,  6125, 14412, 13611, 12250,
	  2065,  1950,  1755,  4129,  3900,  3510,  8647,  8167,  7350, 17294, 16333, 14700,
	  2294,  2167,  1950,  4588,  4333,  3900,  9607,  9074,  8166, 19215, 18148, 16333,
	  2581,  2438,  2194,  5162,  4875,  4388, 10809, 10208,  9188, 21618, 20417, 18375,
	  2868,  2708,  2438,  5735,  5417,  4875, 12010, 11343, 10208, 24019, 22685, 20416,
	}
};

static u32 wave600_calc_bitrate(u8 mode, u8 cbw, u8 scp, u8 mcs, u8 nss)
{
	static const u8 he_cp_map[6] = { 0, 1, 0, 1, 0, 2 };
	u32 idx;

	if (nss < 1)
		nss = 1;
	if (nss > 2)
		nss = 2;
	if (cbw > 3)
		cbw = 0;

	switch (mode) {
	case 4: /* HE */
		if (mcs <= 11) {
			u8 cp_type = (scp < 6) ? he_cp_map[scp] : 0;
			idx = cp_type + (3 * (cbw + (4 * mcs)));
			if (idx < 12 * 4 * 3)
				return w600_rates_11ax[nss - 1][idx];
		}
		break;
	case 3: /* VHT */
		if (mcs <= 9) {
			idx = (scp ? 1 : 0) | (cbw << 1) | (mcs << 3);
			if (idx < 10 * 8)
				return w600_rates_11ac[nss - 1][idx];
		}
		break;
	case 2: /* HT */
		if (mcs <= 31 && cbw < 2) {
			idx = (scp ? 1 : 0) | (cbw << 1) | (mcs << 2);
			if (idx < 32 * 4)
				return w600_rates_11n[idx];
		}
		break;
	case 1: /* 11b */
		if (mcs < 8)
			return w600_rates_11b[mcs];
		break;
	case 0: /* 11a/g */
	default:
		if (mcs < 8)
			return w600_rates_11ag[mcs];
		break;
	}

	return 0;
}

/*
 * Vendor init path (hw_mmb.c hw_mmb_card_fin step HW_MHI_STATISTIC): the
 * descriptor table is fetched exactly ONCE during card finalize, right
 * after HW_DEP_CFG, and kept in a host copy. The firmware may only serve
 * the TABLE request inside the init window, which is why the sysfs dump
 * only re-fetches the counter database (0x0460) once this copy exists.
 */
int wave600_stats_init(struct wave600_dev *wdev)
{
	const struct wave600_chi_stats_mgr_ext *ext = &wdev->chi.stats_mgr_ext;
	struct wave600_stats *stats = &wdev->stats;
	struct wave600_stats_entry *tab;
	u32 db_size, tab_size, alloc_size;
	dma_addr_t dma;
	u8 *mem;
	int ret;

	db_size = ext->stats_data_size;
	tab_size = ext->stats_table_size;

	if (ext->header.id != WAVE600_CHI_EXT_ID_STATS_MANAGER) {
		dev_err(wdev->dev, "fw_stats init: CHI statistics manager extension not present (id=%u magic=0x%08x)\n",
			ext->header.id, ext->header.magic);
		return -ENODEV;
	}

	if (!db_size || !tab_size || tab_size % sizeof(*tab)) {
		dev_err(wdev->dev, "fw_stats init: invalid statistics geometry (db=%u tab=%u)\n",
			db_size, tab_size);
		return -EINVAL;
	}

	/* Allocate coherent buffer for dynamic PHY status (UMI 0x040F) */
	stats->phy_db_size = sizeof(struct wave600_whole_phy_rx_status_db);
	stats->phy_db = dma_alloc_coherent(wdev->dev, stats->phy_db_size,
					   &stats->phy_db_dma, GFP_KERNEL);
	if (!stats->phy_db)
		dev_warn(wdev->dev, "fw_stats init: failed to allocate phy_db buffer (%u B)\n",
			 stats->phy_db_size);

	alloc_size = max(db_size, tab_size);
	mem = dma_alloc_coherent(wdev->dev, alloc_size, &dma, GFP_KERNEL);
	if (!mem) {
		if (stats->phy_db) {
			dma_free_coherent(wdev->dev, stats->phy_db_size,
					  stats->phy_db, stats->phy_db_dma);
			stats->phy_db = NULL;
		}
		return -ENOMEM;
	}

	memset(mem, 0, alloc_size);
	ret = wave600_stats_send(wdev,
				 WAVE600_UM_MAN_GET_STATISTICS_TABLE_REQ,
				 WAVE600_MC_MAN_GET_STATISTICS_TABLE_CFM,
				 dma, (u16)tab_size);
	if (ret) {
		dma_free_coherent(wdev->dev, alloc_size, mem, dma);
		if (stats->phy_db) {
			dma_free_coherent(wdev->dev, stats->phy_db_size,
					  stats->phy_db, stats->phy_db_dma);
			stats->phy_db = NULL;
		}
		return ret;
	}

	/* The confirmation may precede the firmware DMA write; let it land. */
	msleep(1000);

	stats->table = kmalloc(tab_size, GFP_KERNEL);
	if (!stats->table) {
		dma_free_coherent(wdev->dev, alloc_size, mem, dma);
		if (stats->phy_db) {
			dma_free_coherent(wdev->dev, stats->phy_db_size,
					  stats->phy_db, stats->phy_db_dma);
			stats->phy_db = NULL;
		}
		return -ENOMEM;
	}
	memcpy(stats->table, mem, tab_size);
	stats->tab_size = tab_size;
	stats->db_size = db_size;
	stats->db = mem;
	stats->db_dma = dma;

	tab = (struct wave600_stats_entry *)stats->table;
	stats->gen_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
					   WAVE600_STAT_TYPE_GENERAL_STATS);
	stats->baa_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
					   WAVE600_STAT_TYPE_BAA_COUNTERS);
	stats->hostif_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
						 WAVE600_STAT_TYPE_HOSTIF_COUNTERS);
	stats->la_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
					  WAVE600_STAT_TYPE_LINK_ADAPTATION);

	stats->la_bw_off = -1;
	stats->la_mode_off = -1;
	stats->la_mcs_off = -1;
	stats->la_nss_off = -1;
	stats->la_scp_off = -1;

	if (stats->la_start >= 0) {
		u32 entries = tab_size / sizeof(*tab);

		if (stats->la_start + 12 < entries)
			stats->la_bw_off = stat_entry_offset(&tab[stats->la_start + 12]);
		if (stats->la_start + 4 < entries)
			stats->la_mode_off = stat_entry_offset(&tab[stats->la_start + 4]);
		if (stats->la_start + 10 < entries)
			stats->la_scp_off = stat_entry_offset(&tab[stats->la_start + 10]);
		if (stats->la_start + 19 < entries)
			stats->la_mcs_off = stat_entry_offset(&tab[stats->la_start + 19]);
		if (stats->la_start + 17 < entries)
			stats->la_nss_off = stat_entry_offset(&tab[stats->la_start + 17]);
	}

	stats->table_valid = true;

	/*
	 * Raw evidence dump of the captured descriptor table: the head and
	 * the CHI-declared GENERAL_STATS block offset. Distinguishes an
	 * all-zero table (firmware never wrote it) from a layout shift.
	 */
	{
		u32 gen_off = 0;

		if (WAVE600_STAT_TYPE_GENERAL_STATS <
		    ARRAY_SIZE(ext->type_table_list))
			gen_off = ext->type_table_list[WAVE600_STAT_TYPE_GENERAL_STATS];

		print_hex_dump(KERN_INFO, "stattab@0 ",
			       DUMP_PREFIX_OFFSET, 16, 4, stats->table,
			       min((u32)32, tab_size), false);
		if (gen_off && gen_off % sizeof(*tab) == 0 && gen_off < tab_size)
			print_hex_dump(KERN_INFO, "stattab@G ",
				       DUMP_PREFIX_OFFSET, 16, 4,
				       stats->table + gen_off,
				       min((u32)32, tab_size - gen_off), false);
	}

	if (stats->gen_start < 0)
		dev_info(wdev->dev, "fw_stats init: GENERAL_STATS type not found in table (raw dump available via sysfs)\n");
	else
		dev_info(wdev->dev, "fw_stats init: statistics table captured (entries=%u gen_start=%d baa_start=%d la_start=%d la_bw=%d la_mode=%d)\n",
			 tab_size / sizeof(*tab), stats->gen_start,
			 stats->baa_start, stats->la_start,
			 stats->la_bw_off, stats->la_mode_off);

	return 0;
}

void wave600_stats_cleanup(struct wave600_dev *wdev)
{
	struct wave600_stats *stats = &wdev->stats;

	if (stats->phy_db) {
		dma_free_coherent(wdev->dev, stats->phy_db_size,
				  stats->phy_db, stats->phy_db_dma);
		stats->phy_db = NULL;
		stats->phy_db_dma = 0;
	}

	if (stats->db) {
		dma_free_coherent(wdev->dev, max(stats->db_size, stats->tab_size),
				  stats->db, stats->db_dma);
		stats->db = NULL;
		stats->db_dma = 0;
	}
	kfree(stats->table);
	stats->table = NULL;
	stats->table_valid = false;
}

void wave600_stats_update_telemetry(struct wave600_dev *wdev)
{
	struct wave600_stats *stats = &wdev->stats;
	int r, i;

	if (wdev->state != W600_STATE_RUNNING || atomic_read(&wdev->in_recovery))
		return;

	if (!wdev->radios[0].sta_connected && !wdev->radios[1].sta_connected &&
	    !wdev->radios[0].num_stations && !wdev->radios[1].num_stations)
		return;

	/* 1. Fetch PHY Status via UMI 0x040F */
	if (stats->phy_db) {
		struct wave600_umi_phy_status_req phy_req;
		struct wave600_umi_phy_status_req phy_cfm;
		int ret;

		memset(&phy_req, 0, sizeof(phy_req));
		phy_req.ddr_buffer_address = cpu_to_le32((u32)stats->phy_db_dma);
		phy_req.ddr_buffer_size = cpu_to_le32(stats->phy_db_size);

		ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_PHY_STATUS_REQ,
				      WAVE600_MC_MAN_PHY_STATUS_CFM,
				      &phy_req, sizeof(phy_req),
				      &phy_cfm, sizeof(phy_cfm), 2000);
		if (ret)
			dev_dbg(wdev->dev, "telemetry: PHY_STATUS_REQ failed: %d\n", ret);
	}

	/* 2. Fetch MHI Statistics via UMI 0x0460 */
	if (stats->db && stats->db_size) {
		int ret = wave600_stats_send(wdev,
					     WAVE600_UM_MAN_GET_STATISTICS_DATA_REQ,
					     WAVE600_MC_MAN_GET_STATISTICS_DATA_CFM,
					     stats->db_dma, (u16)stats->db_size);
		if (ret)
			dev_dbg(wdev->dev, "telemetry: GET_STATISTICS_DATA_REQ failed: %d\n", ret);
	}

	/* 3. Update stations across active radios */
	for (r = 0; r < 2; r++) {
		struct wave600_radio *radio = &wdev->radios[r];

		if (!radio->ap_started)
			continue;

		for (i = 0; i < WAVE600_MAX_STATIONS; i++) {
			struct wave600_station *sta = &radio->stations[i];
			u32 sid;

			if (!sta->valid)
				continue;

			sid = sta->sid;
			if (sid >= WAVE600_GEN6_MAX_SID)
				continue;

			/* Decode RX metrics from phy_db */
			if (stats->phy_db) {
				struct wave600_whole_phy_rx_status_db *pdb = stats->phy_db;
				struct wave600_station_phy_rx_status *sstatus = &pdb->sta_status[sid];
				s8 max_r = -100;
				int a;

				for (a = 0; a < WAVE600_PHY_STATISTICS_MAX_RX_ANT; a++) {
					s8 sig = sstatus->rssi[a];
					if (sig < 0 && sig > max_r)
						max_r = sig;
				}
				if (max_r > -100 && max_r < 0) {
					sta->signal = max_r;
					if (sta->signal_avg == 0 || sta->signal_avg <= -99)
						sta->signal_avg = max_r;
					else
						sta->signal_avg = (s8)(((int)sta->signal_avg * 7 + max_r) / 8);
					radio->sta_signal = max_r;
				}

				u32 prate_word = le32_to_cpu(sstatus->phy_rate);
				u32 hw_r = (prate_word >> 4) & 0x1FFFF;
				if (hw_r > 0 && hw_r < 0x1FFFF)
					sta->rx_bitrate = hw_r;
			}

			/* Decode TX Link Adaptation metrics from stats->db */
			if (stats->db && stats->la_bw_off >= 0 && stats->la_mode_off >= 0 &&
			    stats->la_mcs_off >= 0 && stats->la_nss_off >= 0 && stats->la_scp_off >= 0) {
				const u8 *db = (const u8 *)stats->db;
				u8 cbw = db[stats->la_bw_off + sid];
				u32 off_2d;
				u8 pmode, mcs, nss, scp;
				u32 tx_r;

				if (cbw > 3)
					cbw = 0;
				off_2d = (sid * 4) + cbw;

				pmode = db[stats->la_mode_off + off_2d];
				mcs = db[stats->la_mcs_off + off_2d];
				nss = db[stats->la_nss_off + off_2d];
				scp = db[stats->la_scp_off + off_2d];

				if (nss < 1)
					nss = 1;
				if (nss > 2)
					nss = 2;

				if (pmode == 4) { /* HE */
					sta->tx_flags = RATE_INFO_FLAGS_HE_MCS;
					sta->tx_mcs = mcs <= 11 ? mcs : 11;
					sta->tx_nss = nss;
					sta->tx_gi = (scp == 1) ? NL80211_RATE_INFO_HE_GI_0_8 :
						     ((scp == 2) ? NL80211_RATE_INFO_HE_GI_1_6 :
								   NL80211_RATE_INFO_HE_GI_3_2);
				} else if (pmode == 3) { /* VHT */
					sta->tx_flags = RATE_INFO_FLAGS_VHT_MCS | (scp ? RATE_INFO_FLAGS_SHORT_GI : 0);
					sta->tx_mcs = mcs <= 9 ? mcs : 9;
					sta->tx_nss = nss;
				} else if (pmode == 2) { /* HT */
					sta->tx_flags = RATE_INFO_FLAGS_MCS | (scp ? RATE_INFO_FLAGS_SHORT_GI : 0);
					sta->tx_mcs = mcs <= 31 ? mcs : 15;
					sta->tx_nss = (sta->tx_mcs / 8) + 1;
				} else { /* Legacy */
					sta->tx_flags = 0;
					sta->tx_mcs = mcs <= 7 ? mcs : 7;
					sta->tx_nss = 1;
				}

				if (cbw == 3)
					sta->tx_bw = RATE_INFO_BW_160;
				else if (cbw == 2)
					sta->tx_bw = RATE_INFO_BW_80;
				else if (cbw == 1)
					sta->tx_bw = RATE_INFO_BW_40;
				else
					sta->tx_bw = RATE_INFO_BW_20;

				tx_r = wave600_calc_bitrate(pmode, cbw, scp, sta->tx_mcs, sta->tx_nss);
				if (tx_r > 0)
					sta->tx_bitrate = tx_r;

				sta->rx_bw = sta->tx_bw;
			}
		}
	}
}

static int wave600_fw_stats_dump(struct wave600_dev *wdev, char *out,
				 size_t out_len)
{
	const struct wave600_chi_stats_mgr_ext *ext = &wdev->chi.stats_mgr_ext;
	struct wave600_stats *stats = &wdev->stats;
	struct wave600_stats_entry *tab;
	u32 db_size, tab_size, alloc_size;
	dma_addr_t dma;
	u8 *mem;
	u8 *runtime_tab = NULL;
	bool temp_mem = false;
	int gen_start, baa_start, hostif_start, la_start;
	int ret;
	int n = 0;
	int i;

	mutex_lock(&wave600_stats_lock);

	if (ext->header.id != WAVE600_CHI_EXT_ID_STATS_MANAGER) {
		dev_err(wdev->dev, "fw_stats: CHI statistics manager extension not present (id=%u magic=0x%08x)\n",
			ext->header.id, ext->header.magic);
		ret = -ENODEV;
		goto unlock;
	}

	db_size = ext->stats_data_size;
	tab_size = ext->stats_table_size;

	if (!db_size || !tab_size || tab_size % sizeof(*tab)) {
		dev_err(wdev->dev, "fw_stats: invalid statistics geometry (db=%u tab=%u)\n",
			db_size, tab_size);
		ret = -EINVAL;
		goto unlock;
	}

	if (stats->table_valid) {
		/*
		 * Vendor runtime path (hw_mmb.c mtlk_hw_mhi_get_stats): only
		 * the counter database (0x0460) is re-fetched; the descriptor
		 * table comes from the copy captured at init.
		 */
		tab = (struct wave600_stats_entry *)stats->table;
		mem = stats->db;
		dma = stats->db_dma;
		alloc_size = max(db_size, tab_size);
	} else {
		/*
		 * Diagnostic fallback when the init fetch failed: retry the
		 * TABLE request at runtime. The vendor never does this; it is
		 * kept so the sysfs dump stays usable for evidence.
		 */
		alloc_size = max(db_size, tab_size);
		mem = dma_alloc_coherent(wdev->dev, alloc_size, &dma, GFP_KERNEL);
		if (!mem) {
			ret = -ENOMEM;
			goto unlock;
		}
		temp_mem = true;

		memset(mem, 0, alloc_size);
		ret = wave600_stats_send(wdev,
					 WAVE600_UM_MAN_GET_STATISTICS_TABLE_REQ,
					 WAVE600_MC_MAN_GET_STATISTICS_TABLE_CFM,
					 dma, (u16)tab_size);
		if (ret)
			goto out_free;

		/*
		 * The confirmation may precede the firmware DMA write; give
		 * the DMA time to land before the buffer is inspected.
		 */
		msleep(1000);

		runtime_tab = kmalloc(tab_size, GFP_KERNEL);
		if (!runtime_tab) {
			ret = -ENOMEM;
			goto out_free;
		}
		memcpy(runtime_tab, mem, tab_size);
		tab = (struct wave600_stats_entry *)runtime_tab;
	}

	gen_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				   WAVE600_STAT_TYPE_GENERAL_STATS);
	baa_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				   WAVE600_STAT_TYPE_BAA_COUNTERS);
	hostif_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				      WAVE600_STAT_TYPE_HOSTIF_COUNTERS);
	la_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				  WAVE600_STAT_TYPE_LINK_ADAPTATION);

	n += scnprintf(out + n, out_len - n,
		       "fw_stats: db_size=%u tab_size=%u dma=0x%08x ext_id=%u table=%s\n",
		       db_size, tab_size, (u32)dma, ext->header.id,
		       stats->table_valid ? "init" : "runtime");
	n += scnprintf(out + n, out_len - n,
		       "fw_stats: entries=%u gen_start=%d baa_start=%d hostif_start=%d la_start=%d offsets/dims=LE\n",
		       tab_size / sizeof(*tab), gen_start, baa_start,
		       hostif_start, la_start);

	if (out_len - n >= 96) {
		n += scnprintf(out + n, out_len - n, "fw_stats type_table_list:");
		for (i = 0; i < (int)ARRAY_SIZE(ext->type_table_list); i++)
			n += scnprintf(out + n, out_len - n, " 0x%08x",
				       ext->type_table_list[i]);
		n += scnprintf(out + n, out_len - n, "\n");
	}

	n += scnprintf(out + n, out_len - n, "fw_stats table raw head:\n");
	n = stat_hexdump(out, out_len, n, (const u8 *)tab,
			 min((u32)128, tab_size), 0);

	/*
	 * Raw bytes at the CHI-declared GENERAL_STATS block offset: an
	 * all-zero table (firmware never wrote it) is distinguishable from
	 * a layout shift without relying on the parsed entry type.
	 */
	if (WAVE600_STAT_TYPE_GENERAL_STATS < ARRAY_SIZE(ext->type_table_list)) {
		u32 gen_off = ext->type_table_list[WAVE600_STAT_TYPE_GENERAL_STATS];

		if (gen_off && gen_off % sizeof(*tab) == 0 && gen_off < tab_size) {
			n += scnprintf(out + n, out_len - n,
				       "fw_stats table raw at GENERAL_STATS offset 0x%04x:\n",
				       gen_off);
			n = stat_hexdump(out, out_len, n,
					 (const u8 *)tab + gen_off,
					 min((u32)32, tab_size - gen_off), gen_off);
		}
	}

	if (la_start >= 0) {
		n += scnprintf(out + n, out_len - n,
			       "fw_stats LINK_ADAPTATION table entries:\n");
		for (i = 0; la_start + i < (int)(tab_size / sizeof(*tab)) &&
			     i < 32; i++) {
			const struct wave600_stats_entry *e = &tab[la_start + i];

			if (stat_entry_type(e) != WAVE600_STAT_TYPE_LINK_ADAPTATION)
				break;
			if (out_len - n < 96)
				break;

			n += scnprintf(out + n, out_len - n,
				       "la_entry[%02d] raw_sub=%02u pos_sub=%02u unit=%u dims=%u dim1=%u dim2=%u off=%u size=%u next=%u fits=%d\n",
				       i, stat_entry_subtype(e), i,
				       stat_entry_unit(e), stat_entry_dims(e),
				       stat_entry_dim1(e), e->dimension2,
				       stat_entry_offset(e), stat_entry_bytes(e),
				       e->next_idx_offset,
				       stat_entry_fits(e, db_size));
		}
	}

	if (gen_start < 0) {
		/*
		 * The firmware did not deliver a parseable table. Return the
		 * collected raw evidence instead of an error so that a plain
		 * sysfs read still shows what (if anything) was DMA'd.
		 */
		dev_info(wdev->dev, "fw_stats: GENERAL_STATS type not found in table (table stays available in raw dump)\n");
		ret = n;
		goto out_free;
	}

	/* 2. Fetch the counter database */
	memset(mem, 0, alloc_size);
	ret = wave600_stats_send(wdev,
				 WAVE600_UM_MAN_GET_STATISTICS_DATA_REQ,
				 WAVE600_MC_MAN_GET_STATISTICS_DATA_CFM,
				 dma, (u16)db_size);
	if (ret)
		goto out_free;

	msleep(500);

	/* GENERAL_STATS counters (subtype = position within the type) */
	for (i = 0; gen_start + i < (int)(tab_size / sizeof(*tab)); i++) {
		const struct wave600_stats_entry *e = &tab[gen_start + i];
		const char *name = i < (int)ARRAY_SIZE(stat_general_names) ?
				   stat_general_names[i] : "?";
		u8 raw[4];
		u32 value;

		if (out_len - n < 96)
			break;

		if (stat_entry_type(e) != WAVE600_STAT_TYPE_GENERAL_STATS)
			break;

		stat_counter_le(mem, db_size, e, &value, raw);
		n += scnprintf(out + n, out_len - n,
			       "general[%02d] %-22s raw_sub=%02u off=%-5u unit=%u raw=%02x%02x%02x%02x value=%u\n",
			       i, name, stat_entry_subtype(e),
			       stat_entry_offset(e), stat_entry_unit(e),
			       raw[0], raw[1], raw[2], raw[3], value);
	}

	/* Selected HOSTIF counters (host -> firmware data path) */
	if (hostif_start >= 0) {
		for (i = 0; i < (int)ARRAY_SIZE(stat_hostif_names); i++) {
			const struct wave600_stats_entry *e =
				&tab[hostif_start + i];
			const char *name = stat_hostif_names[i];
			u8 raw[4];
			u32 value;

			if (out_len - n < 96)
				break;

			if (stat_entry_type(e) != WAVE600_STAT_TYPE_HOSTIF_COUNTERS)
				break;

			stat_counter_le(mem, db_size, e, &value, raw);
			n += scnprintf(out + n, out_len - n,
				       "hostif[%02d] %-22s raw_sub=%02u off=%-5u unit=%u raw=%02x%02x%02x%02x value=%u\n",
				       i, name, stat_entry_subtype(e),
				       stat_entry_offset(e), stat_entry_unit(e),
				       raw[0], raw[1], raw[2], raw[3], value);
		}
	}

	/* Named BAA counters (firmware TX engine view) */
	if (baa_start >= 0) {
		for (i = 0; i < (int)ARRAY_SIZE(stat_baa_named); i++) {
			unsigned int subtype = stat_baa_named[i].subtype;
			const struct wave600_stats_entry *e =
				&tab[baa_start + subtype];
			u8 raw[4];
			u32 value;

			if (out_len - n < 96)
				continue;

			if (stat_entry_type(e) != WAVE600_STAT_TYPE_BAA_COUNTERS)
				continue;

			stat_counter_le(mem, db_size, e, &value, raw);
			n += scnprintf(out + n, out_len - n,
				       "baa[%02d]  %-22s raw_sub=%02u off=%-5u unit=%u raw=%02x%02x%02x%02x value=%u\n",
				       subtype, stat_baa_named[i].name,
				       stat_entry_subtype(e),
				       stat_entry_offset(e), stat_entry_unit(e),
				       raw[0], raw[1], raw[2], raw[3], value);
		}
	}

	/*
	 * LINK_ADAPTATION power fields: the actual TX power the firmware
	 * uses, per radio (2) x per channel bandwidth (20/40/80/160 = 4).
	 * Vendor hw_mmb.c _mtlk_hw_mhi_update_hw_stats reads these as
	 * uint8 values with STAT_PW_PER_DBM=2 units per dBm, block layout
	 * [radio][cbw], so dBm = byte/2. Subtypes: 6=POWER_DATA,
	 * 7=POWER_MANAGEMENT, 21=MIN_POWER, 22=MAX_POWER.
	 */
	if (la_start >= 0) {
		static const struct {
			unsigned int subtype;
			const char *name;
		} la_pw_fields[] = {
			{ W600_LA_POWER_DATA,	 "power_data" },
			{ W600_LA_POWER_MANAGEMENT, "power_mgmt" },
			{ W600_LA_MIN_POWER,	 "min_power" },
			{ W600_LA_MAX_POWER,	 "max_power" },
		};

		for (i = 0; i < (int)ARRAY_SIZE(la_pw_fields); i++) {
			const struct wave600_stats_entry *e =
				&tab[la_start + la_pw_fields[i].subtype];
			u32 offset, block, unit, cbw, radio;
			int j;

			if (out_len - n < 96)
				break;

			if (stat_entry_type(e) != WAVE600_STAT_TYPE_LINK_ADAPTATION)
				continue;
			if (!stat_entry_fits(e, db_size)) {
				n += scnprintf(out + n, out_len - n,
					       "link_adapt[%02d] %-11s invalid entry off=%u size=%u db=%u\n",
					       la_pw_fields[i].subtype,
					       la_pw_fields[i].name,
					       stat_entry_offset(e),
					       stat_entry_bytes(e), db_size);
				continue;
			}

			offset = stat_entry_offset(e);
			unit = stat_entry_unit(e);
			block = stat_entry_bytes(e);
			n += scnprintf(out + n, out_len - n,
				       "link_adapt[%02d] %-11s off=%-5u unit=%u dims=%u dim1=%u dim2=%u size=%u bytes:",
				       la_pw_fields[i].subtype,
				       la_pw_fields[i].name, offset, unit,
				       stat_entry_dims(e), stat_entry_dim1(e),
				       e->dimension2, block);
			for (j = 0; j < (int)block && offset + j < db_size; j++) {
				if (out_len - n < 8)
					break;
				n += scnprintf(out + n, out_len - n, " %02x",
					       mem[offset + j]);
			}
			n += scnprintf(out + n, out_len - n, "  (dBm:");
			for (radio = 0; radio < 2; radio++) {
				for (cbw = 0; cbw < 4; cbw++) {
					u32 idx = radio * 4 + cbw;

					if (idx >= block || offset + idx >= db_size)
						continue;
					if (out_len - n < 8)
						break;
					n += scnprintf(out + n, out_len - n, " %d",
						       (mem[offset + idx] +
							(W600_STAT_PW_PER_DBM / 2)) /
						       W600_STAT_PW_PER_DBM);
				}
			}
			n += scnprintf(out + n, out_len - n, ")  [r0:20/40/80/160 r1:20/40/80/160]\n");
		}
	}

	/* Raw head of the counter database for cross-checking */
	n += scnprintf(out + n, out_len - n, "fw_stats db raw head:\n");
	n = stat_hexdump(out, out_len, n, mem,
			 min((u32)96, db_size), 0);

	ret = n;

out_free:
	kfree(runtime_tab);
	if (temp_mem)
		dma_free_coherent(wdev->dev, alloc_size, mem, dma);
unlock:
	mutex_unlock(&wave600_stats_lock);
	return ret;
}

static ssize_t fw_stats_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wave600_dev *wdev = pci_get_drvdata(pdev);

	if (!wdev)
		return -ENODEV;

	return wave600_fw_stats_dump(wdev, buf, PAGE_SIZE);
}
static DEVICE_ATTR_RO(fw_stats);

/*
 * Read-only diagnostic dump of the RF TX power state:
 *  1. the calibration cache (cal_buf) that the firmware filled during
 *     UM_MAN_CALIBRATE_REQ — shows what TX calibration results FW stored;
 *  2. the PHY TX TPC registers per band/antenna (vendor HwMemoryMap.h
 *     HYP_TD_ANTENNA_x_TPC_POUT/TSSI/GAIN) — the actual output power /
 *     gain the PHY applies. Vendor bases are chip-physical 0xA0xxxxxx;
 *     BAR0 aliases 0xA0000000, so B0_PHY_TX = 0xBC0000, B1 = 0xFC0000.
 * Antenna stride 0x4000, register deltas: POUT +0x000, TSSI +0x200, GAIN +0x400.
 * Each 0x200-byte register table is scanned fully and summarized with the
 * first four words, the count of non-zero non-mask words and the first
 * meaningful byte offset.
 */
#define W600_B0_PHY_TX_OFF	0x00BC0000
#define W600_B1_PHY_TX_OFF	0x00FC0000

static int wave600_cal_dump(struct wave600_dev *wdev, char *out, size_t out_len)
{
	static const struct {
		u32 base;
		const char *band;
	} bands[] = {
		{ W600_B0_PHY_TX_OFF, "B0" },
		{ W600_B1_PHY_TX_OFF, "B1" },
	};
	static const struct {
		u32 delta;
		const char *name;
	} tpc_regs[] = {
		{ 0x000, "TPC_POUT" },
		{ 0x200, "TPC_TSSI" },
		{ 0x400, "TPC_GAIN" },
	};
	int n = 0;
	u32 nz = 0, first_nz = wdev->cal_buf_size;
	int i, bi, ant, ri;

	n += scnprintf(out + n, out_len - n,
		       "cal_dump: cal_buf=%p size=%zu dma=0x%08x\n",
		       wdev->cal_buf, wdev->cal_buf_size, (u32)wdev->cal_dma_addr);

	if (wdev->cal_buf && wdev->cal_buf_size) {
		const u8 *b = wdev->cal_buf;

		for (i = 0; i < (int)wdev->cal_buf_size; i++) {
			if (b[i]) {
				nz++;
				if (first_nz == wdev->cal_buf_size)
					first_nz = (u32)i;
			}
		}
		n += scnprintf(out + n, out_len - n,
			       "cal_dump: nonzero bytes=%u / %zu (first at 0x%x)\n",
			       nz, wdev->cal_buf_size, first_nz);
		n += scnprintf(out + n, out_len - n, "cal_dump head (128 B):\n");
		n = stat_hexdump(out, out_len, n, b,
				 min_t(u32, 128, (u32)wdev->cal_buf_size), 0);
		if (nz && first_nz + 32 <= wdev->cal_buf_size) {
			n += scnprintf(out + n, out_len - n,
				       "cal_dump at first nonzero 0x%x (32 B):\n",
				       first_nz);
			n = stat_hexdump(out, out_len, n, b + first_nz, 32,
					 first_nz);
		}
	} else {
		n += scnprintf(out + n, out_len - n,
			       "cal_dump: cal_buf not allocated\n");
	}

	for (bi = 0; bi < (int)ARRAY_SIZE(bands); bi++) {
		for (ant = 0; ant < 4; ant++) {
			u32 ant_base = 0x22000 + (u32)ant * 0x4000;

			for (ri = 0; ri < (int)ARRAY_SIZE(tpc_regs); ri++) {
				u32 addr = bands[bi].base + ant_base + tpc_regs[ri].delta;
				u32 words[4] = { 0, 0, 0, 0 };
				u32 meaningful = 0;
				bool all_ff = true;
				int first = -1;

				for (i = 0; i < 128; i++) {
					u32 w = wave600_read32(wdev, addr + i * 4);

					if (w != 0xffffffff)
						all_ff = false;
					if (w && w != 0xffffffff) {
						meaningful++;
						if (first < 0)
							first = i * 4;
					}
					if (i < 4)
						words[i] = w;
				}

				if (out_len - (size_t)n < 144)
					goto done;
				n += scnprintf(out + n, out_len - n,
					       "cal_dump %s ant%u %-9s @bar0+0x%06x: %08x %08x %08x %08x nz=%u first=%d%s\n",
					       bands[bi].band, ant,
					       tpc_regs[ri].name, addr,
					       words[0], words[1], words[2], words[3],
					       meaningful, first,
					       all_ff ? " [all ff]" : "");
			}
		}
	}
done:
	return n;
}

static ssize_t cal_dump_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wave600_dev *wdev = pci_get_drvdata(pdev);

	if (!wdev)
		return -ENODEV;

	return wave600_cal_dump(wdev, buf, PAGE_SIZE);
}
static DEVICE_ATTR_RO(cal_dump);

int wave600_stats_sysfs_create(struct wave600_dev *wdev)
{
	int ret;

	ret = device_create_file(wdev->dev, &dev_attr_fw_stats);
	if (ret)
		return ret;
	return device_create_file(wdev->dev, &dev_attr_cal_dump);
}

void wave600_stats_sysfs_remove(struct wave600_dev *wdev)
{
	device_remove_file(wdev->dev, &dev_attr_fw_stats);
	device_remove_file(wdev->dev, &dev_attr_cal_dump);
}
