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

	alloc_size = max(db_size, tab_size);
	mem = dma_alloc_coherent(wdev->dev, alloc_size, &dma, GFP_KERNEL);
	if (!mem)
		return -ENOMEM;

	memset(mem, 0, alloc_size);
	ret = wave600_stats_send(wdev,
				 WAVE600_UM_MAN_GET_STATISTICS_TABLE_REQ,
				 WAVE600_MC_MAN_GET_STATISTICS_TABLE_CFM,
				 dma, (u16)tab_size);
	if (ret) {
		dma_free_coherent(wdev->dev, alloc_size, mem, dma);
		return ret;
	}

	/* The confirmation may precede the firmware DMA write; let it land. */
	msleep(1000);

	stats->table = kmalloc(tab_size, GFP_KERNEL);
	if (!stats->table) {
		dma_free_coherent(wdev->dev, alloc_size, mem, dma);
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
		dev_info(wdev->dev, "fw_stats init: statistics table captured (entries=%u gen_start=%d baa_start=%d la_start=%d)\n",
			 tab_size / sizeof(*tab), stats->gen_start,
			 stats->baa_start, stats->la_start);

	return 0;
}

void wave600_stats_cleanup(struct wave600_dev *wdev)
{
	struct wave600_stats *stats = &wdev->stats;

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
