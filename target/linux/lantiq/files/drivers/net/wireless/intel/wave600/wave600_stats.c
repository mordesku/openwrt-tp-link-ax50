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
 * Endianness: the mailbox request fields (DDR buffer address, length)
 * are MAC-endian (little-endian), exactly like the vendor HOST_TO_MAC32/16
 * coding for every mailbox field. The descriptor table and the counter
 * database are DMA data written by the ARC firmware into host DDR.
 * Their field endianness is NOT guaranteed by the vendor headers (vendor
 * hw_mmb.c applies MAC_TO_HOST16/32, i.e. LE interpretation, while
 * DATA_PATH_INIT told the firmware about a big-endian host via
 * hostEndianessMode). The dump therefore detects the table offset
 * endianness by plausibility and prints every counter in both
 * interpretations plus raw bytes, so a single read is decisive evidence.
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
#define WAVE600_STAT_TYPE_GENERAL_STATS		14

/*
 * vendor mhi_statistics_manager_defines.h StatsEntryDecoded_t (8 bytes).
 * Byte 0 bitfield (same physical layout for BE and LE bitfield code):
 *   [reserved:1][unitSize:3][type:4]
 * Byte 1 bitfield:
 *   [numOfDimensions:2][subtype:6] (subtype is implicit by position)
 * The u16 fields (offset, dimension1) are read in both endiannesses.
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

static u16 stat_be16(const u8 *p)
{
	return (u16)((p[0] << 8) | p[1]);
}

static u8 stat_entry_type(const struct wave600_stats_entry *e)
{
	return e->flags & 0x0f;
}

static u8 stat_entry_unit(const struct wave600_stats_entry *e)
{
	return (e->flags >> 4) & 0x07;
}

static u16 stat_entry_offset(const struct wave600_stats_entry *e, bool le)
{
	return le ? stat_le16(e->offset_raw) : stat_be16(e->offset_raw);
}

static u16 stat_entry_dim1(const struct wave600_stats_entry *e, bool le)
{
	return le ? stat_le16(e->dim1_raw) : stat_be16(e->dim1_raw);
}

/*
 * Find the first table entry index of a statistics type. The CHI
 * type_table_list[] holds byte offsets from the start of the descriptor
 * table (vendor _mtlk_hw_mhi_parse_table compares entry byte offsets).
 * The entry's own type field is verified; if it does not match under the
 * BE bitfield layout, the LE layout is tried and reported.
 */
static int stat_find_type(const struct wave600_stats_entry *tab, u32 tab_num,
			  const struct wave600_chi_stats_mgr_ext *ext,
			  unsigned int type, bool *le_layout)
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

	/* LE bitfield layout fallback: type in the high nibble */
	if ((tab[i].flags >> 4) == type) {
		*le_layout = true;
		return i;
	}

	return -1;
}

static u8 stat_entry_type_auto(const struct wave600_stats_entry *e, bool le_layout)
{
	return le_layout ? (e->flags >> 4) : stat_entry_type(e);
}

/*
 * Count the entries of the GENERAL_STATS block whose
 * offset + unitSize*dim1*dim2 stays inside the counter database.
 * Used to pick the endianness of the u16 offset/dimension fields.
 */
static int stat_count_valid(const struct wave600_stats_entry *tab, u32 tab_num,
			    int start, u32 db_size, bool le, bool le_layout)
{
	const struct wave600_stats_entry *e;
	u32 dim1, dim2, size;
	int i, valid = 0;

	for (i = 0; start >= 0 && start + (int)i < (int)tab_num; i++) {
		e = &tab[start + i];

		if (stat_entry_type_auto(e, le_layout) !=
		    WAVE600_STAT_TYPE_GENERAL_STATS)
			break;

		dim1 = stat_entry_dim1(e, le);
		dim2 = e->dimension2;
		size = (u32)stat_entry_unit(e) *
		       (dim1 ? dim1 : 1) * (dim2 ? dim2 : 1);

		if ((u32)stat_entry_offset(e, le) + size <= db_size)
			valid++;
		else
			break;
	}

	return valid;
}

/* Counter value in both interpretations, zero-extended per unit size */
static void stat_counter_both(const u8 *db, u32 db_size,
			      const struct wave600_stats_entry *e, bool le_offs,
			      u32 *le_value, u32 *be_value, u8 raw[4])
{
	u32 offset = stat_entry_offset(e, le_offs);
	u32 unit = stat_entry_unit(e);
	u32 v = 0;
	int i;

	memset(raw, 0, 4);
	*le_value = 0;
	*be_value = 0;

	if (unit == 0 || unit > 4 || offset + unit > db_size)
		return;

	for (i = unit - 1; i >= 0; i--)
		v = (v << 8) | db[offset + i];
	*le_value = v;

	v = 0;
	for (i = 0; i < unit; i++)
		v = (v << 8) | db[offset + i];
	*be_value = v;

	memcpy(raw, db + offset, 4);
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
	bool le_layout = false;
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
					   WAVE600_STAT_TYPE_GENERAL_STATS,
					   &le_layout);
	stats->baa_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
					   WAVE600_STAT_TYPE_BAA_COUNTERS,
					   &le_layout);
	stats->hostif_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
						 WAVE600_STAT_TYPE_HOSTIF_COUNTERS,
						 &le_layout);
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
		dev_info(wdev->dev, "fw_stats init: statistics table captured (entries=%u gen_start=%d baa_start=%d)\n",
			 tab_size / sizeof(*tab), stats->gen_start,
			 stats->baa_start);

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
	bool le_layout = false;
	bool le_offs;
	bool temp_mem = false;
	int gen_start, baa_start, hostif_start;
	int valid_le, valid_be;
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

		tab = (struct wave600_stats_entry *)mem;
	}

	gen_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				   WAVE600_STAT_TYPE_GENERAL_STATS, &le_layout);
	baa_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				   WAVE600_STAT_TYPE_BAA_COUNTERS, &le_layout);
	hostif_start = stat_find_type(tab, tab_size / sizeof(*tab), ext,
				      WAVE600_STAT_TYPE_HOSTIF_COUNTERS,
				      &le_layout);
	/* Pick the offset-field endianness by entry plausibility */
	valid_le = stat_count_valid(tab, tab_size / sizeof(*tab), gen_start,
				    db_size, true, le_layout);
	valid_be = stat_count_valid(tab, tab_size / sizeof(*tab), gen_start,
				    db_size, false, le_layout);
	le_offs = valid_le > valid_be;

	n += scnprintf(out + n, out_len - n,
		       "fw_stats: db_size=%u tab_size=%u dma=0x%08x ext_id=%u table=%s\n",
		       db_size, tab_size, (u32)dma, ext->header.id,
		       stats->table_valid ? "init" : "runtime");
	n += scnprintf(out + n, out_len - n,
		       "fw_stats: entries=%u gen_start=%d baa_start=%d hostif_start=%d bitfield=%s offs=%s (valid le=%d be=%d)\n",
		       tab_size / sizeof(*tab), gen_start, baa_start,
		       hostif_start, le_layout ? "LE" : "BE",
		       le_offs ? "LE" : "BE", valid_le, valid_be);

	if (out_len - n >= 96) {
		n += scnprintf(out + n, out_len - n, "fw_stats type_table_list:");
		for (i = 0; i < (int)ARRAY_SIZE(ext->type_table_list); i++)
			n += scnprintf(out + n, out_len - n, " 0x%08x",
				       ext->type_table_list[i]);
		n += scnprintf(out + n, out_len - n, "\n");
	}

	n += scnprintf(out + n, out_len - n, "fw_stats table raw head:\n");
	n = stat_hexdump(out, out_len, n, stats->table_valid ? stats->table : mem,
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
					 stats->table_valid ? stats->table : mem,
					 min((u32)32, tab_size - gen_off), gen_off);
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
		u32 le_value, be_value;

		if (out_len - n < 96)
			break;

		if (stat_entry_type_auto(e, le_layout) !=
		    WAVE600_STAT_TYPE_GENERAL_STATS)
			break;

		stat_counter_both(mem, db_size, e, le_offs,
				  &le_value, &be_value, raw);
		n += scnprintf(out + n, out_len - n,
			       "general[%02d] %-22s off=%-5u raw=%02x%02x%02x%02x le=%-10u be=%u\n",
			       i, name, stat_entry_offset(e, le_offs),
			       raw[0], raw[1], raw[2], raw[3],
			       le_value, be_value);
	}

	/* Selected HOSTIF counters (host -> firmware data path) */
	if (hostif_start >= 0) {
		for (i = 0; i < (int)ARRAY_SIZE(stat_hostif_names); i++) {
			const struct wave600_stats_entry *e =
				&tab[hostif_start + i];
			const char *name = stat_hostif_names[i];
			u8 raw[4];
			u32 le_value, be_value;

			if (out_len - n < 96)
				break;

			if (stat_entry_type_auto(e, le_layout) !=
			    WAVE600_STAT_TYPE_HOSTIF_COUNTERS)
				break;

			stat_counter_both(mem, db_size, e, le_offs,
					  &le_value, &be_value, raw);
			n += scnprintf(out + n, out_len - n,
				       "hostif[%02d] %-22s off=%-5u raw=%02x%02x%02x%02x le=%-10u be=%u\n",
				       i, name, stat_entry_offset(e, le_offs),
				       raw[0], raw[1], raw[2], raw[3],
				       le_value, be_value);
		}
	}

	/* Named BAA counters (firmware TX engine view) */
	if (baa_start >= 0) {
		for (i = 0; i < (int)ARRAY_SIZE(stat_baa_named); i++) {
			unsigned int subtype = stat_baa_named[i].subtype;
			const struct wave600_stats_entry *e =
				&tab[baa_start + subtype];
			u8 raw[4];
			u32 le_value, be_value;

			if (out_len - n < 96)
				continue;

			if (stat_entry_type_auto(e, le_layout) !=
			    WAVE600_STAT_TYPE_BAA_COUNTERS)
				continue;

			stat_counter_both(mem, db_size, e, le_offs,
					  &le_value, &be_value, raw);
			n += scnprintf(out + n, out_len - n,
				       "baa[%02d]  %-22s off=%-5u raw=%02x%02x%02x%02x le=%-10u be=%u\n",
				       subtype, stat_baa_named[i].name,
				       stat_entry_offset(e, le_offs),
				       raw[0], raw[1], raw[2], raw[3],
				       le_value, be_value);
		}
	}

	/* Raw head of the counter database for cross-checking */
	n += scnprintf(out + n, out_len - n, "fw_stats db raw head:\n");
	n = stat_hexdump(out, out_len, n, mem,
			 min((u32)96, db_size), 0);

	ret = n;

out_free:
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

int wave600_stats_sysfs_create(struct wave600_dev *wdev)
{
	return device_create_file(wdev->dev, &dev_attr_fw_stats);
}

void wave600_stats_sysfs_remove(struct wave600_dev *wdev)
{
	device_remove_file(wdev->dev, &dev_attr_fw_stats);
}
