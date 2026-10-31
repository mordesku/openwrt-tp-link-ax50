// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Firmware Staging & Boot Engine
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_chi.h"
#include "wave600_fw.h"

static int wave600_fw_copy_chunk(struct wave600_dev *wdev, u32 dst_offset,
				 const u8 *data, size_t size, size_t max_size)
{
	const u32 *src32;
	void __iomem *dst;
	size_t words;
	size_t i;

	if (size > max_size) {
		dev_err(wdev->dev, "firmware chunk size %zu exceeds max %zu\n", size, max_size);
		return -EINVAL;
	}

	dst = wdev->bar0 + dst_offset;
	src32 = (const u32 *)data;
	words = size / 4;

	for (i = 0; i < words; i++) {
		__raw_writel(cpu_to_le32(src32[i]), dst + (i * 4));
		if ((i & 0x3ff) == 0) {
			mmiowb();
			cond_resched();
		}
		if (size >= 262144 && (i & 0xffff) == 0 && i > 0) {
			dev_info(wdev->dev, "IRAM staging progress: %zu / %zu bytes (%zu%%)\n",
				 i * 4, size, (i * 4 * 100) / size);
		}
	}

	if (size % 4) {
		u32 trail = 0;
		memcpy(&trail, data + (words * 4), size % 4);
		__raw_writel(cpu_to_le32(trail), dst + (words * 4));
	}
	mmiowb();

	return 0;
}

static int wave600_fw_load_file(struct wave600_dev *wdev, const char *name,
				u32 dst_offset, size_t max_size)
{
	const struct firmware *fw;
	int ret;

	dev_info(wdev->dev, "requesting firmware '%s'...\n", name);
	ret = request_firmware(&fw, name, wdev->dev);
	if (ret) {
		dev_err(wdev->dev, "failed to load firmware '%s': %d\n", name, ret);
		return ret;
	}

	dev_info(wdev->dev, "staging '%s' (%zu bytes) to IRAM 0x%08x\n",
		 name, fw->size, dst_offset);

	/* Extract firmware version from Upper MAC binary (matching vendor get_firmware_version) */
	if (strcmp(name, WAVE600_FW_UPPER_MAC) == 0 && fw->data && fw->size > 64) {
		const char *sig = "version number: ";
		const char *p = strnstr((const char *)fw->data, sig, fw->size);
		if (p) {
			const char *start = p + strlen(sig);
			const char *end = strnstr(start, "@@@", fw->size - (start - (const char *)fw->data));
			size_t len = end ? (size_t)(end - start) : sizeof(wdev->fw_version) - 1;
			if (len >= sizeof(wdev->fw_version))
				len = sizeof(wdev->fw_version) - 1;
			memcpy(wdev->fw_version, start, len);
			wdev->fw_version[len] = '\0';
			while (len > 0 && (wdev->fw_version[len - 1] == ' ' || wdev->fw_version[len - 1] == '\t')) {
				wdev->fw_version[len - 1] = '\0';
				len--;
			}
			dev_info(wdev->dev, "ARC Upper CPU firmware version: '%s'\n", wdev->fw_version);
		}
	}

	ret = wave600_fw_copy_chunk(wdev, dst_offset, fw->data, fw->size, max_size);
	release_firmware(fw);
	return ret;
}

static int wave600_fw_stage_all(struct wave600_dev *wdev)
{
	int ret;

	/* 1. Upper MAC firmware (ap_ram_gen6_wrx_600_real_phy.bin) */
	ret = wave600_fw_load_file(wdev, WAVE600_FW_UPPER_MAC,
				   WAVE600_CPU_IRAM_START, WAVE600_CPU_IRAM_SIZE);
	if (ret)
		return ret;

	/* 2. TX Sender firmware (Band 0 and Band 1) */
	ret = wave600_fw_load_file(wdev, WAVE600_FW_TX_SENDER,
				   WAVE600_B0_TXCPU_IRAM_START, WAVE600_B0_TXCPU_IRAM_SIZE);
	if (ret)
		return ret;

	ret = wave600_fw_load_file(wdev, WAVE600_FW_TX_SENDER,
				   WAVE600_B1_TXCPU_IRAM_START, WAVE600_B1_TXCPU_IRAM_SIZE);
	if (ret)
		return ret;

	/* 3. Host Interface firmware */
	ret = wave600_fw_load_file(wdev, WAVE600_FW_HOST_IF,
				   WAVE600_IFCPU_IRAM_START, WAVE600_IFCPU_IRAM_SIZE);
	if (ret)
		return ret;

	/* 4. RX Handler firmware (Band 0 and Band 1) */
	ret = wave600_fw_load_file(wdev, WAVE600_FW_RX_HANDLER,
				   WAVE600_B0_RXCPU_IRAM_START, WAVE600_B0_RXCPU_IRAM_SIZE);
	if (ret)
		return ret;

	ret = wave600_fw_load_file(wdev, WAVE600_FW_RX_HANDLER,
				   WAVE600_B1_RXCPU_IRAM_START, WAVE600_B1_RXCPU_IRAM_SIZE);
	if (ret)
		return ret;

	return 0;
}

static void wave600_fw_init_chi(struct wave600_dev *wdev)
{
	void __iomem *shram = wdev->bar0 + WAVE600_SHARED_RAM_BASE;
	struct wave600_chi_card_cfg_ext card_cfg;
	size_t offset = offsetof(struct wave600_chi_area, card_cfg_ext);

	dev_info(wdev->dev, "initializing Shared RAM CHI Vector Area (size %zu, card_cfg offset 0x%zx)\n",
		 sizeof(struct wave600_chi_area), offset);

	/* Zero out the entire CHI Vector Area in SHRAM */
	memset_io(shram, 0, sizeof(struct wave600_chi_area));

	/* Configure card extension (ID 12) per Rule 22 (native host format in SHRAM)
	 * and vendor _mtlk_mmb_chi_init (verified in hybrid boot log:
	 * "_wave_hw_dual_pci_mode_set: dual PCI mode OFF",
	 * "_wave_hw_chi_area_write: id 12, data 0x01, 0x00, 0x00"):
	 * band_cfg  = DUAL_BAND  (WAVE600_BAND_CFG_DUAL_BAND = 1)
	 * pcie_cfg  = SINGLE_PCI (WAVE600_PCI_CFG_SINGLE_PCI = 0)
	 * platform  = 0 (real ASIC) */
	memset(&card_cfg, 0, sizeof(card_cfg));
	card_cfg.header.magic = WAVE600_HOST_EXTENSION_MAGIC;
	card_cfg.header.id = WAVE600_CHI_EXT_ID_CARD_CFG;
	card_cfg.header.size = sizeof(struct wave600_chi_card_cfg_data);

	card_cfg.data.band_cfg = WAVE600_BAND_CFG_DUAL_BAND;
	card_cfg.data.pcie_cfg = WAVE600_PCI_CFG_SINGLE_PCI;
	card_cfg.data.platform_type = 0;

	memcpy_toio(shram + offset, &card_cfg, sizeof(card_cfg));
}

static int wave600_fw_wait_chi_magic(struct wave600_dev *wdev)
{
	u32 magic_reg = WAVE600_SHARED_RAM_BASE + offsetof(struct wave600_chi_area, basic.magic);
	u32 dbg_reg = WAVE600_SHARED_RAM_BASE + offsetof(struct wave600_chi_area, debug_ext);
	u32 val = 0;
	int timeout = 500; /* 500 * 10 ms = 5000 ms */

	dev_info(wdev->dev, "waiting for ARC firmware boot handshake (CHI Magic 0xBEADFEED)...\n");

	while (timeout--) {
		val = wave600_read32(wdev, magic_reg);
		if (val == WAVE600_CHI_MAGIC)
			break;
		msleep(10);
	}

	if (val != WAVE600_CHI_MAGIC) {
		u32 core0 = wave600_read32(wdev, dbg_reg + offsetof(struct wave600_chi_debug_ext, core0_info));
		u32 core1 = wave600_read32(wdev, dbg_reg + offsetof(struct wave600_chi_debug_ext, core1_info));
		u32 core2 = wave600_read32(wdev, dbg_reg + offsetof(struct wave600_chi_debug_ext, core2_info));

		dev_err(wdev->dev, "timeout waiting for ARC firmware boot (read: 0x%08x, expected 0x%08x)\n",
			val, WAVE600_CHI_MAGIC);
		dev_err(wdev->dev, "CHI debug cores info: LM0: 0x%08x, LM1: 0x%08x, UM: 0x%08x\n",
			core0, core1, core2);
		return -ETIMEDOUT;
	}

	dev_info(wdev->dev, "ARC firmware boot handshake verified: 0x%08x (Milestone 2 PASS)\n", val);
	return 0;
}

int wave600_fw_load_and_boot(struct wave600_dev *wdev)
{
	int ret;

	/*
	 * ARC CPUs were already placed into reset at 40 MHz XO during wave600_hw_reset(),
	 * and MAC GenRISC QManager clocks were powered on in wave600_hw_init().
	 * Do NOT touch CPU reset registers at 640 MHz before staging firmware (matching vendor 1:1).
	 */

	/* Initialize Shared RAM CHI vector area */
	wave600_fw_init_chi(wdev);

	/* Stage all 4 firmware images to IRAM */
	ret = wave600_fw_stage_all(wdev);
	if (ret) {
		dev_err(wdev->dev, "firmware staging failed: %d\n", ret);
		return ret;
	}

	/* Sanity check: verify first 16 bytes in Upper MAC IRAM */
	dev_info(wdev->dev, "IRAM upper MAC vectors: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		 wave600_read32(wdev, WAVE600_CPU_IRAM_START + 0x00),
		 wave600_read32(wdev, WAVE600_CPU_IRAM_START + 0x04),
		 wave600_read32(wdev, WAVE600_CPU_IRAM_START + 0x08),
		 wave600_read32(wdev, WAVE600_CPU_IRAM_START + 0x0c));

	/* Clear and mask host interrupts before releasing CPUs */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	mmiowb();

	/* Release ARC CPUs from reset to start execution */
	wave600_hw_release_cpus(wdev);

	/* Wait for CHI magic handshake from ARC firmware */
	return wave600_fw_wait_chi_magic(wdev);
}

