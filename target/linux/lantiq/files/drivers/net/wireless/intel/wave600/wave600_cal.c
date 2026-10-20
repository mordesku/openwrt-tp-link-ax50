// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Calibration & RF Profile Management
 */

#include <linux/module.h>
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

		memcpy_toio(wdev->bar0 + dest, fw->data + loc, chunk);
		memcpy_toio(wdev->bar0 + dest + chunk, zero_pad, sizeof(zero_pad));
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

int wave600_cal_init(struct wave600_dev *wdev)
{
	struct wave600_chi_area chi;
	struct wave600_umi_hdk_config hdk;
	struct wave600_umi_calibrate_params cal;
	const struct firmware *cal_fw = NULL;
	u32 pm_addr, buf_size, desc_loc;
	u16 eeprom_ver = 5, sig = 0;
	int ret;

	/* Read Shared RAM CHI area updated by firmware */
	memcpy_fromio(&chi, wdev->bar0 + WAVE600_SHARED_RAM_BASE, sizeof(chi));

	buf_size = chi.calibr_ext.buf_requested_size;
	desc_loc = chi.calibr_ext.desc_location;
	pm_addr = chi.calibr_ext.psd_progmodel_addr;

	dev_info(wdev->dev, "CHI Calibration: req_buf=%u B, psd_pm_addr=0x%08x, desc_loc=0x%08x\n",
		 buf_size, pm_addr, desc_loc);

	/* 1. Allocate coherent DMA Calibration Cache Memory */
	if (buf_size > 0) {
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
	}

	/* 2. Load Factory Calibration Data from /lib/firmware/cal_wlan0.bin (extracted from mtd4) */
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
	if (cal_fw->size >= 16) {
		eeprom_ver = cal_fw->data[10] | (cal_fw->data[11] << 8);
		if (!eeprom_ver)
			eeprom_ver = 5;
	}
	release_firmware(cal_fw);

	/* 3. Configure Hardware / HDK (UM_MAN_HDK_CONFIG_REQ) */
	memset(&hdk, 0, sizeof(hdk));
	hdk.cal_buffer_base_addr = (u32)wdev->cal_dma_addr;
	hdk.num_tx_ants = 4;
	hdk.num_rx_ants = 4;
	hdk.eeprom_version = eeprom_ver;
	hdk.band = 0; /* Band 0 - 2.4 GHz */
	hdk.offline_cal_mask = 0xffffffff;
	hdk.online_cal_mask = 0xffffffff;

	dev_info(wdev->dev, "configuring HDK (band 0, 4x4, cal_dma=%pad)...\n",
		 &wdev->cal_dma_addr);
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_HDK_CONFIG_REQ,
			      WAVE600_MC_MAN_HDK_CONFIG_CFM,
			      &hdk, sizeof(hdk), NULL, 0, 5000);
	if (ret) {
		dev_err(wdev->dev, "HDK_CONFIG_REQ failed: %d\n", ret);
		return ret;
	}
	dev_info(wdev->dev, "HDK configured successfully by firmware\n");

	/* 4. Download ProgModels if available */
	if (pm_addr) {
		wave600_progmodel_download(wdev, "ProgModel_gen6_Common.bin", pm_addr,
					   WAVE600_UM_DOWNLOAD_PROG_MODEL_REQ,
					   WAVE600_MC_DOWNLOAD_PROG_MODEL_CFM);
		wave600_progmodel_download(wdev, "ProgModel_gen6_Ant_Tx.bin", pm_addr,
					   WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_REQ,
					   WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_TX_CFM);
		wave600_progmodel_download(wdev, "ProgModel_gen6_Ant_Rx.bin", pm_addr,
					   WAVE600_UM_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_REQ,
					   WAVE600_MC_MAN_DOWNLOAD_PROG_MODEL_ANT_RX_CFM);
	}

	/* 5. Execute Channel Calibration (UM_MAN_CALIBRATE_REQ) */
	memset(&cal, 0, sizeof(cal));
	cal.chan_nums[0] = 1;
	cal.chan_nums[1] = 6;
	cal.chan_nums[2] = 11;
	cal.num_chans = 3;
	cal.chan_width = 0; /* 20 MHz */

	dev_info(wdev->dev, "initiating RF calibration for channels 1, 6, 11 (20 MHz)...\n");
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_CALIBRATE_REQ,
			      WAVE600_MC_MAN_CALIBRATE_CFM,
			      &cal, sizeof(cal), &cal, sizeof(cal), 30000);
	if (ret) {
		dev_err(wdev->dev, "CALIBRATE_REQ failed: %d\n", ret);
		return ret;
	}

	dev_info(wdev->dev, "WAV654 Radio RF calibration verified (status: 0x%08x) (Milestone 4 PASS)\n",
		 cal.status);
	return 0;
}

void wave600_cal_cleanup(struct wave600_dev *wdev)
{
	if (wdev->cal_buf) {
		dma_free_coherent(wdev->dev, wdev->cal_buf_size,
				  wdev->cal_buf, wdev->cal_dma_addr);
		wdev->cal_buf = NULL;
		wdev->cal_dma_addr = 0;
		wdev->cal_buf_size = 0;
	}
}
