// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) PCIe Driver Registration & Probe
 */

#include <linux/module.h>
#include <linux/pci.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_fw.h"
#include "wave600_dma.h"
#include "wave600_cfg80211.h"
#include "wave600_recovery.h"

static const struct pci_device_id wave600_pci_ids[] = {
	{ PCI_DEVICE(0x8086, 0x09d0) }, /* Intel WAV654 802.11ax */
	{ PCI_DEVICE(0x1bef, 0x09d0) }, /* Lantiq WAV654 802.11ax */
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, wave600_pci_ids);

static void wave600_pci_diag_work(struct work_struct *work)
{
	struct wave600_dev *wdev =
		container_of(work, struct wave600_dev, diag_work.work);

	wave600_ipc_diag_fw_state(wdev, "10s-after-init");
}

static void wave600_pci_telemetry_work(struct work_struct *work)
{
	struct wave600_dev *wdev =
		container_of(work, struct wave600_dev, telemetry_work.work);

	wave600_dma_tx_clean(&wdev->dma, WAVE600_DMA_RING_SIZE);
	wave600_stats_update_telemetry(wdev);

	if (wdev->state == W600_STATE_RUNNING)
		schedule_delayed_work(&wdev->telemetry_work, msecs_to_jiffies(1000));
}

static int wave600_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct wave600_dev *wdev;
	u32 sys_info;
	int ret;

	dev_info(&pdev->dev, "probing WAV654 802.11ax endpoint [%04x:%04x]\n",
		 pdev->vendor, pdev->device);

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable PCIe device: %d\n", ret);
		return ret;
	}

	pci_set_master(pdev);

	/* Ensure legacy INTx interrupt generation is enabled in PCI config space */
	pci_intx(pdev, 1);
	{
		u16 pci_cmd;
		pci_read_config_word(pdev, PCI_COMMAND, &pci_cmd);
		dev_info(&pdev->dev, "PCI COMMAND: 0x%04x (INTx %s)\n",
			 pci_cmd, (pci_cmd & PCI_COMMAND_INTX_DISABLE) ? "DISABLED" : "ENABLED");
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "failed to set 32-bit DMA mask: %d\n", ret);
		return ret;
	}

	ret = pcim_iomap_regions(pdev, BIT(0), DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "failed to request/map BAR0: %d\n", ret);
		return ret;
	}

	wdev = devm_kzalloc(&pdev->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdev->pdev = pdev;
	wdev->dev = &pdev->dev;
	wdev->bar0 = pcim_iomap_table(pdev)[0];
	wdev->bar0_len = pci_resource_len(pdev, 0);

	pci_set_drvdata(pdev, wdev);
	wave600_recovery_init(wdev);

	/* Read chip system information (bits 0..15: Chip ID, bits 16..31: BootMode & CPU ID) */
	sys_info = wave600_read32(wdev, WAVE600_SYS_IF_SYSTEM_INFO);
	wdev->chip_id = sys_info & 0xffff;
	wdev->chip_rev = (sys_info >> 16) & 0xffff;

	dev_info(&pdev->dev, "BAR0 mapped: %pr -> %p (sys_info: 0x%08x, Chip ID: 0x%04x, Rev: 0x%04x)\n",
		 &pdev->resource[0], wdev->bar0, sys_info, wdev->chip_id, wdev->chip_rev);

	{
		/*
		 * Board strap (vendor wave_pcieg6_is_mode_dual_pci): RCU_BOOT
		 * MSPS bits tell whether this WAV654 is wired in DUAL PCI mode
		 * (main PCIe runs the firmware, the AUX endpoint is the
		 * data-only port). AX50 vendor runner: dual_pci=1.
		 */
		u32 rcu_boot = wave600_read32(wdev, WAVE600_SOC_RCU_BOOT);
		u32 msps = (rcu_boot & WAVE600_RCU_BOOT_MSPS_MASK) >>
			   WAVE600_RCU_BOOT_MSPS_SHIFT;

		dev_info(&pdev->dev, "RCU_BOOT: 0x%08x, MSPS strap: %u (%s)\n",
			 rcu_boot, msps,
			 msps == WAVE600_MSPS_MODE_DUAL_PCI ? "DUAL PCI" : "not dual");
	}


	if (wdev->chip_id == 0xffff) {
		/* No device behind this root port: keep INTx disabled. */
		pci_intx(pdev, 0);
		dev_info(&pdev->dev, "ignoring absent WAV654 endpoint (INTx generation disabled)\n");
		return -ENODEV;
	}

	if (wdev->chip_id == 0) {
		/*
		 * AUX endpoint of the dual-PCI WAV654 (vendor
		 * WAVE_WAVE600_AUX_CHIP_ID = 0). One chip, two PCIe
		 * interfaces: the main endpoint runs the ARC firmware, the
		 * AUX is the data-only port. The vendor never stages
		 * firmware here - probe parks the device after
		 * pci_enable/set_master/BAR map ("PCI AUX Initialization
		 * finished", mtlk_mmb_drv.c). Keep INTx generation
		 * disabled so the uninitialized port cannot hold the line.
		 */
		pci_intx(pdev, 0);
		wdev->aux = true;
		dev_info(&pdev->dev, "WAV654 AUX data endpoint parked (dual-PCI data port, no firmware)\n");
		return 0;
	}

	/* Execute hardware reset and basic clock configuration */
	ret = wave600_hw_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "hardware initialization failed: %d\n", ret);
		return ret;
	}

	/* Execute firmware staging and ARC boot handshake (Milestone 2) */
	ret = wave600_fw_load_and_boot(wdev);
	if (ret) {
		dev_err(&pdev->dev, "firmware load and boot failed: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "WAV654 ARC firmware booted successfully (Milestone 2 PASS)\n");

	/* Initialize Mailbox Circular IPC Engine: CHI parse + IRQ bring-up.
	 * SW_RESET is NOT sent here; the vendor sends it only in card
	 * finalize, after the pre-activation steps below. */
	ret = wave600_ipc_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "mailbox IPC initialization failed: %d\n", ret);
		return ret;
	}

	/*
	 * Vendor HW_EEPROM_READ + HW_PSDB_READ (hw_mmb_eeprom step list):
	 * host parse of the factory calibration and PSD.bin. Runs before
	 * card finalize because the HW_DEP_CFG step needs the XTAL CIS
	 * value and the HDK config needs the PSDB antenna parameters.
	 */
	ret = wave600_cal_parse_profile(wdev);
	if (ret) {
		dev_err(&pdev->dev, "calibration profile parse failed: %d\n", ret);
		goto err_ipc;
	}

	/*
	 * Vendor pre-activation order (hw_mmb_card, hw_mmb.c:7950-7985):
	 * HW_DAT_REQ/IND_BDR -> HW_RX_DATA_BUFFERS -> HW_RX_MGMT_BUFFERS
	 * -> HW_CALIBRATION_CACHE -> HW_FW_LOGGER, all BEFORE the SW_RESET
	 * (which the vendor sends in card finalize). The firmware
	 * application initializes against these posted host resources.
	 */
	ret = wave600_dma_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "data rings pre-activation failed: %d\n", ret);
		goto err_ipc;
	}

	ret = wave600_dma_post_rx_pool(wdev);
	if (ret) {
		dev_err(&pdev->dev, "RX pool pre-activation failed: %d\n", ret);
		goto err_dma;
	}

	ret = wave600_cal_alloc_cache(wdev);
	if (ret) {
		dev_err(&pdev->dev, "calibration cache pre-activation failed: %d\n", ret);
		goto err_dma;
	}

	ret = wave600_ipc_fw_logger_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "FW logger buffer posting failed: %d\n", ret);
		goto err_dma;
	}

	/*
	 * Vendor card finalize order (hw_mmb_card_fin, hw_mmb.c):
	 * HW_TXMM = mtlk_txmm_start (HOST-side TXMM manager start - our
	 * request engine is already running, nothing to do) -> HW_TXDM
	 * -> HW_DATA_PATH -> HW_HW_DEP_CFG -> HW_FW_LOG_SEVERITY -> ...
	 * NOTE: the vendor NEVER sends UM_MAN_SW_RESET_MAC at bring-up;
	 * it is a teardown / hw-recovery message only (hw_mmb.c stop_card
	 * and mtlk_hw_mmb_restore). Sending it during probe resets the
	 * UMAC application state - removed (see
	 * docs/wave600-boot-mode-root-cause.md, 21:18 section).
	 */

	dev_info(&pdev->dev, "WAV654 Mailbox IPC initialized successfully (Milestone 3 PASS)\n");

	ret = wave600_dma_data_path_start(wdev);
	if (ret) {
		dev_err(&pdev->dev, "data path initialization failed: %d\n", ret);
		goto err_dma;
	}

	ret = wave600_hw_send_dep_cfg(wdev);
	if (ret) {
		dev_err(&pdev->dev, "HW dependent configuration failed: %d\n", ret);
		goto err_dma;
	}

	/*
	 * Vendor finalize step HW_FW_LOG_SEVERITY: UM_DBG_LOGGER_SET_SEVERITY_REQ
	 * over the DBG manager. Non-fatal: a timeout here is itself a
	 * diagnostic result (the DBG channel answer is logged either way).
	 */
	ret = wave600_ipc_dbg_set_severity(wdev, WAVE600_LOGGER_SEVERITY_INFORMATION9,
					   WAVE600_UMI_CPU_ID_UM);
	if (ret)
		dev_err(&pdev->dev, "FW logger severity configuration failed: %d\n", ret);

	/*
	 * Vendor finalize step HW_MHI_STATISTIC: the statistics descriptor
	 * table must be fetched inside the init window; the sysfs dump then
	 * only re-fetches the counter database. Non-fatal on failure.
	 */
	ret = wave600_stats_init(wdev);
	if (ret)
		dev_err(&pdev->dev, "FW statistics table init failed: %d\n", ret);

	ret = wave600_stats_sysfs_create(wdev);
	if (ret)
		dev_err(&pdev->dev, "failed to create fw_stats sysfs attribute: %d\n", ret);

	ret = wave600_recovery_sysfs_create(wdev);
	if (ret)
		dev_err(&pdev->dev, "failed to create recovery_trigger sysfs attribute: %d\n", ret);

	/*
	 * AP stack: the cfg80211 layer creates the AP netdev (it owns the
	 * netdev lifecycle), then the DMA layer attaches NAPI and the
	 * ndo_start_xmit to it.
	 */
	ret = wave600_cfg80211_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "cfg80211 initialization failed: %d\n", ret);
		goto err_dma;
	}

	wave600_dma_attach(wdev);

	/*
	 * RF bring-up (vendor radio bring-up after card finalize/READY):
	 * HDK configuration, ProgModel + PSDB download and the RF channel
	 * calibration (Milestone 4).
	 */
	ret = wave600_cal_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "RF bring-up failed: %d\n", ret);
		goto err_rf;
	}

	dev_info(&pdev->dev, "WAV654 RF Calibration completed successfully (Milestone 4 PASS)\n");

	/*
	 * One-shot handler-liveness diagnostic (positive control
	 * GET_MAC_VERSION + negative controls SET_CHAN chan=200 /
	 * SET_MIB object 0xDEAD). Decides whether the mailbox
	 * confirmations carry real handler responses or generic
	 * pseudo-ACKs with zero status.
	 */
	wave600_diag_handlers_probe(wdev);

	/*
	 * Read-only FW state snapshot (core info / MIPS_CTL / ring
	 * counters / FW logger buffers), plus a second snapshot 10 s
	 * later. Any movement between the two snapshots is a platform
	 * heartbeat signal independent of the mailbox pseudo-ACK issue.
	 */
	wave600_ipc_diag_fw_state(wdev, "post-init");
	INIT_DELAYED_WORK(&wdev->diag_work, wave600_pci_diag_work);
	schedule_delayed_work(&wdev->diag_work, msecs_to_jiffies(10000));

	INIT_DELAYED_WORK(&wdev->telemetry_work, wave600_pci_telemetry_work);
	schedule_delayed_work(&wdev->telemetry_work, msecs_to_jiffies(1000));

	return 0;

err_rf:
	wave600_dma_stop_netdev(wdev);
	wave600_cfg80211_cleanup(wdev);
err_dma:
	wave600_recovery_sysfs_remove(wdev);
	wave600_stats_cleanup(wdev);
	wave600_dma_cleanup(wdev);
	wave600_ipc_fw_logger_cleanup(wdev);
	wave600_cal_cleanup(wdev);
	wave600_ipc_cleanup(wdev);
	wave600_hw_reset(wdev);
	return ret;
err_ipc:
	wave600_recovery_cleanup(wdev);
	wave600_ipc_cleanup(wdev);
	wave600_hw_reset(wdev);
	return ret;
}

static void wave600_pci_remove(struct pci_dev *pdev)
{
	struct wave600_dev *wdev = pci_get_drvdata(pdev);

	if (wdev) {
		if (wdev->aux) {
			/* Parked AUX data endpoint: BAR mapping is devm-managed. */
			dev_info(&pdev->dev, "releasing parked WAV654 AUX endpoint\n");
			return;
		}
		dev_info(&pdev->dev, "removing WAV654 driver\n");
		wave600_recovery_sysfs_remove(wdev);
		wave600_recovery_cleanup(wdev);
		cancel_delayed_work_sync(&wdev->diag_work);
		cancel_delayed_work_sync(&wdev->telemetry_work);
		wave600_stats_sysfs_remove(wdev);
		wave600_stats_cleanup(wdev);
		wave600_dma_stop_netdev(wdev);
		wave600_dma_cleanup(wdev);
		wave600_cfg80211_cleanup(wdev);
		wave600_cal_cleanup(wdev);
		wave600_ipc_fw_logger_cleanup(wdev);
		wave600_ipc_cleanup(wdev);
		wave600_hw_reset(wdev);
	}
}

static struct pci_driver wave600_pci_driver = {
	.name = DRV_NAME,
	.id_table = wave600_pci_ids,
	.probe = wave600_pci_probe,
	.remove = wave600_pci_remove,
};

module_pci_driver(wave600_pci_driver);

MODULE_AUTHOR("OpenWrt Community");
MODULE_DESCRIPTION("Intel / MaxLinear Wave600 (WAV654) Wi-Fi 6 Driver");
MODULE_LICENSE("GPL");
