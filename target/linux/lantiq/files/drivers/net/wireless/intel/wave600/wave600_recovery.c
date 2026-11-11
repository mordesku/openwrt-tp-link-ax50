// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Hardware Fault Recovery Engine
 *
 * Implements full hardware fault recovery and re-initialization (mirroring
 * vendor fw_recovery.c / mtlk_hw_mmb_restore in hw_mmb.c):
 *
 * 1. Isolation:
 *    - Mask host interrupts in MAC (HOST_IRQ_MASK = 0).
 *    - Detach NAPI and stop all netdev TX queues.
 *    - Abort pending IPC mailbox commands with -ENETRESET.
 *    - Flush uncompleted TX descriptors in the DMA ring.
 *
 * 2. Hardware Reset & Core Bring-up:
 *    - Put all three ARC CPUs into hardware reset (wave600_hw_put_cpus_to_reset).
 *    - Clear MAC RAB PHI latch.
 *    - Re-initialize hardware clocks, eFuse trims and PLL (wave600_hw_init).
 *    - Re-stage firmwares into IRAM and release CPUs (wave600_fw_load_and_boot).
 *    - Verify CHI Magic 0xBEADFEED.
 *
 * 3. Re-arm IPC & Pre-activation:
 *    - Reset FIFO indices and DMA ring pointers.
 *    - Re-post RX pools and FW logger buffers to SHRAM.
 *    - Bind data rings via UM_MAN_DATA_PATH_INIT_REQ.
 *    - Re-send HW dependent configuration.
 *
 * 4. RF Re-configuration & Calibration:
 *    - Re-download ProgModels (Common, Ant_Tx, Ant_Rx).
 *    - Re-send eFuse AFE & RSSI calibration data.
 *    - Re-send HDK and PSDB platform tables for both radios.
 *    - Execute channel calibration (wave600_cal_init).
 *
 * 5. AP Restoration & Finalize:
 *    - For each running radio (Radio 0 / Radio 1), re-send VAP_DB ADD, MIBs,
 *      ADD_VAP, ENABLE_RADIO, SET_CHAN, TPC config, beacon template, and BSS.
 *    - Re-enable NAPI, unmask interrupts (HOST_IRQ_MASK = MTLK_IRQ_ALL) and
 *      wake netdev queues.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_fw.h"
#include "wave600_dma.h"
#include "wave600_cal.h"
#include "wave600_cfg80211.h"
#include "wave600_recovery.h"

static void wave600_recovery_work_fn(struct work_struct *work);

void wave600_recovery_init(struct wave600_dev *wdev)
{
	INIT_WORK(&wdev->recovery_work, wave600_recovery_work_fn);
	atomic_set(&wdev->in_recovery, 0);
	wdev->state = W600_STATE_RUNNING;
}

void wave600_recovery_cleanup(struct wave600_dev *wdev)
{
	cancel_work_sync(&wdev->recovery_work);
	atomic_set(&wdev->in_recovery, 0);
}

void wave600_recovery_trigger(struct wave600_dev *wdev, const char *reason)
{
	if (!wdev)
		return;

	if (atomic_read(&wdev->in_recovery) || wdev->state == W600_STATE_RECOVERY) {
		dev_warn(wdev->dev, "recovery already in progress (ignoring trigger: %s)\n",
			 reason ? reason : "unspecified");
		return;
	}

	dev_warn(wdev->dev, "triggering hardware fault recovery (reason: %s)\n",
		 reason ? reason : "unspecified");
	wdev->state = W600_STATE_FAULT;
	schedule_work(&wdev->recovery_work);
}

static ssize_t recovery_trigger_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wave600_dev *wdev = pci_get_drvdata(pdev);

	if (!wdev)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE,
			 "state=%d in_recovery=%d consecutive_timeouts=%u\n"
			 "Usage: echo 1 > recovery_trigger to trigger fault recovery\n",
			 wdev->state, atomic_read(&wdev->in_recovery),
			 wdev->ipc.consecutive_timeouts);
}

static ssize_t recovery_trigger_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wave600_dev *wdev = pci_get_drvdata(pdev);

	if (!wdev)
		return -ENODEV;

	dev_warn(wdev->dev, "manual recovery triggered via sysfs\n");
	wave600_recovery_trigger(wdev, "manual sysfs request");
	return count;
}
static DEVICE_ATTR_RW(recovery_trigger);

int wave600_recovery_sysfs_create(struct wave600_dev *wdev)
{
	if (!wdev || !wdev->dev)
		return -EINVAL;
	return device_create_file(wdev->dev, &dev_attr_recovery_trigger);
}

void wave600_recovery_sysfs_remove(struct wave600_dev *wdev)
{
	if (wdev && wdev->dev)
		device_remove_file(wdev->dev, &dev_attr_recovery_trigger);
}

static void wave600_recovery_work_fn(struct work_struct *work)
{
	struct wave600_dev *wdev = container_of(work, struct wave600_dev, recovery_work);
	unsigned long flags;
	int ret, i;

	if (atomic_cmpxchg(&wdev->in_recovery, 0, 1) != 0) {
		dev_warn(wdev->dev, "recovery worker already active, skipping duplicate run\n");
		return;
	}

	wdev->state = W600_STATE_RECOVERY;
	dev_warn(wdev->dev, "=== WAV654 HARDWARE FAULT RECOVERY STARTING ===\n");

	/* ------------------------------------------------------------------ */
	/* 1. Isolation: mask IRQ, stop NAPI & netdev queues, abort IPC       */
	/* ------------------------------------------------------------------ */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	mmiowb();

	cancel_delayed_work(&wdev->telemetry_work);
	wave600_dma_stop_netdev(wdev);

	for (i = 0; i < 2; i++) {
		struct wave600_radio *r = &wdev->radios[i];
		r->radio_enabled = false;
		r->vap_added = false;
		if (r->ndev) {
			netif_carrier_off(r->ndev);
			netif_tx_stop_all_queues(r->ndev);
		}
	}

	/* Abort pending IPC mailbox commands */
	spin_lock_irqsave(&wdev->ipc.lock, flags);
	for (i = 0; i < WAVE600_IPC_MAX_MAN_REQS; i++) {
		if (wdev->ipc.cmds[i].in_use) {
			wdev->ipc.cmds[i].status = -ENETRESET;
			complete(&wdev->ipc.cmds[i].done);
		}
	}
	if (wdev->ipc.dbg_cmd.in_use) {
		wdev->ipc.dbg_cmd.status = -ENETRESET;
		complete(&wdev->ipc.dbg_cmd.done);
	}
	spin_unlock_irqrestore(&wdev->ipc.lock, flags);

	/* Flush uncompleted TX descriptors in the DMA ring */
	spin_lock_irqsave(&wdev->dma.tx_lock, flags);
	for (i = 0; i < WAVE600_DMA_RING_SIZE; i++) {
		if (wdev->dma.tx_skbs[i]) {
			dma_unmap_single(wdev->dev, wdev->dma.tx_dma[i],
					 wdev->dma.tx_len[i], DMA_TO_DEVICE);
			dev_kfree_skb_any(wdev->dma.tx_skbs[i]);
			wdev->dma.tx_skbs[i] = NULL;
		}
	}
	wdev->dma.tx_in_use = 0;
	wdev->dma.tx_write = 0;
	spin_unlock_irqrestore(&wdev->dma.tx_lock, flags);

	dev_info(wdev->dev, "recovery: step 1 (isolation) complete\n");

	/* ------------------------------------------------------------------ */
	/* 2. Hardware Reset & Core Bring-up                                   */
	/* ------------------------------------------------------------------ */
	wave600_hw_put_cpus_to_reset(wdev);
	msleep(20);

	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	mmiowb();

	ret = wave600_hw_init(wdev);
	if (ret) {
		dev_err(wdev->dev, "recovery: hardware re-init failed: %d\n", ret);
		goto err_out;
	}

	ret = wave600_fw_load_and_boot(wdev);
	if (ret) {
		dev_err(wdev->dev, "recovery: firmware reload and boot failed: %d\n", ret);
		goto err_out;
	}

	dev_info(wdev->dev, "recovery: step 2 (HW reset & ARC boot) complete\n");

	/* ------------------------------------------------------------------ */
	/* 3. Re-arm IPC & Pre-activation resources                           */
	/* ------------------------------------------------------------------ */
	spin_lock_irqsave(&wdev->ipc.lock, flags);
	wdev->ipc.req_idx = 0;
	wdev->ipc.ind_idx = 0;
	wdev->ipc.irq_status = 0;
	wdev->ipc.empty_runs = 0;
	wdev->ipc.storm_latched = false;
	wdev->ipc.consecutive_timeouts = 0;
	spin_unlock_irqrestore(&wdev->ipc.lock, flags);

	wave600_dma_reset_rings(wdev);
	wave600_dma_repost_rx_pool(wdev);
	wave600_ipc_fw_logger_init(wdev);

	/*
	 * Enable PCIe endpoint legacy INT generation and unmask host IRQ in MAC
	 * before sending UMI mailbox commands. wave600_ipc_cmd() relies on the
	 * interrupt handler to complete commands.
	 */
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0x1);
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, WAVE600_IRQ_ALL);
	mmiowb();
	(void)wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK);
	wdev->ipc.irq_enabled = true;

	ret = wave600_dma_data_path_start(wdev);
	if (ret) {
		dev_err(wdev->dev, "recovery: DATA_PATH_INIT failed: %d\n", ret);
		goto err_out;
	}

	ret = wave600_hw_send_dep_cfg(wdev);
	if (ret) {
		dev_err(wdev->dev, "recovery: HW_DEP_CFG failed: %d\n", ret);
		goto err_out;
	}

	wave600_ipc_dbg_set_severity(wdev, WAVE600_LOGGER_SEVERITY_INFORMATION9,
				     WAVE600_UMI_CPU_ID_UM);

	dev_info(wdev->dev, "recovery: step 3 (IPC & DMA rings re-armed) complete\n");

	/* ------------------------------------------------------------------ */
	/* 4. RF Re-configuration & Calibration                                */
	/* ------------------------------------------------------------------ */
	ret = wave600_cal_init(wdev);
	if (ret) {
		dev_err(wdev->dev, "recovery: RF calibration re-init failed: %d\n", ret);
		goto err_out;
	}

	dev_info(wdev->dev, "recovery: step 4 (RF calibration) complete\n");

	/* ------------------------------------------------------------------ */
	/* 5. Restore active APs and BSS instances                             */
	/* ------------------------------------------------------------------ */
	for (i = 0; i < 2; i++) {
		struct wave600_radio *r = &wdev->radios[i];

		if (r->ap_started) {
			ret = wave600_cfg80211_restore_ap(r);
			if (ret)
				dev_err(wdev->dev, "recovery: restore AP failed on radio %d: %d\n",
					r->radio_idx, ret);
		}
	}

	dev_info(wdev->dev, "recovery: step 5 (AP restoration) complete\n");

	/* ------------------------------------------------------------------ */
	/* 6. Finalize: enable NAPI, resume network                            */
	/* ------------------------------------------------------------------ */
	wave600_dma_enable_napi(wdev);

	wdev->state = W600_STATE_RUNNING;
	atomic_set(&wdev->in_recovery, 0);

	schedule_delayed_work(&wdev->telemetry_work, msecs_to_jiffies(1000));

	dev_info(wdev->dev, "=== WAV654 HARDWARE FAULT RECOVERY COMPLETED SUCCESSFULLY ===\n");
	return;

err_out:
	dev_err(wdev->dev, "=== WAV654 HARDWARE RECOVERY FAILED (%d) ===\n", ret);
	wdev->state = W600_STATE_FAULT;
	atomic_set(&wdev->in_recovery, 0);
}
