// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) Mailbox IPC & UMI Command Engine
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_chi.h"
#include "wave600_ipc.h"

/*
 * Consume completed messages from the Indication FIFO.
 * Runs in tasklet context only (ind_idx is owned by the tasklet).
 * Returns the number of messages consumed (0 if the FIFO was empty).
 */
static int wave600_ipc_process_fifo_ind(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	int processed = 0;
	int max_to_process = ipc->ind_num ? ipc->ind_num : 16;

	while (processed < max_to_process) {
		u32 descr, type, idx, radio, vap;

		descr = wave600_read32(wdev, ipc->ind_offset + (ipc->ind_idx * sizeof(u32)));

		/* NULL descriptor (type 0) means no more messages in FIFO */
		if (wave600_descr_get_type(descr) == WAVE600_MSG_TYPE_NULL)
			break;

		/* Zero the handled descriptor in Shared RAM */
		wave600_write32(wdev, ipc->ind_offset + (ipc->ind_idx * sizeof(u32)), 0);
		mmiowb();

		/* Clear interrupt unconditionally as vendor does (gen6) */
		wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
		mmiowb();

		type = wave600_descr_get_type(descr);
		idx = wave600_descr_get_index(descr);
		radio = wave600_descr_get_radio(descr);
		vap = wave600_descr_get_vap(descr);

		ipc->ind_idx++;
		if (ipc->ind_idx >= ipc->ind_num)
			ipc->ind_idx = 0;

		processed++;

		if (type == WAVE600_MSG_TYPE_MAN_REQ) {
			/*
			 * Confirmation (MSG_CFM) to a Management Request:
			 * 'idx' corresponds to the request slot allocated by host.
			 */
			if (idx < WAVE600_IPC_MAX_MAN_REQS && ipc->cmds[idx].in_use) {
				struct wave600_ipc_cmd *cmd = &ipc->cmds[idx];
				void __iomem *slot_addr = wdev->bar0 + ipc->man_req_offset +
							  (idx * ipc->man_req_slot_size);
				struct wave600_umi_msg_hdr hdr;

				memcpy_fromio(&hdr, slot_addr, sizeof(hdr));
				cmd->cfm_id = hdr.msg_id;

				if (cmd->resp_buf && cmd->resp_len > 0) {
					size_t copy_len = min(cmd->resp_len,
							      (size_t)(ipc->man_req_slot_size - sizeof(hdr)));
					memcpy_fromio(cmd->resp_buf, slot_addr + sizeof(hdr), copy_len);
				}

				ipc->cfm_count++;
				cmd->status = 0;
				complete(&cmd->done);
			} else {
				dev_warn(wdev->dev, "unexpected MAN CFM for slot %u\n", idx);
			}
		} else if (type == WAVE600_MSG_TYPE_MAN_IND) {
			/*
			 * Asynchronous Indication from firmware:
			 * 'idx' corresponds to an indication slot in sMAN.ind.
			 */
			u32 bounce_len = min(ipc->man_ind_slot_size,
					     (u32)WAVE600_IPC_MAX_IND_BOUNCE);
			void __iomem *ind_slot = wdev->bar0 + ipc->man_ind_offset +
						 (idx * ipc->man_ind_slot_size);
			u8 bounce[WAVE600_IPC_MAX_IND_BOUNCE];
			struct wave600_umi_msg_hdr *ind_hdr = (struct wave600_umi_msg_hdr *)bounce;
			u32 resp_descr;

			/* Copy the full IOM out of the slot (vendor _mtlk_mmb_handle_cm_ind) */
			memcpy_fromio(bounce, ind_slot, bounce_len);
			dev_info_ratelimited(wdev->dev, "asynchronous indication 0x%04x on radio %u vap %u (slot %u)\n",
					    ind_hdr->msg_id, radio, vap, idx);

			/*
			 * Release the slot back to firmware (vendor _mtlk_mmb_resp_cm_ind):
			 * the IOM contents are written back before the release descriptor
			 * is posted, then the ARRAY_MAN_IND descriptor goes through the
			 * Request FIFO so firmware can reuse this slot.
			 */
			memcpy_toio(ind_slot, bounce, bounce_len);
			mmiowb();

			ipc->ind_count++;
			resp_descr = wave600_msg_to_descr(WAVE600_MSG_TYPE_MAN_IND, idx, radio, vap);
			spin_lock(&ipc->lock);
			wave600_write32(wdev, ipc->req_offset + (ipc->req_idx * sizeof(u32)), resp_descr);
			ipc->req_idx++;
			if (ipc->req_idx >= ipc->req_num)
				ipc->req_idx = 0;
			mmiowb();

			/* Ring doorbell */
			wave600_write32(wdev, WAVE600_MAC_GEN_NPU2UPI_SET, 0x1);
			mmiowb();
			spin_unlock(&ipc->lock);
		} else {
			dev_dbg(wdev->dev, "unhandled descriptor type %u, index %u\n", type, idx);
		}
	}

	return processed;
}

/*
 * Deferred interrupt processing (vendor mtlk_hw_mmb_deferred_handler +
 * IRQ_ENABLE_OR_RESCHEDULE). HOST_IRQ_MASK stays 0 while the tasklet runs;
 * it is re-enabled only after the work completes.
 */
static void wave600_ipc_tasklet_fn(struct tasklet_struct *t)
{
	struct wave600_ipc *ipc = from_tasklet(ipc, t, tasklet);
	struct wave600_dev *wdev = ipc->wdev;
	int processed;

	processed = wave600_ipc_process_fifo_ind(wdev);

	if (processed > 0) {
		ipc->empty_runs = 0;
	} else if (++ipc->empty_runs >= WAVE600_IPC_STORM_LIMIT && !ipc->storm_latched) {
		/*
		 * Storm guard (inverse of vendor MTCFG_LEGACY_INT_RECOVERY_MON):
		 * firmware keeps re-asserting the RAB PHI while the Indication
		 * FIFO is empty. Leave HOST_IRQ_MASK at 0 so the PCIe level line
		 * stays deasserted and the system survives; log one snapshot.
		 */
		ipc->storm_latched = true;
		dev_err(wdev->dev,
			"IRQ storm guard: %u empty service runs, HOST_IRQ_MASK left at 0 (STATUS=0x%08x, PHI_IRQ=0x%08x, INTR_VEC=0x%08x, isr=%u spurious=%u cfm=%u ind=%u)\n",
			ipc->empty_runs,
			wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_STATUS),
			wave600_read32(wdev, WAVE600_MAC_GEN_PHI_IRQ),
			wave600_read32(wdev, WAVE600_SOC_PCIE_INTR_VEC),
			ipc->isr_total, ipc->isr_spurious,
			ipc->cfm_count, ipc->ind_count);
		return;
	}

	/* Cleanup path: do not re-arm interrupts once the driver is going away */
	if (!ipc->irq_enabled)
		return;

	/* Re-enable host interrupts (vendor single-IRQ legacy mask MTLK_IRQ_ALL) */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, WAVE600_IRQ_ALL);
	mmiowb();
}

irqreturn_t wave600_ipc_isr(int irq, void *dev_id)
{
	struct wave600_dev *wdev = dev_id;
	struct wave600_ipc *ipc = &wdev->ipc;
	u32 status;

	/*
	 * Vendor-grade level-INTx servicing
	 * (hw_mmb.c mtlk_hw_mmb_interrupt_handler_legacy +
	 *  mtlk_ccr_pcieg6.c _mtlk_pcieg6_{disable,clear}_interrupts_if_pending):
	 *
	 * 1. HOST_IRQ_MASK = 0 immediately deasserts the PCIe level line.
	 *    Read back to make sure the write reached the endpoint.
	 * 2. HOST_IRQ_STATUS is a live condition register (RO); read it
	 *    only after masking.
	 * 3. Clear the RAB PHI latch with 0x1 only if the CONTROL bit is set.
	 * 4. Empty status => not ours / spurious: re-enable and return
	 *    IRQ_NONE so the kernel spurious-IRQ watchdog can catch a
	 *    permanently asserted line instead of looping forever.
	 * 5. Otherwise stash the status, schedule the tasklet and keep the
	 *    mask at 0 until processing completes (vendor isr_action_normal).
	 */
	/*
	 * Vendor pcie_intx_war() (GIC driver, patch 3046): clear the GRX500
	 * Root Complex INTx latch (PCIE_IRNCR, write-1-to-clear, INTA = bit 13)
	 * at irq unmask and at the end of every dispatch. The RC asserts the
	 * GIC line HIGH on Assert_INTA and only the IRNCR write releases it.
	 */
	if (ipc->rc_app)
		__raw_writel(WAVE600_RC_INTA, ipc->rc_app + WAVE600_RC_APP_IRNCR);

	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	(void)wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK);

	status = wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_STATUS);

	if (status & WAVE600_IRQ_CONTROL) {
		wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
		mmiowb();
	}

	if (!ipc->irq_enabled) {
		/* Racing driver cleanup: leave interrupts masked */
		return IRQ_NONE;
	}

	ipc->isr_total++;

	if (!(status & WAVE600_IRQ_ALL)) {
		u32 irnen = 0, irncr = 0, irnicr = 0;

		ipc->isr_spurious++;

		if (ipc->rc_app) {
			/*
			 * RC-side diagnostics: read the GRX500 Root Complex
			 * interrupt node state for this root port.
			 */
			irnicr = __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNICR);
			irncr = __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNCR);
			irnen = __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNEN);
		}

		/*
		 * Diagnostic: an assert source invisible in HOST_IRQ_STATUS
		 * (and ungated by HOST_IRQ_MASK) keeps the INTx level line
		 * asserted. Read the raw 8-source PCIe interrupt vector
		 * (SocRegsRegs.h REG_SOC_REGS_PCIE_INTR_VEC):
		 *   bit 0/1: Host I/F data Tx/Rx out
		 *   bit 2/3: MGMT Tx/Rx out
		 *   bit 4:   FW-Driver mailbox
		 *   bit 5:   RAB PHI interrupt
		 *   bit 6:   WLAN IP's XBAR error
		 *   bit 7:   WDT interrupt (invisible in HOST_IRQ_STATUS,
		 *            which covers only bits 0..6)
		 * Plus the AMBA order manager WDT level status (0x6C).
		 * Print the first occurrence and every 10000th to identify
		 * the storm source without flooding the console.
		 */
		if (ipc->isr_spurious == 1 || (ipc->isr_spurious % 10000) == 0) {
			u32 vec = wave600_read32(wdev, WAVE600_SOC_PCIE_INTR_VEC);
			u32 wdt = wave600_read32(wdev, WAVE600_SOC_PCIE_AMBA_ORDR_WDT);

			dev_err(wdev->dev,
				"unhandled INTx #%u: HOST_IRQ_STATUS=0x%08x, PCIE_INTR_VEC=0x%08x, AMBA_ORDR_WDT_INT=0x%08x, RC%u: IRNEN=0x%08x IRNCR=0x%08x IRNICR=0x%08x\n",
				ipc->isr_spurious, status, vec, wdt,
				ipc->rc_port, irnen, irncr, irnicr);
		}

		wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, WAVE600_IRQ_ALL);
		mmiowb();
		return IRQ_NONE;
	}

	/* Defer all Indication FIFO processing to the tasklet */
	tasklet_schedule(&ipc->tasklet);

	return IRQ_HANDLED;
}

int wave600_ipc_cmd(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
		    const void *req_payload, size_t req_len,
		    void *cfm_payload, size_t cfm_len, unsigned long timeout_ms)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	struct wave600_ipc_cmd *cmd = NULL;
	struct wave600_umi_msg_hdr hdr;
	void __iomem *slot_addr;
	u32 slot, descr;
	unsigned long flags;
	unsigned long t;
	int ret = 0;

	if (!ipc->req_num || !ipc->man_req_num) {
		dev_err(wdev->dev, "IPC not initialized\n");
		return -ENODEV;
	}

	/* Find a free command slot */
	spin_lock_irqsave(&ipc->lock, flags);
	for (slot = 0; slot < min(ipc->man_req_num, (u32)WAVE600_IPC_MAX_MAN_REQS); slot++) {
		if (!ipc->cmds[slot].in_use) {
			cmd = &ipc->cmds[slot];
			cmd->in_use = true;
			break;
		}
	}
	if (!cmd) {
		spin_unlock_irqrestore(&ipc->lock, flags);
		dev_err(wdev->dev, "no free IPC request slots (max %u)\n", WAVE600_IPC_MAX_MAN_REQS);
		return -EBUSY;
	}

	init_completion(&cmd->done);
	cmd->req_id = req_id;
	cmd->cfm_id = cfm_id;
	cmd->resp_buf = cfm_payload;
	cmd->resp_len = cfm_len;
	cmd->status = -ETIMEDOUT;

	/* Prepare message header */
	memset(&hdr, 0, sizeof(hdr));
	hdr.msg_id = req_id;
	hdr.msg_ref = slot;

	/* Write header and payload into SHRAM Management Request slot */
	slot_addr = wdev->bar0 + ipc->man_req_offset + (slot * ipc->man_req_slot_size);
	/* Vendor only writes first 12 bytes of header; bytes 12-15 must not be overwritten */
	memcpy_toio(slot_addr, &hdr, 12);
	if (req_payload && req_len > 0)
		memcpy_toio(slot_addr + sizeof(hdr), req_payload, req_len);
	mmiowb();

	/* Format 32-bit descriptor for Request FIFO */
	descr = wave600_msg_to_descr(WAVE600_MSG_TYPE_MAN_REQ, slot, wdev->radio_idx, 0);

	/* Write descriptor into Request FIFO and advance index */
	wave600_write32(wdev, ipc->req_offset + (ipc->req_idx * sizeof(u32)), descr);
	ipc->req_idx++;
	if (ipc->req_idx >= ipc->req_num)
		ipc->req_idx = 0;
	mmiowb();

	/* Ring doorbell to notify ARC coprocessor */
	wave600_write32(wdev, WAVE600_MAC_GEN_NPU2UPI_SET, 0x1);
	mmiowb();

	spin_unlock_irqrestore(&ipc->lock, flags);

	/*
	 * Native interrupt-driven wait: wave600_ipc_isr processes the Indication FIFO
	 * and signals cmd->done via complete() when firmware completes the command.
	 */
	t = wait_for_completion_timeout(&cmd->done, msecs_to_jiffies(timeout_ms));
	if (t == 0) {
		u32 irq_stat = wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_STATUS);
		u32 irq_vec = wave600_read32(wdev, WAVE600_SOC_PCIE_INTR_VEC);
		u32 wdt_stat = wave600_read32(wdev, WAVE600_SOC_PCIE_AMBA_ORDR_WDT);
		u32 irnen = 0, irncr = 0, irnicr = 0;

		if (ipc->rc_app) {
			irnen = __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNEN);
			irncr = __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNCR);
			irnicr = __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNICR);
		}

		dev_err(wdev->dev, "IPC cmd 0x%04x timed out after %lu ms (slot %u, IRQ_STATUS=0x%08x, INTR_VEC=0x%08x, AMBA_ORDR_WDT_INT=0x%08x, RC%u: IRNEN=0x%08x IRNCR=0x%08x IRNICR=0x%08x)\n",
			req_id, timeout_ms, slot, irq_stat, irq_vec, wdt_stat,
			ipc->rc_port, irnen, irncr, irnicr);
		ret = -ETIMEDOUT;
	} else {
		ret = cmd->status;
	}

	spin_lock_irqsave(&ipc->lock, flags);
	cmd->in_use = false;
	spin_unlock_irqrestore(&ipc->lock, flags);

	return ret;
}

int wave600_ipc_init(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	struct wave600_chi_area chi;
	int ret, i;

	memset(ipc, 0, sizeof(*ipc));
	ipc->wdev = wdev;
	spin_lock_init(&ipc->lock);

	for (i = 0; i < WAVE600_IPC_MAX_MAN_REQS; i++)
		init_completion(&ipc->cmds[i].done);

	/* Deferred interrupt processing (vendor tasklet-based deferred handler) */
	tasklet_setup(&ipc->tasklet, wave600_ipc_tasklet_fn);

	/* Read Shared RAM CHI area updated by ARC firmware */
	memcpy_fromio(&chi, wdev->bar0 + WAVE600_SHARED_RAM_BASE, sizeof(chi));

	if (chi.basic.magic != WAVE600_CHI_MAGIC && chi.basic.magic != swab32(WAVE600_CHI_MAGIC)) {
		dev_err(wdev->dev, "invalid CHI magic in SHRAM (read: 0x%08x, expected 0x%08x)\n",
			chi.basic.magic, WAVE600_CHI_MAGIC);
		return -EINVAL;
	}

	dev_info(wdev->dev, "Shared RAM CHI magic verified: 0x%08x\n", chi.basic.magic);

	/* Extract FIFO queue geometry (in Shared RAM) */
	ipc->req_offset = WAVE600_SHARED_RAM_BASE + chi.basic.fifo_q.req_start_offset;
	ipc->req_num = chi.basic.fifo_q.req_num_elements;
	ipc->ind_offset = WAVE600_SHARED_RAM_BASE + chi.basic.fifo_q.ind_start_offset;
	ipc->ind_num = chi.basic.fifo_q.ind_num_elements;

	/* Extract Management area geometry (in Shared RAM) */
	ipc->man_req_offset = WAVE600_SHARED_RAM_BASE + chi.basic.man.req_start_offset;
	ipc->man_req_num = chi.basic.man.req_num_elements;
	ipc->man_ind_offset = WAVE600_SHARED_RAM_BASE + chi.basic.man.ind_start_offset;
	ipc->man_ind_num = chi.basic.man.ind_num_elements;

	/* Calculate slot sizes in SHRAM */
	if (ipc->man_req_num > 0 && ipc->man_ind_offset > ipc->man_req_offset)
		ipc->man_req_slot_size = (ipc->man_ind_offset - ipc->man_req_offset) / ipc->man_req_num;
	else
		ipc->man_req_slot_size = 256;

	/* Debug area start provides the boundary for man_ind if present */
	if (ipc->man_ind_num > 0 &&
	    (WAVE600_SHARED_RAM_BASE + chi.basic.dbg.req_start_offset) > ipc->man_ind_offset)
		ipc->man_ind_slot_size = ((WAVE600_SHARED_RAM_BASE + chi.basic.dbg.req_start_offset) -
					  ipc->man_ind_offset) / ipc->man_ind_num;
	else
		ipc->man_ind_slot_size = 256;

	dev_info(wdev->dev, "CHI FIFO: Req=0x%08x (%u elements), Ind=0x%08x (%u elements)\n",
		 ipc->req_offset, ipc->req_num, ipc->ind_offset, ipc->ind_num);
	dev_info(wdev->dev, "CHI MAN: Req=0x%08x (%u elements, stride %u B), Ind=0x%08x (%u elements, stride %u B)\n",
		 ipc->man_req_offset, ipc->man_req_num, ipc->man_req_slot_size,
		 ipc->man_ind_offset, ipc->man_ind_num, ipc->man_ind_slot_size);

	/*
	 * INTx storm diagnostics: map the GRX500 Root Complex app register
	 * block of the root port serving this endpoint (bus 1 -> RC port 0,
	 * bus 3 -> RC port 1, per grx500.dtsi ssx@18000000 "app" regions) and
	 * dump the RC interrupt node state BEFORE any endpoint interrupt
	 * register is touched. If the INTA latch (PCIE_IRNCR) or live node
	 * status (PCIE_IRNICR) is already set here, the line was asserted
	 * before this driver enabled anything in the endpoint.
	 */
	{
		phys_addr_t app_base = 0;

		switch (wdev->pdev->bus->number) {
		case 1:
			ipc->rc_port = 0;
			app_base = WAVE600_RC_APP_PORT0_BASE;
			break;
		case 3:
			ipc->rc_port = 1;
			app_base = WAVE600_RC_APP_PORT1_BASE;
			break;
		default:
			dev_info(wdev->dev, "PCI bus %u: RC app mapping skipped\n",
				 wdev->pdev->bus->number);
			break;
		}
		if (app_base) {
			ipc->rc_app = ioremap(app_base, 0x1000);
			if (ipc->rc_app) {
				dev_info(wdev->dev, "RC port %u app regs at %pa: IRNEN=0x%08x, IRNCR=0x%08x, IRNICR=0x%08x\n",
					 ipc->rc_port, &app_base,
					 __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNEN),
					 __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNCR),
					 __raw_readl(ipc->rc_app + WAVE600_RC_APP_IRNICR));
			} else {
				dev_warn(wdev->dev, "failed to map RC port %u app regs\n",
					 ipc->rc_port);
			}
		}
	}

	/* Ensure host interrupts are masked in MAC and hardware legacy INT generation disabled */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	wave600_write32(wdev, WAVE600_SOC_PCIE_MSI_ENABLE, 0);
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0);
	/* Clear any pending interrupts in MAC (0x1 = MTLK_GEN6_CLEAR_RAB_INTERRUPT) */
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	mmiowb();

	/* Register interrupt handler with interrupts safely disabled in the endpoint */
	ret = devm_request_irq(wdev->dev, wdev->pdev->irq, wave600_ipc_isr,
			       IRQF_SHARED, DRV_NAME, wdev);
	if (ret) {
		dev_err(wdev->dev, "failed to request IRQ %d: %d\n", wdev->pdev->irq, ret);
		return ret;
	}

	/*
	 * Vendor gic_unmask_irq() runs pcie_intx_war() when the irq is
	 * unmasked: clear any stale RC INTx latch from before request_irq.
	 */
	if (ipc->rc_app)
		__raw_writel(WAVE600_RC_INTA, ipc->rc_app + WAVE600_RC_APP_IRNCR);

	/*
	 * INTx chain diagnostics + vendor-substantiated fixups.
	 *
	 * The endpoint-side interrupt sources and masks match the vendor
	 * legacy mode exactly, so the missing link is somewhere in the
	 * endpoint Assert_INTa -> RC latch -> GIC level chain. Capture:
	 *  - the trigger type the kernel actually derived from DT,
	 *  - the endpoint's interrupt pin/line and MSI capability state,
	 *  - the root port's PCI_COMMAND INTx state.
	 */
	{
		struct pci_dev *root = wdev->pdev->bus->self;
		u16 msi_ctl = 0, root_cmd = 0;
		int msi_pos = pci_find_capability(wdev->pdev, PCI_CAP_ID_MSI);
		u8 pin = 0, line = 0;

		pci_read_config_byte(wdev->pdev, PCI_INTERRUPT_PIN, &pin);
		pci_read_config_byte(wdev->pdev, PCI_INTERRUPT_LINE, &line);
		if (msi_pos > 0)
			pci_read_config_word(wdev->pdev, msi_pos + 2, &msi_ctl);

		dev_info(wdev->dev, "PCI cfg: IRQ %d trigger=0x%x, INT_PIN=%u INT_LINE=%u, MSI cap@0x%02x MsgCtl=0x%04x\n",
			 wdev->pdev->irq, irq_get_trigger_type(wdev->pdev->irq),
			 pin, line, msi_pos, msi_ctl);

		/*
		 * Vendor _mtlk_pcieg6_set_msi_intr_mode() legacy path clears the
		 * MSI Enable bit in the endpoint's MSI capability: a function
		 * with MSI enabled never sends Assert_INTx messages.
		 */
		if (msi_pos > 0 && (msi_ctl & PCI_MSI_FLAGS_ENABLE)) {
			pci_write_config_word(wdev->pdev, msi_pos + 2,
					      msi_ctl & ~PCI_MSI_FLAGS_ENABLE);
			pci_read_config_word(wdev->pdev, msi_pos + 2, &msi_ctl);
			dev_info(wdev->dev, "cleared endpoint MSI enable (MsgCtl now 0x%04x)\n",
				 msi_ctl);
		}

		if (root) {
			pci_read_config_word(root, PCI_COMMAND, &root_cmd);
			dev_info(wdev->dev, "root port %04x:%02x:%02x.%u PCI_COMMAND=0x%04x (INTx %s)\n",
				 root->vendor, root->bus->number,
				 PCI_SLOT(root->devfn), PCI_FUNC(root->devfn),
				 root_cmd,
				 (root_cmd & PCI_COMMAND_INTX_DISABLE) ? "DISABLED" : "ENABLED");

			/* Root port must forward decoded INTx to the host line */
			if (root_cmd & PCI_COMMAND_INTX_DISABLE) {
				pci_intx(root, 1);
				pci_read_config_word(root, PCI_COMMAND, &root_cmd);
				dev_info(wdev->dev, "re-enabled root port INTx (PCI_COMMAND now 0x%04x)\n",
					 root_cmd);
			}
		}
	}

	/* Now that handler is registered, enable PCIe endpoint legacy INT generation */
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0x1);
	/* Clear again before unmasking */
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	mmiowb();

	/* Unmask host interrupts in MAC (vendor single-IRQ legacy mask MTLK_IRQ_ALL) */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, WAVE600_IRQ_ALL);
	mmiowb();
	(void)wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK);
	ipc->irq_enabled = true;

	dev_info(wdev->dev, "Mailbox IPC initialized on IRQ %d (host IRQ mask 0x%08x)\n",
		 wdev->pdev->irq, (u32)WAVE600_IRQ_ALL);

	/* Read back interrupt registers for verification */
	dev_info(wdev->dev, "IRQ config: IRQ %d, MSI_ENA=0x%08x, LEGACY_ENA=0x%08x, HOST_IRQ_MASK=0x%08x, INTR_VEC=0x%08x, AMBA_ORDR_WDT_INT=0x%08x\n",
		 wdev->pdev->irq,
		 wave600_read32(wdev, WAVE600_SOC_PCIE_MSI_ENABLE),
		 wave600_read32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA),
		 wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK),
		 wave600_read32(wdev, WAVE600_SOC_PCIE_INTR_VEC),
		 wave600_read32(wdev, WAVE600_SOC_PCIE_AMBA_ORDR_WDT));

	/*
	 * Verification Command: Software Reset MAC (UM_MAN_SW_RESET_MAC_REQ)
	 * Exercises the bidirectional Mailbox IPC (TX Request -> FW -> RX Confirmation).
	 */
	dev_info(wdev->dev, "sending UM_MAN_SW_RESET_MAC_REQ (0x%04x)...\n",
		 WAVE600_UM_MAN_SW_RESET_MAC_REQ);
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_SW_RESET_MAC_REQ,
			      WAVE600_MC_MAN_SW_RESET_MAC_CFM,
			      NULL, 0, NULL, 0, 2000);
	if (ret) {
		dev_err(wdev->dev, "SW_RESET_MAC failed: %d\n", ret);
		return ret;
	}

	dev_info(wdev->dev, "SW_RESET_MAC confirmed by firmware\n");
	dev_info(wdev->dev, "ARC FullMAC firmware version: '%s' (Milestone 3 PASS)\n",
		 wdev->fw_version[0] ? wdev->fw_version : "unknown");

	return 0;
}

void wave600_ipc_cleanup(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	unsigned long flags;
	int i;

	/* Stop accepting new interrupts before killing the tasklet */
	ipc->irq_enabled = false;

	/* Mask host interrupts and disable hardware interrupt generation */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0);
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0xffffff);
	mmiowb();

	/* Drain any scheduled tasklet (it will not re-arm interrupts) */
	tasklet_kill(&ipc->tasklet);

	if (ipc->rc_app) {
		iounmap(ipc->rc_app);
		ipc->rc_app = NULL;
	}

	spin_lock_irqsave(&ipc->lock, flags);
	for (i = 0; i < WAVE600_IPC_MAX_MAN_REQS; i++) {
		if (ipc->cmds[i].in_use) {
			ipc->cmds[i].status = -ENODEV;
			complete(&ipc->cmds[i].done);
			ipc->cmds[i].in_use = false;
		}
	}
	spin_unlock_irqrestore(&ipc->lock, flags);
}
