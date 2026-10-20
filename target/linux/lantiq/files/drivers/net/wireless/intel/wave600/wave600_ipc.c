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

static void wave600_ipc_process_fifo_ind(struct wave600_dev *wdev)
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

		/* Clear interrupt unconditionally as vendor does */
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
			void __iomem *ind_slot = wdev->bar0 + ipc->man_ind_offset +
						 (idx * ipc->man_ind_slot_size);
			struct wave600_umi_msg_hdr ind_hdr;
			u32 resp_descr;

			memcpy_fromio(&ind_hdr, ind_slot, sizeof(ind_hdr));
			dev_info_ratelimited(wdev->dev, "asynchronous indication 0x%04x on radio %u vap %u (slot %u)\n",
					     ind_hdr.msg_id, radio, vap, idx);

			/*
			 * Recycle indication slot back to firmware (vendor _mtlk_mmb_resp_cm_ind):
			 * Write descriptor into Request FIFO so firmware can reuse this slot.
			 */
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
}

irqreturn_t wave600_ipc_isr(int irq, void *dev_id)
{
	struct wave600_dev *wdev = dev_id;
	u32 status;

	status = wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_STATUS);
	if (!(status & WAVE600_IRQ_ALL))
		return IRQ_NONE;

	/* Clear PHI interrupt */
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	mmiowb();

	/* Process all completed messages in the indication FIFO */
	wave600_ipc_process_fifo_ind(wdev);

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

	/* Wait for confirmation from firmware */
	t = wait_for_completion_timeout(&cmd->done, msecs_to_jiffies(timeout_ms));
	if (t == 0) {
		/* Polling fallback: check if response arrived in FIFO before failing */
		wave600_ipc_process_fifo_ind(wdev);
		if (cmd->status == 0) {
			dev_dbg(wdev->dev, "IPC cmd 0x%04x confirmed via polling fallback\n", req_id);
			ret = 0;
		} else {
			dev_err(wdev->dev, "IPC cmd 0x%04x timed out after %lu ms (slot %u)\n",
				req_id, timeout_ms, slot);
			ret = -ETIMEDOUT;
		}
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
	struct wave600_mac_version ver;
	int ret, i;

	memset(ipc, 0, sizeof(*ipc));
	ipc->wdev = wdev;
	spin_lock_init(&ipc->lock);

	for (i = 0; i < WAVE600_IPC_MAX_MAN_REQS; i++)
		init_completion(&ipc->cmds[i].done);

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

	/* Enable PCIe endpoint legacy interrupt generation in hardware */
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0x1);
	mmiowb();

	/* Clear any pending interrupts */
	wave600_write32(wdev, WAVE600_MAC_GEN_PHI_IRQ_CLEAR, 0x1);
	mmiowb();

	/* Register interrupt handler */
	ret = devm_request_irq(wdev->dev, wdev->pdev->irq, wave600_ipc_isr,
			       IRQF_SHARED, DRV_NAME, wdev);
	if (ret) {
		dev_err(wdev->dev, "failed to request IRQ %d: %d\n", wdev->pdev->irq, ret);
		return ret;
	}

	/* Unmask host interrupts */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, WAVE600_IRQ_ALL);
	mmiowb();
	ipc->irq_enabled = true;

	dev_info(wdev->dev, "Mailbox IPC initialized on IRQ %d (host IRQ mask 0x%08lx)\n",
		 wdev->pdev->irq, (unsigned long)WAVE600_IRQ_ALL);

	/*
	 * Verification Command 1: Software Reset MAC (UM_MAN_SW_RESET_MAC_REQ)
	 */
	dev_info(wdev->dev, "sending UM_MAN_SW_RESET_MAC_REQ (0x%04x)...\n",
		 WAVE600_UM_MAN_SW_RESET_MAC_REQ);
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_SW_RESET_MAC_REQ,
			      WAVE600_MC_MAN_SW_RESET_MAC_CFM,
			      NULL, 0, NULL, 0, 2000);
	if (ret)
		dev_warn(wdev->dev, "SW_RESET_MAC returned %d\n", ret);
	else
		dev_info(wdev->dev, "SW_RESET_MAC confirmed by firmware\n");

	/*
	 * Verification Command 2: Get MAC Version (UM_MAN_GET_MAC_VERSION_REQ)
	 */
	memset(&ver, 0, sizeof(ver));
	dev_info(wdev->dev, "sending UM_MAN_GET_MAC_VERSION_REQ (0x%04x)...\n",
		 WAVE600_UM_MAN_GET_MAC_VERSION_REQ);
	ret = wave600_ipc_cmd(wdev, WAVE600_UM_MAN_GET_MAC_VERSION_REQ,
			      WAVE600_MC_MAN_GET_MAC_VERSION_CFM,
			      NULL, 0, &ver, sizeof(ver), 2000);
	if (ret) {
		dev_err(wdev->dev, "failed to get MAC version: %d\n", ret);
		return ret;
	}

	ver.ver[sizeof(ver.ver) - 1] = '\0';
	dev_info(wdev->dev, "ARC FullMAC firmware version: '%s' (Milestone 3 PASS)\n", ver.ver);

	return 0;
}

void wave600_ipc_cleanup(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	unsigned long flags;
	int i;

	/* Mask host interrupts and disable hardware interrupt generation */
	wave600_write32(wdev, WAVE600_MAC_GEN_HOST_IRQ_MASK, 0);
	wave600_write32(wdev, WAVE600_SOC_PCIE_LEGACY_INT_ENA, 0);
	mmiowb();

	ipc->irq_enabled = false;

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
