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
#include "wave600_dma.h"

/*
 * DBG CFM fallback poll (verified live 2026-10-31):
 *
 * The firmware PROCESSES UM_DBG_LOGGER_SET_SEVERITY_REQ and writes its
 * CFM (e.g. 0x1434) into DBG request slot 0, but the delivery descriptor
 * posted to the Indication FIFO decodes as type=MAN_REQ/idx=5, so the
 * MSG_TYPE_DBG_REQ branch in the FIFO consumer never fires and the
 * pending DBG command times out. Whenever an unexpected MAN CFM arrives
 * while a DBG command is pending (and before declaring a DBG timeout),
 * re-read DBG request slot 0 directly: if it holds a CFM whose message
 * number matches the expected cfm_id, complete the pending DBG command
 * from the slot content instead of waiting for a descriptor that the
 * firmware never posts correctly.
 *
 * Uses the same unlocked convention as the FIFO consumer itself
 * (completions after the sender cleared in_use are harmless no-ops).
 */
static bool wave600_ipc_dbg_cfm_poll(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	void __iomem *slot_addr;
	struct wave600_umi_msg_hdr hdr;

	if (!ipc->dbg_cmd.in_use || !ipc->dbg_req_offset)
		return false;

	slot_addr = wdev->bar0 + ipc->dbg_req_offset;
	wave600_pas_get(&hdr, slot_addr, sizeof(hdr));

	/*
	 * u16MsgId is MAC-endian (LE) on the wire (vendor MAC_TO_HOST16).
	 * Before the firmware processes the DBG request, the slot holds our
	 * own req_id (0x06xx); once the firmware writes the confirmation,
	 * the slot header changes to 0x04xx / 0x16xx with the same low byte.
	 * Require both that msg_id changed from req_id and that its message
	 * number matches cfm_id.
	 */
	if (le16_to_cpu(hdr.msg_id) == ipc->dbg_cmd.req_id ||
	    (le16_to_cpu(hdr.msg_id) & 0xff) != (ipc->dbg_cmd.cfm_id & 0xff))
		return false;

	dev_info(wdev->dev,
		 "dbg cfm fallback: DBG slot 0 holds cfm 0x%04x (expected 0x%04x), completing pending DBG command\n",
		 le16_to_cpu(hdr.msg_id), ipc->dbg_cmd.cfm_id);

	if (ipc->dbg_cmd.resp_buf && ipc->dbg_cmd.resp_len > 0) {
		size_t copy_len = min(ipc->dbg_cmd.resp_len,
				      (size_t)WAVE600_IPC_MAX_IND_BOUNCE);

		wave600_pas_get(ipc->dbg_cmd.resp_buf, slot_addr + sizeof(hdr),
				copy_len);
	}

	ipc->dbg_cfm_count++;
	ipc->dbg_cmd.status = 0;
	complete(&ipc->dbg_cmd.done);
	return true;
}

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

				wave600_pas_get(&hdr, slot_addr, sizeof(hdr));

				/* Every UMI CFM carries a status field in its
				 * payload (u16Status / u8result / u8Status
				 * depending on the message). Keep a shadow of
				 * the first payload words visible in dmesg so
				 * firmware rejections cannot stay silent. */
				{
					u32 shadow[2] = { 0, 0 };
					size_t shadow_len = min(sizeof(shadow),
								(size_t)(ipc->man_req_slot_size -
									 sizeof(hdr)));

					wave600_pas_get(shadow, slot_addr + sizeof(hdr),
							shadow_len);
					/*
					 * Response payload fields are
					 * MAC-endian (LE-encoded, vendor
					 * MAC_TO_HOST32): print both the raw
					 * word and the decoded value.
					 */
					dev_info(wdev->dev, "ipc cfm 0x%04x: word0=0x%08x word1=0x%08x (le: 0x%08x 0x%08x)\n",
						 le16_to_cpu(hdr.msg_id), shadow[0], shadow[1],
						 le32_to_cpu(shadow[0]),
						 le32_to_cpu(shadow[1]));

				/*
				 * Raw evidence dump: the first 64 B of the
				 * slot exactly as the firmware left them
				 * (UMI header + payload). Resolves the
				 * mirror-echo-vs-real-content question and
				 * any 2-byte layout shift between our
				 * payload structures and the firmware's.
				 */
				{
					u8 raw[64];
					size_t raw_len = min(sizeof(raw),
							     (size_t)ipc->man_req_slot_size);

					wave600_pas_get(raw, slot_addr, raw_len);
					print_hex_dump(KERN_INFO, "cfm-hex ",
						       DUMP_PREFIX_OFFSET, 16, 4,
						       raw, raw_len, false);
				}
				}

				/* The gen6 firmware writes its own
				 * direction/type bits into the slot header
				 * (0x01xx for confirmations); the vendor
				 * driver never reads the slot header at
				 * all and matches by slot index. Compare
				 * only the message number. The u16MsgId
				 * is MAC-endian (LE) on the wire. */
				if ((le16_to_cpu(hdr.msg_id) & 0xff) != (cmd->cfm_id & 0xff))
					dev_warn(wdev->dev,
						 "ipc cfm id mismatch: expected 0x%04x got 0x%04x\n",
						 cmd->cfm_id, le16_to_cpu(hdr.msg_id));

				if (cmd->resp_buf && cmd->resp_len > 0) {
					size_t copy_len = min(cmd->resp_len,
							      (size_t)(ipc->man_req_slot_size - sizeof(hdr)));
					wave600_pas_get(cmd->resp_buf, slot_addr + sizeof(hdr), copy_len);
				}

				ipc->cfm_count++;
				cmd->status = 0;
				complete(&cmd->done);
			} else {
				bool recovered = false;

				/*
				 * Raw descriptor word logged so delivery
				 * anomalies (wrong type/index/endianness)
				 * are diagnosable from the boot log alone
				 * (2026-10-31: type=MAN_REQ/idx=5 arrived
				 * for a pending DBG command).
				 */
				dev_warn(wdev->dev,
					 "unexpected MAN CFM for slot %u (raw descr 0x%08x)\n",
					 idx, descr);

				/*
				 * The gen6 firmware delivers DBG CFMs with
				 * a MAN-type descriptor; recover the pending
				 * DBG command from its slot instead of
				 * dropping the confirmation.
				 */
				if (ipc->dbg_cmd.in_use)
					recovered = wave600_ipc_dbg_cfm_poll(wdev);
				if (recovered)
					dev_info(wdev->dev,
						 "unexpected MAN CFM recovered as pending DBG cfm\n");
			}
		} else if (type == WAVE600_MSG_TYPE_DBG_REQ) {
			/*
			 * Confirmation to a DBG request (vendor ARRAY_DBG_REQ,
			 * handler txdm_on_cfm). Slot index is always 0 on the
			 * gen6 single-element sDBG workaround ring.
			 */
			if (ipc->dbg_cmd.in_use) {
				void __iomem *slot_addr = wdev->bar0 + ipc->dbg_req_offset +
							  (idx * ipc->dbg_req_slot_size);
				struct wave600_umi_msg_hdr hdr;

				wave600_pas_get(&hdr, slot_addr, sizeof(hdr));

				{
					u32 shadow[2] = { 0, 0 };
					size_t shadow_len = min(sizeof(shadow),
								(size_t)WAVE600_IPC_MAX_IND_BOUNCE);

					wave600_pas_get(shadow, slot_addr + sizeof(hdr), shadow_len);
					dev_info(wdev->dev, "ipc dbg cfm 0x%04x: word0=0x%08x word1=0x%08x\n",
						 le16_to_cpu(hdr.msg_id), shadow[0], shadow[1]);
				}

				/* u16MsgId is MAC-endian (LE) on the wire */
				if ((le16_to_cpu(hdr.msg_id) & 0xff) != (ipc->dbg_cmd.cfm_id & 0xff))
					dev_warn(wdev->dev,
						 "ipc dbg cfm id mismatch: expected 0x%04x got 0x%04x\n",
						 ipc->dbg_cmd.cfm_id, le16_to_cpu(hdr.msg_id));

				if (ipc->dbg_cmd.resp_buf && ipc->dbg_cmd.resp_len > 0) {
					size_t copy_len = min(ipc->dbg_cmd.resp_len,
							      (size_t)WAVE600_IPC_MAX_IND_BOUNCE);

					wave600_pas_get(ipc->dbg_cmd.resp_buf,
							slot_addr + sizeof(hdr), copy_len);
				}

				ipc->dbg_cfm_count++;
				ipc->dbg_cmd.status = 0;
				complete(&ipc->dbg_cmd.done);
			} else {
				dev_warn(wdev->dev,
					 "unexpected DBG CFM for slot %u (raw descr 0x%08x)\n",
					 idx, descr);
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
			wave600_pas_get(bounce, ind_slot, bounce_len);
			/* u16MsgId is MAC-endian (LE) on the wire */
			dev_info_ratelimited(wdev->dev, "asynchronous indication 0x%04x on radio %u vap %u (slot %u)\n",
					    le16_to_cpu(ind_hdr->msg_id), radio, vap, idx);

			/*
			 * NETWORK_EVENT (0x3326): AP station connect/disconnect.
			 * The payload layout is not in the GPL headers (vendor
			 * core layer is binary); dump the first payload words as
			 * the diagnostic step of the AP bring-up so the next
			 * iteration can implement the full handler.
			 */
			if (le16_to_cpu(ind_hdr->msg_id) == WAVE600_MC_MAN_NETWORK_EVENT_IND) {
				size_t plen = bounce_len - sizeof(*ind_hdr);

				if (plen > 0)
					print_hex_dump(KERN_INFO, "NET-EVENT ",
						       DUMP_PREFIX_OFFSET, 16, 4,
						       bounce + sizeof(*ind_hdr),
						       min_t(size_t, plen, 64), false);
			} else if (le16_to_cpu(ind_hdr->msg_id) == WAVE600_MC_MAN_MAC_EVENT_IND) {
				size_t plen = bounce_len - sizeof(*ind_hdr);

				if (plen >= sizeof(struct wave600_umi_mac_event)) {
					const struct wave600_umi_mac_event *ev =
						(const void *)(bounce + sizeof(*ind_hdr));

					dev_err(wdev->dev,
						"FW MAC_EVENT_IND (0x3338): event=%u core=%u line/cause=%u status=0x%08x epc=0x%08x file=%u group=%u origin=%u if=%u\n",
						le32_to_cpu(ev->event_id),
						le32_to_cpu(ev->core_num),
						le32_to_cpu(ev->cause_or_line),
						le32_to_cpu(ev->status_reg),
						le32_to_cpu(ev->epc_reg),
						le32_to_cpu(ev->file_id),
						le32_to_cpu(ev->group_id),
						le32_to_cpu(ev->origin_id),
						le32_to_cpu(ev->fw_interface));
				}
			}

			/*
			 * Release the slot back to firmware (vendor _mtlk_mmb_resp_cm_ind):
			 * the IOM contents are written back before the release descriptor
			 * is posted, then the ARRAY_MAN_IND descriptor goes through the
			 * Request FIFO so firmware can reuse this slot.
			 */
			wave600_pas_put(ind_slot, bounce, bounce_len);
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
		} else if (type == WAVE600_MSG_TYPE_DBG_IND) {
			/*
			 * FW logger / debug indication on the sDBG ring
			 * (vendor ARRAY_DBG_IND, handler is_man = FALSE):
			 * read the IOM, dump it as the FW log diagnostic, then
			 * release the slot back through the Request FIFO with
			 * the DBG_IND descriptor type.
			 */
			u32 bounce_len = min(ipc->dbg_ind_slot_size,
					     (u32)WAVE600_IPC_MAX_IND_BOUNCE);
			void __iomem *dbg_slot = wdev->bar0 + ipc->dbg_ind_offset +
						 (idx * ipc->dbg_ind_slot_size);
			u8 bounce[WAVE600_IPC_MAX_IND_BOUNCE];
			struct wave600_umi_msg_hdr *dbg_hdr =
				(struct wave600_umi_msg_hdr *)bounce;
			u32 resp_descr;
			size_t plen, dump_len, i;

			if (!bounce_len)
				bounce_len = sizeof(struct wave600_umi_msg_hdr);

			wave600_pas_get(bounce, dbg_slot, bounce_len);
			ipc->dbg_ind_count++;

			plen = bounce_len > sizeof(*dbg_hdr) ?
			       bounce_len - sizeof(*dbg_hdr) : 0;
			dump_len = min_t(size_t, plen, 64);

			/*
			 * FW log records are mostly printable text (RTLOG
			 * remote format). Show the payload as text when it
			 * decodes as such, otherwise as a hex dump.
			 */
			{
				bool printable = dump_len > 0;

				for (i = 0; i < dump_len; i++) {
					u8 c = bounce[sizeof(*dbg_hdr) + i];

					if (c != '\n' && c != '\r' && c != '\t' &&
					    (c < 0x20 || c > 0x7e)) {
						printable = false;
						break;
					}
				}

				if (printable) {
					char text[65];

					memcpy(text, bounce + sizeof(*dbg_hdr),
					       dump_len);
					text[dump_len] = 0;
					dev_info_ratelimited(wdev->dev,
							     "fw log (dbg ind 0x%04x, radio %u vap %u, slot %u): %s\n",
							     le16_to_cpu(dbg_hdr->msg_id), radio, vap, idx, text);
				} else {
					dev_info_ratelimited(wdev->dev,
							     "fw dbg ind 0x%04x (radio %u vap %u slot %u), %zu B payload:\n",
							     le16_to_cpu(dbg_hdr->msg_id), radio, vap, idx, plen);
					print_hex_dump(KERN_INFO, "fw-dbg ",
						       DUMP_PREFIX_OFFSET, 16, 4,
						       bounce + sizeof(*dbg_hdr),
						       dump_len, false);
				}
			}

			/* Release the slot back to firmware (vendor _mtlk_mmb_resp_cm_ind) */
			wave600_pas_put(dbg_slot, bounce, bounce_len);
			mmiowb();

			resp_descr = wave600_msg_to_descr(WAVE600_MSG_TYPE_DBG_IND,
							  idx, radio, vap);
			spin_lock(&ipc->lock);
			wave600_write32(wdev, ipc->req_offset + (ipc->req_idx * sizeof(u32)),
					resp_descr);
			ipc->req_idx++;
			if (ipc->req_idx >= ipc->req_num)
				ipc->req_idx = 0;
			mmiowb();
			wave600_write32(wdev, WAVE600_MAC_GEN_NPU2UPI_SET, 0x1);
			mmiowb();
			spin_unlock(&ipc->lock);
		} else if (type == WAVE600_MSG_TYPE_DAT_LOGGER_IND) {
			/*
			 * FW logger extension buffer delivered (vendor
			 * _mtlk_mmb_handle_logger_buf_indication WAVE600
			 * branch): dump the log buffer content as the
			 * diagnostic and echo the descriptor back through
			 * the Request FIFO so the firmware can reuse the
			 * buffer.
			 */
			u32 resp_descr;

			ipc->logger_ind_count++;

			if (idx < ipc->fw_log_bufs_num && ipc->fw_log_bufs[idx].virt) {
				u8 *p = ipc->fw_log_bufs[idx].virt;
				u32 len = ipc->fw_log_bufs[idx].size;
				bool printable = true;
				u32 i;

				for (i = 0; i < len; i++) {
					if (p[i] != 0 && p[i] != '\n' && p[i] != '\r' &&
					    p[i] != '\t' && (p[i] < 0x20 || p[i] > 0x7e)) {
						printable = false;
						break;
					}
				}

				if (printable) {
					dev_info_ratelimited(wdev->dev,
							     "fw log buf %u: %.*s\n",
							     idx, (int)len, p);
				} else {
					dev_info_ratelimited(wdev->dev,
							     "fw log buf %u (%u B):\n", idx, len);
					print_hex_dump(KERN_INFO, "fw-log ",
						       DUMP_PREFIX_OFFSET, 16, 4,
						       p, min_t(u32, len, 128), false);
				}
			}

			resp_descr = descr; /* vendor echoes the descriptor as-is */
			spin_lock(&ipc->lock);
			wave600_write32(wdev, ipc->req_offset + (ipc->req_idx * sizeof(u32)),
					resp_descr);
			ipc->req_idx++;
			if (ipc->req_idx >= ipc->req_num)
				ipc->req_idx = 0;
			mmiowb();
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
#define WAVE600_IRQ_RINGS (WAVE600_IRQ_RX | WAVE600_IRQ_TXOUT | \
			   WAVE600_IRQ_BSS_IND | WAVE600_IRQ_BSS_CFM)

static void wave600_ipc_tasklet_fn(struct tasklet_struct *t)
{
	struct wave600_ipc *ipc = from_tasklet(ipc, t, tasklet);
	struct wave600_dev *wdev = ipc->wdev;
	u32 status;
	int processed;

	processed = wave600_ipc_process_fifo_ind(wdev);
	status = ipc->irq_status | wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_STATUS);
	ipc->irq_status = 0;

	if (processed > 0 || (status & WAVE600_IRQ_RINGS)) {
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

	/*
	 * Data and management ring events: hand RX/TXOUT/BSS_IND/BSS_CFM to NAPI.
	 * The mask stays 0 while NAPI runs; wave600_dma_poll re-enables
	 * HOST_IRQ_MASK on completion.
	 */
	if ((status & WAVE600_IRQ_RINGS) && wave600_dma_schedule_napi(wdev))
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
	 * Note: the GRX500 RC INTx latch (PCIE_IRNCR) is serviced by the
	 * pcie-xrx500 INTx irqchip (child EOI = vendor pcie_intx_war), so
	 * this driver consumes interrupts purely through the standard
	 * interrupt-map path and owns no Root Complex registers.
	 */
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
	ipc->irq_status |= status;

	if (!(status & WAVE600_IRQ_ALL)) {
		ipc->isr_spurious++;

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
				"unhandled INTx #%u: HOST_IRQ_STATUS=0x%08x, PCIE_INTR_VEC=0x%08x, AMBA_ORDR_WDT_INT=0x%08x\n",
				ipc->isr_spurious, status, vec, wdt);
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
	/*
	 * u16MsgId is MAC-endian (LE) on the wire: vendor writes it with
	 * HOST_TO_MAC16 = cpu_to_le16 (_txm_send, hw_mmb.c; mtlkinc.h).
	 * A native BE write makes the firmware read a byte-swapped message
	 * number, dispatch to a non-allocated handler and answer with the
	 * generic pseudo-response - the root cause of the 2026-10-31
	 * "everything confirmed but nothing applied" session (GET_MIB
	 * readback returned zeros for a previously SET object).
	 */
	hdr.msg_id = cpu_to_le16(req_id);
	hdr.msg_ref = cpu_to_le32(slot);

	/* Write header and payload into SHRAM Management Request slot */
	slot_addr = wdev->bar0 + ipc->man_req_offset + (slot * ipc->man_req_slot_size);
	/* Vendor only writes first 12 bytes of header; bytes 12-15 must not be overwritten */
	wave600_pas_put(slot_addr, &hdr, 12);
	if (req_payload && req_len > 0)
		wave600_pas_put(slot_addr + sizeof(hdr), req_payload, req_len);
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

		dev_err(wdev->dev, "IPC cmd 0x%04x timed out after %lu ms (slot %u, IRQ_STATUS=0x%08x, INTR_VEC=0x%08x, AMBA_ORDR_WDT_INT=0x%08x)\n",
			req_id, timeout_ms, slot, irq_stat, irq_vec, wdt_stat);
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

	/*
	 * Keep the only valid host copy of the vector area: later ProgModel
	 * downloads stage their chunks at SHRAM offset 0x14+, overwriting
	 * these fields (vendor hw_mmb.c also never re-reads the area).
	 */
	memcpy(&wdev->chi, &chi, sizeof(chi));

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

	/* Extract Debug area geometry (in Shared RAM, vendor sDBG) */
	ipc->dbg_req_offset = WAVE600_SHARED_RAM_BASE + chi.basic.dbg.req_start_offset;
	ipc->dbg_req_num = chi.basic.dbg.req_num_elements;
	ipc->dbg_ind_offset = WAVE600_SHARED_RAM_BASE + chi.basic.dbg.ind_start_offset;
	ipc->dbg_ind_num = chi.basic.dbg.ind_num_elements;
	init_completion(&ipc->dbg_cmd.done);

	/*
	 * Calculate slot sizes in SHRAM (vendor shram_man.h / hw_mmb.c):
	 *  - sMAN.req and sMAN.ind both use SHRAM_MAN_MSG / SHRAM_IND_MSG (620 B).
	 *  - sDBG.ind and sDBG.req precede sMAN.req in SHRAM and use SHRAM_DBG_MSG (528 B).
	 */
	if (ipc->man_req_num > 0 && ipc->man_ind_offset > ipc->man_req_offset)
		ipc->man_req_slot_size = (ipc->man_ind_offset - ipc->man_req_offset) / ipc->man_req_num;
	else
		ipc->man_req_slot_size = 620;

	ipc->man_ind_slot_size = ipc->man_req_slot_size;

	if (ipc->dbg_ind_num > 0 && ipc->dbg_req_offset > ipc->dbg_ind_offset)
		ipc->dbg_ind_slot_size = (ipc->dbg_req_offset - ipc->dbg_ind_offset) /
					 ipc->dbg_ind_num;
	else
		ipc->dbg_ind_slot_size = 528;

	if (ipc->dbg_req_num > 0 && ipc->man_req_offset > ipc->dbg_req_offset)
		ipc->dbg_req_slot_size = (ipc->man_req_offset - ipc->dbg_req_offset) /
					 ipc->dbg_req_num;
	else
		ipc->dbg_req_slot_size = ipc->dbg_ind_slot_size;

	dev_info(wdev->dev, "CHI FIFO: Req=0x%08x (%u elements), Ind=0x%08x (%u elements)\n",
		 ipc->req_offset, ipc->req_num, ipc->ind_offset, ipc->ind_num);
	dev_info(wdev->dev, "CHI MAN: Req=0x%08x (%u elements, stride %u B), Ind=0x%08x (%u elements, stride %u B)\n",
		 ipc->man_req_offset, ipc->man_req_num, ipc->man_req_slot_size,
		 ipc->man_ind_offset, ipc->man_ind_num, ipc->man_ind_slot_size);
	dev_info(wdev->dev, "CHI DBG: Req=0x%08x (%u elements, stride %u B), Ind=0x%08x (%u elements, stride %u B)\n",
		 ipc->dbg_req_offset, ipc->dbg_req_num, ipc->dbg_req_slot_size,
		 ipc->dbg_ind_offset, ipc->dbg_ind_num, ipc->dbg_ind_slot_size);

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
	 * INTx chain diagnostics + vendor-substantiated fixups.
	 *
	 * The GRX500 RC INTx latch (PCIE_IRNCR) is serviced by the
	 * pcie-xrx500 INTx irqchip, so no Root Complex handling is needed
	 * here. Capture the endpoint's interrupt pin/line and MSI capability
	 * state plus the root port's PCI_COMMAND INTx state.
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

		dev_info(wdev->dev, "PCI cfg: IRQ %d, INT_PIN=%u INT_LINE=%u, MSI cap@0x%02x MsgCtl=0x%04x\n",
			 wdev->pdev->irq, pin, line, msi_pos, msi_ctl);

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
	 * NOTE: UM_MAN_SW_RESET_MAC is NOT sent during bring-up. The vendor
	 * finalize step named HW_TXMM is mtlk_txmm_start() - a HOST-side TXMM
	 * manager start, not a firmware message. The vendor sends
	 * SW_RESET_MAC only at teardown / hw recovery; sending it during
	 * probe resets the UMAC application state (see
	 * docs/wave600-boot-mode-root-cause.md, 21:18 section).
	 */

	return 0;
}

/*
 * Blocking command over the DBG ring pair (vendor txdm manager over
 * HW_DBG_REQ_BDR; single gen6 workaround slot). Same contract as
 * wave600_ipc_cmd but the slot and descriptor live on the sDBG areas.
 */
int wave600_ipc_dbg_cmd(struct wave600_dev *wdev, u16 req_id, u16 cfm_id,
			const void *req_payload, size_t req_len,
			void *cfm_payload, size_t cfm_len, unsigned long timeout_ms)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	struct wave600_umi_msg_hdr hdr;
	void __iomem *slot_addr;
	u32 descr;
	unsigned long flags;
	unsigned long t;
	int ret = 0;

	if (!ipc->dbg_req_num) {
		dev_err(wdev->dev, "DBG ring not initialized\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&ipc->lock, flags);
	if (ipc->dbg_cmd.in_use) {
		spin_unlock_irqrestore(&ipc->lock, flags);
		dev_err(wdev->dev, "no free DBG request slot\n");
		return -EBUSY;
	}
	ipc->dbg_cmd.in_use = true;
	init_completion(&ipc->dbg_cmd.done);
	ipc->dbg_cmd.req_id = req_id;
	ipc->dbg_cmd.cfm_id = cfm_id;
	ipc->dbg_cmd.resp_buf = cfm_payload;
	ipc->dbg_cmd.resp_len = cfm_len;
	ipc->dbg_cmd.status = -ETIMEDOUT;

	memset(&hdr, 0, sizeof(hdr));
	/* u16MsgId is MAC-endian (LE) on the wire (vendor HOST_TO_MAC16) */
	hdr.msg_id = cpu_to_le16(req_id);
	hdr.msg_ref = 0;

	slot_addr = wdev->bar0 + ipc->dbg_req_offset;
	/*
	 * Clear the previous confirmation's header first: the DBG CFM
	 * fallback poll matches the slot content by message number, and a
	 * stale CFM from an earlier command with the same message id
	 * (e.g. SET followed by GET severity, both 0x34) would otherwise
	 * complete the new command with old data. The firmware rewrites
	 * the header when it posts the new confirmation.
	 */
	memset_io(slot_addr, 0, sizeof(hdr));
	/* Vendor writes the first 12 header bytes only */
	wave600_pas_put(slot_addr, &hdr, 12);
	if (req_payload && req_len > 0)
		wave600_pas_put(slot_addr + sizeof(hdr), req_payload, req_len);
	mmiowb();

	descr = wave600_msg_to_descr(WAVE600_MSG_TYPE_DBG_REQ, 0, wdev->radio_idx, 0);
	wave600_write32(wdev, ipc->req_offset + (ipc->req_idx * sizeof(u32)), descr);
	ipc->req_idx++;
	if (ipc->req_idx >= ipc->req_num)
		ipc->req_idx = 0;
	mmiowb();

	wave600_write32(wdev, WAVE600_MAC_GEN_NPU2UPI_SET, 0x1);
	mmiowb();

	spin_unlock_irqrestore(&ipc->lock, flags);

	t = wait_for_completion_timeout(&ipc->dbg_cmd.done, msecs_to_jiffies(timeout_ms));
	if (t == 0) {
		u32 irq_stat = wave600_read32(wdev, WAVE600_MAC_GEN_HOST_IRQ_STATUS);
		bool recovered;

		/*
		 * The firmware may have processed the request and written
		 * the CFM into DBG slot 0 while its Indication FIFO
		 * descriptor never reached us (verified live 2026-10-31
		 * for UM_DBG_LOGGER_SET_SEVERITY_REQ). Recover from the
		 * slot before declaring a timeout.
		 */
		recovered = wave600_ipc_dbg_cfm_poll(wdev);
		if (recovered) {
			dev_info(wdev->dev,
				 "DBG cmd 0x%04x recovered from DBG slot after %lu ms timeout\n",
				 req_id, timeout_ms);
			ret = ipc->dbg_cmd.status;
		} else {
			dev_err(wdev->dev, "DBG cmd 0x%04x timed out after %lu ms (IRQ_STATUS=0x%08x)\n",
				req_id, timeout_ms, irq_stat);
			ret = -ETIMEDOUT;
		}
	} else {
		ret = ipc->dbg_cmd.status;
	}

	spin_lock_irqsave(&ipc->lock, flags);
	ipc->dbg_cmd.in_use = false;
	spin_unlock_irqrestore(&ipc->lock, flags);

	return ret;
}

/*
 * Vendor finalize step HW_FW_LOG_SEVERITY
 * (_mtlk_mmb_send_fw_log_severity: LOGGER_SEVERITY_DEFAULT_LEVEL,
 * UMI_CPU_ID_UM) over the DBG manager.
 */
int wave600_ipc_dbg_set_severity(struct wave600_dev *wdev, u32 level, u32 cpu)
{
	struct wave600_umi_logger_set_severity p;
	struct wave600_umi_logger_set_severity cfm;
	u32 probe_cpu;
	int set_ret;
	int ret;

	memset(&p, 0, sizeof(p));
	/* UMI payload scalars are little-endian (vendor HOST_TO_MAC32) */
	p.new_level_le = cpu_to_le32(level);
	p.target_cpu_le = cpu_to_le32(cpu);
	p.get_set_operation = 0;

	dev_info(wdev->dev, "sending UM_DBG_LOGGER_SET_SEVERITY_REQ (0x%04x): level %u, cpu %u...\n",
		 WAVE600_UM_DBG_LOGGER_SET_SEVERITY_REQ, level, cpu);
	set_ret = wave600_ipc_dbg_cmd(wdev, WAVE600_UM_DBG_LOGGER_SET_SEVERITY_REQ,
				      WAVE600_MC_DBG_LOGGER_SET_SEVERITY_CFM,
				      &p, sizeof(p), NULL, 0, 2000);

	/*
	 * Diagnostic readback (2026-10-31): the SET cfm payload observed
	 * live was {0, 1} which is ambiguous (status vs newLevel). Query
	 * the applied severity of each logger agent (get_set_operation=1,
	 * UMI_CPU_ID_LM=0 / UMI_CPU_ID_UM=1, vendor UmiCpuId_e) so the boot
	 * log shows what the firmware really stored.
	 */
	for (probe_cpu = 0; probe_cpu < 2; probe_cpu++) {
		memset(&p, 0, sizeof(p));
		memset(&cfm, 0, sizeof(cfm));
		p.target_cpu_le = cpu_to_le32(probe_cpu);
		p.get_set_operation = 1;

		ret = wave600_ipc_dbg_cmd(wdev,
					  WAVE600_UM_DBG_LOGGER_SET_SEVERITY_REQ,
					  WAVE600_MC_DBG_LOGGER_SET_SEVERITY_CFM,
					  &p, sizeof(p), &cfm, sizeof(cfm), 2000);
		if (ret) {
			dev_info(wdev->dev,
				 "diag: GET severity cpu %u failed: %d\n",
				 probe_cpu, ret);
		} else {
			dev_info(wdev->dev,
				 "diag: GET severity cpu %u: newLevel=%u targetCPU=%u op=%u (raw %*ph)\n",
				 probe_cpu,
				 le32_to_cpu(cfm.new_level_le),
				 le32_to_cpu(cfm.target_cpu_le),
				 cfm.get_set_operation,
				 (int)sizeof(cfm), &cfm);
		}
	}

	return set_ret;
}

/*
 * Vendor pre-activation step HW_FW_LOGGER (_mtlk_mmb_init_fw_logger):
 * allocate the log buffers described by the CHI logger extensions and
 * post their DMA addresses into the SHRAM logger buffer descriptor
 * ring. The firmware then DMAs log records into these buffers and
 * delivers DAT_LOGGER_IND descriptors through the mailbox FIFO.
 */
int wave600_ipc_fw_logger_init(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	const struct wave600_chi_area *chi = &wdev->chi;
	u32 num_descs = chi->logger_ext.num_buf_descs;
	u32 desc_loc = chi->logger_ext.buf_desc_location;
	u32 lm_count = chi->logger_buf_info_ext.lm_buf_info.num_buffers;
	u32 um_count = chi->logger_buf_info_ext.um_buf_info.num_buffers;
	u32 lm_size = chi->logger_buf_info_ext.lm_buf_info.size;
	u32 um_size = chi->logger_buf_info_ext.um_buf_info.size;
	u32 i, bufs_posted = 0;
	int ret = 0;

	if (!num_descs || !desc_loc) {
		dev_info(wdev->dev,
			 "FW logger extension not published by firmware (num_descs=%u, desc_loc=0x%08x) - FW logger buffers not posted\n",
			 num_descs, desc_loc);
		return 0;
	}

	dev_info(wdev->dev,
		 "CHI LoggerExt: desc_loc=0x%08x num_descs=%u, LM bufs=%u x %u B, UM bufs=%u x %u B\n",
		 desc_loc, num_descs, lm_count, lm_size, um_count, um_size);

	if (lm_count + um_count && lm_count + um_count != num_descs) {
		dev_err(wdev->dev,
			"FW logger capabilities mismatch: %u != %u + %u (vendor reports logger broken)\n",
			num_descs, lm_count, um_count);
		return 0;
	}

	ipc->fw_log_bufs = kcalloc(num_descs, sizeof(*ipc->fw_log_bufs), GFP_KERNEL);
	if (!ipc->fw_log_bufs)
		return -ENOMEM;
	ipc->fw_log_bufs_num = num_descs;

	for (i = 0; i < num_descs; i++) {
		u32 size = (i < lm_count) ? lm_size : um_size;

		if (!size)
			size = 120; /* vendor fallback buffer size */

		ipc->fw_log_bufs[i].virt = dma_alloc_coherent(wdev->dev, size,
							     &ipc->fw_log_bufs[i].dma,
							     GFP_KERNEL);
		if (!ipc->fw_log_bufs[i].virt) {
			ret = -ENOMEM;
			break;
		}
		ipc->fw_log_bufs[i].size = size;

		/*
		 * FW logger buffer descriptor (vendor BUFFER_DAT_IND_MSG_DESC):
		 * vendor _mtlk_mmb_set_fw_logger_buffers writes HOST_TO_MAC32
		 * via _mtlk_mmb_pas_put (mtlk_raw_writel), so the two swaps
		 * cancel out to a native 32-bit write via wave600_write32.
		 */
		wave600_write32(wdev, WAVE600_SHARED_RAM_BASE + desc_loc +
				(i * sizeof(u32)),
				(u32)ipc->fw_log_bufs[i].dma);
		bufs_posted++;
	}
	mmiowb();

	if (ret) {
		wave600_ipc_fw_logger_cleanup(wdev);
		return ret;
	}

	dev_info(wdev->dev, "FW logger buffers posted: %u (pre-activation, vendor HW_FW_LOGGER)\n",
		 bufs_posted);
	return 0;
}

void wave600_ipc_fw_logger_cleanup(struct wave600_dev *wdev)
{
	struct wave600_ipc *ipc = &wdev->ipc;
	u32 i;

	for (i = 0; i < ipc->fw_log_bufs_num; i++) {
		if (ipc->fw_log_bufs[i].virt)
			dma_free_coherent(wdev->dev, ipc->fw_log_bufs[i].size,
					  ipc->fw_log_bufs[i].virt, ipc->fw_log_bufs[i].dma);
	}
	kfree(ipc->fw_log_bufs);
	ipc->fw_log_bufs = NULL;
	ipc->fw_log_bufs_num = 0;
}

/*
 * Diagnostic (2026-10-31): dump the firmware's own SHRAM log buffer
 * pools described by the CHI LOGGER_BUF_INFO extension (vendor
 * _mtlk_mmb_parse_fw_logger / __mtlk_mmb_prepare_data_fw_logger,
 * RTLF_REMOTE path). Pool layout: num buffers of (size + 4) bytes, each
 * starting with a USED_NSSBUF (0xABABABAB) marker word and terminated
 * by LOGGER_END_OF_BUFFER_MARK (0xDEADBEEF); marker words are
 * MAC-endian, so the raw BE view of the end mark is 0xEFBEADDE.
 * Non-empty markers here mean the logAgent writes logs into its SHRAM
 * pool even when the host DDR buffers stay untouched.
 */
static void wave600_diag_fw_log_pool(struct wave600_dev *wdev, const char *tag,
				     const char *cpu, u32 bp_addr, u32 num,
				     u32 size)
{
	u32 total, w, words;
	bool found = false;

	if (!bp_addr || !num || !size)
		return;

	total = num * (size + sizeof(u32));
	words = (total + sizeof(u32) - 1) / sizeof(u32);

	for (w = 0; w < words; w++) {
		u32 v = wave600_read32(wdev, WAVE600_SHARED_RAM_BASE +
				       bp_addr + w * sizeof(u32));

		if (v == 0xABABABAB || v == 0xDEADBEEF || v == 0xEFBEADDE) {
			u8 buf[64];
			size_t len = min(sizeof(buf), (size_t)(total - w * 4));

			dev_info(wdev->dev,
				 "fw_state[%s] FW log pool %s: marker 0x%08x at bp+0x%03x (bp=0x%08x, %u B)\n",
				 tag, cpu, v, w * 4, bp_addr, total);
			wave600_pas_get(buf, wdev->bar0 + WAVE600_SHARED_RAM_BASE +
					bp_addr + w * 4, len);
			print_hex_dump(KERN_INFO, "fwshram ", DUMP_PREFIX_OFFSET,
				       16, 4, buf, len, true);
			found = true;
			break;
		}
	}

	if (!found)
		dev_info(wdev->dev,
			 "fw_state[%s] FW log pool %s: no log markers in %u B (bp=0x%08x)\n",
			 tag, cpu, total, bp_addr);
}

/*
 * Read-only FW state snapshot (diagnostic). Needs no firmware
 * cooperation at all; reports:
 *  (a) the boot-time CHI debug extension core info words from the host
 *      snapshot (vendor prints the same LM0/LM1/UM words when the boot
 *      handshake fails, hw_mmb.c),
 *  (b) a live re-read of the CHI magic word and the debug extension core
 *      words (the ProgModel staging area reuses SHRAM offset 0x14
 *      onwards, so live reads there may return staging data),
 *  (c) the ARC MIPS_CTL run status of all three CPUs,
 *  (d) the live SHRAM in/out counters of all four data rings - any
 *      movement between two snapshots is a platform heartbeat,
 *  (e) the raw content of the posted FW logger DMA buffers.
 */
void wave600_ipc_diag_fw_state(struct wave600_dev *wdev, const char *tag)
{
	const struct wave600_chi_area *chi = &wdev->chi;
	struct wave600_ipc *ipc = &wdev->ipc;
	u32 live_magic, live_c0, live_c1, live_c2;
	u32 shram = WAVE600_SHARED_RAM_BASE;
	int i;

	const struct {
		const char *name;
		const struct wave600_chi_ring *ring;
	} rings[] = {
		{ "tx",  &chi->basic.tx_ring },
		{ "rx",  &chi->basic.rx_ring },
		{ "mtx", &chi->basic.mang_tx_ring },
		{ "mrx", &chi->basic.mang_rx_ring },
	};

	dev_info(wdev->dev,
		 "fw_state[%s] CHI debug core info (boot snapshot): LM0=0x%08x LM1=0x%08x UM=0x%08x\n",
		 tag, chi->debug_ext.core0_info, chi->debug_ext.core1_info,
		 chi->debug_ext.core2_info);

	live_magic = wave600_read32(wdev, shram +
				    offsetof(struct wave600_chi_area, basic.magic));
	live_c0 = wave600_read32(wdev, shram +
				offsetof(struct wave600_chi_area, debug_ext.core0_info));
	live_c1 = wave600_read32(wdev, shram +
				offsetof(struct wave600_chi_area, debug_ext.core1_info));
	live_c2 = wave600_read32(wdev, shram +
				offsetof(struct wave600_chi_area, debug_ext.core2_info));
	dev_info(wdev->dev,
		 "fw_state[%s] CHI live re-read: magic=0x%08x core0=0x%08x core1=0x%08x core2=0x%08x (SHRAM >= 0x14 reused by ProgModel staging)\n",
		 tag, live_magic, live_c0, live_c1, live_c2);

	dev_info(wdev->dev,
		 "fw_state[%s] ARC MIPS_CTL: LM0=0x%08x LM1=0x%08x UM=0x%08x (BB_OPMODE=0x%08x)\n",
		 tag,
		 wave600_read32(wdev, WAVE600_SYS_IF_LM0_MIPS_CTL),
		 wave600_read32(wdev, WAVE600_SYS_IF_LM1_MIPS_CTL),
		 wave600_read32(wdev, WAVE600_SYS_IF_UM_MIPS_CTL),
		 wave600_read32(wdev, WAVE600_SYS_IF_BB_OPMODE_STATUS));

	if (chi->mips_control_ext.desc_location) {
		u32 mips_off = shram + chi->mips_control_ext.desc_location;

		dev_info(wdev->dev,
			 "fw_state[%s] CHI MIPS ctrl (desc_loc=0x%08x): core0=0x%08x core1=0x%08x core2=0x%08x\n",
			 tag, chi->mips_control_ext.desc_location,
			 wave600_read32(wdev, mips_off + 0),
			 wave600_read32(wdev, mips_off + 4),
			 wave600_read32(wdev, mips_off + 8));
	}

	for (i = 0; i < ARRAY_SIZE(rings); i++) {
		const struct wave600_chi_ring *r = rings[i].ring;

		if (!r->in_counter_addr)
			continue;
		/*
		 * Ring counter/status words are little-endian (vendor
		 * _mtlk_hw_debug_print_ring_regs decodes with LE32_TO_CPU);
		 * report both the LE-decoded counters and the raw BE view
		 * so a wrong-endian write shows up immediately.
		 *
		 * The CHI counter addresses are FULL BAR0-relative offsets
		 * (HOST_IF_ACC bank at BAR0+0x255000), NOT SHRAM-relative -
		 * the same mapping the runtime doorbell/ack path uses
		 * (wave600_dma.c). Reading them through "shram + addr" hits
		 * an unrelated area (BAR0+0x5550cc) and produced pure
		 * garbage in earlier dumps (verified live 2026-10-31:
		 * real counter at BAR0+0x2550cc read 0).
		 */
		dev_info(wdev->dev,
			 "fw_state[%s] ring %s: in=%u in_st=%u out=%u out_st=%u (le) raw: in=0x%08x in_st=0x%08x out=0x%08x out_st=0x%08x (be freed: in=0x%08x out=0x%08x)\n",
			 tag, rings[i].name,
			 le32_to_cpu(__raw_readl(wdev->bar0 + r->in_counter_addr)),
			 le32_to_cpu(__raw_readl(wdev->bar0 + r->in_status_addr)),
			 le32_to_cpu(__raw_readl(wdev->bar0 + r->out_counter_addr)),
			 le32_to_cpu(__raw_readl(wdev->bar0 + r->out_status_addr)),
			 __raw_readl(wdev->bar0 + r->in_counter_addr),
			 __raw_readl(wdev->bar0 + r->in_status_addr),
			 __raw_readl(wdev->bar0 + r->out_counter_addr),
			 __raw_readl(wdev->bar0 + r->out_status_addr),
			 r->in_counter_addr_be ?
			 __raw_readl(wdev->bar0 + r->in_counter_addr_be) : 0,
			 r->out_freed_counter_addr_be ?
			 __raw_readl(wdev->bar0 + r->out_freed_counter_addr_be) : 0);
	}

	for (i = 0; i < ipc->fw_log_bufs_num; i++) {
		struct wave600_fw_log_buf *b = &ipc->fw_log_bufs[i];

		if (!b->virt)
			continue;
		dev_info(wdev->dev, "fw_state[%s] fw log buf %d (dma %pad, %u B):\n",
			 tag, i, &b->dma, b->size);
		print_hex_dump(KERN_INFO, "fwlog:", DUMP_PREFIX_NONE, 16, 4,
			       b->virt, b->size, true);
	}

	/* FW-side SHRAM log pools (second logger mechanism, vendor RTLF_REMOTE) */
	dev_info(wdev->dev,
		 "fw_state[%s] FW log pools: LM bp=0x%08x (%u x %u B), UM bp=0x%08x (%u x %u B)\n",
		 tag,
		 chi->logger_buf_info_ext.lm_buf_info.buf_pool_addr,
		 chi->logger_buf_info_ext.lm_buf_info.num_buffers,
		 chi->logger_buf_info_ext.lm_buf_info.size,
		 chi->logger_buf_info_ext.um_buf_info.buf_pool_addr,
		 chi->logger_buf_info_ext.um_buf_info.num_buffers,
		 chi->logger_buf_info_ext.um_buf_info.size);
	wave600_diag_fw_log_pool(wdev, tag, "LM",
				 chi->logger_buf_info_ext.lm_buf_info.buf_pool_addr,
				 chi->logger_buf_info_ext.lm_buf_info.num_buffers,
				 chi->logger_buf_info_ext.lm_buf_info.size);
	wave600_diag_fw_log_pool(wdev, tag, "UM",
				 chi->logger_buf_info_ext.um_buf_info.buf_pool_addr,
				 chi->logger_buf_info_ext.um_buf_info.num_buffers,
				 chi->logger_buf_info_ext.um_buf_info.size);
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
