// SPDX-License-Identifier: GPL-2.0
/*
 * Intel / MaxLinear Wave600 (WAV654) PCIe Driver Registration & Probe
 */

#include <linux/module.h>
#include <linux/pci.h>
#include "wave600.h"
#include "wave600_regs.h"
#include "wave600_fw.h"

static const struct pci_device_id wave600_pci_ids[] = {
	{ PCI_DEVICE(0x8086, 0x09d0) }, /* Intel WAV654 802.11ax */
	{ PCI_DEVICE(0x1bef, 0x09d0) }, /* Lantiq WAV654 802.11ax */
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, wave600_pci_ids);

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

	/* Read chip system information (bits 0..15: Chip ID, bits 16..31: BootMode & CPU ID) */
	sys_info = wave600_read32(wdev, WAVE600_SYS_IF_SYSTEM_INFO);
	wdev->chip_id = sys_info & 0xffff;
	wdev->chip_rev = (sys_info >> 16) & 0xffff;

	dev_info(&pdev->dev, "BAR0 mapped: %pr -> %p (sys_info: 0x%08x, Chip ID: 0x%04x, Rev: 0x%04x)\n",
		 &pdev->resource[0], wdev->bar0, sys_info, wdev->chip_id, wdev->chip_rev);


	/*
	 * On TP-Link Archer AX50, the secondary PCIe slot (0000:03:00.0) is an AUX
	 * interface or inactive (sys_info = 0x00000000 or 0xffffffff).
	 * In accordance with vendor driver CID-01, skip AUX device in non-dual-PCI mode.
	 */
	if (wdev->chip_id == 0xffff || wdev->chip_id == 0) {
		dev_info(&pdev->dev, "ignoring WAV654 PCI AUX endpoint (non-dual-PCI mode)\n");
		return -ENODEV;
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

	/* Initialize Mailbox Circular IPC Engine (Milestone 3) */
	ret = wave600_ipc_init(wdev);
	if (ret) {
		dev_err(&pdev->dev, "mailbox IPC initialization failed: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "WAV654 Mailbox IPC initialized successfully (Milestone 3 PASS)\n");
	return 0;
}

static void wave600_pci_remove(struct pci_dev *pdev)
{
	struct wave600_dev *wdev = pci_get_drvdata(pdev);

	if (wdev) {
		dev_info(&pdev->dev, "removing WAV654 driver\n");
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
