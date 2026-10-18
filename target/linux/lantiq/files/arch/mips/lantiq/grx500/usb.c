// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq GRX500 USB 3.0 Platform Glue Driver
 *
 * Copyright (C) 2015 Lantiq Semiconductor Co., Ltd.
 * Copyright (C) 2026 OpenWrt.org
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <lantiq_soc.h>

#define LTQ_GRX500_RCU_USB_PHY0		BIT(4)
#define LTQ_GRX500_RCU_USB_PHY1		BIT(5)

#define LTQ_GRX500_CHIP_TOP_AXI_DEV_END	BIT(9)
#define LTQ_GRX500_CHIP_TOP_AXI_HOST_END	BIT(10)

struct grx500_dwc3_glue {
	struct device		*dev;
	struct clk		*clk;
	struct gpio_desc	*vbus;
	int			usb_id;
};

static int grx500_dwc3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct grx500_dwc3_glue *glue;
	struct resource *res;
	const char *clk_name;
	int ret;

	glue = devm_kzalloc(dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	glue->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to get memory resource\n");
		return -EINVAL;
	}

	if (res->start == 0x1a300000)
		glue->usb_id = 0;
	else if (res->start == 0x1a500000)
		glue->usb_id = 1;
	else {
		dev_err(dev, "unknown USB base address: %pa\n", &res->start);
		return -EINVAL;
	}

	/* Get optional VBUS GPIO */
	glue->vbus = devm_gpiod_get_optional(dev, "vbus", GPIOD_OUT_HIGH);
	if (IS_ERR(glue->vbus)) {
		ret = PTR_ERR(glue->vbus);
		dev_err(dev, "failed to request VBUS GPIO: %d\n", ret);
		return ret;
	}

	/* Get and enable controller clock (CGU_USB0 or CGU_USB1) */
	glue->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR_OR_NULL(glue->clk)) {
		clk_name = (glue->usb_id == 0) ? "1a300000.usb" : "1a500000.usb";
		glue->clk = clk_get_sys(clk_name, NULL);
		if (IS_ERR(glue->clk))
			glue->clk = NULL;
	}

	if (glue->clk) {
		ret = clk_prepare_enable(glue->clk);
		if (ret) {
			dev_err(dev, "failed to enable clock: %d\n", ret);
			return ret;
		}
	}
	usleep_range(5000, 10000);

	/* Turn on VBUS power if GPIO was present */
	if (glue->vbus) {
		gpiod_set_value_cansleep(glue->vbus, 1);
		msleep(100);
	}

	/* Set AXI host port endianness to Big-Endian */
	if (glue->usb_id == 0)
		ltq_usb0_port_endian_set(LTQ_GRX500_CHIP_TOP_AXI_DEV_END,
					 LTQ_GRX500_CHIP_TOP_AXI_HOST_END);
	else
		ltq_usb1_port_endian_set(LTQ_GRX500_CHIP_TOP_AXI_DEV_END,
					 LTQ_GRX500_CHIP_TOP_AXI_HOST_END);

	usleep_range(5000, 10000);

	/* Release PHY reset in RCU_RST_REQ2 */
	ltq_usb_phy_reset(glue->usb_id == 0 ? LTQ_GRX500_RCU_USB_PHY0 :
					      LTQ_GRX500_RCU_USB_PHY1);
	msleep(100);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		dev_warn(dev, "failed to set 32-bit DMA mask: %d\n", ret);

	platform_set_drvdata(pdev, glue);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_noresume(dev);

	/* Populate child nodes (snps,dwc3) */
	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to populate child dwc3 core: %d\n", ret);
		pm_runtime_disable(dev);
		goto err_clk;
	}

	dev_info(dev, "GRX500 USB%d glue initialized (VBUS %s)\n",
		 glue->usb_id, glue->vbus ? "enabled" : "unmanaged");
	return 0;

err_clk:
	if (glue->vbus)
		gpiod_set_value_cansleep(glue->vbus, 0);
	if (glue->clk)
		clk_disable_unprepare(glue->clk);
	return ret;
}

static void grx500_dwc3_remove(struct platform_device *pdev)
{
	struct grx500_dwc3_glue *glue = platform_get_drvdata(pdev);

	if (!glue)
		return;

	of_platform_depopulate(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (glue->vbus)
		gpiod_set_value_cansleep(glue->vbus, 0);
	if (glue->clk)
		clk_disable_unprepare(glue->clk);
}

static const struct of_device_id grx500_dwc3_of_match[] = {
	{ .compatible = "lantiq,grx500-dwc3" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, grx500_dwc3_of_match);

static struct platform_driver grx500_dwc3_driver = {
	.probe		= grx500_dwc3_probe,
	.remove		= grx500_dwc3_remove,
	.driver		= {
		.name	= "grx500-dwc3",
		.of_match_table = grx500_dwc3_of_match,
	},
};

static int __init grx500_dwc3_init(void)
{
	return platform_driver_register(&grx500_dwc3_driver);
}
subsys_initcall(grx500_dwc3_init);

static void __exit grx500_dwc3_exit(void)
{
	platform_driver_unregister(&grx500_dwc3_driver);
}
module_exit(grx500_dwc3_exit);

MODULE_AUTHOR("Antigravity / OpenWrt");
MODULE_DESCRIPTION("Lantiq GRX500 USB3 DWC3 Glue Driver");
MODULE_LICENSE("GPL");
