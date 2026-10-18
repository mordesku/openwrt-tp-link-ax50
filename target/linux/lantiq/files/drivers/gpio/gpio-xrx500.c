// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq GRX500 / XRX500 GPIO Controller Driver
 *
 * Copyright (C) 2012 Thomas Langer <thomas.langer@lantiq.com>
 * Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 * Copyright (C) 2014 Kavitha Subramanian <s.kavitha.EE@lantiq.com>
 * Ported to Linux 6.12 LTS for OpenWrt (TP-Link Archer AX50 v1 bring-up)
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <lantiq_soc.h>

#define PINS_PER_PORT		32

/* GRX500 GPIO Register Offsets */
#define GPIO_OUT		0x00000000
#define GPIO_IN			0x00000004
#define GPIO_DIR		0x00000008
#define GPIO_EXINTCR0		0x00000018
#define GPIO_EXINTCR1		0x0000001C
#define GPIO_IRNCR		0x00000020
#define GPIO_IRNICR		0x00000024
#define GPIO_IRNEN		0x00000028
#define GPIO_IRNCFG		0x0000002C
#define GPIO_IRNENSET		0x00000030
#define GPIO_IRNENCLR		0x00000034
#define GPIO_OUTSET		0x00000040
#define GPIO_OUTCLR		0x00000044
#define GPIO_DIRSET		0x00000048
#define GPIO_DIRCLR		0x0000004C

/* GRX500 PADMUXC Register Offsets (0x16c80000 / 0x16c80100) */
#define PADC_MUX(pin)		((pin) * 0x4)
#define PADC_PUEN		0x00000080
#define PADC_PDEN		0x00000084
#define PADC_SRC		0x00000088
#define PADC_DCC0		0x0000008C
#define PADC_DCC1		0x00000090
#define PADC_OD			0x00000094
#define PADC_MUX_GPIO		0x00000000

struct xrx500_gpio_port {
	struct gpio_chip gc;
	void __iomem *port;
	void __iomem *pad;
	raw_spinlock_t lock;
	unsigned int bank;
	char name[16];
};

static inline void port_w32_mask(struct xrx500_gpio_port *p, u32 clear, u32 set, u32 reg)
{
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&p->lock, flags);
	val = ltq_r32(p->port + reg);
	val &= ~clear;
	val |= set;
	ltq_w32(val, p->port + reg);
	wmb();
	raw_spin_unlock_irqrestore(&p->lock, flags);
}

static void xrx500_pad_mux_gpio(struct xrx500_gpio_port *p, unsigned int offset, bool is_output)
{
	unsigned long flags;
	u32 old_mux, old_od, new_od;

	if (!p->pad)
		return;

	raw_spin_lock_irqsave(&p->lock, flags);
	old_mux = ltq_r32(p->pad + PADC_MUX(offset));
	old_od = ltq_r32(p->pad + PADC_OD);

	/* Switch physical pad multiplexer to function 0 (XRX500_MUX_GPIO) */
	ltq_w32(PADC_MUX_GPIO, p->pad + PADC_MUX(offset));

	if (is_output) {
		/* Clear Open-Drain bit so output drives normal push-pull levels */
		new_od = old_od & ~(1U << offset);
		ltq_w32(new_od, p->pad + PADC_OD);
	} else {
		new_od = old_od;
	}
	wmb();
	raw_spin_unlock_irqrestore(&p->lock, flags);

	if (old_mux != PADC_MUX_GPIO || old_od != new_od) {
		dev_info(p->gc.parent,
			 "gpio%u (bank %u pin %u): pad mux %u->0, OD 0x%08x->0x%08x\n",
			 p->gc.base + offset, p->bank, offset,
			 old_mux, old_od, new_od);
	}
}

static int xrx500_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);

	xrx500_pad_mux_gpio(p, offset, false);
	return 0;
}

static int xrx500_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);

	xrx500_pad_mux_gpio(p, offset, false);
	ltq_w32(1U << offset, p->port + GPIO_DIRCLR);
	wmb();
	return 0;
}

static int xrx500_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);

	if (value)
		ltq_w32(1U << offset, p->port + GPIO_OUTSET);
	else
		ltq_w32(1U << offset, p->port + GPIO_OUTCLR);

	xrx500_pad_mux_gpio(p, offset, true);
	ltq_w32(1U << offset, p->port + GPIO_DIRSET);
	wmb();
	return 0;
}

static int xrx500_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);
	u32 dir = ltq_r32(p->port + GPIO_DIR);

	if (dir & (1U << offset))
		return !!(ltq_r32(p->port + GPIO_OUT) & (1U << offset));
	else
		return !!(ltq_r32(p->port + GPIO_IN) & (1U << offset));
}

static void xrx500_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);

	if (value)
		ltq_w32(1U << offset, p->port + GPIO_OUTSET);
	else
		ltq_w32(1U << offset, p->port + GPIO_OUTCLR);
	wmb();
}

static int xrx500_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);

	if (ltq_r32(p->port + GPIO_DIR) & (1U << offset))
		return GPIO_LINE_DIRECTION_OUT;
	return GPIO_LINE_DIRECTION_IN;
}

static void xrx500_gpio_disable_irq(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);
	unsigned int offset = irqd_to_hwirq(d);

	ltq_w32(1U << offset, p->port + GPIO_IRNENCLR);
}

static void xrx500_gpio_enable_irq(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);
	unsigned int offset = irqd_to_hwirq(d);

	ltq_w32(1U << offset, p->port + GPIO_IRNENSET);
}

static void xrx500_gpio_ack_irq(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);
	unsigned int offset = irqd_to_hwirq(d);

	ltq_w32(1U << offset, p->port + GPIO_IRNCR);
}

static void xrx500_gpio_mask_and_ack_irq(struct irq_data *d)
{
	xrx500_gpio_disable_irq(d);
	xrx500_gpio_ack_irq(d);
}

static int xrx500_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);
	unsigned int offset = irqd_to_hwirq(d);
	u32 mask = 1U << offset;

	if ((type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_NONE)
		return 0;

	if ((type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) != 0) {
		/* level triggered */
		port_w32_mask(p, 0, mask, GPIO_IRNCFG);
		irq_set_handler_locked(d, handle_level_irq);
	} else {
		/* edge triggered */
		port_w32_mask(p, mask, 0, GPIO_IRNCFG);
		irq_set_handler_locked(d, handle_edge_irq);
	}

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		port_w32_mask(p, mask, 0, GPIO_EXINTCR0);
		port_w32_mask(p, 0, mask, GPIO_EXINTCR1);
	} else {
		if ((type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_LEVEL_HIGH)) != 0)
			port_w32_mask(p, mask, 0, GPIO_EXINTCR0);
		else
			port_w32_mask(p, 0, mask, GPIO_EXINTCR0);
		port_w32_mask(p, mask, 0, GPIO_EXINTCR1);
	}

	return xrx500_gpio_direction_input(gc, offset);
}

static const struct irq_chip xrx500_gpio_irq_chip = {
	.name = "gpio-xrx500",
	.irq_mask = xrx500_gpio_disable_irq,
	.irq_unmask = xrx500_gpio_enable_irq,
	.irq_ack = xrx500_gpio_ack_irq,
	.irq_mask_ack = xrx500_gpio_mask_and_ack_irq,
	.irq_set_type = xrx500_gpio_irq_type,
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void xrx500_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct xrx500_gpio_port *p = gpiochip_get_data(gc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long irncr;
	int offset;

	chained_irq_enter(chip, desc);

	irncr = ltq_r32(p->port + GPIO_IRNCR);
	/* acknowledge interrupt */
	ltq_w32(irncr, p->port + GPIO_IRNCR);

	for_each_set_bit(offset, &irncr, gc->ngpio)
		generic_handle_domain_irq(gc->irq.domain, offset);

	chained_irq_exit(chip, desc);
}

static int xrx500_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct xrx500_gpio_port *p;
	u32 bank = 0;
	int irq, ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	raw_spin_lock_init(&p->lock);

	p->port = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->port))
		return PTR_ERR(p->port);

	if (platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		p->pad = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(p->pad))
			return PTR_ERR(p->pad);
	}

	of_property_read_u32(np, "lantiq,bank", &bank);
	p->bank = bank;
	snprintf(p->name, sizeof(p->name), "gpio%u", bank);

	p->gc.label = p->name;
	p->gc.request = xrx500_gpio_request;
	p->gc.direction_input = xrx500_gpio_direction_input;
	p->gc.direction_output = xrx500_gpio_direction_output;
	p->gc.get = xrx500_gpio_get;
	p->gc.set = xrx500_gpio_set;
	p->gc.get_direction = xrx500_gpio_get_direction;
	p->gc.base = bank * PINS_PER_PORT;
	p->gc.ngpio = PINS_PER_PORT;
	p->gc.parent = dev;
	p->gc.owner = THIS_MODULE;

	/*
	 * Hardware reset / hygiene: mask and clear all interrupts on this bank.
	 * U-Boot or bootrom may leave pending interrupts or unmasked bits,
	 * causing an IRQ storm when registered.
	 */
	ltq_w32(0xffffffff, p->port + GPIO_IRNENCLR);
	ltq_w32(0xffffffff, p->port + GPIO_IRNCR);
	if (bank == 1) {
		void __iomem *ifmux = (void __iomem *)0xb6080120;
		u32 old_ifmux = ltq_r32(ifmux);

		ltq_w32_mask(BIT(18), 0, ifmux);
		dev_info(dev, "disabled BBSPI at top-mux (0x16080120: 0x%08x->0x%08x)\n",
			 old_ifmux, ltq_r32(ifmux));
	}
	wmb();

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		struct gpio_irq_chip *girq = &p->gc.irq;

		gpio_irq_chip_set_chip(girq, &xrx500_gpio_irq_chip);
		girq->parent_handler = xrx500_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents), GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
	}

	ret = devm_gpiochip_add_data(dev, &p->gc, p);
	if (ret) {
		dev_err(dev, "failed to register gpiochip: %d\n", ret);
		return ret;
	}

	dev_info(dev, "registered GRX500 GPIO bank %u (%u pins, irq %d)\n",
		 bank, p->gc.ngpio, irq > 0 ? irq : -1);

	return 0;
}

static const struct of_device_id xrx500_gpio_match[] = {
	{ .compatible = "lantiq,gpio-xrx500" },
	{ .compatible = "lantiq,xrx500-gpio" },
	{ .compatible = "lantiq,grx500-gpio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, xrx500_gpio_match);

static struct platform_driver xrx500_gpio_driver = {
	.probe = xrx500_gpio_probe,
	.driver = {
		.name = "gpio-xrx500",
		.of_match_table = xrx500_gpio_match,
	},
};

static int __init xrx500_gpio_init(void)
{
	return platform_driver_register(&xrx500_gpio_driver);
}
postcore_initcall(xrx500_gpio_init);

MODULE_DESCRIPTION("Lantiq GRX500 / XRX500 GPIO Controller Driver");
MODULE_AUTHOR("Thomas Langer, John Crispin, Kavitha Subramanian");
MODULE_LICENSE("GPL v2");
