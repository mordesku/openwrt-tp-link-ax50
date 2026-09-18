// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 John Crispin <blogic@openwrt.org>
 * Copyright (C) 2014-2015 Lei Chuanhua <Chuanhua.lei@lantiq.com>
 * Copyright (C) 2026 OpenWrt hybrid port
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/irqchip.h>
#include <asm/bootinfo.h>
#include <asm/time.h>
#include <asm/mips-cps.h>
#include <asm/mips-gic.h>

#include <lantiq_soc.h>

#include "../prom.h"

#define SOC_GRX500	"GRX500"
#define SOC_GRX350	"GRX350"

#define COMP_GRX500	"lantiq,grx500"

#define PART_SHIFT	12
#define PART_MASK	0x0FFFFFFF
#define REV_SHIFT	28
#define REV_MASK	0x70000000


void __init ltq_soc_detect(struct ltq_soc_info *i)
{
	u32 chipid_raw = ltq_r32(LTQ_MPS_CHIPID);

	i->partnum = (chipid_raw & PART_MASK) >> PART_SHIFT;
	i->rev = (chipid_raw & REV_MASK) >> REV_SHIFT;
	sprintf(i->rev_type, "1.%d", i->rev);

	/* Always log the raw register so unknown part numbers can be identified. */
	pr_info("LTQ_MPS_CHIPID raw=0x%08x  partnum=0x%03x  rev=%u\n",
		chipid_raw, i->partnum, i->rev);

	switch (i->partnum) {
	case SOC_ID_GRX562:
	case SOC_ID_GRX582:
	case SOC_ID_GRX583:
	case SOC_ID_GRX580:
	case SOC_ID_GRX584:
	case SOC_ID_GRX585:
		i->name = SOC_GRX500;
		i->type = SOC_TYPE_GRX500;
		i->compatible = COMP_GRX500;
		break;
	case SOC_ID_GRX3506:
	case SOC_ID_GRX3508:
	case SOC_ID_IRX200:
		i->name = SOC_GRX350;
		i->type = SOC_TYPE_GRX500;
		i->compatible = COMP_GRX500;
		break;
	default:
		/* Unknown GRX500-family part. Log it and continue — do not panic.
		 * The partnum value printed above will allow us to add the correct
		 * SOC_ID_* constant in the next build. */
		pr_warn("GRX500 prom: unknown partnum 0x%03x — treating as GRX350\n",
			i->partnum);
		i->name = SOC_GRX350;
		i->type = SOC_TYPE_GRX500;
		i->compatible = COMP_GRX500;
		break;
	}
}

int get_c0_perfcount_int(void)
{
	return gic_get_c0_perfcount_int();
}
EXPORT_SYMBOL_GPL(get_c0_perfcount_int);

void __init arch_init_irq(void)
{
	irqchip_init();
}
