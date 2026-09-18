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
#include <linux/of.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>
#include <asm/time.h>
#include <asm/mips-cps.h>
#include <asm/mips-cm.h>
#include <asm/mips-gic.h>
#include <asm/traps.h>
#include <asm/mipsregs.h>

#include <lantiq_soc.h>

#include "../prom.h"

#define SOC_GRX500	"GRX500"
#define SOC_GRX350	"GRX350"

#define COMP_GRX500	"lantiq,grx500"

#define PART_SHIFT	12
#define PART_MASK	0x0FFFFFFF
#define REV_SHIFT	28
#define REV_MASK	0x70000000


static const char * const grx500_cm_causes[32] = {
	"None", "GC_WR_ERR", "GC_RD_ERR", "COH_WR_ERR",
	"COH_RD_ERR", "MMIO_WR_ERR", "MMIO_RD_ERR", "0x07",
	"0x08", "0x09", "0x0a", "0x0b",
	"0x0c", "0x0d", "0x0e", "0x0f",
	"0x10", "0x11", "0x12", "0x13",
	"0x14", "0x15", "0x16", "INTVN_WR_ERR",
	"INTVN_RD_ERR", "0x19", "0x1a", "0x1b",
	"0x1c", "0x1d", "0x1e", "0x1f"
};

static int grx500_be_handler(struct pt_regs *regs, int is_fixup)
{
	int retval = is_fixup ? MIPS_BE_FIXUP : MIPS_BE_FATAL;

	if (mips_cm_present()) {
		unsigned long cm_error = read_gcr_error_cause();
		unsigned long cm_addr = read_gcr_error_addr();
		unsigned long cm_other = read_gcr_error_mult();
		unsigned long cause = (cm_error >> 27) & 0x1f;
		unsigned long cca = (cm_error >> 15) & 0x7;
		unsigned long ocause = cm_other & 0x1f;

		pr_err("==================== MIPS CM2 BUS ERROR ====================\n");
		pr_err("CPU%d epc=%08lx badvaddr=%08lx is_fixup=%d\n",
		       smp_processor_id(), regs->cp0_epc, regs->cp0_badvaddr, is_fixup);
		pr_err("CM_ERROR_CAUSE: 0x%08lx [%s] (CCA=%lu)\n",
		       cm_error, grx500_cm_causes[cause], cca);
		pr_err("CM_ERROR_ADDR : 0x%08lx\n", cm_addr);
		pr_err("CM_ERROR_MULT : 0x%08lx [%s]\n",
		       cm_other, grx500_cm_causes[ocause]);
		pr_err("============================================================\n");

		write_gcr_error_cause(0);
	} else {
		pr_err("=== BUS ERROR: CM not present! CPU%d epc=%08lx badvaddr=%08lx ===\n",
		       smp_processor_id(), regs->cp0_epc, regs->cp0_badvaddr);
	}

	return retval;
}

void __init ltq_soc_detect(struct ltq_soc_info *i)
{
	u32 chipid_raw = ltq_r32(LTQ_MPS_CHIPID);

	mips_set_be_handler(grx500_be_handler);

	/* Verify the assembly EVA setup before any TLB-backed allocations. */
	pr_info("GRX500 EVA: Config0=%08x Config5=%08x\n",
		read_c0_config(), read_c0_config5());
	pr_info("GRX500 EVA: SegCtl0=%08x SegCtl1=%08x SegCtl2=%08x\n",
		read_c0_segctl0(), read_c0_segctl1(), read_c0_segctl2());
	pr_info("GRX500 EVA: PAGE_OFFSET=%08lx PHYS_OFFSET=%08lx\n",
		(unsigned long)PAGE_OFFSET, (unsigned long)PHYS_OFFSET);

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

unsigned int get_c0_compare_int(void)
{
	if (mips_gic_present())
		return gic_get_c0_compare_int();

	return MIPS_CPU_IRQ_BASE + cp0_compare_irq;
}
EXPORT_SYMBOL_GPL(get_c0_compare_int);

int get_c0_perfcount_int(void)
{
	if (mips_gic_present())
		return gic_get_c0_perfcount_int();

	if (cp0_perfcount_irq < 0)
		return -1;

	return MIPS_CPU_IRQ_BASE + cp0_perfcount_irq;
}
EXPORT_SYMBOL_GPL(get_c0_perfcount_int);

void __init arch_init_irq(void)
{
	struct device_node *intc_node;

	intc_node = of_find_compatible_node(NULL, NULL,
					    "mti,cpu-interrupt-controller");
	if (!cpu_has_veic && !intc_node)
		mips_cpu_irq_init();
	of_node_put(intc_node);

	irqchip_init();
}
