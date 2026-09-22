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
#include <linux/grx500-memory.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>
#include <asm/time.h>
#include <asm/mips-cps.h>
#include <asm/mips-cm.h>
#include <asm/mips-gic.h>
#include <asm/traps.h>
#include <asm/mipsregs.h>
#include <asm/sections.h>

#include <lantiq_soc.h>

#include "../prom.h"

#define SOC_GRX500	"GRX500"
#define SOC_GRX350	"GRX350"

#define COMP_GRX500	"lantiq,grx500"

/* Keep DMA overlap checks exact after the datapath becomes a module. */
void grx500_kernel_phys_range(phys_addr_t *start, phys_addr_t *end)
{
	*start = __pa_symbol(_text);
	*end = __pa_symbol(_end);
}
EXPORT_SYMBOL_GPL(grx500_kernel_phys_range);

/* Vendor mailbox ABI, checked against the waiting statuses in 165119.log. */
bool grx500_vmb_ready(unsigned int cpu);
int grx500_vmb_launch(unsigned int cpu, unsigned long pc,
		      unsigned long sp, unsigned long gp);

static u32 grx500_ibl_early_hash;

/* Sample the first SPL page only. A changed hash is not by itself proof
 * of code corruption: exact firmware code/data boundaries are unverified.
 */
static void grx500_log_ibl(const char *phase, bool early)
{
	u32 hash = 2166136261U;
	unsigned int word;

	for (word = 0; word < 1024; word++) {
		hash ^= __raw_readl((void __iomem *)(0xa0001000UL + word * 4));
		hash *= 16777619U;
	}
	if (early)
		grx500_ibl_early_hash = hash;
	pr_info("GRX500 IBL %s: sample=a0001000 bytes=4096 hash=%08x early_hash=%08x\n",
		phase, hash, grx500_ibl_early_hash);
}

static void grx500_log_start_ipi(const char *phase, unsigned int intr)
{
	/* CM2's 32-bit register layout, verified in vendor asm/gic.h.
	 * Early SoC detection precedes initialization of mips_gic_base.
	 */
	void __iomem *base = mips_gic_base ? mips_gic_base :
		(void __iomem *)0xb2320000UL;
	unsigned int offset = (intr / 32) * 4, shift = intr % 32;

	pr_info("GRX500 IPI%u %s: enabled=%u pending=%u edge=%u polarity=%u pin=%08x vp=%08x\n",
		intr, phase, (__raw_readl(base + 0x400 + offset) >> shift) & 1,
		(__raw_readl(base + 0x480 + offset) >> shift) & 1,
		(__raw_readl(base + 0x180 + offset) >> shift) & 1,
		(__raw_readl(base + 0x100 + offset) >> shift) & 1,
		__raw_readl(base + 0x500 + intr * 4),
		__raw_readl(base + 0x2000 + intr * 32));
}

bool grx500_vmb_ready(unsigned int cpu)
{
	return cpu < 4 && __raw_readl((void __iomem *)(0xa0000000UL + cpu * 176)) == 8;
}

int grx500_vmb_launch(unsigned int cpu, unsigned long pc,
		      unsigned long sp, unsigned long gp)
{
	void __iomem *command;
	unsigned int word, intr = cpu == 2 ? 21 : 85;

	/* Waiting core1 VPEs are firmware-owned, even after CPU2 is adopted.
	 * Reply IRQ88/110 stay masked; Linux online completion is decisive.
	 * Never force a halt/reset or rewrite another VPE's CP0 state on failure.
	 */
	if ((cpu != 2 && cpu != 3) || !mips_gic_base || !grx500_vmb_ready(cpu))
		return -ENODEV;
	grx500_log_ibl("launch", false);
	grx500_log_start_ipi("before-enable", intr);
	/* Never deliver this firmware command to the boot CPU or another VP. */
	if (read_gic_map_vp(intr) != BIT(cpu)) {
		pr_err("GRX500 IPI%u: refusing launch; firmware route is not exclusively VP%u\n",
			intr, cpu);
		return -ENODEV;
	}
	command = (void __iomem *)(0xa0000000UL + cpu * 176 + 8);
	for (word = 0; word < 42; word++)
		__raw_writel(0, command + word * 4);
	__raw_writel(CKSEG1ADDR(__pa_symbol((void *)pc)), command + 4);
	__raw_writel(sp, command + 8);
	__raw_writel(gp, command + 12);
	/* a0/eva/mt_group/yield/private/TC fields stay zero, as vendor CMP. */
	wmb();
	__raw_writel(1, command); /* VMB_CPU_START */
	wmb();
	pr_info("GRX500 VMB launch: CPU%u pc=%08x sp=%08lx gp=%08lx IPI=%u; no CPC reset\n",
		cpu, __raw_readl(command + 4), sp, gp, intr);
	/* Vendor initialise_vmb_IRQhdlr explicitly enables outgoing channels.
	 * Publish the complete mailbox before unmasking an existing pending IRQ.
	 */
	write_gic_smask(intr);
	wmb();
	grx500_log_start_ipi("after-enable", intr);
	write_gic_wedge(GIC_WEDGE_RW | intr);
	wmb();
	grx500_log_start_ipi("after-trigger", intr);
	return 0;
}

/* Vendor GRX500 lantiq_soc.h: CPC_BASE_ADDR. Used only if CPC is disabled. */
phys_addr_t mips_cpc_default_phys_base(void)
{
	return 0x12310000;
}

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

/* Hypothesis: all online VPEs must inherit the same vendor EVA map. */
/* Vendor ltq_vmb.h: FW reply (8 bytes), command (168 bytes), four slots.
 * Read uncached native-endian words; this is shared RAM, not LE MMIO.
 * Hypothesis: IBL_IN_WAIT (8) survives Linux startup when its page and
 * firmware IPI configuration are preserved. Never write mailbox commands.
 */
static void grx500_log_vmb(const char *phase)
{
	unsigned int cpu;

	for (cpu = 0; cpu < 4; cpu++) {
		void __iomem *slot = (void __iomem *)(0xa0000000UL + cpu * 176);

		pr_info("GRX500 VMB %s: CPU%u slot=%px status=%08x priv=%08x command=%08x pc=%08x\n",
			phase, cpu, slot, __raw_readl(slot), __raw_readl(slot + 4),
			__raw_readl(slot + 8), __raw_readl(slot + 12));
	}
}

static void grx500_log_cpu_eva(void *unused)
{
	pr_info("GRX500 EVA CPU%u: Config0=%08x Config5=%08x SegCtl0=%08x SegCtl1=%08x SegCtl2=%08x\n",
		smp_processor_id(), read_c0_config(), read_c0_config5(),
		read_c0_segctl0(), read_c0_segctl1(), read_c0_segctl2());
}

static int __init grx500_check_online_eva(void)
{
	on_each_cpu(grx500_log_cpu_eva, NULL, 1);
	grx500_log_vmb("late");
	grx500_log_ibl("late", false);
	grx500_log_start_ipi("late", 21);
	grx500_log_start_ipi("late", 85);
	return 0;
}
late_initcall(grx500_check_online_eva);

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
	grx500_log_ibl("early", true);
	grx500_log_start_ipi("early", 21);
	grx500_log_start_ipi("early", 85);
	grx500_log_vmb("early");

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
