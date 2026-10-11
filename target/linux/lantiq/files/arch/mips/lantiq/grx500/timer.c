// SPDX-License-Identifier: GPL-2.0-only
/*
 * GRX500 system timers, adapted from the AX50 GPL Linux 3.10.104
 * arch/mips/kernel/{csrc,cevt}-gptc.c and lantiq_gptc.h.
 *
 * Hypothesis: the CPU-local R4K clock path stalls under the full OpenWrt
 * workload. Reproduce the vendor's shared source and CPU event allocation.
 * R4K boots the machine; before userspace, CPU hotplug callbacks replace
 * its clockevents on each online CPU with higher-rated GPTC devices.
 * This is built-in and owns both GPTC blocks for the lifetime of the kernel.
 */
#define pr_fmt(fmt) "grx500-gptc: " fmt

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/spinlock.h>

#define GPTC_CPUS	4
#define GPTC_CLC		0x00
#define GPTC_ID		0x08
#define GPTC_IRNEN	0xf4
#define GPTC_IRNCR	0xfc
#define GPTC_CON(n)	(0x10 + ((n) / 2) * 0x20 + ((n) % 2) * 4)
#define GPTC_RUN(n)	(GPTC_CON(n) + 0x08)
#define GPTC_RLD(n)	(GPTC_CON(n) + 0x10)
#define GPTC_CNT(n)	(GPTC_CON(n) + 0x18)
#define GPTC_ENABLED	BIT(0)
#define GPTC_UP		BIT(1)
#define GPTC_ONESHOT	BIT(3)
#define GPTC_32BIT	BIT(4)
#define GPTC_START	BIT(0)
#define GPTC_STOP	BIT(1)
#define GPTC_RELOAD	BIT(2)
#define GPTC_SOURCE	4 /* GPTC0 timer 3A; no interrupt */

struct grx500_gptc_event {
	struct clock_event_device ce;
	void __iomem *base;
	unsigned int channel;
	unsigned int irq;
	char name[24];
};

static void __iomem *gptc_base[2];
static struct grx500_gptc_event gptc_events[GPTC_CPUS];
static DEFINE_RAW_SPINLOCK(gptc_lock);
static unsigned int gptc_rate;

static struct grx500_gptc_event *to_gptc(struct clock_event_device *ce)
{
	return container_of(ce, struct grx500_gptc_event, ce);
}

/* Native-endian MMIO matches vendor __raw_readl/__raw_writel on this BE SoC. */
static void gptc_irq_mask(struct grx500_gptc_event *ev, bool enable)
{
	unsigned long flags;
	u32 value;

	raw_spin_lock_irqsave(&gptc_lock, flags);
	value = __raw_readl(ev->base + GPTC_IRNEN);
	if (enable)
		value |= BIT(ev->channel);
	else
		value &= ~BIT(ev->channel);
	__raw_writel(value, ev->base + GPTC_IRNEN);
	__raw_readl(ev->base + GPTC_IRNEN);
	raw_spin_unlock_irqrestore(&gptc_lock, flags);
}

static int gptc_shutdown(struct clock_event_device *ce)
{
	struct grx500_gptc_event *ev = to_gptc(ce);
	u32 value;

	gptc_irq_mask(ev, false);
	__raw_writel(GPTC_STOP, ev->base + GPTC_RUN(ev->channel));
	__raw_writel(0, ev->base + GPTC_RLD(ev->channel));
	__raw_writel(BIT(ev->channel), ev->base + GPTC_IRNCR);
	return read_poll_timeout_atomic(__raw_readl, value,
			!(value & GPTC_ENABLED), 1, 1000, false,
			ev->base + GPTC_CON(ev->channel));
}

static int gptc_oneshot(struct clock_event_device *ce)
{
	struct grx500_gptc_event *ev = to_gptc(ce);
	int ret = gptc_shutdown(ce);

	if (ret)
		return ret;
	/* Internal clock, down-counting, extended 32-bit one-shot mode. */
	__raw_writel(GPTC_32BIT | GPTC_ONESHOT,
		     ev->base + GPTC_CON(ev->channel));
	gptc_irq_mask(ev, true);
	return 0;
}

static int gptc_next_event(unsigned long delta, struct clock_event_device *ce)
{
	struct grx500_gptc_event *ev = to_gptc(ce);

	__raw_writel(GPTC_STOP, ev->base + GPTC_RUN(ev->channel));
	__raw_writel(delta, ev->base + GPTC_RLD(ev->channel));
	__raw_writel(GPTC_START | GPTC_RELOAD,
		     ev->base + GPTC_RUN(ev->channel));
	__raw_readl(ev->base + GPTC_RUN(ev->channel));
	return 0;
}

static irqreturn_t gptc_interrupt(int irq, void *data)
{
	struct grx500_gptc_event *ev = data;

	/* Keep the vendor mask/ack/dispatch/unmask order, without holding a
	 * shared lock across the callback (which can reprogram this timer).
	 */
	gptc_irq_mask(ev, false);
	__raw_writel(BIT(ev->channel), ev->base + GPTC_IRNCR);
	__raw_readl(ev->base + GPTC_IRNCR);
	ev->ce.event_handler(&ev->ce);
	if (!clockevent_state_shutdown(&ev->ce) &&
	    !clockevent_state_detached(&ev->ce))
		gptc_irq_mask(ev, true);
	return IRQ_HANDLED;
}

static u64 notrace gptc_read_sched_clock(void)
{
	return __raw_readl(gptc_base[0] + GPTC_CNT(GPTC_SOURCE));
}

static u64 gptc_read_clocksource(struct clocksource *cs)
{
	return gptc_read_sched_clock();
}

static struct clocksource gptc_clocksource = {
	.name = "gptc-timer",
	.rating = 400, /* Above the retained R4K bootstrap source (240). */
	.read = gptc_read_clocksource,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int gptc_cpu_online(unsigned int cpu)
{
	struct grx500_gptc_event *ev;
	unsigned long flags;
	int ret;

	if (cpu >= GPTC_CPUS)
		return -EINVAL;
	ev = &gptc_events[cpu];
	ret = irq_set_affinity(ev->irq, cpumask_of(cpu));
	if (ret)
		return ret;
	/* These are distinct shared GIC IRQs with fixed affinity, not a single
	 * per-CPU IRQ. Register the clockevent on its owning online CPU.
	 */
	local_irq_save(flags);
	clockevents_config_and_register(&ev->ce, gptc_rate, 0x300, 0xffffffff);
	local_irq_restore(flags);
	enable_irq(ev->irq);
	pr_info("CPU%u clockevent %s IRQ%u channel%u\n",
		cpu, ev->name, ev->irq, ev->channel);
	return 0;
}

static int gptc_cpu_offline(unsigned int cpu)
{
	struct grx500_gptc_event *ev = &gptc_events[cpu];

	disable_irq(ev->irq);
	return gptc_shutdown(&ev->ce);
}

static int __init grx500_gptc_init(void)
{
	static const unsigned int channels[GPTC_CPUS] = { 0, 0, 2, 4 };
	static const char * const gates[] = { "16b00000.gptu", "16300000.gptu" };
	struct device_node *np;
	struct clk *clk;
	unsigned int i;
	u32 value;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "lantiq,grx500-system-timer");
	if (!np || !of_device_is_available(np)) {
		of_node_put(np);
		return 0;
	}
	if (num_possible_cpus() > GPTC_CPUS ||
	    of_property_read_u32(np, "clock-frequency", &gptc_rate) ||
	    !gptc_rate)
		panic("GRX500 GPTC: invalid CPU topology or frequency");

	for (i = 0; i < 2; i++) {
		gptc_base[i] = of_io_request_and_map(np, i, "grx500-system-timer");
		if (IS_ERR(gptc_base[i]))
			panic("GRX500 GPTC: cannot map block %u", i);
		clk = clk_get_sys(gates[i], NULL);
		if (IS_ERR(clk))
			panic("GRX500 GPTC: block %u clock lookup %s failed: %ld",
			      i, gates[i], PTR_ERR(clk));
		ret = clk_prepare_enable(clk);
		if (ret)
			panic("GRX500 GPTC: block %u clock enable %s failed: %d",
			      i, gates[i], ret);
		/* Deliberately retain the enabled gate for this permanent timer. */
		__raw_writel(0, gptc_base[i] + GPTC_IRNEN);
		__raw_writel(0xff, gptc_base[i] + GPTC_IRNCR);
		__raw_writel(BIT(16) | BIT(8), gptc_base[i] + GPTC_CLC);
		if (((__raw_readl(gptc_base[i] + GPTC_ID) >> 8) & 0xff) != 0x59)
			panic("GRX500 GPTC: unexpected block %u ID", i);
	}

	__raw_writel(GPTC_STOP, gptc_base[0] + GPTC_RUN(GPTC_SOURCE));
	/* Vendor csrc-gptc.c modifies CON with |= and keeps the bits the
	 * bootloader/firmware may have set (SYNC=BIT8, CLK_INT/CLK_EXT=BIT9/10,
	 * EDGE=BIT6/7). Overwriting CON reconfigured the shared timer input
	 * behind the firmware's back; on hardware that produced backwards
	 * jumps of the source counter (observed -0.25 s and -10.69 s) and
	 * clocksource instability. Preserve the unknown bits like the vendor.
	 */
	value = __raw_readl(gptc_base[0] + GPTC_CON(GPTC_SOURCE));
	value |= GPTC_UP | GPTC_32BIT;
	__raw_writel(value, gptc_base[0] + GPTC_CON(GPTC_SOURCE));
	__raw_writel(0, gptc_base[0] + GPTC_RLD(GPTC_SOURCE));
	__raw_writel(GPTC_START | GPTC_RELOAD,
		     gptc_base[0] + GPTC_RUN(GPTC_SOURCE));
	ret = read_poll_timeout_atomic(__raw_readl, value,
			value & GPTC_ENABLED, 1, 1000, false,
			gptc_base[0] + GPTC_CON(GPTC_SOURCE));
	if (ret)
		panic("GRX500 GPTC: source did not start");
	ret = clocksource_register_hz(&gptc_clocksource, gptc_rate);
	if (ret)
		panic("GRX500 GPTC: clocksource registration failed: %d", ret);

	for (i = 0; i < GPTC_CPUS; i++) {
		struct grx500_gptc_event *ev = &gptc_events[i];

		ev->base = gptc_base[i == 1 ? 0 : 1];
		ev->channel = channels[i];
		ev->irq = irq_of_parse_and_map(np, i);
		if (!ev->irq)
			panic("GRX500 GPTC: missing CPU%u IRQ", i);
		snprintf(ev->name, sizeof(ev->name), "gptc_clockevent_%u", i);
		ev->ce.name = ev->name;
		ev->ce.rating = 400;
		ev->ce.features = CLOCK_EVT_FEAT_ONESHOT;
		ev->ce.cpumask = cpumask_of(i);
		ev->ce.irq = ev->irq;
		ev->ce.set_next_event = gptc_next_event;
		ev->ce.set_state_shutdown = gptc_shutdown;
		ev->ce.set_state_oneshot = gptc_oneshot;
		if (gptc_shutdown(&ev->ce))
			panic("GRX500 GPTC: CPU%u channel did not stop", i);
		ret = request_irq(ev->irq, gptc_interrupt,
			IRQF_TIMER | IRQF_NOBALANCING | IRQF_NO_AUTOEN, ev->name, ev);
		if (ret)
			panic("GRX500 GPTC: CPU%u IRQ request failed: %d", i, ret);
	}
	of_node_put(np);
	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "clockevents/grx500:online",
			       gptc_cpu_online, gptc_cpu_offline);
	if (ret < 0)
		panic("GRX500 GPTC: CPU event setup failed: %d", ret);
	pr_info("vendor timer layout active: shared GPTC0/3A at %u Hz, four CPU events\n",
		gptc_rate);
	return 0;
}
arch_initcall(grx500_gptc_init);
