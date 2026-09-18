/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GRX500 EVA 1GB segment setup adapted from the TP-Link GPL kernel.
 * Chris Dearman (chris@mips.com)
 * Leonid Yegoshin (yegoshin@mips.com)
 * Copyright (C) 2012 Mips Technologies, Inc.
 *
 * Hypothesis: direct RAM accesses and TLB-backed vmalloc accesses must
 * agree on the vendor physical DDR base (0x20000000).
 * Keep vendor SegCtl fields; expose platform_eva_init for Linux 6.12 CPS.
 */
#ifndef __ASM_MACH_LANTIQ_KERNEL_ENTRY_INIT_H
#define __ASM_MACH_LANTIQ_KERNEL_ENTRY_INIT_H

#if defined(CONFIG_SOC_GRX500) && defined(CONFIG_EVA)
#include <asm/regdef.h>
#include <asm/mipsregs.h>

#ifdef __ASSEMBLY__

	.macro platform_eva_init
	.set push
	.set reorder
	mfc0 t1, CP0_CONFIG
	andi t1, t1, 7
	/* SegCtl0: mapped c0000000; uncached e0000000 -> a0000000. */
	li t0, 0x00280a0a
	ins t0, t1, 16, 3
	mtc0 t0, CP0_SEGCTL0
	/* SegCtl1: 80000000 -> 40000000; a0000000 -> 20000000. */
	li t0, 0x0408020a
	ins t0, t1, 16, 3
	mtc0 t0, CP0_SEGCTL1
	/* SegCtl2: mapped low 1GB; 40000000 -> 00000000 in kernel. */
	li t0, 0x00380048
	ins t0, t1, 0, 3
	mtc0 t0, CP0_SEGCTL2
	jal mips_ihb
	mfc0 t0, $16, 5
	li t2, 0x40000000
	or t0, t0, t2
	mtc0 t0, $16, 5
	sync
	jal mips_ihb
	.set pop
	.endm

	.macro kernel_entry_setup
	sync
	ehb
	platform_eva_init
	.endm

	.macro smp_slave_setup
	sync
	ehb
	platform_eva_init
	.endm

#endif /* __ASSEMBLY__ */
#else
#include <asm/mach-generic/kernel-entry-init.h>
#endif
#endif
