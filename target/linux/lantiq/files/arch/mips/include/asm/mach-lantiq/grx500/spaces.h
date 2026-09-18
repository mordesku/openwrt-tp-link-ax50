/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GRX500 EVA 1GB layout adapted from the TP-Link GPL kernel.
 * Copyright (C) 2012 MIPS Technologies, Inc.
 * Copyright (C) 2014 Lei Chuanhua <Chuanhua.lei@lantiq.com>
 *
 * Preserve vendor PAGE_OFFSET/PHYS_OFFSET while using modern __pa/__va.
 * Low MMIO uses the vendor BEV overlay, not the DDR uncached alias.
 */
#ifndef __ASM_MACH_GRX500_SPACES_H
#define __ASM_MACH_GRX500_SPACES_H

#if defined(CONFIG_SOC_GRX500) && defined(CONFIG_EVA)
#define PAGE_OFFSET _AC(0x60000000, UL)
#define PHYS_OFFSET _AC(0x20000000, UL)
#define HIGHMEM_START _AC(0xffff0000, UL)
#endif

#include <asm/mach-generic/spaces.h>

#if defined(CONFIG_SOC_GRX500) && defined(CONFIG_EVA)
/* Generic 32-bit spaces.h defines CAC_BASE unconditionally. */
#undef CAC_BASE
#define CAC_BASE _AC(0x60000000, UL)
#endif
#endif
