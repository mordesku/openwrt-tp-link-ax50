/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Lei Chuanhua <Chuanhua.lei@lantiq.com>
 *
 * TOP IO Space definition for SSX7 components / PCIe / ToE / Memcpy:
 * physical 0xa0000000..0xbfffffff --> virtual 0xe0000000..0xffffffff (EVA Seg0 UK)
 */
#ifndef __ASM_MACH_LANTIQ_IOREMAP_H
#define __ASM_MACH_LANTIQ_IOREMAP_H

#include <linux/types.h>

#ifdef CONFIG_SOC_GRX500

#define GRX500_TOP_IOREMAP_BASE			0xA0000000UL
#define GRX500_TOP_IOREMAP_SIZE			0x20000000UL
#define GRX500_TOP_IOREMAP_PHYS_VIRT_OFFSET	0x40000000UL

static inline void __iomem *plat_ioremap(phys_addr_t offset, unsigned long size,
					 unsigned long flags)
{
	if (offset >= GRX500_TOP_IOREMAP_BASE &&
	    offset < (GRX500_TOP_IOREMAP_BASE + GRX500_TOP_IOREMAP_SIZE))
		return (void __iomem *)(unsigned long)
			(offset + GRX500_TOP_IOREMAP_PHYS_VIRT_OFFSET);
	return NULL;
}

static inline int plat_iounmap(const volatile void __iomem *addr)
{
	return (unsigned long)addr >=
		(GRX500_TOP_IOREMAP_BASE + GRX500_TOP_IOREMAP_PHYS_VIRT_OFFSET);
}

#else

static inline void __iomem *plat_ioremap(phys_addr_t offset, unsigned long size,
					 unsigned long flags)
{
	return NULL;
}

static inline int plat_iounmap(const volatile void __iomem *addr)
{
	return 0;
}

#endif /* CONFIG_SOC_GRX500 */

#endif /* __ASM_MACH_LANTIQ_IOREMAP_H */
