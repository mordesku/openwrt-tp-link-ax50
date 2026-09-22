/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GRX500_MEMORY_H
#define _LINUX_GRX500_MEMORY_H

#include <linux/types.h>

/* Built-in platform code supplies linker bounds to modular DMA consumers. */
void grx500_kernel_phys_range(phys_addr_t *start, phys_addr_t *end);

#endif
