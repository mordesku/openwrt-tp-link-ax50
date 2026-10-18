/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_MACH_GRX500_MANGLE_PORT_H
#define __ASM_MACH_GRX500_MANGLE_PORT_H

#include <asm/byteorder.h>

#define __swizzle_addr_b(port)	(port)
#define __swizzle_addr_w(port)	(port)
#define __swizzle_addr_l(port)	(port)
#define __swizzle_addr_q(port)	(port)

#if defined(CONFIG_SWAP_IO_SPACE)

static inline bool grx500_is_big_endian_mmio(const volatile void *a)
{
	unsigned long addr = (unsigned long)a;

	/*
	 * On GRX500, U-Boot sets RCU_NGI_ENDIAN to 0xdfffff, which configures
	 * internal SoC slave registers on NGI to Big-Endian 32-bit format.
	 * DWC3 / xHCI controller registers at physical 0x1a300000..0x1a6fffff
	 * (KSEG1 uncached 0xba300000..0xba6fffff) must NOT be byte-swapped
	 * by readl/writel.
	 */
	return (addr >= 0xba300000UL && addr < 0xba700000UL);
}

# define ioswabb(a, x)		(x)
# define __mem_ioswabb(a, x)	(x)
# define ioswabw(a, x)		(grx500_is_big_endian_mmio(a) ? (x) : \
				 le16_to_cpu((__force __le16)(x)))
# define __mem_ioswabw(a, x)	(x)
# define ioswabl(a, x)		(grx500_is_big_endian_mmio(a) ? (x) : \
				 le32_to_cpu((__force __le32)(x)))
# define __mem_ioswabl(a, x)	(x)
# define ioswabq(a, x)		(grx500_is_big_endian_mmio(a) ? (x) : \
				 le64_to_cpu((__force __le64)(x)))
# define __mem_ioswabq(a, x)	(x)

#else

# define ioswabb(a, x)		(x)
# define __mem_ioswabb(a, x)	(x)
# define ioswabw(a, x)		(x)
# define __mem_ioswabw(a, x)	((__force u16)cpu_to_le16(x))
# define ioswabl(a, x)		(x)
# define __mem_ioswabl(a, x)	((__force u32)cpu_to_le32(x))
# define ioswabq(a, x)		(x)
# define __mem_ioswabq(a, x)	((__force u64)cpu_to_le64(x))

#endif

#endif /* __ASM_MACH_GRX500_MANGLE_PORT_H */
