# SPDX-License-Identifier: GPL-2.0
# Lantiq DTS Makefile — extended for GRX500 (xRX500) family
#
# Upstream only has danube_easy50712. Our OpenWrt subtarget adds GRX500 boards.

dtb-$(CONFIG_DT_EASY50712)        += danube_easy50712.dtb

# GRX500 / xRX500 boards (TP-Link Archer AX50 v1 and future variants)
dtb-$(CONFIG_SOC_GRX500)          += grx500_tplink_archer-ax50-v1.dtb

obj-$(CONFIG_BUILTIN_DTB)         += $(addsuffix .o, $(dtb-y))
