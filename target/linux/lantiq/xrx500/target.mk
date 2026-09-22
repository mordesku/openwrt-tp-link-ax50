ARCH:=mips
SUBTARGET:=xrx500
BOARDNAME:=XRX500 / GRX500
FEATURES+=nand ramdisk
CPU_TYPE:=interaptiv

DEFAULT_PACKAGES+=kmod-leds-gpio \
	kmod-gpio-button-hotplug \
	kmod-grx500-switch

define Target/Description
	Lantiq / Intel XRX500 / GRX500 (e.g. TP-Link Archer AX50 v1)
endef
