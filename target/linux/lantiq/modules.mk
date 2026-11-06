# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2010 OpenWrt.org

I2C_LANTIQ_MODULES:= \
  CONFIG_I2C_LANTIQ:drivers/i2c/busses/i2c-lantiq

define KernelPackage/i2c-lantiq
  TITLE:=Lantiq I2C controller
  $(call i2c_defaults,$(I2C_LANTIQ_MODULES),52)
  DEPENDS:=+kmod-i2c-core @TARGET_lantiq_falcon
endef

define KernelPackage/i2c-lantiq/description
  Kernel support for the Lantiq/Falcon I2C controller
endef

$(eval $(call KernelPackage,i2c-lantiq))

define KernelPackage/dsa-gswip
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Lantiq xRx200/xRx300/xRx330 switch support
  DEPENDS:=@TARGET_lantiq_xrx200||TARGET_lantiq_xrx200_legacy
  KCONFIG:= \
  	CONFIG_NET_DSA_LANTIQ_GSWIP \
  	CONFIG_NET_DSA_TAG_GSWIP
  FILES:= \
	$(LINUX_DIR)/drivers/net/dsa/lantiq_gswip.ko \
  	$(LINUX_DIR)/net/dsa/tag_gswip.ko
  AUTOLOAD:=$(call AutoLoad,41,tag_gswip lantiq_gswip,1)
endef

define KernelPackage/dsa-gswip/description
 Lantiq xRx200/xRx300/xRx330 switch support
endef

$(eval $(call KernelPackage,dsa-gswip))

define KernelPackage/grx500-switch
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Lantiq GRX500 GSWIP and Datapath support
  DEPENDS:=@TARGET_lantiq_xrx500
  KCONFIG:= \
	CONFIG_MDIO_GRX500_GSWIP \
	CONFIG_GRX500_DATAPATH
  FILES:= \
	$(LINUX_DIR)/drivers/net/mdio/mdio-grx500.ko \
	$(LINUX_DIR)/drivers/net/ethernet/grx500-datapath.ko
  AUTOLOAD:=$(call AutoLoad,40,mdio-grx500 grx500-datapath)
endef

define KernelPackage/grx500-switch/description
  Lantiq GRX500 GSWIP MDIO and Datapath network driver
endef

$(eval $(call KernelPackage,grx500-switch))

define KernelPackage/wave600-wifi
  SUBMENU:=$(WIRELESS_MENU)
  TITLE:=Intel/MaxLinear Wave600 (WAV654) Wi-Fi 6 driver
  DEPENDS:=@TARGET_lantiq_xrx500
  KCONFIG:= \
    CONFIG_WAVE600_WIFI \
    CONFIG_CFG80211
  FILES:= \
    $(LINUX_DIR)/net/wireless/cfg80211.ko \
    $(LINUX_DIR)/drivers/net/wireless/intel/wave600/wave600_wifi.ko
  AUTOLOAD:=$(call AutoLoad,60,cfg80211 wave600_wifi)
endef

define KernelPackage/wave600-wifi/description
  Clean modern driver for Intel/MaxLinear WAV654 802.11ax PCIe radios
endef

$(eval $(call KernelPackage,wave600-wifi))

