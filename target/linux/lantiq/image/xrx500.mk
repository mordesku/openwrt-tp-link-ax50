define Device/NAND/xrx500
  BLOCKSIZE := 128k
  PAGESIZE := 2048
  SUBPAGESIZE := 2048
  FILESYSTEMS += ubifs
endef

define Device/tplink_archer-ax50-v1
  $(Device/NAND)
  DEVICE_VENDOR := TP-Link
  DEVICE_MODEL := Archer AX50
  DEVICE_VARIANT := v1
  DEVICE_DTS := grx500_tplink_archer-ax50-v1-slot-b
  KERNEL_INSTALL := 1
  KERNEL_LOADADDR := 0xa0020000
  KERNEL_ENTRY := 0xa0020000
  KERNEL := kernel-bin | append-dtb | lzma | uImage lzma
  KERNEL_INITRAMFS := kernel-bin | append-dtb | lzma | uImage lzma
  DEVICE_PACKAGES := kmod-wave600-wifi wpad-basic-openssl iw wireless-regdb
  SUPPORTED_DEVICES += tplink,archer-ax50-v1
endef
TARGET_DEVICES += tplink_archer-ax50-v1
