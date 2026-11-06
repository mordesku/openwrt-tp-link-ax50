#!/bin/sh

. /lib/functions/system.sh

extract_wifi_caldata() {
	case $(board_name) in
	tplink,archer-ax50-v1)
		# Skip if calibration file is already present
		[ -s /lib/firmware/cal_wlan0.bin ] && return 0

		local mtd_dev=$(find_mtd_part "calibration")
		[ -z "$mtd_dev" ] && mtd_dev="/dev/mtd4"
		[ -c "$mtd_dev" ] || [ -b "$mtd_dev" ] || return 0

		mkdir -p /tmp/cal
		dd if="$mtd_dev" of=/tmp/cal/mtd4.bin bs=64k count=4 2>/dev/null
		tar -xzf /tmp/cal/mtd4.bin -C /tmp/cal 2>/dev/null || tar -xf /tmp/cal/mtd4.bin -C /tmp/cal 2>/dev/null
		if [ -f /tmp/cal/wlanconfig.gz ]; then
			tar -xzf /tmp/cal/wlanconfig.gz -C /lib/firmware/ 2>/dev/null
		elif [ -f /tmp/cal/cal_wlan0.bin ]; then
			cp /tmp/cal/cal_wlan*.bin /lib/firmware/ 2>/dev/null
		fi
		rm -rf /tmp/cal
		;;
	esac
}

boot_hook_add preinit_mount_root extract_wifi_caldata
