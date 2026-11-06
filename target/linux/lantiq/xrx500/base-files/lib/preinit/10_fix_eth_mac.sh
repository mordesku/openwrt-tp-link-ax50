#!/bin/sh

. /lib/functions/system.sh

preinit_set_mac_address() {
	case $(board_name) in
	tplink,archer-ax50-v1)
		local label_mac=""
		local lan_mac=""
		local wan_mac=""

		# Check /proc/cmdline for ethaddr passed by U-Boot
		label_mac=$(grep -o 'ethaddr=[0-9a-fA-F:]*' /proc/cmdline 2>/dev/null | cut -d= -f2)
		[ -z "$label_mac" ] && label_mac=$(mtd_get_mac_ascii "ubootconfigA" "ethaddr")
		[ -z "$label_mac" ] && label_mac=$(mtd_get_mac_ascii "ubootconfigB" "ethaddr")

		# Ensure switch and datapath modules are loaded
		[ -d /sys/class/net/eth0 ] || {
			modprobe mdio-grx500 2>/dev/null
			modprobe grx500-datapath 2>/dev/null
		}

		if [ -n "$label_mac" ]; then
			lan_mac="$label_mac"
			wan_mac=$(macaddr_add "$label_mac" 1)
			wlan0_mac=$(macaddr_add "$label_mac" 2)
			wlan1_mac=$(macaddr_add "$label_mac" 3)

			ip link set dev eth0 address "$wan_mac" 2>/dev/null
			ip link set dev eth1 address "$lan_mac" 2>/dev/null
			ip link set dev eth2 address "$lan_mac" 2>/dev/null
			ip link set dev eth3 address "$lan_mac" 2>/dev/null
			ip link set dev eth4 address "$lan_mac" 2>/dev/null
			[ -d /sys/class/net/wlan0 ] && ip link set dev wlan0 address "$wlan0_mac" 2>/dev/null
			[ -d /sys/class/net/wlan1 ] && ip link set dev wlan1 address "$wlan1_mac" 2>/dev/null
		fi
		;;
	esac
}

boot_hook_add preinit_main preinit_set_mac_address
