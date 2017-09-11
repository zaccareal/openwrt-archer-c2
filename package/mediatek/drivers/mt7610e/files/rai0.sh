#!/bin/sh
append DRIVERS "rai0"

prepare_rai0() {
	echo "prepare_rai0()" >>/tmp/wifi.log
}

scan_rai0() {
	echo "scan_rai0()" >>/tmp/wifi.log
	echo " > uci2dat -d rai0 -f /etc/Wireless/iNIC/iNIC_ap.dat" >>/tmp/wifi.log
	uci2dat -d rai0 -f /etc/Wireless/iNIC/iNIC_ap.dat > /tmp/uci2dat.log
}

disable_rai0() {
	echo "disable_rai0()" >>/tmp/wifi.log
	echo " > ifconfig rai0 down" >>/tmp/wifi.log
	ifconfig rai0 down
}

enable_rai0() {
	echo "enable_ralink_wifi()" >>/tmp/wifi.log
	config_get_bool disabled default_rai0 disabled 0
	echo " > ifconfig rai0 down" >>/tmp/wifi.log
	ifconfig rai0 down
	if [ $disabled -eq 1 ]; then
		echo " > rai0 marked disabled, skip" >>/tmp/wifi.log
		continue
	else
		echo " > ifconfig rai0 up" >>/tmp/wifi.log
		ifconfig rai0 up
	fi
}

detect_rai0() {
	echo "detect_rai0()" >>/tmp/wifi.log
	uci get wireless.rai0 >/dev/null 2>&1 && return
	ifconfig rai0 >/dev/null 2>&1 || return
	echo " > rai0 config not found, load default" >>/tmp/wifi.log
	cat <<EOF
config wifi-device rai0
	option type rai0
	option vendor ralink
	option band 5G
	option channel 0
	option autoch 2
	option disabled 1

config wifi-iface default_rai0
	option device rai0
	option ifname rai0
	option network lan
	option mode ap
	option ssid OpenWrt_5G
	option encryption none

EOF
}
