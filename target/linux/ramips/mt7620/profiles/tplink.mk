#
# Copyright (C) 2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/ArcherC2
	NAME:=TP-Link ArcherC2
	PACKAGES:=\
		kmod-usb-core kmod-usb2 kmod-usb-ohci kmod-ledtrig-usbdev \
		kmod-mt7610e luci-mtk-wifi
endef

define Profile/ArcherC2/Description
	Package set compatible with the TP-Link ArcherC2 board.
endef
$(eval $(call Profile,ArcherC2))
