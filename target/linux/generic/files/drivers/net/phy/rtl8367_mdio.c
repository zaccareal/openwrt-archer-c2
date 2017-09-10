/*
 * Realtek RTL8367 MDIO interface driver
 *
 * Copyright (C) 2009-2010 Gabor Juhos <juhosg@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/rtl8366.h>

#ifdef CONFIG_RTL8366_SMI_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include "rtl8367_mdio.h"

// from mii_mgr.c
extern int mii_mgr_read(u32 phy_addr, u32 phy_register, u32 *read_data);
extern int mii_mgr_write(u32 phy_addr, u32 phy_register, u32 write_data);

#define RTL8366_SMI_ACK_RETRY_COUNT         5

#define RTL8366_SMI_HW_STOP_DELAY		25	/* msecs */
#define RTL8366_SMI_HW_START_DELAY		100	/* msecs */

#define RTL8367_ERROR(fmt, args...) printk("\033[1m[ %s ] %03d: "fmt"\033[0m", __FUNCTION__, __LINE__, ##args)
#define RTL8367_DEBUG(fmt, args...) printk("\033[4m[ %s ] %03d: "fmt"\033[0m", __FUNCTION__, __LINE__, ##args)

/* MDC_MDIO */
#define MDC_MDIO_DUMMY_ID           0x0
#define MDC_MDIO_CTRL0_REG          31
#define MDC_MDIO_START_REG          29
#define MDC_MDIO_CTRL1_REG          21
#define MDC_MDIO_ADDRESS_REG        23
#define MDC_MDIO_DATA_WRITE_REG     24
#define MDC_MDIO_DATA_READ_REG      25
#define MDC_MDIO_PREAMBLE_LEN       32
 
#define MDC_MDIO_START_OP          0xFFFF
#define MDC_MDIO_ADDR_OP           0x000E
#define MDC_MDIO_READ_OP           0x0001
#define MDC_MDIO_WRITE_OP          0x0003

//uboot
u32 rtl_smi_write(u32 mAddrs, u32 rData)
{
	//RTL8367_DEBUG("About write 0x%x to 0x%x\n", rData, mAddrs);
	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Write address control code to register 31 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);
	
	/* Write address to register 23 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Write data to register 24 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_DATA_WRITE_REG, rData);

	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Write Start control code to register 21 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_WRITE_OP);

	return 0;
}

u32 rtl_smi_read(u32 mAddrs, u32* rData)
{
	//RTL8367_DEBUG("Try to read at 0x%x\n", mAddrs);
	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Write address control code to register 31 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Write address to register 23 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Write read control code to register 21 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_READ_OP);

	/* Write Start command to register 29 */
	mii_mgr_write(MDC_MDIO_DUMMY_ID, MDC_MDIO_START_REG, MDC_MDIO_START_OP);

	/* Read data from register 25 */
	if (1 != mii_mgr_read(MDC_MDIO_DUMMY_ID, MDC_MDIO_DATA_READ_REG, rData))
	{
		RTL8367_DEBUG("mii_mgr_read error\n");
		return 1;
	}
	//RTL8367_DEBUG("Reg 0x%x with data 0x%x\n", mAddrs, *rData);
	return 0;
}

int rtl8366_smi_read_reg(struct rtl8366_smi *smi, u32 addr, u32 *data)
{
	return rtl_smi_read(addr, data);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_read_reg);

int rtl8366_smi_write_reg(struct rtl8366_smi *smi, u32 addr, u32 data)
{
	return rtl_smi_write(addr, data);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_write_reg);

/*
int rtl8366_smi_write_reg_noack(struct rtl8366_smi *smi, u32 addr, u32 data)
{
	return __rtl8366_smi_write_reg(smi, addr, data, false);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_write_reg_noack);
*/

int rtl8366_smi_rmwr(struct rtl8366_smi *smi, u32 addr, u32 mask, u32 data)
{
	u32 t;
	int err;

	err = rtl8366_smi_read_reg(smi, addr, &t);
	if (err)
		return err;

	err = rtl8366_smi_write_reg(smi, addr, (t & ~mask) | data);
	return err;

}
EXPORT_SYMBOL_GPL(rtl8366_smi_rmwr);

static int rtl8366_reset(struct rtl8366_smi *smi)
{
	if (smi->hw_reset) {
		smi->hw_reset(true);
		msleep(RTL8366_SMI_HW_STOP_DELAY);
		smi->hw_reset(false);
		msleep(RTL8366_SMI_HW_START_DELAY);
		return 0;
	}

	return smi->ops->reset_chip(smi);
}

static int rtl8366_mc_is_used(struct rtl8366_smi *smi, int mc_index, int *used)
{
	int err;
	int i;

	*used = 0;
	for (i = 0; i < smi->num_ports; i++) {
		int index = 0;

		err = smi->ops->get_mc_index(smi, i, &index);
		if (err)
			return err;

		if (mc_index == index) {
			*used = 1;
			break;
		}
	}

	return 0;
}

static int rtl8366_set_vlan(struct rtl8366_smi *smi, int vid, u32 member,
			    u32 untag, u32 fid)
{
	struct rtl8366_vlan_4k vlan4k;
	int err;
	int i;

	/* Update the 4K table */
	err = smi->ops->get_vlan_4k(smi, vid, &vlan4k);
	if (err)
		return err;

	vlan4k.member = member;
	vlan4k.untag = untag;
	vlan4k.fid = fid;
	err = smi->ops->set_vlan_4k(smi, &vlan4k);
	if (err)
		return err;

	/* Try to find an existing MC entry for this VID */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		struct rtl8366_vlan_mc vlanmc;

		err = smi->ops->get_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;

		if (vid == vlanmc.vid) {
			/* update the MC entry */
			vlanmc.member = member;
			vlanmc.untag = untag;
			vlanmc.fid = fid;

			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			break;
		}
	}

	return err;
}

static int rtl8366_get_pvid(struct rtl8366_smi *smi, int port, int *val)
{
	struct rtl8366_vlan_mc vlanmc;
	int err;
	int index;

	err = smi->ops->get_mc_index(smi, port, &index);
	if (err)
		return err;

	err = smi->ops->get_vlan_mc(smi, index, &vlanmc);
	if (err)
		return err;

	*val = vlanmc.vid;
	return 0;
}

static int rtl8366_set_pvid(struct rtl8366_smi *smi, unsigned port,
			    unsigned vid)
{
	struct rtl8366_vlan_mc vlanmc;
	struct rtl8366_vlan_4k vlan4k;
	int err;
	int i;

	/* Try to find an existing MC entry for this VID */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		err = smi->ops->get_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;

		if (vid == vlanmc.vid) {
			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			if (err)
				return err;

			err = smi->ops->set_mc_index(smi, port, i);
			return err;
		}
	}

	/* We have no MC entry for this VID, try to find an empty one */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		err = smi->ops->get_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;

		if (vlanmc.vid == 0 && vlanmc.member == 0) {
			/* Update the entry from the 4K table */
			err = smi->ops->get_vlan_4k(smi, vid, &vlan4k);
			if (err)
				return err;

			vlanmc.vid = vid;
			vlanmc.member = vlan4k.member;
			vlanmc.untag = vlan4k.untag;
			vlanmc.fid = vlan4k.fid;
			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			if (err)
				return err;

			err = smi->ops->set_mc_index(smi, port, i);
			return err;
		}
	}

	/* MC table is full, try to find an unused entry and replace it */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		int used;

		err = rtl8366_mc_is_used(smi, i, &used);
		if (err)
			return err;

		if (!used) {
			/* Update the entry from the 4K table */
			err = smi->ops->get_vlan_4k(smi, vid, &vlan4k);
			if (err)
				return err;

			vlanmc.vid = vid;
			vlanmc.member = vlan4k.member;
			vlanmc.untag = vlan4k.untag;
			vlanmc.fid = vlan4k.fid;
			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			if (err)
				return err;

			err = smi->ops->set_mc_index(smi, port, i);
			return err;
		}
	}

	dev_err(smi->parent,
		"all VLAN member configurations are in use\n");

	return -ENOSPC;
}

int rtl8366_enable_vlan(struct rtl8366_smi *smi, int enable)
{
	int err;

	err = smi->ops->enable_vlan(smi, enable);
	if (err)
		return err;

	smi->vlan_enabled = enable;

	if (!enable) {
		smi->vlan4k_enabled = 0;
		err = smi->ops->enable_vlan4k(smi, enable);
	}

	return err;
}
EXPORT_SYMBOL_GPL(rtl8366_enable_vlan);

static int rtl8366_enable_vlan4k(struct rtl8366_smi *smi, int enable)
{
	int err;

	if (enable) {
		err = smi->ops->enable_vlan(smi, enable);
		if (err)
			return err;

		smi->vlan_enabled = enable;
	}

	err = smi->ops->enable_vlan4k(smi, enable);
	if (err)
		return err;

	smi->vlan4k_enabled = enable;
	return 0;
}

int rtl8366_enable_all_ports(struct rtl8366_smi *smi, int enable)
{
	int port;
	int err;

	for (port = 0; port < smi->num_ports; port++) {
		err = smi->ops->enable_port(smi, port, enable);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_enable_all_ports);

int rtl8366_reset_vlan(struct rtl8366_smi *smi)
{
	struct rtl8366_vlan_mc vlanmc;
	int err;
	int i;

	rtl8366_enable_vlan(smi, 0);
	rtl8366_enable_vlan4k(smi, 0);

	/* clear VLAN member configurations */
	vlanmc.vid = 0;
	vlanmc.priority = 0;
	vlanmc.member = 0;
	vlanmc.untag = 0;
	vlanmc.fid = 0;
	for (i = 0; i < smi->num_vlan_mc; i++) {
		err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_reset_vlan);

static int rtl8366_init_vlan(struct rtl8366_smi *smi)
{
	int port;
	int err;

	err = rtl8366_reset_vlan(smi);
	if (err)
		return err;

	for (port = 0; port < smi->num_ports; port++) {
		u32 mask;

		if (port == smi->cpu_port)
			mask = (1 << smi->num_ports) - 1;
		else
			mask = (1 << port) | (1 << smi->cpu_port);

		err = rtl8366_set_vlan(smi, (port + 1), mask, mask, 0);
		if (err)
			return err;

		err = rtl8366_set_pvid(smi, port, (port + 1));
		if (err)
			return err;
	}

	return rtl8366_enable_vlan(smi, 1);
}

static int rtl8366_smi_mii_init(struct rtl8366_smi *smi)
{
	int ret;
	int i;

	smi->mii_bus = mdiobus_alloc();
	if (smi->mii_bus == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	smi->mii_bus->priv = (void *) smi;
	smi->mii_bus->name = dev_name(smi->parent);
	smi->mii_bus->read = smi->ops->mii_read;
	smi->mii_bus->write = smi->ops->mii_write;
	snprintf(smi->mii_bus->id, MII_BUS_ID_SIZE, "%s",
		 dev_name(smi->parent));
	smi->mii_bus->parent = smi->parent;
	smi->mii_bus->phy_mask = ~(0x1f);
	smi->mii_bus->irq = smi->mii_irq;
	for (i = 0; i < PHY_MAX_ADDR; i++)
		smi->mii_irq[i] = PHY_POLL;

	ret = mdiobus_register(smi->mii_bus);
	if (ret)
		goto err_free;

	return 0;

 err_free:
	mdiobus_free(smi->mii_bus);
 err:
	return ret;
}

static void rtl8366_smi_mii_cleanup(struct rtl8366_smi *smi)
{
	mdiobus_unregister(smi->mii_bus);
	mdiobus_free(smi->mii_bus);
}

int rtl8366_sw_reset_switch(struct switch_dev *dev)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	int err;

	err = rtl8366_reset(smi);
	if (err)
		return err;

	err = smi->ops->setup(smi);
	if (err)
		return err;

	err = rtl8366_reset_vlan(smi);
	if (err)
		return err;

	err = rtl8366_enable_vlan(smi, 1);
	if (err)
		return err;

	return rtl8366_enable_all_ports(smi, 1);
}
EXPORT_SYMBOL_GPL(rtl8366_sw_reset_switch);

int rtl8366_sw_get_port_pvid(struct switch_dev *dev, int port, int *val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	return rtl8366_get_pvid(smi, port, val);
}
EXPORT_SYMBOL_GPL(rtl8366_sw_get_port_pvid);

int rtl8366_sw_set_port_pvid(struct switch_dev *dev, int port, int val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	return rtl8366_set_pvid(smi, port, val);
}
EXPORT_SYMBOL_GPL(rtl8366_sw_set_port_pvid);

int rtl8366_sw_get_port_mib(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	int i, len = 0;
	unsigned long long counter = 0;
	char *buf = smi->buf;

	if (val->port_vlan >= smi->num_ports)
		return -EINVAL;

	len += snprintf(buf + len, sizeof(smi->buf) - len,
			"Port %d MIB counters\n",
			val->port_vlan);

	for (i = 0; i < smi->num_mib_counters; ++i) {
		len += snprintf(buf + len, sizeof(smi->buf) - len,
				"%-36s: ", smi->mib_counters[i].name);
		if (!smi->ops->get_mib_counter(smi, i, val->port_vlan,
					       &counter))
			len += snprintf(buf + len, sizeof(smi->buf) - len,
					"%llu\n", counter);
		else
			len += snprintf(buf + len, sizeof(smi->buf) - len,
					"%s\n", "error");
	}

	val->value.s = buf;
	val->len = len;
	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_sw_get_port_mib);

int rtl8366_sw_get_vlan_info(struct switch_dev *dev,
			     const struct switch_attr *attr,
			     struct switch_val *val)
{
	int i;
	u32 len = 0;
	struct rtl8366_vlan_4k vlan4k;
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	char *buf = smi->buf;
	int err;

	if (!smi->ops->is_vlan_valid(smi, val->port_vlan))
		return -EINVAL;

	memset(buf, '\0', sizeof(smi->buf));

	err = smi->ops->get_vlan_4k(smi, val->port_vlan, &vlan4k);
	if (err)
		return err;

	len += snprintf(buf + len, sizeof(smi->buf) - len,
			"VLAN %d: Ports: '", vlan4k.vid);

	for (i = 0; i < smi->num_ports; i++) {
		if (!(vlan4k.member & (1 << i)))
			continue;

		len += snprintf(buf + len, sizeof(smi->buf) - len, "%d%s", i,
				(vlan4k.untag & (1 << i)) ? "" : "t");
	}

	len += snprintf(buf + len, sizeof(smi->buf) - len,
			"', members=%04x, untag=%04x, fid=%u",
			vlan4k.member, vlan4k.untag, vlan4k.fid);

	val->value.s = buf;
	val->len = len;

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_sw_get_vlan_info);

int rtl8366_sw_get_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	struct switch_port *port;
	struct rtl8366_vlan_4k vlan4k;
	int i;

	if (!smi->ops->is_vlan_valid(smi, val->port_vlan))
		return -EINVAL;

	smi->ops->get_vlan_4k(smi, val->port_vlan, &vlan4k);

	port = &val->value.ports[0];
	val->len = 0;
	for (i = 0; i < smi->num_ports; i++) {
		if (!(vlan4k.member & BIT(i)))
			continue;

		port->id = i;
		port->flags = (vlan4k.untag & BIT(i)) ?
					0 : BIT(SWITCH_PORT_FLAG_TAGGED);
		val->len++;
		port++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_sw_get_vlan_ports);

int rtl8366_sw_set_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	struct switch_port *port;
	u32 member = 0;
	u32 untag = 0;
	int err;
	int i;

	if (!smi->ops->is_vlan_valid(smi, val->port_vlan))
		return -EINVAL;

	port = &val->value.ports[0];
	for (i = 0; i < val->len; i++, port++) {
		int pvid = 0;
		member |= BIT(port->id);

		if (!(port->flags & BIT(SWITCH_PORT_FLAG_TAGGED)))
			untag |= BIT(port->id);

		/*
		 * To ensure that we have a valid MC entry for this VLAN,
		 * initialize the port VLAN ID here.
		 */
		err = rtl8366_get_pvid(smi, port->id, &pvid);
		if (err < 0)
			return err;
		if (pvid == 0) {
			err = rtl8366_set_pvid(smi, port->id, val->port_vlan);
			if (err < 0)
				return err;
		}
	}

	return rtl8366_set_vlan(smi, val->port_vlan, member, untag, 0);
}
EXPORT_SYMBOL_GPL(rtl8366_sw_set_vlan_ports);

int rtl8366_sw_get_vlan_fid(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	struct rtl8366_vlan_4k vlan4k;
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	int err;

	if (!smi->ops->is_vlan_valid(smi, val->port_vlan))
		return -EINVAL;

	err = smi->ops->get_vlan_4k(smi, val->port_vlan, &vlan4k);
	if (err)
		return err;

	val->value.i = vlan4k.fid;

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_sw_get_vlan_fid);

int rtl8366_sw_set_vlan_fid(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	struct rtl8366_vlan_4k vlan4k;
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	int err;

	if (!smi->ops->is_vlan_valid(smi, val->port_vlan))
		return -EINVAL;

	if (val->value.i < 0 || val->value.i > attr->max)
		return -EINVAL;

	err = smi->ops->get_vlan_4k(smi, val->port_vlan, &vlan4k);
	if (err)
		return err;

	return rtl8366_set_vlan(smi, val->port_vlan,
				vlan4k.member,
				vlan4k.untag,
				val->value.i);
}
EXPORT_SYMBOL_GPL(rtl8366_sw_set_vlan_fid);

int rtl8366_sw_get_vlan_enable(struct switch_dev *dev,
			       const struct switch_attr *attr,
			       struct switch_val *val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);

	if (attr->ofs > 2)
		return -EINVAL;

	if (attr->ofs == 1)
		val->value.i = smi->vlan_enabled;
	else
		val->value.i = smi->vlan4k_enabled;

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_sw_get_vlan_enable);

int rtl8366_sw_set_vlan_enable(struct switch_dev *dev,
			       const struct switch_attr *attr,
			       struct switch_val *val)
{
	struct rtl8366_smi *smi = sw_to_rtl8366_smi(dev);
	int err;

	if (attr->ofs > 2)
		return -EINVAL;

	if (attr->ofs == 1)
		err = rtl8366_enable_vlan(smi, val->value.i);
	else
		err = rtl8366_enable_vlan4k(smi, val->value.i);

	return err;
}
EXPORT_SYMBOL_GPL(rtl8366_sw_set_vlan_enable);

struct rtl8366_smi *rtl8366_smi_alloc(struct device *parent)
{
	struct rtl8366_smi *smi;

	BUG_ON(!parent);

	smi = kzalloc(sizeof(*smi), GFP_KERNEL);
	if (!smi) {
		dev_err(parent, "no memory for private data\n");
		return NULL;
	}

	smi->parent = parent;
	return smi;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_alloc);

static int __rtl8366_smi_init(struct rtl8366_smi *smi, const char *name)
{
	spin_lock_init(&smi->lock);

	/* start the switch */
	if (smi->hw_reset) {
		smi->hw_reset(false);
		msleep(RTL8366_SMI_HW_START_DELAY);
	}

	return 0;
}

static void __rtl8366_smi_cleanup(struct rtl8366_smi *smi)
{
	if (smi->hw_reset)
		smi->hw_reset(true);
}

int rtl8366_smi_init(struct rtl8366_smi *smi)
{
	int err;

	if (!smi->ops)
		return -EINVAL;

	err = __rtl8366_smi_init(smi, dev_name(smi->parent));
	if (err)
		goto err_out;

	dev_info(smi->parent, "using MDIO interface\n");

	err = smi->ops->detect(smi);
	if (err) {
		dev_err(smi->parent, "chip detection failed, err=%d\n", err);
		goto err_free_sck;
	}

	err = rtl8366_reset(smi);
	if (err)
		goto err_free_sck;

	err = smi->ops->setup(smi);
	if (err) {
		dev_err(smi->parent, "chip setup failed, err=%d\n", err);
		goto err_free_sck;
	}

	err = rtl8366_init_vlan(smi);
	if (err) {
		dev_err(smi->parent, "VLAN initialization failed, err=%d\n",
			err);
		goto err_free_sck;
	}

	err = rtl8366_enable_all_ports(smi, 1);
	if (err)
		goto err_free_sck;

	err = rtl8366_smi_mii_init(smi);
	if (err)
		goto err_free_sck;

	return 0;

 err_free_sck:
	__rtl8366_smi_cleanup(smi);
 err_out:
	return err;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_init);

void rtl8366_smi_cleanup(struct rtl8366_smi *smi)
{
	rtl8366_smi_mii_cleanup(smi);
	__rtl8366_smi_cleanup(smi);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_cleanup);


struct rtl8366_smi *rtl8366_smi_probe(struct platform_device *pdev)
{
	struct rtl8366_smi *smi;
	smi = rtl8366_smi_alloc(&pdev->dev);
	return smi;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_probe);

MODULE_DESCRIPTION("Realtek RTL8367 MDIO interface driver");
MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org>");
MODULE_LICENSE("GPL v2");
