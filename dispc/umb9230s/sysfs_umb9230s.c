// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "disp_lib.h"
#include "umb9230s.h"

struct umb9230_sysfs {
	int hop_freq;
	int ssc_en;
	u32 base_offset[2];
};

static struct umb9230_sysfs *sysfs;

static ssize_t regs_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE,
			"reg : %x\n"
			"reg length: %x\n",
			sysfs->base_offset[0],
			sysfs->base_offset[1]);

	return ret;
}

static ssize_t regs_offset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 input_param[2];

	str_to_u32_array(buf, 16, input_param, 2);

	if ((input_param[0] < 0xB000 || input_param[0] > 0xBF00) && input_param[0] % 4)
		pr_info("input_param[0] is not a multiple of 4!\n");

	sysfs->base_offset[0] = input_param[0];
	sysfs->base_offset[1] = input_param[1];

	pr_info("addr:0x%08x length: 0x%lx\n", input_param[0], input_param[1]);

	return count;
}
static DEVICE_ATTR_RW(regs_offset);

static ssize_t wr_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct umb9230s_device *umb9230s = dev_get_drvdata(dev);
	u32 addr = sysfs->base_offset[0];
	u32 length = sysfs->base_offset[1];
	int ret = 0;
	int i;
	u32 reg;
	int reg_width = 4;

	mutex_lock(&umb9230s->lock);
	if (!umb9230s->enabled) {
		mutex_unlock(&umb9230s->lock);
		pr_err("umb9230s is not initialized\n");
		return -EINVAL;
	}

	if (addr >= 0xB000 && addr <= 0xBF00)
		reg_width = 1;

	for (i = 0; i < length; i++) {
	        iic2cmd_read(umb9230s->i2c_addr, (addr + i * reg_width), &reg, 1);
		ret += snprintf(buf + ret, PAGE_SIZE, "%x ", reg);
	}

	mutex_unlock(&umb9230s->lock);

	return ret;
}

static ssize_t wr_regs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct umb9230s_device *umb9230s = dev_get_drvdata(dev);
	u32 addr = sysfs->base_offset[0];
	u32 length = sysfs->base_offset[1];
	u32 *value;
	u32 i, actual_len;
	u32 reg[2];

	mutex_lock(&umb9230s->lock);
	if (!umb9230s->enabled) {
		mutex_unlock(&umb9230s->lock);
		pr_err("umb9230s is not initialized\n");
		return -EINVAL;
	}

	value = kzalloc(length * 4 + 4, GFP_KERNEL);
	if (!value) {
		mutex_unlock(&umb9230s->lock);
		pr_err("kzalloc error!\n");
		return -ENOMEM;
	}

	actual_len = str_to_u32_array(buf, 16, &value[1], (u8)length);
	if (!actual_len) {
		kfree(value);
		mutex_unlock(&umb9230s->lock);
		pr_err("input format error\n");
		return -EINVAL;
	}

	if (addr >= 0xB000 && addr <= 0xBF00) {
		for (i = 0; i < actual_len; i++) {
			reg[0] = addr + i;
			reg[1] = value[1 + i];
			iic2cmd_write(umb9230s->i2c_addr, reg, 2);
		}
	} else {
		value[0] = addr;
		iic2cmd_write(umb9230s->i2c_addr, value, actual_len + 1);
	}

	kfree(value);
	mutex_unlock(&umb9230s->lock);

	return count;
}
static DEVICE_ATTR_RW(wr_regs);

static ssize_t ssc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", sysfs->ssc_en);

	return ret;
}

static ssize_t ssc_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct umb9230s_device *umb9230s = dev_get_drvdata(dev);
	const struct dphy_tx_pll_ops *pll = umb9230s->pll;
	struct dphy_tx_context *ctx = &umb9230s->phy_ctx;
	int ret;

	mutex_lock(&umb9230s->lock);
	if (!umb9230s->enabled) {
		mutex_unlock(&umb9230s->lock);
		pr_err("umb9230s is not initialized\n");
		return -ENXIO;
	}

	ret = kstrtoint(buf, 10, &sysfs->ssc_en);
	if (ret) {
		mutex_unlock(&umb9230s->lock);
		pr_err("Invalid input value\n");
		return -EINVAL;
	}

	if (pll->ssc_en)
		pll->ssc_en(ctx, sysfs->ssc_en);

	mutex_unlock(&umb9230s->lock);

	return count;
}
static DEVICE_ATTR_RW(ssc);

static ssize_t hop_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", sysfs->hop_freq);

	return ret;
}

static ssize_t hop_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct umb9230s_device *umb9230s = dev_get_drvdata(dev);
	struct dphy_tx_context *ctx = &umb9230s->phy_ctx;
	const struct dphy_tx_pll_ops *pll = umb9230s->pll;
	int ret;
	int delta;

	mutex_lock(&umb9230s->lock);
	if (!umb9230s->enabled) {
		mutex_unlock(&umb9230s->lock);
		pr_err("umb9230s is not initialized\n");
		return -ENXIO;
	}

	ret = kstrtoint(buf, 10, &sysfs->hop_freq);
	if (ret) {
		mutex_unlock(&umb9230s->lock);
		pr_err("Invalid input freq\n");
		return -EINVAL;
	}

	/*
	 * because of double edge trigger,
	 * the rule is actual freq * 10 / 2,
	 * Eg: Required freq is 500M
	 * Equation: 2500*2*1000/10=500*1000=2500*200=500M
	 */
	if (sysfs->hop_freq == 0)
		sysfs->hop_freq = ctx->freq;
	else
		sysfs->hop_freq *= 200;
	pr_info("input freq is %d\n", sysfs->hop_freq);

	delta = sysfs->hop_freq - ctx->freq;
	if (pll->hop_config)
		pll->hop_config(ctx, delta, 200);

	mutex_unlock(&umb9230s->lock);

	return count;
}
static DEVICE_ATTR_RW(hop);

static ssize_t dsc_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct umb9230s_device *umb9230s = dev_get_drvdata(dev);
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", umb9230s->dsc_en);

	return ret;
}

static ssize_t dsc_en_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct umb9230s_device *umb9230s = dev_get_drvdata(dev);
	int ret;
	int enable;

	mutex_lock(&umb9230s->lock);

	ret = kstrtoint(buf, 10, &enable);
	if (ret) {
		mutex_unlock(&umb9230s->lock);
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	umb9230s->dsc_en = enable;

	mutex_unlock(&umb9230s->lock);

	return count;
}
static DEVICE_ATTR_RW(dsc_en);

static struct attribute *umb9230s_attrs[] = {
	&dev_attr_regs_offset.attr,
	&dev_attr_wr_regs.attr,
	&dev_attr_ssc.attr,
	&dev_attr_hop.attr,
	&dev_attr_dsc_en.attr,
	NULL,
};
ATTRIBUTE_GROUPS(umb9230s);

int umb9230s_sysfs_init(struct device *dev)
{
	int rc;

	sysfs = kzalloc(sizeof(*sysfs), GFP_KERNEL);
	if (!sysfs) {
		pr_err("alloc umb9230s sysfs failed\n");
		return -ENOMEM;
	}
	rc = sysfs_create_groups(&dev->kobj, umb9230s_groups);
	if (rc)
		pr_err("create umb9230s attr node failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(umb9230s_sysfs_init);