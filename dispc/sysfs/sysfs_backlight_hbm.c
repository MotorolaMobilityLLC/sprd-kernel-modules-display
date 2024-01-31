/*
*SPDX-FileCopyrightText: 2020 Unisoc (Shanghai) Technologies Co.Ltd
*SPDX-License-Identifier: GPL-2.0-only
*/
#include <linux/backlight.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>

#include "sprd_bl.h"
extern bool g_hbm_enable;
extern int hbm_set_backlight_level(unsigned int level);
extern int hbm_exit_set_backlight_level(void);

static ssize_t hbm_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("hbm set buf:%s\n", buf);
#ifdef SMT_VERSION
	pr_info("SMT version,No hbm");
#else
	switch (buf[0]){
		case 'n':
		case 'N':
			if (!g_hbm_enable) {
				pr_info("Have been disabled hbm, exit!\n");
				break;
			}
			hbm_exit_set_backlight_level();
			break;
		case 'y':
		case 'Y':
			if (g_hbm_enable) {
				pr_info("Have been enabled hbm, exit!\n");
				break;
			}
			hbm_set_backlight_level(255);
			break;
		default:
			pr_info("pls echo Y/y/N/n open or off hbm!\n");
			break;
	}
#endif

	return count;
}

static ssize_t hbm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char hbm_str_st[8] = {0};
	if (g_hbm_enable){
	    strcpy(hbm_str_st, "hbm:on");
	}else{
	    strcpy(hbm_str_st, "hbm:off");
	}
	return sprintf(buf,"%s\n",hbm_str_st);
}
static DEVICE_ATTR_RW(hbm);

static struct attribute *backlight_attrs[] = {
	&dev_attr_hbm.attr,
	NULL,
};
ATTRIBUTE_GROUPS(backlight);

int sprd_backlight_hbm_sysfs_init(struct device *dev)
{
	int rc;

	rc = sysfs_create_groups(&dev->kobj, backlight_groups);
	if (rc)
		pr_err("create backlight hbm attr node failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(sprd_backlight_hbm_sysfs_init);

MODULE_AUTHOR("Gaofeng Sheng <gaofeng.sheng@unisoc.com>");
MODULE_DESCRIPTION("Add backlight attribute nodes for userspace");
MODULE_LICENSE("GPL v2");
