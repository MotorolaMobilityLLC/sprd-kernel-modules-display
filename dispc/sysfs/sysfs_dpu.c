// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#include "disp_lib.h"
#include "sprd_dpu.h"
#include "sprd_dsi_panel.h"
#include "sprd_dsi.h"
#include "sysfs_display.h"
#include "../dsi/sprd_dsi_api.h"

static uint32_t max_reg_length;

struct dpu_sysfs {
	u32 bg_color;
};

static struct dpu_sysfs *sysfs;

static ssize_t run_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", !dpu->ctx.stopped);

	return ret;
}

static ssize_t run_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	int ret;
	int enable;

	ret = kstrtoint(buf, 10, &enable);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (enable)
		sprd_dpu_run(dpu);
	else
		sprd_dpu_stop(dpu);

	return count;
}
static DEVICE_ATTR_RW(run);

static ssize_t refresh_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct sprd_panel *panel = to_sprd_panel(dpu->dsi->panel);
	struct sprd_crtc *crtc = dpu->crtc;
	struct dpu_context *ctx = &dpu->ctx;
	bool crtc_active_state;

	down(&ctx->lock);

	pr_info("[drm] %s()\n", __func__);

	if ((!ctx->enabled) || (!panel->enabled)) {
		pr_err("dpu or panel is powered off\n");
		up(&ctx->lock);
		return -1;
	}

	crtc_active_state = sprd_check_crtc_active_state(crtc->base.dev, 0);
	if (!crtc_active_state) {
		pr_err("display system has powered off, skip refresh operation\n");
		up(&ctx->lock);
		return -1;
	}

	ctx->flip_pending = false;

	dpu->core->flip(ctx, crtc->planes, crtc->pending_planes);

	up(&ctx->lock);

	return count;
}
static DEVICE_ATTR_WO(refresh);

static ssize_t bg_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%x\n", sysfs->bg_color);

	return ret;
}

static ssize_t bg_color_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct sprd_crtc *crtc = dpu->crtc;
	struct sprd_panel *panel = to_sprd_panel(dpu->dsi->panel);
	struct dpu_context *ctx = &dpu->ctx;
	bool crtc_active_state;
	int ret;

	if (!dpu->core->bg_color)
		return -EIO;

	pr_info("[drm] %s()\n", __func__);

	ret = kstrtou32(buf, 16, &sysfs->bg_color);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	down(&ctx->lock);

	if ((!ctx->enabled) || (!panel->enabled)) {
		pr_err("dpu or panel is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}

	crtc_active_state = sprd_check_crtc_active_state(crtc->base.dev, 0);
	if (!crtc_active_state) {
		pr_err("display system has powered off, skip refresh operation\n");
		up(&ctx->lock);
		return -1;
	}

	ctx->flip_pending = true;
	dpu->core->bg_color(ctx, sysfs->bg_color);

	up(&ctx->lock);

	return count;
}
static DEVICE_ATTR_RW(bg_color);

static ssize_t dpu_rst_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);

	DRM_INFO("enter dpu_rst_store\n");

	sprd_dpu_dsc_reset(dpu);

	return count;
}
static DEVICE_ATTR_WO(dpu_rst);

static ssize_t te_int_gap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", ctx->te_int_max_gap);
	pr_info("te int max gap is %d\n", ctx->te_int_max_gap);

	return ret;
}

static ssize_t te_int_gap_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	int ret;

	pr_info("[drm] %s()\n", __func__);

	ret = kstrtou32(buf, 10, &ctx->te_int_max_gap);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}
	pr_info("set te int max gap to %d\n", ctx->te_int_max_gap);

	return count;
}
static DEVICE_ATTR_RW(te_int_gap);

static ssize_t max_vsync_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	int ret;

	if (!dpu->ctx.enabled) {
		pr_err("dpu is not initialized\n");
		return -EINVAL;
	}

	ret = snprintf(buf, PAGE_SIZE, "%x\n", ctx->max_vsync_count);

	return ret;
}

static ssize_t max_vsync_count_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	int ret, max_vsync_count;


	pr_info("[drm] %s()\n", __func__);

	if (!dpu->ctx.enabled) {
		pr_err("dpu is not initialized\n");
		return -EINVAL;
	}

	ret = kstrtou32(buf, 16, &max_vsync_count);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	ctx->max_vsync_count = 0;

	return count;
}
static DEVICE_ATTR_RW(max_vsync_count);

static ssize_t disable_flip_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	pr_info("[drm] %s()\n", __func__);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", ctx->flip_pending);

	return ret;
}
static DEVICE_ATTR_RO(disable_flip);

static ssize_t enhance_mode_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_MODE, buf, count);

	return count;
}
static BIN_ATTR_WO(enhance_mode, 4);

static ssize_t cabc_hist_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->cabc_lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->cabc_lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_CABC_HIST, buf, count);
	up(&ctx->cabc_lock);

	return count;
}
static BIN_ATTR_RO(cabc_hist, 128);

static ssize_t cabc_hist_v2_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->cabc_lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->cabc_lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_CABC_HIST_V2, buf, count);
	up(&ctx->cabc_lock);

	return count;
}
static BIN_ATTR_RO(cabc_hist_v2, 256);

static ssize_t cabc_cur_bl_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_CABC_CUR_BL, buf, count);

	return count;

}

static BIN_ATTR_RO(cabc_cur_bl, 4);

static ssize_t vsync_count_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_VSYNC_COUNT, buf, count);
	up(&ctx->lock);

	return count;
}

static BIN_ATTR_RO(vsync_count, 4);

static ssize_t frame_no_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu	 *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_FRAME_NO, buf, count);

	return count;
}

static BIN_ATTR_RO(frame_no, 4);

static ssize_t cabc_param_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_CABC_PARAM, buf, count);

	return count;
}

static BIN_ATTR_WO(cabc_param, 292);

static ssize_t cabc_run_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_CABC_RUN, buf, count);

	return count;
}

static BIN_ATTR_WO(cabc_run, 4);

static ssize_t cabc_state_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->cabc_lock);
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_CABC_STATE, buf, count);
	up(&ctx->cabc_lock);

	return count;
}

static ssize_t cabc_state_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	if (off >= attr->size)
		return 0;

	if (off + count > attr->size)
		count = attr->size - off;

	down(&ctx->cabc_lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_CABC_STATE, buf, count);
	up(&ctx->cabc_lock);

	return count;
}

static BIN_ATTR_RW(cabc_state, 8);

#ifdef CONFIG_ARM64
static ssize_t actual_fps_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct time_fifo *tf = &dpu->ctx.tf;
	int ret;

	ret = kstrtou32(buf, 10, &tf->sum_num);
	if (ret) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (tf->sum_num > sizeof(tf->ts) / sizeof(struct timespec64) -1)
		tf->sum_num = sizeof(tf->ts) / sizeof(struct timespec64) -1;

	pr_info("[drm] %s() num:%d\n", __func__, tf->sum_num);

	return count;
}

static ssize_t actual_fps_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct time_fifo *tf = &dpu->ctx.tf;
	static u64 last_frame_count;
	u64 temp, average = 0;
	int i, j, len, head, cnt;

	down(&dpu->ctx.lock);
	if (last_frame_count == dpu->ctx.frame_count) {
		up(&dpu->ctx.lock);
		return snprintf(buf, PAGE_SIZE, "0.0\n");
	}

	last_frame_count = dpu->ctx.frame_count;
	len = sizeof(tf->ts) / sizeof(struct timespec64);
	head = tf->head;

	if (tf->sum_num == 0) {
		i = head - 1 < 0 ? len + head - 1 : head - 1;
		j = head - 2 < 0 ? len + head - 2 : head - 2;
		temp = (tf->ts[i].tv_sec - tf->ts[j].tv_sec) * 1000000LL
			+ (tf->ts[i].tv_nsec - tf->ts[j].tv_nsec) / 1000;
		up(&dpu->ctx.lock);
		return snprintf(buf, PAGE_SIZE, "%u.%u\n",
			1000000LL / temp, 100000000LL / temp % 100);
	} else {
		for (cnt = tf->sum_num; cnt > 0; --cnt) {
			i = head - cnt < 0 ? len + head - cnt : head - cnt;
			j = head - cnt -1 < 0 ? len + head - cnt -1 : head - cnt -1;
			temp = (tf->ts[i].tv_sec - tf->ts[j].tv_sec) * 1000000LL
				+ (tf->ts[i].tv_nsec - tf->ts[j].tv_nsec) / 1000;
			if (temp)
				average += 100000000LL / temp / tf->sum_num;
		}
	}

	up(&dpu->ctx.lock);
	return snprintf(buf, PAGE_SIZE, "avg:%u.%u\n", average / 100, average % 100);
}
static DEVICE_ATTR_RW(actual_fps);
#endif

static ssize_t regs_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	int ret;

	ret = snprintf(buf, PAGE_SIZE,
			"dpu reg offset: %x\n"
			"dpu reg length: %x\n",
			ctx->base_offset[0],
			ctx->base_offset[1]);

	return ret;
}

static ssize_t regs_offset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	u32 input_param[2];

	str_to_u32_array(buf, 16, input_param, 2);
	if ((input_param[0] + input_param[1]) > dpu->ctx.base_offset[1]) {
		pr_err("set reg off set over dpu register limit size\n");
		return -EINVAL;
	}

	if (input_param[0] % 4) {
		pr_err("input_param[0] is not a multiple of 4\n");
	} else {
		if ((input_param[1] > max_reg_length) || (input_param[1] <= 0))
			pr_err("input_param[1] should between 0 and %d\n", max_reg_length);
		else {
			dpu->ctx.base_offset[0] = input_param[0];
			dpu->ctx.base_offset[1] = input_param[1];
		}
	}

	return count;
}
static DEVICE_ATTR_RW(regs_offset);

static ssize_t wr_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	u32 offset = ctx->base_offset[0];
	u32 length = ctx->base_offset[1];
	int ret = 0;
	int i;
	u32 reg;

	down(&dpu->ctx.lock);
	if (!dpu->ctx.enabled) {
		pr_err("dpu is not initialized\n");
		up(&dpu->ctx.lock);
		return -EINVAL;
	}

	for (i = 0; i < length; i++) {
		reg = readl((void __iomem *)(ctx->base + offset));
		ret += snprintf(buf + ret, PAGE_SIZE, "%x ", reg);
	}

	up(&dpu->ctx.lock);

	return ret;
}

static ssize_t wr_regs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	u32 offset = ctx->base_offset[0];
	u32 length = ctx->base_offset[1];
	u32 *value;
	u32 i, actual_len;

	down(&dpu->ctx.lock);
	if (!dpu->ctx.enabled) {
		pr_err("dpu is not initialized\n");
		up(&dpu->ctx.lock);
		return -EINVAL;
	}

	value = kzalloc(length * 4, GFP_KERNEL);
	if (!value) {
		up(&dpu->ctx.lock);
		return -ENOMEM;
	}

	actual_len = str_to_u32_array(buf, 16, value, (u8)length);
	if (!actual_len) {
		pr_err("input format error\n");
		up(&dpu->ctx.lock);
		return -EINVAL;
	}

	for (i = 0; i < actual_len; i++) {
		writel(value[i], (void __iomem *)(ctx->base + offset));
		offset += 0x04;
	}

	kfree(value);
	up(&dpu->ctx.lock);

	return count;
}
static DEVICE_ATTR_RW(wr_regs);

static ssize_t dpu_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%s\n", ctx->version);

	return ret;
}
static DEVICE_ATTR_RO(dpu_version);

#ifdef CONFIG_DRM_SPRD_WB_DEBUG
static ssize_t wb_debug_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	void *vaddr = NULL;
	char filename[128];
	struct timespec64 ts;
	struct rtc_time tm;

	if (!dpu->ctx.enabled) {
		pr_err("dpu is not initialized\n");
		return -EINVAL;
	}

	if (dpu->core && dpu->core->write_back) {
		dpu->core->write_back(ctx, 1, true);
		vaddr = __va(ctx->wb_addr_p);
	} else
		return -ENXIO;

	/* FIXME: wait for writeback done isr */
	mdelay(50);

	ktime_get_real_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec, &tm);
	sprintf(filename, "/data/dump/wb_%d-%d-%dT%d%d%d.bmp",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
			tm.tm_min, tm.tm_sec);

	dump_bmp32(vaddr, ctx->vm.hactive, ctx->vm.vactive, true, filename);

	return count;
}
static DEVICE_ATTR_WO(wb_debug);
#endif

static ssize_t irq_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	u32 value;
	int ret;

	if (kstrtou32(buf, 10, &value)) {
		pr_err("Invalid input for irq_register\n");
		return -EINVAL;
	}

	if (value > 0 && ctx->irq) {
		down(&ctx->lock);
		if (!ctx->enabled) {
			pr_err("dpu is not initialized!\n");
			up(&ctx->lock);
			return -EINVAL;
		}

		if (dpu->core->enable_vsync)
			dpu->core->enable_vsync(ctx);

		ret = devm_request_irq(&dpu->dev, ctx->irq, ctx->dpu_isr,
			0, "DISPC", dpu);
		if (ret) {
			up(&ctx->lock);
			pr_err("error: dpu request irq failed\n");
			return ret;
		}

		/*
		 * We request dpu isr on sprd crtc driver and set the IRQ_NOAUTOEN flag,
		 * so if not clear this flag, need to call "enable_irq" enable it.
		 */
		enable_irq(ctx->irq);
		up(&ctx->lock);
	}

	return count;
}
static DEVICE_ATTR_WO(irq_register);

static ssize_t irq_unregister_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	u32 value;

	if (kstrtou32(buf, 10, &value)) {
		pr_err("Invalid input for irq_unregister\n");
		return -EINVAL;
	}

	if (value > 0 && ctx->irq) {
		down(&ctx->lock);
		if (!ctx->enabled) {
			pr_err("dpu is not initialized!\n");
			up(&ctx->lock);
			return -EINVAL;
		}

		if (dpu->core->disable_vsync)
			dpu->core->disable_vsync(ctx);

		/*
		 * We request dpu isr on sprd crtc driver and set the IRQ_NOAUTOEN flag,
		 * so if not clear this flag, need to call "disable_irq" disable it.
		 */
		disable_irq(ctx->irq);
		devm_free_irq(&dpu->dev, ctx->irq, dpu);
		up(&ctx->lock);
	}

	return count;
}

static DEVICE_ATTR_WO(irq_unregister);

/* frame count show */
static ssize_t frame_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct sprd_dpu *dpu = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE, "%lld\n", dpu->ctx.frame_count);

	return ret;
}
static DEVICE_ATTR_RO(frame_count);

static struct attribute *dpu_attrs[] = {
	&dev_attr_run.attr,
	&dev_attr_refresh.attr,
	&dev_attr_bg_color.attr,
	&dev_attr_te_int_gap.attr,
	&dev_attr_disable_flip.attr,
#ifdef CONFIG_ARM64
	&dev_attr_actual_fps.attr,
#endif
	&dev_attr_regs_offset.attr,
	&dev_attr_wr_regs.attr,
	&dev_attr_dpu_version.attr,
	&dev_attr_max_vsync_count.attr,
#ifdef CONFIG_DRM_SPRD_WB_DEBUG
	&dev_attr_wb_debug.attr,
#endif
	&dev_attr_irq_register.attr,
	&dev_attr_irq_unregister.attr,
	&dev_attr_frame_count.attr,
	&dev_attr_dpu_rst.attr,
	NULL,
};
static const struct attribute_group dpu_group = {
	.attrs = dpu_attrs,
};

static ssize_t ltm_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_LTM, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t ltm_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	if (!strcmp("dpu-r6p0", ctx->version) || !strcmp("dpu-r6p1", ctx->version)) {
		down(&ctx->cabc_lock);
		dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_LTM, buf, count);
		up(&ctx->cabc_lock);
	} else {
		down(&ctx->lock);
		dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_LTM, buf, count);
		up(&ctx->lock);
	}

	return count;
}
static BIN_ATTR_RW(ltm, 48);

static ssize_t gamma_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_GAMMA, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t gamma_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_GAMMA, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(gamma, 1536);

static ssize_t slp_lut_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	u32 data[256];
	int ret = 0;
	int i;

	if (!dpu->core->enhance_get)
		return -EIO;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}

	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_SLP_LUT, data, sizeof(data));
	up(&ctx->lock);

	for (i = 0; i < 256; i++)
		ret += snprintf(buf + ret, PAGE_SIZE,
			"0x%x: 0x%x\n",
			i, data[i]);

	return ret;
}
static DEVICE_ATTR_RO(slp_lut);

static ssize_t slp_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_SLP, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t slp_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_SLP, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(slp, 48);

static ssize_t cm_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_CM, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t cm_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	if (!strcmp("dpu-r6p0", ctx->version) || !strcmp("dpu-r6p1", ctx->version)) {
		down(&ctx->cabc_lock);
		dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_CM, buf, count);
		up(&ctx->cabc_lock);
	} else {
		down(&ctx->lock);
		dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_CM, buf, count);
		up(&ctx->lock);
	}

	return count;
}
static BIN_ATTR_RW(cm, 24);

static ssize_t epf_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_EPF, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t epf_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_EPF, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(epf, 14);

static ssize_t ud_read(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_UD, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t ud_write(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_UD, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(ud, 12);

static ssize_t hsv_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_HSV, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t hsv_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_HSV, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(hsv, 1440);


static ssize_t scl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	u32 param[2] = {};

	if (!dpu->core->enhance_get)
		return -EIO;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_SCL, param, sizeof(param));
	up(&ctx->lock);

	ret = snprintf(buf, PAGE_SIZE, "%d x %d\n", param[0], param[1]);

	return ret;
}

static ssize_t scl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;
	u32 param[2] = {};

	if (!dpu->core->enhance_set)
		return -EIO;

	down(&ctx->lock);
	str_to_u32_array(buf, 10, param, 2);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_SCL, param, sizeof(param));
	up(&ctx->lock);

	return count;
}
static DEVICE_ATTR_RW(scl);

static ssize_t lut3d_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_LUT3D, buf, count);
	up(&ctx->lock);

	return count;
}

static ssize_t lut3d_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_LUT3D, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(lut3d, 2916);

static ssize_t enable_read(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}
	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_ENABLE, buf, count);

	up(&ctx->lock);

	return count;
}

static ssize_t enable_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_ENABLE, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_RW(enable, 4);

static ssize_t disable_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_DISABLE, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_WO(disable, 4);

static ssize_t luts_print_write(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_get)
		return -EIO;

	/* I need to get my data in one piece */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	if (!ctx->enabled) {
		pr_err("dpu is not initialized\n");
		up(&ctx->lock);
		return -EINVAL;
	}

	dpu->core->enhance_get(ctx, ENHANCE_CFG_ID_UPDATE_LUTS, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_WO(luts_print, 4);

static ssize_t update_luts_write(struct file *fp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct dpu_context *ctx = &dpu->ctx;

	if (!dpu->core->enhance_set)
		return -EIO;

	/* I need to get my data in one piece
	 * count: header + payload, header 4 bytes:
	 * header:type + index , type: enhance_type,
	 * index: the choosed luts table
	 */
	if (off != 0)
		return -EINVAL;

	down(&ctx->lock);
	dpu->core->enhance_set(ctx, ENHANCE_CFG_ID_UPDATE_LUTS, buf, count);
	up(&ctx->lock);

	return count;
}
static BIN_ATTR_WO(update_luts, 2052);

static struct attribute *pq_ascii_attrs[] = {
	&dev_attr_scl.attr,
	&dev_attr_slp_lut.attr,
	NULL,
};
static struct bin_attribute *pq_bin_attrs[] = {
	&bin_attr_ltm,
	&bin_attr_gamma,
	&bin_attr_slp,
	&bin_attr_cm,
	&bin_attr_hsv,
	&bin_attr_epf,
	&bin_attr_enhance_mode,
	&bin_attr_cabc_hist,
	&bin_attr_cabc_hist_v2,
	&bin_attr_cabc_param,
	&bin_attr_vsync_count,
	&bin_attr_frame_no,
	&bin_attr_cabc_run,
	&bin_attr_cabc_cur_bl,
	&bin_attr_cabc_state,
	&bin_attr_lut3d,
	&bin_attr_enable,
	&bin_attr_disable,
	&bin_attr_ud,
	&bin_attr_luts_print,
	&bin_attr_update_luts,
	NULL,
};
static const struct attribute_group pq_group = {
	.name = "PQ",
	.attrs = pq_ascii_attrs,
	.bin_attrs = pq_bin_attrs,
};

int sprd_dpu_sysfs_init(struct device *dev)
{
	int rc;
	struct sprd_dpu *dpu = dev_get_drvdata(dev);

	max_reg_length = dpu->ctx.base_offset[1];

	sysfs = kzalloc(sizeof(*sysfs), GFP_KERNEL);
	if (!sysfs) {
		pr_err("alloc dpu sysfs failed\n");
		return -ENOMEM;
	}
	rc = sysfs_create_group(&(dev->kobj), &dpu_group);
	if (rc) {
		pr_err("create dpu attr node failed, rc=%d\n", rc);
		goto free_sysfs;
	}

	if (!strcmp(dev->kobj.name, "dispc0")) {
		rc = sysfs_create_group(&(dev->kobj), &pq_group);
		if (rc) {
			pr_err("create dpu PQ node failed, rc=%d\n", rc);
			goto remove_dpu_group;
		}
	}

	return rc;

remove_dpu_group:
	sysfs_remove_group(&(dev->kobj), &dpu_group);

free_sysfs:
	kfree(sysfs);
	sysfs = NULL;

	return rc;
}
EXPORT_SYMBOL(sprd_dpu_sysfs_init);

void sprd_dpu_sysfs_deinit(struct device *dev)
{
	if (!strncmp(dev->kobj.name, "dispc0", strlen("dispc0")))
		sysfs_remove_group(&(dev->kobj), &pq_group);

	sysfs_remove_group(&(dev->kobj), &dpu_group);

	kfree(sysfs);
	sysfs = NULL;
}
EXPORT_SYMBOL(sprd_dpu_sysfs_deinit);

MODULE_AUTHOR("Leon He <leon.he@unisoc.com>");
MODULE_DESCRIPTION("Add dpu attribute nodes for userspace");
MODULE_LICENSE("GPL v2");
