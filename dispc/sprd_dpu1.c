// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <linux/component.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/sprd_iommu.h>
#include <linux/timer.h>
#include <linux/memblock.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_vblank.h>

#include "sprd_crtc.h"
#include "sprd_dpu1.h"
#include "sprd_drm.h"
#include "sprd_gem.h"
#include "sprd_plane.h"
#include "sysfs/sysfs_display.h"

static void sprd_dpu_enable(struct sprd_dpu *dpu);
static void sprd_dpu_disable(struct sprd_dpu *dpu);

static void dpu1_int_cnt_timer_callback(struct timer_list *timer)
{
	struct dpu_context *ctx = container_of(timer, struct dpu_context, dpu1_int_cnt_timer);
	struct sprd_dpu *dpu = container_of(ctx, struct sprd_dpu, ctx);
	struct dpu_int_cnt *cnt = &ctx->int_cnt;

	if (cnt->int_cnt_all > DPU1_INT_MAX_CNT) {
		DRM_INFO("dpu1 int cnt over threshold value!\n");
		DRM_INFO("dpu1: int_cnt_all------------------%d\n", cnt->int_cnt_all);
		DRM_INFO("dpu1: int_cnt_vsync----------------%d\n", cnt->int_cnt_vsync);
		DRM_INFO("dpu1: int_cnt_te-------------------%d\n", cnt->int_cnt_te);
		DRM_INFO("dpu1: int_cnt_lay_reg_update_done--%d\n", cnt->int_cnt_lay_reg_update_done);
		DRM_INFO("dpu1: int_cnt_dpu_reg_update_done--%d\n", cnt->int_cnt_dpu_reg_update_done);
		DRM_INFO("dpu1: int_cnt_dpu_all_update_done--%d\n", cnt->int_cnt_dpu_all_update_done);
		DRM_INFO("dpu1: int_cnt_pq_reg_update_done---%d\n", cnt->int_cnt_pq_reg_update_done);
		DRM_INFO("dpu1: int_cnt_pq_lut_update_done---%d\n", cnt->int_cnt_pq_lut_update_done);
		DRM_INFO("dpu1: int_cnt_dpu_int_done---------%d\n", cnt->int_cnt_dpu_int_done);
		DRM_INFO("dpu1: int_cnt_dpu_int_err----------%d\n", cnt->int_cnt_dpu_int_err);
		DRM_INFO("dpu1: int_cnt_dpu_int_wb_done------%d\n", cnt->int_cnt_dpu_int_wb_done);
		DRM_INFO("dpu1: int_cnt_dpu_int_wb_err-------%d\n", cnt->int_cnt_dpu_int_wb_err);
		DRM_INFO("dpu1: int_cnt_dpu_int_fbc_pld_err--%d\n", cnt->int_cnt_dpu_int_fbc_pld_err);
		DRM_INFO("dpu1: int_cnt_dpu_int_fbc_hdr_err--%d\n", cnt->int_cnt_dpu_int_fbc_hdr_err);
		DRM_INFO("dpu1: int_cnt_dpu_int_mmu----------%d\n", cnt->int_cnt_dpu_int_mmu);

		if (dpu->core->reg_dump)
			dpu->core->reg_dump(ctx);
	}

	memset(cnt, 0, sizeof(struct dpu_int_cnt));

	mod_timer(&ctx->dpu1_int_cnt_timer, jiffies + msecs_to_jiffies(1000));
}

static void int_cnt_timer_init(struct dpu_context *ctx)
{
	timer_setup(&ctx->dpu1_int_cnt_timer, dpu1_int_cnt_timer_callback, 0);
	ctx->dpu1_int_cnt_timer.expires = jiffies + msecs_to_jiffies(1000);
	add_timer(&ctx->dpu1_int_cnt_timer);
}

static void int_cnt_timer_exit(struct dpu_context *ctx)
{
	del_timer_sync(&ctx->dpu1_int_cnt_timer);
	memset(&ctx->int_cnt, 0, sizeof(struct dpu_int_cnt));
}

static int sprd_dpu_prepare_fb(struct sprd_crtc *crtc,
				struct drm_plane_state *new_state)
{
	struct drm_gem_object *obj;
	struct sprd_gem_obj *sprd_gem;
	struct sprd_dpu *dpu = crtc->priv;
	int i, ret = 0;

	if (!dpu->ctx.enabled) {
		DRM_WARN("dpu has already powered off\n");
		return 0;
	}

	for (i = 0; i < new_state->fb->format->num_planes; i++) {
		obj = drm_gem_fb_get_obj(new_state->fb, i);
		sprd_gem = to_sprd_gem_obj(obj);
		ret = sprd_crtc_iommu_map(&dpu->dev, sprd_gem);
		if (ret)
			break;
	}

	return ret;
}

static void sprd_dpu_cleanup_fb(struct sprd_crtc *crtc,
				struct drm_plane_state *old_state)
{
	struct drm_gem_object *obj;
	struct sprd_gem_obj *sprd_gem;
	struct sprd_dpu *dpu = crtc->priv;
	int i;

	if (!dpu->ctx.enabled) {
		DRM_WARN("dpu has already powered off\n");
		return;
	}

	for (i = 0; i < old_state->fb->format->num_planes; i++) {
		obj = drm_gem_fb_get_obj(old_state->fb, i);
		sprd_gem = to_sprd_gem_obj(obj);
		sprd_crtc_iommu_unmap(&dpu->dev, sprd_gem);
	}
}

static void sprd_dpu_mode_set_nofb(struct sprd_crtc *crtc)
{
	struct sprd_dpu *dpu = crtc->priv;
	struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;

	DRM_INFO("%s() set mode: %s\n", __func__, mode->name);

	if (dpu->core->modeset && crtc->base.state->mode_changed)
		dpu->core->modeset(&dpu->ctx, mode);
}

static enum drm_mode_status sprd_dpu_mode_valid(struct sprd_crtc *crtc,
					const struct drm_display_mode *mode)
{
	struct sprd_dpu *dpu = crtc->priv;
	int vic;

	DRM_INFO("%s() mode: "DRM_MODE_FMT"\n", __func__, DRM_MODE_ARG(mode));

	vic = drm_match_cea_mode(mode);

	/* 1920x1080@60Hz is used by default */
	if (vic == 16 && mode->clock == 148500) {
		DRM_INFO("%s() mode: "DRM_MODE_FMT"\n",
				__func__, DRM_MODE_ARG(mode));
		drm_display_mode_to_videomode(mode, &dpu->ctx.vm);
	}

	return MODE_OK;
}

static void sprd_dpu_atomic_enable(struct sprd_crtc *crtc)
{
	struct sprd_dpu *dpu = crtc->priv;

	DRM_INFO("%s()\n", __func__);

	pm_runtime_get_sync(dpu->dev.parent);

	sprd_dpu_enable(dpu);

	enable_irq(dpu->ctx.irq);

	sprd_iommu_restore(&dpu->dev);
}

static void sprd_dpu_atomic_disable(struct sprd_crtc *crtc)
{
	struct sprd_dpu *dpu = crtc->priv;

	DRM_INFO("%s()\n", __func__);

	sprd_crtc_wait_last_commit_complete(&crtc->base);

	disable_irq(dpu->ctx.irq);

	sprd_dpu_disable(dpu);

	pm_runtime_put(dpu->dev.parent);
}

static void sprd_dpu_atomic_begin(struct sprd_crtc *crtc)
{
	struct sprd_dpu *dpu = crtc->priv;

	DRM_DEBUG("%s()\n", __func__);

	down(&dpu->ctx.lock);

	crtc->pending_planes = 0;
}

static void sprd_dpu_atomic_flush(struct sprd_crtc *crtc)

{
	struct sprd_dpu *dpu = crtc->priv;

	DRM_DEBUG("%s()\n", __func__);

	if (crtc->pending_planes && !dpu->ctx.flip_pending) {
		dpu->core->flip(&dpu->ctx, crtc->planes, crtc->pending_planes);
		dpu->ctx.frame_count++;
	}
	up(&dpu->ctx.lock);
}

static int sprd_dpu_enable_vblank(struct sprd_crtc *crtc)
{
	struct sprd_dpu *dpu = crtc->priv;

	DRM_INFO("%s()\n", __func__);

	if (dpu->core->enable_vsync)
		dpu->core->enable_vsync(&dpu->ctx);

	return 0;
}

static int sprd_dpu_atomic_get_property(struct sprd_crtc *crtc,
					const struct drm_crtc_state *crtc_state,
					struct drm_property *property, uint64_t *val)
{
	struct sprd_dpu *dpu = crtc->priv;
	struct dpu_context *ctx = &dpu->ctx;

	if (property == crtc->blend_limit_property) {
		*val = ctx->default_layer_count_limit;
	} else if (property == crtc->vrr_enabled_property) {
		*val = false;
	} else {
		DRM_ERROR("property %s is invalid\n", property->name);
		return -EINVAL;
	}

	return 0;
}

static void sprd_dpu_disable_vblank(struct sprd_crtc *crtc)
{
	struct sprd_dpu *dpu = crtc->priv;

	DRM_INFO("%s()\n", __func__);

	if (dpu->core->disable_vsync)
		dpu->core->disable_vsync(&dpu->ctx);
}

static const struct sprd_crtc_ops sprd_dpu_ops = {
	.mode_set_nofb	= sprd_dpu_mode_set_nofb,
	.mode_valid	= sprd_dpu_mode_valid,
	.atomic_begin	= sprd_dpu_atomic_begin,
	.atomic_flush	= sprd_dpu_atomic_flush,
	.atomic_enable	= sprd_dpu_atomic_enable,
	.atomic_disable	= sprd_dpu_atomic_disable,
	.enable_vblank	= sprd_dpu_enable_vblank,
	.disable_vblank	= sprd_dpu_disable_vblank,
	.prepare_fb = sprd_dpu_prepare_fb,
	.cleanup_fb = sprd_dpu_cleanup_fb,
	.atomic_get_property = sprd_dpu_atomic_get_property,
};

void sprd_dpu1_run(struct sprd_dpu *dpu)
{
	struct dpu_context *ctx = &dpu->ctx;

	down(&ctx->lock);
	if (!ctx->enabled) {
		DRM_ERROR("dpu is not initialized\n");
		up(&ctx->lock);
		return;
	}

	if (!ctx->stopped) {
		up(&ctx->lock);
		return;
	}

	if (dpu->core->run)
		dpu->core->run(ctx);

	up(&ctx->lock);

	drm_crtc_vblank_on(&dpu->crtc->base);
}

void sprd_dpu1_stop(struct sprd_dpu *dpu)
{
	struct dpu_context *ctx = &dpu->ctx;

	down(&ctx->lock);

	if (!ctx->enabled) {
		DRM_ERROR("dpu is not initialized\n");
		up(&ctx->lock);
		return;
	}

	if (ctx->stopped) {
		up(&ctx->lock);
		return;
	}

	if (dpu->core->stop)
		dpu->core->stop(ctx);

	up(&ctx->lock);

	drm_crtc_handle_vblank(&dpu->crtc->base);
	drm_crtc_vblank_off(&dpu->crtc->base);
}

static void sprd_dpu_enable(struct sprd_dpu *dpu)
{
	struct dpu_context *ctx = &dpu->ctx;

	down(&ctx->lock);

	if (ctx->enabled) {
		up(&ctx->lock);
		return;
	}

	if (dpu->glb->power)
		dpu->glb->power(ctx, true);
	if (dpu->glb->enable)
		dpu->glb->enable(ctx);

	if (ctx->stopped && dpu->glb->reset)
		dpu->glb->reset(ctx);

	if (dpu->clk->init)
		dpu->clk->init(ctx);
	if (dpu->clk->enable)
		dpu->clk->enable(ctx);

	if (dpu->core->init)
		dpu->core->init(ctx);
	if (dpu->core->ifconfig)
		dpu->core->ifconfig(ctx);

	int_cnt_timer_init(ctx);

	ctx->enabled = true;

	up(&ctx->lock);
}

static void sprd_dpu_disable(struct sprd_dpu *dpu)
{
	struct dpu_context *ctx = &dpu->ctx;

	down(&ctx->lock);
	if (!ctx->enabled) {
		up(&ctx->lock);
		return;
	}

	int_cnt_timer_exit(ctx);

	if (dpu->core->fini)
		dpu->core->fini(ctx);
	if (dpu->clk->disable)
		dpu->clk->disable(ctx);
	if (dpu->glb->disable)
		dpu->glb->disable(ctx);
	if (dpu->glb->power)
		dpu->glb->power(ctx, false);

	ctx->enabled = false;
	up(&ctx->lock);
}

static irqreturn_t sprd_dpu_isr(int irq, void *data)
{
	struct sprd_dpu *dpu = data;
	struct dpu_context *ctx = &dpu->ctx;
	u32 int_mask = 0;

	int_mask = dpu->core->isr(ctx);

	if (int_mask & BIT_DPU_INT_ERR) {
		ctx->int_cnt.int_cnt_dpu_int_err++;
		DRM_WARN("Warning: dpu1 underflow!\n");
	}

	return IRQ_HANDLED;
}

static int sprd_dpu_irq_request(struct sprd_dpu *dpu)
{
	struct dpu_context *ctx = &dpu->ctx;
	int irq_num;
	int ret;

	irq_num = irq_of_parse_and_map(dpu->dev.of_node, 0);
	if (!irq_num) {
		DRM_ERROR("error: dpu parse irq num failed\n");
		return -EINVAL;
	}
	DRM_INFO("dpu irq_num = %d\n", irq_num);

	irq_set_status_flags(irq_num, IRQ_NOAUTOEN);
	ret = devm_request_irq(&dpu->dev, irq_num, sprd_dpu_isr,
					0, "DISPC1", dpu);
	if (ret) {
		DRM_ERROR("error: dpu request irq failed\n");
		return -EINVAL;
	}
	ctx->irq = irq_num;
	ctx->dpu_isr = sprd_dpu_isr;

	return 0;
}

static int sprd_dpu_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct sprd_dpu *dpu = dev_get_drvdata(dev);
	struct sprd_crtc_capability cap = {};
	struct sprd_plane *planes;
	struct dpu_context *ctx = &dpu->ctx;
	int ret;

	DRM_INFO("%s()\n", __func__);

	dpu->core->version(&dpu->ctx);
	dpu->core->capability(&dpu->ctx, &cap);

	ctx->default_layer_count_limit = cap.max_layers;
	planes = sprd_plane_init(drm, &cap, 0xff);
	if (IS_ERR_OR_NULL(planes))
		return PTR_ERR(planes);

	dpu->crtc = sprd_crtc_init(drm, planes, SPRD_DISPLAY_TYPE_DP,
				&sprd_dpu_ops, dpu->ctx.version, dpu->ctx.corner_size, "dispc1", dpu);
	if (IS_ERR(dpu->crtc))
		return PTR_ERR(dpu->crtc);

	ret = sprd_dpu_irq_request(dpu);
	if (ret)
		return -EINVAL;

	pm_runtime_enable(dev);

	return 0;
}

static void sprd_dpu_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct sprd_dpu *dpu = dev_get_drvdata(dev);

	DRM_INFO("%s()\n", __func__);

	drm_crtc_cleanup(&dpu->crtc->base);
}

static const struct component_ops dpu_component_ops = {
	.bind = sprd_dpu_bind,
	.unbind = sprd_dpu_unbind,
};

static int sprd_dpu_device_create(struct sprd_dpu *dpu,
				struct device *parent)
{
	int ret;

	dpu->dev.class = display_class;
	dpu->dev.parent = parent;
	dpu->dev.of_node = parent->of_node;
	dev_set_name(&dpu->dev, "dispc1");
	dev_set_drvdata(&dpu->dev, dpu);

	ret = device_register(&dpu->dev);
	if (ret) {
		DRM_ERROR("dpu device register failed\n");
		return ret;
	}

	return 0;
}

static int sprd_dpu_context_init(struct sprd_dpu *dpu,
				struct device *dev)
{
	struct resource r;
	struct dpu_context *ctx = &dpu->ctx;
	struct device_node *np = dev->of_node;
	int ret;

	if (dpu->core->context_init) {
		ret = dpu->core->context_init(ctx, dev);
		if (ret)
			return ret;
	}

	if (dpu->clk->parse_dt)
		dpu->clk->parse_dt(ctx, np);
	if (dpu->glb->parse_dt)
		dpu->glb->parse_dt(ctx, np);

	if (of_property_read_bool(np, "sprd,initial-stop-state")) {
		DRM_WARN("DPU is not initialized before entering kernel\n");
		dpu->ctx.stopped = true;
	}

	if (of_address_to_resource(np, 0, &r)) {
		DRM_ERROR("parse dt base address failed\n");
		return -ENODEV;
	}
	ctx->base = devm_ioremap(dev, r.start, resource_size(&r));
	if (!ctx->base) {
		DRM_ERROR("ioremap base address failed\n");
		return -EFAULT;
	}

	sema_init(&ctx->lock, 1);
	init_waitqueue_head(&ctx->wait_queue);

	ctx->panel_ready = true;
	ctx->time = 5000;

	return 0;
}

static const struct sprd_dpu_ops qogirn6pro1_dpu = {
	.core = &dpu_lite_r3p0_core_ops,
	.clk = &qogirn6pro_dpu1_clk_ops,
	.glb = &qogirn6pro_dpu1_glb_ops,
};

static const struct of_device_id dpu_match_table[] = {
	{ .compatible = "sprd,qogirn6pro-dpu1",
	  .data = &qogirn6pro1_dpu },
	{ /* sentinel */ },
};

static int sprd_dpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sprd_dpu_ops *pdata;
	struct sprd_dpu *dpu;
	int ret;

	dpu = devm_kzalloc(dev, sizeof(*dpu), GFP_KERNEL);
	if (!dpu)
		return -ENOMEM;

	pdata = of_device_get_match_data(dev);
	if (pdata) {
		dpu->core = pdata->core;
		dpu->clk = pdata->clk;
		dpu->glb = pdata->glb;
	} else {
		DRM_ERROR("No matching driver data found\n");
		return -EINVAL;
	}

	ret = sprd_dpu_context_init(dpu, dev);
	if (ret)
		return ret;

	ret = sprd_dpu_device_create(dpu, dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dpu);

	return component_add(dev, &dpu_component_ops);
}

static int sprd_dpu_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dpu_component_ops);
	return 0;
}

struct platform_driver sprd_dpu1_driver = {
	.probe = sprd_dpu_probe,
	.remove = sprd_dpu_remove,
	.driver = {
		.name = "sprd-dpu1-drv",
		.of_match_table = dpu_match_table,
	},
};

MODULE_AUTHOR("Leon He <leon.he@unisoc.com>");
MODULE_AUTHOR("Kevin Tang <kevin.tang@unisoc.com>");
MODULE_DESCRIPTION("Unisoc Display Controller Driver");
MODULE_LICENSE("GPL v2");
