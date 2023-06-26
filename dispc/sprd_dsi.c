// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <linux/component.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mode.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>

#include "disp_lib.h"
#include "sprd_crtc.h"
#include "sprd_dpu.h"
#include "sprd_dsi.h"
#include "sprd_dsi_panel.h"
#include "dsi/sprd_dsi_api.h"
#include "dphy/sprd_dphy_api.h"
#include "sysfs/sysfs_display.h"

#define encoder_to_dsi(encoder) \
	container_of(encoder, struct sprd_dsi, encoder)
#define host_to_dsi(host) \
	container_of(host, struct sprd_dsi, host)
#define connector_to_dsi(connector) \
	container_of(connector, struct sprd_dsi, connector)

static void sprd_dsi_enable(struct sprd_dsi *dsi)
{
	if (dsi->glb->power)
		dsi->glb->power(&dsi->ctx, true);
	if (dsi->glb->enable)
		dsi->glb->enable(&dsi->ctx);
	if (dsi->glb->reset)
		dsi->glb->reset(&dsi->ctx);

	sprd_dsi_init(dsi);

	if (dsi->ctx.work_mode == DSI_MODE_VIDEO)
		sprd_dsi_dpi_video(dsi);
	else
		sprd_dsi_edpi_video(dsi);

	if (dsi->dsi_slave)
		sprd_dsi_enable(dsi->dsi_slave);
}

static void sprd_dsi_disable(struct sprd_dsi *dsi)
{
	sprd_dsi_fini(dsi);

	if (dsi->glb->disable)
		dsi->glb->disable(&dsi->ctx);
	if (dsi->glb->power)
		dsi->glb->power(&dsi->ctx, false);

	if (dsi->dsi_slave)
		sprd_dsi_disable(dsi->dsi_slave);
}

int dsi_panel_set_dpms_mode(struct sprd_dsi *dsi)
{
	mutex_lock(&dsi->lock);

	/*
	 * FIXME:
	 * Doze Suspend -> OFF, dsi has suspended
	 */
	if ((dsi->ctx.dpms == DRM_MODE_DPMS_OFF) &&
		(dsi->ctx.last_dpms == DRM_MODE_DPMS_SUSPEND)) {
		DRM_INFO("%s(panel off)\n", __func__);
		drm_panel_unprepare(dsi->panel);
		dsi->ctx.last_dpms = dsi->ctx.dpms;
		mutex_unlock(&dsi->lock);
		return 0;
	}

	if (!dsi->ctx.enabled) {
		mutex_unlock(&dsi->lock);
		DRM_INFO("dsi is not inited,just skip\n");
		return 0;
	}

	if ((dsi->ctx.dpms == DRM_MODE_DPMS_STANDBY) &&
		(dsi->ctx.last_dpms == DRM_MODE_DPMS_ON)) {
		sprd_panel_enter_doze(dsi->panel);
		DRM_INFO("%s(panel enter doze)\n", __func__);
		dsi->ctx.last_dpms = dsi->ctx.dpms;
	} else if ((dsi->ctx.dpms == DRM_MODE_DPMS_ON) &&
		(dsi->ctx.last_dpms == DRM_MODE_DPMS_STANDBY)) {
		sprd_panel_exit_doze(dsi->panel);
		DRM_INFO("%s(panel exit doze)\n", __func__);
		dsi->ctx.last_dpms = dsi->ctx.dpms;
	} else {
		DRM_INFO("%s(just skip it)\n", __func__);
	}
	mutex_unlock(&dsi->lock);

	return 0;
}

static void sprd_dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct sprd_dsi *dsi = encoder_to_dsi(encoder);
	struct sprd_crtc *crtc = to_sprd_crtc(encoder->crtc);
	struct sprd_dpu *dpu = crtc->priv;
	struct sprd_panel *panel = container_of(dsi->panel, struct sprd_panel, base);

	DRM_INFO("%s(last_dpms=%d, dpms=%d)\n",
			__func__, dsi->ctx.last_dpms, dsi->ctx.dpms);

	mutex_lock(&dsi->lock);
	/* add if condition to avoid resume dsi for SR feature.
	 * if esd recovery happened during display suspend, skip dsi resume.
	 */
	if (!encoder->crtc || !encoder->crtc->state->active ||
	    (encoder->crtc->state->mode_changed &&
	     !encoder->crtc->state->active_changed)) {
		DRM_INFO("skip dsi resume\n");
		mutex_unlock(&dsi->lock);
		return;
	}

	if (dsi->ctx.enabled) {
		DRM_INFO("dsi is initialized\n");
		if (!strcmp(dpu->ctx.version, "dpu-r6p0")) {
			if (!dpu->ctx.enabled) {
				DRM_ERROR("dpu has no power, powered dpu\n");
				dpu_r6p0_glb_enable(&dpu->ctx);
				sprd_dpu_resume(dpu);
			}
		}

		mutex_unlock(&dsi->lock);
		return;
	}

	if (!strcmp(dpu->ctx.version, "dpu-r6p0") || !strcmp(dpu->ctx.version, "dpu-r6p1"))
		pm_runtime_get_sync(dsi->dev.parent);

	sprd_dsi_enable(dsi);
	sprd_dphy_enable(dsi->phy);

	sprd_dsi_lp_cmd_enable(dsi, true);
	if (dsi->dsi_slave)
		sprd_dsi_lp_cmd_enable(dsi->dsi_slave, true);

	if (dsi->panel) {
		if ((dsi->ctx.last_dpms == DRM_MODE_DPMS_SUSPEND) &&
		    (dsi->ctx.dpms == DRM_MODE_DPMS_ON)) {
			sprd_panel_exit_doze(dsi->panel);
			DRM_INFO("%s(panel exit doze)\n", __func__);
		} else if ((dsi->ctx.last_dpms == DRM_MODE_DPMS_SUSPEND) &&
			   (dsi->ctx.dpms == DRM_MODE_DPMS_STANDBY)) {
			DRM_INFO("%s(keep panel doze)\n", __func__);
		} else {
			drm_panel_prepare(dsi->panel);
			drm_panel_enable(dsi->panel);
			if (dsi->ctx.dpms == DRM_MODE_DPMS_STANDBY) {
				DRM_INFO("%s(panel enter doze)\n", __func__);
				sprd_panel_enter_doze(dsi->panel);
			}
		}
	}

	sprd_dsi_set_work_mode(dsi, dsi->ctx.work_mode);
	sprd_dsi_state_reset(dsi);

	if ((dsi->ctx.work_mode == DSI_MODE_VIDEO)
			&& dsi->ctx.video_lp_cmd_en)
		sprd_dsi_lp_cmd_enable(dsi, true);

	if (dsi->ctx.nc_clk_en)
		sprd_dsi_nc_clk_en(dsi, true);
	else
		sprd_dphy_hs_clk_en(dsi->phy, true);

	if (dsi->dsi_slave) {
		sprd_dsi_set_work_mode(dsi->dsi_slave,
				dsi->dsi_slave->ctx.work_mode);
		sprd_dsi_state_reset(dsi->dsi_slave);

		if (dsi->dsi_slave->ctx.nc_clk_en)
			sprd_dsi_nc_clk_en(dsi->dsi_slave, true);
		else
			sprd_dphy_hs_clk_en(dsi->phy->slave, true);
	}

	/* workaround:
	 * dpu r6p0 need resume after dsi resume on div6 scences
	 * for dsi core and dpi clk depends on dphy clk. And esd
	 * recovery do not resume dpu, so need switch dpi clk to
	 * div6 source when dpu enable div6 function.
	 */
	if (!strcmp(dpu->ctx.version, "dpu-r6p0")) {
		if (!panel->is_esd_rst) {
			sprd_dpu_resume(dpu);
		} else {
			if (dsi->ctx.dpi_clk_div)
				dpu_r6p0_enable_div6_clk(&dpu->ctx);
		}
	}
	/*
	 * FIXME:
	 * When last dpms is doze_suspend,cmd mode panel remain on in a low power state and continue displaying
	 * its current contents indefinitely. If call sprd_dpu_run, background color will appear
	 * that will cause panel flickering. So we should call sprd_dpu_run when flip in edpi mode.
	 */
	if (dsi->ctx.last_dpms != DRM_MODE_DPMS_SUSPEND)
		sprd_dpu_run(crtc->priv);

	dsi->ctx.enabled = true;
	dsi->ctx.last_dpms = dsi->ctx.dpms;

	mutex_unlock(&dsi->lock);
}

static void sprd_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct sprd_dsi *dsi = encoder_to_dsi(encoder);
	struct sprd_crtc *crtc = to_sprd_crtc(encoder->crtc);
	struct sprd_dpu *dpu = crtc->priv;

	DRM_INFO("%s(last_dpms=%d, dpms=%d)\n",
			__func__, dsi->ctx.last_dpms, dsi->ctx.dpms);

	mutex_lock(&dsi->lock);
	/* add if condition to avoid suspend dsi for SR feature */
	if (encoder->crtc->state->mode_changed &&
	    !encoder->crtc->state->active_changed) {
			DRM_INFO("skip dsi encoder disable\n");
			mutex_unlock(&dsi->lock);
			return;
	}

	if (!dsi->ctx.enabled) {
		DRM_INFO("dsi isn't initialized\n");
		mutex_unlock(&dsi->lock);
		return;
	}

	sprd_dpu_stop(dpu);
	if (dsi->ctx.dpi_clk_div) {
		if (!strcmp(dpu->ctx.version, "dpu-r6p0")) {
			dsi->ctx.clk_dpi_384m = true;
			dsi->glb->disable(&dsi->ctx);
		}
	}
	sprd_dsi_set_work_mode(dsi, DSI_MODE_CMD);
	sprd_dsi_lp_cmd_enable(dsi, true);

	if (dsi->dsi_slave) {
		sprd_dsi_set_work_mode(dsi->dsi_slave, DSI_MODE_CMD);
		sprd_dsi_lp_cmd_enable(dsi->dsi_slave, true);
	}

	if (dsi->panel) {
		if ((dsi->ctx.dpms == DRM_MODE_DPMS_SUSPEND) &&
		    ((dsi->ctx.last_dpms == DRM_MODE_DPMS_STANDBY)
		     || (dsi->ctx.last_dpms == DRM_MODE_DPMS_ON))) {
			sprd_panel_enter_doze(dsi->panel);
			DRM_INFO("%s(panel enter doze)\n", __func__);
		} else {
			drm_panel_disable(dsi->panel);
			if (dsi->phy->ctx.ulps_enable)
				sprd_dphy_ulps_enter(dsi->phy);
			drm_panel_unprepare(dsi->panel);
		}
	}

	sprd_dphy_disable(dsi->phy);
	sprd_dsi_disable(dsi);

	if (!strcmp(dpu->ctx.version, "dpu-r6p0") || !strcmp(dpu->ctx.version, "dpu-r6p1"))
		pm_runtime_put(dsi->dev.parent);

	dsi->ctx.enabled = false;
	dsi->ctx.last_dpms = dsi->ctx.dpms;

	mutex_unlock(&dsi->lock);
}

void sprd_dsi_encoder_disable_force(struct drm_crtc *crtc)
{
	struct sprd_crtc *sprd_crtc = container_of(crtc, struct sprd_crtc, base);
	struct sprd_dpu *dpu = sprd_crtc->priv;
	struct sprd_dsi *dsi = dpu->dsi;

	DRM_INFO("%s()\n", __func__);
	mutex_lock(&dsi->lock);

	sprd_dpu_stop(dpu);

	if (!strcmp(dpu->ctx.version, "dpu-r6p0") && dsi->ctx.dpi_clk_div) {
		dsi->ctx.clk_dpi_384m = true;
		dsi->glb->disable(&dsi->ctx);
	}

	sprd_dsi_set_work_mode(dsi, DSI_MODE_CMD);
	sprd_dsi_lp_cmd_enable(dsi, true);

	if (dsi->panel) {
		drm_panel_disable(dsi->panel);
		sprd_dphy_ulps_enter(dsi->phy);
		drm_panel_unprepare(dsi->panel);
	}

	sprd_dphy_disable(dsi->phy);
	sprd_dsi_disable(dsi);
	dsi->ctx.enabled = false;

	if (!strcmp(dpu->ctx.version, "dpu-r6p0")) {
		disable_irq(dpu->ctx.irq);
		sprd_dpu_disable(dpu);
	}

	pm_runtime_put(dsi->dev.parent);
	mutex_unlock(&dsi->lock);
}

static void sprd_dsi_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adj_mode)
{
	DRM_INFO("%s() mode: "DRM_MODE_FMT"\n", __func__, DRM_MODE_ARG(mode));
}

static int sprd_dsi_encoder_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	DRM_INFO("%s()\n", __func__);

	return 0;
}

static const struct drm_encoder_helper_funcs sprd_encoder_helper_funcs = {
	.atomic_check	= sprd_dsi_encoder_atomic_check,
	.mode_set	= sprd_dsi_encoder_mode_set,
	.enable		= sprd_dsi_encoder_enable,
	.disable	= sprd_dsi_encoder_disable
};

static const struct drm_encoder_funcs sprd_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int sprd_dsi_encoder_init(struct drm_device *drm,
			       struct sprd_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	int ret;

	ret = drm_encoder_init(drm, encoder, &sprd_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, NULL);
	if (ret) {
		DRM_ERROR("failed to initialize dsi encoder\n");
		return ret;
	}

	ret = sprd_drm_set_possible_crtcs(encoder, SPRD_DISPLAY_TYPE_LCD);
	if (ret) {
		DRM_ERROR("failed to find possible crtc\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &sprd_encoder_helper_funcs);

	return 0;
}

static int sprd_dsi_find_panel(struct sprd_dsi *dsi)
{
	struct device *dev = dsi->host.dev;
	struct device_node *child, *lcds_node;
	struct drm_panel *panel;

	/* search /lcds child node first */
	lcds_node = of_find_node_by_path("/lcds");
	for_each_child_of_node(lcds_node, child) {
		panel = of_drm_find_panel(child);
		if (!IS_ERR(panel)) {
			dsi->panel = panel;
			return 0;
		}
	}

	/*
	 * If /lcds child node search failed, we search
	 * the child of dsi host node.
	 */
	for_each_child_of_node(dev->of_node, child) {
		panel = of_drm_find_panel(child);
		if (!IS_ERR(panel)) {
			dsi->panel = panel;
			return 0;
		}
	}

	DRM_ERROR("of_drm_find_panel() failed\n");
	return -ENODEV;
}

static int sprd_dsi_phy_attach(struct sprd_dsi *dsi)
{
	struct device *dev;

	dev = sprd_disp_pipe_get_output(&dsi->dev);
	if (!dev)
		return -ENODEV;

	dsi->phy = dev_get_drvdata(dev);
	if (!dsi->phy) {
		DRM_ERROR("dsi attach phy failed\n");
		return -EINVAL;
	}

	dsi->phy->ctx.lanes = dsi->ctx.lanes;
	dsi->phy->ctx.freq = dsi->ctx.byte_clk * 8;

	if (dsi->dsi_slave && dsi->phy->slave) {
		dsi->phy->slave->ctx.lanes = dsi->ctx.lanes;
		dsi->phy->slave->ctx.freq = dsi->phy->ctx.freq;
	}

	return 0;
}

static int sprd_dsi_host_attach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *slave)
{
	struct sprd_dsi *dsi = host_to_dsi(host);
	struct dsi_context *ctx = &dsi->ctx;
	struct dsi_context *ctx_slave;
	struct device_node *lcd_node;
	u32 val;
	int ret;

	DRM_INFO("%s()\n", __func__);

	dsi->slave = slave;
	ctx->lanes = slave->lanes;
	ctx->format = slave->format;
	ctx->byte_clk = slave->hs_rate / 8;
	ctx->esc_clk = slave->lp_rate;

	if (slave->mode_flags & MIPI_DSI_MODE_VIDEO)
		ctx->work_mode = DSI_MODE_VIDEO;
	else
		ctx->work_mode = DSI_MODE_CMD;

	if (slave->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		ctx->burst_mode = VIDEO_BURST_WITH_SYNC_PULSES;
	else if (slave->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		ctx->burst_mode = VIDEO_NON_BURST_WITH_SYNC_PULSES;
	else
		ctx->burst_mode = VIDEO_NON_BURST_WITH_SYNC_EVENTS;

	if (slave->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		ctx->nc_clk_en = true;

	ret = sprd_dsi_phy_attach(dsi);
	if (ret)
		return ret;

	ret = sprd_dsi_find_panel(dsi);
	if (ret)
		return ret;

	lcd_node = dsi->panel->dev->of_node;

	ctx->lcd_name = lcd_node->name;

	if (!of_property_read_u32(lcd_node, "sprd,video-lp-cmd-enable", &val))
		ctx->video_lp_cmd_en = val;
	else
		ctx->video_lp_cmd_en = 0;

	if (!of_property_read_u32(lcd_node, "sprd,hporch-lp-disable", &val))
		ctx->hporch_lp_disable = val;
	else
		ctx->hporch_lp_disable = 0;

	if (!of_property_read_u32(lcd_node, "sprd,dpi-clk-div", &val))
		ctx->dpi_clk_div = val;

	if (!of_property_read_u32(lcd_node, "sprd,phy-aod-mode", &val))
		dsi->phy->ctx.aod_mode = val;
	else
		dsi->phy->ctx.aod_mode = 0;

	if (!of_property_read_u32(lcd_node, "sprd,video-lp-en-mode", &val))
		ctx->video_lp_config = val;
	else
		ctx->video_lp_config = 15;

	if (dsi->dsi_slave) {
		ctx_slave = &dsi->dsi_slave->ctx;
		ctx_slave->lanes = ctx->lanes;
		ctx_slave->format = ctx->format;
		ctx_slave->work_mode = ctx->work_mode;
		ctx_slave->burst_mode = ctx->burst_mode;
		ctx_slave->nc_clk_en = ctx->nc_clk_en;
		ctx_slave->byte_clk = ctx->byte_clk;
		ctx_slave->esc_clk = ctx->esc_clk;
	}

	return 0;
}

static int sprd_dsi_host_detach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *slave)
{
	DRM_INFO("%s()\n", __func__);
	/* do nothing */
	return 0;
}

static ssize_t sprd_dsi_host_transfer(struct mipi_dsi_host *host,
				const struct mipi_dsi_msg *msg)
{
	struct sprd_dsi *dsi = host_to_dsi(host);
	const u8 *tx_buf = msg->tx_buf;
	int ret = 0;

	if (msg->rx_buf && msg->rx_len) {
		u8 lsb = (msg->tx_len > 0) ? tx_buf[0] : 0;
		u8 msb = (msg->tx_len > 1) ? tx_buf[1] : 0;

		return sprd_dsi_rd_pkt(dsi, msg->channel, msg->type,
				msb, lsb, msg->rx_buf, msg->rx_len);
	}

	if (msg->tx_buf && msg->tx_len) {
		ret = sprd_dsi_wr_pkt(dsi, msg->channel, msg->type,
					tx_buf, msg->tx_len);

		if (dsi->dsi_slave)
			ret = sprd_dsi_wr_pkt(dsi->dsi_slave, msg->channel, msg->type,
					tx_buf, msg->tx_len);
		return ret;
	}

	return 0;
}

static const struct mipi_dsi_host_ops sprd_dsi_host_ops = {
	.attach = sprd_dsi_host_attach,
	.detach = sprd_dsi_host_detach,
	.transfer = sprd_dsi_host_transfer,
};

static int sprd_dsi_host_init(struct device *dev, struct sprd_dsi *dsi)
{
	int ret;

	dsi->host.dev = dev;
	dsi->host.ops = &sprd_dsi_host_ops;

	ret = mipi_dsi_host_register(&dsi->host);
	if (ret)
		DRM_ERROR("failed to register dsi host\n");

	return ret;
}

static int sprd_dsi_connector_get_modes(struct drm_connector *connector)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);

	DRM_INFO("%s()\n", __func__);

	return drm_panel_get_modes(dsi->panel, connector);
}

static enum drm_mode_status
sprd_dsi_connector_mode_valid(struct drm_connector *connector,
			 struct drm_display_mode *mode)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);
	struct drm_display_mode *pmode;

	DRM_INFO("%s() mode: "DRM_MODE_FMT"\n", __func__, DRM_MODE_ARG(mode));

	if (mode->type & DRM_MODE_TYPE_USERDEF) {
		list_for_each_entry(pmode, &connector->modes, head) {
			if ((pmode->type & DRM_MODE_TYPE_PREFERRED) || (pmode->type == DRM_MODE_TYPE_DRIVER)) {
				list_del(&pmode->head);
				drm_mode_destroy(connector->dev, pmode);
				dsi->mode = mode;
				break;
			}
		}
	}

	return MODE_OK;
}

static struct drm_encoder *
sprd_dsi_connector_best_encoder(struct drm_connector *connector)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);

	DRM_INFO("%s()\n", __func__);
	return &dsi->encoder;
}

static struct drm_connector_helper_funcs sprd_dsi_connector_helper_funcs = {
	.get_modes = sprd_dsi_connector_get_modes,
	.mode_valid = sprd_dsi_connector_mode_valid,
	.best_encoder = sprd_dsi_connector_best_encoder,
};

static enum drm_connector_status
sprd_dsi_connector_detect(struct drm_connector *connector, bool force)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);

	DRM_INFO("%s()\n", __func__);

	if (dsi->panel) {
		//drm_panel_attach(dsi->panel, connector);
		return connector_status_connected;
	}

	return connector_status_disconnected;
}

static void sprd_dsi_connector_destroy(struct drm_connector *connector)
{
	DRM_INFO("%s()\n", __func__);

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static int sprd_dsi_atomic_get_property(struct drm_connector *connector,
					const struct drm_connector_state *state,
					struct drm_property *property,
					uint64_t *val)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);

	DRM_DEBUG("%s()\n", __func__);

	if (property == dsi->edid_prop) {
		memcpy(dsi->edid_blob->data, &dsi->edid_info, sizeof(struct edid));
		*val = dsi->edid_blob->base.id;
		DRM_INFO("%s() val = %d\n", __func__, dsi->edid_blob->base.id);
	} else {
		DRM_ERROR("property %s is invalid\n", property->name);
 		return -EINVAL;
	}

	return 0;
}

static const struct drm_connector_funcs sprd_dsi_atomic_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sprd_dsi_connector_detect,
	.destroy = sprd_dsi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_get_property = sprd_dsi_atomic_get_property,
};

static int sprd_dsi_connector_init(struct drm_device *drm, struct sprd_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_connector *connector = &dsi->connector;
	struct drm_property *prop;
	int ret;

	ret = drm_connector_init(drm, connector,
				 &sprd_dsi_atomic_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		DRM_ERROR("drm_connector_init() failed\n");
		return ret;
	}

	drm_connector_helper_add(connector,
				 &sprd_dsi_connector_helper_funcs);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	dsi->edid_blob = drm_property_create_blob(drm, (sizeof(struct edid) + 1), &dsi->edid_info);
	if (IS_ERR(dsi->edid_blob)) {
		DRM_ERROR("drm_property_create_blob edid blob failed\n");
		return PTR_ERR(dsi->edid_blob);
	}

	prop = drm_property_create(drm, DRM_MODE_PROP_BLOB, "EDID INFO", 0);
	if (!prop) {
		DRM_ERROR("drm_property_create dpu version failed\n");
		return -ENOMEM;
	}

	drm_object_attach_property(&connector->base, prop, dsi->edid_blob->base.id);
	dsi->edid_prop = prop;

	DRM_INFO("dsi->edid_blob->base.id:%d\n", dsi->edid_blob->base.id);

	return 0;
}

static int sprd_dsi_bridge_attach(struct sprd_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_bridge *bridge = dsi->bridge;
	struct device *dev = dsi->host.dev;
	struct device_node *bridge_node;
	int ret;

	bridge_node = of_graph_get_remote_node(dev->of_node, 2, 0);
	if (!bridge_node)
		return 0;

	bridge = of_drm_find_bridge(bridge_node);
	if (!bridge) {
		DRM_ERROR("of_drm_find_bridge() failed\n");
		return -ENODEV;
	}
	dsi->bridge = bridge;

	ret = drm_bridge_attach(encoder, bridge, NULL, 0);
	if (ret) {
		DRM_ERROR("failed to attach external bridge\n");
		return ret;
	}

	return 0;
}

static int sprd_dsi_glb_init(struct sprd_dsi *dsi)
{
	if (dsi->glb->power)
		dsi->glb->power(&dsi->ctx, true);
	if (dsi->glb->enable)
		dsi->glb->enable(&dsi->ctx);

	if (dsi->dsi_slave) {
		if (dsi->dsi_slave->glb && dsi->dsi_slave->glb->power)
			dsi->dsi_slave->glb->power(&dsi->dsi_slave->ctx, true);
		if (dsi->dsi_slave->glb && dsi->dsi_slave->glb->enable)
			dsi->dsi_slave->glb->enable(&dsi->dsi_slave->ctx);
	}

	return 0;
}

static irqreturn_t sprd_dsi_isr(int irq, void *data)
{
	struct sprd_dsi *dsi = data;
	u32 status = 0;

	if (dsi->ctx.irq0 == irq)
		status = sprd_dsi_int_status(dsi, 0);
	else if (dsi->ctx.irq1 == irq)
		status = sprd_dsi_int_status(dsi, 1);

	if (status & DSI_INT_STS_NEED_SOFT_RESET)
		sprd_dsi_state_reset(dsi);

	return IRQ_HANDLED;
}

static int sprd_dsi_irq_request(struct sprd_dsi *dsi)
{
	struct dsi_context *ctx = &dsi->ctx;
	int irq0, irq1;
	int ret;

	irq0 = irq_of_parse_and_map(dsi->host.dev->of_node, 0);
	if (irq0) {
		DRM_INFO("dsi irq0 num = %d\n", irq0);
		ret = request_irq(irq0, sprd_dsi_isr, 0, "DSI_INT0", dsi);
		if (ret) {
			DRM_ERROR("dsi failed to request irq int0!\n");
			return -EINVAL;
		}
	}
	ctx->irq0 = irq0;

	irq1 = irq_of_parse_and_map(dsi->host.dev->of_node, 1);
	if (irq1) {
		DRM_INFO("dsi irq1 num = %d\n", irq1);
		ret = request_irq(irq1, sprd_dsi_isr, 0, "DSI_INT1", dsi);
		if (ret) {
			DRM_ERROR("dsi failed to request irq int1!\n");
			return -EINVAL;
		}
	}
	ctx->irq1 = irq1;

	return 0;
}

static int sprd_dsi_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct sprd_dsi *dsi = dev_get_drvdata(dev);
	int ret;

	ret = sprd_dsi_encoder_init(drm, dsi);
	if (ret)
		goto cleanup_host;

	ret = sprd_dsi_connector_init(drm, dsi);
	if (ret)
		goto cleanup_encoder;

	ret = sprd_dsi_bridge_attach(dsi);
	if (ret)
		goto cleanup_connector;

	ret = sprd_dsi_glb_init(dsi);
	if (ret)
		goto cleanup_connector;

	ret = sprd_dsi_irq_request(dsi);
	if (ret)
		goto cleanup_connector;

	return 0;

cleanup_connector:
	drm_connector_cleanup(&dsi->connector);
cleanup_encoder:
	drm_encoder_cleanup(&dsi->encoder);
cleanup_host:
	mipi_dsi_host_unregister(&dsi->host);
	return ret;
}

static void sprd_dsi_unbind(struct device *dev,
			struct device *master, void *data)
{
	/* do nothing */
	DRM_INFO("%s()\n", __func__);

}

static const struct component_ops dsi_component_ops = {
	.bind	= sprd_dsi_bind,
	.unbind	= sprd_dsi_unbind,
};

static int sprd_dsi_device_create(struct sprd_dsi *dsi,
				struct device *parent)
{
	int ret;

	dsi->dev.class = display_class;
	dsi->dev.parent = parent;
	dsi->dev.of_node = parent->of_node;
	dev_set_name(&dsi->dev, "dsi%d", dsi->ctx.id);
	dev_set_drvdata(&dsi->dev, dsi);

	ret = device_register(&dsi->dev);
	if (ret)
		DRM_ERROR("dsi device register failed\n");

	return ret;
}

static unsigned char kEdid0[128] = {
        0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1c, 0xec, 0x01, 0x00,
        0x01, 0x00, 0x00, 0x00, 0x1b, 0x10, 0x01, 0x03, 0x80, 0x50, 0x2d, 0x78,
        0x0a, 0x0d, 0xc9, 0xa0, 0x57, 0x47, 0x98, 0x27, 0x12, 0x48, 0x4c, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
        0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xfc, 0x00, 0x45, 0x4d, 0x55, 0x5f, 0x64, 0x69, 0x73,
        0x70, 0x6c, 0x61, 0x79, 0x5f, 0x30, 0x00, 0x4b
};

static void sprd_edid_set_default_prop(struct edid *edid_info)
{
	memcpy(edid_info, kEdid0, sizeof(struct edid));
}

static int sprd_dsi_context_init(struct sprd_dsi *dsi, struct device_node *np)
{
	struct dsi_context *ctx = &dsi->ctx;
	struct resource r;
	u32 tmp;

	if (dsi->glb->parse_dt)
		dsi->glb->parse_dt(&dsi->ctx, np);

	if (of_address_to_resource(np, 0, &r)) {
		DRM_ERROR("parse dsi ctrl reg base failed\n");
		return -ENODEV;
	}
	ctx->base = (unsigned long)
	    ioremap(r.start, resource_size(&r));
	if (ctx->base == 0) {
		DRM_ERROR("dsi ctrl reg base ioremap failed\n");
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "dev-id", &tmp))
		ctx->id = tmp;

	if (!of_property_read_u32(np, "sprd,data-hs2lp", &tmp))
		ctx->data_hs2lp = tmp;
	else
		ctx->data_hs2lp = 120;

	if (!of_property_read_u32(np, "sprd,data-lp2hs", &tmp))
		ctx->data_lp2hs = tmp;
	else
		ctx->data_lp2hs = 500;

	if (!of_property_read_u32(np, "sprd,clk-hs2lp", &tmp))
		ctx->clk_hs2lp = tmp;
	else
		ctx->clk_hs2lp = 4;

	if (!of_property_read_u32(np, "sprd,clk-lp2hs", &tmp))
		ctx->clk_lp2hs = tmp;
	else
		ctx->clk_lp2hs = 15;

	if (!of_property_read_u32(np, "sprd,max-read-time", &tmp))
		ctx->max_rd_time = tmp;
	else
		ctx->max_rd_time = 0x8000;

	if (!of_property_read_u32(np, "sprd,int0_mask", &tmp))
		ctx->int0_mask = tmp;
	else
		ctx->int0_mask = 0xffffffff;

	if (!of_property_read_u32(np, "sprd,int1_mask", &tmp))
		ctx->int1_mask = tmp;
	else
		ctx->int1_mask = 0xffffffff;

	sprd_edid_set_default_prop(&dsi->edid_info);

	dsi->ctx.enabled = true;

	return 0;
}

static const struct sprd_dsi_ops sharkle_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &sharkle_dsi_glb_ops,
};

static const struct sprd_dsi_ops pike2_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &pike2_dsi_glb_ops,
};

static const struct sprd_dsi_ops sharkl3_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &sharkl3_dsi_glb_ops,
};

static const struct sprd_dsi_ops sharkl5_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &sharkl5_dsi_glb_ops,
};

static const struct sprd_dsi_ops sharkl5pro_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &sharkl5pro_dsi_glb_ops,
};

static const struct sprd_dsi_ops qogirl6_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &qogirl6_dsi_glb_ops,
};

static const struct sprd_dsi_ops qogirn6pro_dsi = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &qogirn6pro_dsi_glb_ops
};

static const struct sprd_dsi_ops qogirn6pro_dsi1 = {
	.core = &dsi_ctrl_r1p0_ops,
	.glb = &qogirn6pro_dsi_s_glb_ops,
};

static const struct sprd_dsi_ops qogirn6lite_dsi = {
	.core = &dsi_ctrl_r1p1_ops,
	.glb = &qogirn6lite_dsi_glb_ops
};

static const struct of_device_id dsi_match_table[] = {
	{ .compatible = "sprd,sharkle-dsi-host",
	  .data = &sharkle_dsi },
	{ .compatible = "sprd,pike2-dsi-host",
	  .data = &pike2_dsi },
	{ .compatible = "sprd,sharkl3-dsi-host",
	  .data = &sharkl3_dsi },
	{ .compatible = "sprd,sharkl5-dsi-host",
	  .data = &sharkl5_dsi },
	{ .compatible = "sprd,sharkl5pro-dsi-host",
	  .data = &sharkl5pro_dsi },
	{ .compatible = "sprd,qogirl6-dsi-host",
	  .data = &qogirl6_dsi },
	{ .compatible = "sprd,qogirn6pro-dsi-host",
	  .data = &qogirn6pro_dsi },
	{ .compatible = "sprd,qogirn6pro-dsi1-host",
	  .data = &qogirn6pro_dsi1 },
	{ .compatible = "sprd,qogirn6lite-dsi-host",
	  .data = &qogirn6lite_dsi },
	{ /* sentinel */ },
};

int sprd_dual_dsi_parse_dt(struct sprd_dsi *dsi, struct device_node *np) {
	struct device_node *lcd_node;
	int rc;
	u32 val;

	lcd_node = sprd_get_panel_node_by_name();
	if (!lcd_node)
		return -ENODEV;

	rc = of_property_read_u32(lcd_node, "sprd,dual-dsi-enable", &val);
	if (!rc)
		dsi->dual_dsi_en = val;
	else
		DRM_DEBUG("dual-dsi-enable is not found!\n");

	return 0;
}

static int sprd_dsi_dual_channel_init(struct sprd_dsi *dsi)
{
	struct device_node *np;
	struct platform_device *secondary;

	np = of_parse_phandle(dsi->dev.of_node, "sprd,dual-channel", 0);
	if (np) {
		DRM_INFO("find sprd,dual-channel\n");
		secondary = of_find_device_by_node(np);
		dsi->dsi_slave = dev_get_drvdata(&secondary->dev);
		of_node_put(np);

		if (!dsi->dsi_slave)
			return -EPROBE_DEFER;

		dsi->dsi_slave->dsi_master = dsi;
		dsi->dsi_slave->dual_dsi_en = 1;
	}

	return 0;
}

static int sprd_dsi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct sprd_dsi_ops *pdata;
	struct sprd_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(&pdev->dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi) {
		DRM_ERROR("failed to allocate dsi data.\n");
		return -ENOMEM;
	}

	/*
	 * FIXME:
	 * Every time hwc finished init, it will set connector dpms to DRM_MODE_DPMS_ON.
	 * We use connector dpms status to check if display pipeline is ready instead of active state of crtc,
	 * because releation ship between drm object connetor & encoder & crtc will be destroied during modeset.
	 * However, after we use kzalloc allocing dsi struct, the dpms value of connetor is set to default value 0,
	 * the value of DRM_MODE_DPMS_ON is set to 0 by linux kernel original design.
	 * So, we initialize dpms status to off when after dsi sturct created.
	 */
	dsi->connector.dpms = DRM_MODE_DPMS_OFF;

	pdata = of_device_get_match_data(&pdev->dev);
	if (pdata) {
		dsi->core = pdata->core;
		dsi->glb = pdata->glb;
	} else {
		DRM_ERROR("No matching driver data found\n");
		return -EINVAL;
	}

	ret = sprd_dsi_context_init(dsi, np);
	if (ret)
		return ret;

	ret = sprd_dsi_device_create(dsi, &pdev->dev);
	if (ret)
		return ret;

	ret = sprd_dsi_sysfs_init(&dsi->dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dsi);

	if (dsi->ctx.id) {
		DRM_INFO("dsi slave skip other action\n");
		return 0;
	}

	ret = sprd_dual_dsi_parse_dt(dsi, np);
	if (ret) {
		DRM_ERROR("parse dual dsi info failed\n");
		return ret;
	}

	if (dsi->dual_dsi_en) {
		ret = sprd_dsi_dual_channel_init(dsi);
		if (ret)
			return ret;
	}

	ret = sprd_dsi_host_init(&pdev->dev, dsi);
	if (ret)
		return ret;

	mutex_init(&dsi->lock);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return component_add(&pdev->dev, &dsi_component_ops);
}

static int sprd_dsi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dsi_component_ops);

	return 0;
}

struct platform_driver sprd_dsi_driver = {
	.probe = sprd_dsi_probe,
	.remove = sprd_dsi_remove,
	.driver = {
		.name = "sprd-dsi-drv",
		.of_match_table = dsi_match_table,
	},
};

MODULE_AUTHOR("Leon He <leon.he@unisoc.com>");
MODULE_AUTHOR("Kevin Tang <kevin.tang@unisoc.com>");
MODULE_DESCRIPTION("Unisoc MIPI DSI HOST Controller Driver");
MODULE_LICENSE("GPL v2");
