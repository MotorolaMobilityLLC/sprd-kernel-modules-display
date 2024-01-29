// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <video/of_display_timing.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drm_atomic_helper.h>

#include "sprd_crtc.h"
#include "sprd_dpu.h"
#include "sprd_drm.h"
#include "sprd_dsi_panel.h"
#include "dsi/sprd_dsi_api.h"
#include "sysfs/sysfs_display.h"

#ifdef CONFIG_BL_I2C_CTRL
extern bool check_aw99703_probed(void);
extern int aw99703_sleepin(void);
extern int aw99703_sleepout(void);
#endif
#ifdef CONFIG_HBM_SUPPORT
bool g_hbm_enable = false;
EXPORT_SYMBOL(g_hbm_enable);
unsigned int g_last_level = 32;
struct backlight_device *g_bdev;
#endif
#define SPRD_MIPI_DSI_FMT_DSC 0xff

#define host_to_dsi(host) \
	container_of(host, struct sprd_dsi, host)

static int sprd_oled_set_brightness(struct backlight_device *bdev);

#ifdef CONFIG_TOUCH_GESTURE_CTRL
static bool touch_gesture_disable = true;
void touch_set_state(int state)
{
    if (state)
        touch_gesture_disable = false;
    else
        touch_gesture_disable = true;
}
EXPORT_SYMBOL(touch_set_state);
#endif

struct device_node *sprd_get_panel_node_by_name(void)
{
	struct device_node *lcd_node, *cmdline_node;
	const char *cmd_line, *lcd_name_p;
	char lcd_path[60];
	char lcd_name[50];
	int rc;

	cmdline_node = of_find_node_by_path("/chosen");
	rc = of_property_read_string(cmdline_node, "bootargs", &cmd_line);
	if (!rc) {
		lcd_name_p = strstr(cmd_line, "lcd_name=");
		if (lcd_name_p) {
			sscanf(lcd_name_p, "lcd_name=%s", lcd_name);
			DRM_INFO("lcd name: %s\n", lcd_name);
		}
	} else {
		DRM_ERROR("can't not parse bootargs property\n");
		return NULL;
	}

	snprintf(lcd_path, sizeof(lcd_path), "/lcds/%s", lcd_name);
	lcd_node = of_find_node_by_path(lcd_path);
	if (!lcd_node) {
		DRM_ERROR("could not find %s node\n", lcd_name);
		return NULL;
	}

	return lcd_node;
}

static int sprd_panel_send_cmds(struct mipi_dsi_device *dsi,
				const void *data, int size)
{
	struct sprd_panel *panel;
	const struct dsi_cmd_desc *cmds = data;
	u16 len;

	if ((cmds == NULL) || (dsi == NULL))
		return -EINVAL;

	panel = mipi_dsi_get_drvdata(dsi);

	while (size > 0) {
		len = (cmds->wc_h << 8) | cmds->wc_l;

		if (panel->info.use_dcs)
			mipi_dsi_dcs_write_buffer(dsi, cmds->payload, len);
		else
			mipi_dsi_generic_write(dsi, cmds->payload, len);

		if (cmds->wait)
			msleep(cmds->wait);
		cmds = (const struct dsi_cmd_desc *)(cmds->payload + len);
		size -= (len + 4);
	}

	return 0;
}

int sprd_panel_send_vrefresh_cmd(struct sprd_panel *panel, int index)
{
	DRM_INFO("%s(), index is :%d\n", __func__, index);

	sprd_panel_send_cmds(panel->slave,
			     panel->info.vrefresh_cmds[index],
			     panel->info.vrefresh_cmds_len[index]);

	return 0;
}

static int sprd_panel_unprepare(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);
	struct gpio_timing *timing;
	int items, i;

	DRM_INFO("%s()\n", __func__);

	if (panel->info.panel_type == SPRD_PANEL_TYPE_AMOLED) {
		if (panel->info.reset_gpio) {
			items = panel->info.rst_off_seq.items;
			timing = panel->info.rst_off_seq.timing;
			for (i = 0; i < items; i++) {
				gpiod_direction_output(panel->info.reset_gpio,
							timing[i].level);
				mdelay(timing[i].delay);
			}
		}
	} else {
#ifdef CONFIG_TOUCH_GESTURE_CTRL
		if (touch_gesture_disable) {
#endif
			if (panel->info.reset_gpio) {
				items = panel->info.rst_off_seq.items;
				timing = panel->info.rst_off_seq.timing;
				for (i = 0; i < items; i++) {
					gpiod_direction_output(panel->info.reset_gpio,
								timing[i].level);
					mdelay(timing[i].delay);
				}
			}

			if (panel->info.avee_gpio) {
				gpiod_direction_output(panel->info.avee_gpio, 0);
				mdelay(5);
			}

			if (panel->info.avdd_gpio) {
				gpiod_direction_output(panel->info.avdd_gpio, 0);
				mdelay(5);
			}

			regulator_disable(panel->supply);
#ifdef CONFIG_TOUCH_GESTURE_CTRL
		}
#endif
	}

	return 0;
}

void  sprd_panel_enter_doze(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s() enter\n", __func__);

	mutex_lock(&panel->lock);

	if (panel->esd_work_pending) {
		cancel_delayed_work_sync(&panel->esd_work);
		panel->esd_work_pending = false;
	}

	sprd_panel_send_cmds(panel->slave,
	       panel->info.cmds[CMD_CODE_DOZE_IN],
	       panel->info.cmds_len[CMD_CODE_DOZE_IN]);

	mutex_unlock(&panel->lock);
}

void  sprd_panel_exit_doze(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s() enter\n", __func__);

	mutex_lock(&panel->lock);

	sprd_panel_send_cmds(panel->slave,
		panel->info.cmds[CMD_CODE_DOZE_OUT],
		panel->info.cmds_len[CMD_CODE_DOZE_OUT]);

	if (panel->info.esd_check_en) {
		schedule_delayed_work(&panel->esd_work,
				      msecs_to_jiffies(1000));
		panel->esd_work_pending = true;
	}

	mutex_unlock(&panel->lock);
}

static int sprd_panel_prepare(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);
	struct gpio_timing *timing;
	int items, i, ret;

	DRM_INFO("%s()\n", __func__);

	if (panel->info.panel_type == SPRD_PANEL_TYPE_AMOLED) {
		if (panel->info.reset_gpio) {
			items = panel->info.rst_on_seq.items;
			timing = panel->info.rst_on_seq.timing;
			for (i = 0; i < items; i++) {
				gpiod_direction_output(panel->info.reset_gpio,
							timing[i].level);
				mdelay(timing[i].delay);
			}
		}
	} else {
		ret = regulator_enable(panel->supply);
		if (ret < 0)
			DRM_ERROR("enable lcd regulator failed\n");

		if (panel->info.avdd_gpio) {
			gpiod_direction_output(panel->info.avdd_gpio, 1);
			mdelay(5);
		}

		if (panel->info.avee_gpio) {
			gpiod_direction_output(panel->info.avee_gpio, 1);
			mdelay(5);
		}

		if (panel->info.reset_gpio) {
			items = panel->info.rst_on_seq.items;
			timing = panel->info.rst_on_seq.timing;
			for (i = 0; i < items; i++) {
				gpiod_direction_output(panel->info.reset_gpio,
							timing[i].level);
				mdelay(timing[i].delay);
			}
		}
	}

	return 0;
}

static int sprd_panel_disable(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s()\n", __func__);

	/*
	 * FIXME:
	 * The cancel work should be executed before DPU stop,
	 * otherwise the esd check will be failed if the DPU
	 * stopped in video mode and the DSI has not change to
	 * CMD mode yet. Since there is no VBLANK timing for
	 * LP cmd transmission.
	 */
	if (panel->esd_work_pending) {
		cancel_delayed_work_sync(&panel->esd_work);
		panel->esd_work_pending = false;
	}

	mutex_lock(&panel->lock);

	if (panel->backlight && !panel->sprd_bl_mipi_type) {
		panel->backlight->props.power = FB_BLANK_POWERDOWN;
		panel->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(panel->backlight);
	}

	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_CODE_SLEEP_IN],
			     panel->info.cmds_len[CMD_CODE_SLEEP_IN]);

#ifdef CONFIG_BL_I2C_CTRL
	if (true == check_aw99703_probed())
		aw99703_sleepin();
#endif
	panel->enabled = false;
	mutex_unlock(&panel->lock);

	return 0;
}

static int sprd_panel_enable(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s()\n", __func__);

	mutex_lock(&panel->lock);
#ifdef CONFIG_BL_I2C_CTRL
	if (true == check_aw99703_probed())
		aw99703_sleepout();
#endif
	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_CODE_INIT],
			     panel->info.cmds_len[CMD_CODE_INIT]);

	if (panel->backlight && !panel->sprd_bl_mipi_type) {
		panel->backlight->props.power = FB_BLANK_UNBLANK;
		panel->backlight->props.state &= ~BL_CORE_FBBLANK;
		backlight_update_status(panel->backlight);
	}

	if (panel->info.esd_check_en) {
		schedule_delayed_work(&panel->esd_work,
				      msecs_to_jiffies(1000));
		panel->esd_work_pending = true;
	}

	panel->enabled = true;
	panel->info.vrefresh_cmd_changed = true;
	mutex_unlock(&panel->lock);

	return 0;
}

static struct drm_display_mode * sprd_panel_create_sr_mode(struct drm_device *drm,
							struct drm_display_mode *original_mode,
							u32 sr_width, u32 sr_height)
{
	struct drm_display_mode *new_mode;
	struct videomode vm = {};

	new_mode = drm_mode_create(drm);
	if (!new_mode) {
		DRM_ERROR("drm mode create failed\n");
		return NULL;
	}

	drm_display_mode_to_videomode(original_mode, &vm);
	vm.hactive = sr_width;
	vm.vactive = sr_height;
	drm_display_mode_from_videomode(&vm, new_mode);
	new_mode->clock = new_mode->htotal * new_mode->vtotal *
				drm_mode_vrefresh(original_mode) / 1000;

	DRM_INFO("%s() mode: "DRM_MODE_FMT"\n", __func__, DRM_MODE_ARG(new_mode));

	return new_mode;
}

static int sprd_panel_get_modes(struct drm_panel *p, struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct sprd_panel *panel = to_sprd_panel(p);
	struct sprd_dsi *dsi = container_of(connector, struct sprd_dsi, connector);
	struct device_node *np = panel->slave->dev.of_node;
	u32 surface_width = 0, surface_height = 0;
	u32 sr_width = 0, sr_height = 0;
	int i, mode_count = 0;

	DRM_INFO("%s()\n", __func__);
	mode = drm_mode_duplicate(connector->dev, &panel->info.mode);
	if (!mode) {
		DRM_ERROR("failed to alloc mode %s\n", panel->info.mode.name);
		return 0;
	}
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	mode_count++;

	for (i = 0; i < panel->info.num_buildin_modes - 1; i++) {
		mode = drm_mode_duplicate(connector->dev,
			&(panel->info.buildin_modes[i]));
		if (!mode) {
			DRM_ERROR("failed to alloc mode %s\n",
				panel->info.buildin_modes[i].name);
			return 0;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);
		mode_count++;
	}

	/* parse and create sr mode config */
	of_property_read_u32(panel->info.of_node, "sprd,sr-width", &sr_width);
	of_property_read_u32(panel->info.of_node, "sprd,sr-height", &sr_height);
	if (sr_width && sr_height) {
		DRM_INFO("find sr config: width:%d, height:%d\n", sr_width, sr_height);
		for (i = 0; i < panel->info.num_buildin_modes; i++) {
			mode = sprd_panel_create_sr_mode(connector->dev,
						&panel->info.buildin_modes[i], sr_width, sr_height);
			if (!mode) {
				DRM_ERROR("create sr display mode failed\n");
				break;
			}
			mode->type = DRM_MODE_TYPE_DRIVER;
			mode->width_mm = panel->info.mode.width_mm;
			mode->height_mm = panel->info.mode.height_mm;
			drm_mode_probed_add(connector, mode);
			mode_count++;
		}
	}

	of_property_read_u32(np, "sprd,surface-width", &surface_width);
	of_property_read_u32(np, "sprd,surface-height", &surface_height);
	if (surface_width && surface_height) {
		struct videomode vm = {};

		vm.hactive = surface_width;
		vm.vactive = surface_height;
		vm.pixelclock = surface_width * surface_height * 60;
		mode = drm_mode_create(connector->dev);
		mode->width_mm = panel->info.mode.width_mm;
		mode->height_mm = panel->info.mode.height_mm;
		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_USERDEF;
		drm_display_mode_from_videomode(&vm, mode);
		drm_mode_probed_add(connector, mode);
		mode_count++;
		dsi->ctx.surface_mode = true;
	}

	connector->display_info.width_mm = panel->info.mode.width_mm;
	connector->display_info.height_mm = panel->info.mode.height_mm;

	panel->info.display_mode_count = mode_count;

	return mode_count;
}

static const struct drm_panel_funcs sprd_panel_funcs = {
	.get_modes = sprd_panel_get_modes,
	.enable = sprd_panel_enable,
	.disable = sprd_panel_disable,
	.prepare = sprd_panel_prepare,
	.unprepare = sprd_panel_unprepare,
};

static int sprd_panel_esd_check(struct sprd_panel *panel)
{
	struct mipi_dsi_host *host = panel->slave->host;
	struct sprd_dsi *dsi = host_to_dsi(host);
	struct drm_connector *connector = &dsi->connector;
	struct panel_info *info = &panel->info;
	struct sprd_dpu *dpu;
	bool crtc_active_state;
	u8 read_val = 0;

	crtc_active_state = sprd_check_crtc_active_state(connector->dev, 0);
	if (!crtc_active_state) {
		DRM_INFO("skip esd during panel suspend\n");
		return 0;
	}

	if (!dsi->ctx.enabled) {
		DRM_WARN("dsi is not initialized，skip esd check\n");
		return 0;
	}

	mutex_lock(&panel->lock);
	if (!panel->enabled) {
		DRM_INFO("panel is not enabled, skip esd check");
		mutex_unlock(&panel->lock);
		return 0;
	}

	dpu = dsi->dpu;
	mutex_lock(&dpu->ctx.vrr_lock);

	if (info->cmd_dpi_mode)
		dpu_wait_te_flush(&dpu->ctx);

	/* FIXME: we should enable HS cmd tx here */
	mipi_dsi_set_maximum_return_packet_size(panel->slave, 1);
	if (panel->info.cmds[CMD_CODE_BL_PREFIX] &&
		panel->info.cmds_len[CMD_CODE_BL_PREFIX]) {
		sprd_panel_send_cmds(panel->slave,
				panel->info.cmds[CMD_CODE_BL_PREFIX],
				panel->info.cmds_len[CMD_CODE_BL_PREFIX]);
	}
	mipi_dsi_dcs_read(panel->slave, info->esd_check_reg,
			  &read_val, 1);
	mutex_unlock(&dpu->ctx.vrr_lock);

	/*
	 * TODO:
	 * Should we support multi-registers check in the future?
	 */
	if (read_val != info->esd_check_val) {
		DRM_ERROR("esd check failed, read value = 0x%02x\n",
			  read_val);
		mutex_unlock(&panel->lock);
		return -EINVAL;
	}

	mutex_unlock(&panel->lock);

	return 0;
}

static int sprd_panel_te_check(struct sprd_panel *panel)
{
	struct mipi_dsi_host *host = panel->slave->host;
	struct sprd_dsi *dsi = host_to_dsi(host);
	struct drm_connector *connector = &dsi->connector;
	struct sprd_dpu *dpu;
	int ret;
	bool crtc_active_state;
	bool irq_occur = false;

	crtc_active_state = sprd_check_crtc_active_state(connector->dev, 0);
	if (!crtc_active_state) {
		DRM_INFO("skip esd during panel suspend\n");
		return 0;
	}

	if (!dsi->ctx.enabled) {
		DRM_WARN("dsi is not initialized，skip esd check\n");
		return 0;
	}

	mutex_lock(&panel->lock);
	if (!panel->enabled) {
		DRM_INFO("panel is not enabled, skip esd check");
		mutex_unlock(&panel->lock);
		return 0;
	}

	dpu = dsi->dpu;

	/* DPU TE irq maybe enabled in kernel */
	if (!dpu->ctx.enabled) {
		mutex_unlock(&panel->lock);
		return 0;
	}

	dpu->ctx.te_check_en = true;

	/* wait for TE interrupt */
	ret = wait_event_interruptible_timeout(dpu->ctx.te_wq,
		dpu->ctx.evt_te, msecs_to_jiffies(500));
	if (!ret) {
		/* double check TE interrupt through dpu_int_raw register */
		if (dpu->core && dpu->core->check_raw_int) {
			down(&dpu->ctx.lock);
			if (dpu->ctx.enabled)
				irq_occur = dpu->core->check_raw_int(&dpu->ctx,
					BIT_DPU_INT_TE);
			up(&dpu->ctx.lock);
			if (!irq_occur) {
				DRM_ERROR("TE esd timeout.\n");
				ret = -ETIMEDOUT;
			} else
				DRM_WARN("TE occur, but isr schedule delay\n");
		} else {
			DRM_ERROR("TE esd timeout.\n");
			ret = -ETIMEDOUT;
		}
	}

	dpu->ctx.te_check_en = false;
	dpu->ctx.evt_te = false;

	mutex_unlock(&panel->lock);

	return ret < 0 ? ret : 0;
}

static int sprd_panel_mix_check(struct sprd_panel *panel)
{
	int ret;

	ret = sprd_panel_esd_check(panel);
	if (ret) {
		DRM_ERROR("mix check use read reg with error\n");
		return ret;
	}

	ret = sprd_panel_te_check(panel);
	if (ret) {
		DRM_ERROR("mix check use te signal with error\n");
		return ret;
	}

	return 0;
}

static void sprd_recovery_oled_backlight(struct sprd_panel *panel)
{
	if (!panel->sprd_bl_mipi_type) {
		DRM_DEBUG("not support mipi command backlight, skip recovery backlight\n");
		return;
	}

	if (panel->backlight) {
		DRM_INFO("====== recovery oled backlight ========\n");
		if (panel->backlight->props.brightness) {
			sprd_oled_set_brightness(panel->backlight);
		} else {
			panel->backlight->props.brightness = SPRD_OLED_DEFAULT_BRIGHTNESS;
			sprd_oled_set_brightness(panel->backlight);
		}
	}
}

static void sprd_panel_esd_work_func(struct work_struct *work)
{
	struct sprd_panel *panel = container_of(work, struct sprd_panel,
						esd_work.work);
	struct panel_info *info = &panel->info;
	struct mipi_dsi_host *host = panel->slave->host;
	struct sprd_dsi *dsi = host_to_dsi(host);
	struct drm_connector *connector = &dsi->connector;
	int ret;

	if ((!dsi->ctx.enabled) || (!panel->enabled)) {
		DRM_ERROR("dsi is suspended, skip esd work\n");
		return;
	}

	if (info->esd_check_mode == ESD_MODE_REG_CHECK)
		ret = sprd_panel_esd_check(panel);
	else if (info->esd_check_mode == ESD_MODE_TE_CHECK)
		ret = sprd_panel_te_check(panel);
	else if (info->esd_check_mode == ESD_MODE_MIX_CHECK)
		ret = sprd_panel_mix_check(panel);
	else {
		DRM_ERROR("unknown esd check mode:%d\n", info->esd_check_mode);
		return;
	}

	if(!sprd_check_crtc_active_state(connector->dev, 0)) {
		DRM_ERROR("crtc is inactive, skip esd work\n");
		schedule_delayed_work(&panel->esd_work,
			msecs_to_jiffies(info->esd_check_period));
		return;
	} else if (dsi->dpu->crtc->mode_change_pending) {
		DRM_INFO("vrr is going, skip esd work\n");
		schedule_delayed_work(&panel->esd_work,
			msecs_to_jiffies(info->esd_check_period));
		return;
	}

	if (ret) {
		const struct drm_encoder_helper_funcs *encoder_funcs;
		const struct drm_connector_helper_funcs *conn_funcs;
		struct drm_encoder *encoder;

		conn_funcs = connector->helper_private;
		encoder = conn_funcs->best_encoder(connector);
		encoder_funcs = encoder->helper_private;
		panel->esd_work_pending = false;

		if (!sprd_check_crtc_active_state(connector->dev, 0)) {
			DRM_INFO("skip esd recovery during panel suspend\n");
			return;
		}

		DRM_INFO("====== esd recovery start ========\n");
		panel->is_esd_rst = true;
		encoder_funcs->disable(encoder);
		encoder_funcs->enable(encoder);
		sprd_recovery_oled_backlight(panel);
		panel->is_esd_rst = false;
		if (!panel->esd_work_pending && panel->enabled)
			schedule_delayed_work(&panel->esd_work,
					msecs_to_jiffies(info->esd_check_period));
		DRM_INFO("======= esd recovery end =========\n");
	} else
		schedule_delayed_work(&panel->esd_work,
			msecs_to_jiffies(info->esd_check_period));
}

static int sprd_panel_gpio_request(struct device *dev,
			struct sprd_panel *panel)
{
	panel->info.avdd_gpio = devm_gpiod_get_optional(dev,
					"avdd", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(panel->info.avdd_gpio))
		DRM_WARN("can't get panel avdd gpio: %ld\n",
				 PTR_ERR(panel->info.avdd_gpio));

	panel->info.avee_gpio = devm_gpiod_get_optional(dev,
					"avee", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(panel->info.avee_gpio))
		DRM_WARN("can't get panel avee gpio: %ld\n",
				 PTR_ERR(panel->info.avee_gpio));

	panel->info.reset_gpio = devm_gpiod_get_optional(dev,
					"reset", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(panel->info.reset_gpio))
		DRM_WARN("can't get panel reset gpio: %ld\n",
				 PTR_ERR(panel->info.reset_gpio));

	return 0;
}

static int of_parse_reset_seq(struct device_node *np,
				struct panel_info *info)
{
	struct property *prop;
	int bytes, rc;
	u32 *p;

	prop = of_find_property(np, "sprd,reset-on-sequence", &bytes);
	if (!prop) {
		DRM_ERROR("sprd,reset-on-sequence property not found\n");
		return -EINVAL;
	}

	p = kzalloc(bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	rc = of_property_read_u32_array(np, "sprd,reset-on-sequence", p, bytes / 4);
	if (rc) {
		DRM_ERROR("parse sprd,reset-on-sequence failed\n");
		kfree(p);
		return rc;
	}

	info->rst_on_seq.items = bytes / 8;
	info->rst_on_seq.timing = (struct gpio_timing *)p;

	prop = of_find_property(np, "sprd,reset-off-sequence", &bytes);
	if (!prop) {
		DRM_ERROR("sprd,reset-off-sequence property not found\n");
		return -EINVAL;
	}

	p = kzalloc(bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	rc = of_property_read_u32_array(np, "sprd,reset-off-sequence", p, bytes / 4);
	if (rc) {
		DRM_ERROR("parse sprd,reset-off-sequence failed\n");
		kfree(p);
		return rc;
	}

	info->rst_off_seq.items = bytes / 8;
	info->rst_off_seq.timing = (struct gpio_timing *)p;

	return 0;
}

static int of_parse_buildin_modes(struct panel_info *info,
	struct device_node *lcd_node)
{
	int i, rc, num_timings;
	struct device_node *timings_np;

	timings_np = of_get_child_by_name(lcd_node, "display-timings");
	if (!timings_np) {
		DRM_ERROR("%s: can not find display-timings node\n",
			lcd_node->name);
		return -ENODEV;
	}

	num_timings = of_get_child_count(timings_np);
	if (num_timings == 0) {
		/* should never happen, as entry was already found above */
		DRM_ERROR("%s: no timings specified\n", lcd_node->name);
		goto done;
	}

	info->buildin_modes = kzalloc(sizeof(struct drm_display_mode) *
				num_timings, GFP_KERNEL);
	if (!info->buildin_modes) {
		DRM_ERROR("alloc modes memory failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_timings; i++) {
		rc = of_get_drm_display_mode(lcd_node,
			&info->buildin_modes[i], NULL, i);
		if (rc) {
			DRM_ERROR("get display timing failed\n");
			goto entryfail;
		}

		info->buildin_modes[i].width_mm = info->mode.width_mm;
		info->buildin_modes[i].height_mm = info->mode.height_mm;
		//info->buildin_modes[i].vrefresh = info->mode.vrefresh;
	}
	info->num_buildin_modes = num_timings;
	DRM_INFO("info->num_buildin_modes = %d\n", num_timings);
	goto done;

entryfail:
	kfree(info->buildin_modes);
done:
	of_node_put(timings_np);

	return 0;
}

static int of_parse_oled_cmds(struct sprd_oled *oled,
		const void *data, int size)
{
	const struct dsi_cmd_desc *cmds = data;
	struct dsi_cmd_desc *p;
	u16 len;
	int i, total;

	if (cmds == NULL)
		return -EINVAL;

	/*
	 * TODO:
	 * Currently, we only support the same length cmds
	 * for oled brightness level. So we take the first
	 * cmd payload length as all.
	 */
	len = (cmds->wc_h << 8) | cmds->wc_l;
	total =  size / (len + 4);
	if (total > (sizeof(oled->cmds) / sizeof(oled->cmds[0]))) {
		DRM_ERROR("oled backlight cmds length over range\n");
		total = sizeof(oled->cmds) / sizeof(oled->cmds[0]);
	}

	p = (struct dsi_cmd_desc *)kzalloc(size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	memcpy(p, cmds, size);
	for (i = 0; i < total; i++) {
		oled->cmds[i] = p;
		p = (struct dsi_cmd_desc *)(p->payload + len);
	}

	oled->cmds_total = total;
	oled->cmd_len = len + 4;

	return 0;
}

static int sprd_oled_set_brightness(struct backlight_device *bdev)
{
	int brightness;
	struct sprd_oled *oled = bl_get_data(bdev);
	struct sprd_panel *panel = oled->panel;

	mutex_lock(&panel->lock);
	if (!panel->enabled) {
		mutex_unlock(&panel->lock);
		DRM_WARN("oled panel has been powered off\n");
		return -ENXIO;
	}
#ifdef CONFIG_HBM_SUPPORT
	if (unlikely(g_hbm_enable)) {
		if (bdev->props.brightness)
			brightness = 255;
		else
			brightness = bdev->props.brightness;
		DRM_INFO("Now hbm enable, set brightness level: %d\n", brightness);
	} else {
		brightness = ((bdev->props.brightness * 36)+50)/51;
		g_last_level  = bdev->props.brightness;
		DRM_INFO("%s brightness: %d, g_last_level: %d\n", __func__, brightness, g_last_level);
	}
#else
	brightness = bdev->props.brightness;

	DRM_INFO("%s brightness: %d\n", __func__, brightness);
#endif
	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_OLED_REG_LOCK],
			     panel->info.cmds_len[CMD_OLED_REG_LOCK]);

	if (panel->info.cmds[CMD_CODE_BL_PREFIX] &&
		panel->info.cmds_len[CMD_CODE_BL_PREFIX]) {
		sprd_panel_send_cmds(panel->slave,
				panel->info.cmds[CMD_CODE_BL_PREFIX],
				panel->info.cmds_len[CMD_CODE_BL_PREFIX]);
	}

	if (oled->cmds_total == 1) {
		if (oled->cmds[0]->wc_l == 3) {
			oled->cmds[0]->payload[1] = brightness >> 8;
			oled->cmds[0]->payload[2] = brightness & 0xFF;
		} else
			oled->cmds[0]->payload[1] = brightness;

		sprd_panel_send_cmds(panel->slave,
			     oled->cmds[0],
			     oled->cmd_len);
	} else {
		if (brightness >= oled->cmds_total)
			brightness = oled->cmds_total - 1;

		sprd_panel_send_cmds(panel->slave,
			     oled->cmds[brightness],
			     oled->cmd_len);
	}

	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_OLED_REG_UNLOCK],
			     panel->info.cmds_len[CMD_OLED_REG_UNLOCK]);

	mutex_unlock(&panel->lock);

	return 0;
}

static const struct backlight_ops sprd_oled_backlight_ops = {
	.update_status = sprd_oled_set_brightness,
};

#ifdef CONFIG_HBM_SUPPORT
int hbm_set_backlight_level(unsigned int level)
{
	if (g_bdev != NULL) {
		g_hbm_enable = true;
		g_bdev->props.brightness = level;
		sprd_oled_set_brightness(g_bdev);
		return 0;
	} else {
		DRM_INFO("hbm g_bdev is null, please register sprd backlight\n");
		return -1;
	}
}
EXPORT_SYMBOL(hbm_set_backlight_level);

int hbm_exit_set_backlight_level(void)
{
	if (g_bdev != NULL) {
		g_hbm_enable = false;
		DRM_INFO("hbm mode exit, g_last_level = %d\n", g_last_level);
		g_bdev->props.brightness = g_last_level;
		sprd_oled_set_brightness(g_bdev);
		return 0;
	} else {
		DRM_INFO("hbm g_bdev is null, please register sprd backlight\n");
		return -1;
	}
}
EXPORT_SYMBOL(hbm_exit_set_backlight_level);
#endif

static int sprd_oled_backlight_device_create(struct sprd_oled *oled,
					struct device_node *oled_node)
{
	int ret;

	oled->oled_dev.class = display_class;
	oled->oled_dev.of_node = oled_node;
	oled->oled_dev.parent = &oled->bdev->dev;
	dev_set_name(&oled->oled_dev, "backlight");
	dev_set_drvdata(&oled->oled_dev, oled->bdev);
	ret = device_register(&oled->oled_dev);
	if (ret) {
		DRM_ERROR("oled backlight device register failed\n");
		return ret;
	}

	return 0;
}

static int sprd_oled_backlight_init(struct sprd_panel *panel,
					struct device_node *oled_node)
{
	struct sprd_oled *oled;
	struct panel_info *info = &panel->info;
	const void *p;
	int bytes, rc, ret;
	u32 temp;

	oled = devm_kzalloc(&panel->dev,
			sizeof(struct sprd_oled), GFP_KERNEL);
	if (!oled)
		return -ENOMEM;

	oled->bdev = devm_backlight_device_register(&panel->dev,
			"sprd_backlight", &panel->dev, oled,
			&sprd_oled_backlight_ops, NULL);
	if (IS_ERR(oled->bdev)) {
		DRM_ERROR("failed to register oled backlight ops\n");
		return PTR_ERR(oled->bdev);
	}

	ret = sprd_oled_backlight_device_create(oled, oled_node);
	if (ret)
		return ret;

	ret = sprd_backlight_sysfs_init(&oled->oled_dev);
	if (ret)
		return ret;

	p = of_get_property(oled_node, "brightness-levels", &bytes);
	if (p) {
		info->cmds[CMD_OLED_BRIGHTNESS] = p;
		info->cmds_len[CMD_OLED_BRIGHTNESS] = bytes;
	} else
		DRM_ERROR("can't find brightness-levels property\n");

	p = of_get_property(oled_node, "sprd,reg-lock", &bytes);
	if (p) {
		info->cmds[CMD_OLED_REG_LOCK] = p;
		info->cmds_len[CMD_OLED_REG_LOCK] = bytes;
	} else
		DRM_INFO("can't find sprd,reg-lock property\n");

	p = of_get_property(oled_node, "sprd,reg-unlock", &bytes);
	if (p) {
		info->cmds[CMD_OLED_REG_UNLOCK] = p;
		info->cmds_len[CMD_OLED_REG_UNLOCK] = bytes;
	} else
		DRM_INFO("can't find sprd,reg-unlock property\n");

	rc = of_property_read_u32(oled_node, "default-brightness-level", &temp);
	if (!rc)
		oled->bdev->props.brightness = temp;
	else
		oled->bdev->props.brightness = 25;

	rc = of_property_read_u32(oled_node, "sprd,max-level", &temp);
	if (!rc)
		oled->max_level = temp;
	else
		oled->max_level = 255;

	oled->bdev->props.max_brightness = oled->max_level;
	oled->panel = panel;
	panel->backlight = oled->bdev;
	panel->sprd_bl_mipi_type = true;
	of_parse_oled_cmds(oled,
			panel->info.cmds[CMD_OLED_BRIGHTNESS],
			panel->info.cmds_len[CMD_OLED_BRIGHTNESS]);
#ifdef CONFIG_HBM_SUPPORT
	g_bdev = oled->bdev;
	rc = sprd_backlight_hbm_sysfs_init(&oled->bdev->dev);
	if (rc) {
		DRM_ERROR("sprd backlight hbm sysfs init fail\n", __func__);
		return rc;
	}
#endif
	DRM_INFO("%s() ok\n", __func__);

	return 0;
}

int sprd_panel_parse_lcddtb(struct device_node *lcd_node,
	struct sprd_panel *panel)
{
	struct panel_info *info = &panel->info;
	int bytes, rc;
	const void *p;
	const char *str;
	u32 val;

	if (!lcd_node) {
		DRM_ERROR("lcd node from dtb is null\n");
		return -ENODEV;
	}
	info->of_node = lcd_node;

	rc = of_property_read_u32(lcd_node, "sprd,dsi-work-mode", &val);
	if (!rc) {
		if (val == SPRD_DSI_MODE_CMD)
			info->mode_flags = 0;
		else if (val == SPRD_DSI_MODE_CMD_DPI) {
			info->mode_flags = 0;
			info->cmd_dpi_mode = true;
		} else if (val == SPRD_DSI_MODE_VIDEO_BURST)
			info->mode_flags = MIPI_DSI_MODE_VIDEO |
					   MIPI_DSI_MODE_VIDEO_BURST;
		else if (val == SPRD_DSI_MODE_VIDEO_SYNC_PULSE)
			info->mode_flags = MIPI_DSI_MODE_VIDEO |
					   MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
		else if (val == SPRD_DSI_MODE_VIDEO_SYNC_EVENT)
			info->mode_flags = MIPI_DSI_MODE_VIDEO;
	} else {
		DRM_ERROR("dsi work mode is not found! use video mode\n");
		info->mode_flags = MIPI_DSI_MODE_VIDEO |
				   MIPI_DSI_MODE_VIDEO_BURST;
	}

	rc = of_property_read_u32(lcd_node, "sprd,panel-type", &val);
	if (!rc) {
		info->panel_type = val;
	} else {
		info->panel_type = SPRD_PANEL_TYPE_LCD;
	}

	if (of_property_read_bool(lcd_node, "sprd,dsi-non-continuous-clock"))
		info->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

	if (of_property_read_bool(lcd_node, "sprd,dpi-clk-pixelpll")) {
		info->dpi_clk_pixelpll = true;
		pr_info("read sprd,dpi-clk-pixelpll success, dpi_clk_pixelpll = true\n");
	} else {
		info->dpi_clk_pixelpll = false;
		pr_info("read sprd,dpi-clk-pixelpll failed, dpi_clk_pixelpll = false\n");
	}

	rc = of_property_read_u32(lcd_node, "sprd,dsi-lane-number", &val);
	if (!rc)
		info->lanes = val;
	else
		info->lanes = 4;

	rc = of_property_read_u32(lcd_node, "sprd,slice-width", &val);
	if (!rc)
		info->slice_width = val;
	else
		DRM_DEBUG("slice-width is not found!\n");

	rc = of_property_read_u32(lcd_node, "sprd,slice-height", &val);
	if (!rc)
		info->slice_height = val;
	else
		DRM_DEBUG("slice-height is not found!\n");

	rc = of_property_read_u32(lcd_node, "sprd,output-bpc", &val);
	if (!rc)
		info->output_bpc = val;
	else
		DRM_DEBUG("output-bpc is not found!\n");

	rc = of_property_read_u32(lcd_node, "sprd,dsc-enable", &val);
	if (!rc)
		info->dsc_en = val;
	else
		DRM_DEBUG("dsc-enable is not found!\n");

	rc = of_property_read_u32(lcd_node, "sprd,dual-dsi-enable", &val);
	if (!rc)
		info->dual_dsi_en = val;
	else
		DRM_DEBUG("dual-dsi-enable is not found!\n");

	rc = of_property_read_string(lcd_node, "sprd,dsi-color-format", &str);
	if (rc)
		info->format = MIPI_DSI_FMT_RGB888;
	else if (!strcmp(str, "rgb888"))
		info->format = MIPI_DSI_FMT_RGB888;
	else if (!strcmp(str, "rgb666"))
		info->format = MIPI_DSI_FMT_RGB666;
	else if (!strcmp(str, "rgb666_packed"))
		info->format = MIPI_DSI_FMT_RGB666_PACKED;
	else if (!strcmp(str, "rgb565"))
		info->format = MIPI_DSI_FMT_RGB565;
	else if (!strcmp(str, "dsc"))
		info->format = SPRD_MIPI_DSI_FMT_DSC;
	else
		DRM_ERROR("dsi-color-format (%s) is not supported\n", str);

	rc = of_property_read_u32(lcd_node, "sprd,phy-bit-clock", &val);
	if (!rc)
		info->hs_rate = val;
	else
		info->hs_rate = 500000;

	rc = of_property_read_u32(lcd_node, "sprd,phy-escape-clock", &val);
	if (!rc)
		info->lp_rate = val > 20000 ? 20000 : val;
	else
		info->lp_rate = 20000;

	rc = of_property_read_u32(lcd_node, "sprd,width-mm", &val);
	if (!rc)
		info->mode.width_mm = val;
	else
		info->mode.width_mm = 68;

	rc = of_property_read_u32(lcd_node, "sprd,height-mm", &val);
	if (!rc)
		info->mode.height_mm = val;
	else
		info->mode.height_mm = 121;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-enable", &val);
	if (!rc)
		info->esd_check_en = val;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-mode", &val);
	if (!rc)
		info->esd_check_mode = val;
	else
		info->esd_check_mode = 1;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-period", &val);
	if (!rc)
		info->esd_check_period = val;
	else
		info->esd_check_period = 1000;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-register", &val);
	if (!rc)
		info->esd_check_reg = val;
	else
		info->esd_check_reg = 0x0A;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-value", &val);
	if (!rc)
		info->esd_check_val = val;
	else
		info->esd_check_val = 0x9C;

	if (of_property_read_bool(lcd_node, "sprd,use-dcs-write"))
		info->use_dcs = true;
	else
		info->use_dcs = false;

	rc = of_parse_reset_seq(lcd_node, info);
	if (rc)
		DRM_ERROR("parse lcd reset sequence failed\n");

	p = of_get_property(lcd_node, "sprd,initial-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_INIT] = p;
		info->cmds_len[CMD_CODE_INIT] = bytes;
	} else
		DRM_ERROR("can't find sprd,init-command property\n");

	p = of_get_property(lcd_node, "sprd,sleep-in-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_SLEEP_IN] = p;
		info->cmds_len[CMD_CODE_SLEEP_IN] = bytes;
	} else
		DRM_ERROR("can't find sprd,sleep-in-command property\n");

	p = of_get_property(lcd_node, "sprd,sleep-out-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_SLEEP_OUT] = p;
		info->cmds_len[CMD_CODE_SLEEP_OUT] = bytes;
	} else
		DRM_ERROR("can't find sprd,sleep-out-command property\n");

	p = of_get_property(lcd_node, "sprd,doze-in-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_DOZE_IN] = p;
		info->cmds_len[CMD_CODE_DOZE_IN] = bytes;
	} else
		DRM_INFO("can't find sprd,doze-in-command property\n");

	p = of_get_property(lcd_node, "sprd,doze-out-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_DOZE_OUT] = p;
		info->cmds_len[CMD_CODE_DOZE_OUT] = bytes;
	} else
		DRM_INFO("can't find sprd,doze-out-command property\n");

	rc = of_get_drm_display_mode(lcd_node, &info->mode, 0,
				     OF_USE_NATIVE_MODE);
	if (rc) {
		DRM_ERROR("get display timing failed\n");
		return rc;
	}

	//info->mode.vrefresh = drm_mode_vrefresh(&info->mode);
	of_parse_buildin_modes(info, lcd_node);

	return 0;
}

int sprd_panel_vrr_config(struct device_node *lcd_node,
	struct sprd_panel *panel)
{
	struct panel_info *info = &panel->info;
	int bytes, rc, i, max_vrefresh;
	const void *p;
	char vrefresh_rate_node[60];
	u32 val;

	if (!info->cmd_dpi_mode)
		return 0;

	DRM_INFO("cmd mode panel vrr enabled");

	rc = of_property_read_u32(lcd_node, "sprd,supported-vrefresh-count", &val);
	if (rc) {
		DRM_ERROR("get supported vrefresh count failed\n");
		return rc;
	}
	info->vrr_mode_count = val;
	info->vrr_mode_vrefresh = kzalloc(sizeof(uint32_t) * info->vrr_mode_count, GFP_KERNEL);
	if (!info->vrr_mode_vrefresh) {
		DRM_ERROR("alloc vrr mode vfresh array space failed\n");
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(lcd_node, "sprd,supported-vrefresh-rate",
					info->vrr_mode_vrefresh, info->vrr_mode_count);
	if (rc) {
		DRM_ERROR("get supported vrefresh rate config failed\n");
		kfree (info->vrr_mode_vrefresh);
		return rc;
	}

	max_vrefresh = info->vrr_mode_vrefresh[0];
	for (i = 0; i < info->vrr_mode_count; i++) {
		if (max_vrefresh < info->vrr_mode_vrefresh[i])
			max_vrefresh = info->vrr_mode_vrefresh[i];
	}
	info->max_vrefresh = max_vrefresh;
	DRM_INFO("current platform support max vrefresh rate is :%d\n", max_vrefresh);

	for (i = 0; i < info->vrr_mode_count; i++) {
		memset(vrefresh_rate_node, 0, sizeof(char) * 60);
		snprintf(vrefresh_rate_node, sizeof(vrefresh_rate_node), "sprd,vrefresh-cmd-%d", info->vrr_mode_vrefresh[i]);
		p = of_get_property(lcd_node, vrefresh_rate_node, &bytes);
		if (p) {
			info->vrefresh_cmds[i] = p;
			info->vrefresh_cmds_len[i] = bytes;
		} else {
			DRM_ERROR("can't find %s property\n", vrefresh_rate_node);
			return -ENODEV;
		}
	}

	p = of_get_property(lcd_node, "sprd,bl-prefix", &bytes);
	if (p) {
		info->cmds[CMD_CODE_BL_PREFIX] = p;
		info->cmds_len[CMD_CODE_BL_PREFIX] = bytes;
	} else {
		DRM_INFO("no need send prefix cmd\n");
	}

	return 0;
}

static int sprd_panel_parse_dt(struct device_node *np, struct sprd_panel *panel)
{
	struct device_node *lcd_node;
	int rc;

	lcd_node = sprd_get_panel_node_by_name();
	if (!lcd_node)
		return -ENODEV;

	rc = sprd_panel_parse_lcddtb(lcd_node, panel);
	if (rc)
		return rc;

	rc = sprd_panel_vrr_config(lcd_node, panel);
	if (rc)
		return rc;

	return 0;
}

static int sprd_panel_device_create(struct device *parent,
				    struct sprd_panel *panel)
{
	panel->dev.class = display_class;
	panel->dev.parent = parent;
	panel->dev.of_node = panel->info.of_node;
	dev_set_name(&panel->dev, "panel0");
	dev_set_drvdata(&panel->dev, panel);

	return device_register(&panel->dev);
}

static int sprd_panel_probe(struct mipi_dsi_device *slave)
{
	struct sprd_panel *panel;
	struct device_node *bl_node, *oled_bl_node, *lcd_node;
	int ret;

	panel = devm_kzalloc(&slave->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	lcd_node = sprd_get_panel_node_by_name();
	if (!lcd_node)
		return -ENODEV;

	oled_bl_node = of_get_child_by_name(lcd_node, "oled-backlight");
	if (!oled_bl_node) {
		bl_node = of_parse_phandle(slave->dev.of_node,
					"sprd,backlight", 0);
		if (bl_node) {
			panel->backlight = of_find_backlight_by_node(bl_node);
			of_node_put(bl_node);

			if (panel->backlight) {
				panel->backlight->props.state &= ~BL_CORE_FBBLANK;
				panel->backlight->props.power = FB_BLANK_UNBLANK;
				backlight_update_status(panel->backlight);
			} else {
				DRM_WARN("backlight is not ready, panel probe deferred\n");
				return -EPROBE_DEFER;
			}
		} else {
			DRM_WARN("backlight node not found\n");
			return -ENODEV;
		}
	}

	panel->supply = devm_regulator_get(&slave->dev, "power");
	if (IS_ERR(panel->supply)) {/*  */
		if (PTR_ERR(panel->supply) == -EPROBE_DEFER)
			DRM_ERROR("regulator driver not initialized, probe deffer\n");
		else
			DRM_ERROR("can't get regulator: %ld\n", PTR_ERR(panel->supply));

		return PTR_ERR(panel->supply);
	}

	INIT_DELAYED_WORK(&panel->esd_work, sprd_panel_esd_work_func);

	ret = sprd_panel_parse_dt(slave->dev.of_node, panel);
	if (ret) {
		DRM_ERROR("parse panel info failed\n");
		return ret;
	}

	ret = sprd_panel_gpio_request(&slave->dev, panel);
	if (ret) {
		DRM_WARN("gpio is not ready, panel probe deferred\n");
		return -EPROBE_DEFER;
	}

	ret = sprd_panel_device_create(&slave->dev, panel);
	if (ret)
		return ret;

	if (oled_bl_node) {
		ret = sprd_oled_backlight_init(panel, oled_bl_node);
		if (ret)
			return ret;
	}

	panel->base.dev = &panel->dev;
	panel->base.funcs = &sprd_panel_funcs;
	drm_panel_init(&panel->base, &panel->dev, &sprd_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&panel->base);
	if (ret) {
		DRM_ERROR("drm_panel_add() failed\n");
		return ret;
	}

	slave->lanes = panel->info.lanes;
	slave->format = panel->info.format;
	slave->hs_rate = panel->info.hs_rate;
	slave->lp_rate = panel->info.lp_rate;
	slave->mode_flags = panel->info.mode_flags;

	ret = mipi_dsi_attach(slave);
	if (ret) {
		DRM_ERROR("failed to attach dsi panel to host\n");
		drm_panel_remove(&panel->base);
		return ret;
	}
	panel->slave = slave;
	ret = sprd_panel_sysfs_init(&panel->dev);
	if (ret)
		return ret;
	mipi_dsi_set_drvdata(slave, panel);
	/*
	 * FIXME:
	 * The esd check work should not be scheduled in probe
	 * function. It should be scheduled in the enable()
	 * callback function. But the dsi encoder will not call
	 * drm_panel_enable() the first time in encoder_enable().
	 */
	if (panel->info.esd_check_en) {
		schedule_delayed_work(&panel->esd_work,
				      msecs_to_jiffies(2000));
		panel->esd_work_pending = true;
	}
	panel->enabled = true;

	mutex_init(&panel->lock);

	return 0;
}

static int sprd_panel_remove(struct mipi_dsi_device *slave)
{
	struct sprd_panel *panel = mipi_dsi_get_drvdata(slave);
	int ret;

	DRM_INFO("%s()\n", __func__);

	sprd_panel_disable(&panel->base);
	sprd_panel_unprepare(&panel->base);

	ret = mipi_dsi_detach(slave);
	if (ret < 0)
		DRM_ERROR("failed to detach from DSI host: %d\n", ret);

	//drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	return 0;
}

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "sprd,generic-mipi-panel", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, panel_of_match);

struct mipi_dsi_driver sprd_panel_driver = {
	.driver = {
		.name = "sprd-mipi-panel-drv",
		.of_match_table = panel_of_match,
	},
	.probe = sprd_panel_probe,
	.remove = sprd_panel_remove,
};

MODULE_AUTHOR("Leon He <leon.he@unisoc.com>");
MODULE_AUTHOR("Kevin Tang <kevin.tang@unisoc.com>");
MODULE_DESCRIPTION("UNISOC MIPI DSI Panel Driver");
MODULE_LICENSE("GPL v2");
