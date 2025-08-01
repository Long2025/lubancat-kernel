/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/display_timing.h>
#include <video/mipi_display.h>
#include <video/of_display_timing.h>
#include <linux/of_graph.h>
#include <video/videomode.h>

#include "../rockchip/rockchip_drm_drv.h"

#include <linux/nvmem-consumer.h>

struct panel_cmd_header {
	u8 data_type;
	u8 delay;
	u8 payload_length;
} __packed;

struct panel_cmd_desc {
	struct panel_cmd_header header;
	u8 *payload;
};

struct panel_cmd_seq {
	struct panel_cmd_desc *cmds;
	unsigned int cmd_cnt;
};

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	/**
	 * @width: width (in millimeters) of the panel's active display area
	 * @height: height (in millimeters) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 * @reset: the time (in milliseconds) that it takes for the panel
	 *         to reset itself completely
	 * @init: the time (in milliseconds) that it takes for the panel to
	 *	  send init command sequence after reset deassert
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
		unsigned int reset;
		unsigned int init;
	} delay;

	u32 bus_format;
	u32 bus_flags;

	struct panel_cmd_seq *init_seq;
	struct panel_cmd_seq *exit_seq;
};

struct panel_simple {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;
	bool prepared;
	bool enabled;
	bool power_invert;

	const struct panel_desc *desc;

	struct backlight_device *backlight;
	struct regulator *supply;
	struct regulator_bulk_data supplies[2];
	struct i2c_adapter *ddc;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	int cmd_type;

	struct gpio_desc *spi_sdi_gpio;
	struct gpio_desc *spi_scl_gpio;
	struct gpio_desc *spi_cs_gpio;
	struct device_node *np_crtc;
};

enum rockchip_cmd_type {
	CMD_TYPE_DEFAULT,
	CMD_TYPE_SPI,
	CMD_TYPE_MCU
};

enum MCU_IOCTL {
	MCU_WRCMD = 0,
	MCU_WRDATA,
	MCU_SETBYPASS,
};

enum rockchip_spi_cmd_type {
	SPI_3LINE_9BIT_MODE_CMD = 0,
	SPI_3LINE_9BIT_MODE_DATA,
	SPI_4LINE_8BIT_MODE,
};

static void panel_simple_sleep(unsigned int msec)
{
	if (msec > 20)
		msleep(msec);
	else
		usleep_range(msec * 1000, (msec + 1) * 1000);
}

static inline int get_panel_cmd_type(const char *s)
{
	if (!s)
		return -EINVAL;

	if (strncmp(s, "spi", 3) == 0)
		return CMD_TYPE_SPI;
	else if (strncmp(s, "mcu", 3) == 0)
		return CMD_TYPE_MCU;

	return CMD_TYPE_DEFAULT;
}

struct entry {
	u32 offset;
	u32 length;
};

/**
* @magic: LCD config firmware magic number. 
* @vendor: LCD vendor name.
* @model: LCD model name.
* @version: LCD config firmware version.
* @timing_entry: Entry of timing table.
* @init_seq_entry: Entry of init sequence.
* @eixt_seq_entry: Entry of exit sequence.
* @touchscreen_entry: Entry of touchscreen properties.
* @firmware_size: Firmware size.
*/
struct firmware_header {
	u32 magic;
	u8 vendor[16];
	u8 model[32];
	u8 version[8];
	struct entry timing_entry;
	struct entry init_seq_entry;
	struct entry eixt_seq_entry;
	struct entry touchscreen_entry;
	u32 firmware_size;
};

static inline struct panel_simple *to_panel_simple(struct drm_panel *panel)
{
	return container_of(panel, struct panel_simple, base);
}

static int panel_simple_parse_cmd_seq(struct device *dev,
				      const u8 *data, int length,
				      struct panel_cmd_seq *seq)
{
	struct panel_cmd_header *header;
	struct panel_cmd_desc *desc;
	char *buf, *d;
	unsigned int i, cnt, len;

	if (!seq)
		return -EINVAL;

	buf = devm_kmemdup(dev, data, length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	d = buf;
	len = length;
	cnt = 0;
	while (len > sizeof(*header)) {
		header = (struct panel_cmd_header *)d;

		d += sizeof(*header);
		len -= sizeof(*header);

		if (header->payload_length > len)
			return -EINVAL;

		d += header->payload_length;
		len -= header->payload_length;
		cnt++;
	}

	if (len)
		return -EINVAL;

	seq->cmd_cnt = cnt;
	seq->cmds = devm_kcalloc(dev, cnt, sizeof(*desc), GFP_KERNEL);
	if (!seq->cmds)
		return -ENOMEM;

	d = buf;
	len = length;
	for (i = 0; i < cnt; i++) {
		header = (struct panel_cmd_header *)d;
		len -= sizeof(*header);
		d += sizeof(*header);

		desc = &seq->cmds[i];
		desc->header = *header;
		desc->payload = d;

		d += header->payload_length;
		len -= header->payload_length;
	}

	return 0;
}

static void panel_simple_spi_write_cmd(struct panel_simple *panel,
				       u8 type, int value)
{
	int i;

	gpiod_direction_output(panel->spi_cs_gpio, 0);

	/**
	 * send cmd or data flag for 3line 9bit serial data
	 */
	if (type == SPI_3LINE_9BIT_MODE_CMD) {
		gpiod_direction_output(panel->spi_sdi_gpio, 0);
		gpiod_direction_output(panel->spi_scl_gpio, 0);
		udelay(10);
		gpiod_direction_output(panel->spi_scl_gpio, 1);
		udelay(10);
	} else if (type == SPI_3LINE_9BIT_MODE_DATA) {
		gpiod_direction_output(panel->spi_sdi_gpio, 1);
		gpiod_direction_output(panel->spi_scl_gpio, 0);
		udelay(10);
		gpiod_direction_output(panel->spi_scl_gpio, 1);
		udelay(10);
	}

	/**
	 * send the 8bit value from the MSB
	 */
	for (i = 0; i < 8; i++) {
		if (value & 0x80)
			gpiod_direction_output(panel->spi_sdi_gpio, 1);
		else
			gpiod_direction_output(panel->spi_sdi_gpio, 0);

		gpiod_direction_output(panel->spi_scl_gpio, 0);
		udelay(10);
		gpiod_direction_output(panel->spi_scl_gpio, 1);
		value <<= 1;
		udelay(10);
	}

	gpiod_direction_output(panel->spi_cs_gpio, 1);
}

static int panel_simple_xfer_mcu_cmd_seq(struct panel_simple *panel,
				      struct panel_cmd_seq *cmds)
{
	int i;

	if (!cmds)
		return -EINVAL;

	rockchip_drm_crtc_send_mcu_cmd(panel->base.drm,
				       panel->np_crtc, MCU_SETBYPASS, 1);
	for (i = 0; i < cmds->cmd_cnt; i++) {
		struct panel_cmd_desc *cmd = &cmds->cmds[i];
		u32 value = 0;

		value = cmd->payload[0];
		rockchip_drm_crtc_send_mcu_cmd(panel->base.drm, panel->np_crtc,
					       cmd->header.data_type, value);
		if (cmd->header.delay)
			panel_simple_sleep(cmd->header.delay);
	}
	rockchip_drm_crtc_send_mcu_cmd(panel->base.drm,
				       panel->np_crtc, MCU_SETBYPASS, 0);

	return 0;
}

static int panel_simple_xfer_spi_cmd_seq(struct panel_simple *panel,
					 struct panel_cmd_seq *cmds)
{
	int i;

	if (!cmds)
		return -EINVAL;

	for (i = 0; i < cmds->cmd_cnt; i++) {
		struct panel_cmd_desc *cmd = &cmds->cmds[i];
		int value = 0;

		if (cmd->header.payload_length == 2)
			value = (cmd->payload[0] << 8) | cmd->payload[1];
		else
			value = cmd->payload[0];
		panel_simple_spi_write_cmd(panel, cmd->header.data_type, value);

		if (cmd->header.delay)
			panel_simple_sleep(cmd->header.delay);
	}

	return 0;
}

#if IS_ENABLED(CONFIG_DRM_MIPI_DSI)
static int panel_simple_xfer_dsi_cmd_seq(struct panel_simple *panel,
				     struct panel_cmd_seq *seq)
{
	struct device *dev = panel->base.dev;
	struct mipi_dsi_device *dsi = panel->dsi;
	unsigned int i;
	int err;

	if (!seq)
		return -EINVAL;

	for (i = 0; i < seq->cmd_cnt; i++) {
		struct panel_cmd_desc *cmd = &seq->cmds[i];

		switch (cmd->header.data_type) {
		case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
		case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
		case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		case MIPI_DSI_GENERIC_LONG_WRITE:
			err = mipi_dsi_generic_write(dsi, cmd->payload,
						     cmd->header.payload_length);
			break;
		case MIPI_DSI_DCS_SHORT_WRITE:
		case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		case MIPI_DSI_DCS_LONG_WRITE:
			err = mipi_dsi_dcs_write_buffer(dsi, cmd->payload,
							cmd->header.payload_length);
			break;
		default:
			return -EINVAL;
		}

		if (err < 0)
			dev_err(dev, "failed to write dcs cmd: %d\n", err);

		if (cmd->header.delay)
			panel_simple_sleep(cmd->header.delay);
	}

	return 0;
}
#else
static inline int panel_simple_xfer_dsi_cmd_seq(struct panel_simple *panel,
						struct panel_cmd_seq *seq)
{
	return -EINVAL;
}
#endif

static int panel_simple_get_fixed_modes(struct panel_simple *panel)
{
	struct drm_connector *connector = panel->base.connector;
	struct drm_device *drm = panel->base.drm;
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	if (!panel->desc)
		return 0;

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];
		struct videomode vm;

		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(drm);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u\n",
				dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_timings == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		num++;
	}

	for (i = 0; i < panel->desc->num_modes; i++) {
		const struct drm_display_mode *m = &panel->desc->modes[i];

		mode = drm_mode_duplicate(drm, m);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	if (panel->desc->bpc)
		connector->display_info.bpc = panel->desc->bpc;
	if (panel->desc->size.width)
		connector->display_info.width_mm = panel->desc->size.width;
	if (panel->desc->size.height)
		connector->display_info.height_mm = panel->desc->size.height;
	if (panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->desc->bus_format, 1);
	if (panel->desc->bus_flags)
		connector->display_info.bus_flags = panel->desc->bus_flags;

	return num;
}

static int panel_simple_regulator_enable(struct panel_simple *p)
{
	int err;

	err = regulator_bulk_enable(ARRAY_SIZE(p->supplies), p->supplies);
	if (err < 0)
		return err;

	if (p->power_invert) {
		if (regulator_is_enabled(p->supply) > 0)
			regulator_disable(p->supply);
	} else {
		err = regulator_enable(p->supply);
		if (err < 0)
			return err;
	}

	return 0;
}

static int panel_simple_regulator_disable(struct panel_simple *p)
{
	int err;

	if (p->power_invert) {
		if (!regulator_is_enabled(p->supply)) {
			err = regulator_enable(p->supply);
			if (err < 0)
				return err;
		}
	} else {
		regulator_disable(p->supply);
	}

	regulator_bulk_disable(ARRAY_SIZE(p->supplies), p->supplies);

	return 0;
}

static int panel_simple_loader_protect(struct drm_panel *panel, bool on)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err;

	if (on) {
		err = panel_simple_regulator_enable(p);
		if (err < 0) {
			dev_err(panel->dev, "failed to enable supply: %d\n",
				err);
			return err;
		}

		p->prepared = true;
		p->enabled = true;
	} else {
		/* do nothing */
	}

	return 0;
}

static int panel_simple_disable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err = 0;

	if (!p->enabled)
		return 0;

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_POWERDOWN;
		p->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(p->backlight);
	}

	if (p->desc->delay.disable)
		panel_simple_sleep(p->desc->delay.disable);

	if (p->cmd_type == CMD_TYPE_MCU) {
		err = panel_simple_xfer_mcu_cmd_seq(p, p->desc->exit_seq);
		if (err)
			dev_err(panel->dev, "failed to send exit cmds seq\n");
	}
	p->enabled = false;

	return 0;
}

static int panel_simple_unprepare(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err = 0;

	if (!p->prepared)
		return 0;

	if (p->desc->exit_seq) {
		if (p->dsi)
			panel_simple_xfer_dsi_cmd_seq(p, p->desc->exit_seq);
		else if (p->cmd_type == CMD_TYPE_SPI)
			err = panel_simple_xfer_spi_cmd_seq(p, p->desc->exit_seq);
		if (err)
			dev_err(panel->dev, "failed to send exit cmds seq\n");
	}

	gpiod_direction_output(p->reset_gpio, 1);

	gpiod_direction_output(p->enable_gpio, 0);

	panel_simple_regulator_disable(p);

	if (p->desc->delay.unprepare)
		panel_simple_sleep(p->desc->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int panel_simple_prepare(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err;

	if (p->prepared)
		return 0;

	err = panel_simple_regulator_enable(p);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	gpiod_direction_output(p->enable_gpio, 1);

	if (p->desc->delay.prepare)
		panel_simple_sleep(p->desc->delay.prepare);

	gpiod_direction_output(p->reset_gpio, 1);

	if (p->desc->delay.reset)
		panel_simple_sleep(p->desc->delay.reset);

	gpiod_direction_output(p->reset_gpio, 0);

	if (p->desc->delay.init)
		panel_simple_sleep(p->desc->delay.init);

	if (p->desc->init_seq) {
		if (p->dsi)
			panel_simple_xfer_dsi_cmd_seq(p, p->desc->init_seq);
		else if (p->cmd_type == CMD_TYPE_SPI)
			err = panel_simple_xfer_spi_cmd_seq(p, p->desc->init_seq);
		if (err)
			dev_err(panel->dev, "failed to send init cmds seq\n");
	}

	p->prepared = true;

	return 0;
}

static int panel_simple_enable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err = 0;

	if (p->enabled)
		return 0;

	if (p->cmd_type == CMD_TYPE_MCU) {
		err = panel_simple_xfer_mcu_cmd_seq(p, p->desc->init_seq);
		if (err)
			dev_err(panel->dev, "failed to send init cmds seq\n");
	}
	if (p->desc->delay.enable)
		panel_simple_sleep(p->desc->delay.enable);

	if (p->backlight) {
		p->backlight->props.state &= ~BL_CORE_FBBLANK;
		p->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(p->backlight);
	}

	p->enabled = true;

	return 0;
}

static int panel_simple_get_modes(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int num = 0;

	/* probe EDID if a DDC bus is available */
	if (p->ddc) {
		struct edid *edid = drm_get_edid(panel->connector, p->ddc);
		drm_connector_update_edid_property(panel->connector, edid);
		if (edid) {
			num += drm_add_edid_modes(panel->connector, edid);
			kfree(edid);
		}
	}

	/* add hard-coded panel modes */
	num += panel_simple_get_fixed_modes(p);

	return num;
}

static int panel_simple_get_timings(struct drm_panel *panel,
				    unsigned int num_timings,
				    struct display_timing *timings)
{
	struct panel_simple *p = to_panel_simple(panel);
	unsigned int i;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static const struct drm_panel_funcs panel_simple_funcs = {
	.loader_protect = panel_simple_loader_protect,
	.disable = panel_simple_disable,
	.unprepare = panel_simple_unprepare,
	.prepare = panel_simple_prepare,
	.enable = panel_simple_enable,
	.get_modes = panel_simple_get_modes,
	.get_timings = panel_simple_get_timings,
};

static int panel_simple_probe(struct device *dev, const struct panel_desc *desc)
{
	struct device_node *backlight, *ddc;
	struct panel_simple *panel;
	const char *cmd_type;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;
	panel->desc = desc;

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->supplies[0].supply = "vsp";
	panel->supplies[1].supply = "vsn";

	err = devm_regulator_bulk_get(dev, ARRAY_SIZE(panel->supplies),
				      panel->supplies);
	if (err)
		return err;

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_ASIS);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "failed to get enable GPIO: %d\n", err);
		return err;
	}

	panel->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(panel->reset_gpio)) {
		err = PTR_ERR(panel->reset_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "failed to get reset GPIO: %d\n", err);
		return err;
	}

	if (of_property_read_string(dev->of_node, "rockchip,cmd-type",
				    &cmd_type))
		panel->cmd_type = CMD_TYPE_DEFAULT;
	else
		panel->cmd_type = get_panel_cmd_type(cmd_type);

	if (panel->cmd_type == CMD_TYPE_SPI) {
		panel->spi_sdi_gpio =
				devm_gpiod_get_optional(dev, "spi-sdi", 0);
		if (IS_ERR(panel->spi_sdi_gpio)) {
			err = PTR_ERR(panel->spi_sdi_gpio);
			dev_err(dev, "failed to request spi_sdi: %d\n", err);
			return err;
		}

		panel->spi_scl_gpio =
				devm_gpiod_get_optional(dev, "spi-scl", 0);
		if (IS_ERR(panel->spi_scl_gpio)) {
			err = PTR_ERR(panel->spi_scl_gpio);
			dev_err(dev, "failed to request spi_scl: %d\n", err);
			return err;
		}

		panel->spi_cs_gpio = devm_gpiod_get_optional(dev, "spi-cs", 0);
		if (IS_ERR(panel->spi_cs_gpio)) {
			err = PTR_ERR(panel->spi_cs_gpio);
			dev_err(dev, "failed to request spi_cs: %d\n", err);
			return err;
		}
		gpiod_direction_output(panel->spi_cs_gpio, 1);
		gpiod_direction_output(panel->spi_sdi_gpio, 1);
		gpiod_direction_output(panel->spi_scl_gpio, 1);
	} else if (panel->cmd_type == CMD_TYPE_MCU) {
		struct device_node *port, *endpoint;
		struct device_node *np;

		port = of_graph_get_port_by_id(dev->of_node, 0);
		if (port) {
			endpoint = of_get_next_child(port, NULL);
			/* get connect device node */
			np = of_graph_get_remote_port_parent(endpoint);

			port = of_graph_get_port_by_id(np, 0);
			if (port) {
				endpoint = of_get_next_child(port, NULL);
				/* get crtc device node */
				np = of_graph_get_remote_port_parent(endpoint);
				panel->np_crtc = np;
			}
		}
	}
	panel->power_invert =
			of_property_read_bool(dev->of_node, "power-invert");

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		panel->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!panel->backlight)
			return -EPROBE_DEFER;
	}

	ddc = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
	if (ddc) {
		panel->ddc = of_find_i2c_adapter_by_node(ddc);
		of_node_put(ddc);

		if (!panel->ddc) {
			err = -EPROBE_DEFER;
			goto free_backlight;
		}
	}

	drm_panel_init(&panel->base);
	panel->base.dev = dev;
	panel->base.funcs = &panel_simple_funcs;

	err = drm_panel_add(&panel->base);
	if (err < 0)
		goto free_ddc;

	dev_set_drvdata(dev, panel);

	return 0;

free_ddc:
	if (panel->ddc)
		put_device(&panel->ddc->dev);
free_backlight:
	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return err;
}

static int panel_simple_remove(struct device *dev)
{
	struct panel_simple *panel = dev_get_drvdata(dev);

	drm_panel_remove(&panel->base);

	panel_simple_disable(&panel->base);
	panel_simple_unprepare(&panel->base);

	if (panel->ddc)
		put_device(&panel->ddc->dev);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return 0;
}

static void panel_simple_shutdown(struct device *dev)
{
	struct panel_simple *panel = dev_get_drvdata(dev);

	panel_simple_disable(&panel->base);

	if (panel->prepared) {
		gpiod_direction_output(panel->reset_gpio, 1);
		gpiod_direction_output(panel->enable_gpio, 0);
		panel_simple_regulator_disable(panel);
	}
}

static const struct drm_display_mode ampire_am_480272h3tmqw_t01h_mode = {
	.clock = 9000,
	.hdisplay = 480,
	.hsync_start = 480 + 2,
	.hsync_end = 480 + 2 + 41,
	.htotal = 480 + 2 + 41 + 2,
	.vdisplay = 272,
	.vsync_start = 272 + 2,
	.vsync_end = 272 + 2 + 10,
	.vtotal = 272 + 2 + 10 + 2,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

static const struct panel_desc ampire_am_480272h3tmqw_t01h = {
	.modes = &ampire_am_480272h3tmqw_t01h_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 105,
		.height = 67,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode ampire_am800480r3tmqwa1h_mode = {
	.clock = 33333,
	.hdisplay = 800,
	.hsync_start = 800 + 0,
	.hsync_end = 800 + 0 + 255,
	.htotal = 800 + 0 + 255 + 0,
	.vdisplay = 480,
	.vsync_start = 480 + 2,
	.vsync_end = 480 + 2 + 45,
	.vtotal = 480 + 2 + 45 + 0,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

static const struct panel_desc ampire_am800480r3tmqwa1h = {
	.modes = &ampire_am800480r3tmqwa1h_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct display_timing santek_st0700i5y_rbslw_f_timing = {
	.pixelclock = { 26400000, 33300000, 46800000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 16, 210, 354 },
	.hback_porch = { 45, 36, 6 },
	.hsync_len = { 1, 10, 40 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 7, 22, 147 },
	.vback_porch = { 22, 13, 3 },
	.vsync_len = { 1, 10, 20 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW |
		DISPLAY_FLAGS_DE_HIGH | DISPLAY_FLAGS_PIXDATA_POSEDGE
};

static const struct panel_desc armadeus_st0700_adapt = {
	.timings = &santek_st0700i5y_rbslw_f_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 86,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode auo_b101aw03_mode = {
	.clock = 51450,
	.hdisplay = 1024,
	.hsync_start = 1024 + 156,
	.hsync_end = 1024 + 156 + 8,
	.htotal = 1024 + 156 + 8 + 156,
	.vdisplay = 600,
	.vsync_start = 600 + 16,
	.vsync_end = 600 + 16 + 6,
	.vtotal = 600 + 16 + 6 + 16,
	.vrefresh = 60,
};

static const struct panel_desc auo_b101aw03 = {
	.modes = &auo_b101aw03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode auo_b101ean01_mode = {
	.clock = 72500,
	.hdisplay = 1280,
	.hsync_start = 1280 + 119,
	.hsync_end = 1280 + 119 + 32,
	.htotal = 1280 + 119 + 32 + 21,
	.vdisplay = 800,
	.vsync_start = 800 + 4,
	.vsync_end = 800 + 4 + 20,
	.vtotal = 800 + 4 + 20 + 8,
	.vrefresh = 60,
};

static const struct panel_desc auo_b101ean01 = {
	.modes = &auo_b101ean01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 217,
		.height = 136,
	},
};

static const struct drm_display_mode auo_b101xtn01_mode = {
	.clock = 72000,
	.hdisplay = 1366,
	.hsync_start = 1366 + 20,
	.hsync_end = 1366 + 20 + 70,
	.htotal = 1366 + 20 + 70,
	.vdisplay = 768,
	.vsync_start = 768 + 14,
	.vsync_end = 768 + 14 + 42,
	.vtotal = 768 + 14 + 42,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc auo_b101xtn01 = {
	.modes = &auo_b101xtn01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode auo_b116xw03_mode = {
	.clock = 70589,
	.hdisplay = 1366,
	.hsync_start = 1366 + 40,
	.hsync_end = 1366 + 40 + 40,
	.htotal = 1366 + 40 + 40 + 32,
	.vdisplay = 768,
	.vsync_start = 768 + 10,
	.vsync_end = 768 + 10 + 12,
	.vtotal = 768 + 10 + 12 + 6,
	.vrefresh = 60,
};

static const struct panel_desc auo_b116xw03 = {
	.modes = &auo_b116xw03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 256,
		.height = 144,
	},
};

static const struct drm_display_mode auo_b133xtn01_mode = {
	.clock = 69500,
	.hdisplay = 1366,
	.hsync_start = 1366 + 48,
	.hsync_end = 1366 + 48 + 32,
	.htotal = 1366 + 48 + 32 + 20,
	.vdisplay = 768,
	.vsync_start = 768 + 3,
	.vsync_end = 768 + 3 + 6,
	.vtotal = 768 + 3 + 6 + 13,
	.vrefresh = 60,
};

static const struct panel_desc auo_b133xtn01 = {
	.modes = &auo_b133xtn01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 293,
		.height = 165,
	},
};

static const struct drm_display_mode auo_b133htn01_mode = {
	.clock = 150660,
	.hdisplay = 1920,
	.hsync_start = 1920 + 172,
	.hsync_end = 1920 + 172 + 80,
	.htotal = 1920 + 172 + 80 + 60,
	.vdisplay = 1080,
	.vsync_start = 1080 + 25,
	.vsync_end = 1080 + 25 + 10,
	.vtotal = 1080 + 25 + 10 + 10,
	.vrefresh = 60,
};

static const struct panel_desc auo_b133htn01 = {
	.modes = &auo_b133htn01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 293,
		.height = 165,
	},
	.delay = {
		.prepare = 105,
		.enable = 20,
		.unprepare = 50,
	},
};

static const struct display_timing auo_g070vvn01_timings = {
	.pixelclock = { 33300000, 34209000, 45000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 20, 40, 200 },
	.hback_porch = { 87, 40, 1 },
	.hsync_len = { 1, 48, 87 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 5, 13, 200 },
	.vback_porch = { 31, 31, 29 },
	.vsync_len = { 1, 1, 3 },
};

static const struct panel_desc auo_g070vvn01 = {
	.timings = &auo_g070vvn01_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 152,
		.height = 91,
	},
	.delay = {
		.prepare = 200,
		.enable = 50,
		.disable = 50,
		.unprepare = 1000,
	},
};

static const struct drm_display_mode auo_g104sn02_mode = {
	.clock = 40000,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 216,
	.htotal = 800 + 40 + 216 + 128,
	.vdisplay = 600,
	.vsync_start = 600 + 10,
	.vsync_end = 600 + 10 + 35,
	.vtotal = 600 + 10 + 35 + 2,
	.vrefresh = 60,
};

static const struct panel_desc auo_g104sn02 = {
	.modes = &auo_g104sn02_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 211,
		.height = 158,
	},
};

static const struct display_timing auo_g133han01_timings = {
	.pixelclock = { 134000000, 141200000, 149000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 39, 58, 77 },
	.hback_porch = { 59, 88, 117 },
	.hsync_len = { 28, 42, 56 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 3, 8, 11 },
	.vback_porch = { 5, 14, 19 },
	.vsync_len = { 4, 14, 19 },
};

static const struct panel_desc auo_g133han01 = {
	.timings = &auo_g133han01_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 293,
		.height = 165,
	},
	.delay = {
		.prepare = 200,
		.enable = 50,
		.disable = 50,
		.unprepare = 1000,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,
};

static const struct display_timing auo_g185han01_timings = {
	.pixelclock = { 120000000, 144000000, 175000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 36, 120, 148 },
	.hback_porch = { 24, 88, 108 },
	.hsync_len = { 20, 48, 64 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 6, 10, 40 },
	.vback_porch = { 2, 5, 20 },
	.vsync_len = { 2, 5, 20 },
};

static const struct panel_desc auo_g185han01 = {
	.timings = &auo_g185han01_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 409,
		.height = 230,
	},
	.delay = {
		.prepare = 50,
		.enable = 200,
		.disable = 110,
		.unprepare = 1000,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing auo_p320hvn03_timings = {
	.pixelclock = { 106000000, 148500000, 164000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 25, 50, 130 },
	.hback_porch = { 25, 50, 130 },
	.hsync_len = { 20, 40, 105 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 8, 17, 150 },
	.vback_porch = { 8, 17, 150 },
	.vsync_len = { 4, 11, 100 },
};

static const struct panel_desc auo_p320hvn03 = {
	.timings = &auo_p320hvn03_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 698,
		.height = 393,
	},
	.delay = {
		.prepare = 1,
		.enable = 450,
		.unprepare = 500,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode auo_t215hvn01_mode = {
	.clock = 148800,
	.hdisplay = 1920,
	.hsync_start = 1920 + 88,
	.hsync_end = 1920 + 88 + 44,
	.htotal = 1920 + 88 + 44 + 148,
	.vdisplay = 1080,
	.vsync_start = 1080 + 4,
	.vsync_end = 1080 + 4 + 5,
	.vtotal = 1080 + 4 + 5 + 36,
	.vrefresh = 60,
};

static const struct panel_desc auo_t215hvn01 = {
	.modes = &auo_t215hvn01_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 430,
		.height = 270,
	},
	.delay = {
		.disable = 5,
		.unprepare = 1000,
	}
};

static const struct drm_display_mode avic_tm070ddh03_mode = {
	.clock = 51200,
	.hdisplay = 1024,
	.hsync_start = 1024 + 160,
	.hsync_end = 1024 + 160 + 4,
	.htotal = 1024 + 160 + 4 + 156,
	.vdisplay = 600,
	.vsync_start = 600 + 17,
	.vsync_end = 600 + 17 + 1,
	.vtotal = 600 + 17 + 1 + 17,
	.vrefresh = 60,
};

static const struct panel_desc avic_tm070ddh03 = {
	.modes = &avic_tm070ddh03_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 154,
		.height = 90,
	},
	.delay = {
		.prepare = 20,
		.enable = 200,
		.disable = 200,
	},
};

static const struct drm_display_mode boe_hv070wsa_mode = {
	.clock = 40800,
	.hdisplay = 1024,
	.hsync_start = 1024 + 90,
	.hsync_end = 1024 + 90 + 90,
	.htotal = 1024 + 90 + 90 + 90,
	.vdisplay = 600,
	.vsync_start = 600 + 3,
	.vsync_end = 600 + 3 + 4,
	.vtotal = 600 + 3 + 4 + 3,
	.vrefresh = 60,
};

static const struct panel_desc boe_hv070wsa = {
	.modes = &boe_hv070wsa_mode,
	.num_modes = 1,
	.size = {
		.width = 154,
		.height = 90,
	},
};

static const struct drm_display_mode boe_nv101wxmn51_modes[] = {
	{
		.clock = 71900,
		.hdisplay = 1280,
		.hsync_start = 1280 + 48,
		.hsync_end = 1280 + 48 + 32,
		.htotal = 1280 + 48 + 32 + 80,
		.vdisplay = 800,
		.vsync_start = 800 + 3,
		.vsync_end = 800 + 3 + 5,
		.vtotal = 800 + 3 + 5 + 24,
		.vrefresh = 60,
	},
	{
		.clock = 57500,
		.hdisplay = 1280,
		.hsync_start = 1280 + 48,
		.hsync_end = 1280 + 48 + 32,
		.htotal = 1280 + 48 + 32 + 80,
		.vdisplay = 800,
		.vsync_start = 800 + 3,
		.vsync_end = 800 + 3 + 5,
		.vtotal = 800 + 3 + 5 + 24,
		.vrefresh = 48,
	},
};

static const struct panel_desc boe_nv101wxmn51 = {
	.modes = boe_nv101wxmn51_modes,
	.num_modes = ARRAY_SIZE(boe_nv101wxmn51_modes),
	.bpc = 8,
	.size = {
		.width = 217,
		.height = 136,
	},
	.delay = {
		.prepare = 210,
		.enable = 50,
		.unprepare = 160,
	},
};

static const struct drm_display_mode chunghwa_claa070wp03xg_mode = {
	.clock = 66770,
	.hdisplay = 800,
	.hsync_start = 800 + 49,
	.hsync_end = 800 + 49 + 33,
	.htotal = 800 + 49 + 33 + 17,
	.vdisplay = 1280,
	.vsync_start = 1280 + 1,
	.vsync_end = 1280 + 1 + 7,
	.vtotal = 1280 + 1 + 7 + 15,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc chunghwa_claa070wp03xg = {
	.modes = &chunghwa_claa070wp03xg_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 94,
		.height = 150,
	},
};

static const struct drm_display_mode chunghwa_claa101wa01a_mode = {
	.clock = 72070,
	.hdisplay = 1366,
	.hsync_start = 1366 + 58,
	.hsync_end = 1366 + 58 + 58,
	.htotal = 1366 + 58 + 58 + 58,
	.vdisplay = 768,
	.vsync_start = 768 + 4,
	.vsync_end = 768 + 4 + 4,
	.vtotal = 768 + 4 + 4 + 4,
	.vrefresh = 60,
};

static const struct panel_desc chunghwa_claa101wa01a = {
	.modes = &chunghwa_claa101wa01a_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 220,
		.height = 120,
	},
};

static const struct drm_display_mode chunghwa_claa101wb01_mode = {
	.clock = 69300,
	.hdisplay = 1366,
	.hsync_start = 1366 + 48,
	.hsync_end = 1366 + 48 + 32,
	.htotal = 1366 + 48 + 32 + 20,
	.vdisplay = 768,
	.vsync_start = 768 + 16,
	.vsync_end = 768 + 16 + 8,
	.vtotal = 768 + 16 + 8 + 16,
	.vrefresh = 60,
};

static const struct panel_desc chunghwa_claa101wb01 = {
	.modes = &chunghwa_claa101wb01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode dataimage_scf0700c48ggu18_mode = {
	.clock = 33260,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 128,
	.htotal = 800 + 40 + 128 + 88,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 2,
	.vtotal = 480 + 10 + 2 + 33,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc dataimage_scf0700c48ggu18 = {
	.modes = &dataimage_scf0700c48ggu18_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct display_timing dlc_dlc0700yzg_1_timing = {
	.pixelclock = { 45000000, 51200000, 57000000 },
	.hactive = { 1024, 1024, 1024 },
	.hfront_porch = { 100, 106, 113 },
	.hback_porch = { 100, 106, 113 },
	.hsync_len = { 100, 108, 114 },
	.vactive = { 600, 600, 600 },
	.vfront_porch = { 8, 11, 15 },
	.vback_porch = { 8, 11, 15 },
	.vsync_len = { 9, 13, 15 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc dlc_dlc0700yzg_1 = {
	.timings = &dlc_dlc0700yzg_1_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 86,
	},
	.delay = {
		.prepare = 30,
		.enable = 200,
		.disable = 200,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct drm_display_mode edt_et057090dhu_mode = {
	.clock = 25175,
	.hdisplay = 640,
	.hsync_start = 640 + 16,
	.hsync_end = 640 + 16 + 30,
	.htotal = 640 + 16 + 30 + 114,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 3,
	.vtotal = 480 + 10 + 3 + 32,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc edt_et057090dhu = {
	.modes = &edt_et057090dhu_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 115,
		.height = 86,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_NEGEDGE,
};

static const struct drm_display_mode edt_etm0700g0dh6_mode = {
	.clock = 33260,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 128,
	.htotal = 800 + 40 + 128 + 88,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 2,
	.vtotal = 480 + 10 + 2 + 33,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct panel_desc edt_etm0700g0dh6 = {
	.modes = &edt_etm0700g0dh6_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_NEGEDGE,
};

static const struct panel_desc edt_etm0700g0bdh6 = {
	.modes = &edt_etm0700g0dh6_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode foxlink_fl500wvr00_a0t_mode = {
	.clock = 32260,
	.hdisplay = 800,
	.hsync_start = 800 + 168,
	.hsync_end = 800 + 168 + 64,
	.htotal = 800 + 168 + 64 + 88,
	.vdisplay = 480,
	.vsync_start = 480 + 37,
	.vsync_end = 480 + 37 + 2,
	.vtotal = 480 + 37 + 2 + 8,
	.vrefresh = 60,
};

static const struct panel_desc foxlink_fl500wvr00_a0t = {
	.modes = &foxlink_fl500wvr00_a0t_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 108,
		.height = 65,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode giantplus_gpg482739qs5_mode = {
	.clock = 9000,
	.hdisplay = 480,
	.hsync_start = 480 + 5,
	.hsync_end = 480 + 5 + 1,
	.htotal = 480 + 5 + 1 + 40,
	.vdisplay = 272,
	.vsync_start = 272 + 8,
	.vsync_end = 272 + 8 + 1,
	.vtotal = 272 + 8 + 1 + 8,
	.vrefresh = 60,
};

static const struct panel_desc giantplus_gpg482739qs5 = {
	.modes = &giantplus_gpg482739qs5_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct display_timing hannstar_hsd070pww1_timing = {
	.pixelclock = { 64300000, 71100000, 82000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 1, 1, 10 },
	.hback_porch = { 1, 1, 10 },
	/*
	 * According to the data sheet, the minimum horizontal blanking interval
	 * is 54 clocks (1 + 52 + 1), but tests with a Nitrogen6X have shown the
	 * minimum working horizontal blanking interval to be 60 clocks.
	 */
	.hsync_len = { 58, 158, 661 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 1, 1, 10 },
	.vback_porch = { 1, 1, 10 },
	.vsync_len = { 1, 21, 203 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc hannstar_hsd070pww1 = {
	.timings = &hannstar_hsd070pww1_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 151,
		.height = 94,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct display_timing hannstar_hsd100pxn1_timing = {
	.pixelclock = { 55000000, 65000000, 75000000 },
	.hactive = { 1024, 1024, 1024 },
	.hfront_porch = { 40, 40, 40 },
	.hback_porch = { 220, 220, 220 },
	.hsync_len = { 20, 60, 100 },
	.vactive = { 768, 768, 768 },
	.vfront_porch = { 7, 7, 7 },
	.vback_porch = { 21, 21, 21 },
	.vsync_len = { 10, 10, 10 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc hannstar_hsd100pxn1 = {
	.timings = &hannstar_hsd100pxn1_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 203,
		.height = 152,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct drm_display_mode hitachi_tx23d38vm0caa_mode = {
	.clock = 33333,
	.hdisplay = 800,
	.hsync_start = 800 + 85,
	.hsync_end = 800 + 85 + 86,
	.htotal = 800 + 85 + 86 + 85,
	.vdisplay = 480,
	.vsync_start = 480 + 16,
	.vsync_end = 480 + 16 + 13,
	.vtotal = 480 + 16 + 13 + 16,
	.vrefresh = 60,
};

static const struct panel_desc hitachi_tx23d38vm0caa = {
	.modes = &hitachi_tx23d38vm0caa_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 195,
		.height = 117,
	},
	.delay = {
		.enable = 160,
		.disable = 160,
	},
};

static const struct drm_display_mode innolux_at043tn24_mode = {
	.clock = 9000,
	.hdisplay = 480,
	.hsync_start = 480 + 2,
	.hsync_end = 480 + 2 + 41,
	.htotal = 480 + 2 + 41 + 2,
	.vdisplay = 272,
	.vsync_start = 272 + 2,
	.vsync_end = 272 + 2 + 10,
	.vtotal = 272 + 2 + 10 + 2,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct panel_desc innolux_at043tn24 = {
	.modes = &innolux_at043tn24_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode innolux_at070tn92_mode = {
	.clock = 33333,
	.hdisplay = 800,
	.hsync_start = 800 + 210,
	.hsync_end = 800 + 210 + 20,
	.htotal = 800 + 210 + 20 + 46,
	.vdisplay = 480,
	.vsync_start = 480 + 22,
	.vsync_end = 480 + 22 + 10,
	.vtotal = 480 + 22 + 23 + 10,
	.vrefresh = 60,
};

static const struct panel_desc innolux_at070tn92 = {
	.modes = &innolux_at070tn92_mode,
	.num_modes = 1,
	.size = {
		.width = 154,
		.height = 86,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct display_timing innolux_g070y2_l01_timing = {
	.pixelclock = { 28000000, 29500000, 32000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 61, 91, 141 },
	.hback_porch = { 60, 90, 140 },
	.hsync_len = { 12, 12, 12 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 4, 9, 30 },
	.vback_porch = { 4, 8, 28 },
	.vsync_len = { 2, 2, 2 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc innolux_g070y2_l01 = {
	.timings = &innolux_g070y2_l01_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.delay = {
		.prepare = 10,
		.enable = 100,
		.disable = 100,
		.unprepare = 800,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing innolux_g101ice_l01_timing = {
	.pixelclock = { 60400000, 71100000, 74700000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 41, 80, 100 },
	.hback_porch = { 40, 79, 99 },
	.hsync_len = { 1, 1, 1 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 5, 11, 14 },
	.vback_porch = { 4, 11, 14 },
	.vsync_len = { 1, 1, 1 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc innolux_g101ice_l01 = {
	.timings = &innolux_g101ice_l01_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 217,
		.height = 135,
	},
	.delay = {
		.enable = 200,
		.disable = 200,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing innolux_g121i1_l01_timing = {
	.pixelclock = { 67450000, 71000000, 74550000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 40, 80, 160 },
	.hback_porch = { 39, 79, 159 },
	.hsync_len = { 1, 1, 1 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 5, 11, 100 },
	.vback_porch = { 4, 11, 99 },
	.vsync_len = { 1, 1, 1 },
};

static const struct panel_desc innolux_g121i1_l01 = {
	.timings = &innolux_g121i1_l01_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 261,
		.height = 163,
	},
	.delay = {
		.enable = 200,
		.disable = 20,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode innolux_g121x1_l03_mode = {
	.clock = 65000,
	.hdisplay = 1024,
	.hsync_start = 1024 + 0,
	.hsync_end = 1024 + 1,
	.htotal = 1024 + 0 + 1 + 320,
	.vdisplay = 768,
	.vsync_start = 768 + 38,
	.vsync_end = 768 + 38 + 1,
	.vtotal = 768 + 38 + 1 + 0,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct panel_desc innolux_g121x1_l03 = {
	.modes = &innolux_g121x1_l03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 246,
		.height = 185,
	},
	.delay = {
		.enable = 200,
		.unprepare = 200,
		.disable = 400,
	},
};

static const struct drm_display_mode innolux_n116bge_mode = {
	.clock = 76420,
	.hdisplay = 1366,
	.hsync_start = 1366 + 136,
	.hsync_end = 1366 + 136 + 30,
	.htotal = 1366 + 136 + 30 + 60,
	.vdisplay = 768,
	.vsync_start = 768 + 8,
	.vsync_end = 768 + 8 + 12,
	.vtotal = 768 + 8 + 12 + 12,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct panel_desc innolux_n116bge = {
	.modes = &innolux_n116bge_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 256,
		.height = 144,
	},
};

static const struct drm_display_mode innolux_n156bge_l21_mode = {
	.clock = 69300,
	.hdisplay = 1366,
	.hsync_start = 1366 + 16,
	.hsync_end = 1366 + 16 + 34,
	.htotal = 1366 + 16 + 34 + 50,
	.vdisplay = 768,
	.vsync_start = 768 + 2,
	.vsync_end = 768 + 2 + 6,
	.vtotal = 768 + 2 + 6 + 12,
	.vrefresh = 60,
};

static const struct panel_desc innolux_n156bge_l21 = {
	.modes = &innolux_n156bge_l21_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 344,
		.height = 193,
	},
};

static const struct drm_display_mode innolux_tv123wam_mode = {
	.clock = 206016,
	.hdisplay = 2160,
	.hsync_start = 2160 + 48,
	.hsync_end = 2160 + 48 + 32,
	.htotal = 2160 + 48 + 32 + 80,
	.vdisplay = 1440,
	.vsync_start = 1440 + 3,
	.vsync_end = 1440 + 3 + 10,
	.vtotal = 1440 + 3 + 10 + 27,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

static const struct panel_desc innolux_tv123wam = {
	.modes = &innolux_tv123wam_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 259,
		.height = 173,
	},
	.delay = {
		.unprepare = 500,
	},
};

static const struct drm_display_mode innolux_zj070na_01p_mode = {
	.clock = 51501,
	.hdisplay = 1024,
	.hsync_start = 1024 + 128,
	.hsync_end = 1024 + 128 + 64,
	.htotal = 1024 + 128 + 64 + 128,
	.vdisplay = 600,
	.vsync_start = 600 + 16,
	.vsync_end = 600 + 16 + 4,
	.vtotal = 600 + 16 + 4 + 16,
	.vrefresh = 60,
};

static const struct panel_desc innolux_zj070na_01p = {
	.modes = &innolux_zj070na_01p_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 90,
	},
};

static const struct display_timing koe_tx31d200vm0baa_timing = {
	.pixelclock = { 39600000, 43200000, 48000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 16, 36, 56 },
	.hback_porch = { 16, 36, 56 },
	.hsync_len = { 8, 8, 8 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 6, 21, 33 },
	.vback_porch = { 6, 21, 33 },
	.vsync_len = { 8, 8, 8 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc koe_tx31d200vm0baa = {
	.timings = &koe_tx31d200vm0baa_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 292,
		.height = 109,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct display_timing kyo_tcg121xglp_timing = {
	.pixelclock = { 52000000, 65000000, 71000000 },
	.hactive = { 1024, 1024, 1024 },
	.hfront_porch = { 2, 2, 2 },
	.hback_porch = { 2, 2, 2 },
	.hsync_len = { 86, 124, 244 },
	.vactive = { 768, 768, 768 },
	.vfront_porch = { 2, 2, 2 },
	.vback_porch = { 2, 2, 2 },
	.vsync_len = { 6, 34, 73 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc kyo_tcg121xglp = {
	.timings = &kyo_tcg121xglp_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 246,
		.height = 184,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode lg_lb070wv8_mode = {
	.clock = 33246,
	.hdisplay = 800,
	.hsync_start = 800 + 88,
	.hsync_end = 800 + 88 + 80,
	.htotal = 800 + 88 + 80 + 88,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 25,
	.vtotal = 480 + 10 + 25 + 10,
	.vrefresh = 60,
};

static const struct panel_desc lg_lb070wv8 = {
	.modes = &lg_lb070wv8_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 151,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode lg_lp079qx1_sp0v_mode = {
	.clock = 200000,
	.hdisplay = 1536,
	.hsync_start = 1536 + 12,
	.hsync_end = 1536 + 12 + 16,
	.htotal = 1536 + 12 + 16 + 48,
	.vdisplay = 2048,
	.vsync_start = 2048 + 8,
	.vsync_end = 2048 + 8 + 4,
	.vtotal = 2048 + 8 + 4 + 8,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc lg_lp079qx1_sp0v = {
	.modes = &lg_lp079qx1_sp0v_mode,
	.num_modes = 1,
	.size = {
		.width = 129,
		.height = 171,
	},
};

static const struct drm_display_mode lg_lp097qx1_spa1_mode = {
	.clock = 205210,
	.hdisplay = 2048,
	.hsync_start = 2048 + 150,
	.hsync_end = 2048 + 150 + 5,
	.htotal = 2048 + 150 + 5 + 5,
	.vdisplay = 1536,
	.vsync_start = 1536 + 3,
	.vsync_end = 1536 + 3 + 1,
	.vtotal = 1536 + 3 + 1 + 9,
	.vrefresh = 60,
};

static const struct panel_desc lg_lp097qx1_spa1 = {
	.modes = &lg_lp097qx1_spa1_mode,
	.num_modes = 1,
	.size = {
		.width = 208,
		.height = 147,
	},
};

static const struct drm_display_mode lg_lp120up1_mode = {
	.clock = 162300,
	.hdisplay = 1920,
	.hsync_start = 1920 + 40,
	.hsync_end = 1920 + 40 + 40,
	.htotal = 1920 + 40 + 40+ 80,
	.vdisplay = 1280,
	.vsync_start = 1280 + 4,
	.vsync_end = 1280 + 4 + 4,
	.vtotal = 1280 + 4 + 4 + 12,
	.vrefresh = 60,
};

static const struct panel_desc lg_lp120up1 = {
	.modes = &lg_lp120up1_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 267,
		.height = 183,
	},
};

static const struct drm_display_mode lg_lp129qe_mode = {
	.clock = 285250,
	.hdisplay = 2560,
	.hsync_start = 2560 + 48,
	.hsync_end = 2560 + 48 + 32,
	.htotal = 2560 + 48 + 32 + 80,
	.vdisplay = 1700,
	.vsync_start = 1700 + 3,
	.vsync_end = 1700 + 3 + 10,
	.vtotal = 1700 + 3 + 10 + 36,
	.vrefresh = 60,
};

static const struct panel_desc lg_lp129qe = {
	.modes = &lg_lp129qe_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 272,
		.height = 181,
	},
};

static const struct drm_display_mode mitsubishi_aa070mc01_mode = {
	.clock = 30400,
	.hdisplay = 800,
	.hsync_start = 800 + 0,
	.hsync_end = 800 + 1,
	.htotal = 800 + 0 + 1 + 160,
	.vdisplay = 480,
	.vsync_start = 480 + 0,
	.vsync_end = 480 + 48 + 1,
	.vtotal = 480 + 48 + 1 + 0,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct panel_desc mitsubishi_aa070mc01 = {
	.modes = &mitsubishi_aa070mc01_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 152,
		.height = 91,
	},

	.delay = {
		.enable = 200,
		.unprepare = 200,
		.disable = 400,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH,
};

static const struct display_timing nec_nl12880bc20_05_timing = {
	.pixelclock = { 67000000, 71000000, 75000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 2, 30, 30 },
	.hback_porch = { 6, 100, 100 },
	.hsync_len = { 2, 30, 30 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 5, 5, 5 },
	.vback_porch = { 11, 11, 11 },
	.vsync_len = { 7, 7, 7 },
};

static const struct panel_desc nec_nl12880bc20_05 = {
	.timings = &nec_nl12880bc20_05_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 261,
		.height = 163,
	},
	.delay = {
		.enable = 50,
		.disable = 50,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode nec_nl4827hc19_05b_mode = {
	.clock = 10870,
	.hdisplay = 480,
	.hsync_start = 480 + 2,
	.hsync_end = 480 + 2 + 41,
	.htotal = 480 + 2 + 41 + 2,
	.vdisplay = 272,
	.vsync_start = 272 + 2,
	.vsync_end = 272 + 2 + 4,
	.vtotal = 272 + 2 + 4 + 2,
	.vrefresh = 74,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc nec_nl4827hc19_05b = {
	.modes = &nec_nl4827hc19_05b_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode netron_dy_e231732_mode = {
	.clock = 66000,
	.hdisplay = 1024,
	.hsync_start = 1024 + 160,
	.hsync_end = 1024 + 160 + 70,
	.htotal = 1024 + 160 + 70 + 90,
	.vdisplay = 600,
	.vsync_start = 600 + 127,
	.vsync_end = 600 + 127 + 20,
	.vtotal = 600 + 127 + 20 + 3,
	.vrefresh = 60,
};

static const struct panel_desc netron_dy_e231732 = {
	.modes = &netron_dy_e231732_mode,
	.num_modes = 1,
	.size = {
		.width = 154,
		.height = 87,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode newhaven_nhd_43_480272ef_atxl_mode = {
	.clock = 9000,
	.hdisplay = 480,
	.hsync_start = 480 + 2,
	.hsync_end = 480 + 2 + 41,
	.htotal = 480 + 2 + 41 + 2,
	.vdisplay = 272,
	.vsync_start = 272 + 2,
	.vsync_end = 272 + 2 + 10,
	.vtotal = 272 + 2 + 10 + 2,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc newhaven_nhd_43_480272ef_atxl = {
	.modes = &newhaven_nhd_43_480272ef_atxl_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE |
		     DRM_BUS_FLAG_SYNC_POSEDGE,
};

static const struct display_timing nlt_nl192108ac18_02d_timing = {
	.pixelclock = { 130000000, 148350000, 163000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 80, 100, 100 },
	.hback_porch = { 100, 120, 120 },
	.hsync_len = { 50, 60, 60 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 12, 30, 30 },
	.vback_porch = { 4, 10, 10 },
	.vsync_len = { 4, 5, 5 },
};

static const struct panel_desc nlt_nl192108ac18_02d = {
	.timings = &nlt_nl192108ac18_02d_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 344,
		.height = 194,
	},
	.delay = {
		.unprepare = 500,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode nvd_9128_mode = {
	.clock = 29500,
	.hdisplay = 800,
	.hsync_start = 800 + 130,
	.hsync_end = 800 + 130 + 98,
	.htotal = 800 + 0 + 130 + 98,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 50,
	.vtotal = 480 + 0 + 10 + 50,
};

static const struct panel_desc nvd_9128 = {
	.modes = &nvd_9128_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 156,
		.height = 88,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing okaya_rs800480t_7x0gp_timing = {
	.pixelclock = { 30000000, 30000000, 40000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 40, 40, 40 },
	.hback_porch = { 40, 40, 40 },
	.hsync_len = { 1, 48, 48 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 13, 13, 13 },
	.vback_porch = { 29, 29, 29 },
	.vsync_len = { 3, 3, 3 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc okaya_rs800480t_7x0gp = {
	.timings = &okaya_rs800480t_7x0gp_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 87,
	},
	.delay = {
		.prepare = 41,
		.enable = 50,
		.unprepare = 41,
		.disable = 50,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode olimex_lcd_olinuxino_43ts_mode = {
	.clock = 9000,
	.hdisplay = 480,
	.hsync_start = 480 + 5,
	.hsync_end = 480 + 5 + 30,
	.htotal = 480 + 5 + 30 + 10,
	.vdisplay = 272,
	.vsync_start = 272 + 8,
	.vsync_end = 272 + 8 + 5,
	.vtotal = 272 + 8 + 5 + 3,
	.vrefresh = 60,
};

static const struct panel_desc olimex_lcd_olinuxino_43ts = {
	.modes = &olimex_lcd_olinuxino_43ts_mode,
	.num_modes = 1,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

/*
 * 800x480 CVT. The panel appears to be quite accepting, at least as far as
 * pixel clocks, but this is the timing that was being used in the Adafruit
 * installation instructions.
 */
static const struct drm_display_mode ontat_yx700wv03_mode = {
	.clock = 29500,
	.hdisplay = 800,
	.hsync_start = 824,
	.hsync_end = 896,
	.htotal = 992,
	.vdisplay = 480,
	.vsync_start = 483,
	.vsync_end = 493,
	.vtotal = 500,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

/*
 * Specification at:
 * https://www.adafruit.com/images/product-files/2406/c3163.pdf
 */
static const struct panel_desc ontat_yx700wv03 = {
	.modes = &ontat_yx700wv03_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 154,
		.height = 83,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode ortustech_com43h4m85ulc_mode  = {
	.clock = 25000,
	.hdisplay = 480,
	.hsync_start = 480 + 10,
	.hsync_end = 480 + 10 + 10,
	.htotal = 480 + 10 + 10 + 15,
	.vdisplay = 800,
	.vsync_start = 800 + 3,
	.vsync_end = 800 + 3 + 3,
	.vtotal = 800 + 3 + 3 + 3,
	.vrefresh = 60,
};

static const struct panel_desc ortustech_com43h4m85ulc = {
	.modes = &ortustech_com43h4m85ulc_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 56,
		.height = 93,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode qd43003c0_40_mode = {
	.clock = 9000,
	.hdisplay = 480,
	.hsync_start = 480 + 8,
	.hsync_end = 480 + 8 + 4,
	.htotal = 480 + 8 + 4 + 39,
	.vdisplay = 272,
	.vsync_start = 272 + 4,
	.vsync_end = 272 + 4 + 10,
	.vtotal = 272 + 4 + 10 + 2,
	.vrefresh = 60,
};

static const struct panel_desc qd43003c0_40 = {
	.modes = &qd43003c0_40_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 53,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct display_timing rocktech_rk070er9427_timing = {
	.pixelclock = { 26400000, 33300000, 46800000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 16, 210, 354 },
	.hback_porch = { 46, 46, 46 },
	.hsync_len = { 1, 1, 1 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 7, 22, 147 },
	.vback_porch = { 23, 23, 23 },
	.vsync_len = { 1, 1, 1 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc rocktech_rk070er9427 = {
	.timings = &rocktech_rk070er9427_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 86,
	},
	.delay = {
		.prepare = 41,
		.enable = 50,
		.unprepare = 41,
		.disable = 50,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode samsung_lsn122dl01_c01_mode = {
	.clock = 271560,
	.hdisplay = 2560,
	.hsync_start = 2560 + 48,
	.hsync_end = 2560 + 48 + 32,
	.htotal = 2560 + 48 + 32 + 80,
	.vdisplay = 1600,
	.vsync_start = 1600 + 2,
	.vsync_end = 1600 + 2 + 5,
	.vtotal = 1600 + 2 + 5 + 57,
	.vrefresh = 60,
};

static const struct panel_desc samsung_lsn122dl01_c01 = {
	.modes = &samsung_lsn122dl01_c01_mode,
	.num_modes = 1,
	.size = {
		.width = 263,
		.height = 164,
	},
};

static const struct drm_display_mode samsung_ltn101nt05_mode = {
	.clock = 54030,
	.hdisplay = 1024,
	.hsync_start = 1024 + 24,
	.hsync_end = 1024 + 24 + 136,
	.htotal = 1024 + 24 + 136 + 160,
	.vdisplay = 600,
	.vsync_start = 600 + 3,
	.vsync_end = 600 + 3 + 6,
	.vtotal = 600 + 3 + 6 + 61,
	.vrefresh = 60,
};

static const struct panel_desc samsung_ltn101nt05 = {
	.modes = &samsung_ltn101nt05_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode samsung_ltn140at29_301_mode = {
	.clock = 76300,
	.hdisplay = 1366,
	.hsync_start = 1366 + 64,
	.hsync_end = 1366 + 64 + 48,
	.htotal = 1366 + 64 + 48 + 128,
	.vdisplay = 768,
	.vsync_start = 768 + 2,
	.vsync_end = 768 + 2 + 5,
	.vtotal = 768 + 2 + 5 + 17,
	.vrefresh = 60,
};

static const struct panel_desc samsung_ltn140at29_301 = {
	.modes = &samsung_ltn140at29_301_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 320,
		.height = 187,
	},
};

static const struct drm_display_mode sharp_lq035q7db03_mode = {
	.clock = 5500,
	.hdisplay = 240,
	.hsync_start = 240 + 16,
	.hsync_end = 240 + 16 + 7,
	.htotal = 240 + 16 + 7 + 5,
	.vdisplay = 320,
	.vsync_start = 320 + 9,
	.vsync_end = 320 + 9 + 1,
	.vtotal = 320 + 9 + 1 + 7,
	.vrefresh = 60,
};

static const struct panel_desc sharp_lq035q7db03 = {
	.modes = &sharp_lq035q7db03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 54,
		.height = 72,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct display_timing sharp_lq101k1ly04_timing = {
	.pixelclock = { 60000000, 65000000, 80000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 20, 20, 20 },
	.hback_porch = { 20, 20, 20 },
	.hsync_len = { 10, 10, 10 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 4, 4, 4 },
	.vback_porch = { 4, 4, 4 },
	.vsync_len = { 4, 4, 4 },
	.flags = DISPLAY_FLAGS_PIXDATA_POSEDGE,
};

static const struct panel_desc sharp_lq101k1ly04 = {
	.timings = &sharp_lq101k1ly04_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 217,
		.height = 136,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,
};

static const struct display_timing sharp_lq123p1jx31_timing = {
	.pixelclock = { 252750000, 252750000, 266604720 },
	.hactive = { 2400, 2400, 2400 },
	.hfront_porch = { 48, 48, 48 },
	.hback_porch = { 80, 80, 84 },
	.hsync_len = { 32, 32, 32 },
	.vactive = { 1600, 1600, 1600 },
	.vfront_porch = { 3, 3, 3 },
	.vback_porch = { 33, 33, 120 },
	.vsync_len = { 10, 10, 10 },
	.flags = DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_LOW,
};

static const struct panel_desc sharp_lq123p1jx31 = {
	.timings = &sharp_lq123p1jx31_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 259,
		.height = 173,
	},
	.delay = {
		.prepare = 110,
		.enable = 50,
		.unprepare = 550,
	},
};

static const struct drm_display_mode sharp_lq150x1lg11_mode = {
	.clock = 71100,
	.hdisplay = 1024,
	.hsync_start = 1024 + 168,
	.hsync_end = 1024 + 168 + 64,
	.htotal = 1024 + 168 + 64 + 88,
	.vdisplay = 768,
	.vsync_start = 768 + 37,
	.vsync_end = 768 + 37 + 2,
	.vtotal = 768 + 37 + 2 + 8,
	.vrefresh = 60,
};

static const struct panel_desc sharp_lq150x1lg11 = {
	.modes = &sharp_lq150x1lg11_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 304,
		.height = 228,
	},
	.bus_format = MEDIA_BUS_FMT_RGB565_1X16,
};

static const struct drm_display_mode shelly_sca07010_bfn_lnn_mode = {
	.clock = 33300,
	.hdisplay = 800,
	.hsync_start = 800 + 1,
	.hsync_end = 800 + 1 + 64,
	.htotal = 800 + 1 + 64 + 64,
	.vdisplay = 480,
	.vsync_start = 480 + 1,
	.vsync_end = 480 + 1 + 23,
	.vtotal = 480 + 1 + 23 + 22,
	.vrefresh = 60,
};

static const struct panel_desc shelly_sca07010_bfn_lnn = {
	.modes = &shelly_sca07010_bfn_lnn_mode,
	.num_modes = 1,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode starry_kr122ea0sra_mode = {
	.clock = 147000,
	.hdisplay = 1920,
	.hsync_start = 1920 + 16,
	.hsync_end = 1920 + 16 + 16,
	.htotal = 1920 + 16 + 16 + 32,
	.vdisplay = 1200,
	.vsync_start = 1200 + 15,
	.vsync_end = 1200 + 15 + 2,
	.vtotal = 1200 + 15 + 2 + 18,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc starry_kr122ea0sra = {
	.modes = &starry_kr122ea0sra_mode,
	.num_modes = 1,
	.size = {
		.width = 263,
		.height = 164,
	},
	.delay = {
		.prepare = 10 + 200,
		.enable = 50,
		.unprepare = 10 + 500,
	},
};

static const struct display_timing tianma_tm070jdhg30_timing = {
	.pixelclock = { 62600000, 68200000, 78100000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 15, 64, 159 },
	.hback_porch = { 5, 5, 5 },
	.hsync_len = { 1, 1, 256 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 3, 40, 99 },
	.vback_porch = { 2, 2, 2 },
	.vsync_len = { 1, 1, 128 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc tianma_tm070jdhg30 = {
	.timings = &tianma_tm070jdhg30_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 151,
		.height = 95,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing tianma_tm070rvhg71_timing = {
	.pixelclock = { 27700000, 29200000, 39600000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 12, 40, 212 },
	.hback_porch = { 88, 88, 88 },
	.hsync_len = { 1, 1, 40 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 1, 13, 88 },
	.vback_porch = { 32, 32, 32 },
	.vsync_len = { 1, 1, 3 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc tianma_tm070rvhg71 = {
	.timings = &tianma_tm070rvhg71_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 154,
		.height = 86,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode toshiba_lt089ac29000_mode = {
	.clock = 79500,
	.hdisplay = 1280,
	.hsync_start = 1280 + 192,
	.hsync_end = 1280 + 192 + 128,
	.htotal = 1280 + 192 + 128 + 64,
	.vdisplay = 768,
	.vsync_start = 768 + 20,
	.vsync_end = 768 + 20 + 7,
	.vtotal = 768 + 20 + 7 + 3,
	.vrefresh = 60,
};

static const struct panel_desc toshiba_lt089ac29000 = {
	.modes = &toshiba_lt089ac29000_mode,
	.num_modes = 1,
	.size = {
		.width = 194,
		.height = 116,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode tpk_f07a_0102_mode = {
	.clock = 33260,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 128,
	.htotal = 800 + 40 + 128 + 88,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 2,
	.vtotal = 480 + 10 + 2 + 33,
	.vrefresh = 60,
};

static const struct panel_desc tpk_f07a_0102 = {
	.modes = &tpk_f07a_0102_mode,
	.num_modes = 1,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_flags = DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode tpk_f10a_0102_mode = {
	.clock = 45000,
	.hdisplay = 1024,
	.hsync_start = 1024 + 176,
	.hsync_end = 1024 + 176 + 5,
	.htotal = 1024 + 176 + 5 + 88,
	.vdisplay = 600,
	.vsync_start = 600 + 20,
	.vsync_end = 600 + 20 + 5,
	.vtotal = 600 + 20 + 5 + 25,
	.vrefresh = 60,
};

static const struct panel_desc tpk_f10a_0102 = {
	.modes = &tpk_f10a_0102_mode,
	.num_modes = 1,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct display_timing urt_umsh_8596md_timing = {
	.pixelclock = { 33260000, 33260000, 33260000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 41, 41, 41 },
	.hback_porch = { 216 - 128, 216 - 128, 216 - 128 },
	.hsync_len = { 71, 128, 128 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 10, 10, 10 },
	.vback_porch = { 35 - 2, 35 - 2, 35 - 2 },
	.vsync_len = { 2, 2, 2 },
	.flags = DISPLAY_FLAGS_DE_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE |
		DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW,
};

static const struct panel_desc urt_umsh_8596md_lvds = {
	.timings = &urt_umsh_8596md_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct panel_desc urt_umsh_8596md_parallel = {
	.timings = &urt_umsh_8596md_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode winstar_wf35ltiacd_mode = {
	.clock = 6410,
	.hdisplay = 320,
	.hsync_start = 320 + 20,
	.hsync_end = 320 + 20 + 30,
	.htotal = 320 + 20 + 30 + 38,
	.vdisplay = 240,
	.vsync_start = 240 + 4,
	.vsync_end = 240 + 4 + 3,
	.vtotal = 240 + 4 + 3 + 15,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc winstar_wf35ltiacd = {
	.modes = &winstar_wf35ltiacd_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 70,
		.height = 53,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "simple-panel",
		.data = NULL,
#ifndef CONFIG_DRM_PANEL_SIMPLE_OF_ONLY
	}, {
		.compatible = "ampire,am-480272h3tmqw-t01h",
		.data = &ampire_am_480272h3tmqw_t01h,
	}, {
		.compatible = "ampire,am800480r3tmqwa1h",
		.data = &ampire_am800480r3tmqwa1h,
	}, {
		.compatible = "armadeus,st0700-adapt",
		.data = &armadeus_st0700_adapt,
	}, {
		.compatible = "auo,b101aw03",
		.data = &auo_b101aw03,
	}, {
		.compatible = "auo,b101ean01",
		.data = &auo_b101ean01,
	}, {
		.compatible = "auo,b101xtn01",
		.data = &auo_b101xtn01,
	}, {
		.compatible = "auo,b116xw03",
		.data = &auo_b116xw03,
	}, {
		.compatible = "auo,b133htn01",
		.data = &auo_b133htn01,
	}, {
		.compatible = "auo,b133xtn01",
		.data = &auo_b133xtn01,
	}, {
		.compatible = "auo,g070vvn01",
		.data = &auo_g070vvn01,
	}, {
		.compatible = "auo,g104sn02",
		.data = &auo_g104sn02,
	}, {
		.compatible = "auo,g133han01",
		.data = &auo_g133han01,
	}, {
		.compatible = "auo,g185han01",
		.data = &auo_g185han01,
	}, {
		.compatible = "auo,p320hvn03",
		.data = &auo_p320hvn03,
	}, {
		.compatible = "auo,t215hvn01",
		.data = &auo_t215hvn01,
	}, {
		.compatible = "avic,tm070ddh03",
		.data = &avic_tm070ddh03,
	}, {
		.compatible = "boe,hv070wsa-100",
		.data = &boe_hv070wsa
	}, {
		.compatible = "boe,nv101wxmn51",
		.data = &boe_nv101wxmn51,
	}, {
		.compatible = "chunghwa,claa070wp03xg",
		.data = &chunghwa_claa070wp03xg,
	}, {
		.compatible = "chunghwa,claa101wa01a",
		.data = &chunghwa_claa101wa01a
	}, {
		.compatible = "chunghwa,claa101wb01",
		.data = &chunghwa_claa101wb01
	}, {
		.compatible = "dataimage,scf0700c48ggu18",
		.data = &dataimage_scf0700c48ggu18,
	}, {
		.compatible = "dlc,dlc0700yzg-1",
		.data = &dlc_dlc0700yzg_1,
	}, {
		.compatible = "edt,et057090dhu",
		.data = &edt_et057090dhu,
	}, {
		.compatible = "edt,et070080dh6",
		.data = &edt_etm0700g0dh6,
	}, {
		.compatible = "edt,etm0700g0dh6",
		.data = &edt_etm0700g0dh6,
	}, {
		.compatible = "edt,etm0700g0bdh6",
		.data = &edt_etm0700g0bdh6,
	}, {
		.compatible = "edt,etm0700g0edh6",
		.data = &edt_etm0700g0bdh6,
	}, {
		.compatible = "foxlink,fl500wvr00-a0t",
		.data = &foxlink_fl500wvr00_a0t,
	}, {
		.compatible = "giantplus,gpg482739qs5",
		.data = &giantplus_gpg482739qs5
	}, {
		.compatible = "hannstar,hsd070pww1",
		.data = &hannstar_hsd070pww1,
	}, {
		.compatible = "hannstar,hsd100pxn1",
		.data = &hannstar_hsd100pxn1,
	}, {
		.compatible = "hit,tx23d38vm0caa",
		.data = &hitachi_tx23d38vm0caa
	}, {
		.compatible = "innolux,at043tn24",
		.data = &innolux_at043tn24,
	}, {
		.compatible = "innolux,at070tn92",
		.data = &innolux_at070tn92,
	}, {
		.compatible = "innolux,g070y2-l01",
		.data = &innolux_g070y2_l01,
	}, {
		.compatible = "innolux,g101ice-l01",
		.data = &innolux_g101ice_l01
	}, {
		.compatible = "innolux,g121i1-l01",
		.data = &innolux_g121i1_l01
	}, {
		.compatible = "innolux,g121x1-l03",
		.data = &innolux_g121x1_l03,
	}, {
		.compatible = "innolux,n116bge",
		.data = &innolux_n116bge,
	}, {
		.compatible = "innolux,n156bge-l21",
		.data = &innolux_n156bge_l21,
	}, {
		.compatible = "innolux,tv123wam",
		.data = &innolux_tv123wam,
	}, {
		.compatible = "innolux,zj070na-01p",
		.data = &innolux_zj070na_01p,
	}, {
		.compatible = "koe,tx31d200vm0baa",
		.data = &koe_tx31d200vm0baa,
	}, {
		.compatible = "kyo,tcg121xglp",
		.data = &kyo_tcg121xglp,
	}, {
		.compatible = "lg,lb070wv8",
		.data = &lg_lb070wv8,
	}, {
		.compatible = "lg,lp079qx1-sp0v",
		.data = &lg_lp079qx1_sp0v,
	}, {
		.compatible = "lg,lp097qx1-spa1",
		.data = &lg_lp097qx1_spa1,
	}, {
		.compatible = "lg,lp120up1",
		.data = &lg_lp120up1,
	}, {
		.compatible = "lg,lp129qe",
		.data = &lg_lp129qe,
	}, {
		.compatible = "mitsubishi,aa070mc01-ca1",
		.data = &mitsubishi_aa070mc01,
	}, {
		.compatible = "nec,nl12880bc20-05",
		.data = &nec_nl12880bc20_05,
	}, {
		.compatible = "nec,nl4827hc19-05b",
		.data = &nec_nl4827hc19_05b,
	}, {
		.compatible = "netron-dy,e231732",
		.data = &netron_dy_e231732,
	}, {
		.compatible = "newhaven,nhd-4.3-480272ef-atxl",
		.data = &newhaven_nhd_43_480272ef_atxl,
	}, {
		.compatible = "nlt,nl192108ac18-02d",
		.data = &nlt_nl192108ac18_02d,
	}, {
		.compatible = "nvd,9128",
		.data = &nvd_9128,
	}, {
		.compatible = "okaya,rs800480t-7x0gp",
		.data = &okaya_rs800480t_7x0gp,
	}, {
		.compatible = "olimex,lcd-olinuxino-43-ts",
		.data = &olimex_lcd_olinuxino_43ts,
	}, {
		.compatible = "ontat,yx700wv03",
		.data = &ontat_yx700wv03,
	}, {
		.compatible = "ortustech,com43h4m85ulc",
		.data = &ortustech_com43h4m85ulc,
	}, {
		.compatible = "qiaodian,qd43003c0-40",
		.data = &qd43003c0_40,
	}, {
		.compatible = "rocktech,rk070er9427",
		.data = &rocktech_rk070er9427,
	}, {
		.compatible = "samsung,lsn122dl01-c01",
		.data = &samsung_lsn122dl01_c01,
	}, {
		.compatible = "samsung,ltn101nt05",
		.data = &samsung_ltn101nt05,
	}, {
		.compatible = "samsung,ltn140at29-301",
		.data = &samsung_ltn140at29_301,
	}, {
		.compatible = "sharp,lq035q7db03",
		.data = &sharp_lq035q7db03,
	}, {
		.compatible = "sharp,lq101k1ly04",
		.data = &sharp_lq101k1ly04,
	}, {
		.compatible = "sharp,lq123p1jx31",
		.data = &sharp_lq123p1jx31,
	}, {
		.compatible = "sharp,lq150x1lg11",
		.data = &sharp_lq150x1lg11,
	}, {
		.compatible = "shelly,sca07010-bfn-lnn",
		.data = &shelly_sca07010_bfn_lnn,
	}, {
		.compatible = "starry,kr122ea0sra",
		.data = &starry_kr122ea0sra,
	}, {
		.compatible = "tianma,tm070jdhg30",
		.data = &tianma_tm070jdhg30,
	}, {
		.compatible = "tianma,tm070rvhg71",
		.data = &tianma_tm070rvhg71,
	}, {
		.compatible = "toshiba,lt089ac29000",
		.data = &toshiba_lt089ac29000,
	}, {
		.compatible = "tpk,f07a-0102",
		.data = &tpk_f07a_0102,
	}, {
		.compatible = "tpk,f10a-0102",
		.data = &tpk_f10a_0102,
	}, {
		.compatible = "urt,umsh-8596md-t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "urt,umsh-8596md-1t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "urt,umsh-8596md-7t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "urt,umsh-8596md-11t",
		.data = &urt_umsh_8596md_lvds,
	}, {
		.compatible = "urt,umsh-8596md-19t",
		.data = &urt_umsh_8596md_lvds,
	}, {
		.compatible = "urt,umsh-8596md-20t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "winstar,wf35ltiacd",
		.data = &winstar_wf35ltiacd,
#endif /* !CONFIG_DRM_PANEL_SIMPLE_OF_ONLY */
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int panel_simple_of_get_firmware_desc_data(struct device *dev,
					   struct panel_desc *desc)
{
	struct device_node *np = dev->of_node;
	struct nvmem_device *nvmem;
	struct firmware_header *header;
	struct drm_display_mode *mode;
	struct videomode *vm;
	const u8 *init_data;
	const u8 *exit_data;
	u32 bus_flags;
	int ret;

	nvmem = devm_nvmem_device_get(dev, "eeprom");
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	header = (struct firmware_header *)devm_kzalloc(dev, sizeof(*header), GFP_KERNEL);
	if (!header)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, 0, sizeof(*header), header);
	if (ret < 0) {
		dev_err(dev, "failed to read firmware header: %d\n", ret);
		return ret;
	}

	if (header->firmware_size <= 0 || header->magic != 0xDEAD5A5A) {
		dev_err(dev, "Invalid eeprom firmware");
		return -EINVAL;
	}

	dev_info(dev, "lcd firmware magic: %x\n", header->magic);
	dev_info(dev, "lcd firmware version: %s, size: %d\n", header->version, header->firmware_size);
	dev_info(dev, "lcd vendor: %s, model: %s\n", header->vendor, header->model);
	
	vm = (struct videomode *)devm_kzalloc(dev, sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, header->timing_entry.offset,
				  header->timing_entry.length, vm);
	if (ret < 0)
		return ret;

	init_data = (const u8 *)devm_kzalloc(dev, header->init_seq_entry.length, GFP_KERNEL);
	if (!init_data)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, header->init_seq_entry.offset,
				  header->init_seq_entry.length,
				  (void *)init_data);
	if (ret < 0)
		return ret;

	exit_data = (const u8 *)devm_kzalloc(dev, header->eixt_seq_entry.length,
				  GFP_KERNEL);
	if (!exit_data)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, header->eixt_seq_entry.offset,
				  header->eixt_seq_entry.length,
				  (void *)exit_data);
	if (ret < 0)
		return ret;

	devm_nvmem_device_put(dev, nvmem);

	mode = (struct drm_display_mode*)devm_kzalloc(dev, sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return -ENOMEM;
	
	drm_display_mode_from_videomode(vm, mode);
	if (bus_flags)
		drm_bus_flags_from_videomode(vm, &bus_flags);
	
	desc->modes = mode;
	desc->num_modes = 1;
	desc->bus_flags = bus_flags;

	of_property_read_u32(np, "prepare-delay-ms", &desc->delay.prepare);
	of_property_read_u32(np, "enable-delay-ms", &desc->delay.enable);
	of_property_read_u32(np, "disable-delay-ms", &desc->delay.disable);
	of_property_read_u32(np, "unprepare-delay-ms", &desc->delay.unprepare);
	of_property_read_u32(np, "reset-delay-ms", &desc->delay.reset);
	of_property_read_u32(np, "init-delay-ms", &desc->delay.init);
		
	desc->init_seq = devm_kzalloc(dev, sizeof(*desc->init_seq),
				      GFP_KERNEL);
	if (!desc->init_seq)
		return -ENOMEM;

	ret = panel_simple_parse_cmd_seq(dev, init_data, header->init_seq_entry.length,
						 desc->init_seq);
	if (ret) {
		dev_err(dev, "failed to parse init sequence\n");
		return ret;
	}

	desc->exit_seq = devm_kzalloc(dev, sizeof(*desc->exit_seq),
				      GFP_KERNEL);
	if (!desc->exit_seq)
		return -ENOMEM;

	ret = panel_simple_parse_cmd_seq(dev, exit_data, header->eixt_seq_entry.length,
						 desc->exit_seq);
	if (ret) {
		dev_err(dev, "failed to parse exit sequence\n");
		return ret;
	}	

	return 0;
}

static int panel_simple_of_get_desc_data(struct device *dev,
					 struct panel_desc *desc)
{
	struct device_node *np = dev->of_node;
	struct drm_display_mode *mode;
	u32 bus_flags;
	const void *data;
	int len;
	int err;

	mode = devm_kzalloc(dev, sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return -ENOMEM;

	err = of_get_drm_display_mode(np, mode, &bus_flags, OF_USE_NATIVE_MODE);
	if (!err) {
		desc->modes = mode;
		desc->num_modes = 1;
		desc->bus_flags = bus_flags;

		of_property_read_u32(np, "bpc", &desc->bpc);
		of_property_read_u32(np, "bus-format", &desc->bus_format);
		of_property_read_u32(np, "width-mm", &desc->size.width);
		of_property_read_u32(np, "height-mm", &desc->size.height);
	}

	of_property_read_u32(np, "prepare-delay-ms", &desc->delay.prepare);
	of_property_read_u32(np, "enable-delay-ms", &desc->delay.enable);
	of_property_read_u32(np, "disable-delay-ms", &desc->delay.disable);
	of_property_read_u32(np, "unprepare-delay-ms", &desc->delay.unprepare);
	of_property_read_u32(np, "reset-delay-ms", &desc->delay.reset);
	of_property_read_u32(np, "init-delay-ms", &desc->delay.init);

	data = of_get_property(np, "panel-init-sequence", &len);
	if (data) {
		desc->init_seq = devm_kzalloc(dev, sizeof(*desc->init_seq),
					      GFP_KERNEL);
		if (!desc->init_seq)
			return -ENOMEM;

		err = panel_simple_parse_cmd_seq(dev, data, len,
						 desc->init_seq);
		if (err) {
			dev_err(dev, "failed to parse init sequence\n");
			return err;
		}
	}

	data = of_get_property(np, "panel-exit-sequence", &len);
	if (data) {
		desc->exit_seq = devm_kzalloc(dev, sizeof(*desc->exit_seq),
					      GFP_KERNEL);
		if (!desc->exit_seq)
			return -ENOMEM;

		err = panel_simple_parse_cmd_seq(dev, data, len,
						 desc->exit_seq);
		if (err) {
			dev_err(dev, "failed to parse exit sequence\n");
			return err;
		}
	}

	return 0;
}

static int panel_simple_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *id;
	const struct panel_desc *desc;
	struct panel_desc *d;
	int err;

	id = of_match_node(platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	if (!id->data) {
		d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
		if (!d)
			return -ENOMEM;

		err = panel_simple_of_get_desc_data(dev, d);
		if (err) {
			dev_err(dev, "failed to get desc data: %d\n", err);
			return err;
		}
	}

	desc = id->data ? id->data : d;

	return panel_simple_probe(&pdev->dev, desc);
}

static int panel_simple_platform_remove(struct platform_device *pdev)
{
	return panel_simple_remove(&pdev->dev);
}

static void panel_simple_platform_shutdown(struct platform_device *pdev)
{
	panel_simple_shutdown(&pdev->dev);
}

static struct platform_driver panel_simple_platform_driver = {
	.driver = {
		.name = "panel-simple",
		.of_match_table = platform_of_match,
	},
	.probe = panel_simple_platform_probe,
	.remove = panel_simple_platform_remove,
	.shutdown = panel_simple_platform_shutdown,
};

struct panel_desc_dsi {
	struct panel_desc desc;

	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

static const struct drm_display_mode auo_b080uan01_mode = {
	.clock = 154500,
	.hdisplay = 1200,
	.hsync_start = 1200 + 62,
	.hsync_end = 1200 + 62 + 4,
	.htotal = 1200 + 62 + 4 + 62,
	.vdisplay = 1920,
	.vsync_start = 1920 + 9,
	.vsync_end = 1920 + 9 + 2,
	.vtotal = 1920 + 9 + 2 + 8,
	.vrefresh = 60,
};

static const struct panel_desc_dsi auo_b080uan01 = {
	.desc = {
		.modes = &auo_b080uan01_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 108,
			.height = 272,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode boe_tv080wum_nl0_mode = {
	.clock = 160000,
	.hdisplay = 1200,
	.hsync_start = 1200 + 120,
	.hsync_end = 1200 + 120 + 20,
	.htotal = 1200 + 120 + 20 + 21,
	.vdisplay = 1920,
	.vsync_start = 1920 + 21,
	.vsync_end = 1920 + 21 + 3,
	.vtotal = 1920 + 21 + 3 + 18,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static const struct panel_desc_dsi boe_tv080wum_nl0 = {
	.desc = {
		.modes = &boe_tv080wum_nl0_mode,
		.num_modes = 1,
		.size = {
			.width = 107,
			.height = 172,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO |
		 MIPI_DSI_MODE_VIDEO_BURST |
		 MIPI_DSI_MODE_VIDEO_SYNC_PULSE,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode lg_ld070wx3_sl01_mode = {
	.clock = 71000,
	.hdisplay = 800,
	.hsync_start = 800 + 32,
	.hsync_end = 800 + 32 + 1,
	.htotal = 800 + 32 + 1 + 57,
	.vdisplay = 1280,
	.vsync_start = 1280 + 28,
	.vsync_end = 1280 + 28 + 1,
	.vtotal = 1280 + 28 + 1 + 14,
	.vrefresh = 60,
};

static const struct panel_desc_dsi lg_ld070wx3_sl01 = {
	.desc = {
		.modes = &lg_ld070wx3_sl01_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 94,
			.height = 151,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode lg_lh500wx1_sd03_mode = {
	.clock = 67000,
	.hdisplay = 720,
	.hsync_start = 720 + 12,
	.hsync_end = 720 + 12 + 4,
	.htotal = 720 + 12 + 4 + 112,
	.vdisplay = 1280,
	.vsync_start = 1280 + 8,
	.vsync_end = 1280 + 8 + 4,
	.vtotal = 1280 + 8 + 4 + 12,
	.vrefresh = 60,
};

static const struct panel_desc_dsi lg_lh500wx1_sd03 = {
	.desc = {
		.modes = &lg_lh500wx1_sd03_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 62,
			.height = 110,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode panasonic_vvx10f004b00_mode = {
	.clock = 157200,
	.hdisplay = 1920,
	.hsync_start = 1920 + 154,
	.hsync_end = 1920 + 154 + 16,
	.htotal = 1920 + 154 + 16 + 32,
	.vdisplay = 1200,
	.vsync_start = 1200 + 17,
	.vsync_end = 1200 + 17 + 2,
	.vtotal = 1200 + 17 + 2 + 16,
	.vrefresh = 60,
};

static const struct panel_desc_dsi panasonic_vvx10f004b00 = {
	.desc = {
		.modes = &panasonic_vvx10f004b00_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 217,
			.height = 136,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		 MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct of_device_id dsi_of_match[] = {
	{
		.compatible = "simple-panel-dsi",
		.data = NULL,
#ifndef CONFIG_DRM_PANEL_SIMPLE_OF_ONLY
	}, {
		.compatible = "auo,b080uan01",
		.data = &auo_b080uan01
	}, {
		.compatible = "boe,tv080wum-nl0",
		.data = &boe_tv080wum_nl0
	}, {
		.compatible = "lg,ld070wx3-sl01",
		.data = &lg_ld070wx3_sl01
	}, {
		.compatible = "lg,lh500wx1-sd03",
		.data = &lg_lh500wx1_sd03
	}, {
		.compatible = "panasonic,vvx10f004b00",
		.data = &panasonic_vvx10f004b00
#endif /* !CONFIG_DRM_PANEL_SIMPLE_OF_ONLY */
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, dsi_of_match);

static int panel_simple_dsi_of_get_desc_data(struct device *dev,
					     struct panel_desc_dsi *desc)
{
	struct device_node *np = dev->of_node;
	u32 val;
	int err;

	err = panel_simple_of_get_firmware_desc_data(dev, &desc->desc);
	if (!err) {
		dev_info(dev, "found firmware desc data\n");
	} else {
		dev_info(dev, "not found firmware desc data, using defaults\n");
		err = panel_simple_of_get_desc_data(dev, &desc->desc);
		if (err)
			return err;
	}

	if (!of_property_read_u32(np, "dsi,flags", &val))
		desc->flags = val;
	if (!of_property_read_u32(np, "dsi,format", &val))
		desc->format = val;
	if (!of_property_read_u32(np, "dsi,lanes", &val))
		desc->lanes = val;

	return 0;
}

static int panel_simple_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct panel_simple *panel;
	struct device *dev = &dsi->dev;
	const struct panel_desc_dsi *desc;
	struct panel_desc_dsi *d;
	const struct of_device_id *id;
	int err;

	id = of_match_node(dsi_of_match, dsi->dev.of_node);
	if (!id)
		return -ENODEV;

	if (!id->data) {
		d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
		if (!d)
			return -ENOMEM;

		err = panel_simple_dsi_of_get_desc_data(dev, d);
		if (err) {
			dev_err(dev, "failed to get desc data: %d\n", err);
			return err;
		}
	}

	desc = id->data ? id->data : d;

	err = panel_simple_probe(&dsi->dev, &desc->desc);
	if (err < 0)
		return err;

	panel = dev_get_drvdata(dev);
	panel->dsi = dsi;

	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	err = mipi_dsi_attach(dsi);
	if (err) {
		struct panel_simple *panel = dev_get_drvdata(&dsi->dev);

		drm_panel_remove(&panel->base);
	}

	return err;
}

static int panel_simple_dsi_remove(struct mipi_dsi_device *dsi)
{
	int err;

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	return panel_simple_remove(&dsi->dev);
}

static void panel_simple_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	panel_simple_shutdown(&dsi->dev);
}

static struct mipi_dsi_driver panel_simple_dsi_driver = {
	.driver = {
		.name = "panel-simple-dsi",
		.of_match_table = dsi_of_match,
	},
	.probe = panel_simple_dsi_probe,
	.remove = panel_simple_dsi_remove,
	.shutdown = panel_simple_dsi_shutdown,
};

static int __init panel_simple_init(void)
{
	int err;

	err = platform_driver_register(&panel_simple_platform_driver);
	if (err < 0)
		return err;

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
		err = mipi_dsi_driver_register(&panel_simple_dsi_driver);
		if (err < 0)
			return err;
	}

	return 0;
}
#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
rootfs_initcall(panel_simple_init);
#else
// module_init(panel_simple_init);
late_initcall(panel_simple_init);
#endif

static void __exit panel_simple_exit(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&panel_simple_dsi_driver);

	platform_driver_unregister(&panel_simple_platform_driver);
}
module_exit(panel_simple_exit);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("DRM Driver for Simple Panels");
MODULE_LICENSE("GPL and additional rights");
