// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DS90UB953 video serializer
 *
 * Based on a driver from Luca Ceresoli <luca@lucaceresoli.net>
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

#define UB953_PAD_SINK			0
#define UB953_PAD_SOURCE		1

#define UB953_NUM_GPIOS			4

#define UB953_REG_RESET_CTL			0x01
#define UB953_REG_RESET_CTL_DIGITAL_RESET_1	BIT(1)
#define UB953_REG_RESET_CTL_DIGITAL_RESET_0	BIT(0)

#define UB953_REG_GENERAL_CFG			0x02
#define UB953_REG_MODE_SEL			0x03

#define UB953_REG_CLKOUT_CTRL0			0x06
#define UB953_REG_CLKOUT_CTRL1			0x07

#define UB953_REG_SCL_HIGH_TIME			0x0B
#define UB953_REG_SCL_LOW_TIME			0x0C

#define UB953_REG_LOCAL_GPIO_DATA		0x0d
#define UB953_REG_LOCAL_GPIO_DATA_GPIO_RMTEN(n)		BIT(4 + (n))
#define UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(n)	BIT(0 + (n))

#define UB953_REG_GPIO_INPUT_CTRL		0x0e
#define UB953_REG_GPIO_INPUT_CTRL_OUT_EN(n)	BIT(4 + (n))
#define UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(n)	BIT(0 + (n))

#define UB953_REG_REV_MASK_ID			0x50
#define UB953_REG_GENERAL_STATUS		0x52

#define UB953_REG_GPIO_PIN_STS			0x53
#define UB953_REG_GPIO_PIN_STS_GPIO_STS(n)	BIT(0 + (n))

#define UB953_REG_BIST_ERR_CNT			0x54
#define UB953_REG_CRC_ERR_CNT1			0x55
#define UB953_REG_CRC_ERR_CNT2			0x56

#define UB953_REG_CSI_ERR_CNT			0x5c
#define UB953_REG_CSI_ERR_STATUS		0x5D
#define UB953_REG_CSI_ERR_DLANE01		0x5E
#define UB953_REG_CSI_ERR_DLANE23		0x5F
#define UB953_REG_CSI_ERR_CLK_LANE		0x60
#define UB953_REG_CSI_PKT_HDR_VC_ID		0x61
#define UB953_REG_PKT_HDR_WC_LSB		0x62
#define UB953_REG_PKT_HDR_WC_MSB		0x63
#define UB953_REG_CSI_ECC			0x64

#define UB953_REG_IND_ACC_CTL			0xb0
#define UB953_REG_IND_ACC_ADDR			0xb1
#define UB953_REG_IND_ACC_DATA			0xb2

#define UB953_REG_FPD3_RX_ID(n)			(0xf0 + (n))

/* Indirect register blocks */
#define UB953_IND_TARGET_PAT_GEN		0x00
#define UB953_IND_TARGET_FPD3_TX		0x01
#define UB953_IND_TARGET_DIE_ID			0x02

#define UB953_IND_PGEN_CTL			0x01
#define UB953_IND_PGEN_CTL_PGEN_ENABLE		BIT(0)
#define UB953_IND_PGEN_CFG			0x02
#define UB953_IND_PGEN_CSI_DI			0x03
#define UB953_IND_PGEN_LINE_SIZE1		0x04
#define UB953_IND_PGEN_LINE_SIZE0		0x05
#define UB953_IND_PGEN_BAR_SIZE1		0x06
#define UB953_IND_PGEN_BAR_SIZE0		0x07
#define UB953_IND_PGEN_ACT_LPF1			0x08
#define UB953_IND_PGEN_ACT_LPF0			0x09
#define UB953_IND_PGEN_TOT_LPF1			0x0A
#define UB953_IND_PGEN_TOT_LPF0			0x0B
#define UB953_IND_PGEN_LINE_PD1			0x0C
#define UB953_IND_PGEN_LINE_PD0			0x0D
#define UB953_IND_PGEN_VBP			0x0E
#define UB953_IND_PGEN_VFP			0x0F
#define UB953_IND_PGEN_COLOR(n)			(0x10 + (n)) /* n <= 15 */

struct ub953_hw_data {
	const char *model;
};

struct ub953_data {
	const struct ub953_hw_data	*hw_data;

	struct i2c_client	*client;
	struct regmap		*regmap;

	u32			num_data_lanes;

	struct gpio_chip	gpio_chip;
	char			gpio_chip_name[64];

	struct v4l2_subdev	sd;
	struct media_pad	pads[2];

	struct v4l2_async_notifier	notifier;

	struct v4l2_subdev	*source_sd;

	struct v4l2_ctrl_handler   ctrl_handler;

	bool			streaming;

	struct device_node	*tx_ep_np;

	/* lock for register access */
	struct mutex		reg_lock;

	u8			current_indirect_target;
};

static inline struct ub953_data *sd_to_ub953(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub953_data, sd);
}

/*
 * HW Access
 */

static int ub953_read(struct ub953_data *priv, u8 reg, u8 *val)
{
	unsigned int v;
	int ret;

	mutex_lock(&priv->reg_lock);

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret)
		dev_err(&priv->client->dev,
			"Cannot read register 0x%02x: %d!\n", reg, ret);
	else
		*val = v;

	mutex_unlock(&priv->reg_lock);

	return ret;
}

static int ub953_write(struct ub953_data *priv, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&priv->reg_lock);

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->client->dev,
			"Cannot write register 0x%02x: %d!\n", reg, ret);

	mutex_unlock(&priv->reg_lock);

	return ret;
}

static int _ub953_select_ind_reg_block(struct ub953_data *priv, u8 block)
{
	struct device *dev = &priv->client->dev;
	int ret;

	if (priv->current_indirect_target == block)
		return 0;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_CTL, block << 2);
	if (ret) {
		dev_err(dev, "%s: cannot select indirect target %u (%d)!\n",
			__func__, block, ret);
		return ret;
	}

	priv->current_indirect_target = block;

	return 0;
}

__maybe_unused
static int ub953_read_ind(struct ub953_data *priv, u8 block, u8 reg, u8 *val)
{
	unsigned int v;
	int ret;

	mutex_lock(&priv->reg_lock);

	ret = _ub953_select_ind_reg_block(priv, block);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_ADDR, reg);
	if (ret)
		goto out;

	ret = regmap_read(priv->regmap, UB953_REG_IND_ACC_DATA, &v);
	if (ret)
		goto out;

	*val = v;

out:
	mutex_unlock(&priv->reg_lock);

	return ret;
}

static int ub953_write_ind(struct ub953_data *priv, u8 block, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&priv->reg_lock);

	ret = _ub953_select_ind_reg_block(priv, block);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_ADDR, reg);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_DATA, val);

out:
	mutex_unlock(&priv->reg_lock);

	return ret;
}

static int ub953_write_ind16(struct ub953_data *priv, u8 block, u8 reg, u16 val)
{
	int ret;

	mutex_lock(&priv->reg_lock);

	ret = _ub953_select_ind_reg_block(priv, block);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_ADDR, reg);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_DATA, val >> 8);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_ADDR, reg + 1);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, UB953_REG_IND_ACC_DATA, val & 0xff);
	if (ret)
		goto out;

out:
	mutex_unlock(&priv->reg_lock);

	return ret;
}

/*
 * GPIO chip
 */
static int ub953_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct ub953_data *priv = gpiochip_get_data(gc);
	int ret;
	u8 v;

	ret = ub953_read(priv, UB953_REG_GPIO_INPUT_CTRL, &v);
	if (ret)
		return ret;

	if (v & UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset))
		return GPIO_LINE_DIRECTION_IN;
	else
		return GPIO_LINE_DIRECTION_OUT;
}

static int ub953_gpio_direction_in(struct gpio_chip *gc, unsigned int offset)
{
	struct ub953_data *priv = gpiochip_get_data(gc);

	return regmap_update_bits(
		priv->regmap, UB953_REG_GPIO_INPUT_CTRL,
		UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset) |
			UB953_REG_GPIO_INPUT_CTRL_OUT_EN(offset),
		UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset));
}

static int ub953_gpio_direction_out(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	struct ub953_data *priv = gpiochip_get_data(gc);
	int ret;

	ret = regmap_update_bits(
		priv->regmap, UB953_REG_LOCAL_GPIO_DATA,
		UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset),
		value ? UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset) : 0);

	if (ret)
		return ret;

	return regmap_update_bits(
		priv->regmap, UB953_REG_GPIO_INPUT_CTRL,
		UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset) |
			UB953_REG_GPIO_INPUT_CTRL_OUT_EN(offset),
		UB953_REG_GPIO_INPUT_CTRL_OUT_EN(offset));
}

static int ub953_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct ub953_data *priv = gpiochip_get_data(gc);
	int ret;
	u8 v;

	ret = ub953_read(priv, UB953_REG_GPIO_PIN_STS, &v);
	if (ret)
		return ret;

	return !!(v & UB953_REG_GPIO_PIN_STS_GPIO_STS(offset));
}

static void ub953_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct ub953_data *priv = gpiochip_get_data(gc);

	regmap_update_bits(
		priv->regmap, UB953_REG_LOCAL_GPIO_DATA,
		UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset),
		value ? UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset) : 0);
}

static int ub953_gpio_of_xlate(struct gpio_chip *gc,
			       const struct of_phandle_args *gpiospec,
			       u32 *flags)
{
	if (flags)
		*flags = gpiospec->args[1];

	return gpiospec->args[0];
}

static int ub953_gpiochip_probe(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct gpio_chip *gc = &priv->gpio_chip;
	int ret;

	/* Set all GPIOs to local input mode */
	ub953_write(priv, UB953_REG_LOCAL_GPIO_DATA, 0);
	ub953_write(priv, UB953_REG_GPIO_INPUT_CTRL, 0xf);

	scnprintf(priv->gpio_chip_name, sizeof(priv->gpio_chip_name), "%s",
		  dev_name(dev));

	gc->label = priv->gpio_chip_name;
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->can_sleep = 1;
	gc->ngpio = UB953_NUM_GPIOS;
	gc->get_direction = ub953_gpio_get_direction;
	gc->direction_input = ub953_gpio_direction_in;
	gc->direction_output = ub953_gpio_direction_out;
	gc->get = ub953_gpio_get;
	gc->set = ub953_gpio_set;
	gc->of_xlate = ub953_gpio_of_xlate;
	gc->of_node = priv->client->dev.of_node;
	gc->of_gpio_n_cells = 2;

	ret = gpiochip_add_data(gc, priv);
	if (ret) {
		dev_err(dev, "Failed to add GPIOs: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ub953_gpiochip_remove(struct ub953_data *priv)
{
	gpiochip_remove(&priv->gpio_chip);
}

/*
 * V4L2
 */

static int ub953_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	int ret;

	priv->streaming = enable;

	ret = v4l2_subdev_call(priv->source_sd, video, s_stream, enable);
	if (ret && enable)
		priv->streaming = false;

	return ret;
}

static const struct v4l2_subdev_video_ops ub953_video_ops = {
	.s_stream = ub953_s_stream,
};

static int _ub953_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_krouting *routing)
{
	const struct v4l2_mbus_framefmt format = {
		.width = 640,
		.height = 480,
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_LIM_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};
	int ret;

	/*
	 * Note: we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until
	 * frame desc is made dynamically allocated.
	 */

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_routing_simple_verify(routing);
	if (ret)
		return ret;

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);

	v4l2_subdev_unlock_state(state);

	if (ret)
		return ret;

	return 0;
}

static int ub953_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     enum v4l2_subdev_format_whence which,
			     struct v4l2_subdev_krouting *routing)
{
	struct ub953_data *priv = sd_to_ub953(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	return _ub953_set_routing(sd, state, routing);
}

static int ub953_get_source_frame_desc(struct ub953_data *priv,
				       struct v4l2_mbus_frame_desc *desc)
{
	struct media_pad *pad;
	int ret;

	pad = media_entity_remote_pad(&priv->pads[UB953_PAD_SINK]);
	if (!pad)
		return -EPIPE;

	ret = v4l2_subdev_call(priv->source_sd, pad, get_frame_desc, pad->index,
			       desc);
	if (ret)
		return ret;

	return 0;
}

static int ub953_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_frame_desc *fd)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	const struct v4l2_subdev_krouting *routing;
	struct v4l2_mbus_frame_desc source_fd;
	struct v4l2_subdev_route *route;
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (pad != 1) /* first tx pad */
		return -EINVAL;

	ret = ub953_get_source_frame_desc(priv, &source_fd);
	if (ret)
		return ret;

	state = v4l2_subdev_lock_active_state(sd);

	routing = &state->routing;

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for_each_active_route(routing, route) {
		struct v4l2_mbus_frame_desc_entry *source_entry = NULL;
		unsigned int j;

		if (route->source_pad != pad)
			continue;

		for (j = 0; j < source_fd.num_entries; ++j)
			if (source_fd.entry[j].stream == route->sink_stream) {
				source_entry = &source_fd.entry[j];
				break;
			}

		if (!source_entry) {
			dev_err(&priv->client->dev,
				"Failed to find stream from source frame desc\n");
			ret = -EPIPE;
			goto out;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;

		fd->entry[fd->num_entries].flags =
			V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length = source_entry->length;
		fd->entry[fd->num_entries].pixelcode = source_entry->pixelcode;
		fd->entry[fd->num_entries].bus.csi2.vc =
			source_entry->bus.csi2.vc;
		fd->entry[fd->num_entries].bus.csi2.dt =
			source_entry->bus.csi2.dt;

		fd->num_entries++;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ub953_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (format->pad == 1)
		return v4l2_subdev_get_fmt(sd, state, format);

	v4l2_subdev_lock_state(state);

	/* Set sink format */
	fmt = v4l2_state_get_stream_format(state, format->pad, format->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = format->format;

	/* Propagate to source format */
	fmt = v4l2_state_get_opposite_stream_format(state, format->pad,
							   format->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = format->format;

	v4l2_subdev_unlock_state(state);

	return 0;
}

static int ub953_init_cfg(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = 0,
			.sink_stream = 0,
			.source_pad = 1,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return _ub953_set_routing(sd, state, &routing);
}

static int ub953_log_status(struct v4l2_subdev *sd)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	struct device *dev = &priv->client->dev;
	u8 v = 0, v1 = 0, v2 = 0;
	unsigned int i;
	char id[7];
	u8 gpio_local_data;
	u8 gpio_input_ctrl;
	u8 gpio_pin_sts;

	for (i = 0; i < 6; ++i)
		ub953_read(priv, UB953_REG_FPD3_RX_ID(i), &id[i]);
	id[6] = 0;

	dev_info(dev, "ID '%s'\n", id);

	ub953_read(priv, UB953_REG_GENERAL_STATUS, &v);
	dev_info(dev, "GENERAL_STATUS %#x\n", v);

	ub953_read(priv, UB953_REG_CRC_ERR_CNT1, &v1);
	ub953_read(priv, UB953_REG_CRC_ERR_CNT2, &v2);
	dev_info(dev, "CRC error count %u\n", v1 | (v2 << 8));

	ub953_read(priv, UB953_REG_CSI_ERR_CNT, &v);
	dev_info(dev, "CSI error count %u\n", v);

	ub953_read(priv, UB953_REG_CSI_ERR_STATUS, &v);
	dev_info(dev, "CSI_ERR_STATUS %#x\n", v);

	ub953_read(priv, UB953_REG_CSI_ERR_DLANE01, &v);
	dev_info(dev, "CSI_ERR_DLANE01 %#x\n", v);

	ub953_read(priv, UB953_REG_CSI_ERR_DLANE23, &v);
	dev_info(dev, "CSI_ERR_DLANE23 %#x\n", v);

	ub953_read(priv, UB953_REG_CSI_ERR_CLK_LANE, &v);
	dev_info(dev, "CSI_ERR_CLK_LANE %#x\n", v);

	ub953_read(priv, UB953_REG_CSI_PKT_HDR_VC_ID, &v);
	dev_info(dev, "CSI packet header VC %u ID %u\n", v >> 6, v & 0x3f);

	ub953_read(priv, UB953_REG_PKT_HDR_WC_LSB, &v1);
	ub953_read(priv, UB953_REG_PKT_HDR_WC_MSB, &v2);
	dev_info(dev, "CSI packet header WC %u\n", (v2 << 8) | v1);

	ub953_read(priv, UB953_REG_CSI_ECC, &v);
	dev_info(dev, "CSI ECC %#x\n", v);

	ub953_read(priv, UB953_REG_LOCAL_GPIO_DATA, &gpio_local_data);
	ub953_read(priv, UB953_REG_GPIO_INPUT_CTRL, &gpio_input_ctrl);
	ub953_read(priv, UB953_REG_GPIO_PIN_STS, &gpio_pin_sts);

	for (i = 0; i < UB953_NUM_GPIOS; ++i) {
		dev_info(dev,
			 "GPIO%u: remote: %u is_input: %u is_output: %u val: %u sts: %u\n",
			 i,
			 !!(gpio_local_data & UB953_REG_LOCAL_GPIO_DATA_GPIO_RMTEN(i)),
			 !!(gpio_input_ctrl & UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(i)),
			 !!(gpio_input_ctrl & UB953_REG_GPIO_INPUT_CTRL_OUT_EN(i)),
			 !!(gpio_local_data & UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(i)),
			 !!(gpio_pin_sts & UB953_REG_GPIO_PIN_STS_GPIO_STS(i)));
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops ub953_pad_ops = {
	.set_routing = ub953_set_routing,
	.get_frame_desc = ub953_get_frame_desc,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ub953_set_fmt,
	.init_cfg = ub953_init_cfg,
};

static const struct v4l2_subdev_core_ops ub953_subdev_core_ops = {
	.log_status		= ub953_log_status,
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops ub953_subdev_ops = {
	.core = &ub953_subdev_core_ops,
	.video = &ub953_video_ops,
	.pad = &ub953_pad_ops,
};

static const struct media_entity_operations ub953_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

enum {
	TEST_PATTERN_DISABLED = 0,
	TEST_PATTERN_V_COLOR_BARS_1,
	TEST_PATTERN_V_COLOR_BARS_2,
	TEST_PATTERN_V_COLOR_BARS_4,
	TEST_PATTERN_V_COLOR_BARS_8,
};

static const char *const ub953_tpg_qmenu[] = {
	"Disabled",
	"1 vertical color bar",
	"2 vertical color bars",
	"4 vertical color bars",
	"8 vertical color bars",
};

static void ub953_enable_tpg(struct ub953_data *priv, int tpg_num)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u8 vbp, vfp;
	u16 blank_lines;
	u16 width;
	u16 height;

	u16 bytespp = 2; /* For MEDIA_BUS_FMT_UYVY8_1X16 */
	u8 cbars_idx = tpg_num - TEST_PATTERN_V_COLOR_BARS_1;
	u8 num_cbars = 1 << cbars_idx;

	u16 line_size; /* Line size [bytes] */
	u16 bar_size; /* cbar size [bytes] */
	u16 act_lpf; /* active lines/frame */
	u16 tot_lpf; /* tot lines/frame */
	u16 line_pd; /* Line period in 10-ns units */

	u16 fps = 30;

	vbp = 33;
	vfp = 10;
	blank_lines = vbp + vfp + 2; /* total blanking lines */

	state = v4l2_subdev_lock_active_state(sd);

	fmt = v4l2_state_get_stream_format(state, UB953_PAD_SOURCE, 0);

	width = fmt->width;
	height = fmt->height;

	line_size = width * bytespp;
	bar_size = line_size / num_cbars;
	act_lpf = height;
	tot_lpf = act_lpf + blank_lines;
	line_pd = 100000000 / fps / tot_lpf;

	ub953_write_ind(priv, UB953_IND_TARGET_PAT_GEN, UB953_IND_PGEN_CTL,
			 UB953_IND_PGEN_CTL_PGEN_ENABLE);

	/* YUV422 8bit: 2 bytes/block, CSI-2 data type 0x1e */
	ub953_write_ind(priv, UB953_IND_TARGET_PAT_GEN, UB953_IND_PGEN_CFG,
			 cbars_idx << 4 | 0x2);
	ub953_write_ind(priv, UB953_IND_TARGET_PAT_GEN, UB953_IND_PGEN_CSI_DI,
			 0x1e);

	ub953_write_ind16(priv, UB953_IND_TARGET_PAT_GEN,
			  UB953_IND_PGEN_LINE_SIZE1, line_size);
	ub953_write_ind16(priv, UB953_IND_TARGET_PAT_GEN,
			  UB953_IND_PGEN_BAR_SIZE1, bar_size);
	ub953_write_ind16(priv, UB953_IND_TARGET_PAT_GEN,
			  UB953_IND_PGEN_ACT_LPF1, act_lpf);
	ub953_write_ind16(priv, UB953_IND_TARGET_PAT_GEN,
			  UB953_IND_PGEN_TOT_LPF1, tot_lpf);
	ub953_write_ind16(priv, UB953_IND_TARGET_PAT_GEN,
			  UB953_IND_PGEN_LINE_PD1, line_pd);
	ub953_write_ind(priv, UB953_IND_TARGET_PAT_GEN, UB953_IND_PGEN_VBP,
			 vbp);
	ub953_write_ind(priv, UB953_IND_TARGET_PAT_GEN, UB953_IND_PGEN_VFP,
			 vfp);

	v4l2_subdev_unlock_state(state);
}

static void ub953_disable_tpg(struct ub953_data *priv)
{
	ub953_write_ind(priv, UB953_IND_TARGET_PAT_GEN, UB953_IND_PGEN_CTL,
			 0x0);
}

static int ub953_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub953_data *priv =
		container_of(ctrl->handler, struct ub953_data, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == 0)
			ub953_disable_tpg(priv);
		else
			ub953_enable_tpg(priv, ctrl->val);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ub953_ctrl_ops = {
	.s_ctrl = ub953_s_ctrl,
};

static int ub953_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *source_subdev,
			      struct v4l2_async_subdev *asd)
{
	struct ub953_data *priv = sd_to_ub953(notifier->sd);
	struct device *dev = &priv->client->dev;
	unsigned int src_pad;
	int ret;

	dev_dbg(dev, "Bind %s\n", source_subdev->name);

	ret = media_entity_get_fwnode_pad(&source_subdev->entity,
					  source_subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n",
			source_subdev->name);
		return ret;
	}

	priv->source_sd = source_subdev;
	src_pad = ret;

	ret = media_create_pad_link(
		&source_subdev->entity, src_pad, &priv->sd.entity, 0,
		MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:0\n",
			source_subdev->name, src_pad, priv->sd.name);
		return ret;
	}

	dev_dbg(dev, "Bound %s:%u\n", source_subdev->name, src_pad);

	dev_dbg(dev, "All subdevs bound\n");

	return 0;
}

static void ub953_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *source_subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub953_data *priv = sd_to_ub953(notifier->sd);
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unbind %s\n", source_subdev->name);
}

static const struct v4l2_async_notifier_operations ub953_notify_ops = {
	.bound = ub953_notify_bound,
	.unbind = ub953_notify_unbind,
};

static int ub953_v4l2_notifier_register(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_async_subdev *asd;
	struct device_node *ep_node;
	int ret;

	dev_dbg(dev, "register async notif\n");

	ep_node = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (!ep_node) {
		dev_err(dev, "No graph endpoint\n");
		return -ENODEV;
	}

	v4l2_async_notifier_init(&priv->notifier);

	asd = v4l2_async_notifier_add_fwnode_remote_subdev(
		&priv->notifier, of_fwnode_handle(ep_node),
		sizeof(*asd));

	of_node_put(ep_node);

	if (IS_ERR(asd)) {
		dev_err(dev, "Failed to add subdev: %ld", PTR_ERR(asd));
		v4l2_async_notifier_cleanup(&priv->notifier);
		return PTR_ERR(asd);
	}

	priv->notifier.ops = &ub953_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void ub953_v4l2_notifier_unregister(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unregister async notif\n");

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

/*
 * Probing
 */

static void ub953_reset(struct ub953_data *priv, bool reset_regs)
{
	struct device *dev = &priv->client->dev;
	unsigned long timeout;
	int ret;
	u8 bit;

	bit = reset_regs ? BIT(1) : BIT(0);

	ub953_write(priv, UB953_REG_RESET_CTL, bit);

	/*
	 * Device is not accessible right after reset. We need to wait an
	 * undefined time.
	 */

	usleep_range(1000, 5000);

	timeout = jiffies + msecs_to_jiffies(1000);

	while (time_before(jiffies, timeout)) {
		unsigned int v;

		/* ignore errors in case the device is not yet accessible */
		ret = regmap_read(priv->regmap, UB953_REG_RESET_CTL, &v);
		if (!ret && !(v & bit))
			return;

		usleep_range(500, 5000);
	}

	dev_err(dev, "reset timeout\n");
}

static int ub953_i2c_init(struct ub953_data *priv)
{
	/* i2c fast mode */
	u32 scl_high = 915; /* ns */
	u32 scl_low = 1641; /* ns */
	u32 ref = 25000000; /* TODO: get refclock from deserializer */
	int ret;

	scl_high = div64_u64((u64)scl_high * ref, 1000000000) - 5;
	scl_low = div64_u64((u64)scl_low * ref, 1000000000) - 5;

	ret = ub953_write(priv, UB953_REG_SCL_HIGH_TIME, scl_high);
	if (ret)
		return ret;

	ret = ub953_write(priv, UB953_REG_SCL_LOW_TIME, scl_low);
	if (ret)
		return ret;

	return 0;
}

static int ub953_parse_dt(struct ub953_data *priv)
{
	struct device_node *np = priv->client->dev.of_node;
	struct device *dev = &priv->client->dev;
	struct device_node *ep_np;
	int ret;

	if (!np) {
		dev_err(dev, "OF: no device tree node!\n");
		return -ENOENT;
	}

	ep_np = of_graph_get_endpoint_by_regs(np, 0, 0);
	if (!ep_np) {
		dev_err(dev, "OF: no endpoint\n");
		return -ENOENT;
	}

	ret = of_property_count_u32_elems(ep_np, "data-lanes");
	if (ret <= 0) {
		dev_err(dev, "OF: failed to parse data-lanes: %d\n", ret);
		return ret;
	}

	if (ret != 1 && ret != 2 && ret != 4) {
		dev_err(dev, "OF: bad number of data-lanes: %d\n", ret);
		return -EINVAL;
	}

	priv->num_data_lanes = ret;

	return 0;
}

static const struct regmap_config ub953_regmap_config = {
	.name = "ds90ub953",
	.reg_bits = 8,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_DEFAULT,
	.val_format_endian = REGMAP_ENDIAN_DEFAULT,
};

static void ub953_setup_clkout(struct ub953_data *priv)
{
	unsigned long refclk;
	unsigned long fc_rate;
	unsigned long rate;
	u32 hs_clk_div;
	u32 m, n;

	/* TODO: get refclock from deserializer */
	refclk = 25000000;
	hs_clk_div = 4;
	m = 1;
	n = 40;

	/* Only sync mode supported for now */

	fc_rate = 160 * refclk;
	rate = fc_rate / hs_clk_div;
	rate = rate * m / n;

	dev_dbg(&priv->client->dev, "clkout rate %lu\n", rate);

	ub953_write(priv, UB953_REG_CLKOUT_CTRL0, (__ffs(hs_clk_div) << 5) | m);
	ub953_write(priv, UB953_REG_CLKOUT_CTRL1, n);
}

static int ub953_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub953_data *priv;
	int ret;
	u8 v;
	u8 mode;
	bool mode_override;

	dev_dbg(dev, "probing, addr 0x%02x\n", client->addr);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->hw_data = of_device_get_match_data(dev);
	if (!priv->hw_data)
		return -ENODEV;

	mutex_init(&priv->reg_lock);

	/*
	 * Initialize to invalid values so that the first reg writes will
	 * configure the target.
	 */
	priv->current_indirect_target = 0xff;

	priv->regmap = devm_regmap_init_i2c(client, &ub953_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		ret = PTR_ERR(priv->regmap);
		goto err_mutex_destroy;
	}

	ret = ub953_parse_dt(priv);
	if (ret)
		goto err_mutex_destroy;

	ret = ub953_read(priv, UB953_REG_MODE_SEL, &v);
	if (ret)
		goto err_mutex_destroy;

	if (!(v & BIT(3))) {
		dev_err(dev, "Mode value not stabilized\n");
		ret = -ENODEV;
		goto err_mutex_destroy;
	}

	mode_override = v & BIT(4);
	mode = v & 0x7;

	dev_dbg(dev, "mode from %s: %#x\n", mode_override ? "reg" : "strap",
		mode);

	if (mode != 0) {
		dev_err(dev, "Only synchronous mode supported\n");
		ret = -ENODEV;
		goto err_mutex_destroy;
	}

	/* Do a full reset only if we have not overridden the mode */
	ub953_reset(priv, !mode_override);

	ret = ub953_read(priv, UB953_REG_REV_MASK_ID, &v);
	if (ret) {
		dev_err(dev, "Failed to read revision: %d", ret);
		goto err_mutex_destroy;
	}

	dev_info(dev, "Found %s rev/mask %#04x\n", priv->hw_data->model, v);

	ret = ub953_read(priv, UB953_REG_GENERAL_CFG, &v);
	if (ret)
		goto err_mutex_destroy;

	dev_dbg(dev, "i2c strap setting %s V\n", (v & 1) ? "1.8" : "3.3");

	ret = ub953_i2c_init(priv);
	if (ret) {
		dev_err(dev, "i2c init failed: %d\n", ret);
		goto err_mutex_destroy;
	}

	ret = ub953_gpiochip_probe(priv);
	if (ret) {
		dev_err(dev, "Failed to init gpiochip\n");
		goto err_mutex_destroy;
	}

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &ub953_subdev_ops);

	v4l2_ctrl_handler_init(&priv->ctrl_handler,
			       ARRAY_SIZE(ub953_tpg_qmenu) - 1);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &ub953_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ub953_tpg_qmenu) - 1, 0, 0,
				     ub953_tpg_qmenu);

	if (priv->ctrl_handler.error) {
		ret = priv->ctrl_handler.error;
		goto err_gpiochip_remove;
	}

	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS |
		V4L2_SUBDEV_FL_MULTIPLEXED;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &ub953_entity_ops;

	priv->pads[0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 2, priv->pads);
	if (ret) {
		dev_err(dev, "Failed to init pads\n");
		goto err_remove_ctrls;
	}

	priv->tx_ep_np = of_graph_get_endpoint_by_regs(dev->of_node, 1, 0);
	priv->sd.fwnode = of_fwnode_handle(priv->tx_ep_np);

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = ub953_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "v4l2 subdev notifier register failed: %d\n", ret);
		goto err_free_state;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(dev, "v4l2_async_register_subdev error: %d\n", ret);
		goto err_unreg_notif;
	}

	ub953_setup_clkout(priv);

	ub953_write(priv, UB953_REG_GENERAL_CFG,
		    (1 << 6) | /* continuous clk */
		    ((priv->num_data_lanes - 1) << 4) |
		    (1 << 1)); /* CRC TX gen */

	dev_dbg(dev, "Successfully probed\n");

	return 0;

err_unreg_notif:
	ub953_v4l2_notifier_unregister(priv);
err_free_state:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	if (priv->tx_ep_np)
		of_node_put(priv->tx_ep_np);

	media_entity_cleanup(&priv->sd.entity);
err_remove_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
err_gpiochip_remove:
	ub953_gpiochip_remove(priv);
err_mutex_destroy:
	mutex_destroy(&priv->reg_lock);

	return ret;
}

static int ub953_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ub953_data *priv = sd_to_ub953(sd);

	dev_dbg(&client->dev, "Removing\n");

	ub953_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(&priv->sd);

	v4l2_subdev_cleanup(&priv->sd);

	of_node_put(priv->tx_ep_np);

	media_entity_cleanup(&priv->sd.entity);

	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	ub953_gpiochip_remove(priv);
	mutex_destroy(&priv->reg_lock);

	return 0;
}

static const struct ub953_hw_data ds90ub953_hw = {
	.model = "ub953",
};

static const struct i2c_device_id ub953_id[] = {
	{ "ds90ub953-q1", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ub953_id);

#ifdef CONFIG_OF
static const struct of_device_id ub953_dt_ids[] = {
	{ .compatible = "ti,ds90ub953-q1", .data = &ds90ub953_hw },
	{}
};
MODULE_DEVICE_TABLE(of, ub953_dt_ids);
#endif

static struct i2c_driver ds90ub953_driver = {
	.probe_new	= ub953_probe,
	.remove		= ub953_remove,
	.id_table	= ub953_id,
	.driver = {
		.name	= "ds90ub953",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub953_dt_ids),
	},
};

module_i2c_driver(ds90ub953_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Texas Instruments DS90UB953 serializer driver");
MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>");
