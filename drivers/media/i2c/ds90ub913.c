// SPDX-License-Identifier: GPL-2.0
/**
 * Driver for the Texas Instruments DS90UB913-Q1 video serializer
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
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <media/v4l2-subdev.h>

#include <dt-bindings/media/ds90ub9xx.h>

#define UB913_PAD_SINK			0
#define UB913_PAD_SOURCE		1

#define UB913_NUM_GPIOS			4

#define UB913_REG_RESET_CTL			0x01
#define UB913_REG_RESET_CTL_DIGITAL_RESET_1	BIT(1)
#define UB913_REG_RESET_CTL_DIGITAL_RESET_0	BIT(0)

#define UB913_REG_GENERAL_CFG			0x03
#define UB913_REG_MODE_SEL			0x05

#define UB913_REG_GPIO_CFG(n)			(0x0d + (n))
#define UB913_REG_GPIO_CFG_ENABLE(n)		BIT(0 + (n) * 4)
#define UB913_REG_GPIO_CFG_DIR_INPUT(n)		BIT(1 + (n) * 4)
#define UB913_REG_GPIO_CFG_REMOTE_EN(n)		BIT(2 + (n) * 4)
#define UB913_REG_GPIO_CFG_OUT_VAL(n)		BIT(3 + (n) * 4)
#define UB913_REG_GPIO_CFG_MASK(n)		(0xf << ((n) * 4))

struct ub913_data {
	struct i2c_client	*client;
	struct regmap		*regmap;

	u32			gpio_func[UB913_NUM_GPIOS];

	struct gpio_chip	gpio_chip;
	char			gpio_chip_name[64];

	struct v4l2_subdev	sd;
	struct media_pad	pads[2];

	struct v4l2_async_notifier	notifier;

	struct v4l2_subdev	*source_sd;
};

static inline struct ub913_data *sd_to_ub913(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub913_data, sd);
}

static int ub913_read(const struct ub913_data *priv, u8 reg, u8 *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret < 0) {
		dev_err(&priv->client->dev,
			"Cannot read register 0x%02x: %d!\n", reg, ret);
		return ret;
	}

	*val = v;
	return 0;
}

static int ub913_write(const struct ub913_data *priv, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret < 0)
		dev_err(&priv->client->dev,
			"Cannot write register 0x%02x: %d!\n", reg, ret);

	return ret;
}

/*
 * GPIO chip
 */
static int ub913_gpio_direction_out(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	struct ub913_data *priv = gpiochip_get_data(gc);
	unsigned int reg_idx;
	unsigned int field_idx;
	int ret;

	reg_idx = offset / 2;
	field_idx = offset % 2;

	ret = regmap_update_bits(
		priv->regmap, UB913_REG_GPIO_CFG(reg_idx),
		UB913_REG_GPIO_CFG_MASK(field_idx),
		UB913_REG_GPIO_CFG_ENABLE(field_idx) |
			(value ? UB913_REG_GPIO_CFG_OUT_VAL(field_idx) : 0));

	return ret;
}

static void ub913_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	ub913_gpio_direction_out(gc, offset, value);
}

static int ub913_gpio_of_xlate(struct gpio_chip *gc,
			       const struct of_phandle_args *gpiospec,
			       u32 *flags)
{
	if (flags)
		*flags = gpiospec->args[1];

	return gpiospec->args[0];
}

static int ub913_gpiochip_probe(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct gpio_chip *gc = &priv->gpio_chip;
	int err;

	scnprintf(priv->gpio_chip_name, sizeof(priv->gpio_chip_name), "%s",
		  dev_name(dev));

	gc->label = priv->gpio_chip_name;
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->can_sleep = 1;
	gc->ngpio = UB913_NUM_GPIOS;
	gc->direction_output = ub913_gpio_direction_out;
	gc->set = ub913_gpio_set;
	gc->of_xlate = ub913_gpio_of_xlate;
	gc->of_node = priv->client->dev.of_node;
	gc->of_gpio_n_cells = 2;

	err = gpiochip_add_data(gc, priv);
	if (err) {
		dev_err(dev, "Failed to add GPIOs: %d\n", err);
		return err;
	}

	return 0;
}

static void ub913_gpiochip_remove(struct ub913_data *priv)
{
	gpiochip_remove(&priv->gpio_chip);
}

// XXX disable for now
#if 0
static void ub913_configure_gpios(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	u8 gpio_reg_val[2] = { 0 };
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->gpio_func); i++) {
		unsigned int reg_idx;
		unsigned int field_idx;

		reg_idx = i / 2;
		field_idx = i % 2;

		switch (priv->gpio_func[i]) {
		case ub913_GPIO_FUNC_UNUSED:
			break;
		case ub913_GPIO_FUNC_OUTPUT:
			gpio_reg_val[reg_idx] |=
				UB913_REG_GPIO_CFG_ENABLE(field_idx);
			break;
		case ub913_GPIO_FUNC_INPUT:
			gpio_reg_val[reg_idx] |=
				UB913_REG_GPIO_CFG_ENABLE(field_idx) |
				UB913_REG_GPIO_CFG_DIR_INPUT(field_idx);
			break;
		case ub913_GPIO_FUNC_OUTPUT_REMOTE:
			gpio_reg_val[reg_idx] |=
				UB913_REG_GPIO_CFG_ENABLE(field_idx) |
				UB913_REG_GPIO_CFG_REMOTE_EN(field_idx);
			break;
		default:
			dev_err(dev,
				"Unknown gpio-functions value %u, GPIO%d will be unused",
				priv->gpio_func[i], i);
			break;
		}
	}

	ub913_write(priv, UB913_REG_GPIO_CFG(0), gpio_reg_val[0]);
	ub913_write(priv, UB913_REG_GPIO_CFG(1), gpio_reg_val[1]);
}
#endif

/*
 * Reset via registers (useful from remote).
 * Note: the procedure is undocumented, but this one seems to work.
 */
static void ub913_soft_reset(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	int retries;

	ub913_write(priv, UB913_REG_RESET_CTL,
		    UB913_REG_RESET_CTL_DIGITAL_RESET_0);

	usleep_range(10000, 30000);

	retries = 10;
	while (retries-- > 0) {
		int ret;
		u8 v;

		ret = ub913_read(priv, UB913_REG_RESET_CTL, &v);

		if (ret >= 0 &&
		    (v & UB913_REG_RESET_CTL_DIGITAL_RESET_0) == 0) {
			dev_dbg(dev, "reset done\n");
			break;
		}

		usleep_range(1000, 3000);
	}

	if (retries == 0)
		dev_err(dev, "reset timeout\n");
}

static int ub913_parse_dt(struct ub913_data *priv)
{
	struct device_node *np = priv->client->dev.of_node;
	struct device *dev = &priv->client->dev;
	int err;

	if (!np) {
		dev_err(dev, "OF: no device tree node!\n");
		return -ENOENT;
	}

	/* optional, if absent all GPIO pins are unused */
	err = of_property_read_u32_array(np, "gpio-functions", priv->gpio_func,
					 ARRAY_SIZE(priv->gpio_func));
	if (err && err != -EINVAL)
		dev_err(dev, "DT: invalid gpio-functions property (%d)", err);

	return 0;
}

static const struct regmap_config ub913_regmap_config = {
	.name = "ds90ub913",
	.reg_bits = 8,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_DEFAULT,
	.val_format_endian = REGMAP_ENDIAN_DEFAULT,
};

/*
 * V4L2
 */

static int ub913_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub913_data *priv = sd_to_ub913(sd);

	return v4l2_subdev_call(priv->source_sd, video, s_stream, enable);
}

static const struct v4l2_subdev_video_ops ub913_video_ops = {
	.s_stream = ub913_s_stream,
};

static const struct v4l2_subdev_ops ub913_subdev_ops = {
	.video = &ub913_video_ops,
};

static const struct media_entity_operations ub913_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ub913_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *source_subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub913_data *priv = sd_to_ub913(notifier->sd);
	struct device *dev = &priv->client->dev;
	unsigned int src_pad;
	int ret;

	dev_dbg(dev, "Bind %s\n", source_subdev->name);

	ret = media_entity_get_fwnode_pad(&source_subdev->entity,
	                                  source_subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n", source_subdev->name);
		return ret;
	}

	priv->source_sd = source_subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source_subdev->entity, src_pad,
				    &priv->sd.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:0\n",
			source_subdev->name, src_pad, priv->sd.name);
		return ret;
	}

	dev_dbg(dev, "Bound %s:%u\n", source_subdev->name, src_pad);

	dev_dbg(dev, "All subdevs bound\n");

	return 0;
}

static void ub913_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *source_subdev,
				  struct v4l2_async_subdev *asd)
{
	struct ub913_data *priv = sd_to_ub913(notifier->sd);
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unbind %s\n", source_subdev->name);
}
static const struct v4l2_async_notifier_operations ub913_notify_ops = {
	.bound = ub913_notify_bound,
	.unbind = ub913_notify_unbind,
};

static int ub913_v4l2_notifier_register(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_async_subdev *asd;
	struct device_node *ep_node;
	int ret;

	dev_dbg(dev, "register async notif\n");

	v4l2_async_notifier_init(&priv->notifier);

	ep_node = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (!ep_node) {
		dev_err(dev, "No graph endpoint\n");
		return -ENODEV;
	}

	asd = v4l2_async_notifier_add_fwnode_remote_subdev(
		&priv->notifier, of_fwnode_handle(ep_node), sizeof(struct v4l2_async_subdev));

	of_node_put(ep_node);

	if (IS_ERR(asd)) {
		dev_err(dev, "Failed to add subdev: %ld", PTR_ERR(asd));
		v4l2_async_notifier_cleanup(&priv->notifier);
		return PTR_ERR(asd);
	}

	priv->notifier.ops = &ub913_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void ub913_v4l2_notifier_unregister(struct ub913_data *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unregister async notif\n");

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

static int ub913_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub913_data *priv;
	int ret;

	dev_dbg(dev, "probing, addr 0x%02x\n", client->addr);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->regmap = devm_regmap_init_i2c(client, &ub913_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(priv->regmap);
	}

	ret = ub913_parse_dt(priv);
	if (ret)
		goto err_parse_dt;

	ub913_soft_reset(priv);

	ret = ub913_gpiochip_probe(priv);
	if (ret) {
		dev_err(dev, "Failed to init gpiochip\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &ub913_subdev_ops);
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &ub913_entity_ops;

	priv->pads[0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 2, priv->pads);
	if (ret) {
		dev_err(dev, "Failed to init pads\n");
		return ret;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(dev, "v4l2_async_register_subdev error: %d\n", ret);
		return ret;
	}

	ret = ub913_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "v4l2 subdev notifier register failed: %d\n", ret);
		return ret;
	}

	//ub913_configure_gpios(priv);

	dev_dbg(dev, "Successfully probed\n");

	return 0;

err_parse_dt:
	return ret;
}

static int ub913_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ub913_data *priv = sd_to_ub913(sd);

	dev_dbg(&client->dev, "Removing\n");

	ub913_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);

	ub913_gpiochip_remove(priv);

	return 0;
}

static const struct i2c_device_id ub913_id[] = { { "ds90ub913a-q1", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, ub913_id);

#ifdef CONFIG_OF
static const struct of_device_id ub913_dt_ids[] = {
	{ .compatible = "ti,ds90ub913a-q1", },
	{}
};
MODULE_DEVICE_TABLE(of, ub913_dt_ids);
#endif

static struct i2c_driver ds90ub913_driver = {
	.probe_new	= ub913_probe,
	.remove		= ub913_remove,
	.id_table	= ub913_id,
	.driver = {
		.name	= "ds90ub913a",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub913_dt_ids),
	},
};

module_i2c_driver(ds90ub913_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Texas Instruments DS90UB913-Q1 CSI-2 serializer driver");
MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
