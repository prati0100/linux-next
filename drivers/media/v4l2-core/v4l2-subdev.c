// SPDX-License-Identifier: GPL-2.0-only
/*
 * V4L2 sub-device
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *	    Sakari Ailus <sakari.ailus@iki.fi>
 */

#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/export.h>
#include <linux/version.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>

#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
static int subdev_fh_init(struct v4l2_subdev_fh *fh, struct v4l2_subdev *sd)
{
	if (sd->entity.num_pads) {
		fh->pad = v4l2_subdev_alloc_pad_config(sd);
		if (fh->pad == NULL)
			return -ENOMEM;
	}

	return 0;
}

static void subdev_fh_free(struct v4l2_subdev_fh *fh)
{
	v4l2_subdev_free_pad_config(fh->pad);
	fh->pad = NULL;
}

static int subdev_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_subdev_fh *subdev_fh;
	int ret;

	subdev_fh = kzalloc(sizeof(*subdev_fh), GFP_KERNEL);
	if (subdev_fh == NULL)
		return -ENOMEM;

	ret = subdev_fh_init(subdev_fh, sd);
	if (ret) {
		kfree(subdev_fh);
		return ret;
	}

	v4l2_fh_init(&subdev_fh->vfh, vdev);
	v4l2_fh_add(&subdev_fh->vfh);
	file->private_data = &subdev_fh->vfh;
#if defined(CONFIG_MEDIA_CONTROLLER)
	if (sd->v4l2_dev->mdev && sd->entity.graph_obj.mdev->dev) {
		struct module *owner;

		owner = sd->entity.graph_obj.mdev->dev->driver->owner;
		if (!try_module_get(owner)) {
			ret = -EBUSY;
			goto err;
		}
		subdev_fh->owner = owner;
	}
#endif

	if (sd->internal_ops && sd->internal_ops->open) {
		ret = sd->internal_ops->open(sd, subdev_fh);
		if (ret < 0)
			goto err;
	}

	return 0;

err:
	module_put(subdev_fh->owner);
	v4l2_fh_del(&subdev_fh->vfh);
	v4l2_fh_exit(&subdev_fh->vfh);
	subdev_fh_free(subdev_fh);
	kfree(subdev_fh);

	return ret;
}

static int subdev_close(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *vfh = file->private_data;
	struct v4l2_subdev_fh *subdev_fh = to_v4l2_subdev_fh(vfh);

	if (sd->internal_ops && sd->internal_ops->close)
		sd->internal_ops->close(sd, subdev_fh);
	module_put(subdev_fh->owner);
	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	subdev_fh_free(subdev_fh);
	kfree(subdev_fh);
	file->private_data = NULL;

	return 0;
}
#else /* CONFIG_VIDEO_V4L2_SUBDEV_API */
static int subdev_open(struct file *file)
{
	return -ENODEV;
}

static int subdev_close(struct file *file)
{
	return -ENODEV;
}
#endif /* CONFIG_VIDEO_V4L2_SUBDEV_API */

static inline int check_which(u32 which)
{
	if (which != V4L2_SUBDEV_FORMAT_TRY &&
	    which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	return 0;
}

static inline int check_pad(struct v4l2_subdev *sd, u32 pad)
{
#if defined(CONFIG_MEDIA_CONTROLLER)
	if (sd->entity.num_pads) {
		if (pad >= sd->entity.num_pads)
			return -EINVAL;
		return 0;
	}
#endif
	/* allow pad 0 on subdevices not registered as media entities */
	if (pad > 0)
		return -EINVAL;
	return 0;
}

static int check_cfg(u32 which, struct v4l2_subdev_pad_config *cfg)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY && !cfg)
		return -EINVAL;

	return 0;
}

static inline int check_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	if (!format)
		return -EINVAL;

	return check_which(format->which) ? : check_pad(sd, format->pad) ? :
	       check_cfg(format->which, cfg);
}

static int call_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	return check_format(sd, cfg, format) ? :
	       sd->ops->pad->get_fmt(sd, cfg, format);
}

static int call_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	return check_format(sd, cfg, format) ? :
	       sd->ops->pad->set_fmt(sd, cfg, format);
}

static int call_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	if (!code)
		return -EINVAL;

	return check_which(code->which) ? : check_pad(sd, code->pad) ? :
	       check_cfg(code->which, cfg) ? :
	       sd->ops->pad->enum_mbus_code(sd, cfg, code);
}

static int call_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	if (!fse)
		return -EINVAL;

	return check_which(fse->which) ? : check_pad(sd, fse->pad) ? :
	       check_cfg(fse->which, cfg) ? :
	       sd->ops->pad->enum_frame_size(sd, cfg, fse);
}

static inline int check_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_frame_interval *fi)
{
	if (!fi)
		return -EINVAL;

	return check_pad(sd, fi->pad);
}

static int call_g_frame_interval(struct v4l2_subdev *sd,
				 struct v4l2_subdev_frame_interval *fi)
{
	return check_frame_interval(sd, fi) ? :
	       sd->ops->video->g_frame_interval(sd, fi);
}

static int call_s_frame_interval(struct v4l2_subdev *sd,
				 struct v4l2_subdev_frame_interval *fi)
{
	return check_frame_interval(sd, fi) ? :
	       sd->ops->video->s_frame_interval(sd, fi);
}

static int call_enum_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_interval_enum *fie)
{
	if (!fie)
		return -EINVAL;

	return check_which(fie->which) ? : check_pad(sd, fie->pad) ? :
	       check_cfg(fie->which, cfg) ? :
	       sd->ops->pad->enum_frame_interval(sd, cfg, fie);
}

static inline int check_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_selection *sel)
{
	if (!sel)
		return -EINVAL;

	return check_which(sel->which) ? : check_pad(sd, sel->pad) ? :
	       check_cfg(sel->which, cfg);
}

static int call_get_selection(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_selection *sel)
{
	return check_selection(sd, cfg, sel) ? :
	       sd->ops->pad->get_selection(sd, cfg, sel);
}

static int call_set_selection(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_selection *sel)
{
	return check_selection(sd, cfg, sel) ? :
	       sd->ops->pad->set_selection(sd, cfg, sel);
}

static inline int check_edid(struct v4l2_subdev *sd,
			     struct v4l2_subdev_edid *edid)
{
	if (!edid)
		return -EINVAL;

	if (edid->blocks && edid->edid == NULL)
		return -EINVAL;

	return check_pad(sd, edid->pad);
}

static int call_get_edid(struct v4l2_subdev *sd, struct v4l2_subdev_edid *edid)
{
	return check_edid(sd, edid) ? : sd->ops->pad->get_edid(sd, edid);
}

static int call_set_edid(struct v4l2_subdev *sd, struct v4l2_subdev_edid *edid)
{
	return check_edid(sd, edid) ? : sd->ops->pad->set_edid(sd, edid);
}

static int call_dv_timings_cap(struct v4l2_subdev *sd,
			       struct v4l2_dv_timings_cap *cap)
{
	if (!cap)
		return -EINVAL;

	return check_pad(sd, cap->pad) ? :
	       sd->ops->pad->dv_timings_cap(sd, cap);
}

static int call_enum_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_enum_dv_timings *dvt)
{
	if (!dvt)
		return -EINVAL;

	return check_pad(sd, dvt->pad) ? :
	       sd->ops->pad->enum_dv_timings(sd, dvt);
}

static int call_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *config)
{
	return check_pad(sd, pad) ? :
	       sd->ops->pad->get_mbus_config(sd, pad, config);
}

static int call_set_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *config)
{
	return check_pad(sd, pad) ? :
	       sd->ops->pad->get_mbus_config(sd, pad, config);
}

static const struct v4l2_subdev_pad_ops v4l2_subdev_call_pad_wrappers = {
	.get_fmt		= call_get_fmt,
	.set_fmt		= call_set_fmt,
	.enum_mbus_code		= call_enum_mbus_code,
	.enum_frame_size	= call_enum_frame_size,
	.enum_frame_interval	= call_enum_frame_interval,
	.get_selection		= call_get_selection,
	.set_selection		= call_set_selection,
	.get_edid		= call_get_edid,
	.set_edid		= call_set_edid,
	.dv_timings_cap		= call_dv_timings_cap,
	.enum_dv_timings	= call_enum_dv_timings,
	.get_mbus_config	= call_get_mbus_config,
	.set_mbus_config	= call_set_mbus_config,
};

static const struct v4l2_subdev_video_ops v4l2_subdev_call_video_wrappers = {
	.g_frame_interval	= call_g_frame_interval,
	.s_frame_interval	= call_s_frame_interval,
};

const struct v4l2_subdev_ops v4l2_subdev_call_wrappers = {
	.pad	= &v4l2_subdev_call_pad_wrappers,
	.video	= &v4l2_subdev_call_video_wrappers,
};
EXPORT_SYMBOL(v4l2_subdev_call_wrappers);

#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
static long subdev_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *vfh = file->private_data;
	struct v4l2_subdev_fh *subdev_fh = to_v4l2_subdev_fh(vfh);
	bool ro_subdev = test_bit(V4L2_FL_SUBDEV_RO_DEVNODE, &vdev->flags);
	int rval;

	switch (cmd) {
	case VIDIOC_SUBDEV_QUERYCAP: {
		struct v4l2_subdev_capability *cap = arg;

		memset(cap->reserved, 0, sizeof(cap->reserved));
		cap->version = LINUX_VERSION_CODE;
		cap->capabilities = ro_subdev ? V4L2_SUBDEV_CAP_RO_SUBDEV : 0;

		return 0;
	}

	case VIDIOC_QUERYCTRL:
		/*
		 * TODO: this really should be folded into v4l2_queryctrl (this
		 * currently returns -EINVAL for NULL control handlers).
		 * However, v4l2_queryctrl() is still called directly by
		 * drivers as well and until that has been addressed I believe
		 * it is safer to do the check here. The same is true for the
		 * other control ioctls below.
		 */
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_queryctrl(vfh->ctrl_handler, arg);

	case VIDIOC_QUERY_EXT_CTRL:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_query_ext_ctrl(vfh->ctrl_handler, arg);

	case VIDIOC_QUERYMENU:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_querymenu(vfh->ctrl_handler, arg);

	case VIDIOC_G_CTRL:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_g_ctrl(vfh->ctrl_handler, arg);

	case VIDIOC_S_CTRL:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_s_ctrl(vfh, vfh->ctrl_handler, arg);

	case VIDIOC_G_EXT_CTRLS:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_g_ext_ctrls(vfh->ctrl_handler,
					vdev, sd->v4l2_dev->mdev, arg);

	case VIDIOC_S_EXT_CTRLS:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_s_ext_ctrls(vfh, vfh->ctrl_handler,
					vdev, sd->v4l2_dev->mdev, arg);

	case VIDIOC_TRY_EXT_CTRLS:
		if (!vfh->ctrl_handler)
			return -ENOTTY;
		return v4l2_try_ext_ctrls(vfh->ctrl_handler,
					  vdev, sd->v4l2_dev->mdev, arg);

	case VIDIOC_DQEVENT:
		if (!(sd->flags & V4L2_SUBDEV_FL_HAS_EVENTS))
			return -ENOIOCTLCMD;

		return v4l2_event_dequeue(vfh, arg, file->f_flags & O_NONBLOCK);

	case VIDIOC_DQEVENT_TIME32: {
		struct v4l2_event_time32 *ev32 = arg;
		struct v4l2_event ev = { };

		if (!(sd->flags & V4L2_SUBDEV_FL_HAS_EVENTS))
			return -ENOIOCTLCMD;

		rval = v4l2_event_dequeue(vfh, &ev, file->f_flags & O_NONBLOCK);

		*ev32 = (struct v4l2_event_time32) {
			.type		= ev.type,
			.pending	= ev.pending,
			.sequence	= ev.sequence,
			.timestamp.tv_sec  = ev.timestamp.tv_sec,
			.timestamp.tv_nsec = ev.timestamp.tv_nsec,
			.id		= ev.id,
		};

		memcpy(&ev32->u, &ev.u, sizeof(ev.u));
		memcpy(&ev32->reserved, &ev.reserved, sizeof(ev.reserved));

		return rval;
	}

	case VIDIOC_SUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, subscribe_event, vfh, arg);

	case VIDIOC_UNSUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, unsubscribe_event, vfh, arg);

#ifdef CONFIG_VIDEO_ADV_DEBUG
	case VIDIOC_DBG_G_REGISTER:
	{
		struct v4l2_dbg_register *p = arg;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		return v4l2_subdev_call(sd, core, g_register, p);
	}
	case VIDIOC_DBG_S_REGISTER:
	{
		struct v4l2_dbg_register *p = arg;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		return v4l2_subdev_call(sd, core, s_register, p);
	}
	case VIDIOC_DBG_G_CHIP_INFO:
	{
		struct v4l2_dbg_chip_info *p = arg;

		if (p->match.type != V4L2_CHIP_MATCH_SUBDEV || p->match.addr)
			return -EINVAL;
		if (sd->ops->core && sd->ops->core->s_register)
			p->flags |= V4L2_CHIP_FL_WRITABLE;
		if (sd->ops->core && sd->ops->core->g_register)
			p->flags |= V4L2_CHIP_FL_READABLE;
		strscpy(p->name, sd->name, sizeof(p->name));
		return 0;
	}
#endif

	case VIDIOC_LOG_STATUS: {
		int ret;

		pr_info("%s: =================  START STATUS  =================\n",
			sd->name);
		ret = v4l2_subdev_call(sd, core, log_status);
		pr_info("%s: ==================  END STATUS  ==================\n",
			sd->name);
		return ret;
	}

	case VIDIOC_SUBDEV_G_FMT: {
		struct v4l2_subdev_format *format = arg;

		memset(format->reserved, 0, sizeof(format->reserved));
		memset(format->format.reserved, 0, sizeof(format->format.reserved));
		return v4l2_subdev_call(sd, pad, get_fmt, subdev_fh->pad, format);
	}

	case VIDIOC_SUBDEV_S_FMT: {
		struct v4l2_subdev_format *format = arg;

		if (format->which != V4L2_SUBDEV_FORMAT_TRY && ro_subdev)
			return -EPERM;

		memset(format->reserved, 0, sizeof(format->reserved));
		memset(format->format.reserved, 0, sizeof(format->format.reserved));
		return v4l2_subdev_call(sd, pad, set_fmt, subdev_fh->pad, format);
	}

	case VIDIOC_SUBDEV_G_CROP: {
		struct v4l2_subdev_crop *crop = arg;
		struct v4l2_subdev_selection sel;

		memset(crop->reserved, 0, sizeof(crop->reserved));
		memset(&sel, 0, sizeof(sel));
		sel.which = crop->which;
		sel.pad = crop->pad;
		sel.target = V4L2_SEL_TGT_CROP;

		rval = v4l2_subdev_call(
			sd, pad, get_selection, subdev_fh->pad, &sel);

		crop->rect = sel.r;

		return rval;
	}

	case VIDIOC_SUBDEV_S_CROP: {
		struct v4l2_subdev_crop *crop = arg;
		struct v4l2_subdev_selection sel;

		if (crop->which != V4L2_SUBDEV_FORMAT_TRY && ro_subdev)
			return -EPERM;

		memset(crop->reserved, 0, sizeof(crop->reserved));
		memset(&sel, 0, sizeof(sel));
		sel.which = crop->which;
		sel.pad = crop->pad;
		sel.target = V4L2_SEL_TGT_CROP;
		sel.r = crop->rect;

		rval = v4l2_subdev_call(
			sd, pad, set_selection, subdev_fh->pad, &sel);

		crop->rect = sel.r;

		return rval;
	}

	case VIDIOC_SUBDEV_ENUM_MBUS_CODE: {
		struct v4l2_subdev_mbus_code_enum *code = arg;

		memset(code->reserved, 0, sizeof(code->reserved));
		return v4l2_subdev_call(sd, pad, enum_mbus_code, subdev_fh->pad,
					code);
	}

	case VIDIOC_SUBDEV_ENUM_FRAME_SIZE: {
		struct v4l2_subdev_frame_size_enum *fse = arg;

		memset(fse->reserved, 0, sizeof(fse->reserved));
		return v4l2_subdev_call(sd, pad, enum_frame_size, subdev_fh->pad,
					fse);
	}

	case VIDIOC_SUBDEV_G_FRAME_INTERVAL: {
		struct v4l2_subdev_frame_interval *fi = arg;

		memset(fi->reserved, 0, sizeof(fi->reserved));
		return v4l2_subdev_call(sd, video, g_frame_interval, arg);
	}

	case VIDIOC_SUBDEV_S_FRAME_INTERVAL: {
		struct v4l2_subdev_frame_interval *fi = arg;

		if (ro_subdev)
			return -EPERM;

		memset(fi->reserved, 0, sizeof(fi->reserved));
		return v4l2_subdev_call(sd, video, s_frame_interval, arg);
	}

	case VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL: {
		struct v4l2_subdev_frame_interval_enum *fie = arg;

		memset(fie->reserved, 0, sizeof(fie->reserved));
		return v4l2_subdev_call(sd, pad, enum_frame_interval, subdev_fh->pad,
					fie);
	}

	case VIDIOC_SUBDEV_G_SELECTION: {
		struct v4l2_subdev_selection *sel = arg;

		memset(sel->reserved, 0, sizeof(sel->reserved));
		return v4l2_subdev_call(
			sd, pad, get_selection, subdev_fh->pad, sel);
	}

	case VIDIOC_SUBDEV_S_SELECTION: {
		struct v4l2_subdev_selection *sel = arg;

		if (sel->which != V4L2_SUBDEV_FORMAT_TRY && ro_subdev)
			return -EPERM;

		memset(sel->reserved, 0, sizeof(sel->reserved));
		return v4l2_subdev_call(
			sd, pad, set_selection, subdev_fh->pad, sel);
	}

	case VIDIOC_G_EDID: {
		struct v4l2_subdev_edid *edid = arg;

		return v4l2_subdev_call(sd, pad, get_edid, edid);
	}

	case VIDIOC_S_EDID: {
		struct v4l2_subdev_edid *edid = arg;

		return v4l2_subdev_call(sd, pad, set_edid, edid);
	}

	case VIDIOC_SUBDEV_DV_TIMINGS_CAP: {
		struct v4l2_dv_timings_cap *cap = arg;

		return v4l2_subdev_call(sd, pad, dv_timings_cap, cap);
	}

	case VIDIOC_SUBDEV_ENUM_DV_TIMINGS: {
		struct v4l2_enum_dv_timings *dvt = arg;

		return v4l2_subdev_call(sd, pad, enum_dv_timings, dvt);
	}

	case VIDIOC_SUBDEV_QUERY_DV_TIMINGS:
		return v4l2_subdev_call(sd, video, query_dv_timings, arg);

	case VIDIOC_SUBDEV_G_DV_TIMINGS:
		return v4l2_subdev_call(sd, video, g_dv_timings, arg);

	case VIDIOC_SUBDEV_S_DV_TIMINGS:
		if (ro_subdev)
			return -EPERM;

		return v4l2_subdev_call(sd, video, s_dv_timings, arg);

	case VIDIOC_SUBDEV_G_STD:
		return v4l2_subdev_call(sd, video, g_std, arg);

	case VIDIOC_SUBDEV_S_STD: {
		v4l2_std_id *std = arg;

		if (ro_subdev)
			return -EPERM;

		return v4l2_subdev_call(sd, video, s_std, *std);
	}

	case VIDIOC_SUBDEV_ENUMSTD: {
		struct v4l2_standard *p = arg;
		v4l2_std_id id;

		if (v4l2_subdev_call(sd, video, g_tvnorms, &id))
			return -EINVAL;

		return v4l_video_std_enumstd(p, id);
	}

	case VIDIOC_SUBDEV_QUERYSTD:
		return v4l2_subdev_call(sd, video, querystd, arg);

	case VIDIOC_SUBDEV_G_ROUTING: {
		struct v4l2_subdev_routing *routing = arg;
		struct v4l2_subdev_krouting krouting = {
			.which = routing->which,
			.num_routes = routing->num_routes,
			.routes = (struct v4l2_subdev_route *)(uintptr_t)
					  routing->routes,
		};
		int ret;

		ret = v4l2_subdev_call(sd, pad, get_routing, &krouting);

		routing->num_routes = krouting.num_routes;

		return ret;
	}

	case VIDIOC_SUBDEV_S_ROUTING: {
		struct v4l2_subdev_routing *routing = arg;
		struct v4l2_subdev_route *routes =
			(struct v4l2_subdev_route *)(uintptr_t)routing->routes;
		struct v4l2_subdev_krouting krouting = {};
		unsigned int i;

		if (routing->which != V4L2_SUBDEV_FORMAT_TRY && ro_subdev)
			return -EPERM;

		for (i = 0; i < routing->num_routes; ++i) {
			if (routes[i].sink_pad >= sd->entity.num_pads ||
			    routes[i].source_pad >= sd->entity.num_pads)
				return -EINVAL;

			if (!(sd->entity.pads[routes[i].sink_pad].flags &
			      MEDIA_PAD_FL_SINK) ||
			    !(sd->entity.pads[routes[i].source_pad].flags &
			      MEDIA_PAD_FL_SOURCE))
				return -EINVAL;
		}

		krouting.which = routing->which;
		krouting.num_routes = routing->num_routes;
		krouting.routes = routes;

		return v4l2_subdev_call(sd, pad, set_routing, &krouting);
	}

	default:
		return v4l2_subdev_call(sd, core, ioctl, cmd, arg);
	}

	return 0;
}

static long subdev_do_ioctl_lock(struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct mutex *lock = vdev->lock;
	long ret = -ENODEV;

	if (lock && mutex_lock_interruptible(lock))
		return -ERESTARTSYS;
	if (video_is_registered(vdev))
		ret = subdev_do_ioctl(file, cmd, arg);
	if (lock)
		mutex_unlock(lock);
	return ret;
}

static long subdev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, subdev_do_ioctl_lock);
}

#ifdef CONFIG_COMPAT
static long subdev_compat_ioctl32(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	return v4l2_subdev_call(sd, core, compat_ioctl32, cmd, arg);
}
#endif

#else /* CONFIG_VIDEO_V4L2_SUBDEV_API */
static long subdev_ioctl(struct file *file, unsigned int cmd,
			 unsigned long arg)
{
	return -ENODEV;
}

#ifdef CONFIG_COMPAT
static long subdev_compat_ioctl32(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	return -ENODEV;
}
#endif
#endif /* CONFIG_VIDEO_V4L2_SUBDEV_API */

static __poll_t subdev_poll(struct file *file, poll_table *wait)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *fh = file->private_data;

	if (!(sd->flags & V4L2_SUBDEV_FL_HAS_EVENTS))
		return EPOLLERR;

	poll_wait(file, &fh->wait, wait);

	if (v4l2_event_pending(fh))
		return EPOLLPRI;

	return 0;
}

const struct v4l2_file_operations v4l2_subdev_fops = {
	.owner = THIS_MODULE,
	.open = subdev_open,
	.unlocked_ioctl = subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = subdev_compat_ioctl32,
#endif
	.release = subdev_close,
	.poll = subdev_poll,
};

#ifdef CONFIG_MEDIA_CONTROLLER

int v4l2_subdev_get_fwnode_pad_1_to_1(struct media_entity *entity,
				      struct fwnode_endpoint *endpoint)
{
	struct fwnode_handle *fwnode;
	struct v4l2_subdev *sd;

	if (!is_media_entity_v4l2_subdev(entity))
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);

	fwnode = fwnode_graph_get_port_parent(endpoint->local_fwnode);
	fwnode_handle_put(fwnode);

	if (dev_fwnode(sd->dev) == fwnode)
		return endpoint->port;

	return -ENXIO;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_get_fwnode_pad_1_to_1);

int v4l2_subdev_link_validate_default(struct v4l2_subdev *sd,
				      struct media_link *link,
				      struct v4l2_subdev_format *source_fmt,
				      struct v4l2_subdev_format *sink_fmt)
{
	/* The width, height and code must match. */
	if (source_fmt->format.width != sink_fmt->format.width
	    || source_fmt->format.height != sink_fmt->format.height
	    || source_fmt->format.code != sink_fmt->format.code)
		return -EPIPE;

	/* The field order must match, or the sink field order must be NONE
	 * to support interlaced hardware connected to bridges that support
	 * progressive formats only.
	 */
	if (source_fmt->format.field != sink_fmt->format.field &&
	    sink_fmt->format.field != V4L2_FIELD_NONE)
		return -EPIPE;

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_link_validate_default);

static int
v4l2_subdev_link_validate_get_format(struct media_pad *pad,
				     struct v4l2_subdev_format *fmt)
{
	if (is_media_entity_v4l2_subdev(pad->entity)) {
		struct v4l2_subdev *sd =
			media_entity_to_v4l2_subdev(pad->entity);

		fmt->which = V4L2_SUBDEV_FORMAT_ACTIVE;
		fmt->pad = pad->index;
		return v4l2_subdev_call(sd, pad, get_fmt, NULL, fmt);
	}

	WARN(pad->entity->function != MEDIA_ENT_F_IO_V4L,
	     "Driver bug! Wrong media entity type 0x%08x, entity %s\n",
	     pad->entity->function, pad->entity->name);

	return -EINVAL;
}

int v4l2_subdev_get_krouting(struct v4l2_subdev *sd,
			     struct v4l2_subdev_krouting *routing)
{
	int ret;

	routing->which = V4L2_SUBDEV_FORMAT_ACTIVE;
	routing->routes = NULL;
	routing->num_routes = 0;

	ret = v4l2_subdev_call(sd, pad, get_routing, routing);
	if (ret == 0)
		return 0;
	if (ret != -ENOSPC)
		return ret;

	routing->routes = kvmalloc_array(routing->num_routes,
					 sizeof(*routing->routes), GFP_KERNEL);
	if (!routing->routes)
		return -ENOMEM;

	ret = v4l2_subdev_call(sd, pad, get_routing, routing);
	if (ret) {
		kvfree(routing->routes);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_get_krouting);

void v4l2_subdev_free_routing(struct v4l2_subdev_krouting *routing)
{
	kvfree(routing->routes);
	routing->routes = NULL;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_free_routing);

void v4l2_subdev_cpy_routing(struct v4l2_subdev_krouting *dst,
			     const struct v4l2_subdev_krouting *src)
{
	memcpy(dst->routes, src->routes,
	       src->num_routes * sizeof(*src->routes));
	dst->num_routes = src->num_routes;
	dst->which = src->which;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_cpy_routing);

int v4l2_subdev_dup_routing(struct v4l2_subdev_krouting *dst,
			    const struct v4l2_subdev_krouting *src)
{
	if (dst->routes)
		kvfree(dst->routes);

	if (src->num_routes == 0) {
		dst->which = src->which;
		dst->routes = NULL;
		dst->num_routes = 0;
		return 0;
	}

	dst->routes = kvmalloc_array(src->num_routes, sizeof(*src->routes),
				     GFP_KERNEL);
	if (!dst->routes)
		return -ENOMEM;

	v4l2_subdev_cpy_routing(dst, src);

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_dup_routing);

bool v4l2_subdev_has_route(struct v4l2_subdev_krouting *routing,
			   unsigned int pad0, unsigned int pad1)
{
	unsigned int i;

	for (i = 0; i < routing->num_routes; ++i) {
		struct v4l2_subdev_route *route = &routing->routes[i];

		if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		if (route->sink_pad == pad0 && route->source_pad == pad1)
			return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_has_route);

int v4l2_subdev_get_format_dir(struct media_pad *pad, u16 stream,
			       enum v4l2_direction dir,
			       struct v4l2_subdev_format *fmt)
{
	struct device *dev = pad->entity->graph_obj.mdev->dev;
	int ret;
	int i;

	dev_dbg(dev, "%s '%s':%u:%u %s\n", __func__,
		pad->entity->name, pad->index, stream,
		dir == V4L2_DIR_SOURCEWARD ? "sourceward" : "sinkward");

	while (true) {
		struct v4l2_subdev_krouting routing;
		struct v4l2_subdev_route *route;

		if (pad->entity->obj_type != MEDIA_ENTITY_TYPE_V4L2_SUBDEV)
			return -EINVAL;

		ret = v4l2_subdev_link_validate_get_format(pad, fmt);
		if (ret == 0)
			return 0;
		else if (ret != -ENOIOCTLCMD)
			return ret;

		if (pad->flags &
		    (dir == V4L2_DIR_SINKWARD ? MEDIA_PAD_FL_SOURCE :
						MEDIA_PAD_FL_SINK)) {
			pad = media_entity_remote_pad(pad);

			if (!pad)
				return -EINVAL;

			if (pad->entity->obj_type != MEDIA_ENTITY_TYPE_V4L2_SUBDEV)
				return -EINVAL;

			ret = v4l2_subdev_link_validate_get_format(pad, fmt);
			if (ret == 0)
				return 0;
			else if (ret != -ENOIOCTLCMD)
				return ret;
		}

		ret = v4l2_subdev_get_krouting(media_entity_to_v4l2_subdev(pad->entity), &routing);
		if (ret)
			return ret;

		route = NULL;
		for (i = 0; i < routing.num_routes; ++i) {
			u16 near_pad = dir == V4L2_DIR_SINKWARD ?
					       routing.routes[i].sink_pad :
					       routing.routes[i].source_pad;
			u16 near_stream = dir == V4L2_DIR_SINKWARD ?
						  routing.routes[i].sink_stream :
						  routing.routes[i].source_stream;

			if (!(routing.routes[i].flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
				continue;

			if (near_pad != pad->index)
				continue;

			if (near_stream != stream)
				continue;

			if (route) {
				dev_err(dev,
					"%s: '%s' has multiple active routes for stream %u\n",
					__func__, pad->entity->name, stream);
				v4l2_subdev_free_routing(&routing);
				return -EINVAL;
			}

			route = &routing.routes[i];
		}

		if (!route) {
			dev_err(dev, "%s: no route found in '%s' for stream %u\n",
				__func__, pad->entity->name, stream);
			v4l2_subdev_free_routing(&routing);
			return -EINVAL;
		}

		if (dir == V4L2_DIR_SINKWARD) {
			pad = &pad->entity->pads[route->source_pad];
			stream = route->source_stream;
		} else {
			pad = &pad->entity->pads[route->sink_pad];
			stream = route->sink_stream;
		}

		v4l2_subdev_free_routing(&routing);
	}
}
EXPORT_SYMBOL_GPL(v4l2_subdev_get_format_dir);

static int v4l2_subdev_link_validate_routing_stream(
	struct media_link *link, struct media_pad *sink_pad, u16 sink_stream,
	struct media_pad *source_pad, u16 source_stream)
{
	struct v4l2_subdev_format source_fmt;
	struct v4l2_subdev_format sink_fmt;
	struct v4l2_subdev *sink_sd;
	int ret;

	ret = v4l2_subdev_get_format_dir(sink_pad, sink_stream,
					 V4L2_DIR_SINKWARD, &sink_fmt);
	if (ret)
		return ret;

	ret = v4l2_subdev_get_format_dir(source_pad, source_stream,
					 V4L2_DIR_SOURCEWARD, &source_fmt);
	if (ret)
		return ret;

	sink_sd = media_entity_to_v4l2_subdev(sink_pad->entity);

	ret = v4l2_subdev_call(sink_sd, pad, link_validate, link, &source_fmt,
			       &sink_fmt);
	if (ret != -ENOIOCTLCMD)
		return ret;

	ret = v4l2_subdev_link_validate_default(sink_sd, link, &source_fmt,
						&sink_fmt);
	if (ret)
		return ret;

	return 0;
}

static int v4l2_subdev_link_validate_routing(struct media_link *link)
{
	int ret;
	unsigned int i, j;

	struct route_info {
		struct v4l2_subdev_krouting routing;
		struct media_pad *pad;
		struct v4l2_subdev *subdev;
	};

	struct route_info source_route_info = {
		.pad = link->source,
		.subdev = media_entity_to_v4l2_subdev(link->source->entity),
	};

	struct route_info sink_route_info = {
		.pad = link->sink,
		.subdev = media_entity_to_v4l2_subdev(link->sink->entity),
	};

	struct device *dev = sink_route_info.subdev->entity.graph_obj.mdev->dev;

	dev_dbg(dev, "validating link \"%s\":%u -> \"%s\":%u\n",
		link->source->entity->name, link->source->index,
		link->sink->entity->name, link->sink->index);

	ret = v4l2_subdev_get_krouting(source_route_info.subdev,
				       &source_route_info.routing);
	if (ret)
		return ret;

	ret = v4l2_subdev_get_krouting(sink_route_info.subdev,
				       &sink_route_info.routing);
	if (ret) {
		v4l2_subdev_free_routing(&source_route_info.routing);
		return ret;
	}

	/*
	 * Every active sink route needs an active source route, but it's ok
	 * to have active source routes without matching sink route.
	 */
	for (i = 0; i < sink_route_info.routing.num_routes; ++i) {
		struct v4l2_subdev_route *sink_route =
			&sink_route_info.routing.routes[i];

		if (!(sink_route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		if (sink_route->sink_pad != sink_route_info.pad->index)
			continue;

		for (j = 0; j < source_route_info.routing.num_routes; ++j) {
			struct v4l2_subdev_route *source_route =
				&source_route_info.routing.routes[j];

			if (!(source_route->flags &
			      V4L2_SUBDEV_ROUTE_FL_ACTIVE))
				continue;

			if (source_route->source_pad !=
			    source_route_info.pad->index)
				continue;

			if (source_route->source_stream !=
			    sink_route->sink_stream)
				continue;

			ret = v4l2_subdev_link_validate_routing_stream(
				link,
				&sink_route_info.pad->entity
					 ->pads[sink_route->sink_pad],
				sink_route->sink_stream,
				&source_route_info.pad->entity
					 ->pads[source_route->source_pad],
				source_route->source_stream);
			if (ret)
				goto out;

			break;
		}

		if (j == source_route_info.routing.num_routes) {
			dev_err(dev,
				"%s: no active source route found for sink route '%s':%u:%u\n",
				__func__, sink_route_info.pad->entity->name,
				sink_route->sink_pad, sink_route->sink_stream);
			ret = -EINVAL;
			goto out;
		}
	}

out:
	v4l2_subdev_free_routing(&source_route_info.routing);
	v4l2_subdev_free_routing(&sink_route_info.routing);

	return ret;
}

int v4l2_subdev_link_validate(struct media_link *link)
{
	struct v4l2_subdev *sink;
	struct v4l2_subdev_format sink_fmt, source_fmt;
	int rval;

	if (WARN_ON((link->source->flags & MEDIA_PAD_FL_MULTIPLEXED) !=
		    (link->sink->flags & MEDIA_PAD_FL_MULTIPLEXED)))
		return -EINVAL;

	if (link->source->flags & MEDIA_PAD_FL_MULTIPLEXED)
		return v4l2_subdev_link_validate_routing(link);

	rval = v4l2_subdev_link_validate_get_format(link->source, &source_fmt);
	if (rval < 0)
		return 0;

	rval = v4l2_subdev_link_validate_get_format(
		link->sink, &sink_fmt);
	if (rval < 0)
		return 0;

	sink = media_entity_to_v4l2_subdev(link->sink->entity);

	rval = v4l2_subdev_call(sink, pad, link_validate, link,
				&source_fmt, &sink_fmt);
	if (rval != -ENOIOCTLCMD)
		return rval;

	return v4l2_subdev_link_validate_default(
		sink, link, &source_fmt, &sink_fmt);
}
EXPORT_SYMBOL_GPL(v4l2_subdev_link_validate);

struct v4l2_subdev_pad_config *
v4l2_subdev_alloc_pad_config(struct v4l2_subdev *sd)
{
	struct v4l2_subdev_pad_config *cfg;
	int ret;

	if (!sd->entity.num_pads)
		return NULL;

	cfg = kvmalloc_array(sd->entity.num_pads, sizeof(*cfg),
			     GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return NULL;

	ret = v4l2_subdev_call(sd, pad, init_cfg, cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		kvfree(cfg);
		return NULL;
	}

	return cfg;
}
EXPORT_SYMBOL_GPL(v4l2_subdev_alloc_pad_config);

void v4l2_subdev_free_pad_config(struct v4l2_subdev_pad_config *cfg)
{
	kvfree(cfg);
}
EXPORT_SYMBOL_GPL(v4l2_subdev_free_pad_config);
#endif /* CONFIG_MEDIA_CONTROLLER */

void v4l2_subdev_init(struct v4l2_subdev *sd, const struct v4l2_subdev_ops *ops)
{
	INIT_LIST_HEAD(&sd->list);
	BUG_ON(!ops);
	sd->ops = ops;
	sd->v4l2_dev = NULL;
	sd->flags = 0;
	sd->name[0] = '\0';
	sd->grp_id = 0;
	sd->dev_priv = NULL;
	sd->host_priv = NULL;
#if defined(CONFIG_MEDIA_CONTROLLER)
	sd->entity.name = sd->name;
	sd->entity.obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	sd->entity.function = MEDIA_ENT_F_V4L2_SUBDEV_UNKNOWN;
#endif
}
EXPORT_SYMBOL(v4l2_subdev_init);

void v4l2_subdev_notify_event(struct v4l2_subdev *sd,
			      const struct v4l2_event *ev)
{
	v4l2_event_queue(sd->devnode, ev);
	v4l2_subdev_notify(sd, V4L2_DEVICE_NOTIFY_EVENT, (void *)ev);
}
EXPORT_SYMBOL_GPL(v4l2_subdev_notify_event);
