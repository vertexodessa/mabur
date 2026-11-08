/* Hi3516CV610 capture graph. See cv610_pipeline.h.
 *
 * Derived from waybeam_venc src/cv610_pipeline.c, generalised from that
 * project's IMX662 to a table of sensors.
 * Copyright (c) 2023 OpenIPC, MIT License.
 *
 * The MPP objects below are kernel state and outlive the process that made
 * them, which is why setup begins by clearing what a dead predecessor may
 * have left. The MPP kernel modules themselves must stay loaded: unloading
 * them under a live consumer hangs the SoC. */
#include "cv610_pipeline.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "ot_common.h"
#include "ot_common_3a.h"
#include "ot_common_isp.h"
#include "ot_common_sys.h"
#include "ot_common_vb.h"
#include "ot_common_vi.h"
#include "ot_common_video.h"
#include "ot_common_vpss.h"
#include "ot_mipi_rx.h"
#include "ot_sns_ctrl.h"
#include "ot_type.h"
#include "ss_mpi_ae.h"
#include "ss_mpi_awb.h"
#include "ss_mpi_isp.h"
#include "ss_mpi_sys.h"
#include "ss_mpi_sys_bind.h"
#include "ss_mpi_sys_mem.h"
#include "ss_mpi_vb.h"
#include "ss_mpi_vi.h"
#include "ss_mpi_vpss.h"

#define MIPI_DEV_NODE "/dev/ot_mipi_rx"

#define VI_DEV 0
#define VI_PIPE 0
#define VI_CHN 0
#define VPSS_GRP CV610_VPSS_GRP
#define VPSS_CHN CV610_VPSS_CHN

/* Frames in flight between VI and VENC. One block per stage plus one spare;
 * a larger pool only spends MMZ, which a 5 MP capture exhausts quickly. */
#define YUV_POOL_BLOCKS 4

typedef struct {
	const char *lib_path;
	const char *obj_symbol;
	td_u32 sns_id;       /* the ISP bind key; must match the plugin's */
	uint32_t width, height, fps;
	int lanes;
	int raw_bit;         /* 10 or 12 */
	int bayer;           /* ot_isp_bayer_format */
	td_u32 component_mask; /* VI data-bus mask: follows raw_bit */
	int i2c_bus;
} Cv610Sensor;

static const Cv610Sensor kSensor = {
	.lib_path = "/usr/lib/sensors/libsns_imx335.so",
	.obj_symbol = "g_sns_imx335_obj",
	.sns_id = 335,
	.width = 2592,
	.height = 1944,
	.fps = 30,
	.lanes = 4,
	.raw_bit = 12,
	.bayer = OT_ISP_BAYER_RGGB,
	.component_mask = 0xFFF00000,
	.i2c_bus = 0,
};

uint32_t cv610_pipeline_sensor_width(void) { return kSensor.width; }
uint32_t cv610_pipeline_sensor_height(void) { return kSensor.height; }
uint32_t cv610_pipeline_sensor_fps(void) { return kSensor.fps; }

#define CV610_CHECK(expr) do { \
		td_s32 check_ret = (expr); \
		if (check_ret != TD_SUCCESS) { \
			fprintf(stderr, "ERROR: venc: %s = 0x%x\n", #expr, \
				(unsigned)check_ret); \
			return -1; \
		} \
	} while (0)

static pthread_t g_isp_thread;
static int g_isp_thread_ok;
static void *g_sns_handle;
static ot_isp_sns_obj *g_sns_obj;

static ot_isp_3a_alg_lib g_ae_lib = {.id = VI_PIPE, .lib_name = "ot_ae_lib"};
static ot_isp_3a_alg_lib g_awb_lib = {.id = VI_PIPE, .lib_name = "ot_awb_lib"};

static void *isp_thread_fn(void *arg)
{
	(void)arg;
	/* Does not return until ss_mpi_isp_exit(). */
	(void)ss_mpi_isp_run(VI_PIPE);
	return NULL;
}

static int mipi_setup(void)
{
	combo_dev_attr_t attr;
	combo_dev_t dev = 0;
	sns_clk_source_t clk = 0;
	sns_rst_source_t rst = 0;
	lane_divide_mode_t hs = LANE_DIVIDE_MODE_0;
	int fd, i;

	memset(&attr, 0, sizeof(attr));
	attr.devno = dev;
	attr.input_mode = INPUT_MODE_MIPI;
	attr.data_rate = MIPI_DATA_RATE_X1;
	attr.img_rect.width = kSensor.width;
	attr.img_rect.height = kSensor.height;
	attr.mipi_attr.input_data_type = (kSensor.raw_bit == 10) ?
		DATA_TYPE_RAW_10BIT : DATA_TYPE_RAW_12BIT;
	attr.mipi_attr.wdr_mode = OT_MIPI_WDR_MODE_NONE;
	for (i = 0; i < MIPI_LANE_NUM; i++)
		attr.mipi_attr.lane_id[i] = (i < kSensor.lanes) ? (short)i : (short)-1;

	fd = open(MIPI_DEV_NODE, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "ERROR: venc: open %s: %s\n", MIPI_DEV_NODE,
			strerror(errno));
		return -1;
	}

#define mipi_ioctl(req, arg) do { \
		if (ioctl(fd, (req), (arg)) < 0) { \
			fprintf(stderr, "ERROR: venc: mipi %s: %s\n", #req, \
				strerror(errno)); \
			close(fd); \
			return -1; \
		} \
	} while (0)

	mipi_ioctl(OT_MIPI_SET_HS_MODE, &hs);
	mipi_ioctl(OT_MIPI_DISABLE_MIPI_CLOCK, &dev);
	mipi_ioctl(OT_MIPI_RESET_MIPI, &dev);
	mipi_ioctl(OT_MIPI_DISABLE_SENSOR_CLOCK, &clk);
	mipi_ioctl(OT_MIPI_RESET_SENSOR, &rst);
	mipi_ioctl(OT_MIPI_SET_DEV_ATTR, &attr);
	mipi_ioctl(OT_MIPI_ENABLE_MIPI_CLOCK, &dev);
	mipi_ioctl(OT_MIPI_UNRESET_MIPI, &dev);
	mipi_ioctl(OT_MIPI_ENABLE_SENSOR_CLOCK, &clk);
	mipi_ioctl(OT_MIPI_UNRESET_SENSOR, &rst);
#undef mipi_ioctl

	close(fd);
	return 0;
}

static int sys_setup(void)
{
	ot_vb_cfg vb;
	ot_vi_vpss_mode mode;
	unsigned int i;

	memset(&vb, 0, sizeof(vb));
	vb.max_pool_cnt = 1;
	/* One pool at the capture size. Online VI needs no RAW pool, and a
	 * single size lets a later restart pick any output size without
	 * reallocating VB, which cannot change while the graph runs. */
	vb.common_pool[0].blk_size =
		(td_u64)kSensor.width * kSensor.height * 3 / 2 + 0x4000;
	vb.common_pool[0].blk_cnt = YUV_POOL_BLOCKS;

	/* A SIGKILLed run leaves the ISP inited and the next mem_init fails
	 * with "already inited"; clearing here avoids needing a reboot. Errors
	 * are the expected answer when nothing was left behind. */
	(void)ss_mpi_isp_exit(VI_PIPE);
	(void)ss_mpi_sys_exit();
	(void)ss_mpi_vb_exit();

	memset(&mode, 0, sizeof(mode));
	for (i = 0; i < OT_VI_MAX_PIPE_NUM; i++)
		mode.mode[i] = OT_VI_OFFLINE_VPSS_OFFLINE;
	mode.mode[VI_PIPE] = OT_VI_ONLINE_VPSS_OFFLINE;

	CV610_CHECK(ss_mpi_vb_set_cfg(&vb));
	CV610_CHECK(ss_mpi_vb_init());
	/* Online mode can only be selected after a complete SYS init. */
	CV610_CHECK(ss_mpi_sys_init());
	CV610_CHECK(ss_mpi_sys_set_vi_vpss_mode(&mode));
	CV610_CHECK(ss_mpi_sys_set_vi_aiisp_mode(VI_PIPE, OT_VI_AIISP_MODE_DEFAULT));
	return 0;
}

static int vi_setup(void)
{
	ot_vi_dev_attr dev_attr;
	ot_vi_pipe_attr pipe_attr;
	ot_vi_chn_attr chn_attr;

	/* A run that died leaves the device enabled and set_dev_attr then
	 * fails NOT_DISABLE. */
	(void)ss_mpi_vi_disable_chn(VI_PIPE, VI_CHN);
	(void)ss_mpi_vi_stop_pipe(VI_PIPE);
	(void)ss_mpi_vi_destroy_pipe(VI_PIPE);
	(void)ss_mpi_vi_unbind(VI_DEV, VI_PIPE);
	(void)ss_mpi_vi_disable_dev(VI_DEV);

	memset(&dev_attr, 0, sizeof(dev_attr));
	dev_attr.intf_mode = OT_VI_INTF_MODE_MIPI;
	dev_attr.work_mode = OT_VI_WORK_MODE_MULTIPLEX_1;
	dev_attr.component_mask[0] = kSensor.component_mask;
	dev_attr.scan_mode = OT_VI_SCAN_PROGRESSIVE;
	dev_attr.ad_chn_id[0] = -1;
	dev_attr.ad_chn_id[1] = -1;
	dev_attr.ad_chn_id[2] = -1;
	dev_attr.ad_chn_id[3] = -1;
	dev_attr.data_seq = OT_VI_DATA_SEQ_YUYV;
	dev_attr.data_type = OT_VI_DATA_TYPE_RAW;
	dev_attr.data_reverse = TD_FALSE;
	dev_attr.in_size.width = kSensor.width;
	dev_attr.in_size.height = kSensor.height;
	dev_attr.data_rate = OT_DATA_RATE_X1;
	CV610_CHECK(ss_mpi_vi_set_dev_attr(VI_DEV, &dev_attr));
	CV610_CHECK(ss_mpi_vi_enable_dev(VI_DEV));
	CV610_CHECK(ss_mpi_vi_bind(VI_DEV, VI_PIPE));

	memset(&pipe_attr, 0, sizeof(pipe_attr));
	pipe_attr.pipe_bypass_mode = OT_VI_PIPE_BYPASS_NONE;
	pipe_attr.isp_bypass = TD_FALSE;
	pipe_attr.size.width = kSensor.width;
	pipe_attr.size.height = kSensor.height;
	pipe_attr.pixel_format = (kSensor.raw_bit == 10) ?
		OT_PIXEL_FORMAT_RGB_BAYER_10BPP : OT_PIXEL_FORMAT_RGB_BAYER_12BPP;
	pipe_attr.compress_mode = OT_COMPRESS_MODE_NONE;
	pipe_attr.frame_rate_ctrl.src_frame_rate = OT_VI_INVALID_FRAME_RATE;
	pipe_attr.frame_rate_ctrl.dst_frame_rate = OT_VI_INVALID_FRAME_RATE;
	CV610_CHECK(ss_mpi_vi_create_pipe(VI_PIPE, &pipe_attr));
	/* The ISP's mem_init queries the pipe size from VI. */
	CV610_CHECK(ss_mpi_vi_start_pipe(VI_PIPE));

	memset(&chn_attr, 0, sizeof(chn_attr));
	chn_attr.size.width = kSensor.width;
	chn_attr.size.height = kSensor.height;
	/* VI on this SoC accepts only YVU semiplanar; set_chn_attr rejects NV12. */
	chn_attr.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
	chn_attr.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
	chn_attr.video_format = OT_VIDEO_FORMAT_LINEAR;
	chn_attr.compress_mode = OT_COMPRESS_MODE_NONE;
	chn_attr.frame_rate_ctrl.src_frame_rate = OT_VI_INVALID_FRAME_RATE;
	chn_attr.frame_rate_ctrl.dst_frame_rate = OT_VI_INVALID_FRAME_RATE;
	/* Before isp_mem_init: the VI export it uses to learn the pipe's
	 * dynamic range reads physical channel 0's attributes. */
	CV610_CHECK(ss_mpi_vi_set_chn_attr(VI_PIPE, VI_CHN, &chn_attr));
	CV610_CHECK(ss_mpi_vi_enable_chn(VI_PIPE, VI_CHN));
	return 0;
}

static int sensor_setup(void)
{
	ot_isp_sns_commbus bus;

	g_sns_handle = dlopen(kSensor.lib_path, RTLD_NOW | RTLD_GLOBAL);
	if (!g_sns_handle) {
		fprintf(stderr, "ERROR: venc: dlopen %s: %s\n", kSensor.lib_path,
			dlerror());
		return -1;
	}
	g_sns_obj = (ot_isp_sns_obj *)dlsym(g_sns_handle, kSensor.obj_symbol);
	if (!g_sns_obj) {
		fprintf(stderr, "ERROR: venc: dlsym %s: %s\n", kSensor.obj_symbol,
			dlerror());
		return -1;
	}
	if (g_sns_obj->pfn_set_bus_info) {
		memset(&bus, 0, sizeof(bus));
		bus.i2c_dev = (td_s8)kSensor.i2c_bus;
		CV610_CHECK(g_sns_obj->pfn_set_bus_info(VI_PIPE, bus));
	}
	CV610_CHECK(ss_mpi_ae_register(VI_PIPE, &g_ae_lib));
	CV610_CHECK(ss_mpi_awb_register(VI_PIPE, &g_awb_lib));
	CV610_CHECK(g_sns_obj->pfn_register_callback(VI_PIPE, &g_ae_lib, &g_awb_lib));
	return 0;
}

static int isp_setup(void)
{
	ot_isp_pub_attr pub;
	ot_isp_bind_attr bind;

	memset(&pub, 0, sizeof(pub));
	pub.wnd_rect.width = kSensor.width;
	pub.wnd_rect.height = kSensor.height;
	pub.sns_size.width = kSensor.width;
	pub.sns_size.height = kSensor.height;
	pub.frame_rate = (float)kSensor.fps;
	pub.bayer_format = (ot_isp_bayer_format)kSensor.bayer;
	pub.wdr_mode = OT_WDR_MODE_NONE;
	pub.sns_mode = 0;

	/* Without the bind, isp_mem_init cannot resolve the sensor and fails
	 * NOT_CFG. */
	memset(&bind, 0, sizeof(bind));
	bind.sns_id = kSensor.sns_id;
	bind.ae_lib = g_ae_lib;
	bind.awb_lib = g_awb_lib;
	CV610_CHECK(ss_mpi_isp_set_bind_attr(VI_PIPE, &bind));
	CV610_CHECK(ss_mpi_isp_mem_init(VI_PIPE));
	CV610_CHECK(ss_mpi_isp_set_pub_attr(VI_PIPE, &pub));
	CV610_CHECK(ss_mpi_isp_init(VI_PIPE));
	return 0;
}

/* Written unconditionally, the disable included: the sensor holds whatever it
 * was last given. Must follow isp_setup, because the plugin opens its i2c fd
 * inside ss_mpi_isp_init and a register write before that does nothing. The
 * call returns void at every layer, so nothing here can report success. */
static void apply_sensor_orientation(int mirror, int flip)
{
	ot_isp_sns_mirrorflip_type type = ISP_SNS_NORMAL;

	if (!g_sns_obj || !g_sns_obj->pfn_mirror_flip) {
		if (mirror || flip)
			fprintf(stderr, "WARNING: venc: sensor plugin has no "
				"mirror/flip entry; orientation not applied\n");
		return;
	}
	if (mirror && flip)
		type = ISP_SNS_MIRROR_FLIP;
	else if (mirror)
		type = ISP_SNS_MIRROR;
	else if (flip)
		type = ISP_SNS_FLIP;
	g_sns_obj->pfn_mirror_flip(VI_PIPE, type);
}

/* The VENC VUI advertises full-range BT.709; the ISP CSC has to agree. */
static int configure_output_color(void)
{
	ot_isp_csc_attr csc;

	memset(&csc, 0, sizeof(csc));
	CV610_CHECK(ss_mpi_isp_get_csc_attr(VI_PIPE, &csc));
	csc.enable = TD_TRUE;
	csc.satu = 60;
	csc.contr = 53;
	CV610_CHECK(ss_mpi_isp_set_csc_attr(VI_PIPE, &csc));
	return 0;
}

/* The centre crop of the capture that has the output's aspect, so a 16:9
 * encode of this 4:3 sensor is framed instead of squashed. */
static ot_rect aspect_crop(uint32_t out_w, uint32_t out_h)
{
	const uint64_t sensor_ar = (uint64_t)kSensor.width * out_h;
	const uint64_t image_ar = (uint64_t)out_w * kSensor.height;
	ot_rect r;

	r.x = 0;
	r.y = 0;
	r.width = kSensor.width;
	r.height = kSensor.height;
	if (sensor_ar > image_ar) {
		r.width = (kSensor.height * out_w / out_h) & ~1u;
		r.x = (td_s32)(((kSensor.width - r.width) / 2) & ~1u);
	} else if (sensor_ar < image_ar) {
		r.height = (kSensor.width * out_h / out_w) & ~1u;
		r.y = (td_s32)(((kSensor.height - r.height) / 2) & ~1u);
	}
	return r;
}

/* The group's crop and the channel's size, with the group stopped. Written
 * unconditionally, the disable of the crop included, so no earlier geometry
 * can survive. */
static int vpss_apply_output(const Cv610PipelineConfig *c)
{
	const ot_rect rect = aspect_crop(c->out_width, c->out_height);
	ot_vpss_crop_info crop;
	ot_vpss_chn_attr chn;

	memset(&crop, 0, sizeof(crop));
	crop.enable = (rect.width != kSensor.width ||
		rect.height != kSensor.height) ? TD_TRUE : TD_FALSE;
	crop.crop_mode = OT_COORD_ABS;
	crop.crop_rect = rect;
	CV610_CHECK(ss_mpi_vpss_set_grp_crop(VPSS_GRP, &crop));

	memset(&chn, 0, sizeof(chn));
	chn.width = c->out_width;
	chn.height = c->out_height;
	/* One pixel format through the chain, whether or not VPSS scaled. */
	chn.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
	chn.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
	chn.video_format = OT_VIDEO_FORMAT_LINEAR;
	chn.compress_mode = OT_COMPRESS_MODE_NONE;
	/* USER, since AUTO makes the channel follow the input size and rejects
	 * an explicit geometry. The -1 rates are required: 0/0 is rejected. */
	chn.chn_mode = OT_VPSS_CHN_MODE_USER;
	chn.frame_rate.src_frame_rate = -1;
	chn.frame_rate.dst_frame_rate = -1;
	CV610_CHECK(ss_mpi_vpss_set_chn_attr(VPSS_GRP, VPSS_CHN, &chn));
	return 0;
}

static int vpss_setup(const Cv610PipelineConfig *c)
{
	ot_vpss_grp_attr grp;
	ot_mpp_chn src = {OT_ID_VI, VI_PIPE, VI_CHN};
	ot_mpp_chn dst = {OT_ID_VPSS, VPSS_GRP, 0};

	memset(&grp, 0, sizeof(grp));
	grp.max_width = kSensor.width;
	grp.max_height = kSensor.height;
	grp.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
	grp.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
	grp.dei_mode = OT_VPSS_DEI_MODE_OFF;
	grp.frame_rate.src_frame_rate = -1;
	grp.frame_rate.dst_frame_rate = -1;
	CV610_CHECK(ss_mpi_vpss_create_grp(VPSS_GRP, &grp));
	if (vpss_apply_output(c) != 0)
		return -1;
	CV610_CHECK(ss_mpi_vpss_enable_chn(VPSS_GRP, VPSS_CHN));
	CV610_CHECK(ss_mpi_vpss_start_grp(VPSS_GRP));
	CV610_CHECK(ss_mpi_sys_bind(&src, &dst));
	return 0;
}

static void vpss_teardown(void)
{
	ot_mpp_chn src = {OT_ID_VI, VI_PIPE, VI_CHN};
	ot_mpp_chn dst = {OT_ID_VPSS, VPSS_GRP, 0};

	/* Unconditional: a group an earlier step created outlives this process
	 * and the next create_grp would fail EXIST. Errors for objects that
	 * were never made are expected. */
	(void)ss_mpi_sys_unbind(&src, &dst);
	(void)ss_mpi_vpss_stop_grp(VPSS_GRP);
	(void)ss_mpi_vpss_disable_chn(VPSS_GRP, VPSS_CHN);
	(void)ss_mpi_vpss_destroy_grp(VPSS_GRP);
}

int cv610_pipeline_start(const Cv610PipelineConfig *c)
{
	if (!c || c->out_width == 0 || c->out_height == 0)
		return -1;
	if (sys_setup() != 0 || mipi_setup() != 0 || vi_setup() != 0 ||
	    sensor_setup() != 0 || isp_setup() != 0 ||
	    configure_output_color() != 0)
		return -1;
	apply_sensor_orientation(c->mirror, c->flip);
	if (vpss_setup(c) != 0)
		return -1;
	if (pthread_create(&g_isp_thread, NULL, isp_thread_fn, NULL) != 0) {
		fprintf(stderr, "ERROR: venc: ISP thread: %s\n", strerror(errno));
		return -1;
	}
	g_isp_thread_ok = 1;
	return 0;
}

int cv610_pipeline_reconfigure(const Cv610PipelineConfig *c)
{
	if (!c || c->out_width == 0 || c->out_height == 0)
		return -1;
	/* Best effort: after a failed reconfigure these are already stopped, and
	 * the retry the restart contract allows has to get past them. */
	(void)ss_mpi_vpss_stop_grp(VPSS_GRP);
	(void)ss_mpi_vpss_disable_chn(VPSS_GRP, VPSS_CHN);
	if (vpss_apply_output(c) != 0)
		return -1;
	CV610_CHECK(ss_mpi_vpss_enable_chn(VPSS_GRP, VPSS_CHN));
	CV610_CHECK(ss_mpi_vpss_start_grp(VPSS_GRP));
	apply_sensor_orientation(c->mirror, c->flip);
	return 0;
}

void cv610_pipeline_stop(void)
{
	/* Consumers first, so nothing reads from a stage being dismantled. */
	vpss_teardown();
	(void)ss_mpi_isp_exit(VI_PIPE);
	if (g_isp_thread_ok) {
		pthread_join(g_isp_thread, NULL);
		g_isp_thread_ok = 0;
	}
	if (g_sns_obj && g_sns_obj->pfn_un_register_callback)
		(void)g_sns_obj->pfn_un_register_callback(VI_PIPE, &g_ae_lib,
			&g_awb_lib);
	(void)ss_mpi_awb_unregister(VI_PIPE, &g_awb_lib);
	(void)ss_mpi_ae_unregister(VI_PIPE, &g_ae_lib);
	(void)ss_mpi_vi_disable_chn(VI_PIPE, VI_CHN);
	(void)ss_mpi_vi_stop_pipe(VI_PIPE);
	(void)ss_mpi_vi_destroy_pipe(VI_PIPE);
	(void)ss_mpi_vi_unbind(VI_DEV, VI_PIPE);
	(void)ss_mpi_vi_disable_dev(VI_DEV);
	if (g_sns_handle) {
		dlclose(g_sns_handle);
		g_sns_handle = NULL;
		g_sns_obj = NULL;
	}
	(void)ss_mpi_sys_exit();
	(void)ss_mpi_vb_exit();
}
