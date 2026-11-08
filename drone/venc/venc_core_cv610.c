/* The venc_core.h facade on Hi3516CV610: H.265 CBR on one VENC channel bound
 * to the capture graph of cv610_pipeline.c, access units published into the
 * shared frame ring.
 *
 * Channel setup derived from waybeam_venc src/cv610_runtime.c.
 * Copyright (c) 2023 OpenIPC, MIT License.
 *
 * A restart rebuilds only the VENC channel and the VPSS output. The sensor,
 * VI and ISP keep running, so the "session" of the facade's restart contract
 * is the capture graph. */
#include "venc_core.h"

#include "camera_modes.h"
#include "cv610_pipeline.h"
#include "idr_rate_limit.h"
#include "venc_frame_ring.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>

#include "ot_common_venc.h"
#include "ot_common_video.h"
#include "ss_mpi_sys.h"
#include "ss_mpi_sys_bind.h"
#include "ss_mpi_venc.h"

#define VENC_CHN 0
#define ENCODER_THREAD_PRIO 50
/* Same geometry as the Star6E ring: frame_source reads either. */
#define RING_SLOTS 8
#define RING_SLOT_BYTES (384 * 1024)

static VencCallbacks g_cb;
static venc_frame_ring_t *g_ring;
static pthread_t g_thread;
static int g_thread_started;
static int g_thread_stop;
static int g_session;           /* capture graph up */
static int g_started;           /* verbs may touch the channel */
static int g_fail_restarts_read;
static int g_fail_restarts;
static int g_cur_bitrate_kbps = -1;
static uint32_t g_frames_encoded;
static double g_gop_s;          /* the rate verb re-derives the GOP from it */
/* Serialises the read-modify-write verbs: each is a get_chn_attr/set pair. */
static pthread_mutex_t g_verb_lock = PTHREAD_MUTEX_INITIALIZER;

static int running(void)
{
	return __atomic_load_n(&g_started, __ATOMIC_ACQUIRE);
}

static Cv610PipelineConfig pipeline_cfg(const VencCfg *cfg)
{
	Cv610PipelineConfig p;

	memset(&p, 0, sizeof(p));
	p.out_width = cfg->width;
	p.out_height = cfg->height;
	p.mirror = cfg->mirror ? 1 : 0;
	p.flip = cfg->flip ? 1 : 0;
	return p;
}

/* Rate the channel encodes at: the sensor rate is the ceiling, since VENC can
 * only drop frames. */
static uint32_t clamp_fps(uint32_t fps)
{
	const uint32_t max = cv610_pipeline_sensor_fps();

	return (fps == 0 || fps > max) ? max : fps;
}

static int channel_create(const VencCfg *cfg)
{
	const uint32_t fps = clamp_fps(cfg->fps);
	ot_venc_chn_attr attr;
	ot_venc_h265_vui vui;
	ot_venc_start_param start;
	ot_mpp_chn src = {OT_ID_VPSS, CV610_VPSS_GRP, CV610_VPSS_CHN};
	ot_mpp_chn dst = {OT_ID_VENC, 0, VENC_CHN};
	td_s32 ret;

	memset(&attr, 0, sizeof(attr));
	attr.venc_attr.type = OT_PT_H265;
	attr.venc_attr.max_pic_width = cfg->width;
	attr.venc_attr.max_pic_height = cfg->height;
	attr.venc_attr.buf_size = ((uint32_t)cfg->width * cfg->height * 3 / 4 + 63) & ~63u;
	attr.venc_attr.is_by_frame = TD_TRUE;
	attr.venc_attr.pic_width = cfg->width;
	attr.venc_attr.pic_height = cfg->height;
	attr.venc_attr.h265_attr.rcn_ref_share_buf_en = TD_TRUE;
	attr.venc_attr.h265_attr.frame_buf_ratio = 75;
	attr.rc_attr.rc_mode = OT_VENC_RC_MODE_H265_CBR;
	g_gop_s = cfg->gop_s;
	attr.rc_attr.h265_cbr.gop = venc_cfg_gop_frames(cfg->gop_s, fps);
	attr.rc_attr.h265_cbr.stats_time = 1;
	attr.rc_attr.h265_cbr.src_frame_rate = cv610_pipeline_sensor_fps();
	attr.rc_attr.h265_cbr.dst_frame_rate = fps;
	attr.rc_attr.h265_cbr.bit_rate = VENC_BOOT_BITRATE_KBPS;
	attr.gop_attr.gop_mode = OT_VENC_GOP_MODE_NORMAL_P;

	ret = ss_mpi_venc_create_chn(VENC_CHN, &attr);
	if (ret != TD_SUCCESS) {
		fprintf(stderr, "ERROR: venc: create_chn = 0x%x\n", (unsigned)ret);
		return -1;
	}

	memset(&vui, 0, sizeof(vui));
	ret = ss_mpi_venc_get_h265_vui(VENC_CHN, &vui);
	if (ret == TD_SUCCESS) {
		vui.vui_time_info.timing_info_present_flag = 1;
		vui.vui_time_info.num_units_in_tick = 1000;
		vui.vui_time_info.time_scale = fps * 1000;
		vui.vui_video_signal.video_signal_type_present_flag = 1;
		vui.vui_video_signal.video_format = 5;
		vui.vui_video_signal.video_full_range_flag = 1;
		vui.vui_video_signal.colour_description_present_flag = 1;
		vui.vui_video_signal.colour_primaries = 1;
		vui.vui_video_signal.transfer_characteristics = 1;
		vui.vui_video_signal.matrix_coefficients = 1;
		ret = ss_mpi_venc_set_h265_vui(VENC_CHN, &vui);
	}
	if (ret != TD_SUCCESS) {
		fprintf(stderr, "ERROR: venc: h265 vui = 0x%x\n", (unsigned)ret);
		return -1;
	}

	memset(&start, 0, sizeof(start));
	start.recv_pic_num = -1;
	ret = ss_mpi_venc_start_chn(VENC_CHN, &start);
	if (ret != TD_SUCCESS) {
		fprintf(stderr, "ERROR: venc: start_chn = 0x%x\n", (unsigned)ret);
		return -1;
	}
	ret = ss_mpi_sys_bind(&src, &dst);
	if (ret != TD_SUCCESS) {
		fprintf(stderr, "ERROR: venc: bind = 0x%x\n", (unsigned)ret);
		return -1;
	}
	return 0;
}

static void channel_destroy(void)
{
	ot_mpp_chn src = {OT_ID_VPSS, CV610_VPSS_GRP, CV610_VPSS_CHN};
	ot_mpp_chn dst = {OT_ID_VENC, 0, VENC_CHN};

	/* Each is a harmless error on an object a partial create never made. */
	(void)ss_mpi_sys_unbind(&src, &dst);
	(void)ss_mpi_venc_stop_chn(VENC_CHN);
	(void)ss_mpi_venc_destroy_chn(VENC_CHN);
}

static void publish_frame(const ot_venc_stream *stream)
{
	VencFrameMeta meta;
	td_u64 cur = 0;
	int is_idr = 0;
	td_u32 i;

	for (i = 0; i < stream->pack_cnt; ++i)
		if (stream->pack[i].data_type.h265_type == OT_VENC_H265_NALU_IDR_SLICE)
			is_idr = 1;

	memset(&meta, 0, sizeof(meta));
	meta.pts = (uint32_t)stream->pack[0].pts;
	meta.codec = VENC_FRAME_CODEC_H265;
	meta.flags = is_idr ? VENC_FRAME_FLAG_IDR : 0;
	if (ss_mpi_sys_get_cur_pts(&cur) == TD_SUCCESS && cur >= stream->pack[0].pts) {
		const uint64_t d = cur - stream->pack[0].pts;

		/* A huge delta means the SDK re-based pts: report unknown. */
		if (d < 1000000u)
			meta.enc_us = (uint16_t)(d > 65535 ? 65535 : d);
	}
	__atomic_add_fetch(&g_frames_encoded, 1, __ATOMIC_RELAXED);

	if (venc_frame_ring_begin_write(g_ring, &meta) != 0)
		goto dropped;
	for (i = 0; i < stream->pack_cnt; ++i) {
		const ot_venc_pack *p = &stream->pack[i];

		if (p->len <= p->offset)
			continue;
		if (venc_frame_ring_append(g_ring, p->addr + p->offset,
		    p->len - p->offset) != 0) {
			venc_frame_ring_abort_write(g_ring);
			goto dropped;
		}
	}
	venc_frame_ring_commit_write(g_ring);
	return;

dropped:
	/* Every frame here is a reference for the next one. */
	if (g_cb.on_chain_break)
		g_cb.on_chain_break(g_cb.user);
}

static void encoder_fault(const char *what)
{
	fprintf(stderr, "ERROR: venc encoder loop died: %s\n", what);
	if (g_cb.on_fault)
		g_cb.on_fault(g_cb.user, what);
}

static void *encoder_thread(void *arg)
{
	struct sched_param sp;
	ot_venc_pack *packs = NULL;
	td_u32 packs_cap = 0;
	int fd = ss_mpi_venc_get_fd(VENC_CHN);

	(void)arg;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = ENCODER_THREAD_PRIO;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
		fprintf(stderr, "WARNING: venc: encoder thread stays SCHED_OTHER\n");

	if (fd < 0) {
		encoder_fault("ss_mpi_venc_get_fd");
		return NULL;
	}
	while (!__atomic_load_n(&g_thread_stop, __ATOMIC_ACQUIRE)) {
		ot_venc_chn_status status;
		ot_venc_stream stream;
		struct timeval timeout = {0, 200000};
		fd_set readfds;
		int ready;

		FD_ZERO(&readfds);
		FD_SET(fd, &readfds);
		ready = select(fd + 1, &readfds, NULL, NULL, &timeout);
		if (ready < 0 && errno == EINTR)
			continue;
		if (ready < 0) {
			encoder_fault("select on the VENC fd");
			break;
		}
		if (ready == 0)
			continue;

		memset(&status, 0, sizeof(status));
		if (ss_mpi_venc_query_status(VENC_CHN, &status) != TD_SUCCESS ||
		    status.cur_packs == 0)
			continue;
		if (status.cur_packs > packs_cap) {
			ot_venc_pack *grown = realloc(packs,
				status.cur_packs * sizeof(*packs));

			if (!grown) {
				encoder_fault("out of memory");
				break;
			}
			packs = grown;
			packs_cap = status.cur_packs;
		}
		memset(&stream, 0, sizeof(stream));
		memset(packs, 0, status.cur_packs * sizeof(*packs));
		stream.pack = packs;
		stream.pack_cnt = status.cur_packs;
		if (ss_mpi_venc_get_stream(VENC_CHN, &stream, 1000) != TD_SUCCESS)
			continue;
		if (stream.pack_cnt > 0)
			publish_frame(&stream);
		if (ss_mpi_venc_release_stream(VENC_CHN, &stream) != TD_SUCCESS) {
			encoder_fault("ss_mpi_venc_release_stream");
			break;
		}
	}
	free(packs);
	return NULL;
}

static int thread_start(void)
{
	__atomic_store_n(&g_thread_stop, 0, __ATOMIC_RELEASE);
	if (pthread_create(&g_thread, NULL, encoder_thread, NULL) != 0)
		return -1;
	g_thread_started = 1;
	pthread_setname_np(g_thread, "venc-enc");
	return 0;
}

static void thread_stop(void)
{
	__atomic_store_n(&g_thread_stop, 1, __ATOMIC_RELEASE);
	if (g_thread_started) {
		pthread_join(g_thread, NULL);
		g_thread_started = 0;
	}
}

static void warn_ignored(const VencCfg *cfg)
{
	if (cfg->intra_refresh_frames)
		fprintf(stderr, "WARNING: venc: intra refresh is not implemented "
			"on this SoC; the stream uses IDRs\n");
	if (cfg->slices > 1)
		fprintf(stderr, "WARNING: venc: slices are not implemented on "
			"this SoC; one slice per picture\n");
	if (cfg->roi_enabled)
		fprintf(stderr, "WARNING: venc: ROI is not implemented on this SoC\n");
}

int venc_core_preload(void)
{
	return 0;  /* the SDK is linked, there is nothing to dlopen */
}

int venc_core_start(const VencCfg *cfg, const VencCallbacks *cb)
{
	Cv610PipelineConfig p;
	const CameraMode mode = {(uint16_t)cv610_pipeline_sensor_width(),
		(uint16_t)cv610_pipeline_sensor_height(),
		(uint16_t)cv610_pipeline_sensor_fps()};

	if (!cfg)
		return -1;
	if (g_session) {
		fprintf(stderr, "ERROR: venc_core_start: already started "
			"(venc_core_stop first)\n");
		return -1;
	}
	if (!g_fail_restarts_read) {
		const char *env = getenv("MABUR_VENC_FAIL_RESTARTS");

		g_fail_restarts_read = 1;
		g_fail_restarts = env ? atoi(env) : 0;
	}
	memset(&g_cb, 0, sizeof(g_cb));
	if (cb)
		g_cb = *cb;
	g_cur_bitrate_kbps = -1;
	warn_ignored(cfg);

	g_ring = venc_frame_ring_create(VENC_RING_NAME, RING_SLOTS, RING_SLOT_BYTES);
	if (!g_ring) {
		fprintf(stderr, "ERROR: venc_frame_ring_create(%s) failed\n",
			VENC_RING_NAME);
		return -1;
	}
	p = pipeline_cfg(cfg);
	g_session = 1;
	if (cv610_pipeline_start(&p) != 0 || channel_create(cfg) != 0 ||
	    thread_start() != 0) {
		venc_core_stop();
		return -1;
	}
	camera_modes_publish(&mode, 1);
	__atomic_store_n(&g_started, 1, __ATOMIC_RELEASE);
	printf("> venc_core: started (%ux%u@%u, ring %s)\n",
		(unsigned)cfg->width, (unsigned)cfg->height,
		(unsigned)clamp_fps(cfg->fps), VENC_RING_NAME);
	fflush(stdout);
	return 0;
}

void venc_core_stop(void)
{
	if (!g_session)
		return;
	__atomic_store_n(&g_started, 0, __ATOMIC_RELEASE);
	pthread_mutex_lock(&g_verb_lock);
	thread_stop();
	channel_destroy();
	cv610_pipeline_stop();
	if (g_ring) {
		venc_frame_ring_destroy(g_ring);
		g_ring = NULL;
	}
	g_session = 0;
	g_cur_bitrate_kbps = -1;
	pthread_mutex_unlock(&g_verb_lock);
}

int venc_core_restart(const VencCfg *cfg)
{
	Cv610PipelineConfig p;

	if (!cfg || !g_session)
		return -1;
	pthread_mutex_lock(&g_verb_lock);
	__atomic_store_n(&g_started, 0, __ATOMIC_RELEASE);
	thread_stop();
	channel_destroy();
	g_cur_bitrate_kbps = -1;
	warn_ignored(cfg);

	if (g_fail_restarts > 0) {
		--g_fail_restarts;
		fprintf(stderr, "ERROR: venc_core_restart: injected failure\n");
		goto failed;
	}
	p = pipeline_cfg(cfg);
	if (cv610_pipeline_reconfigure(&p) != 0 || channel_create(cfg) != 0)
		goto failed;
	if (thread_start() != 0) {
		channel_destroy();
		goto failed;
	}
	__atomic_store_n(&g_started, 1, __ATOMIC_RELEASE);
	pthread_mutex_unlock(&g_verb_lock);
	return 0;

failed:
	fprintf(stderr, "ERROR: venc_core_restart: pipeline restart failed\n");
	pthread_mutex_unlock(&g_verb_lock);
	return -1;
}

int venc_set_bitrate_kbps(int kbps)
{
	ot_venc_chn_attr attr;
	int programmed = kbps;
	int rc = -1;

	if (!running() || kbps <= 0)
		return -1;
	if (programmed < VENC_BITRATE_MIN_KBPS)
		programmed = VENC_BITRATE_MIN_KBPS;
	if (programmed > VENC_BITRATE_MAX_KBPS)
		programmed = VENC_BITRATE_MAX_KBPS;

	pthread_mutex_lock(&g_verb_lock);
	memset(&attr, 0, sizeof(attr));
	if (ss_mpi_venc_get_chn_attr(VENC_CHN, &attr) == TD_SUCCESS &&
	    attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H265_CBR) {
		attr.rc_attr.h265_cbr.bit_rate = (td_u32)programmed;
		if (ss_mpi_venc_set_chn_attr(VENC_CHN, &attr) == TD_SUCCESS) {
			__atomic_store_n(&g_cur_bitrate_kbps, kbps, __ATOMIC_RELAXED);
			rc = 0;
		}
	}
	pthread_mutex_unlock(&g_verb_lock);
	return rc;
}

int venc_set_fps(int fps)
{
	ot_venc_chn_attr attr;
	int rc = -1;

	if (!running() || fps <= 0)
		return -1;
	pthread_mutex_lock(&g_verb_lock);
	memset(&attr, 0, sizeof(attr));
	if (ss_mpi_venc_get_chn_attr(VENC_CHN, &attr) == TD_SUCCESS &&
	    attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H265_CBR) {
		/* The GOP is a frame count: it has to follow the rate to keep
		 * the keyframe interval in seconds. */
		attr.rc_attr.h265_cbr.dst_frame_rate = clamp_fps((uint32_t)fps);
		attr.rc_attr.h265_cbr.gop =
			venc_cfg_gop_frames(g_gop_s, clamp_fps((uint32_t)fps));
		if (ss_mpi_venc_set_chn_attr(VENC_CHN, &attr) == TD_SUCCESS)
			rc = 0;
	}
	pthread_mutex_unlock(&g_verb_lock);
	return rc;
}

int venc_request_idr(void)
{
	int rc = 0;

	if (!running())
		return -1;
	pthread_mutex_lock(&g_verb_lock);
	if (idr_rate_limit_allow(VENC_CHN) &&
	    ss_mpi_venc_request_idr(VENC_CHN, TD_TRUE) != TD_SUCCESS)
		rc = -1;
	pthread_mutex_unlock(&g_verb_lock);
	return rc;
}

/* Accepted and ignored, once logged: RcAgent counts a refusal as a failed
 * apply and then re-sends every verb on each tick for as long as the bitrate
 * stays below roi_threshold_kbps. */
int venc_set_roi_qp(int qp)
{
	static int warned;

	(void)qp;
	if (!__atomic_exchange_n(&warned, 1, __ATOMIC_RELAXED))
		fprintf(stderr, "venc: ROI QP is not implemented on this SoC, ignored\n");
	return 0;
}

/* Not implemented on this SoC. Each caller logs the failure and carries on. */
int venc_set_qp_delta(int qp_delta) { (void)qp_delta; return -1; }
int venc_set_max_ipprop(int prop) { (void)prop; return -1; }
int venc_set_superframe_p_pct(int pct) { (void)pct; return -1; }
int venc_set_min_iqp(int qp) { (void)qp; return -1; }
int venc_set_max_iqp(int qp) { (void)qp; return -1; }
int venc_set_sensor_offset(int mfps) { (void)mfps; return -1; }

uint64_t venc_cur_pts_us(void)
{
	td_u64 cur = 0;

	if (!running() || ss_mpi_sys_get_cur_pts(&cur) != TD_SUCCESS)
		return 0;
	return cur;
}

void venc_get_stats(VencStats *out)
{
	venc_frame_ring_fill_t fill;

	if (!out)
		return;
	memset(out, 0, sizeof(*out));
	out->cur_bitrate_kbps = __atomic_load_n(&g_cur_bitrate_kbps, __ATOMIC_RELAXED);
	if (!running())
		return;
	out->frames_encoded = __atomic_load_n(&g_frames_encoded, __ATOMIC_RELAXED);
	if (venc_frame_ring_get_fill(g_ring, &fill) == 0) {
		out->full_drops = fill.full_drops;
		out->ring_fill_pct = fill.fill_pct;
	}
}

int venc_snapshot_jpeg(uint8_t **out, size_t *out_len, int quality)
{
	(void)quality;
	if (out)
		*out = NULL;
	if (out_len)
		*out_len = 0;
	return -1;
}

void venc_record_configure(const VencRecordConfig *cfg, VencRecordSink sink, void *user)
{
	(void)sink;
	(void)user;
	if (cfg && cfg->enabled)
		fprintf(stderr, "WARNING: venc: the onboard recorder is not "
			"implemented on this SoC\n");
}

int venc_record_start(void) { return -1; }
void venc_record_stop(void) {}
void venc_record_request_idr(void) {}
