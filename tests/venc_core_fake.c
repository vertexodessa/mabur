/* See venc_core_fake.h. */
#include "venc_core_fake.h"

#include "star6e_controls.h"
#include "star6e_mi.h"
#include "star6e_output.h"
#include "star6e_runtime.h"
#include "venc_jpeg.h"

#include <string.h>
#include <time.h>

VencCoreFake g_venc_fake;
star6e_sys_impl g_mi_sys;

static int g_stop;

void venc_core_fake_reset_counters(void)
{
	g_venc_fake.sys_init = 0;
	g_venc_fake.sys_init_on_live = 0;
	g_venc_fake.sys_exit = 0;
	g_venc_fake.restarts = 0;
	g_venc_fake.injected = 0;
	g_venc_fake.fail_restarts = 0;
}

int star6e_mi_init(void) { return 0; }

int star6e_runtime_prepare(void)
{
	__atomic_store_n(&g_stop, 0, __ATOMIC_RELEASE);
	return 0;
}

int star6e_runtime_init(Star6eRunnerContext *ctx)
{
	++g_venc_fake.sys_init;
	if (g_venc_fake.session_up)
		++g_venc_fake.sys_init_on_live;
	g_venc_fake.session_up = 1;
	ctx->system_initialized = 1;
	g_venc_fake.pipeline_up = 1;
	g_venc_fake.width = ctx->cfg.width;
	ctx->pipeline_started = 1;
	return 0;
}

void star6e_runtime_teardown(Star6eRunnerContext *ctx)
{
	if (ctx->pipeline_started) {
		g_venc_fake.pipeline_up = 0;
		ctx->pipeline_started = 0;
	}
	if (ctx->system_initialized) {
		++g_venc_fake.sys_exit;
		g_venc_fake.session_up = 0;
		ctx->system_initialized = 0;
	}
}

int star6e_runtime_pipeline_restart(Star6eRunnerContext *ctx, const VencCfg *cfg,
	int inject_fail)
{
	if (!ctx || !cfg || !ctx->system_initialized)
		return -1;
	++g_venc_fake.restarts;
	if (ctx->pipeline_started) {
		g_venc_fake.pipeline_up = 0;
		ctx->pipeline_started = 0;
	}
	ctx->cfg = *cfg;
	__atomic_store_n(&g_stop, 0, __ATOMIC_RELEASE);
	if (inject_fail) {
		++g_venc_fake.injected;
		return -1;
	}
	if (g_venc_fake.fail_restarts > 0) {
		--g_venc_fake.fail_restarts;
		return -1;
	}
	g_venc_fake.pipeline_up = 1;
	g_venc_fake.width = cfg->width;
	ctx->pipeline_started = 1;
	return 0;
}

void *star6e_runtime_encoder_loop(void *opaque)
{
	const struct timespec ms = {0, 1000000};

	(void)opaque;
	while (!__atomic_load_n(&g_stop, __ATOMIC_ACQUIRE))
		nanosleep(&ms, NULL);
	return NULL;
}

void star6e_runtime_request_stop(void)
{
	__atomic_store_n(&g_stop, 1, __ATOMIC_RELEASE);
}

uint32_t star6e_runtime_sensor_mhz(void) { return 0; }
void star6e_runtime_request_sensor_offset(int32_t mfps) { (void)mfps; }
int32_t star6e_runtime_sensor_offset(void) { return 0; }

int star6e_controls_apply_bitrate(uint32_t kbps) { (void)kbps; return 0; }
int star6e_controls_apply_fps(uint32_t fps) { (void)fps; return 0; }
int star6e_controls_apply_roi_qp(int qp) { (void)qp; return 0; }
int star6e_controls_apply_qp_delta(int delta) { (void)delta; return 0; }
int star6e_controls_set_max_ipprop(uint32_t prop) { (void)prop; return 0; }
int star6e_controls_set_iqp_bound(int is_min, uint32_t qp) { (void)is_min; (void)qp; return 0; }
int star6e_controls_set_superframe_p_pct(uint32_t pct) { (void)pct; return 0; }
int star6e_controls_request_idr(void) { return 0; }
int star6e_controls_superframe_pending(void) { return 0; }
void star6e_controls_superframe_service(void) {}
int star6e_controls_set_superframe_mode(int discard) { (void)discard; return 0; }

int star6e_output_frame_ring_fill(const Star6eOutput *output, venc_frame_ring_fill_t *out)
{
	(void)output;
	memset(out, 0, sizeof(*out));
	return -1;
}

int venc_jpeg_capture(uint8_t **out_buf, size_t *out_len, uint32_t timeout_ms)
{
	(void)out_buf; (void)out_len; (void)timeout_ms;
	return -1;
}

int venc_jpeg_set_quality(uint32_t q) { (void)q; return 0; }
