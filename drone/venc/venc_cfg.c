/* ported-from/derived-from waybeam_venc f956a52:src/venc_config.c — by now
 * only the absent-key defaults and two pure helpers survive; every cJSON
 * parse/save/load and live-apply grouping went with the venc fold-in, and
 * the resilience preset table went when its fields became config keys of
 * their own. */
#include "venc_cfg.h"

#include <string.h>

/* Absent-key defaults.  See venc_cfg.h.
 *
 * These are the values the shipped bundle carries, so a config that omits
 * a venc key gets the flown configuration rather than a zero.  sensor_bin
 * is deliberately left EMPTY: it is the one device-specific field (the ISP
 * calibration blob path), and the config loader treats an empty sensor_bin
 * as a boot failure.
 *
 * Kept in venc_cfg.c, next to the struct it fills, rather than as C++ member
 * initialisers in config.h, so the vendored C struct owns its own defaults
 * and any future non-mabur caller of venc_core_start() gets them too. */
void venc_cfg_defaults(VencCfg *cfg)
{
	if (!cfg)
		return;
	memset(cfg, 0, sizeof(*cfg));
	/* sensor_bin: intentionally left "" — required, no default. */
	cfg->width = 1920;
	cfg->height = 1080;
	cfg->fps = 60;
	cfg->binning = true;
	cfg->gop_s = 2.0;
	cfg->qp_delta = -4;
	cfg->max_ipprop = 0; /* leave the firmware default (unbounded) */
	cfg->min_iqp = 0;    /* leave the firmware I-QP floor */
	cfg->superframe_p_pct = 0; /* no P-frame ceiling */
	/* On even without the ceiling: a drone's kept config may carry
	 * superframe_p_pct but no floor key, and must still be safe. */
	cfg->superframe_p_floor_kb = 14;
	/* Error-resilience structure. These reproduce, at 1080p60, exactly
	 * what the deleted "rally" preset expanded to: its "fast" intra mode
	 * targeted a 150 ms sweep: 9 frames, ceil(34 CTU rows / 9) = 4 rows
	 * per P-frame at QP 36, over 1:1 SVC-T with prediction on. Being a
	 * sweep length, it re-derives its rows at any venc.size. */
	cfg->intra_refresh_frames = 9;
	cfg->intra_refresh_qp = 36;
	cfg->ref_base = 1;
	cfg->ref_enhance = 1;
	cfg->ref_pred = true;
	cfg->mirror = false;
	cfg->flip = false;
	cfg->slices = 1; /* split off */
	cfg->roi_enabled = true;
	cfg->roi_steps = 2;
	cfg->roi_center = 0.4;
	cfg->ae_fps = 15;
	cfg->awb_fps = 15;
	cfg->snapshot_quality = 80;
}

/* H.265 CTU is 32x32, so a 1080-line picture is 34 CTU rows. */
uint16_t venc_cfg_ctu_rows(uint16_t height)
{
	return (uint16_t)((height + 31u) / 32u);
}

uint16_t venc_cfg_intra_rows(uint16_t height, uint16_t frames)
{
	uint16_t total = venc_cfg_ctu_rows(height);

	if (frames == 0 || total == 0)
		return 0;
	return (uint16_t)((total + frames - 1u) / frames);
}

uint16_t venc_cfg_ctb64_rows(uint16_t height)
{
	return (uint16_t)((height + 63u) / 64u);
}

uint8_t venc_cfg_slice_rows(uint16_t height, uint8_t slices)
{
	uint16_t r = venc_cfg_ctb64_rows(height);
	uint16_t k;

	if (slices <= 1 || r == 0 || slices > r)
		return 0;
	k = (uint16_t)((r + slices - 1u) / slices);
	if ((uint16_t)((r + k - 1u) / k) != slices)
		return 0;
	return (uint8_t)k;
}

uint32_t venc_superframe_p_bytes(unsigned pct, unsigned kbps, unsigned fps,
				 unsigned width, unsigned height,
				 unsigned floor_kb)
{
	uint64_t bits_per_frame, bytes, floor_bytes;

	if (pct == 0 || fps == 0)
		return 0;
	/* Per-frame budget in bits = programmed bps / fps; programmed bps is
	 * kbps * 1024 (star6e_controls.c apply_bitrate). */
	bits_per_frame = ((uint64_t)kbps * 1024ull) / fps;
	bytes = bits_per_frame * pct / 100ull / 8ull;
	/* The floor scales with the picture: an ordinary P frame at the QP
	 * rail grows with the pixel count, the budget does not. */
	floor_bytes = (uint64_t)floor_kb * width * height / 1000ull;
	if (bytes < floor_bytes)
		bytes = floor_bytes;
	if (bytes > 0xFFFFFFFFull / 8ull)
		bytes = 0xFFFFFFFFull / 8ull;
	return (uint32_t)bytes;
}

void venc_sf_guard_init(VencSfGuard *g)
{
	memset(g, 0, sizeof(*g));
	g->lift_pct = 100;
}

void venc_sf_guard_request(VencSfGuard *g, uint64_t now_us)
{
	g->last_req_us = now_us;
}

int venc_sf_guard_frame(VencSfGuard *g, uint64_t now_us, int is_idr)
{
	uint32_t before = g->lift_pct;

	if (is_idr) {
		int asked = g->last_req_us != 0 &&
			now_us - g->last_req_us <= VENC_SF_REQ_US;
		int close = g->last_idr_us != 0 &&
			now_us - g->last_idr_us <= VENC_SF_STORM_US;

		g->last_idr_us = now_us;
		/* A request explains one keyframe, not every keyframe in its
		 * window: in a storm on a weak link the GS asks again and the
		 * ladder writes a new rate, and those must not hide the run. */
		if (asked)
			g->last_req_us = 0;
		if (close && !asked) {
			uint32_t up = g->lift_pct * 5u / 4u;

			g->storms++;
			g->lift_pct = up > VENC_SF_LIFT_MAX ? VENC_SF_LIFT_MAX : up;
			g->calm_from_us = now_us;
		}
	}
	if (g->lift_pct > 100 && now_us - g->calm_from_us >= VENC_SF_CALM_US) {
		uint32_t down = g->lift_pct * 4u / 5u;

		g->lift_pct = down < 100 ? 100 : down;
		g->calm_from_us = now_us;
	}
	return g->lift_pct != before;
}

uint32_t venc_sf_guard_apply(const VencSfGuard *g, uint32_t bytes)
{
	uint64_t b = (uint64_t)bytes * g->lift_pct / 100ull;

	if (b > 0xFFFFFFFFull / 8ull)
		b = 0xFFFFFFFFull / 8ull;
	return (uint32_t)b;
}

uint32_t venc_cfg_gop_frames(double gop_s, uint32_t fps)
{
	uint32_t gop_fps = fps ? fps : 30;
	uint32_t frames;

	if (gop_s <= 0.0)
		return VENC_GOP_NONE;
	frames = (uint32_t)(gop_s * gop_fps + 0.5);
	return frames < 1 ? 1 : frames;
}
