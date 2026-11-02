#include "camera_modes.h"

/* waybeam's driver, six-mode build (waybeam_venc b723e91, the module
 * gilankpam/openipc-builder ships): up to 90 fps the 2x2-binned full view;
 * above it the sensor cannot read the full height fast enough (a wide
 * binned line needs ~4.9 us and the full height ~2250 lines), so 91-100
 * and 101-120 use the widest binned crops that stay clean at those rates.
 * 2944x1656@60 and 2304x1296@100 are the native windows. */
const CameraMode camera_lineup_waybeam[6] = {
	{3840, 2160, 33}, {2944, 1656, 60}, {1920, 1080, 90},
	{2304, 1296, 100}, {1728, 972, 100}, {1728, 816, 120},
};

/* The earlier five-mode build: one native window, 2816x1584@60. */
const CameraMode camera_lineup_waybeam5[5] = {
	{3840, 2160, 33}, {2816, 1584, 60}, {1920, 1080, 90},
	{1728, 972, 100}, {1728, 816, 120},
};

/* OpenIPC's stock driver: one high-rate crop, narrower than waybeam's, and
 * one native window, 2560x1440@60. */
const CameraMode camera_lineup_stock[4] = {
	{3840, 2160, 30}, {2560, 1440, 60}, {1920, 1080, 90}, {1472, 816, 120},
};

static int is_4k(CameraMode m)
{
	return m.w >= 3840 || m.h >= 2160;
}

static int in_readout(CameraMode m, int binning)
{
	const int small = m.w <= 1920 && m.h <= 1080;
	return binning ? small : !small && !is_4k(m);
}

int camera_mode_pick(const CameraMode *offered, int n, int binning,
	uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h)
{
	int tier = -1;

	for (int i = 0; i < n; ++i) {
		const CameraMode m = offered[i];
		if (m.max_fps < venc_fps || !in_readout(m, binning))
			continue;
		if (tier < 0) {
			tier = i;
			continue;
		}
		const CameraMode b = offered[tier];
		const uint32_t area = (uint32_t)m.w * m.h, barea = (uint32_t)b.w * b.h;
		if (area > barea || (area == barea && m.max_fps < b.max_fps))
			tier = i;
	}
	if (rec_w <= 1920 && rec_h <= 1080)
		return tier;
	if (tier >= 0 && rec_w <= offered[tier].w && rec_h <= offered[tier].h)
		return tier;  /* VPE port 1 scales the recording down from it */

	int best = -1;
	for (int i = 0; i < n; ++i) {
		const CameraMode m = offered[i];
		if (m.max_fps < venc_fps || !is_4k(m) || m.w < rec_w || m.h < rec_h)
			continue;
		if (best < 0 || (uint32_t)m.w * m.h <
				(uint32_t)offered[best].w * offered[best].h)
			best = i;
	}
	return best;
}

int camera_mode_find(const CameraMode *list, int n, CameraMode want)
{
	for (int i = 0; i < n; ++i)
		if (list[i].w == want.w && list[i].h == want.h &&
		    list[i].max_fps == want.max_fps)
			return i;
	return -1;
}

int camera_mode_size_ok(CameraMode mode, uint32_t w, uint32_t h)
{
	if (!w || !h || !mode.w || !mode.h)
		return 0;
	const uint64_t a = (uint64_t)w * mode.h, b = (uint64_t)h * mode.w;
	const uint64_t diff = a > b ? a - b : b - a;
	if (diff * 50 > b)
		return 0;
	/* The keep-aspect window inside the mode (pipeline_common_compute_precrop). */
	uint32_t cw = mode.w, ch = mode.h;
	if (b > a)  /* the mode is wider than w:h */
		cw = (uint32_t)(((uint64_t)mode.h * w / h) & ~1u);
	else if (a > b)
		ch = (uint32_t)(((uint64_t)mode.w * h / w) & ~1u);
	return w <= cw && h <= ch;
}

static int fits_lineup(const CameraMode *l, int n, int binning, uint32_t fps,
	uint32_t rec_w, uint32_t rec_h, uint32_t w, uint32_t h, CameraMode *hint)
{
	const int i = camera_mode_pick(l, n, binning, fps, rec_w, rec_h);
	const CameraMode none = {0, 0, 0};
	if (hint)
		*hint = i < 0 ? none : l[i];
	return i >= 0 && camera_mode_size_ok(l[i], w, h);
}

int camera_mode_known_fit(int binning, uint32_t venc_fps, uint32_t rec_w,
	uint32_t rec_h, uint32_t w, uint32_t h, CameraMode *hint,
	CameraMode *hint_stock)
{
	const int a = fits_lineup(camera_lineup_waybeam, 6, binning, venc_fps,
		rec_w, rec_h, w, h, hint);
	const int b = fits_lineup(camera_lineup_waybeam5, 5, binning, venc_fps,
		rec_w, rec_h, w, h, 0);
	const int c = fits_lineup(camera_lineup_stock, 4, binning, venc_fps,
		rec_w, rec_h, w, h, hint_stock);
	return a || b || c;
}

int camera_mode_known_tier(int binning, uint32_t venc_fps, CameraMode *out)
{
	const struct { const CameraMode *l; int n; } lineups[] = {
		{camera_lineup_waybeam, 6}, {camera_lineup_waybeam5, 5},
		{camera_lineup_stock, 4},
	};
	const CameraMode none = {0, 0, 0};
	*out = none;
	for (unsigned k = 0; k < sizeof lineups / sizeof lineups[0]; ++k) {
		const int i = camera_mode_pick(lineups[k].l, lineups[k].n, binning,
			venc_fps, 0, 0);
		if (i < 0)
			continue;
		const CameraMode m = lineups[k].l[i];
		if ((uint32_t)m.w * m.h > (uint32_t)out->w * out->h)
			*out = m;
	}
	return out->w != 0;
}
