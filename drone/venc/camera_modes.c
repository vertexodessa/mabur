#include "camera_modes.h"

/* waybeam's driver: up to 90 fps the 2x2-binned full view. Above it the
 * sensor cannot read the full height fast enough (a wide binned line needs
 * ~4.9 us and the full height ~2250 lines), so 91-100 and 101-120 use the
 * widest binned crops that stay clean at those rates. */
const CameraMode camera_lineup_waybeam[5] = {
	{3840, 2160, 33}, {2816, 1584, 60}, {1920, 1080, 90},
	{1728, 972, 100}, {1728, 816, 120},
};

/* OpenIPC's stock driver: one high-rate crop, narrower than waybeam's. */
const CameraMode camera_lineup_stock[4] = {
	{3840, 2160, 30}, {2560, 1440, 60}, {1920, 1080, 90}, {1472, 816, 120},
};

int camera_mode_pick(const CameraMode *offered, int n, uint32_t venc_fps,
	uint32_t rec_w, uint32_t rec_h)
{
	const int big = rec_w > 1920 || rec_h > 1080;
	int best = -1;

	for (int i = 0; i < n; ++i) {
		const CameraMode m = offered[i];
		if (m.max_fps < venc_fps)
			continue;
		if (big ? (m.w < rec_w || m.h < rec_h) : (m.w > 1920 || m.h > 1080))
			continue;
		if (best < 0) {
			best = i;
			continue;
		}
		const CameraMode b = offered[best];
		const uint32_t area = (uint32_t)m.w * m.h, barea = (uint32_t)b.w * b.h;
		/* For 4K the smallest holder; otherwise the widest view. */
		if (big ? area < barea : area > barea)
			best = i;
		else if (area == barea && m.max_fps < b.max_fps)
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

static int fits_lineup(const CameraMode *l, int n, uint32_t fps,
	uint32_t rec_w, uint32_t rec_h, uint32_t w, uint32_t h, CameraMode *hint)
{
	const int i = camera_mode_pick(l, n, fps, rec_w, rec_h);
	const CameraMode none = {0, 0, 0};
	*hint = i < 0 ? none : l[i];
	return i >= 0 && camera_mode_size_ok(l[i], w, h);
}

int camera_mode_known_fit(uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h,
	uint32_t w, uint32_t h, CameraMode *hint, CameraMode *hint_stock)
{
	const int a = fits_lineup(camera_lineup_waybeam, 5, venc_fps, rec_w,
		rec_h, w, h, hint);
	const int b = fits_lineup(camera_lineup_stock, 4, venc_fps, rec_w,
		rec_h, w, h, hint_stock);
	return a || b;
}
