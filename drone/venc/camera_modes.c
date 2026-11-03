#include "camera_modes.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>

/* Up to 90 fps: the 2x2-binned full view. Above it the sensor cannot read
 * the full height fast enough (a wide binned line needs ~4.9 us and the
 * full height ~2250 lines), so 91-100 and 101-120 use the widest binned
 * crops that stay clean at those rates. */
static const CameraMode kFullView = {1920, 1080, 90};
static const CameraMode kWide100 = {1728, 972, 100};
static const CameraMode kWide120 = {1728, 816, 120};
/* The native (non-binned) readout: a narrower fps range than the binned
 * one, and nothing above 100 fps (kNone — the loader refuses it). Both
 * modes are timed ~0.5 Hz above nominal (60.57 and 100.52 Hz) so the GS
 * phase lock can steer them up to a panel at the nominal rate; the 100 mode
 * is 2272x1278 rather than 2304x1296 because the wider window had no blank
 * lines left to give (docs/camera-modes.md). */
static const CameraMode kNative60 = {2944, 1656, 60};
static const CameraMode kNative100 = {2272, 1278, 100};
static const CameraMode kNone = {0, 0, 0};
static const CameraMode k4K = {3840, 2160, 33};

CameraMode camera_mode_for(int binning, uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h)
{
	CameraMode tier;
	if (binning) {
		if (venc_fps > kWide100.max_fps)
			tier = kWide120;
		else if (venc_fps > kFullView.max_fps)
			tier = kWide100;
		else
			tier = kFullView;
	} else {
		if (venc_fps > kNative100.max_fps)
			tier = kNone;
		else if (venc_fps > kNative60.max_fps)
			tier = kNative100;
		else
			tier = kNative60;
	}
	if ((rec_w > 1920 || rec_h > 1080) && (rec_w > tier.w || rec_h > tier.h))
		return k4K;
	return tier;
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
	return (uint64_t)w * mode.h == (uint64_t)h * mode.w &&
		w <= mode.w && h <= mode.h;
}

static pthread_mutex_t offered_mu = PTHREAD_MUTEX_INITIALIZER;
static CameraMode offered_list[16];
static int offered_n;

void camera_modes_publish(const CameraMode *list, int n)
{
	const int cap = (int)(sizeof offered_list / sizeof offered_list[0]);
	if (n < 0)
		n = 0;
	if (n > cap)
		n = cap;
	pthread_mutex_lock(&offered_mu);
	if (n > 0)
		memcpy(offered_list, list, (size_t)n * sizeof *list);
	offered_n = n;
	pthread_mutex_unlock(&offered_mu);
}

int camera_modes_offered(CameraMode *out, int cap)
{
	pthread_mutex_lock(&offered_mu);
	const int n = offered_n < cap ? offered_n : cap;
	if (n > 0)
		memcpy(out, offered_list, (size_t)n * sizeof *out);
	pthread_mutex_unlock(&offered_mu);
	return n > 0 ? n : 0;
}

size_t camera_modes_text(const CameraMode *list, int n, char *out, size_t cap)
{
	size_t len = 0;
	if (cap == 0)
		return 0;
	out[0] = '\0';
	for (int i = 0; i < n; ++i) {
		const CameraMode m = list[i];
		/* Only the tiers camera_mode_for picks from. */
		const CameraMode binned_tiers[] = {kFullView, kWide100, kWide120};
		const CameraMode native_tiers[] = {kNative60, kNative100};
		const int binned = camera_mode_find(binned_tiers, 3, m) >= 0;
		if (!binned && camera_mode_find(native_tiers, 2, m) < 0)
			continue;
		char e[48];
		const int k = snprintf(e, sizeof e, "%s%u@%ux%u/%s",
			len ? "," : "", (unsigned)m.max_fps, (unsigned)m.w,
			(unsigned)m.h, binned ? "binned" : "native");
		if (k <= 0 || len + (size_t)k + 1 > cap)
			break;
		memcpy(out + len, e, (size_t)k + 1);
		len += (size_t)k;
	}
	return len;
}
