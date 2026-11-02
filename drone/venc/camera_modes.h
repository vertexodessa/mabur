/* Which IMX415 sensor mode serves which venc.fps and venc.binning, and
 * which encode sizes that mode can serve. The pipeline picks from the list
 * the loaded driver offers (waybeam's sensor_imx415_star6e.ko, five- or
 * six-mode build, or OpenIPC's stock sensor_imx415_mipi.ko: different
 * high-rate and native modes); the config loader, which cannot see the
 * driver, accepts what one of the known lineups can serve.
 *
 * Two readouts share a lineup. Binned: the 2x2-binned modes, at most
 * 1920x1080, the full view (or a binned crop above 90 fps). Native: the
 * modes larger than 1920x1080 and smaller than 4K, 1:1 pixels from a
 * window of the sensor, so a zoomed view. docs/camera-modes.md has the
 * per-mode view and limits. Pure C, shared by the pipeline and the config
 * loader. */
#ifndef MABUR_VENC_CAMERA_MODES_H
#define MABUR_VENC_CAMERA_MODES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t w, h;     /* the mode's output size */
	uint16_t max_fps;  /* the driver's maxFps for it */
} CameraMode;

/* The driver lineups the loader knows, in each driver's own order. */
extern const CameraMode camera_lineup_waybeam[6];   /* six-mode build */
extern const CameraMode camera_lineup_waybeam5[5];  /* earlier five-mode build */
extern const CameraMode camera_lineup_stock[4];

/* Index into `offered` of the mode for venc_fps under the readout
 * (binning non-zero: binned, zero: native), or -1 when none can run it.
 * Among that readout's modes that reach venc_fps the one with the largest
 * area wins, i.e. the widest view, then the lowest ceiling. A recording
 * larger than 1920x1080 that does not fit that mode in either dimension
 * takes the smallest 4K mode that reaches venc_fps instead. */
int camera_mode_pick(const CameraMode *offered, int n, int binning,
	uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h);

/* Index of the mode with want's size and ceiling in a driver's list, or -1. */
int camera_mode_find(const CameraMode *list, int n, CameraMode want);

/* 1 when w x h fits the mode after the pipeline's keep-aspect precrop: the
 * aspect may differ from the mode's by at most 2 % (the precrop trims the
 * rest of the view), and w x h must fit inside the cropped window (the VPE
 * scales down, never up). */
int camera_mode_size_ok(CameraMode mode, uint32_t w, uint32_t h);

/* 1 when one of the known lineups picks a mode for venc_fps and the
 * readout that w x h fits. Otherwise 0, with *hint set to the six-mode
 * waybeam lineup's pick and *hint_stock to the stock one's (w == 0 when
 * that lineup has none). */
int camera_mode_known_fit(int binning, uint32_t venc_fps, uint32_t rec_w,
	uint32_t rec_h, uint32_t w, uint32_t h, CameraMode *hint,
	CameraMode *hint_stock);

/* The largest mode any known lineup picks for venc_fps and the readout
 * with no recording (the loader's most permissive view of the tier), in
 * *out; 0 with *out zeroed when no known lineup has one (native above
 * 100 fps). */
int camera_mode_known_tier(int binning, uint32_t venc_fps, CameraMode *out);

#ifdef __cplusplus
}
#endif

#endif
