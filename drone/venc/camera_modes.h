/* Which IMX415 sensor mode serves which venc.fps, and which encode sizes
 * that mode can serve. The pipeline picks from the list the loaded driver
 * offers (waybeam's sensor_imx415_star6e.ko or OpenIPC's stock
 * sensor_imx415_mipi.ko have different high-rate modes); the config loader,
 * which cannot see the driver, accepts a size that one of the two known
 * lineups can serve. docs/camera-modes.md has the per-mode view, binning
 * and limits. Pure C, shared by the pipeline and the config loader. */
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

/* The two driver lineups the loader knows, widest view first. */
extern const CameraMode camera_lineup_waybeam[5];
extern const CameraMode camera_lineup_stock[4];

/* Index into `offered` of the mode for venc_fps, or -1 when none can run
 * it. Without a recording above 1920x1080 only the 2x2-binned modes (at
 * most 1920x1080) are candidates: the larger ones below 4K are unbinned
 * crops that narrow the view. Among the candidates that reach venc_fps
 * the one with the largest area wins, i.e. the widest view, then the
 * lowest ceiling. With a recording above 1080p the smallest mode that
 * holds the recording wins. */
int camera_mode_pick(const CameraMode *offered, int n, uint32_t venc_fps,
	uint32_t rec_w, uint32_t rec_h);

/* Index of the mode with want's size and ceiling in a driver's list, or -1. */
int camera_mode_find(const CameraMode *list, int n, CameraMode want);

/* 1 when w x h fits the mode after the pipeline's keep-aspect precrop: the
 * aspect may differ from the mode's by at most 2 % (the precrop trims the
 * rest of the view), and w x h must fit inside the cropped window (the VPE
 * scales down, never up). */
int camera_mode_size_ok(CameraMode mode, uint32_t w, uint32_t h);

/* 1 when one of the known lineups picks a mode for venc_fps that w x h
 * fits. Otherwise 0, with *hint set to the waybeam lineup's pick and
 * *hint_stock to the stock one's (w == 0 when that lineup has none). */
int camera_mode_known_fit(uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h,
	uint32_t w, uint32_t h, CameraMode *hint, CameraMode *hint_stock);

#ifdef __cplusplus
}
#endif
#endif
