/* Which IMX415 sensor mode serves which venc.fps, and which encode sizes
 * that mode can serve. The table is waybeam's star6e IMX415 driver lineup
 * (sensor_imx415_star6e.ko), the driver the drone image ships; modes are
 * found in the driver's list by geometry and ceiling, never by index.
 * Two readouts share the lineup: binned (2x2, the full view) and native
 * (full-resolution, a narrower fps range). camera_mode_for's binning flag
 * picks which readout's tiers apply; a zero-size result (w == 0) means
 * there is no mode for that readout at that rate, and the config loader
 * refuses it so the pipeline never sees it.
 * docs/camera-modes.md has the per-mode view, binning and limits. Pure C,
 * shared by the pipeline and the config loader. */
#ifndef MABUR_VENC_CAMERA_MODES_H
#define MABUR_VENC_CAMERA_MODES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t w, h;     /* the mode's output size */
	uint16_t max_fps;  /* the driver's maxFps for it */
} CameraMode;

/* The mode for a configured venc.fps and readout (binning non-zero selects
 * the binned tiers, zero selects the native tiers), or the 4K mode when
 * the recorder wants more than 1920x1080 and more than the tier's mode in
 * either dimension (the loader then requires venc.fps <= 30). */
CameraMode camera_mode_for(int binning, uint32_t venc_fps, uint32_t rec_w, uint32_t rec_h);

/* Index of the mode with want's size and ceiling in a driver's list, or -1. */
int camera_mode_find(const CameraMode *list, int n, CameraMode want);

/* 1 when w x h has the mode's aspect ratio and fits inside it (the VPE
 * scales down; a crop or an upscale is not offered). */
int camera_mode_size_ok(CameraMode mode, uint32_t w, uint32_t h);

/* The list the loaded driver offers, kept as the pipeline last read it so
 * the MAVLink component can report it (VENC_MODES). At most 16 modes are
 * kept; until the camera has started there are none. Both are safe to call
 * from any thread. */
void camera_modes_publish(const CameraMode *list, int n);
int camera_modes_offered(CameraMode *out, int cap);

/* The modes of `list` that camera_mode_for can pick, spelled as VENC_MODE
 * values at each mode's own size and top rate,
 * "<max_fps>@<W>x<H>/binned|native", comma-separated in the driver's order:
 * a driver mode outside the lineup (another driver build's crop, the 4K
 * mode) is left out, since the pipeline would not run it. An entry that
 * would not fit in `cap` bytes with the closing NUL ends the text there, so
 * it never holds half an entry. Returns the text's length; `out` is always
 * NUL-terminated when cap > 0. */
size_t camera_modes_text(const CameraMode *list, int n, char *out, size_t cap);

#ifdef __cplusplus
}
#endif
#endif
