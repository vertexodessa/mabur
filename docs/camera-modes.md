# Camera modes: frame rate, resolution and field of view

The drone's IMX415 sensor offers a handful of readout modes, and `venc.fps`
with `venc.binning` picks one from the loaded driver's list. This page says which, what each costs in field of view, which
`venc.size` values each can serve, and what the ground station has to do to
show it smoothly.

## The drivers

Three IMX415 drivers exist for the drone image, and maburd runs on any of
them. waybeam_venc's star6e driver, `sensor_imx415_star6e.ko`, six-mode
build (waybeam_venc b723e91, the prebuilt module gilankpam/openipc-builder#4
ships; md5 `9ad7d1ac86ccf3817cb563ae858c1e68`), enumerates:

| idx | mode | readout | share of the view (w × h) |
|---|---|---|---|
| 0 | 3840x2160 @ ≤ 33 | all pixels, not binned | 100 % × 100 % |
| 1 | 2944x1656 @ ≤ 60 | native window | ≈ 77 %, 1.29x zoom |
| 2 | 1920x1080 @ ≤ 90 | 2x2 binned | 100 % × 100 % |
| 3 | 2304x1296 @ ≤ 100 | native window | ≈ 61 %, 1.64x zoom |
| 4 | 1728x972 @ ≤ 100 | 2x2 binned crop | 90 % × 90 % |
| 5 | 1728x816 @ ≤ 120 | 2x2 binned crop | 90 % × 76 % |

The views of modes 1 and 3 are upstream's bench measurements
(`docs/findings/2026-11-02-native-readout-bench.md`). The earlier five-mode
build (from `../waybeam_venc/sensors/star6e/`, waybeam 0c880d8) has no
2304x1296 and a narrower 60 fps window:

| idx | mode | readout | sensor window | share of the view (w × h) | aspect |
|---|---|---|---|---|---|
| 0 | 3840x2160 @ ≤ 33 | all pixels, not binned | 3840x2160 | 100 % × 100 % | 16:9 |
| 1 | 2816x1584 @ ≤ 60 | crop, not binned | 2952x1656 → 2816x1584 | native window | 16:9 |
| 2 | 1920x1080 @ ≤ 90 | 2x2 binned | 3840x2160 | 100 % × 100 % | 16:9 |
| 3 | 1728x972 @ ≤ 100 | 2x2 binned crop | 3456x1944, centred | 90 % × 90 % (81 % of the area) | 16:9 |
| 4 | 1728x816 @ ≤ 120 | 2x2 binned crop | 3456x1632, centred | 90 % × 76 % | 2.12:1 |

OpenIPC's stock `sensor_imx415_mipi.ko` enumerates four:

| idx | mode | readout | aspect |
|---|---|---|---|
| 0 | 3840x2160 @ ≤ 30 | all pixels, not binned | 16:9 |
| 1 | 2560x1440 @ ≤ 60 | crop, not binned | 16:9 |
| 2 | 1920x1080 @ ≤ 90 | 2x2 binned | 16:9 |
| 3 | 1472x816 @ ≤ 120 | 2x2 binned crop | 1.80:1 |

maburd reads the loaded driver's list and picks from it by output size and
maxFps (`camera_mode_pick` in `drone/venc/camera_modes.c`, called from
`sensor_select.c`), never by index. Check what is running with
`grep Cur /proc/mi_modules/mi_sensor/mi_sensor0`.

## Which mode `venc.fps` and `venc.binning` select

`venc.binning` picks the readout. `true` (the default): the candidates are
the modes of at most 1920x1080, the 2x2-binned ones. `false` (native): the
modes larger than 1920x1080 and smaller than 4K, which read 1:1 pixels from
a window of the sensor: sharper per output pixel, a zoomed view. Among the
candidates whose maxFps reaches `venc.fps` the largest wins, i.e. the
widest view, then the lower ceiling.

| `venc.fps` | readout | waybeam six-mode | waybeam five-mode | stock driver |
|---|---|---|---|---|
| 1–90 | binned | 1920x1080, full view | 1920x1080 | 1920x1080 |
| 91–100 | binned | 1728x972 crop, 1.11x zoom | 1728x972 | 1472x816 crop |
| 101–120 | binned | 1728x816 crop, 1.11x wider, 1.32x taller zoom | 1728x816 | 1472x816 crop |
| 1–60 | native | 2944x1656, 1.29x | 2816x1584 | 2560x1440, 1.5x |
| 61–100 | native | 2304x1296, 1.64x | none | none |
| 101–120 | native | none: the loader refuses `venc.fps` | | |

A recording larger than 1920x1080 that does not fit the picked mode in
either dimension takes the 3840x2160 mode instead (the loader then requires
`venc.fps` ≤ 30); one that fits (up to 2944x1656 under native 60 on the
six-mode driver) is scaled down from the picked mode on VPE port 1.

A driver that offers no mode for `venc.fps` and the readout fails the
pipeline start with `ERROR: pad N offers no binned|native mode for F fps`.
The loader cannot see the driver, so native 61–100 passes it and fails the
start on the stock or five-mode driver.

Upstream (gilankpam/mabur#108) names the native tiers in a fixed table for
the six-mode driver only. This fork keeps choosing from the driver's own
list, so native 60 also runs on the stock driver the drone images load by
default.

The full view stops at 90 fps because of the sensor, not the encoder: a
frame is VMAX lines × the line time, full height binned needs VMAX ≥ 2250,
and a full-width binned line needs HMAX ≥ ~365 (4.92 µs) or the readout is
malformed (black frames with coloured lines at the right fps). 74.25 MHz /
(2250 × 365) = 90.4 fps. Neither clock nor bit depth moves either floor.
At HMAX 365 a 100 fps frame holds 972 binned lines and a 120 fps frame 816,
which is where modes 3 and 4 come from (waybeam's device mapping:
`../waybeam_venc/documentation/STAR6E_IMX415_HEADROOM.md` §2, §5.6). The
measured views: `docs/findings/2026-10-19-waybeam-imx415-1728x972.md`.

Mode 4's real ceiling is 119.660 Hz (VMAX 1700 at the line rate), not 120:
at `venc.fps = 120` the rate trim locks at the mode's ceiling
(`[sensor] cal: locked at the mode's ceiling`). The trim, the shutter cap
above 90 fps and the steer's ceiling clamp are in `docs/airtime-model.md`
§3 and `docs/phase-lock.md`.

## Sizes

The pipeline crops the mode to `venc.size`'s aspect (the keep-aspect
precrop) and the VPE scales that window down; it never upscales. So
`venc.size` must be within 2 % of the picked mode's aspect (the precrop
trims the rest of the view), fit inside the cropped window, and be even and
at least 320x180. The config loader cannot see the driver: it accepts a size
that the mode either known lineup picks for `venc.fps` can serve, and the
pipeline refuses a size that the loaded driver's pick cannot serve with
`ERROR: venc.size WxH does not fit the driver's WxH@F mode for F fps`
(over the air the previous pipeline is restored). Over the air, fps, size and
readout move together as the single `VENC_MODE` parameter,
`<fps>@<WxH>/binned|native` (`docs/mavlink.md`):

- 1–90 fps: any 16:9 size up to 1920x1080 (1280x720, 1920x1080).
- 91–100 fps: 16:9 up to 1728x972 on waybeam's driver; on the stock one any
  size that fits 1472x816 after the precrop (1280x720: 1.5 % trimmed off the
  sides, 1450x816 scaled down). 1920x1080 is refused on both.
- 101–120 fps: waybeam's 1728:816 shape up to 1728x816 (1728x816, 1152x544,
  864x408), where 1280x720 is refused (16 % off its aspect); on the stock
  driver 1280x720 as above.
- Native: any 16:9 size up to the window (1280x720 and 1920x1080 on every
  driver; up to 2944x1656 or 2304x1296 on the six-mode one). Encoding above
  1920x1080 costs encoder time (upstream measured enc p50 14 ms at
  2944x1656@60 against 5 ms at 1280x720) and needs a ground decoder for it.

Encoding the mode's own size keeps all its detail but costs encoder time:
1728x972 at 100 fps reads 158 % of the H.265 engine's rated macroblock rate
(1280x720 at 100 fps reads 86 %); it held 100 fps on the bench with the
480 MHz encoder clock, with enc 6/8 ms p50/p90 against 4/5 ms at 1280x720.

## The ground station

The GS output has to run at the drone's rate for the phase lock to hold
(`docs/phase-lock.md`): set `screen_mode` to a mode the display offers at
that rate. The player picks only modes the sink's EDID lists, within 0.5 Hz
of the request, and otherwise falls back to the sink's preferred mode (a
log line). Known sinks:

- Avatar Goggles X: 1280x720@100.000 and 1920x1080@100.000 only above
  60 Hz; range limits 50–76 Hz otherwise, so no 90 Hz. Use
  `screen_mode = "1920x1080@100"` with a 91–100 fps drone.

When the drone's measured rate and the panel's refresh are too far apart
for the phase lock (more than 0.55 Hz: a wrong `venc.fps`, or a sink that
fell back), the OSD fps cell reads `source/panel` in caution (`100/60`),
the phase lock pauses and sends nothing, and the player logs
`maburplay: rate mismatch`. A drone in low-power is never a mismatch. If
the rates match but the lock still gives up (the steer cannot reach the
panel, e.g. a panel a hair faster than a mode's ceiling) the fps cell turns
caution without the `/`.

A picture whose aspect differs from the panel (waybeam's 1728x816 on 16:9) is
letterboxed with black bars on the screen, and in the burned DVR when its
size (`dvr.burned.size`) has another aspect, never stretched
(`gs/player/src/fit_rect.h`). The burned DVR's default size is the
decoded picture's own, which needs no letterbox.

## The GS burned recorder at 100 fps

At a 100 Hz panel the GS's burned DVR cannot encode every vblank and
costs live view ~4 ms while it records; run it at `dvr.burned.every = 2`.
Details: `docs/colortrans.md` ("Known limitations").

## The SD recorder above 90 fps

`[record]` keeps its own rules (`docs/vtx-recorder.md`): `record.fps` ≤
`venc.fps`, `record.size` with `venc.size`'s aspect. Above 90 fps:

- A `record.size` larger than the mode (the bundle's 1920x1080 against
  1728x972) is an upscale on VPE port 1.
- A `record.fps` that does not divide `venc.fps` (60 from 100) gives the
  recording uneven frame spacing: the bind's frame-rate limiter passes 60
  of every 100 frames.
- The H.265 engine runs one encode at a time and the live 1728x972@100
  encode alone is past its rated rate, so a recording adds encode time to
  the live link.
