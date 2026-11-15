// Covers what survives in drone/venc/venc_cfg.c after venc.resilience was
// decomposed into its components: the absent-key defaults and
// the SuperFrame byte math. The preset table, venc_cfg_expand_preset() and
// VencPresetOut are gone — the pipeline reads VencCfg directly.
#include <cstdio>
#include <string>

#include "mtest.h"
#include "venc_cfg.h"

// venc_cfg_defaults() is the one table of truth for "key absent from the
// config". The intra-refresh + SVC-T defaults reproduce, at 1080p60,
// exactly what the deleted "rally" preset expanded to: a 150 ms sweep is
// 9 frames = ceil(34 CTU rows / 9) = 4 rows per P-frame, its stripe QP was 36,
// and rally ran 1:1 SVC-T with enhance->base prediction on. Deploying this
// change therefore does not move the encoder.
TEST(defaults_reproduce_the_old_rally_preset) {
  VencCfg c{};
  venc_cfg_defaults(&c);
  CHECK(c.intra_refresh_frames == 9);
  CHECK(venc_cfg_intra_rows(1080, c.intra_refresh_frames) == 4);
  CHECK(c.intra_refresh_qp == 36);
  CHECK(c.ref_base == 1);
  CHECK(c.ref_enhance == 1);
  CHECK(c.ref_pred == true);
  CHECK(c.gop_s == 2.0);
}

// venc_cfg_ctu_rows(): H.265 CTU is 32x32, so a picture is ceil(height/32)
// CTU rows -- what venc_cfg_intra_rows() spreads over the sweep, so its
// boundaries matter at every supported picture size.
// (The deleted test_venc_intra_refresh.cpp pinned the 1080 and 720 cases
// through intra_refresh_compute(); this is the same arithmetic, direct.)
TEST(ctu_rows_is_ceil_height_over_32) {
  CHECK(venc_cfg_ctu_rows(1080) == 34);  // 33.75, rounds up
  CHECK(venc_cfg_ctu_rows(720) == 23);   // 22.5
  CHECK(venc_cfg_ctu_rows(1024) == 32);  // exact multiple
  CHECK(venc_cfg_ctu_rows(1056) == 33);  // exact multiple
  CHECK(venc_cfg_ctu_rows(1088) == 34);  // exact multiple, same as 1080
  CHECK(venc_cfg_ctu_rows(1) == 1);      // never rounds to zero rows
  CHECK(venc_cfg_ctu_rows(0) == 0);
}

// venc_cfg_intra_rows(): venc.intra_refresh_frames is the sweep length, so
// the rows per P-frame follow the ENCODED height -- the same key means the
// same self-heal time at 1080p and 720p (the old raw rows key failed boot
// at 720p with its 1080p value). ceil() so the sweep never runs longer than
// asked; a sweep longer than the picture's rows bottoms out at 1 row.
TEST(intra_rows_spreads_the_picture_over_the_sweep) {
  CHECK(venc_cfg_intra_rows(1080, 1) == 34);  // whole picture every P
  CHECK(venc_cfg_intra_rows(720, 1) == 23);
  CHECK(venc_cfg_intra_rows(1080, 2) == 17);
  CHECK(venc_cfg_intra_rows(720, 2) == 12);   // 11.5 rounds up
  CHECK(venc_cfg_intra_rows(1080, 9) == 4);   // the default, = old rows 4
  CHECK(venc_cfg_intra_rows(720, 9) == 3);
  CHECK(venc_cfg_intra_rows(1080, 100) == 1); // never 0 rows while on
  CHECK(venc_cfg_intra_rows(1080, 0) == 0);   // off
  CHECK(venc_cfg_intra_rows(0, 9) == 0);
}

// venc_superframe_p_bytes(): P-frame SuperFrame threshold as a percentage
// of the per-frame budget at the PROGRAMMED rate (kbps x 1024), so the cap
// follows the rung. Rung 5 at 60 fps: 16000 kbps -> 16.384 Mbit/s -> 34133 B
// per frame; 200 % -> 68266 B. The venc-overshoot bench saw scene-cut
// frames of 195 kB against that budget.
TEST(superframe_p_bytes_follows_the_rung_budget) {
  CHECK(venc_superframe_p_bytes(100, 16000, 60, 0, 0, 0) == 34133);
  CHECK(venc_superframe_p_bytes(200, 16000, 60, 0, 0, 0) == 68266);
  CHECK(venc_superframe_p_bytes(150, 3900, 60, 0, 0, 0) == 12480);  // rung 1: 66560 bits/frame x 1.5 / 8
  CHECK(venc_superframe_p_bytes(0, 16000, 60, 0, 0, 0) == 0);       // off
  CHECK(venc_superframe_p_bytes(200, 16000, 0, 0, 0, 0) == 0);      // no fps yet
  CHECK(venc_superframe_p_bytes(1000, 200000, 1, 0, 0, 0) > 0);     // no overflow at the rails
  // The SDK takes the threshold in bits (x8 in u32): never past what fits.
  CHECK(venc_superframe_p_bytes(1000, 4000000, 1, 0, 0, 0) == 0xFFFFFFFFu / 8u);
}

// The floor: below an ordinary P frame the SDK aborts every P and opens a
// new GOP, so every frame is a keyframe (2026-11-15: fl-1980 rung 0, cap
// 5760 B against 6-7 kB P frames; fl-1978 low_power, P max 8472 B at the
// 8533 B cap, then 66-100 kB keyframes). 14 kB per megapixel is at least
// the p99 of full-bitrate P frames in every mode flown that day.
TEST(superframe_p_bytes_never_below_the_floor) {
  // rung 0 at 120@1728x816: 2700 kbps -> 5760 B, floored to 19740 B.
  CHECK(venc_superframe_p_bytes(200, 2700, 120, 0, 0, 0) == 5760);
  CHECK(venc_superframe_p_bytes(200, 2700, 120, 1728, 816, 14) == 19740);
  // low_power at 2944x1656: 1000 kbps at 30 fps -> 8533 B, floored to 68253 B.
  CHECK(venc_superframe_p_bytes(200, 1000, 30, 0, 0, 0) == 8533);
  CHECK(venc_superframe_p_bytes(200, 1000, 30, 2944, 1656, 14) == 68253);
  // MAX_RANGE (boot, failsafe) at 120 fps: 1300 kbps -> 2773 B.
  CHECK(venc_superframe_p_bytes(200, 1300, 120, 1728, 816, 14) == 19740);
  // The top rung is untouched: 14000 kbps at 120 fps -> 29866 B > 19740 B.
  CHECK(venc_superframe_p_bytes(200, 14000, 120, 1728, 816, 14) == 29866);
  // The floor never switches a ceiling on.
  CHECK(venc_superframe_p_bytes(0, 2700, 120, 1728, 816, 14) == 0);
}

// The SSC338Q cuts slices in whole 64-px CTU rows
// (docs/findings/2026-10-09-venc-slice.md):
// 1080p = 17 rows, and only counts whose fixed row height reproduces the
// count are achievable.
TEST(slice_rows_follow_the_sdk_row_geometry) {
  CHECK(venc_cfg_ctb64_rows(1080) == 17);
  CHECK(venc_cfg_ctb64_rows(720) == 12);
  CHECK(venc_cfg_slice_rows(1080, 4) == 5);    // 5/5/5/2
  CHECK(venc_cfg_slice_rows(1080, 5) == 4);    // 4/4/4/4/1
  CHECK(venc_cfg_slice_rows(1080, 9) == 2);
  CHECK(venc_cfg_slice_rows(1080, 17) == 1);
  CHECK(venc_cfg_slice_rows(1080, 7) == 0);    // k=3 gives 6 slices, not 7
  CHECK(venc_cfg_slice_rows(1080, 18) == 0);   // more slices than rows
  CHECK(venc_cfg_slice_rows(1080, 1) == 0);    // off
  CHECK(venc_cfg_slice_rows(1080, 0) == 0);
}

TEST(slices_default_off) {
  VencCfg c{};
  venc_cfg_defaults(&c);
  CHECK(c.slices == 1);
}

TEST(gop_frames_zero_seconds_is_the_no_refresh_sentinel) {
  CHECK(venc_cfg_gop_frames(0.0, 60) == VENC_GOP_NONE);
  CHECK(venc_cfg_gop_frames(-1.0, 60) == VENC_GOP_NONE);
}

TEST(gop_frames_scales_seconds_by_fps) {
  CHECK(venc_cfg_gop_frames(0.5, 60) == 30);
  CHECK(venc_cfg_gop_frames(10.0, 60) == 600);
  CHECK(venc_cfg_gop_frames(0.5, 0) == 15);    // fps 0 reads as 30 (the waybeam default)
  CHECK(venc_cfg_gop_frames(0.001, 60) == 1);  // never below one frame
}

MTEST_MAIN
