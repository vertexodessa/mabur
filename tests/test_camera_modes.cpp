// Host test for drone/venc/camera_modes.c: which mode of a driver's list
// serves which venc.fps, and which encode sizes a mode can serve.
#include "camera_modes.h"
#include "mtest.h"

namespace {

bool same(CameraMode a, CameraMode b) {
  return a.w == b.w && a.h == b.h && a.max_fps == b.max_fps;
}

const CameraMode kWaybeam[] = {
    {3840, 2160, 33}, {2816, 1584, 60}, {1920, 1080, 90}, {1728, 972, 100}, {1728, 816, 120}};
const CameraMode kWaybeam6[] = {{3840, 2160, 33}, {2944, 1656, 60},  {1920, 1080, 90},
                                {2304, 1296, 100}, {1728, 972, 100}, {1728, 816, 120}};
const CameraMode kStock[] = {
    {3840, 2160, 30}, {2560, 1440, 60}, {1920, 1080, 90}, {1472, 816, 120}};

}  // namespace

TEST(pick_waybeam_by_fps) {
  auto at = [](uint32_t fps) { return camera_mode_pick(kWaybeam, 5, 1, fps, 0, 0); };
  CHECK(at(30) == 2);
  CHECK(at(60) == 2);  // the binned full view, not the 2816x1584 crop
  CHECK(at(90) == 2);
  CHECK(at(91) == 3);
  CHECK(at(100) == 3);
  CHECK(at(101) == 4);
  CHECK(at(120) == 4);
  CHECK(at(121) == -1);
}

TEST(pick_stock_by_fps) {
  auto at = [](uint32_t fps) { return camera_mode_pick(kStock, 4, 1, fps, 0, 0); };
  CHECK(at(60) == 2);  // not the 2560x1440 crop
  CHECK(at(90) == 2);
  CHECK(at(100) == 3);
  CHECK(at(120) == 3);
}

TEST(pick_six_mode_binned_skips_the_native_windows) {
  auto at = [](uint32_t fps) { return camera_mode_pick(kWaybeam6, 6, 1, fps, 0, 0); };
  CHECK(at(60) == 2);
  CHECK(at(100) == 4);  // 1728x972, not the larger 2304x1296 native window
  CHECK(at(120) == 5);
}

TEST(pick_native_takes_the_widest_window_that_reaches_the_rate) {
  auto w6 = [](uint32_t fps) { return camera_mode_pick(kWaybeam6, 6, 0, fps, 0, 0); };
  CHECK(w6(30) == 1);
  CHECK(w6(60) == 1);   // 2944x1656
  CHECK(w6(61) == 3);   // 2304x1296
  CHECK(w6(100) == 3);
  CHECK(w6(101) == -1);
  auto w5 = [](uint32_t fps) { return camera_mode_pick(kWaybeam, 5, 0, fps, 0, 0); };
  CHECK(w5(60) == 1);   // 2816x1584
  CHECK(w5(61) == -1);
  auto st = [](uint32_t fps) { return camera_mode_pick(kStock, 4, 0, fps, 0, 0); };
  CHECK(st(60) == 1);   // 2560x1440
  CHECK(st(61) == -1);
}

TEST(pick_4k_record_wins) {
  CHECK(camera_mode_pick(kWaybeam, 5, 1, 30, 3840, 2160) == 0);
  CHECK(camera_mode_pick(kStock, 4, 1, 30, 3840, 2160) == 0);
  CHECK(camera_mode_pick(kStock, 4, 1, 60, 3840, 2160) == -1);
  // Binned stays full view: a 1440p recording takes the 4K mode, never the
  // smaller native window that would also hold it.
  CHECK(camera_mode_pick(kStock, 4, 1, 30, 2560, 1440) == 0);
  CHECK(camera_mode_pick(kWaybeam, 5, 1, 30, 2560, 1440) == 0);
}

TEST(a_recording_that_fits_the_native_window_stays_on_it) {
  CHECK(camera_mode_pick(kWaybeam6, 6, 0, 60, 2560, 1440) == 1);
  CHECK(camera_mode_pick(kWaybeam6, 6, 0, 60, 2944, 1656) == 1);
  CHECK(camera_mode_pick(kStock, 4, 0, 60, 2560, 1440) == 1);
  // Larger than the window: the 4K mode, which caps the rate.
  CHECK(camera_mode_pick(kWaybeam6, 6, 0, 100, 2560, 1440) == -1);
  CHECK(camera_mode_pick(kWaybeam6, 6, 0, 30, 3840, 2160) == 0);
}

TEST(known_lineups_match_the_tables) {
  CHECK(same(camera_lineup_waybeam5[4], kWaybeam[4]));
  CHECK(same(camera_lineup_waybeam[3], kWaybeam6[3]));
  CHECK(same(camera_lineup_stock[3], kStock[3]));
}

TEST(known_tier_is_the_widest_of_the_lineups) {
  CameraMode t{};
  CHECK(camera_mode_known_tier(0, 60, &t) == 1 && same(t, CameraMode{2944, 1656, 60}));
  CHECK(camera_mode_known_tier(0, 100, &t) == 1 && same(t, CameraMode{2304, 1296, 100}));
  CHECK(camera_mode_known_tier(0, 101, &t) == 0 && t.w == 0);
  CHECK(camera_mode_known_tier(1, 60, &t) == 1 && same(t, CameraMode{1920, 1080, 90}));
  CHECK(camera_mode_known_tier(1, 120, &t) == 1 && same(t, CameraMode{1728, 816, 120}));
}

TEST(find_matches_geometry_and_max_fps) {
  CHECK(camera_mode_find(kWaybeam, 5, CameraMode{1728, 972, 100}) == 3);
  CHECK(camera_mode_find(kWaybeam, 5, CameraMode{1920, 1080, 90}) == 2);
  CHECK(camera_mode_find(kStock, 4, CameraMode{1728, 972, 100}) == -1);
  // Same size, different ceiling: not the mode the table means.
  CHECK(camera_mode_find(kStock, 4, CameraMode{3840, 2160, 33}) == -1);
}

TEST(size_ok_allows_a_small_precrop) {
  const CameraMode m100{1728, 972, 100}, m120{1728, 816, 120}, s120{1472, 816, 120};
  CHECK(camera_mode_size_ok(m100, 1280, 720) == 1);
  CHECK(camera_mode_size_ok(m100, 1728, 972) == 1);
  CHECK(camera_mode_size_ok(m100, 1920, 1080) == 0);  // larger than the mode
  CHECK(camera_mode_size_ok(m120, 1728, 816) == 1);
  CHECK(camera_mode_size_ok(m120, 1152, 544) == 1);
  CHECK(camera_mode_size_ok(m120, 1280, 720) == 0);   // 16:9 is 16 % off 2.12:1
  // 16:9 on 1472x816 is 1.5 % off: the precrop keeps 1450x816.
  CHECK(camera_mode_size_ok(s120, 1280, 720) == 1);
  CHECK(camera_mode_size_ok(s120, 1440, 810) == 1);
  CHECK(camera_mode_size_ok(s120, 1472, 828) == 0);   // taller than the mode
  CHECK(camera_mode_size_ok(s120, 1460, 822) == 0);   // wider than the 1450 window
  CHECK(camera_mode_size_ok(s120, 0, 0) == 0);
}

TEST(known_fit_takes_either_lineup) {
  CameraMode a{}, b{};
  CHECK(camera_mode_known_fit(1, 120, 0, 0, 1280, 720, &a, &b) == 1);   // stock
  CHECK(camera_mode_known_fit(1, 120, 0, 0, 1728, 816, &a, &b) == 1);   // waybeam
  CHECK(camera_mode_known_fit(1, 100, 0, 0, 1920, 1080, &a, &b) == 0);
  CHECK(same(a, CameraMode{1728, 972, 100}) && same(b, CameraMode{1472, 816, 120}));
  CHECK(camera_mode_known_fit(1, 130, 0, 0, 1280, 720, &a, &b) == 0);
  CHECK(a.w == 0 && b.w == 0);
}

TEST(known_fit_native) {
  CameraMode a{}, b{};
  CHECK(camera_mode_known_fit(0, 60, 0, 0, 1920, 1080, &a, &b) == 1);
  CHECK(same(a, CameraMode{2944, 1656, 60}) && same(b, CameraMode{2560, 1440, 60}));
  CHECK(camera_mode_known_fit(0, 60, 0, 0, 2944, 1656, &a, &b) == 1);  // six-mode only
  CHECK(camera_mode_known_fit(0, 100, 0, 0, 1920, 1080, &a, &b) == 1);
  CHECK(b.w == 0);  // the stock driver has no native mode at 100
  CHECK(camera_mode_known_fit(0, 100, 0, 0, 2560, 1440, &a, &b) == 0);  // larger than 2304x1296
  CHECK(camera_mode_known_fit(0, 120, 0, 0, 1280, 720, &a, &b) == 0);
}

MTEST_MAIN
