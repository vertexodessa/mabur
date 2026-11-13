#include "mtest.h"
#include "cal_analysis.h"

#include <vector>

using namespace maburgs;

namespace {

// Builds one cell. rssi is the best card's median; -999 = unmeasured.
CalCell cell(int idx, int pct, int rssi = -70, int expected = 100) {
  CalCell c;
  c.idx = idx;
  c.expected = static_cast<uint16_t>(expected);
  c.received[0] = static_cast<uint16_t>(expected * pct / 100);
  c.received[1] = 0;
  c.rssi_dbm[0] = rssi;
  c.have_rssi[0] = rssi != kRssiNone;
  c.have_rssi[1] = false;
  return c;
}

}  // namespace

// mcs7 as measured on the bench: a sensitivity floor below idx 34, a clean run
// to 49, then the comb -- 2% at 56 but 95% at 57. The wall is the END OF THE
// FIRST CONTIGUOUS RUN (49). The island at 57 is not headroom.
TEST(mcs7_first_dip_not_last_good) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 33; ++i) cells.push_back(cell(i, 10));   // floor
  for (int i = 34; i <= 49; ++i) cells.push_back(cell(i, 98));  // clean run
  for (int i = 50; i <= 55; ++i) cells.push_back(cell(i, 30));  // dip
  cells.push_back(cell(56, 2));
  // An island ABOVE the threshold: the measured comb produces these (mcs6
  // reads 63% at idx 52 but 95-97% at 53-57). This is the cell that
  // discriminates -- a "last index >= 90%" implementation returns 57 here,
  // the correct first-contiguous-run implementation returns 49.
  cells.push_back(cell(57, 95));
  for (int i = 58; i <= 63; ++i) cells.push_back(cell(i, 0));

  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 49);
  CHECK(w.floor_idx == 34);
  CHECK((w.flags & kCalNoDip) == 0);
  CHECK((w.flags & kCalUndetermined) == 0);
}

// mcs0: never dips anywhere in the sweep. There is no compression wall to
// find, so the row is parked at the highest index this unit can express --
// kRailRel, the +63 rail of the chip's 7-bit per-rate diff field, relative
// to whatever per-channel anchor the chip adds itself.
//
// This replaced an RSSI "saturation knee" (the lowest index within 1 dB of
// the row's peak median RSSI). That number was not reproducible: on one unit
// it read 72, 56, 84 and 56 on four runs an hour apart, and in one run mcs0
// and mcs1 -- the same PA, the same modulation class -- came out 3 dB apart.
// The rule cannot do better: the transfer curve creeps at ~0.2 dB/idx, so a
// 1 dB tolerance band already spans ~5 indices before 1 dB of RSSI
// quantization moves it further. Parking at the rail is instead exact,
// identical every run, and PROVABLY INSIDE MEASURED-GOOD TERRITORY -- the
// rail IS the top cell of the sweep (kSweepLo = -41 is chosen so the grid
// lands on kRailRel exactly, gs/src/cal_plan.h), so a no-dip row
// just delivered >=90% AT the very index it parks on. The old knee was
// never validated by delivery at any index.
//
// The fixture sweeps -41..63 step 4, a grid that ends on the rail: if
// kSweepLo/kSweepHi ever drift so the rail stops being a measured cell,
// this test is where that shows up.
TEST(mcs0_no_dip_parks_at_the_diff_field_rail) {
  std::vector<CalCell> cells;
  // Flat floor to 28, ~0.3 dB/idx ramp, then a flat ceiling -- the real
  // measured shape, which this rule deliberately no longer reads.
  for (int i = -41; i <= 63; i += 4) {
    int rssi;
    if (i <= 28) rssi = -80;
    else if (i <= 91) rssi = -80 + (i - 28) * 3 / 10;
    else rssi = -80 + (91 - 28) * 3 / 10;
    cells.push_back(cell(i, 100, rssi));
  }
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalNoDip) != 0);
  CHECK((w.flags & kCalUndetermined) == 0);
  CHECK(w.wall == kRailRel);
}

TEST(row_never_reaching_threshold_is_undetermined) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 127; i += 4) cells.push_back(cell(i, 40));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalUndetermined) != 0);
  CHECK(w.wall == kNoWall);  // caller must leave the config entry alone
}

TEST(negative_relative_indices_are_ordinary_cells) {
  // Relative indices: a wall can sit below the anchor (mcs7 read -4 on
  // the bench at anchor 53). Nothing may treat < 0 as a sentinel.
  std::vector<CalCell> cells;
  for (int i = -40; i <= -8; i += 4) cells.push_back(cell(i, 98));
  for (int i = -4; i <= 60; i += 4) cells.push_back(cell(i, 10));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == -8);
  CHECK(w.floor_idx == -40);
  CHECK((w.flags & kCalUndetermined) == 0);
}

// Diversity is not protection against PA compression: both cards see the same
// degraded waveform. Scoring the union inflates delivery and pushes the wall
// UP, which is the overdriven direction.
TEST(uses_best_single_card_not_union) {
  std::vector<CalCell> cells;
  for (int i = 40; i <= 60; ++i) {
    CalCell c;
    c.idx = static_cast<uint8_t>(i);
    c.expected = 100;
    // Card 0 is the good card and walls at 50.
    c.received[0] = static_cast<uint16_t>(i <= 50 ? 98 : 20);
    // Card 1 hears nothing below the wall and 70% above it. A summed/union
    // implementation would therefore score 20+70 = 90% past idx 50, find no
    // dip at all, and fall through to the knee -- so this fixture fails
    // against a union scorer and passes only for best-single-card. Card 1's
    // total volume (700) stays under card 0's (1278), so card 0 is still the
    // card selected.
    c.received[1] = static_cast<uint16_t>(i <= 50 ? 0 : 70);
    c.rssi_dbm[0] = -70;
    c.rssi_dbm[1] = -85;
    c.have_rssi[0] = c.have_rssi[1] = true;
    cells.push_back(c);
  }
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 50);
  CHECK(w.best_card == 0);
  CHECK((w.flags & kCalNoDip) == 0);  // where a union scorer would land
}

TEST(flags_saturation_from_peak_rssi) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 60; ++i) cells.push_back(cell(i, 98, -40));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalSaturated) != 0);
}

TEST(flags_narrow_window) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 43; ++i) cells.push_back(cell(i, 10));
  for (int i = 44; i <= 46; ++i) cells.push_back(cell(i, 98));  // 3-wide
  for (int i = 47; i <= 60; ++i) cells.push_back(cell(i, 5));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 46);
  CHECK((w.flags & kCalNarrow) != 0);
}

TEST(flags_card_disagreement) {
  std::vector<CalCell> cells;
  for (int i = 40; i <= 60; ++i) {
    CalCell c;
    c.idx = static_cast<uint8_t>(i);
    c.expected = 100;
    c.received[0] = static_cast<uint16_t>(i <= 55 ? 98 : 10);
    c.received[1] = static_cast<uint16_t>(i <= 45 ? 98 : 10);
    c.rssi_dbm[0] = c.rssi_dbm[1] = -70;
    c.have_rssi[0] = c.have_rssi[1] = true;
    cells.push_back(c);
  }
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalCardDisagree) != 0);
}

TEST(empty_input_is_undetermined) {
  const auto w = analyze_rate({}, CalThresholds{});
  CHECK(w.wall == kNoWall);
  CHECK((w.flags & kCalUndetermined) != 0);
}

// mcs4/HT20, long GI: phy_rate_mbps gives 39 Mbps. The drone's TX queue
// keeps airing after its 2 s blast returns, so the GS heard this point for
// 2.3 s: 5705 frames of 1435 air bytes, the first at 1000 ms and the last
// at 3300 ms. The air was saturated for that whole span, and the 5704
// frames after the first one are what it carried in it:
// 5704 * 11480 / (2.3 * 39e6) = 0.7300... -- not the 0.84 the commanded
// dwell would read.
TEST(efficiency_is_delivered_over_the_receive_span) {
  constexpr uint64_t kBits = 1435 * 8;
  CapCell c;
  c.offered = 5705;
  c.blocked_pct = 80;  // well clear of tx_bound
  c.frames = {5705, 0};
  c.air_bits = {5705 * kBits, 0};
  c.first_bits = {kBits, 0};
  c.first_ms = {1000, 0};
  c.last_ms = {3300, 0};
  c.rssi_dbm = {-70, 0};
  c.have_rssi = {true, false};
  const auto r = analyze_capacity(c, /*mcs=*/4, /*bw=*/20, /*dwell_ms=*/2000,
                                   CalThresholds{});
  CHECK(r.eff == 0.73);
  CHECK(r.span_ms == 2300);
  CHECK((r.flags & kCalShortSpan) == 0);
  CHECK(r.best_card == 0);
}

// The span is the best card's own: card 1 heard more bits, so its first
// and last frame time the point even though card 0's span is longer.
TEST(the_span_is_the_best_cards) {
  constexpr uint64_t kBits = 1435 * 8;
  CapCell c;
  c.offered = 5705;
  c.blocked_pct = 80;
  c.frames = {3000, 5705};
  c.air_bits = {3000 * kBits, 5705 * kBits};
  c.first_bits = {kBits, kBits};
  c.first_ms = {500, 1000};
  c.last_ms = {4000, 3300};
  const auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK(r.best_card == 1);
  CHECK(r.span_ms == 2300);
  CHECK(r.eff == 0.73);
}

// Too few frames, or too short a span, to time the delivery: eff falls back
// to the commanded dwell and the point says so. mcs4/20 at 39 Mbps:
// 5000 frames * 11480 bits / (2 s * 39e6) = 0.7358 -> 0.74.
TEST(a_short_span_falls_back_to_the_commanded_dwell_and_says_so) {
  constexpr uint64_t kBits = 1435 * 8;
  CapCell c;
  c.offered = 5000;
  c.blocked_pct = 80;
  c.frames = {5000, 0};
  c.air_bits = {5000 * kBits, 0};
  c.first_bits = {kBits, 0};
  c.first_ms = {1000, 0};
  c.last_ms = {1499, 0};  // 499 ms: under the 500 ms floor
  auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalShortSpan) != 0);
  CHECK(r.eff == 0.74);
  CHECK(r.span_ms == 499);

  c.last_ms = {1500, 0};  // 500 ms: long enough to time
  r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalShortSpan) == 0);
  CHECK(r.eff == 1.0);  // 4999 * 11480 / (0.5 s * 39e6) clamps at 1.0

  // mcs0/HT20 (6.5 Mbps), a 500 ms span. 19 frames are too few to time:
  // 19 * 11480 / (2 s * 6.5e6) = 0.017, floored at 0.05.
  c.frames = {19, 0};
  c.air_bits = {19 * kBits, 0};
  r = analyze_capacity(c, 0, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalShortSpan) != 0);
  CHECK(r.eff == 0.05);
  // 20 are enough, and the span is what times them: 19 * 11480 /
  // (0.5 s * 6.5e6) = 0.067 -> 0.07, where the dwell would read 0.05.
  c.frames = {20, 0};
  c.air_bits = {20 * kBits, 0};
  r = analyze_capacity(c, 0, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalShortSpan) == 0);
  CHECK(r.span_ms == 500);
  CHECK(r.eff == 0.07);
}

TEST(best_card_is_the_one_with_more_bits) {
  CapCell c;
  c.offered = 1000;
  c.blocked_pct = 80;
  c.frames = {400, 900};
  c.air_bits = {10000000, 56800000};
  c.rssi_dbm = {-70, -70};
  c.have_rssi = {true, true};
  const auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK(r.best_card == 1);
}

TEST(tx_bound_when_the_sender_was_rarely_blocked) {
  CapCell c;
  c.offered = 1000;
  c.blocked_pct = 30;  // under the default 50 % threshold
  c.frames = {980, 0};
  c.air_bits = {56800000, 0};
  c.rssi_dbm = {-70, 0};
  c.have_rssi = {true, false};
  const auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalTxBound) != 0);
}

TEST(lossy_when_fewer_than_nine_tenths_arrive) {
  CapCell c;
  c.offered = 1000;
  c.blocked_pct = 80;
  c.frames = {800, 0};  // 80 % < 90 % of offered
  c.air_bits = {46000000, 0};
  c.rssi_dbm = {-70, 0};
  c.have_rssi = {true, false};
  const auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalLossy) != 0);
}

TEST(zero_frames_floors_efficiency_and_flags_lossy) {
  CapCell c;
  c.offered = 1000;
  c.blocked_pct = 80;
  c.frames = {0, 0};
  c.air_bits = {0, 0};
  c.rssi_dbm = {-70, 0};
  c.have_rssi = {true, false};
  const auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK(r.eff == 0.05);
  CHECK((r.flags & kCalLossy) != 0);
  CHECK((r.flags & kCalLost) != 0);
}

TEST(lost_when_under_half_arrive_and_lossy_alone_from_half_up) {
  // Under half of what was offered measured the loss, not the air: the
  // point is lost (and lossy). From half up to nine tenths it is lossy and
  // still a measurement.
  CapCell c;
  c.offered = 1000;
  c.blocked_pct = 80;
  c.rssi_dbm = {-70, 0};
  c.have_rssi = {true, false};
  c.frames = {499, 0};
  c.air_bits = {499ull * 1435 * 8, 0};
  auto r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalLossy) != 0);
  CHECK((r.flags & kCalLost) != 0);
  c.frames = {500, 0};
  c.air_bits = {500ull * 1435 * 8, 0};
  r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & kCalLossy) != 0);
  CHECK((r.flags & kCalLost) == 0);
  c.frames = {950, 0};
  c.air_bits = {950ull * 1435 * 8, 0};
  r = analyze_capacity(c, 4, 20, 2000, CalThresholds{});
  CHECK((r.flags & (kCalLossy | kCalLost)) == 0);
}

namespace {

// A row from frames received out of 20 per cell, starting at idx -41.
std::vector<CalCell> row20(const std::vector<int>& got) {
  std::vector<CalCell> cells;
  for (size_t i = 0; i < got.size(); ++i)
    cells.push_back(cell(-41 + static_cast<int>(i), got[i] * 5));
  return cells;
}

// 40 MHz mcs5 of a field run, a room between the pair: every cell 19 or 20
// of 20 up to idx 31 except idx -12, which got 15; delivery ends at 32.
const std::vector<int> kFieldMcs5 = {
    20, 20, 20, 20, 20, 20, 19, 20, 19, 20, 20, 20, 20, 20, 20, 19, 20, 20, 20, 18, 20,  // -41..-21
    20, 20, 19, 20, 20, 20, 19, 20, 15, 20, 20, 19, 20, 20, 20, 19, 20, 20, 20, 20,      // -20..-1
    20, 20, 20, 20, 20, 20, 20, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19, 20, 20, 19,  // 0..20
    20, 20, 20, 20, 20, 20, 20, 19, 19, 20, 19, 10, 7,  8,  3,  1,  0,  0,  0,  0,       // 21..40
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       // 41..60
    0,  0,  0};                                                                          // 61..63

}  // namespace

// One cell that lost 5 of 20 frames used to end the run 44 indices under the
// real edge, and the table went to the drone 10 dB low with no flag. The
// three cells above it deliver, so power was not what failed it.
TEST(a_single_failing_cell_with_clean_cells_above_does_not_end_the_run) {
  const auto cells = row20(kFieldMcs5);
  CHECK(cells.size() == 105);
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 31);
  CHECK(w.floor_idx == -41);
  CHECK((w.flags & kCalBridged) != 0);
  CHECK((w.flags & kCalNoDip) == 0);

  // REVERT CHECK: with the exception off the old wall comes back.
  CalThresholds strict;
  strict.bridge_clean = 0;
  const auto old = analyze_rate(cells, strict);
  CHECK(old.wall == -13);
  CHECK((old.flags & kCalBridged) == 0);
}

// The real edge, as the same run's 20 MHz mcs6 measured it: 19 19 16 20 13
// 10 ... The cell after the first dip delivers, the one after that does not:
// that is the comb starting, and the wall stays under the first dip.
TEST(a_dip_at_the_edge_still_ends_the_run) {
  std::vector<int> got(66, 20);                    // -41..24
  got.insert(got.end(), {19, 16, 20, 13, 10, 12, 10});  // 25..31
  got.resize(105, 0);
  const auto w = analyze_rate(row20(got), CalThresholds{});
  CHECK(w.wall == 25);
  CHECK((w.flags & kCalBridged) == 0);
}

TEST(two_failing_cells_in_a_row_end_the_run) {
  std::vector<int> got(105, 20);
  got[30] = got[31] = 15;  // idx -11 and -10
  const auto w = analyze_rate(row20(got), CalThresholds{});
  CHECK(w.wall == -12);
  CHECK((w.flags & (kCalBridged | kCalNoDip)) == 0);
}

// Fewer than three cells left above the dip: nothing proves that more power
// delivers, so the run ends there and the row is not a no-dip row.
TEST(a_failing_cell_near_the_top_of_the_sweep_ends_the_run) {
  std::vector<int> got(105, 20);
  got[102] = 15;  // idx 61, two cells above it
  auto w = analyze_rate(row20(got), CalThresholds{});
  CHECK(w.wall == 60);
  CHECK((w.flags & (kCalBridged | kCalNoDip)) == 0);

  got[102] = 20;
  got[101] = 15;  // idx 60, three clean cells above it
  w = analyze_rate(row20(got), CalThresholds{});
  CHECK(w.wall == kRailRel);
  CHECK((w.flags & kCalBridged) != 0);
  CHECK((w.flags & kCalNoDip) != 0);
}

// The walls that field run wrote at 40 MHz before the exception: mcs5 sat 43
// indices under mcs6. A slower rate cannot have the lower wall.
TEST(a_wall_under_a_faster_rates_wall_is_marked) {
  const int walls[8] = {63, 63, 63, 63, 44, -13, 30, 25};
  RateWall w[8];
  for (int r = 0; r < 8; ++r) w[r].wall = walls[r];
  w[7].flags = kCalSaturated;
  mark_under_faster(w, 8);
  for (int r = 0; r < 8; ++r) CHECK(((w[r].flags & kCalUnderFaster) != 0) == (r == 5));
  CHECK((w[7].flags & kCalSaturated) != 0);

  // An undetermined rate is neither marked nor a reference; equal walls pass.
  RateWall u[4];
  u[0].wall = 20;
  u[1].wall = kNoWall;
  u[2].wall = 20;
  u[3].wall = kNoWall;
  mark_under_faster(u, 4);
  for (const auto& x : u) CHECK((x.flags & kCalUnderFaster) == 0);
  u[1].wall = 10;
  mark_under_faster(u, 4);
  CHECK((u[1].flags & kCalUnderFaster) != 0);
  CHECK((u[0].flags & kCalUnderFaster) == 0);
}

TEST(drift_flag_bit_is_not_reused) {
  CHECK(kCalTxBound == 64);
  CHECK(kCalLossy == 128);
  CHECK(kCalLost == 256);
  CHECK(kCalShortSpan == 512);
  CHECK(kCalBridged == 1024);
  CHECK(kCalUnderFaster == 2048);
}

MTEST_MAIN
