#pragma once
// A drone radio with a wide-TX floor edge, as CalSweep::CapacityCtl sees it:
// a blast at a floor at or above `edge` sends the full rate every second, one
// below it sends nothing (the chip holds every 40 MHz frame), and `marginal`
// (when set) passes a short dwell but loses one second in a longer one --
// the shape measured on one RTL8822E at 0x22: a worst second of 2130 against
// a 2484 median.
#include <cstdint>
#include <utility>
#include <vector>

#include "cal_sweep.h"

namespace mabur::test {

struct FakeFloorUnit : CalSweep::CapacityCtl {
  uint8_t edge = 0x24;
  uint8_t marginal = 0;  // 0 = none
  uint32_t full_fps = 2484;
  // Frames a blast's first second runs above the rest while the send queue
  // fills (0 = a flat rate): 2868 then 2682 on one RTL8822E.
  uint32_t first_second_extra_fps = 0;
  // Frames/s the receiver delivers during a blast (other traffic on the
  // channel), from the `heard_from` blast on.
  uint32_t heard_per_s = 0;
  size_t heard_from = 0;
  // Carrier-sense events a second the driver counts during a blast, frames
  // the receiver starts on and does not deliver: from the `seen_from` blast
  // on, and at `seen_floor` only when that is set.
  uint32_t seen_per_s = 0;
  size_t seen_from = 0;
  uint8_t seen_floor = 0;
  // Those events free a held transmitter (a floor under the edge then sends
  // the full rate), as 40 undelivered frames/s do on one RTL8822E.
  bool seen_frees_the_hold = false;
  // Frames/s a healthy floor loses to carrier sense while they are counted.
  uint32_t seen_defers_fps = 0;
  uint32_t marginal_worst_fps = 2130;
  // Configured floor, put back by restore_wide_floor().
  uint8_t flown = 0x35;
  uint8_t floor = 0x35;  // the live floor
  bool has_floor = true;
  std::vector<std::pair<uint8_t, uint8_t>> points;  // (mcs, bw)
  std::vector<uint8_t> floors_set;                  // every set_wide_floor value
  std::vector<std::pair<uint8_t, uint32_t>> blasts;  // (floor, dwell_ms)
  int restores = 0;

  bool set_point(uint8_t mcs, uint8_t bw) override {
    points.emplace_back(mcs, bw);
    return true;
  }
  bool set_wide_floor(uint8_t f) override {
    if (!has_floor) return false;
    floors_set.push_back(f);
    floor = f;
    return true;
  }
  bool restore_wide_floor() override {
    ++restores;
    return set_wide_floor(flown);
  }
  BlastResult blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) override {
    (void)frame;
    (void)len;
    blasts.emplace_back(floor, dwell_ms);
    BlastResult r;
    const uint32_t secs = dwell_ms / 1000;
    if (blasts.size() > heard_from) r.heard = heard_per_s * secs;
    const bool seen_now =
        seen_per_s != 0 && blasts.size() > seen_from && (seen_floor == 0 || floor == seen_floor);
    // A radio that has the count reports its window for every blast.
    if (seen_per_s != 0) r.seen_ms = dwell_ms;
    if (seen_now) r.seen = seen_per_s * secs;
    r.per_second.assign(secs, 0);
    if (floor >= edge || (seen_now && seen_frees_the_hold)) {
      r.per_second.assign(secs, full_fps - (seen_now ? seen_defers_fps : 0));
      if (secs > 0) r.per_second[0] += first_second_extra_fps;
    } else if (marginal != 0 && floor == marginal) {
      r.per_second.assign(secs, full_fps);
      // Holds through 4 s, slips in the fifth.
      if (secs > 4) r.per_second[4] = marginal_worst_fps;
    }
    for (uint32_t n : r.per_second) r.offered += n;
    return r;
  }
};

}  // namespace mabur::test
