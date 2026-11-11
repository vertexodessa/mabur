#pragma once
// The wide-TX receive-gain floor sweep (docs/calibration.md "The floor
// sweep"): the one rule both ends share -- which floors are tried, how long,
// what passes, the margin the written value carries, and how the drone's
// SweepDone carries the result. Pure; no I/O.
//
// What is measured: a Jaguar3 card tuned to 40 MHz on a quiet channel can
// park its receiver where the secondary 20 MHz never reads idle, and its
// MAC then holds every 40 MHz frame (devourer DeviceConfig::Tuning::
// wide_tx_igi_floor). Which floor clears that is per unit. The drone
// saturates 40 MHz transmission at each candidate with the ground silent
// (frames the drone hears re-run its AGC and hide the hold) and counts
// what its TX path sends in every second of the dwell. A value passes when
// no second falls below kFloorPassPct of the reference rate, taken first at
// kFloorRef (receiver as deaf as the field allows, so the channel always
// reads idle).
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "mabur/cal_proto.h"

namespace mabur::cal {

// HT MCS 2 at 40 MHz, production-sized frames: the rate the stall was
// measured at (0 frames/s at floors 0-0x1e, the full ~2500-2800/s from
// 0x24 on one RTL8822E).
constexpr uint8_t kFloorMcs = 2;
// Candidates, ascending: 0x20 is the bottom of the receiver's normal DIG
// window on a DFS channel (a lower floor changes nothing there), 0x36 the
// first even step past devourer's default 0x35, the lowest value that has
// cleared every unit measured. Even steps: the measured edges sit on 0x22
// (marginal), 0x24 and 0x2c; one step of resolution under a 4-step margin
// costs at most one step of sensitivity.
constexpr uint8_t kFloorLo = 0x20;
constexpr uint8_t kFloorHi = 0x36;
constexpr uint8_t kFloorStep = 2;
constexpr size_t kFloorMaxCandidates = 12;
// The reference point's floor: the top of the 7-bit IGI field.
constexpr uint8_t kFloorRef = 0x7f;
constexpr uint8_t kFloorMax = 0x7f;
// Per point: settle, then a dwell of whole seconds. A unit that airs ~1500
// frames and only then wedges is caught by the second second of a 4 s
// dwell; a marginal value (0x22 above: one run's worst second 2130 of a
// 2484 median) has to hold twice as long again before it is believed.
constexpr uint16_t kFloorSettleMs = 300;
constexpr uint16_t kFloorDwellMs = 4000;
constexpr uint32_t kFloorRefDwellMs = 2000;
constexpr uint32_t kFloorConfirmMs = 8000;
// A candidate that passes its dwell and fails its confirmation lets the
// sweep go on upward; after this many failed confirmations it stops, which
// bounds the sweep (and the GS's silent window) at about 71 s.
constexpr int kFloorMaxConfirms = 2;
constexpr uint32_t kFloorPassPct = 95;
// What is written: the lowest confirmed value plus this many IGI steps
// (about 1 dB each of the drone's uplink sensitivity while 40 MHz flies).
constexpr int kFloorMarginSteps = 4;

// lo, lo + kFloorStep, ... up to hi, at most kFloorMaxCandidates; empty for
// a range outside [1, kFloorMax] or upside down.
inline std::vector<uint8_t> floor_candidates(int lo, int hi) {
  std::vector<uint8_t> out;
  if (lo < 1 || hi > kFloorMax || lo > hi) return out;
  for (int v = lo; v <= hi && out.size() < kFloorMaxCandidates; v += kFloorStep)
    out.push_back(static_cast<uint8_t>(v));
  return out;
}

// The reference rate: the median second of the reference dwell.
inline uint32_t floor_ref_rate(std::vector<uint32_t> per_second) {
  if (per_second.empty()) return 0;
  std::sort(per_second.begin(), per_second.end());
  return per_second[per_second.size() / 2];
}

// The dwell's worst second, 0 for a dwell that counted nothing.
inline uint32_t floor_min_second(const std::vector<uint32_t>& per_second) {
  if (per_second.empty()) return 0;
  return *std::min_element(per_second.begin(), per_second.end());
}

// Every second at kFloorPassPct of `ref` or above; never against a zero
// reference (a transmitter that sent nothing at kFloorRef measured nothing).
inline bool floor_seconds_pass(const std::vector<uint32_t>& per_second, uint32_t ref) {
  if (ref == 0 || per_second.empty()) return false;
  return static_cast<uint64_t>(floor_min_second(per_second)) * 100 >=
         static_cast<uint64_t>(ref) * kFloorPassPct;
}

inline uint8_t floor_with_margin(uint8_t found) {
  return static_cast<uint8_t>(std::min<int>(found + kFloorMarginSteps, kFloorMax));
}

// The floor sweep as the GS commands it: candidates kFloorLo..kFloorHi in
// idx_lo/idx_hi, the per-candidate dwell in dwell_ms, the settle in
// settle_ms, the one MCS in mcs_mask. Always 40 MHz.
inline SweepCmd make_floor_sweep(uint32_t nonce, uint8_t seq) {
  SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = Kind::Floor;
  c.bw = 40;
  c.frames_per_cell = 0;
  c.gap_us = 0;
  c.settle_ms = kFloorSettleMs;
  c.dwell_ms = kFloorDwellMs;
  c.idx_lo = static_cast<int8_t>(kFloorLo);
  c.idx_hi = static_cast<int8_t>(kFloorHi);
  c.mcs_mask = static_cast<uint8_t>(1u << kFloorMcs);
  return c;
}

// The longest the drone can take for `c`: the reference point, every
// candidate, and kFloorMaxConfirms confirmations, each with its settle.
inline uint32_t floor_sweep_max_ms(const SweepCmd& c) {
  const uint32_t n = static_cast<uint32_t>(floor_candidates(c.idx_lo, c.idx_hi).size());
  const uint32_t points = 1 + n + static_cast<uint32_t>(kFloorMaxConfirms);
  return points * c.settle_ms + kFloorRefDwellMs + n * c.dwell_ms +
         static_cast<uint32_t>(kFloorMaxConfirms) * kFloorConfirmMs;
}

// What a floor sweep found, as the drone's SweepDone carries it.
struct FloorReport {
  uint32_t ref_fps = 0;   // reference rate, frames/s
  uint8_t floor = 0;      // lowest confirmed value; 0 = none passed
  uint8_t tried = 0;      // candidates dwelt on, from the first
  uint8_t confirm_pct = 0;  // the last confirmation's worst second, % of ref
  // Per candidate tried, its worst second in frames/s (saturating at 0xFFFF).
  std::array<uint16_t, kFloorMaxCandidates> min_fps{};
};

// SweepDone for Kind::Floor: offered[0] the reference rate, offered[1] the
// lowest confirmed floor (0 = none), offered[2..7] the candidates' worst
// seconds two per word (the even candidate in the low half), blocked_pct[0]
// the candidates tried, blocked_pct[1] the last confirmation's worst second
// in percent of the reference.
inline SweepDone pack_floor_done(uint32_t nonce, uint8_t seq, const FloorReport& r) {
  SweepDone d;
  d.nonce = nonce;
  d.seq = seq;
  d.offered[0] = r.ref_fps;
  d.offered[1] = r.floor;
  for (size_t i = 0; i < kFloorMaxCandidates / 2; ++i)
    d.offered[2 + i] = static_cast<uint32_t>(r.min_fps[2 * i]) |
                       (static_cast<uint32_t>(r.min_fps[2 * i + 1]) << 16);
  d.blocked_pct[0] = r.tried;
  d.blocked_pct[1] = r.confirm_pct;
  return d;
}

inline FloorReport unpack_floor_done(const SweepDone& d) {
  FloorReport r;
  r.ref_fps = d.offered[0];
  r.floor = static_cast<uint8_t>(std::min<uint32_t>(d.offered[1], kFloorMax));
  for (size_t i = 0; i < kFloorMaxCandidates / 2; ++i) {
    r.min_fps[2 * i] = static_cast<uint16_t>(d.offered[2 + i] & 0xFFFF);
    r.min_fps[2 * i + 1] = static_cast<uint16_t>(d.offered[2 + i] >> 16);
  }
  r.tried = static_cast<uint8_t>(std::min<size_t>(d.blocked_pct[0], kFloorMaxCandidates));
  r.confirm_pct = d.blocked_pct[1];
  return r;
}

}  // namespace mabur::cal
