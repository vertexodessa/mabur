#pragma once
// The wide-TX receive-gain floor sweep (docs/calibration.md "The floor
// sweep"): the one rule both ends share -- which floors are tried, how long,
// what passes, which value is written for the edge found, and how the
// drone's SweepDone carries the result. Pure; no I/O.
//
// What is measured: a Jaguar3 card tuned to 40 MHz on a quiet channel can
// park its receiver where the secondary 20 MHz never reads idle, and its
// MAC then holds every 40 MHz frame (devourer DeviceConfig::Tuning::
// wide_tx_igi_floor). Which floor clears that is per unit. The drone
// saturates 40 MHz transmission at each candidate with the ground silent
// (frames the drone hears re-run its AGC and hide the hold) and counts
// what its TX path sends in every second of the dwell, and what its
// receiver hears: a channel with other traffic ends the sweep with no
// result (kFloorQuietMaxHeardPerS). A value passes when
// no second falls below kFloorPassPct of the reference rate, taken first at
// kFloorRef (a receiver deaf enough that the channel always reads idle) and
// judged the same way: the worst second of a dwell.
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
// The reference point's floor, and what a held blast is raised to. Far
// above every measured edge (0x22, 0x24, 0x2c), and far below the top of the
// field on purpose: devourer's energy-detect threshold follows the receive
// gain (IGI + 8, one biased byte), and at 0x7c and above an unbounded
// threshold stopped a saturated 40 MHz transmitter for good in 9 of 17
// starts on one RTL8822E. From a hold at 0x20 a raise to 0x50 brought the
// transmitter back to the full rate within a second in 4 of 4.
constexpr uint8_t kFloorRef = 0x50;
// The top of the 7-bit IGI field.
constexpr uint8_t kFloorMax = 0x7f;
// Per point: settle, then a dwell of whole seconds. A unit that airs ~1500
// frames and only then wedges is caught by the second second of a 4 s
// dwell; a marginal value (0x22 above: worst seconds of 1665, 2232 and 2343
// against a reference of 2682 in three sweeps) has to hold twice as long
// again before it is believed.
constexpr uint16_t kFloorSettleMs = 300;
constexpr uint16_t kFloorDwellMs = 4000;
// The reference dwells as long as a candidate and is read the same way. A
// blast's first second runs above the rest while the send queue fills (2859
// to 2871 frames against 2679 to 2697 in every later second on one
// RTL8822E); the larger second of a 2 s reference was that one, a rate no
// candidate's worst second reaches (2682 is 93.5 % of 2868).
constexpr uint32_t kFloorRefDwellMs = 4000;
constexpr uint32_t kFloorConfirmMs = 8000;
// A candidate that passes its dwell and fails its confirmation lets the
// sweep go on upward; after this many failed confirmations it stops, which
// bounds the sweep (and the GS's silent window) at about 73 s.
constexpr int kFloorMaxConfirms = 2;
constexpr uint32_t kFloorPassPct = 95;
// The walk has an answer on a quiet channel only, and other traffic spoils
// it two ways (one RTL8822E, edge 0x24). Carrier sense defers to it, so a
// healthy floor loses rate (0x24: 96 % with 20 frames/s of 1.8 ms on the
// channel, 86 % with 80) and the walk reads the edge too high or finds
// none. And every frame the receiver starts on frees a held transmitter
// for a while, most of all a frame it then does not deliver: 0x20 sends
// nothing in silence, 90 % of the full rate under 20 such frames/s and
// 96 % under 40, where it passes the bar in 6 of 8 dwells and the walk
// would report an edge under the real one.
//
// The GS is silent for the sweep, so what the receiver takes during a
// point is other traffic. Two counts of it, one bound:
//
// - Frames delivered to the receive callback, as a mean over the candidate
//   points flown: above the bound the walk ends at once. The reference
//   point is not in the mean; its receiver is deaf.
// - Receiver events (floor_heard_rate): the larger of those frames and the
//   driver's carrier-sense count (BlastResult::seen), which has a frame
//   start whether or not the frame is delivered. They judge the stretch
//   that decides the report only: the dwell and confirmation of the value
//   about to be reported, or of the last candidate when none passed. Above
//   the bound the sweep ends with no result. A healthy floor in silence
//   counts 0 (0x22 to 0x50, 140 s in all), and from 20 a second a held
//   floor starts to pass.
//
// The carrier-sense count is no measure of the channel under the unit's
// edge, which is why it is kept out of the walk's mean: 0x20 in silence
// reads either 0 or a steady 19 a second for many seconds on end (7 of 10
// runs of 20 s), sending 26 to 60 % of the full rate while it does. Such a
// candidate fails the bar either way.
//
// At the bound a healthy floor still reads 99 % or more. Neither count
// sees traffic on the secondary 20 MHz channel alone, which takes the same
// rate from a healthy floor.
constexpr uint32_t kFloorQuietMaxHeardPerS = 5;
// The unit's edge plus this many IGI steps (about 1 dB each of the drone's
// uplink sensitivity while 40 MHz flies) is what the unit needs.
constexpr int kFloorMarginSteps = 4;
// devourer's default floor, what the bundle's drone config ships.
constexpr uint8_t kFloorDefault = 0x35;

// lo, lo + kFloorStep, ... up to hi, at most kFloorMaxCandidates; empty for
// a range outside [1, kFloorMax] or upside down.
inline std::vector<uint8_t> floor_candidates(int lo, int hi) {
  std::vector<uint8_t> out;
  if (lo < 1 || hi > kFloorMax || lo > hi) return out;
  for (int v = lo; v <= hi && out.size() < kFloorMaxCandidates; v += kFloorStep)
    out.push_back(static_cast<uint8_t>(v));
  return out;
}

// The dwell's worst second, 0 for a dwell that counted nothing.
inline uint32_t floor_min_second(const std::vector<uint32_t>& per_second) {
  if (per_second.empty()) return 0;
  return *std::min_element(per_second.begin(), per_second.end());
}

// Receiver events a second over one stretch, saturating at 0xFF (the
// report's byte): the larger of the frames delivered over `ms` of dwell and
// the carrier-sense events over the `seen_ms` their counter windows cover
// (0 = no reading).
inline uint8_t floor_heard_rate(uint64_t heard, uint64_t ms, uint64_t seen, uint64_t seen_ms) {
  const uint64_t a = ms ? heard * 1000 / ms : 0;
  const uint64_t b = seen_ms ? seen * 1000 / seen_ms : 0;
  return static_cast<uint8_t>(std::min<uint64_t>(0xFF, std::max(a, b)));
}

// Every second at kFloorPassPct of `ref` or above; never against a zero
// reference (a transmitter that sent nothing at kFloorRef measured nothing).
inline bool floor_seconds_pass(const std::vector<uint32_t>& per_second, uint32_t ref) {
  if (ref == 0 || per_second.empty()) return false;
  return static_cast<uint64_t>(floor_min_second(per_second)) * 100 >=
         static_cast<uint64_t>(ref) * kFloorPassPct;
}

// The value written for a unit whose lowest confirmed floor is `edge`:
// edge + kFloorMarginSteps, never under kFloorDefault, at most kFloorMax.
// Calibration raises the floor for a unit that needs more than the default
// and otherwise leaves the default; it never lowers it. On one RTL8822E at
// 40 MHz (edge 0x24) floors 0x28 and 0x35 gave the same video (capture to
// complete AU p50 24 ms, p90 27-28 ms) but the drone took 62 % and 73 % of
// the GS's RCFs at 0x28 against 93 % and 98 % at 0x35: the lower floor
// hears the close-range uplink worse. What a lower floor buys at long
// range is unmeasured, so the policy stays here, in one place.
inline uint8_t floor_to_write(uint8_t edge) {
  const int need = std::min<int>(edge + kFloorMarginSteps, kFloorMax);
  return static_cast<uint8_t>(std::max<int>(need, kFloorDefault));
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
  // Frames/s the drone's receiver delivered over the candidate points
  // flown, saturating at 0xFF; with `noisy`, the rate that ended the walk,
  // frames or receiver events (floor_heard_rate).
  uint8_t heard_fps = 0;
  // The channel was not quiet (kFloorQuietMaxHeardPerS): the walk has no
  // answer and `floor` is 0.
  bool noisy = false;
  // Per candidate tried, its worst second in frames/s (saturating at 0xFFFF).
  std::array<uint16_t, kFloorMaxCandidates> min_fps{};
};

// SweepDone for Kind::Floor: offered[0] the reference rate, offered[1] the
// lowest confirmed floor (0 = none), offered[2..7] the candidates' worst
// seconds two per word (the even candidate in the low half), blocked_pct[0]
// the candidates tried, blocked_pct[1] the last confirmation's worst second
// in percent of the reference, blocked_pct[2] the frames/s heard,
// blocked_pct[3] 1 when the channel was not quiet.
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
  d.blocked_pct[2] = r.heard_fps;
  d.blocked_pct[3] = r.noisy ? 1 : 0;
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
  r.heard_fps = d.blocked_pct[2];
  r.noisy = d.blocked_pct[3] != 0;
  return r;
}

}  // namespace mabur::cal
