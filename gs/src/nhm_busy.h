#pragma once
#include <cstdint>
#include <optional>

#include "NoiseFloorMath.h"   // devourer::nf::kNhmAbsThDbm
#include "scout_radio.h"

namespace maburgs {

// Busy-airtime share of one NHM window:
// the buckets whose LOWER edge is >= busy_dbm over all buckets. Bucket 0
// sits below th[0]; bucket i (1..11) starts at th[i-1].
inline bool busy_dbm_is_edge(int dbm) {
  for (int8_t e : devourer::nf::kNhmAbsThDbm) if (e == dbm) return true;
  return false;
}
inline std::optional<double> nhm_busy_pct(const NhmBusyRead& r, int busy_dbm) {
  if (!r.valid) return std::nullopt;
  int first = -1;
  for (int i = 0; i < 11; ++i)
    if (devourer::nf::kNhmAbsThDbm[i] == busy_dbm) { first = i + 1; break; }
  if (first < 0) return std::nullopt;
  uint32_t all = 0, above = 0;
  for (int i = 0; i < 12; ++i) {
    all += r.buckets[i];
    if (i >= first) above += r.buckets[i];
  }
  if (all == 0) return std::nullopt;   // ready but empty: no reading, not "0 % busy"
  return 100.0 * above / all;
}
inline uint16_t nhm_period_4us(int ms) {
  const long v = static_cast<long>(ms) * 250;
  return static_cast<uint16_t>(v < 1 ? 1 : v > 65535 ? 65535 : v);
}

// Which sensor one observation's busy window was armed on: the NHM share
// above busy_dbm where the card has it (Jaguar3), else CLM busy airtime
// over the same span (Jaguar1). Without the fallback an RTL8812AU GS read no
// busy at all: every pick kept the channel it started on and no hop fired.
enum class BusySensor : uint8_t { kNone, kNhm, kClm };
struct BusyArm {
  BusySensor sensor = BusySensor::kNone;
  uint16_t period_4us = 0;   // the NHM period asked for
  uint32_t clm_us = 0;       // the CLM window devourer armed
};
inline BusyArm arm_busy(ScoutRadio& r, uint16_t period_4us) {
  BusyArm a;
  a.period_4us = period_4us;
  if (r.arm_nhm_busy(period_4us)) {
    a.sensor = BusySensor::kNhm;
  } else if ((a.clm_us = r.arm_clm_busy(static_cast<uint32_t>(period_4us) * 4)) != 0) {
    a.sensor = BusySensor::kClm;
  }
  return a;
}
// A CLM read is this window's only if it spans the window armed.
inline std::optional<double> clm_busy_pct(const ClmBusyRead& c, const BusyArm& a) {
  if (!c.valid || c.window_us != a.clm_us) return std::nullopt;
  return static_cast<double>(c.pct);
}
// The busy % of the window `a` armed, read at the observation's end.
inline std::optional<double> read_busy(ScoutRadio& r, const BusyArm& a, int busy_dbm) {
  switch (a.sensor) {
    case BusySensor::kNhm: {
      const NhmBusyRead nb = r.read_nhm_busy();
      if (!nb.valid || nb.period != a.period_4us) return std::nullopt;
      return nhm_busy_pct(nb, busy_dbm);
    }
    case BusySensor::kClm:
      return clm_busy_pct(r.read_clm_busy(), a);
    case BusySensor::kNone:
      break;
  }
  return std::nullopt;
}

}  // namespace maburgs
