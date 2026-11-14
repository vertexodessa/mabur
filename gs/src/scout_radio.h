#pragma once
#include <cstdint>
#include <string>

namespace maburgs {

// mabur-owned subset of devourer's RxEnergy: the fields the ranker and the
// scan.log D/A lines use. Validity flags are the sensor-support record.
struct ScoutEnergy {
  bool fa_valid = false;
  uint32_t cca_ofdm = 0, fa_ofdm = 0;
  bool igi_valid = false;
  uint8_t igi = 0;
  bool nhm_valid = false;
  bool floor_valid = false;
  int8_t floor_dbm = 0;
};

// Cumulative decoded-frame counts on one card: canonical-SA (own) vs the
// rest. The scout diffs two snapshots to attribute a dwell's occupancy.
struct ScoutFrames {
  uint64_t own = 0, foreign = 0;
  uint64_t own_air_us = 0;
};

// One busy-airtime NHM window from devourer (IRtlRadio::ReadNhmBusy):
// `period` is the LAST arm's period on the card, whoever armed it.
struct NhmBusyRead {
  bool valid = false;
  uint8_t buckets[12] = {};
  uint16_t duration = 0;
  uint16_t period = 0;
};

// One armed busy-airtime CLM window from devourer (IRadio::GetChannelBusy
// after ArmChannelBusy): the share of 4 us ticks the baseband held CCA busy,
// receive-side only. The fallback for a card whose NHM window is not ported
// (Jaguar1: the RTL8812AU). `window_us` is the window the read covers;
// `own_tx` = the card transmitted inside it, which reads LOW (the receiver
// is deaf while its own PA is up).
struct ClmBusyRead {
  bool valid = false;
  uint8_t pct = 0;
  uint32_t window_us = 0;
  bool own_tx = false;
};

// "What does this card support" -- the scan.log C record. Static identity
// from devourer's AdapterCaps plus the validity flags of one energy read.
struct CardCaps {
  bool valid = false;
  std::string chip, gen;
  int tx_chains = 0, rx_chains = 0;
  uint8_t bw_mask = 0;
  uint16_t tune5g_lo = 0, tune5g_hi = 0;
  bool fast_retune = false;
  bool fa_ok = false, igi_ok = false, nhm_ok = false, floor_ok = false;
  // Per-frame PHY SNR is a real measurement independent of RSSI (Realtek).
  // False on the CPE510 relay: ath9k's "SNR" is RSSI above a calibrated
  // noise floor -- one measurement, not two -- so no cross-card SNR
  // comparison may include it.
  bool snr_ok = true;
};

// The one card's control plane the scout drives. RadioFrontend implements
// it; tests use a fake. All three are called from the scout thread only.
class ScoutRadio {
 public:
  virtual ~ScoutRadio() = default;
  // Retune to `ch` at the card's current width. The scout never changes a
  // card's width: the card is opened at the receiver width, and only the
  // core retunes it to another one (a live width flip).
  virtual bool retune(uint8_t ch) = 0;
  virtual ScoutEnergy read_energy(bool with_nhm) = 0;
  // Cheapest frame-free OFDM FA+CCA delta (devourer's GetRxEnergyScout):
  // only fa_valid/fa_ofdm/cca_ofdm are meaningful, everything else
  // (including nhm) is left invalid. For the ~10 ms in-flight dwell
  // (inflight_scout.h), which cannot afford GetRxEnergy's NHM window.
  virtual ScoutEnergy read_energy_scout() = 0;
  virtual ScoutFrames frames() const = 0;

  // Busy-airtime NHM window: arm at the start
  // of an observation, read at its end. Defaults = unsupported, so fakes and
  // non-Jaguar3 radios read as "no airtime evidence".
  virtual bool arm_nhm_busy(uint16_t period_4us) { (void)period_4us; return false; }
  virtual NhmBusyRead read_nhm_busy() { return {}; }

  // Busy-airtime CLM window, for a card that refuses arm_nhm_busy: the
  // window actually armed in us (0 = unsupported). Read only after an arm
  // that returned nonzero: an unarmed read on devourer takes a 2 ms NHM
  // sample and drains the FA/CCA delta the dwell is measuring.
  virtual uint32_t arm_clm_busy(uint32_t window_us) { (void)window_us; return 0; }
  virtual ClmBusyRead read_clm_busy() { return {}; }
};

}  // namespace maburgs
