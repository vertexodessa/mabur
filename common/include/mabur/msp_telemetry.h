#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mabur/mavlink_lite.h"
#include "mabur/msp_dp.h"

namespace mabur {

// Flight-controller telemetry over the MSP UART the DisplayPort OSD already
// uses (2026-09-18). MSP is request/response on that wire, so the drone asks
// Betaflight and INAV for attitude, altitude, GPS, battery, status and RC at
// fixed rates (msposd polled MSP_RC and MSP_ATTITUDE there the same way) and
// re-encodes each reply as the MAVLink message PixelPilot's OSD reads
// (app/mavlink/src/main/cpp/mavlink.cpp), batched into T_MAV payloads
// (rc_proto.h). Pure: the caller owns the serial port and the clock
// (drone/src/main.cpp, the mbr-msp thread).
//
// Reply layouts are the firmware sources' (INAV fc_msp.c, Betaflight msp.c);
// where the two differ the decoder branches on the MSP_FC_VARIANT id. Flight
// modes come out in ArduPlane custom_mode numbering because that is the only
// table PixelPilot names: INAV's own MAVLink telemetry uses the same
// mapping (telemetry/mavlink.c inavToArduPlaneMap), Betaflight's ANGLE/
// HORIZON/ACRO have no ArduPlane twin and land on STABILIZE/TRAINING/ACRO.
struct MspTelemetryCfg {
  // Poll periods, ms. next_request() hands out one request per call at most
  // (INAV drops requests that arrive back to back), the drone calls it every
  // ~20 ms, so the summed rates must stay well under 50 Hz: these sum to ~28.
  uint32_t attitude_ms = 100, altitude_ms = 200, gps_ms = 250, comp_gps_ms = 500, analog_ms = 500,
           battery_ms = 1000, status_ms = 500, rc_ms = 250, variant_ms = 1000, boxids_ms = 1000;
  uint32_t heartbeat_ms = 1000;
  uint32_t flush_bytes = 400, flush_ms = 100;  // T_MAV aggregation
  uint32_t fc_timeout_ms = 3000;               // no reply this long: no HEARTBEAT either
  // Requests still going out with nothing coming back for silent_after_ms:
  // a WARNING STATUSTEXT, repeated every silent_text_ms. Without it the OSD
  // holds the last mode and battery and reads as current (a Betaflight that
  // fell off the UART at a drone reboot showed FAILSAFE for 25 min, 2026-09-18).
  uint32_t silent_after_ms = 5000, silent_text_ms = 10000;
  uint8_t sysid = 1, compid = 1;
  // MAVLink 2 framing: what mabur's T_MAV tunnel carries (docs/mavlink.md);
  // maburd sets it. false = v1, the framing the unit tests' goldens use.
  bool mavlink2 = false;
};

class MspTelemetry {
 public:
  enum class Variant : uint8_t { Unknown, Betaflight, Inav, Other };

  struct State {
    Variant variant = Variant::Unknown;
    char variant_id[5] = {'-', '-', '-', '-', 0};
    uint64_t last_reply_ms = 0;
    bool have_reply = false;
    bool armed = false, failsafe = false;
    uint32_t arm_disable = 0;      // Betaflight armingDisableFlags (MSP_STATUS); 0 = may arm
    uint8_t arm_disable_count = 0; // its bit count; the top bit is ARMSWITCH
    uint32_t custom_mode = 4;      // PLANE_MODE_ACRO until a status reply says otherwise
    uint64_t box_mask = 0;         // flight-mode flags, one bit per active box
    std::vector<uint8_t> box_ids;  // permanent id per active box (MSP_BOXIDS); empty = identity 0..2
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    int32_t alt_cm = 0;
    int16_t vario_cms = 0;
    uint8_t fix = 0, sats = 0;     // fix: MAVLink GPS_FIX_TYPE (0 none, 2 2D, 3 3D)
    int32_t lat_e7 = 0, lon_e7 = 0, gps_alt_m = 0;
    uint16_t speed_cms = 0, course_ddeg = 0, hdop = 0;
    uint16_t home_dist_m = 0;
    int16_t home_dir_deg = 0;
    uint16_t vbat_cv = 0;          // 0.01 V
    int16_t amps_ca = 0;           // 0.01 A
    uint32_t mah = 0;
    uint16_t rssi_1023 = 0;
    uint8_t cells = 0;
    uint16_t rc[18] = {};
    uint8_t rc_count = 0;
  };

  struct Stats {
    uint32_t polls = 0, replies = 0, displayport = 0, unknown = 0, mav_frames = 0, payloads = 0,
             payload_bytes = 0, silent_texts = 0;
  };

  explicit MspTelemetry(const MspTelemetryCfg& cfg = {});

  // Raw bytes from the FC UART (replies and DisplayPort alike); parsed here.
  void on_serial_bytes(const uint8_t* p, size_t n, uint64_t now_ms);
  // One parsed message, for callers with their own parser.
  void on_msp(const MspMessage& m, uint64_t now_ms);
  // The next MSP request to write to the FC, empty when none is due.
  std::vector<uint8_t> next_request(uint64_t now_ms);
  // T_MAV payloads ready to send: whole MAVLink frames, each payload at most
  // MAV_MAX_PAYLOAD bytes; also the 1 Hz HEARTBEAT while the FC answers.
  std::vector<std::vector<uint8_t>> take_payloads(uint64_t now_ms);
  // The same packets one by one, as soon as they are queued, for a caller
  // that batches them itself (the drone's T_MAV tunnel uplink, which packs
  // whole packets into frames of at most rc::kMavMaxPayload). Also emits the
  // 1 Hz HEARTBEAT and the FC-silence text, like take_payloads.
  std::vector<std::vector<uint8_t>> take_frames(uint64_t now_ms);

  const State& state() const { return st_; }
  const Stats& stats() const { return stats_; }
  bool fc_alive(uint64_t now_ms) const;
  // When the last status reply (MSP_STATUS, MSP2_INAV_STATUS) was applied,
  // 0 = none yet: each one is a fresh arm-state report (state().armed).
  // Only those two stamp it: the MSP_BOXIDS reply also re-derives armed,
  // from a box mask that is still 0 before the first status, and must not
  // read as a DISARMED report (a maburd restart in flight would enter
  // low_power for one status period).
  uint64_t last_status_ms() const { return status_ms_; }
  // Bench simulation (maburd's /sim?arm=): -1 = the FC's own arm flag,
  // 0/1 = report disarmed/armed in the HEARTBEAT whatever the FC says. A
  // change re-emits the HEARTBEAT at once once the FC has answered.
  void set_forced_arm(int v, uint64_t now_ms);
  // True when the box with this permanent id is set in the last status.
  bool box_active(uint8_t permanent_id) const;

  static const char* variant_name(Variant v);
  // "ARM: RXLOSS THROTTLE" / "ARM OK" from Betaflight's armingDisableFlags.
  static std::string arming_disable_text(uint32_t flags, uint8_t count);

 private:
  enum Poll : uint8_t {
    kVariant, kBoxIds, kStatus, kInavStatus, kAttitude, kAltitude, kRawGps, kCompGps, kAnalog,
    kBattery, kRc, kPollCount
  };
  struct PollSlot {
    uint16_t cmd = 0;
    bool v2 = false, enabled = false;
    uint32_t period_ms = 0;
    uint64_t due_ms = 0;
  };

  void decode_variant(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_status_v1(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_inav_status(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_attitude(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_altitude(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_raw_gps(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_comp_gps(const std::vector<uint8_t>& p);
  void decode_analog(const std::vector<uint8_t>& p, uint64_t now_ms);
  void decode_battery_state(const std::vector<uint8_t>& p);
  void decode_rc(const std::vector<uint8_t>& p, uint64_t now_ms);
  void apply_mode_flags(uint64_t now_ms);
  uint32_t resolve_custom_mode() const;
  void emit_heartbeat(uint64_t now_ms);
  void emit_battery(uint64_t now_ms);
  void note_silence(uint64_t now_ms);
  void queue(std::vector<uint8_t>&& frame, uint64_t now_ms);

  MspTelemetryCfg cfg_;
  MspParser parser_;
  mav::Writer writer_;
  State st_;
  Stats stats_;
  PollSlot polls_[kPollCount];
  bool have_box_ids_ = false, armed_known_ = false;
  uint64_t last_heartbeat_ms_ = 0, battery_state_ms_ = 0;
  uint64_t first_poll_ms_ = 0, silent_text_ms_ = 0;  // silent_text_ms_: last "FC silent" line, 0 = none owed
  uint32_t polls_at_reply_ = 0;                       // stats_.polls when the last reply landed
  bool hb_armed_ = false;  // armed flag the last HEARTBEAT carried
  uint64_t status_ms_ = 0;
  int forced_arm_ = -1;
  bool have_battery_state_ = false;
  std::vector<std::vector<uint8_t>> pending_;
  size_t pending_bytes_ = 0;
  uint64_t pending_first_ms_ = 0;
};

}  // namespace mabur
