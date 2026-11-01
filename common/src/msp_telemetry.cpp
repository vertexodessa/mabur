#include "mabur/msp_telemetry.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "mabur/rc_proto.h"  // MAV_MAX_PAYLOAD

namespace mabur {
namespace {

// MSP command ids (msp_protocol.h, shared by Betaflight and INAV).
constexpr uint16_t MSP_FC_VARIANT = 2, MSP_STATUS = 101, MSP_RC = 105, MSP_RAW_GPS = 106,
                   MSP_COMP_GPS = 107, MSP_ATTITUDE = 108, MSP_ALTITUDE = 109, MSP_ANALOG = 110,
                   MSP_BOXIDS = 119, MSP_BATTERY_STATE = 130;
constexpr uint16_t MSP2_INAV_STATUS = 0x2000;

// Permanent box ids (identical in both firmwares' box tables where listed).
constexpr uint8_t BOX_ARM = 0, BOX_ANGLE = 1, BOX_HORIZON = 2, BOX_NAV_ALTHOLD = 3, BOX_NAV_RTH = 10,
                  BOX_NAV_POSHOLD = 11, BOX_MANUAL_OR_PASSTHRU = 12, BOX_FAILSAFE = 27,
                  BOX_NAV_WP = 28, BOX_NAV_LAUNCH = 36, BOX_NAV_COURSE_HOLD = 45,
                  BOX_GPS_RESCUE = 46, BOX_NAV_CRUISE = 53, BOX_ANGLE_HOLD = 64;

// ArduPlane custom_mode numbers PixelPilot names (APM_PLANE_MODE).
constexpr uint32_t PLANE_MANUAL = 0, PLANE_STABILIZE = 2, PLANE_TRAINING = 3, PLANE_ACRO = 4,
                   PLANE_FBWA = 5, PLANE_FBWB = 6, PLANE_CRUISE = 7, PLANE_AUTO = 10,
                   PLANE_RTL = 11, PLANE_LOITER = 12, PLANE_TAKEOFF = 13;
// Not an ArduPlane mode: PixelPilot shows "Unknown" for it, the STATUSTEXT
// sent alongside says why.
constexpr uint32_t MODE_FAILSAFE = 200;

constexpr uint32_t INAV_ARMED = 1u << 2;  // runtime_config.h armingFlag_e

uint16_t u16(const std::vector<uint8_t>& p, size_t off) {
  return static_cast<uint16_t>(p[off] | (p[off + 1] << 8));
}
int16_t i16(const std::vector<uint8_t>& p, size_t off) { return static_cast<int16_t>(u16(p, off)); }
uint32_t u32(const std::vector<uint8_t>& p, size_t off) {
  return static_cast<uint32_t>(u16(p, off)) | (static_cast<uint32_t>(u16(p, off + 2)) << 16);
}
int32_t i32(const std::vector<uint8_t>& p, size_t off) { return static_cast<int32_t>(u32(p, off)); }

constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

}  // namespace

MspTelemetry::MspTelemetry(const MspTelemetryCfg& cfg)
    : cfg_(cfg), writer_(cfg.sysid, cfg.compid, cfg.mavlink2) {
  auto set = [&](Poll k, uint16_t cmd, bool v2, uint32_t period, bool enabled) {
    polls_[k].cmd = cmd;
    polls_[k].v2 = v2;
    polls_[k].period_ms = period;
    polls_[k].enabled = enabled;
    polls_[k].due_ms = 0;
  };
  set(kVariant, MSP_FC_VARIANT, false, cfg.variant_ms, true);
  set(kBoxIds, MSP_BOXIDS, false, cfg.boxids_ms, false);  // once the variant is known
  set(kStatus, MSP_STATUS, false, cfg.status_ms, true);
  set(kInavStatus, MSP2_INAV_STATUS, true, cfg.status_ms, false);  // INAV: replaces kStatus
  set(kAttitude, MSP_ATTITUDE, false, cfg.attitude_ms, true);
  set(kAltitude, MSP_ALTITUDE, false, cfg.altitude_ms, true);
  set(kRawGps, MSP_RAW_GPS, false, cfg.gps_ms, true);
  set(kCompGps, MSP_COMP_GPS, false, cfg.comp_gps_ms, true);
  set(kAnalog, MSP_ANALOG, false, cfg.analog_ms, true);
  set(kBattery, MSP_BATTERY_STATE, false, cfg.battery_ms, true);
  set(kRc, MSP_RC, false, cfg.rc_ms, true);
  // Stagger the first round so a fresh link does not fire every request in
  // the same tick.
  for (int k = 0; k < kPollCount; ++k) polls_[k].due_ms = static_cast<uint64_t>(k) * 20;
}

const char* MspTelemetry::variant_name(Variant v) {
  switch (v) {
    case Variant::Betaflight: return "betaflight";
    case Variant::Inav: return "inav";
    case Variant::Other: return "other";
    default: return "unknown";
  }
}

bool MspTelemetry::fc_alive(uint64_t now_ms) const {
  return st_.have_reply && now_ms - st_.last_reply_ms <= cfg_.fc_timeout_ms;
}

bool MspTelemetry::box_active(uint8_t permanent_id) const {
  if (st_.box_ids.empty()) {
    // ARM, ANGLE, HORIZON head both firmwares' active-box lists in that order.
    return permanent_id <= BOX_HORIZON && ((st_.box_mask >> permanent_id) & 1u);
  }
  for (size_t i = 0; i < st_.box_ids.size() && i < 64; ++i)
    if (st_.box_ids[i] == permanent_id) return (st_.box_mask >> i) & 1u;
  return false;
}

void MspTelemetry::on_serial_bytes(const uint8_t* p, size_t n, uint64_t now_ms) {
  for (const auto& m : parser_.feed(p, n)) on_msp(m, now_ms);
}

std::vector<uint8_t> MspTelemetry::next_request(uint64_t now_ms) {
  int pick = -1;
  for (int k = 0; k < kPollCount; ++k) {
    const PollSlot& s = polls_[k];
    if (!s.enabled || s.due_ms > now_ms) continue;
    if (pick < 0 || s.due_ms < polls_[pick].due_ms) pick = k;
  }
  std::vector<uint8_t> req;
  if (pick < 0) return req;
  PollSlot& s = polls_[pick];
  // Keep the slot on its own grid so a one-tick wait behind another slot does
  // not stretch its period for good; a stall longer than a period re-anchors
  // at now instead of being followed by a catch-up burst.
  s.due_ms += s.period_ms;
  if (s.due_ms <= now_ms) s.due_ms = now_ms + s.period_ms;
  if (s.v2) msp_append_message_v2(req, s.cmd, nullptr, 0);
  else msp_append_message(req, static_cast<uint8_t>(s.cmd), nullptr, 0);
  if (stats_.polls == 0) first_poll_ms_ = now_ms;
  ++stats_.polls;
  return req;
}

void MspTelemetry::on_msp(const MspMessage& m, uint64_t now_ms) {
  const auto& p = m.payload;
  const uint16_t cmd = m.cmd16;
  if (!m.v2 && cmd == MSP_CMD_DISPLAYPORT) {
    ++stats_.displayport;
    return;
  }
  bool known = true;
  if (m.v2) {
    if (cmd == MSP2_INAV_STATUS) decode_inav_status(p, now_ms);
    else known = false;
  } else {
    switch (cmd) {
      case MSP_FC_VARIANT: decode_variant(p, now_ms); break;
      case MSP_BOXIDS:
        st_.box_ids.assign(p.begin(), p.end());
        have_box_ids_ = true;
        polls_[kBoxIds].enabled = false;
        apply_mode_flags(now_ms);
        break;
      case MSP_STATUS: decode_status_v1(p, now_ms); break;
      case MSP_ATTITUDE: decode_attitude(p, now_ms); break;
      case MSP_ALTITUDE: decode_altitude(p, now_ms); break;
      case MSP_RAW_GPS: decode_raw_gps(p, now_ms); break;
      case MSP_COMP_GPS: decode_comp_gps(p); break;
      case MSP_ANALOG: decode_analog(p, now_ms); break;
      case MSP_BATTERY_STATE:
        decode_battery_state(p);
        battery_state_ms_ = now_ms;
        have_battery_state_ = true;
        break;
      case MSP_RC: decode_rc(p, now_ms); break;
      default: known = false; break;
    }
  }
  if (!known) {
    ++stats_.unknown;
    return;
  }
  if (silent_text_ms_) {
    // The silence was announced; say once that it is over.
    const uint64_t since = st_.have_reply ? st_.last_reply_ms : first_poll_ms_;
    char text[50];
    std::snprintf(text, sizeof(text), "FC back after %u s",
                  static_cast<unsigned>((now_ms - since) / 1000));
    std::vector<uint8_t> f;
    writer_.statustext(f, mav::kSeverityInfo, text);
    queue(std::move(f), now_ms);
    silent_text_ms_ = 0;
  }
  ++stats_.replies;
  polls_at_reply_ = stats_.polls;
  st_.last_reply_ms = now_ms;
  st_.have_reply = true;
}

void MspTelemetry::note_silence(uint64_t now_ms) {
  // Nothing owed while the FC answers or before the first request (a booting
  // FC gets silent_after_ms of grace either way).
  if (stats_.polls == 0 || fc_alive(now_ms)) return;
  const uint64_t since = st_.have_reply ? st_.last_reply_ms : first_poll_ms_;
  if (now_ms - since < cfg_.silent_after_ms) return;
  if (silent_text_ms_ && now_ms - silent_text_ms_ < cfg_.silent_text_ms) return;
  char text[50];
  std::snprintf(text, sizeof(text), "FC silent %u s, %u polls unanswered",
                static_cast<unsigned>((now_ms - since) / 1000), stats_.polls - polls_at_reply_);
  std::vector<uint8_t> f;
  writer_.statustext(f, mav::kSeverityWarning, text);
  queue(std::move(f), now_ms);
  silent_text_ms_ = now_ms;
  ++stats_.silent_texts;
}

void MspTelemetry::decode_variant(const std::vector<uint8_t>& p, uint64_t now_ms) {
  if (p.size() < 4) return;
  char id[5] = {static_cast<char>(p[0]), static_cast<char>(p[1]), static_cast<char>(p[2]),
                static_cast<char>(p[3]), 0};
  const bool changed = std::memcmp(id, st_.variant_id, 4) != 0;
  std::memcpy(st_.variant_id, id, 5);
  Variant v = Variant::Other;
  if (std::memcmp(id, "BTFL", 4) == 0) v = Variant::Betaflight;
  else if (std::memcmp(id, "INAV", 4) == 0) v = Variant::Inav;
  st_.variant = v;
  // INAV's MSP_STATUS carries only the first 32 box bits and no arming flag;
  // its MSP2_INAV_STATUS carries both.
  polls_[kInavStatus].enabled = v == Variant::Inav;
  polls_[kStatus].enabled = v != Variant::Inav;
  if (!have_box_ids_) polls_[kBoxIds].enabled = true;
  if (changed) {
    char text[50];
    std::snprintf(text, sizeof(text), "FC %s via MSP", id);
    std::vector<uint8_t> f;
    writer_.statustext(f, mav::kSeverityInfo, text);
    queue(std::move(f), now_ms);
  }
}

void MspTelemetry::decode_status_v1(const std::vector<uint8_t>& p, uint64_t now_ms) {
  // cycleTime u16, i2cErrors u16, sensors u16, flightModeFlags u32, profile u8
  // (both firmwares); Betaflight then load u16, gyroCycle u16, a byte count
  // and that many more flag bytes (msp.c MSP_STATUS).
  if (p.size() < 10) return;
  uint64_t mask = u32(p, 6);
  if (st_.variant == Variant::Betaflight && p.size() >= 16) {
    const size_t extra = p[15];
    for (size_t i = 0; i < extra && i < 4 && 16 + i < p.size(); ++i)
      mask |= static_cast<uint64_t>(p[16 + i]) << (32 + 8 * i);
    // Then armingDisableFlagsCount u8 and armingDisableFlags u32: the reason
    // the FC refuses to arm, otherwise only shown by its own OSD.
    const size_t at = 16 + extra;
    if (p.size() >= at + 5) {
      const uint8_t count = p[at];
      const uint32_t flags = u32(p, at + 1);
      st_.arm_disable_count = count;
      if (flags != st_.arm_disable) {  // a flag set change, so a clean boot stays silent
        st_.arm_disable = flags;
        std::vector<uint8_t> f;
        writer_.statustext(f, flags ? mav::kSeverityWarning : mav::kSeverityInfo,
                           arming_disable_text(flags, count).c_str());
        queue(std::move(f), now_ms);
      }
    }
  }
  st_.box_mask = mask;
  armed_known_ = false;  // box-derived below
  apply_mode_flags(now_ms);
  status_ms_ = now_ms;  // a status reply, not the BOXIDS remap: box_mask is real
}

std::string MspTelemetry::arming_disable_text(uint32_t flags, uint8_t count) {
  // Betaflight runtime_config.h armingDisableFlags_e, bit order stable since
  // 4.1; whatever the firmware appends later shows as BITn, and the last bit
  // of the reported count is always ARM_SWITCH.
  static const char* const kNames[] = {
      "NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "RUNAWAY", "CRASH", "THROTTLE",
      "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI", "CMS", "BST", "MSP", "PARALYZE",
      "GPS", "RESCUE_SW", "RPMFILTER", "REBOOT_REQD", "DSHOT_BBANG", "ACC_CALIB", "MOTOR_PROTO"};
  constexpr size_t kNamed = sizeof(kNames) / sizeof(kNames[0]);
  if (!flags) return "ARM OK";
  std::string s = "ARM:";
  for (unsigned bit = 0; bit < 32; ++bit) {
    if (!((flags >> bit) & 1u)) continue;
    s += ' ';
    if (count && bit == static_cast<unsigned>(count - 1)) s += "ARMSWITCH";
    else if (bit < kNamed) s += kNames[bit];
    else s += "BIT" + std::to_string(bit);
  }
  return s;
}

void MspTelemetry::decode_inav_status(const std::vector<uint8_t>& p, uint64_t now_ms) {
  // cycleTime u16, i2cErrors u16, sensors u16, load u16, profiles u8,
  // armingFlags u32, boxBitmask_t (8 bytes for INAV 8's 60 boxes), mixerProfile u8.
  if (p.size() < 14) return;
  const uint32_t arming = u32(p, 9);
  uint64_t mask = 0;
  const size_t nmask = p.size() > 14 ? p.size() - 14 : 0;  // minus the trailing mixer byte
  for (size_t i = 0; i < nmask && i < 8; ++i) mask |= static_cast<uint64_t>(p[13 + i]) << (8 * i);
  st_.box_mask = mask;
  st_.armed = (arming & INAV_ARMED) != 0;
  armed_known_ = true;
  apply_mode_flags(now_ms);
  status_ms_ = now_ms;
}

uint32_t MspTelemetry::resolve_custom_mode() const {
  if (st_.variant == Variant::Inav) {
    // runtime_config.c getFlightModeForTelemetry priority, then
    // telemetry/mavlink.c inavToArduPlaneMap.
    if (box_active(BOX_FAILSAFE)) return box_active(BOX_NAV_RTH) ? PLANE_RTL : MODE_FAILSAFE;
    if (box_active(BOX_MANUAL_OR_PASSTHRU)) return PLANE_MANUAL;
    if (box_active(BOX_NAV_LAUNCH)) return PLANE_TAKEOFF;
    if (box_active(BOX_NAV_RTH)) return PLANE_RTL;
    if (box_active(BOX_NAV_POSHOLD)) return PLANE_LOITER;
    if (box_active(BOX_NAV_COURSE_HOLD) || box_active(BOX_NAV_CRUISE)) return PLANE_CRUISE;
    if (box_active(BOX_NAV_WP)) return PLANE_AUTO;
    if (box_active(BOX_NAV_ALTHOLD)) return PLANE_FBWB;
    if (box_active(BOX_ANGLE)) return PLANE_FBWA;
    if (box_active(BOX_HORIZON) || box_active(BOX_ANGLE_HOLD)) return PLANE_STABILIZE;
    return PLANE_ACRO;
  }
  // Betaflight (and anything else): no plane modes to mirror.
  if (box_active(BOX_FAILSAFE)) return box_active(BOX_GPS_RESCUE) ? PLANE_RTL : MODE_FAILSAFE;
  if (box_active(BOX_GPS_RESCUE)) return PLANE_RTL;
  if (box_active(BOX_MANUAL_OR_PASSTHRU)) return PLANE_MANUAL;
  if (box_active(BOX_ANGLE)) return PLANE_STABILIZE;
  if (box_active(BOX_HORIZON)) return PLANE_TRAINING;
  return PLANE_ACRO;
}

void MspTelemetry::apply_mode_flags(uint64_t now_ms) {
  if (!armed_known_) st_.armed = box_active(BOX_ARM);
  const bool failsafe = box_active(BOX_FAILSAFE);
  const uint32_t mode = resolve_custom_mode();
  const bool changed = mode != st_.custom_mode || failsafe != st_.failsafe || st_.armed != hb_armed_;
  if (failsafe && !st_.failsafe) {
    std::vector<uint8_t> f;
    writer_.statustext(f, mav::kSeverityCritical, "FAILSAFE");
    queue(std::move(f), now_ms);
  }
  st_.custom_mode = mode;
  st_.failsafe = failsafe;
  // A mode or arming change goes out at once rather than at the next 1 Hz beat.
  if (changed || last_heartbeat_ms_ == 0) emit_heartbeat(now_ms);
}

void MspTelemetry::emit_heartbeat(uint64_t now_ms) {
  std::vector<uint8_t> f;
  const uint8_t type = st_.variant == Variant::Inav ? mav::kTypeFixedWing : mav::kTypeQuadrotor;
  const uint8_t base = static_cast<uint8_t>(mav::kModeFlagCustomModeEnabled |
                                            (st_.armed ? mav::kModeFlagSafetyArmed : 0));
  const uint8_t status = st_.failsafe ? mav::kStateCritical
                                      : (st_.armed ? mav::kStateActive : mav::kStateStandby);
  writer_.heartbeat(f, st_.custom_mode, type, mav::kAutopilotArdupilotmega, base, status);
  queue(std::move(f), now_ms);
  last_heartbeat_ms_ = now_ms;
  hb_armed_ = st_.armed;
}

void MspTelemetry::decode_attitude(const std::vector<uint8_t>& p, uint64_t now_ms) {
  if (p.size() < 6) return;
  st_.roll_deg = i16(p, 0) / 10.0f;
  st_.pitch_deg = i16(p, 2) / 10.0f;
  st_.yaw_deg = static_cast<float>(i16(p, 4));
  std::vector<uint8_t> f;
  writer_.attitude(f, static_cast<uint32_t>(now_ms), st_.roll_deg * kDegToRad,
                   st_.pitch_deg * kDegToRad, st_.yaw_deg * kDegToRad);
  queue(std::move(f), now_ms);
}

void MspTelemetry::decode_altitude(const std::vector<uint8_t>& p, uint64_t now_ms) {
  if (p.size() < 6) return;
  st_.alt_cm = i32(p, 0);
  st_.vario_cms = i16(p, 4);
  // Throttle from the mapped RC data: index 3 is THROTTLE in both firmwares'
  // rcData (channel mapping already applied by the FC).
  uint16_t thr = 0;
  if (st_.rc_count > 3) {
    const int t = (static_cast<int>(st_.rc[3]) - 1000) / 10;
    thr = static_cast<uint16_t>(t < 0 ? 0 : (t > 100 ? 100 : t));
  }
  int16_t hdg = static_cast<int16_t>(std::lround(st_.yaw_deg));
  std::vector<uint8_t> f;
  writer_.vfr_hud(f, 0.0f, st_.speed_cms / 100.0f, st_.alt_cm / 100.0f, st_.vario_cms / 100.0f, hdg,
                  thr);
  // The position rides the altitude cadence: PixelPilot reads its altitude
  // and heading from this message, and the fix itself is at most one GPS
  // poll old. Heading for the home arrow is the compass yaw, as INAV's own
  // telemetry sends.
  float yaw = st_.yaw_deg;
  while (yaw < 0) yaw += 360.0f;
  while (yaw >= 360.0f) yaw -= 360.0f;
  writer_.global_position_int(f, static_cast<uint32_t>(now_ms), st_.lat_e7, st_.lon_e7,
                              st_.alt_cm * 10, st_.alt_cm * 10, 0, 0, 0,
                              static_cast<uint16_t>(std::lround(yaw * 100.0f)));
  queue(std::move(f), now_ms);
}

void MspTelemetry::decode_raw_gps(const std::vector<uint8_t>& p, uint64_t now_ms) {
  // fix u8, numSat u8, lat i32, lon i32, alt u16 (m), speed u16 (cm/s),
  // course u16 (0.1 deg), then hdop (INAV) / pdop (Betaflight) u16.
  if (p.size() < 16) return;
  const uint8_t fix = p[0];
  // Betaflight sends STATE(GPS_FIX) (0/1), INAV its fixType (0 none, 1 2D, 2 3D).
  if (st_.variant == Variant::Inav) st_.fix = fix == 0 ? 0 : (fix == 1 ? 2 : 3);
  else st_.fix = fix ? 3 : 0;
  st_.sats = p[1];
  st_.lat_e7 = i32(p, 2);
  st_.lon_e7 = i32(p, 6);
  st_.gps_alt_m = i16(p, 10);
  st_.speed_cms = u16(p, 12);
  st_.course_ddeg = u16(p, 14);
  st_.hdop = p.size() >= 18 ? u16(p, 16) : 0;
  std::vector<uint8_t> f;
  writer_.gps_raw_int(f, now_ms * 1000ull, st_.lat_e7, st_.lon_e7, st_.gps_alt_m * 1000,
                      st_.hdop ? st_.hdop : UINT16_MAX, UINT16_MAX, st_.speed_cms,
                      static_cast<uint16_t>(st_.course_ddeg * 10u), st_.fix, st_.sats);
  queue(std::move(f), now_ms);
}

void MspTelemetry::decode_comp_gps(const std::vector<uint8_t>& p) {
  if (p.size() < 4) return;
  st_.home_dist_m = u16(p, 0);
  st_.home_dir_deg = i16(p, 2);
}

void MspTelemetry::decode_analog(const std::vector<uint8_t>& p, uint64_t now_ms) {
  // vbat u8 (0.1 V), mAh u16, rssi u16 (0..1023), amps i16 (0.01 A); Betaflight
  // appends vbat u16 (0.01 V). INAV's 0.01 V value comes from MSP_BATTERY_STATE.
  if (p.size() < 7) return;
  const uint16_t coarse = static_cast<uint16_t>(p[0]) * 10;
  const bool fine_recent = have_battery_state_ && now_ms - battery_state_ms_ <= 2500;
  if (p.size() >= 9) st_.vbat_cv = u16(p, 7);
  else if (!fine_recent) st_.vbat_cv = coarse;
  st_.mah = u16(p, 1);
  st_.rssi_1023 = u16(p, 3);
  st_.amps_ca = i16(p, 5);
  emit_battery(now_ms);
}

void MspTelemetry::decode_battery_state(const std::vector<uint8_t>& p) {
  // cells u8, capacity u16, vbat u8 (0.1 V), mAh u16, amps i16, state u8, vbat u16 (0.01 V).
  if (p.size() < 8) return;
  st_.cells = p[0];
  st_.mah = u16(p, 4);
  st_.amps_ca = i16(p, 6);
  st_.vbat_cv = p.size() >= 11 ? u16(p, 9) : static_cast<uint16_t>(p[3]) * 10;
}

void MspTelemetry::emit_battery(uint64_t now_ms) {
  std::vector<uint8_t> f;
  writer_.sys_status(f, static_cast<uint16_t>(st_.vbat_cv * 10u), st_.amps_ca, -1);
  writer_.battery_status(f, static_cast<int32_t>(st_.mah), st_.amps_ca,
                         static_cast<uint16_t>(st_.vbat_cv * 10u), st_.cells, -1);
  queue(std::move(f), now_ms);
}

void MspTelemetry::decode_rc(const std::vector<uint8_t>& p, uint64_t now_ms) {
  const size_t n = p.size() / 2;
  st_.rc_count = static_cast<uint8_t>(n > 18 ? 18 : n);
  for (size_t i = 0; i < st_.rc_count; ++i) st_.rc[i] = u16(p, 2 * i);
  // MSP RSSI 0..1023 -> RC_CHANNELS.rssi 0..254 (255 = unknown).
  const uint32_t r = static_cast<uint32_t>(st_.rssi_1023) * 254u / 1023u;
  std::vector<uint8_t> f;
  writer_.rc_channels(f, static_cast<uint32_t>(now_ms), st_.rc, st_.rc_count,
                      static_cast<uint8_t>(r > 254 ? 254 : r));
  queue(std::move(f), now_ms);
}

void MspTelemetry::queue(std::vector<uint8_t>&& frame, uint64_t now_ms) {
  if (frame.empty()) return;
  if (pending_.empty()) pending_first_ms_ = now_ms;
  pending_bytes_ += frame.size();
  ++stats_.mav_frames;
  pending_.push_back(std::move(frame));
}

std::vector<std::vector<uint8_t>> MspTelemetry::take_payloads(uint64_t now_ms) {
  if (fc_alive(now_ms) && now_ms - last_heartbeat_ms_ >= cfg_.heartbeat_ms) emit_heartbeat(now_ms);
  note_silence(now_ms);
  std::vector<std::vector<uint8_t>> out;
  if (pending_.empty()) return out;
  if (pending_bytes_ < cfg_.flush_bytes && now_ms - pending_first_ms_ < cfg_.flush_ms) return out;
  // Whole frames only; a batch stops before the frame that would not fit.
  std::vector<uint8_t> cur;
  for (auto& f : pending_) {
    if (!cur.empty() && cur.size() + f.size() > rc::kMavMaxPayload) {
      out.push_back(std::move(cur));
      cur.clear();
    }
    cur.insert(cur.end(), f.begin(), f.end());
  }
  if (!cur.empty()) out.push_back(std::move(cur));
  pending_.clear();
  pending_bytes_ = 0;
  stats_.payloads += static_cast<uint32_t>(out.size());
  for (const auto& p : out) stats_.payload_bytes += static_cast<uint32_t>(p.size());
  return out;
}

std::vector<std::vector<uint8_t>> MspTelemetry::take_frames(uint64_t now_ms) {
  if (fc_alive(now_ms) && now_ms - last_heartbeat_ms_ >= cfg_.heartbeat_ms) emit_heartbeat(now_ms);
  note_silence(now_ms);
  std::vector<std::vector<uint8_t>> out;
  out.swap(pending_);
  pending_bytes_ = 0;
  stats_.payloads += static_cast<uint32_t>(out.size());
  for (const auto& p : out) stats_.payload_bytes += static_cast<uint32_t>(p.size());
  return out;
}

}  // namespace mabur
