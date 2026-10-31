#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

// The nine MAVLink v1 messages PixelPilot's OSD reads (app/mavlink/src/main/
// cpp/mavlink.cpp), packed by hand instead of vendoring the generated
// library: v1 framing is STX 0xFE, len, seq, sysid, compid, msgid, payload,
// then CRC-16/MCRF4XX over len..payload with the message's CRC_EXTRA byte
// folded in. Payload fields are in MAVLink WIRE order (sorted by type size,
// largest first), not declaration order. Goldens in tests/test_mavlink_lite.cpp
// were produced by the generated headers PixelPilot ships, so a wrong CRC_EXTRA
// or field order fails there.
namespace mabur::mav {

constexpr uint8_t kStx = 0xFE;
constexpr uint8_t kStx2 = 0xFD;  // MAVLink 2, unsigned
constexpr uint8_t kMsgHeartbeat = 0, kMsgSysStatus = 1, kMsgGpsRawInt = 24, kMsgAttitude = 30,
                  kMsgGlobalPositionInt = 33, kMsgRcChannels = 65, kMsgVfrHud = 74,
                  kMsgBatteryStatus = 147, kMsgStatustext = 253;

// HEARTBEAT.base_mode / .type / .autopilot / .system_status values used here.
constexpr uint8_t kModeFlagCustomModeEnabled = 1, kModeFlagSafetyArmed = 128;
constexpr uint8_t kTypeFixedWing = 1, kTypeQuadrotor = 2;
constexpr uint8_t kAutopilotArdupilotmega = 3;
constexpr uint8_t kStateStandby = 3, kStateActive = 4, kStateCritical = 5;
// STATUSTEXT.severity.
constexpr uint8_t kSeverityCritical = 2, kSeverityWarning = 4, kSeverityInfo = 6;

uint16_t crc_accumulate(uint8_t b, uint16_t crc);
uint16_t crc_calculate(const uint8_t* p, size_t n);  // init 0xFFFF

class Writer {
 public:
  // v2: MAVLink 2 framing (mabur's T_MAV tunnel carries MAVLink 2 only),
  // else v1, which is all PixelPilot's own OSD module needed on its own.
  explicit Writer(uint8_t sysid = 1, uint8_t compid = 1, bool v2 = false)
      : sysid_(sysid), compid_(compid), v2_(v2) {}
  uint8_t seq() const { return seq_; }

  // One raw frame around a wire-ordered payload.
  void frame(std::vector<uint8_t>& out, uint8_t msgid, const uint8_t* payload, size_t len,
             uint8_t crc_extra);

  void heartbeat(std::vector<uint8_t>& out, uint32_t custom_mode, uint8_t type, uint8_t autopilot,
                 uint8_t base_mode, uint8_t system_status);
  // voltage in mV, current in 10 mA (cA), remaining -1 = unknown.
  void sys_status(std::vector<uint8_t>& out, uint16_t voltage_mv, int16_t current_ca,
                  int8_t battery_remaining);
  void battery_status(std::vector<uint8_t>& out, int32_t current_consumed_mah, int16_t current_ca,
                      uint16_t voltage_mv, uint8_t cell_count, int8_t battery_remaining);
  void attitude(std::vector<uint8_t>& out, uint32_t time_boot_ms, float roll_rad, float pitch_rad,
                float yaw_rad);
  void vfr_hud(std::vector<uint8_t>& out, float airspeed_ms, float groundspeed_ms, float alt_m,
               float climb_ms, int16_t heading_deg, uint16_t throttle_pct);
  void global_position_int(std::vector<uint8_t>& out, uint32_t time_boot_ms, int32_t lat_e7,
                           int32_t lon_e7, int32_t alt_mm, int32_t relative_alt_mm, int16_t vx,
                           int16_t vy, int16_t vz, uint16_t hdg_cdeg);
  void gps_raw_int(std::vector<uint8_t>& out, uint64_t time_usec, int32_t lat_e7, int32_t lon_e7,
                   int32_t alt_mm, uint16_t eph, uint16_t epv, uint16_t vel_cms, uint16_t cog_cdeg,
                   uint8_t fix_type, uint8_t satellites_visible);
  // Up to 18 channels; missing ones are 0 (UINT16_MAX means unused in the
  // spec, but msposd/PixelPilot read chan8 as a number). rssi 0..254, 255 = unknown.
  void rc_channels(std::vector<uint8_t>& out, uint32_t time_boot_ms, const uint16_t* ch, size_t n,
                   uint8_t rssi);
  void statustext(std::vector<uint8_t>& out, uint8_t severity, const char* text);

 private:
  uint8_t seq_ = 0, sysid_, compid_;
  bool v2_;
};

}  // namespace mabur::mav
