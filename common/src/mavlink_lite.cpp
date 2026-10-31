#include "mabur/mavlink_lite.h"

#include <cstring>

namespace mabur::mav {
namespace {

// CRC_EXTRA per message (MAVLink common.xml, the generated MAVLINK_MSG_ID_*_CRC).
constexpr uint8_t kCrcHeartbeat = 50, kCrcSysStatus = 124, kCrcGpsRawInt = 24, kCrcAttitude = 39,
                  kCrcGlobalPositionInt = 104, kCrcRcChannels = 118, kCrcVfrHud = 20,
                  kCrcBatteryStatus = 154, kCrcStatustext = 83;

struct Payload {
  uint8_t b[64];
  size_t n = 0;
  void u8(uint8_t v) { b[n++] = v; }
  void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
  void u16(uint16_t v) { u8(static_cast<uint8_t>(v)); u8(static_cast<uint8_t>(v >> 8)); }
  void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
  void u32(uint32_t v) { u16(static_cast<uint16_t>(v)); u16(static_cast<uint16_t>(v >> 16)); }
  void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
  void u64(uint64_t v) { u32(static_cast<uint32_t>(v)); u32(static_cast<uint32_t>(v >> 32)); }
  void f32(float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);  // IEEE-754 little-endian on every target here
    u32(u);
  }
};

}  // namespace

uint16_t crc_accumulate(uint8_t b, uint16_t crc) {
  uint8_t tmp = b ^ static_cast<uint8_t>(crc & 0xFF);
  tmp ^= static_cast<uint8_t>(tmp << 4);
  return static_cast<uint16_t>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

uint16_t crc_calculate(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) crc = crc_accumulate(p[i], crc);
  return crc;
}

void Writer::frame(std::vector<uint8_t>& out, uint8_t msgid, const uint8_t* payload, size_t len,
                   uint8_t crc_extra) {
  const size_t start = out.size();
  if (v2_) {
    // MAVLink 2 (STX 0xFD, no signing): trailing zero payload bytes are cut,
    // at least one kept (_mav_trim_payload); the receiver zero-fills them.
    while (len > 1 && payload[len - 1] == 0) --len;
    out.push_back(kStx2);
    out.push_back(static_cast<uint8_t>(len));
    out.push_back(0);  // incompat_flags
    out.push_back(0);  // compat_flags
    out.push_back(seq_++);
    out.push_back(sysid_);
    out.push_back(compid_);
    out.push_back(msgid);
    out.push_back(0);  // msgid bits 8..23: every message here is < 256
    out.push_back(0);
    out.insert(out.end(), payload, payload + len);
    uint16_t crc = crc_calculate(out.data() + start + 1, len + 9);
    crc = crc_accumulate(crc_extra, crc);
    out.push_back(static_cast<uint8_t>(crc & 0xFF));
    out.push_back(static_cast<uint8_t>(crc >> 8));
    return;
  }
  out.push_back(kStx);
  out.push_back(static_cast<uint8_t>(len));
  out.push_back(seq_++);
  out.push_back(sysid_);
  out.push_back(compid_);
  out.push_back(msgid);
  out.insert(out.end(), payload, payload + len);
  uint16_t crc = crc_calculate(out.data() + start + 1, len + 5);
  crc = crc_accumulate(crc_extra, crc);
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>(crc >> 8));
}

void Writer::heartbeat(std::vector<uint8_t>& out, uint32_t custom_mode, uint8_t type,
                       uint8_t autopilot, uint8_t base_mode, uint8_t system_status) {
  Payload p;
  p.u32(custom_mode);
  p.u8(type);
  p.u8(autopilot);
  p.u8(base_mode);
  p.u8(system_status);
  p.u8(3);  // mavlink_version
  frame(out, kMsgHeartbeat, p.b, p.n, kCrcHeartbeat);
}

void Writer::sys_status(std::vector<uint8_t>& out, uint16_t voltage_mv, int16_t current_ca,
                        int8_t battery_remaining) {
  Payload p;
  p.u32(0);  // sensors present
  p.u32(0);  // enabled
  p.u32(0);  // health
  p.u16(0);  // load
  p.u16(voltage_mv);
  p.i16(current_ca);
  p.u16(0);  // drop_rate_comm
  p.u16(0);  // errors_comm
  p.u16(0);
  p.u16(0);
  p.u16(0);
  p.u16(0);  // errors_count1..4
  p.i8(battery_remaining);
  frame(out, kMsgSysStatus, p.b, p.n, kCrcSysStatus);
}

void Writer::battery_status(std::vector<uint8_t>& out, int32_t current_consumed_mah,
                            int16_t current_ca, uint16_t voltage_mv, uint8_t cell_count,
                            int8_t battery_remaining) {
  Payload p;
  p.i32(current_consumed_mah);
  p.i32(-1);      // energy_consumed unknown
  p.i16(INT16_MAX);  // temperature unknown
  // voltages[10]: the pack voltage in cell 0, the rest "not available" (UINT16_MAX).
  p.u16(voltage_mv);
  for (int i = 1; i < 10; ++i) p.u16(UINT16_MAX);
  p.i16(current_ca);
  p.u8(0);  // id
  p.u8(0);  // battery_function unknown
  p.u8(cell_count);  // type field carries nothing standard here; consumers ignore it
  p.i8(battery_remaining);
  frame(out, kMsgBatteryStatus, p.b, p.n, kCrcBatteryStatus);
}

void Writer::attitude(std::vector<uint8_t>& out, uint32_t time_boot_ms, float roll_rad,
                      float pitch_rad, float yaw_rad) {
  Payload p;
  p.u32(time_boot_ms);
  p.f32(roll_rad);
  p.f32(pitch_rad);
  p.f32(yaw_rad);
  p.f32(0.0f);
  p.f32(0.0f);
  p.f32(0.0f);
  frame(out, kMsgAttitude, p.b, p.n, kCrcAttitude);
}

void Writer::vfr_hud(std::vector<uint8_t>& out, float airspeed_ms, float groundspeed_ms,
                     float alt_m, float climb_ms, int16_t heading_deg, uint16_t throttle_pct) {
  Payload p;
  p.f32(airspeed_ms);
  p.f32(groundspeed_ms);
  p.f32(alt_m);
  p.f32(climb_ms);
  p.i16(heading_deg);
  p.u16(throttle_pct);
  frame(out, kMsgVfrHud, p.b, p.n, kCrcVfrHud);
}

void Writer::global_position_int(std::vector<uint8_t>& out, uint32_t time_boot_ms, int32_t lat_e7,
                                 int32_t lon_e7, int32_t alt_mm, int32_t relative_alt_mm,
                                 int16_t vx, int16_t vy, int16_t vz, uint16_t hdg_cdeg) {
  Payload p;
  p.u32(time_boot_ms);
  p.i32(lat_e7);
  p.i32(lon_e7);
  p.i32(alt_mm);
  p.i32(relative_alt_mm);
  p.i16(vx);
  p.i16(vy);
  p.i16(vz);
  p.u16(hdg_cdeg);
  frame(out, kMsgGlobalPositionInt, p.b, p.n, kCrcGlobalPositionInt);
}

void Writer::gps_raw_int(std::vector<uint8_t>& out, uint64_t time_usec, int32_t lat_e7,
                         int32_t lon_e7, int32_t alt_mm, uint16_t eph, uint16_t epv,
                         uint16_t vel_cms, uint16_t cog_cdeg, uint8_t fix_type,
                         uint8_t satellites_visible) {
  Payload p;
  p.u64(time_usec);
  p.i32(lat_e7);
  p.i32(lon_e7);
  p.i32(alt_mm);
  p.u16(eph);
  p.u16(epv);
  p.u16(vel_cms);
  p.u16(cog_cdeg);
  p.u8(fix_type);
  p.u8(satellites_visible);
  frame(out, kMsgGpsRawInt, p.b, p.n, kCrcGpsRawInt);
}

void Writer::rc_channels(std::vector<uint8_t>& out, uint32_t time_boot_ms, const uint16_t* ch,
                         size_t n, uint8_t rssi) {
  Payload p;
  p.u32(time_boot_ms);
  for (size_t i = 0; i < 18; ++i) p.u16(i < n ? ch[i] : 0);
  p.u8(static_cast<uint8_t>(n > 18 ? 18 : n));
  p.u8(rssi);
  frame(out, kMsgRcChannels, p.b, p.n, kCrcRcChannels);
}

void Writer::statustext(std::vector<uint8_t>& out, uint8_t severity, const char* text) {
  Payload p;
  p.u8(severity);
  const size_t len = std::strlen(text);
  for (size_t i = 0; i < 50; ++i) p.u8(i < len ? static_cast<uint8_t>(text[i]) : 0);
  frame(out, kMsgStatustext, p.b, p.n, kCrcStatustext);
}

}  // namespace mabur::mav
