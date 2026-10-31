// FC telemetry over MSP (msp_telemetry.h): the request scheduler, the reply
// decoders for Betaflight and INAV, the flight-mode and arming mapping, and
// the T_MAV batching. Replies are laid out here byte for byte as the two
// firmwares' MSP handlers write them (INAV fc_msp.c, Betaflight msp.c), and
// the MAVLink side is read back with a minimal v1 reader whose CRC path is the
// one test_mavlink_lite.cpp pins against the generated headers.
#include "mtest.h"

#include <cmath>
#include <cstring>
#include <map>
#include <string>

#include "mabur/msp_telemetry.h"
#include "mabur/rc_proto.h"

using namespace mabur;

namespace {

std::vector<uint8_t> reply_v1(uint8_t cmd, std::vector<uint8_t> payload) {
  std::vector<uint8_t> f = {'$', 'M', '>', static_cast<uint8_t>(payload.size()), cmd};
  uint8_t cks = static_cast<uint8_t>(payload.size()) ^ cmd;
  for (uint8_t b : payload) { f.push_back(b); cks ^= b; }
  f.push_back(cks);
  return f;
}

std::vector<uint8_t> reply_v2(uint16_t cmd, std::vector<uint8_t> payload) {
  std::vector<uint8_t> f = {'$', 'X', '>'};
  const uint8_t hdr[5] = {0, static_cast<uint8_t>(cmd & 0xFF), static_cast<uint8_t>(cmd >> 8),
                          static_cast<uint8_t>(payload.size() & 0xFF),
                          static_cast<uint8_t>(payload.size() >> 8)};
  uint8_t crc = 0;
  for (uint8_t h : hdr) { f.push_back(h); crc = msp_crc8_dvb_s2(crc, h); }
  for (uint8_t b : payload) { f.push_back(b); crc = msp_crc8_dvb_s2(crc, b); }
  f.push_back(crc);
  return f;
}

void put16(std::vector<uint8_t>& v, uint16_t x) { v.push_back(x & 0xFF); v.push_back(x >> 8); }
void put32(std::vector<uint8_t>& v, uint32_t x) { put16(v, x & 0xFFFF); put16(v, x >> 16); }

struct Frame {
  uint8_t msgid = 0;
  std::vector<uint8_t> payload;
  bool crc_ok = false;
};

// crc_extra per message id, as in mavlink_lite.cpp.
uint8_t crc_extra(uint8_t msgid) {
  switch (msgid) {
    case 0: return 50;
    case 1: return 124;
    case 24: return 24;
    case 30: return 39;
    case 33: return 104;
    case 65: return 118;
    case 74: return 20;
    case 147: return 154;
    case 253: return 83;
    default: return 0;
  }
}

std::vector<Frame> read_frames(const std::vector<uint8_t>& bytes) {
  std::vector<Frame> out;
  size_t i = 0;
  while (i + 8 <= bytes.size()) {
    if (bytes[i] != mav::kStx) { ++i; continue; }
    const size_t len = bytes[i + 1];
    if (i + 8 + len > bytes.size()) break;
    Frame f;
    f.msgid = bytes[i + 5];
    f.payload.assign(bytes.begin() + static_cast<long>(i + 6), bytes.begin() + static_cast<long>(i + 6 + len));
    uint16_t crc = mav::crc_calculate(bytes.data() + i + 1, len + 5);
    crc = mav::crc_accumulate(crc_extra(f.msgid), crc);
    f.crc_ok = bytes[i + 6 + len] == (crc & 0xFF) && bytes[i + 7 + len] == (crc >> 8);
    out.push_back(std::move(f));
    i += 8 + len;
  }
  return out;
}

uint16_t rd16(const std::vector<uint8_t>& p, size_t o) { return static_cast<uint16_t>(p[o] | (p[o + 1] << 8)); }
uint32_t rd32(const std::vector<uint8_t>& p, size_t o) { return rd16(p, o) | (static_cast<uint32_t>(rd16(p, o + 2)) << 16); }
float rdf(const std::vector<uint8_t>& p, size_t o) { float f; uint32_t u = rd32(p, o); std::memcpy(&f, &u, 4); return f; }

// Everything MspTelemetry has flushed so far, drained through take_payloads
// at `now` (forcing the age flush by advancing well past flush_ms).
std::vector<Frame> drain(MspTelemetry& t, uint64_t now) {
  std::vector<uint8_t> all;
  for (auto& p : t.take_payloads(now + 1000)) all.insert(all.end(), p.begin(), p.end());
  return read_frames(all);
}

const Frame* last_of(const std::vector<Frame>& fs, uint8_t msgid) {
  const Frame* r = nullptr;
  for (const auto& f : fs) if (f.msgid == msgid) r = &f;
  return r;
}

// One request per tick: which command was asked.
int req_cmd(const std::vector<uint8_t>& r) {
  if (r.size() >= 5 && r[1] == 'M') return r[4];
  if (r.size() >= 8 && r[1] == 'X') return r[4] | (r[5] << 8);
  return -1;
}

}  // namespace

TEST(scheduler_paces_one_request_per_tick_at_the_configured_rates) {
  MspTelemetry t;
  std::map<int, int> count;
  // One request per 20 ms tick, the way the maburd serial thread drives it.
  for (uint64_t now = 0; now < 60000; now += 20) {
    auto r = t.next_request(now);
    if (r.empty()) continue;
    count[req_cmd(r)]++;
  }
  // 60 s at the defaults: attitude 10 Hz, altitude 5, gps 4, rc 4, comp_gps 2,
  // analog 2, status 2, battery 1, variant 1; boxids only after a variant reply.
  // Slots keep their own grid, so a tick spent behind another slot never
  // costs a request: the counts are exact.
  CHECK(count[108] == 600);
  CHECK(count[109] == 300);
  CHECK(count[106] == 240);
  CHECK(count[105] == 240);
  CHECK(count[107] == 120);
  CHECK(count[110] == 120);
  CHECK(count[101] == 120);
  CHECK(count[130] == 60);
  CHECK(count[2] == 60);
  CHECK(count.count(119) == 0);
  CHECK(count.count(0x2000) == 0);
  CHECK(t.stats().polls == static_cast<uint32_t>(count[108] + count[109] + count[106] + count[105] +
                                                  count[107] + count[110] + count[101] + count[130] +
                                                  count[2]));
}

TEST(no_reply_means_no_heartbeat_and_no_payloads) {
  MspTelemetry t;
  for (uint64_t now = 0; now < 5000; now += 20) {
    (void)t.next_request(now);
    CHECK(t.take_payloads(now).empty());
  }
  CHECK(!t.fc_alive(5000));
}

// last_status_ms(): only a status reply is an arm-state report. The BOXIDS
// reply re-derives armed from a box mask that is still 0 before the first
// status, and must not read as a fresh DISARMED report (fork, low_power).
TEST(boxids_reply_is_not_an_arm_report) {
  MspTelemetry t;
  uint64_t now = 1000;
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  auto ids = reply_v1(119, {0, 1, 2});
  t.on_serial_bytes(ids.data(), ids.size(), now += 20);
  CHECK(t.last_status_ms() == 0);
  std::vector<uint8_t> st;
  put16(st, 125); put16(st, 0); put16(st, 0x23); put32(st, 0x1); st.push_back(0); put16(st, 12); put16(st, 0);
  st.push_back(0); st.push_back(0); put32(st, 0); st.push_back(0);
  auto s = reply_v1(101, st);
  t.on_serial_bytes(s.data(), s.size(), now += 20);
  CHECK(t.last_status_ms() == now);
  CHECK(t.state().armed);
}

TEST(betaflight_session_decodes_and_emits_every_message) {
  MspTelemetry t;
  uint64_t now = 1000;
  // FC_VARIANT -> variant, one STATUSTEXT, box ids requested next.
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  CHECK(t.state().variant == MspTelemetry::Variant::Betaflight);
  bool asked_boxids = false;
  for (int i = 0; i < 40 && !asked_boxids; ++i) asked_boxids = req_cmd(t.next_request(now += 20)) == 119;
  CHECK(asked_boxids);
  // Betaflight's active-box order: ARM, ANGLE, HORIZON, ..., FAILSAFE(27) at bit 8, GPS RESCUE(46) at bit 9.
  auto ids = reply_v1(119, {0, 1, 2, 5, 6, 12, 13, 26, 27, 46, 47});
  t.on_serial_bytes(ids.data(), ids.size(), now);
  CHECK(t.state().box_ids.size() == 11);
  // MSP_STATUS: cycle, i2c, sensors, flags = ARM|ANGLE, profile, load, gyro,
  // byteCount 1 + one extra byte, arming-disable count + flags, config state.
  std::vector<uint8_t> st;
  put16(st, 125); put16(st, 0); put16(st, 0x23); put32(st, 0x3); st.push_back(0); put16(st, 12); put16(st, 0);
  st.push_back(1); st.push_back(0); st.push_back(25); put32(st, 0); st.push_back(0);
  auto s = reply_v1(101, st);
  t.on_serial_bytes(s.data(), s.size(), now);
  CHECK(t.state().armed);
  CHECK(t.state().custom_mode == 2);  // ANGLE -> STABILIZE
  // ATTITUDE 10.0 deg roll, -5.0 deg pitch, yaw 271.
  std::vector<uint8_t> at; put16(at, 100); put16(at, static_cast<uint16_t>(-50)); put16(at, 271);
  auto a = reply_v1(108, at);
  t.on_serial_bytes(a.data(), a.size(), now);
  // ANALOG (Betaflight, 9 bytes): vbat 0.1 V = 164, mAh 1500, rssi 511, amps 12.34 A, vbat 0.01 V = 1642.
  std::vector<uint8_t> an; an.push_back(164); put16(an, 1500); put16(an, 511); put16(an, 1234); put16(an, 1642);
  auto b = reply_v1(110, an);
  t.on_serial_bytes(b.data(), b.size(), now);
  // RC: 8 channels, throttle index 3 = 1420 -> 42 %; carries the ANALOG rssi.
  std::vector<uint8_t> rc; for (uint16_t c : {1500, 1501, 1502, 1420, 1000, 2000, 1500, 1800}) put16(rc, c);
  auto r = reply_v1(105, rc);
  t.on_serial_bytes(r.data(), r.size(), now);
  // RAW_GPS: fix 1 (STATE(GPS_FIX)), 14 sats, lat/lon e7, alt 90 m, speed 12.5 m/s, course 271.0, pdop 120.
  std::vector<uint8_t> g; g.push_back(1); g.push_back(14); put32(g, 524000000u); put32(g, 134000000u);
  put16(g, 90); put16(g, 1250); put16(g, 2710); put16(g, 120);
  auto gg = reply_v1(106, g);
  t.on_serial_bytes(gg.data(), gg.size(), now);
  // ALTITUDE: 87.25 m, climb -1.5 m/s, baro.
  std::vector<uint8_t> al; put32(al, 8725); put16(al, static_cast<uint16_t>(-150)); put32(al, 8700);
  auto aa = reply_v1(109, al);
  t.on_serial_bytes(aa.data(), aa.size(), now);
  CHECK(t.stats().replies == 8);
  CHECK(t.stats().unknown == 0);

  const auto fs = drain(t, now);
  for (const auto& f : fs) CHECK(f.crc_ok);
  const Frame* hb = last_of(fs, 0);
  CHECK(hb && rd32(hb->payload, 0) == 2 && hb->payload[4] == 2 /*quadrotor*/ && (hb->payload[6] & 128) &&
        hb->payload[7] == 4);
  const Frame* txt = last_of(fs, 253);
  CHECK(txt && txt->payload[0] == 6 && std::strncmp(reinterpret_cast<const char*>(&txt->payload[1]), "FC BTFL via MSP", 50) == 0);
  const Frame* att = last_of(fs, 30);
  CHECK(att && std::fabs(rdf(att->payload, 4) - 10.0f * 3.14159265f / 180.0f) < 1e-5f &&
        std::fabs(rdf(att->payload, 8) + 5.0f * 3.14159265f / 180.0f) < 1e-5f);
  const Frame* rcf = last_of(fs, 65);
  CHECK(rcf && rd16(rcf->payload, 4) == 1500 && rd16(rcf->payload, 10) == 1420 && rcf->payload[40] == 8 &&
        rcf->payload[41] == 511 * 254 / 1023);
  const Frame* sys = last_of(fs, 1);
  CHECK(sys && rd16(sys->payload, 14) == 16420 && static_cast<int16_t>(rd16(sys->payload, 16)) == 1234);
  const Frame* bat = last_of(fs, 147);
  CHECK(bat && static_cast<int32_t>(rd32(bat->payload, 0)) == 1500 && rd16(bat->payload, 10) == 16420);
  const Frame* gps = last_of(fs, 24);
  CHECK(gps && gps->payload[28] == 3 && gps->payload[29] == 14 && static_cast<int32_t>(rd32(gps->payload, 8)) == 524000000 &&
        rd16(gps->payload, 24) == 1250 && rd16(gps->payload, 26) == 27100 && rd16(gps->payload, 20) == 120);
  const Frame* pos = last_of(fs, 33);
  CHECK(pos && static_cast<int32_t>(rd32(pos->payload, 4)) == 524000000 && static_cast<int32_t>(rd32(pos->payload, 16)) == 87250 &&
        rd16(pos->payload, 26) == 27100);
  const Frame* hud = last_of(fs, 74);
  CHECK(hud && rdf(hud->payload, 4) == 12.5f && rdf(hud->payload, 8) == 87.25f && rdf(hud->payload, 12) == -1.5f &&
        static_cast<int16_t>(rd16(hud->payload, 16)) == 271 && rd16(hud->payload, 18) == 42);
}

TEST(betaflight_failsafe_and_gps_rescue_modes) {
  MspTelemetry t;
  uint64_t now = 1000;
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  auto ids = reply_v1(119, {0, 1, 2, 27, 46});
  t.on_serial_bytes(ids.data(), ids.size(), now);
  auto status = [&](uint32_t flags) {
    std::vector<uint8_t> st;
    put16(st, 125); put16(st, 0); put16(st, 0); put32(st, flags); st.push_back(0);
    auto s = reply_v1(101, st);
    t.on_serial_bytes(s.data(), s.size(), now += 100);
  };
  (void)drain(t, now);
  status(1u << 0 | 1u << 3);  // ARM + FAILSAFE, no rescue
  auto fs = drain(t, now);
  const Frame* hb = last_of(fs, 0);
  CHECK(hb && rd32(hb->payload, 0) == 200 && hb->payload[7] == 5 /*critical*/);
  const Frame* txt = last_of(fs, 253);
  CHECK(txt && txt->payload[0] == 2 && std::strncmp(reinterpret_cast<const char*>(&txt->payload[1]), "FAILSAFE", 50) == 0);
  status(1u << 0 | 1u << 3 | 1u << 4);  // + GPS RESCUE -> RTL
  fs = drain(t, now);
  hb = last_of(fs, 0);
  CHECK(hb && rd32(hb->payload, 0) == 11);
  CHECK(!last_of(fs, 253));  // failsafe already announced
  status(1u << 2);  // HORIZON, disarmed
  fs = drain(t, now);
  hb = last_of(fs, 0);
  CHECK(hb && rd32(hb->payload, 0) == 3 && !(hb->payload[6] & 128) && hb->payload[7] == 3);
  CHECK(!t.state().failsafe);
}

TEST(inav_session_uses_msp2_status_and_the_plane_mode_map) {
  MspTelemetry t;
  uint64_t now = 1000;
  auto v = reply_v1(2, {'I', 'N', 'A', 'V'});
  t.on_serial_bytes(v.data(), v.size(), now);
  // Status polls switch to the v2 request; the v1 one stops.
  std::map<int, int> count;
  for (int i = 0; i < 200; ++i) count[req_cmd(t.next_request(now += 20))]++;
  CHECK(count[0x2000] > 0);
  CHECK(count[101] == 0);
  CHECK(count[119] > 0);
  auto ids = reply_v1(119, {0, 1, 2, 3, 5, 10, 11, 12, 27, 28, 36, 45, 53});
  t.on_serial_bytes(ids.data(), ids.size(), now);
  auto inav_status = [&](uint32_t arming, uint64_t mask) {
    std::vector<uint8_t> st;
    put16(st, 500); put16(st, 0); put16(st, 0x63); put16(st, 20); st.push_back(0x10); put32(st, arming);
    for (int i = 0; i < 8; ++i) st.push_back(static_cast<uint8_t>(mask >> (8 * i)));
    st.push_back(0);  // mixer profile
    auto s = reply_v2(0x2000, st);
    t.on_serial_bytes(s.data(), s.size(), now += 100);
  };
  inav_status(1u << 2, 1ull << 5);  // armed via armingFlags, NAV RTH (bit 5 = permanent id 10)
  auto fs = drain(t, now);
  const Frame* hb = last_of(fs, 0);
  CHECK(hb && rd32(hb->payload, 0) == 11 && (hb->payload[6] & 128) && hb->payload[4] == 1 /*fixed wing*/);
  CHECK(t.state().armed);
  inav_status(0, 1ull << 1);  // disarmed, ANGLE -> FBWA like INAV's own MAVLink
  fs = drain(t, now);
  hb = last_of(fs, 0);
  CHECK(hb && rd32(hb->payload, 0) == 5 && !(hb->payload[6] & 128));
  inav_status(0, (1ull << 11) | (1ull << 3));  // COURSE HOLD + ALTHOLD -> CRUISE
  CHECK(t.state().custom_mode == 7);
  inav_status(0, 1ull << 7);  // MANUAL
  CHECK(t.state().custom_mode == 0);
  inav_status(0, 1ull << 9);  // NAV WP -> AUTO
  CHECK(t.state().custom_mode == 10);
  inav_status(0, 1ull << 10);  // NAV LAUNCH -> TAKEOFF
  CHECK(t.state().custom_mode == 13);
  inav_status(0, 1ull << 8);  // FAILSAFE without RTH
  CHECK(t.state().custom_mode == 200 && t.state().failsafe);
  // INAV RAW_GPS fix 2 = 3D; ANALOG is the 7-byte form, BATTERY_STATE refines vbat.
  std::vector<uint8_t> g; g.push_back(2); g.push_back(9); put32(g, 1); put32(g, 2); put16(g, 10); put16(g, 0); put16(g, 0); put16(g, 150);
  auto gg = reply_v1(106, g);
  t.on_serial_bytes(gg.data(), gg.size(), now);
  CHECK(t.state().fix == 3 && t.state().hdop == 150);
  std::vector<uint8_t> an; an.push_back(164); put16(an, 300); put16(an, 800); put16(an, 250);
  auto b = reply_v1(110, an);
  t.on_serial_bytes(b.data(), b.size(), now);
  CHECK(t.state().vbat_cv == 1640);
  std::vector<uint8_t> bs; bs.push_back(4); put16(bs, 5000); bs.push_back(164); put16(bs, 301); put16(bs, 251); bs.push_back(0); put16(bs, 1643);
  auto bb = reply_v1(130, bs);
  t.on_serial_bytes(bb.data(), bb.size(), now);
  CHECK(t.state().vbat_cv == 1643 && t.state().cells == 4);
  // A coarse ANALOG right after keeps the fine reading.
  t.on_serial_bytes(b.data(), b.size(), now + 100);
  CHECK(t.state().vbat_cv == 1643);
  fs = drain(t, now + 100);
  const Frame* sys = last_of(fs, 1);
  CHECK(sys && rd16(sys->payload, 14) == 16430);
}

TEST(heartbeat_keeps_1hz_while_the_fc_answers_and_stops_after_the_timeout) {
  MspTelemetry t;
  uint64_t now = 1000;
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  std::vector<uint8_t> st; put16(st, 125); put16(st, 0); put16(st, 0); put32(st, 0); st.push_back(0);
  auto s = reply_v1(101, st);
  t.on_serial_bytes(s.data(), s.size(), now);
  int beats = 0;
  for (; now < 11000; now += 20) {
    for (auto& p : t.take_payloads(now))
      for (const auto& f : read_frames(p)) beats += f.msgid == 0;
  }
  // The status reply's own beat at t=1 s plus one per second after it while
  // alive (3 s timeout from the last reply at 1 s): 1 + beats at 2, 3, 4 s.
  CHECK(beats >= 3 && beats <= 5);
  CHECK(!t.fc_alive(now));
}

TEST(payloads_hold_whole_frames_under_the_t_mav_limit) {
  MspTelemetryCfg cfg;
  cfg.flush_ms = 100;
  cfg.flush_bytes = 400;
  MspTelemetry t(cfg);
  uint64_t now = 1000;
  std::vector<uint8_t> at; put16(at, 100); put16(at, 0); put16(at, 0);
  auto a = reply_v1(108, at);
  // 30 attitude replies = 30 x 36 bytes = 1080 bytes queued within one flush
  // period; the first reply makes the FC alive, so the first take_payloads
  // also queues the 17-byte heartbeat, and 400 bytes are reached with the
  // 11th attitude frame (i = 10).
  for (int i = 0; i < 30; ++i) {
    t.on_serial_bytes(a.data(), a.size(), now + i);
    CHECK(t.take_payloads(now + i).empty() || i >= 10);  // nothing before 400 bytes are queued
  }
  auto out = t.take_payloads(now + 200);
  size_t total = 0;
  for (const auto& p : out) {
    CHECK(p.size() <= rc::kMavMaxPayload);
    CHECK(p.size() % 36 == 0);  // whole ATTITUDE frames only
    total += p.size();
    for (const auto& f : read_frames(p)) CHECK(f.crc_ok && f.msgid == 30);
  }
  CHECK(total + t.stats().payload_bytes >= 30 * 36);
  CHECK(t.take_payloads(now + 400).empty());
}

TEST(displayport_and_unknown_messages_are_counted_not_decoded) {
  MspTelemetry t;
  auto dp = reply_v1(182, {4});
  t.on_serial_bytes(dp.data(), dp.size(), 10);
  auto odd = reply_v1(77, {1, 2, 3});
  t.on_serial_bytes(odd.data(), odd.size(), 10);
  CHECK(t.stats().displayport == 1 && t.stats().unknown == 1 && t.stats().replies == 0);
  CHECK(!t.fc_alive(10));
  CHECK(t.take_payloads(500).empty());
}

MTEST_MAIN

TEST(arming_without_a_mode_change_beats_at_once) {
  // The OSD's ARMED flag must not wait for the 1 Hz beat: an arm or disarm
  // with the mode unchanged emits a HEARTBEAT on the spot, an unchanged
  // status emits nothing.
  MspTelemetry t;
  uint64_t now = 1000;
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  auto status = [&](uint32_t flags) {
    std::vector<uint8_t> st; put16(st, 125); put16(st, 0); put16(st, 0); put32(st, flags); st.push_back(0);
    auto s = reply_v1(101, st);
    t.on_serial_bytes(s.data(), s.size(), now);
  };
  // take_payloads 100 ms later flushes by age but is too early for a periodic beat.
  auto beats = [&](uint64_t at) {
    std::vector<uint8_t> all;
    for (auto& p : t.take_payloads(at)) all.insert(all.end(), p.begin(), p.end());
    std::vector<const Frame*> out;
    static std::vector<Frame> keep;
    keep = read_frames(all);
    for (const auto& f : keep) if (f.msgid == 0) out.push_back(&f);
    return out;
  };
  status(1u << 1);  // ANGLE, disarmed: the first beat
  auto hb = beats(now + 100);
  CHECK(hb.size() == 1 && !(hb[0]->payload[6] & 128) && rd32(hb[0]->payload, 0) == 2);
  now = 1200;
  status((1u << 1) | 1u);  // ARM joins, mode unchanged
  hb = beats(now + 100);
  CHECK(hb.size() == 1 && (hb[0]->payload[6] & 128));
  now = 1400;
  status(1u << 1);  // disarm
  hb = beats(now + 100);
  CHECK(hb.size() == 1 && !(hb[0]->payload[6] & 128));
  now = 1600;
  status(1u << 1);  // nothing changed
  hb = beats(now + 100);
  CHECK(hb.empty());
}

TEST(betaflight_arming_disable_flags_become_statustext) {
  // MSP_STATUS's armingDisableFlags answer "why can't I arm" on the OSD: a
  // change emits one STATUSTEXT naming the bits, the reported count's top bit
  // is ARMSWITCH, clearing them says ARM OK, an unchanged set says nothing.
  MspTelemetry t;
  uint64_t now = 1000;
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  auto status = [&](uint32_t flags, uint8_t count) {
    std::vector<uint8_t> st;
    put16(st, 125); put16(st, 0); put16(st, 0); put32(st, 0); st.push_back(0); put16(st, 12); put16(st, 0);
    st.push_back(1); st.push_back(0); st.push_back(count); put32(st, flags); st.push_back(0);
    auto s = reply_v1(101, st);
    t.on_serial_bytes(s.data(), s.size(), now);
  };
  auto texts = [&](uint64_t at) {
    std::vector<uint8_t> all;
    for (auto& p : t.take_payloads(at)) all.insert(all.end(), p.begin(), p.end());
    std::vector<std::string> out;
    for (const auto& f : read_frames(all))
      if (f.msgid == 253) out.emplace_back(reinterpret_cast<const char*>(&f.payload[1]), strnlen(reinterpret_cast<const char*>(&f.payload[1]), 50));
    return out;
  };
  (void)texts(now + 100);  // the variant STATUSTEXT
  status((1u << 2) | (1u << 7), 25);
  auto tx = texts(now + 100);
  CHECK(tx.size() == 1 && tx[0] == "ARM: RXLOSS THROTTLE");
  CHECK(t.state().arm_disable == ((1u << 2) | (1u << 7)) && t.state().arm_disable_count == 25);
  now = 1200;
  status((1u << 2) | (1u << 7), 25);  // unchanged: silent
  CHECK(texts(now + 100).empty());
  now = 1400;
  status((1u << 24) | (1u << 26), 27);  // the count's top bit is ARMSWITCH, an unknown one is BITn
  tx = texts(now + 100);
  CHECK(tx.size() == 1 && tx[0] == "ARM: MOTOR_PROTO ARMSWITCH");
  now = 1600;
  status((1u << 27), 28);
  tx = texts(now + 100);
  CHECK(tx.size() == 1 && tx[0] == "ARM: ARMSWITCH");
  now = 1800;
  status((1u << 25), 28);
  tx = texts(now + 100);
  CHECK(tx.size() == 1 && tx[0] == "ARM: BIT25");
  now = 2000;
  status(0, 28);
  tx = texts(now + 100);
  CHECK(tx.size() == 1 && tx[0] == "ARM OK");
}

TEST(fc_silence_is_announced_on_the_status_line) {
  // Requests going out with nothing coming back: one WARNING STATUSTEXT after
  // silent_after_ms, another every silent_text_ms, none while replies flow,
  // and one "back" line when the FC answers again. The OSD holds the last
  // mode and battery otherwise, so a stale FAILSAFE read as current for 25
  // minutes on 2026-09-18 while the drone had heard nothing since its boot.
  MspTelemetry t;
  uint64_t now = 1000;
  std::vector<std::string> got;
  auto collect = [&](uint64_t at) {
    std::vector<uint8_t> all;
    for (auto& p : t.take_payloads(at)) all.insert(all.end(), p.begin(), p.end());
    for (const auto& f : read_frames(all))
      if (f.msgid == 253)
        got.emplace_back(reinterpret_cast<const char*>(&f.payload[1]),
                         strnlen(reinterpret_cast<const char*>(&f.payload[1]), 50));
  };
  // The drone's loop: a request slot every 20 ms, payloads drained each tick.
  auto run = [&](uint64_t until, bool answer) {
    got.clear();
    for (; now < until; now += 20) {
      (void)t.next_request(now);
      if (answer && now % 500 == 0) {
        auto a = reply_v1(108, {0, 0, 0, 0, 0, 0});  // ATTITUDE
        t.on_serial_bytes(a.data(), a.size(), now);
      }
      collect(now);
    }
  };
  run(1000 + 4900, false);
  CHECK(got.empty());  // a booting FC gets the grace period without nagging
  run(1000 + 5200, false);
  CHECK(got.size() == 1 && got[0].rfind("FC silent 5 s, ", 0) == 0);
  CHECK(got.size() == 1 && got[0].find(" polls unanswered") != std::string::npos);
  CHECK(t.stats().silent_texts == 1);
  run(1000 + 15200, false);
  CHECK(got.size() == 1 && got[0].rfind("FC silent 15 s, ", 0) == 0);  // every 10 s, not every tick
  // A reply ends it: the "back" line once, then nothing while answers keep coming.
  auto v = reply_v1(2, {'B', 'T', 'F', 'L'});
  t.on_serial_bytes(v.data(), v.size(), now);
  run(now + 200, false);
  CHECK(got.size() == 2 && got[0] == "FC BTFL via MSP" && got[1] == "FC back after 15 s");
  run(now + 12000, true);
  CHECK(got.empty());
  CHECK(t.stats().silent_texts == 2);
  // Silence again counts from the last reply, not from boot.
  run(now + 4000, false);
  CHECK(got.empty());
  run(now + 1600, false);
  CHECK(got.size() == 1 && got[0].rfind("FC silent 5 s, ", 0) == 0);
}
