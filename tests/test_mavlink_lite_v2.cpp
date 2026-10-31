// mavlink_lite's MAVLink 2 framing against the vendored c_library_v2
// (third_party/mavlink): every v2 frame must parse with the library's own
// parser (STX, header, CRC with CRC_EXTRA), and carry the same payload as the
// v1 frame of the same message, whose bytes test_mavlink_lite.cpp pins to
// the generated headers. The tunnel (gs/src/mav_tunnel.h) drops anything
// that fails exactly this parse.
#include "mtest.h"

#include <cstring>
#include <vector>

#include "mabur/mav.h"
#include "mabur/mavlink_lite.h"

namespace {

using namespace mabur;

// Every message mavlink_lite writes, once, in a fixed order.
void write_all(mav::Writer& w, std::vector<std::vector<uint8_t>>& out) {
  auto add = [&](auto fn) {
    std::vector<uint8_t> b;
    fn(b);
    out.push_back(std::move(b));
  };
  add([&](std::vector<uint8_t>& b) { w.heartbeat(b, 11, 1, 3, 129, 4); });
  add([&](std::vector<uint8_t>& b) { w.sys_status(b, 16420, 1234, -1); });
  add([&](std::vector<uint8_t>& b) { w.battery_status(b, 1500, 1234, 16420, 4, -1); });
  add([&](std::vector<uint8_t>& b) { w.attitude(b, 123456, 0.25f, -0.125f, 3.0f); });
  add([&](std::vector<uint8_t>& b) { w.vfr_hud(b, 0.0f, 12.5f, 87.25f, -1.5f, 271, 42); });
  add([&](std::vector<uint8_t>& b) {
    w.global_position_int(b, 123456, 524000000, 134000000, 87250, 87250, 0, 0, 0, 27100);
  });
  add([&](std::vector<uint8_t>& b) {
    w.gps_raw_int(b, 123456000ull, 524000000, 134000000, 90000, 120, 65535, 1250, 2710, 3, 14);
  });
  const uint16_t ch[8] = {1500, 1501, 1502, 1100, 1000, 2000, 1500, 1800};
  add([&](std::vector<uint8_t>& b) { w.rc_channels(b, 123456, ch, 8, 200); });
  add([&](std::vector<uint8_t>& b) { w.statustext(b, 2, "FAILSAFE"); });
}

// One whole packet through the library parser on `chan`; true with *msg set.
bool parse(const std::vector<uint8_t>& b, uint8_t chan, mavlink_message_t* msg) {
  mavlink_status_t st{};
  bool done = false;
  for (size_t i = 0; i < b.size(); ++i) {
    const uint8_t r = mavlink_parse_char(chan, b[i], msg, &st);
    if (r == MAVLINK_FRAMING_OK) done = (i + 1 == b.size());
  }
  return done;
}

}  // namespace

TEST(v2_frames_parse_and_carry_the_v1_payload) {
  mav::Writer w1(1, 1, /*v2=*/false), w2(1, 1, /*v2=*/true);
  std::vector<std::vector<uint8_t>> f1, f2;
  write_all(w1, f1);
  write_all(w2, f2);
  REQUIRE(f1.size() == f2.size());
  for (size_t i = 0; i < f1.size(); ++i) {
    CHECK(f2[i][0] == mav::kStx2);
    mavlink_message_t m1{}, m2{};
    REQUIRE(parse(f1[i], MAVLINK_COMM_0, &m1));
    REQUIRE(parse(f2[i], MAVLINK_COMM_1, &m2));
    CHECK(m2.magic == MAVLINK_STX);  // 0xFD: parsed as MAVLink 2
    CHECK(m1.msgid == m2.msgid);
    CHECK(m2.sysid == 1 && m2.compid == 1);
    CHECK(m2.seq == static_cast<uint8_t>(i));
    // v2 cut trailing zeros; the bytes it kept are v1's and the rest are 0.
    REQUIRE(m2.len <= m1.len);
    const auto* p1 = reinterpret_cast<const uint8_t*>(m1.payload64);
    const auto* p2 = reinterpret_cast<const uint8_t*>(m2.payload64);
    CHECK(std::memcmp(p1, p2, m2.len) == 0);
    for (size_t k = m2.len; k < m1.len; ++k) CHECK(p1[k] == 0);
  }
  // A decode through the generated helpers, as the GS-side consumer reads it.
  mavlink_message_t m{};
  REQUIRE(parse(f2[3], MAVLINK_COMM_1, &m));
  mavlink_attitude_t att;
  mavlink_msg_attitude_decode(&m, &att);
  CHECK(att.time_boot_ms == 123456u);
  CHECK(att.roll == 0.25f && att.pitch == -0.125f && att.yaw == 3.0f);
}

TEST(v2_payload_keeps_one_byte_when_all_zero) {
  mav::Writer w(1, 1, true);
  std::vector<uint8_t> b;
  w.heartbeat(b, 0, 0, 0, 0, 0);
  // HEARTBEAT's mavlink_version byte is 3, so the payload is never all zero;
  // the cut still stops at the last non-zero byte.
  mavlink_message_t m{};
  REQUIRE(parse(b, MAVLINK_COMM_2, &m));
  CHECK(m.msgid == MAVLINK_MSG_ID_HEARTBEAT);
  CHECK(m.len == 9);
}

MTEST_MAIN
