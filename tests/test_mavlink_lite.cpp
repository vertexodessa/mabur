// The hand-packed MAVLink v1 frames (mavlink_lite.h) against goldens produced
// on 2026-09-18 by the generated MAVLink headers PixelPilot ships
// (pixelpilot/app/mavlink/src/main/cpp/mavlink, mavlink_parse_char +
// mavlink_msg_*_decode of these exact bytes, every field checked). A wrong
// CRC_EXTRA, a field out of wire order or a changed default lands here.
#include "mtest.h"

#include <string>

#include "mabur/mavlink_lite.h"
#include "vectors.h"

using namespace mabur;

namespace {
std::string hex(const std::vector<uint8_t>& v) { return mtest::hex(v); }
}  // namespace

TEST(crc_mcrf4xx_check_value) {
  // "123456789" -> 0x6F91 for CRC-16/MCRF4XX (init 0xFFFF, reflected).
  const char* s = "123456789";
  CHECK(mav::crc_calculate(reinterpret_cast<const uint8_t*>(s), 9) == 0x6F91);
}

TEST(frames_match_the_generated_library) {
  mav::Writer w(1, 1);
  std::vector<uint8_t> b;
  w.heartbeat(b, 11, 1, 3, 129, 4);
  CHECK(hex(b) == "fe09000101000b00000001038104033834");
  b.clear();
  w.sys_status(b, 16420, 1234, -1);
  CHECK(hex(b) == "fe1f0101010100000000000000000000000000002440d204000000000000000000000000ffb549");
  b.clear();
  w.battery_status(b, 1500, 1234, 16420, 4, -1);
  CHECK(hex(b) == "fe2402010193dc050000ffffffffff7f2440ffffffffffffffffffffffffffffffffffffd204000004ff5bdf");
  b.clear();
  w.attitude(b, 123456, 0.25f, -0.125f, 3.0f);
  CHECK(hex(b) == "fe1c0301011e40e201000000803e000000be0000404000000000000000000000000041ea");
  b.clear();
  w.vfr_hud(b, 0.0f, 12.5f, 87.25f, -1.5f, 271, 42);
  CHECK(hex(b) == "fe140401014a00000000000048410080ae420000c0bf0f012a00a09b");
  b.clear();
  w.global_position_int(b, 123456, 524000000, 134000000, 87250, 87250, 0, 0, 0, 27100);
  CHECK(hex(b) == "fe1c0501012140e20100009b3b1f80adfc07d2540100d2540100000000000000dc698da2");
  b.clear();
  w.gps_raw_int(b, 123456000ull, 524000000, 134000000, 90000, 120, 65535, 1250, 2710, 3, 14);
  CHECK(hex(b) == "fe1e0601011800ca5b0700000000009b3b1f80adfc07905f01007800ffffe204960a030e486b");
  b.clear();
  const uint16_t ch[8] = {1500, 1501, 1502, 1100, 1000, 2000, 1500, 1800};
  w.rc_channels(b, 123456, ch, 8, 200);
  CHECK(hex(b) == "fe2a0701014140e20100dc05dd05de054c04e803d007dc050807000000000000000000000000000000000000000008c8c97c");
  b.clear();
  w.statustext(b, 2, "FAILSAFE");
  CHECK(hex(b) == "fe33080101fd024641494c53414645000000000000000000000000000000000000000000000000000000000000000000000000000000000000bd82");
  // The sequence counter ran 0..8 across those nine frames.
  CHECK(w.seq() == 9);
}

TEST(statustext_is_cut_at_fifty_bytes_and_channels_at_eighteen) {
  mav::Writer w;
  std::vector<uint8_t> b;
  std::string longtext(80, 'x');
  w.statustext(b, 6, longtext.c_str());
  CHECK(b.size() == 8 + 51);
  b.clear();
  uint16_t ch[20];
  for (int i = 0; i < 20; ++i) ch[i] = static_cast<uint16_t>(1000 + i);
  w.rc_channels(b, 1, ch, 20, 255);
  CHECK(b.size() == 8 + 42);
  CHECK(b[6 + 40] == 18);  // chancount
  CHECK(b[6 + 41] == 255);
}

MTEST_MAIN
