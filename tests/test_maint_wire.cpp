#include <cstring>
#include <functional>
#include <vector>
#include "mabur/maint_wire.h"
#include "mtest.h"
using namespace mabur::maint;

namespace {
// The golden frame the Rust port checks against byte for byte.
Frame golden() {
  Frame f;
  f.kind = kData;
  f.session = 0x11223344;
  f.seq = 5;
  f.ack = 7;
  f.flags = kFlagFin;
  f.status = -2;
  f.data = {'o', 'k', '\n'};
  return f;
}
const std::vector<uint8_t> kGolden = {
    0x4D, 0x54, 0x01, 0x02, 0x44, 0x33, 0x22, 0x11, 0x05, 0x00, 0x00, 0x00, 0x07, 0x00,
    0x00, 0x00, 0x01, 0xFE, 0xFF, 0x03, 0x00, 'o',  'k',  '\n'};

std::vector<uint8_t> bytes(size_t n, uint32_t seed) {
  std::vector<uint8_t> v(n);
  for (auto& b : v) {
    seed = seed * 1103515245u + 12345u;
    b = static_cast<uint8_t>(seed >> 16);
  }
  return v;
}

// Two ends joined through pack/parse; `drop(i)` loses the i-th frame on the
// air (both directions counted together).
struct Pipe {
  Stream a, b;
  std::function<bool(int)> drop = [](int) { return false; };
  int n = 0, lost = 0;
  uint64_t now = 0;
  Pipe(Stream::Cfg c) : a(9, c), b(9, c) {}
  void carry(Stream& from, Stream& to) {
    std::vector<Frame> fs;
    from.poll(now, &fs);
    for (auto& f : fs) {
      const auto w = pack(f);
      if (drop(n++)) { ++lost; continue; }
      auto g = parse(w.data(), w.size());
      REQUIRE(g.has_value());
      to.on_frame(*g, now);
    }
  }
  void step() {
    carry(a, b);
    carry(b, a);
    ++now;
  }
};

// Sends `data` a -> b with FIN, reading b as it goes; returns ms taken.
uint64_t transfer(Pipe& p, const std::vector<uint8_t>& data, std::vector<uint8_t>* got,
                  uint64_t limit_ms = 60000) {
  size_t off = 0;
  const uint64_t t0 = p.now;
  while (p.now - t0 < limit_ms) {
    if (off < data.size()) off += p.a.write(data.data() + off, data.size() - off);
    if (off == data.size()) p.a.finish(3);
    p.step();
    p.b.read(got);
    if (p.a.out_done() && p.b.peer_fin()) break;
  }
  return p.now - t0;
}
}  // namespace

TEST(golden_frame_packs_and_parses) {
  const auto w = pack(golden());
  REQUIRE(w.size() == kGolden.size() + 2);
  CHECK(std::memcmp(w.data(), kGolden.data(), kGolden.size()) == 0);
  // The CRC the Rust port pins too.
  CHECK(w[24] == 0x8D && w[25] == 0x5D);
  auto f = parse(w.data(), w.size());
  REQUIRE(f.has_value());
  CHECK(f->kind == kData && f->session == 0x11223344 && f->seq == 5 && f->ack == 7);
  CHECK(f->flags == kFlagFin && f->status == -2 && f->data.size() == 3);
}

TEST(parse_refuses_bad_frames_and_ignores_a_trailing_fcs) {
  auto w = pack(golden());
  auto fcs = w;
  fcs.insert(fcs.end(), {1, 2, 3, 4});
  CHECK(parse(fcs.data(), fcs.size()).has_value());
  auto bad = w;
  bad[21] ^= 1;
  CHECK(!parse(bad.data(), bad.size()).has_value());
  auto ver = w;
  ver[2] = 2;
  CHECK(!parse(ver.data(), ver.size()).has_value());
  CHECK(!is_maint(ver.data(), ver.size()));
  CHECK(!parse(w.data(), w.size() - 1).has_value());
  Frame big;
  big.data.assign(kMaxData, 0);
  auto wb = pack(big);
  CHECK(parse(wb.data(), wb.size()).has_value());
  wb[19] = static_cast<uint8_t>((kMaxData + 1) & 0xFF);
  wb[20] = static_cast<uint8_t>((kMaxData + 1) >> 8);
  CHECK(!parse(wb.data(), wb.size()).has_value());
  // An RC frame is not a maintenance frame.
  const uint8_t rc[] = {0x43, 0x52, 18, 1, 0, 0};
  CHECK(!is_maint(rc, sizeof rc));
}

TEST(clean_transfer_delivers_every_byte_and_the_fin_status) {
  Pipe p(Stream::Cfg{});
  const auto data = bytes(300 * 1000, 1);
  std::vector<uint8_t> got;
  transfer(p, data, &got);
  CHECK(got == data);
  CHECK(p.b.peer_fin() && p.b.peer_status() == 3);
  CHECK(p.a.out_done());
  CHECK(p.a.resends() == 0);
}

TEST(lossy_air_still_delivers_in_order_both_ways) {
  Pipe p(Stream::Cfg{});
  uint32_t s = 7;
  p.drop = [&s](int) {
    s = s * 1103515245u + 12345u;
    return (s >> 16) % 100 < 20;  // 20 % loss, acks included
  };
  const auto up = bytes(100 * 1000, 2);
  const auto down = bytes(20 * 1000, 3);
  std::vector<uint8_t> got_b, got_a;
  size_t ua = 0, ub = 0;
  for (int i = 0; i < 60000 && !(p.a.out_done() && p.b.out_done() && p.a.peer_fin() && p.b.peer_fin()); ++i) {
    if (ua < up.size()) ua += p.a.write(up.data() + ua, up.size() - ua);
    if (ua == up.size()) p.a.finish(0);
    if (ub < down.size()) ub += p.b.write(down.data() + ub, down.size() - ub);
    if (ub == down.size()) p.b.finish(42);
    p.step();
    p.b.read(&got_b);
    p.a.read(&got_a);
  }
  CHECK(p.lost > 0);
  CHECK(got_b == up);
  CHECK(got_a == down);
  CHECK(p.a.peer_status() == 42 && p.b.peer_status() == 0);
  CHECK(p.a.resends() > 0);
}

TEST(a_lost_frame_is_resent_on_duplicate_acks_before_the_timeout) {
  Stream::Cfg c;
  c.rto_ms = 10000;  // the timeout alone would take 10 s
  Pipe p(c);
  p.drop = [](int i) { return i == 0; };  // the first data frame
  const auto data = bytes(10 * kMaxData, 4);
  std::vector<uint8_t> got;
  const uint64_t ms = transfer(p, data, &got);
  CHECK(got == data);
  CHECK(ms < 100);
  CHECK(p.a.resends() == 1);
}

TEST(a_reader_that_stops_holds_the_sender_and_loses_nothing) {
  Stream::Cfg c;
  c.recv_cap = 4 * kMaxData;
  c.rto_ms = 20;
  Pipe p(c);
  const auto data = bytes(40 * kMaxData, 5);
  size_t off = 0;
  for (int i = 0; i < 500; ++i) {  // nobody reads b
    if (off < data.size()) off += p.a.write(data.data() + off, data.size() - off);
    p.step();
  }
  CHECK(p.b.readable() <= c.recv_cap);
  CHECK(p.a.unacked() > 0);
  std::vector<uint8_t> got;
  for (int i = 0; i < 5000 && !(p.a.out_done() && p.b.peer_fin()); ++i) {
    if (off < data.size()) off += p.a.write(data.data() + off, data.size() - off);
    if (off == data.size()) p.a.finish(0);
    p.step();
    p.b.read(&got);
  }
  CHECK(got == data);
}

TEST(fin_without_data_and_a_pure_ack_carry_no_bytes) {
  Pipe p(Stream::Cfg{});
  p.a.finish(-1);
  std::vector<uint8_t> got;
  for (int i = 0; i < 20; ++i) p.step();
  p.b.read(&got);
  CHECK(got.empty());
  CHECK(p.b.peer_fin() && p.b.peer_status() == -1);
  CHECK(p.a.out_done());
  // A frame of another session is not ours.
  Frame f;
  f.session = 10;
  f.seq = 0;
  f.data = {1};
  p.b.on_frame(f, p.now);
  CHECK(p.b.readable() == 0);
}

// The Rust port (pal-wire maint.rs) runs this same scenario and must produce
// the same frames: an FNV-1a over every packed frame, dropped ones included.
TEST(trace_hash_pins_the_frame_sequence_for_the_rust_port) {
  Stream a(9), b(9);
  const auto up = bytes(30000, 11), down = bytes(5000, 12);
  size_t ua = 0, ub = 0;
  uint32_t s = 7, h = 2166136261u;
  uint64_t now = 0;
  std::vector<uint8_t> got_a, got_b;
  auto carry = [&](Stream& from, Stream& to) {
    std::vector<Frame> fs;
    from.poll(now, &fs);
    for (auto& f : fs) {
      const auto w = pack(f);
      for (uint8_t x : w) h = (h ^ x) * 16777619u;
      s = s * 1103515245u + 12345u;
      if ((s >> 16) % 100 < 20) continue;
      to.on_frame(*parse(w.data(), w.size()), now);
    }
  };
  for (; now < 20000 && !(a.out_done() && b.out_done() && a.peer_fin() && b.peer_fin()); ++now) {
    if (ua < up.size()) ua += a.write(up.data() + ua, up.size() - ua);
    if (ua == up.size()) a.finish(0);
    if (ub < down.size()) ub += b.write(down.data() + ub, down.size() - ub);
    if (ub == down.size()) b.finish(42);
    carry(a, b);
    carry(b, a);
    a.read(&got_a);
    b.read(&got_b);
  }
  CHECK(got_b == up && got_a == down);
  std::printf("trace: now=%llu hash=%08x\n", static_cast<unsigned long long>(now), h);
  CHECK(now == 1510 && h == 0x6aea47cc);
}

MTEST_MAIN
