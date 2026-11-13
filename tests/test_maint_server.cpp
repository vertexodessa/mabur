#include <string>
#include <vector>
#include "maint_server.h"
#include "mtest.h"
using namespace mabur::maint;

namespace {
// A command that echoes stdin back upper-cased and exits with 7 at stdin EOF.
struct FakeChild : MaintChild {
  std::string in, out;
  bool in_closed = false, killed = false;
  size_t write_stdin(const uint8_t* p, size_t n) override {
    in.append(reinterpret_cast<const char*>(p), n);
    for (size_t i = 0; i < n; ++i) out.push_back(static_cast<char>(toupper(p[i])));
    return n;
  }
  void close_stdin() override { in_closed = true; }
  size_t read_out(uint8_t* buf, size_t cap) override {
    const size_t n = std::min(cap, out.size());
    std::copy(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n), buf);
    out.erase(0, n);
    return n;
  }
  bool out_eof() const override { return in_closed && out.empty(); }
  std::optional<int> exited() override {
    if (killed) return 137;
    return in_closed ? std::optional<int>(7) : std::nullopt;
  }
  void kill() override { killed = true; }
};

struct World {
  std::vector<std::string> spawned;
  FakeChild* last = nullptr;
  bool fail_spawn = false;
  MaintServer srv{MaintServerCfg{}, [this](const std::string& cmd) -> std::unique_ptr<MaintChild> {
                    spawned.push_back(cmd);
                    if (fail_spawn) return nullptr;
                    auto c = std::make_unique<FakeChild>();
                    last = c.get();
                    return c;
                  }};
  uint64_t now = 1000;
  bool armed = false;
  std::vector<Frame> to_gs;
  void in(const Frame& f) { srv.on_frame(f, now, armed); }
  void tick() {
    srv.poll(now, armed, &to_gs);
    ++now;
  }
};

Frame open(uint32_t s, const std::string& cmd) {
  Frame f;
  f.kind = kOpen;
  f.session = s;
  f.data.assign(cmd.begin(), cmd.end());
  return f;
}
Frame close_f(uint32_t s) {
  Frame f;
  f.kind = kClose;
  f.session = s;
  return f;
}
}  // namespace

TEST(a_session_runs_the_command_and_streams_both_ways_to_the_exit_status) {
  World w;
  w.in(open(5, "tr a-z A-Z"));
  REQUIRE(w.spawned.size() == 1 && w.spawned[0] == "tr a-z A-Z");
  w.tick();
  REQUIRE(!w.to_gs.empty());
  CHECK(w.to_gs[0].kind == kData && w.to_gs[0].session == 5 && w.to_gs[0].data.empty());
  for (auto& f : w.to_gs) CHECK(f.flags & kFlagFromDrone);
  // A frame that says it is the drone's is never taken as the GS's.
  Frame echo = open(6, "x");
  echo.flags = kFlagFromDrone;
  w.in(echo);
  CHECK(w.spawned.size() == 1);
  // The GS end: a Stream of the same session.
  Stream gs(5);
  const std::string text = "hello drone\n";
  gs.write(reinterpret_cast<const uint8_t*>(text.data()), text.size());
  gs.finish();
  std::vector<uint8_t> got;
  for (int i = 0; i < 200 && !(gs.peer_fin() && gs.out_done()); ++i) {
    std::vector<Frame> up;
    gs.poll(w.now, &up);
    for (auto& f : up) w.in(f);
    w.to_gs.clear();
    w.tick();
    for (auto& f : w.to_gs) gs.on_frame(f, w.now);
    gs.read(&got);
  }
  CHECK(std::string(got.begin(), got.end()) == "HELLO DRONE\n");
  CHECK(w.last->in == text && w.last->in_closed);
  CHECK(gs.peer_fin() && gs.peer_status() == 7);
}

TEST(a_repeated_open_is_answered_not_rerun_and_a_new_session_replaces_the_old) {
  World w;
  w.in(open(5, "a"));
  w.in(open(5, "a"));
  CHECK(w.spawned.size() == 1);
  w.tick();
  CHECK(!w.to_gs.empty());
  w.in(open(6, "b"));
  CHECK(w.spawned.size() == 2);
  CHECK(w.srv.session_open());
}

TEST(armed_refuses_a_session_kills_one_open_and_keeps_video_on) {
  World w;
  w.armed = true;
  w.in(open(5, "x"));
  CHECK(w.spawned.empty());
  w.tick();
  REQUIRE(w.to_gs.size() == 1);
  CHECK(w.to_gs[0].kind == kClose && w.to_gs[0].status == kStatusArmed);
  w.armed = false;
  w.to_gs.clear();
  w.in(open(6, "y"));
  Frame v;
  v.kind = kVideo;
  v.data = {0xE8, 0x03};  // 1000 ms
  w.in(v);
  CHECK(w.srv.video_off(w.now));
  w.armed = true;
  w.tick();
  CHECK(!w.srv.session_open());
  CHECK(!w.srv.video_off(w.now));
  bool told = false;
  for (auto& f : w.to_gs) told |= f.kind == kClose && f.session == 6 && f.status == kStatusArmed;
  CHECK(told);
}

TEST(the_video_lease_expires_by_itself_and_is_clamped) {
  World w;
  Frame v;
  v.kind = kVideo;
  v.data = {0xFF, 0xFF};
  w.in(v);
  w.tick();
  REQUIRE(w.to_gs.size() == 1);
  CHECK(w.to_gs[0].kind == kVideo && w.to_gs[0].data[0] == 0x10 && w.to_gs[0].data[1] == 0x27);  // 10000
  CHECK(w.srv.video_off(w.now + 9000));
  CHECK(!w.srv.video_off(w.now + 10000));
  v.data = {0, 0};
  w.in(v);
  CHECK(!w.srv.video_off(w.now));
}

TEST(unknown_sessions_kinds_and_spawn_failures_are_answered) {
  World w;
  Frame d;
  d.kind = kData;
  d.session = 9;
  d.data = {1};
  w.in(d);
  Frame odd;
  odd.kind = 77;
  odd.session = 9;
  w.in(odd);
  w.fail_spawn = true;
  w.in(open(3, "z"));
  w.tick();
  REQUIRE(w.to_gs.size() == 3);
  CHECK(w.to_gs[0].kind == kClose && w.to_gs[0].status == kStatusUnknown);
  CHECK(w.to_gs[1].kind == kClose && w.to_gs[1].status == kStatusUnsupported);
  CHECK(w.to_gs[2].kind == kClose && w.to_gs[2].status == kStatusSpawnFailed);
  CHECK(!w.srv.session_open());
}

TEST(close_and_silence_end_the_session) {
  World w;
  w.in(open(4, "q"));
  w.in(close_f(4));
  CHECK(!w.srv.session_open());
  w.in(open(8, "r"));
  w.now += 30000;
  w.tick();
  CHECK(!w.srv.session_open());
  bool idle = false;
  for (auto& f : w.to_gs) idle |= f.kind == kClose && f.session == 8 && f.status == kStatusIdle;
  CHECK(idle);
}

// The real /bin/sh on the host: output, stdin, exit code.
TEST(spawn_shell_runs_sh_with_pipes_and_reports_the_exit_code) {
  auto c = spawn_shell("read x; echo got:$x; echo err >&2; exit 3");
  REQUIRE(c != nullptr);
  const std::string line = "abc\n";
  CHECK(c->write_stdin(reinterpret_cast<const uint8_t*>(line.data()), line.size()) == line.size());
  c->close_stdin();
  std::string out;
  std::optional<int> st;
  for (int i = 0; i < 2000 && !(c->out_eof() && st); ++i) {
    uint8_t buf[256];
    const size_t n = c->read_out(buf, sizeof buf);
    out.append(reinterpret_cast<char*>(buf), n);
    if (c->out_eof()) st = c->exited();
    if (n == 0) usleep(1000);
  }
  CHECK(out == "got:abc\nerr\n");
  CHECK(st && *st == 3);
  auto k = spawn_shell("sleep 30");
  REQUIRE(k != nullptr);
  k->kill();
  std::optional<int> ks;
  for (int i = 0; i < 2000 && !ks; ++i) {
    ks = k->exited();
    if (!ks) usleep(1000);
  }
  CHECK(ks && *ks == 128 + 9);
}

MTEST_MAIN
