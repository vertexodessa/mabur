#include "mtest.h"
#include "cal_sweep.h"
#include "fake_floor_unit.h"
#include "mabur/cal_floor.h"
#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"
#include "radio_tx.h"

#include <array>
#include <set>
#include <utility>
#include <vector>

using namespace mabur;

namespace {

// Captures every built frame so the test can parse the payloads back out.
struct CaptureSink : FrameSink {
  std::vector<std::vector<uint8_t>> frames;
  bool send(const uint8_t* f, size_t n) override {
    frames.emplace_back(f, f + n);
    return true;
  }
};

struct FakePowerCtl : CalSweep::PowerCtl {
  std::vector<int> index_writes;
  int zero_diff_calls = 0;
  int read_anchor_calls = 0;
  int anchor = 53;
  bool set_index_override(int idx) override {
    index_writes.push_back(idx);
    return true;
  }
  bool zero_ok = true;
  bool zero_rate_diffs() override { ++zero_diff_calls; return zero_ok; }
  // The anchor read at 40 MHz when anchor40 >= 0 (the chip's reference is
  // per width); `anchor` otherwise.
  int anchor40 = -1;
  int read_anchor_idx() override {
    ++read_anchor_calls;
    return (width == 40 && anchor40 >= 0) ? anchor40 : anchor;
  }
  // 0 = an owner that does not retune (every pre-existing test).
  uint8_t width = 0;
  bool has_pair = true;
  std::vector<uint8_t> tunes;  // every width tune_width moved the radio to
  uint8_t tuned_width() override { return width; }
  int tune_width(uint8_t bw) override {
    if (bw == width) return 0;
    if (bw == 40 && !has_pair) return -1;
    width = bw;
    tunes.push_back(bw);
    return 1;
  }
};

struct FakeCapacityCtl : CalSweep::CapacityCtl {
  std::vector<std::pair<uint8_t, uint8_t>> points;  // (mcs, bw) per set_point call
  std::vector<std::vector<uint8_t>> blasts;         // captured frames per blast call
  BlastResult to_return;
  bool set_point(uint8_t mcs, uint8_t bw) override {
    points.emplace_back(mcs, bw);
    return true;
  }
  BlastResult blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) override {
    (void)dwell_ms;
    blasts.emplace_back(frame, frame + len);
    return to_return;
  }
};

cal::SweepCmd power_cmd(uint32_t nonce, uint8_t seq, uint8_t mcs_mask = 0xFF, int8_t lo = -41,
                       int8_t hi = 63, uint8_t bw = 20, uint16_t frames_per_cell = 2,
                       uint16_t settle_ms = 0, uint16_t gap_us = 0) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = cal::Kind::Power;
  c.bw = bw;
  c.frames_per_cell = frames_per_cell;
  c.settle_ms = settle_ms;
  c.gap_us = gap_us;
  c.idx_lo = lo;
  c.idx_hi = hi;
  c.mcs_mask = mcs_mask;
  return c;
}

cal::SweepCmd capacity_cmd(uint32_t nonce, uint8_t seq, uint8_t mcs_mask = 0xFF, uint8_t bw = 20,
                          uint32_t dwell_ms = 0, uint16_t settle_ms = 0) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = cal::Kind::Capacity;
  c.bw = bw;
  c.mcs_mask = mcs_mask;
  c.dwell_ms = dwell_ms;
  c.settle_ms = settle_ms;
  return c;
}

// Runs pump() until the sweep leaves Sweeping or the budget is spent.
void run_to_quiescence(CalSweep& s, RadioTx& tx, FakePowerCtl& pwr, FakeCapacityCtl& cap,
                       uint64_t start_ms = 0, uint64_t step_ms = 1, int max_steps = 1000000) {
  uint64_t t = start_ms;
  for (int i = 0; i < max_steps && s.state() == CalSweep::State::Sweeping; ++i) {
    s.pump(t, tx, pwr, cap);
    t += step_ms;
  }
}

// Parses a captured frame's calibration payload. Frames are
// radiotap | dot11(26) | body, so the body is found by scanning for the
// magic rather than by hard-coding a header length the builder owns.
bool payload_of(const std::vector<uint8_t>& frame, cal::CalFrameInfo* out) {
  for (size_t i = 0; i + cal::kCalHeaderLen <= frame.size(); ++i)
    if (cal::parse_cal_payload(frame.data() + i, frame.size() - i, out)) return true;
  return false;
}

}  // namespace

TEST(power_sweep_walks_every_cell_and_stamps_kind_bw_flags) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, /*mask*/ 0x03, /*lo*/ -2, /*hi*/ 1, /*bw*/ 40, /*fpc*/ 2);
  s.on_cmd(c, 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  CHECK(ack->flags == 3);  // kFlagStbc | kFlagLdpc (radio.ldpc defaults true)
  run_to_quiescence(s, tx, pwr, cap);
  CHECK(sink.frames.size() == 16);  // 4 idx x 2 mcs x 2 frames/cell
  std::set<std::pair<int, int>> seen;
  for (const auto& f : sink.frames) {
    cal::CalFrameInfo info;
    REQUIRE(payload_of(f, &info));
    CHECK(info.kind == cal::Kind::Power);
    CHECK(info.bw == 40);
    CHECK(info.flags == 3);
    CHECK(info.rate <= 1);
    CHECK(info.idx >= -2 && info.idx <= 1);
    seen.insert({info.rate, info.idx});
  }
  CHECK(seen.size() == 8);  // every (mcs, idx) cell visited exactly once
}

TEST(capacity_sweep_sets_each_point_and_reports_offered_and_blocked) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  cap.to_return = BlastResult{500, 90};
  CalSweep s(CalSweepCfg{});
  auto c = capacity_cmd(1, 0, /*mask*/ 0xFF, /*bw*/ 20, /*dwell*/ 0, /*settle*/ 0);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  REQUIRE(cap.points.size() == 8);
  for (int m = 0; m < 8; ++m) {
    CHECK(cap.points[m].first == m);
    CHECK(cap.points[m].second == 20);
  }
  auto done = s.take_done();
  REQUIRE(done.has_value());
  for (int m = 0; m < 8; ++m) {
    CHECK(done->offered[m] == 500);
    CHECK(done->blocked_pct[m] == 90);
  }
}

TEST(a_second_power_sweep_zeroes_again_but_reads_the_anchor_once) {
  FakePowerCtl pwr;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0);
  s.on_cmd(c, 0, pwr);
  CHECK(pwr.zero_diff_calls == 1);
  CHECK(pwr.read_anchor_calls == 1);
  c.seq = 1;  // next power sweep, same run
  s.on_cmd(c, 1000, pwr);
  CHECK(pwr.zero_diff_calls == 2);
  CHECK(pwr.read_anchor_calls == 1);  // not re-read
}

// A run's anchor is read with the diffs zeroed whatever kind opens it (a
// custom table live would read reference + diff), so a capacity-first run
// zeroes once, before its one anchor read; after that neither a later
// capacity sweep of the run nor a verify zeroes.
TEST(capacity_first_run_zeroes_before_the_anchor_read_then_verify_and_capacity_do_not) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = capacity_cmd(1, 0, 0x01, 20, /*dwell*/ 0, /*settle*/ 0);
  s.on_cmd(c, 0, pwr);
  CHECK(pwr.zero_diff_calls == 1);
  CHECK(pwr.read_anchor_calls == 1);
  CHECK(s.anchor_idx() == 53);
  run_to_quiescence(s, tx, pwr, cap);  // let the Capacity sweep finish: arm_verify refuses mid-sweep
  CHECK(pwr.zero_diff_calls == 1);

  auto c2 = capacity_cmd(1, 1, 0x01, 40, /*dwell*/ 0, /*settle*/ 0);
  s.on_cmd(c2, 2000, pwr);  // a later capacity sweep of the same run
  CHECK(pwr.zero_diff_calls == 1);
  CHECK(pwr.read_anchor_calls == 1);
  run_to_quiescence(s, tx, pwr, cap);

  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 5;
  s.arm_verify(park, 20, 5000);
  CHECK(pwr.zero_diff_calls == 1);
  CHECK(s.state() == CalSweep::State::Sweeping);
}

// The table is flattened before every new run's anchor read, so a refusal
// owes the caller a restore whatever kind was refused.
TEST(unreadable_anchor_on_a_capacity_command_also_flags_the_refusal) {
  FakePowerCtl pwr;
  pwr.anchor = -1;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(capacity_cmd(1, 0), 0, pwr);
  CHECK(!s.active());
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(pwr.zero_diff_calls == 1);
  CHECK(s.take_refused());
}

// A table that would not zero leaves reference + diff where the anchor
// read expects the reference: every wall of the run would shift by its
// rate's diff. Refused like an unreadable anchor, and the owner restores.
TEST(a_table_that_would_not_zero_refuses_a_new_run) {
  FakePowerCtl pwr;
  pwr.zero_ok = false;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(1, 0), 0, pwr);
  CHECK(!s.active());
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(pwr.read_anchor_calls == 0);  // a read off a live table is never taken
  CHECK(s.take_refused());
  CHECK(pwr.index_writes.empty());
}

TEST(a_table_that_would_not_zero_refuses_a_later_power_sweep_and_closes_the_run) {
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(capacity_cmd(1, 0), 0, pwr);
  REQUIRE(s.take_ack()->result == cal::AckResult::Accepted);
  REQUIRE(s.active());
  pwr.zero_ok = false;
  s.on_cmd(power_cmd(1, 1, 0xFF, -41, 63, 40), 100, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(ack->seq == 1);
  CHECK(!s.active());
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(s.take_refused());
  // The run is closed: the GS's resend of the refused command does not
  // reopen it.
  pwr.zero_ok = true;
  s.on_cmd(power_cmd(1, 1, 0xFF, -41, 63, 40), 300, pwr);
  CHECK(!s.take_ack().has_value());
  CHECK(!s.active());
}

TEST(seq_behind_is_ignored_equal_rearms_ack_ahead_starts) {
  FakePowerCtl pwr;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 5);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());

  // Equal seq: re-arm the ack, no restart (no extra zero).
  s.on_cmd(c, 10, pwr);
  auto ack2 = s.take_ack();
  REQUIRE(ack2.has_value());
  CHECK(ack2->result == cal::AckResult::Accepted);
  CHECK(pwr.zero_diff_calls == 1);  // not re-entered

  // Behind seq: ignored outright, no ack.
  auto behind = c;
  behind.seq = 4;
  s.on_cmd(behind, 20, pwr);
  CHECK(!s.take_ack().has_value());

  // Ahead seq: a new sweep starts.
  auto ahead = c;
  ahead.seq = 6;
  ahead.mcs_mask = 0x02;
  s.on_cmd(ahead, 30, pwr);
  auto ack3 = s.take_ack();
  REQUIRE(ack3.has_value());
  CHECK(ack3->result == cal::AckResult::Accepted);
  CHECK(s.state() == CalSweep::State::Sweeping);
  CHECK(pwr.zero_diff_calls == 2);  // the new sweep zeroed again (Power kind)
}

TEST(done_is_armed_once_after_the_last_cell) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 3);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(!s.take_done().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  CHECK(done->nonce == 1);
  CHECK(done->seq == 0);
  CHECK(done->offered[0] == 3);
  CHECK(!s.take_done().has_value());  // drained once
}

// Pumps a one-cell power sweep from acceptance at t = 0 until it leaves
// Sweeping; returns the ms the sweep finished at (its SweepDone armed).
uint64_t finish_one_cell(CalSweep& s, RadioTx& tx, FakePowerCtl& pwr, FakeCapacityCtl& cap,
                         uint32_t nonce, uint8_t seq) {
  s.on_cmd(power_cmd(nonce, seq, 0x01, 0, 0, 20, 3), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  uint64_t t = 0;
  while (s.state() == CalSweep::State::Sweeping && t < 10000) s.pump(t++, tx, pwr, cap);
  REQUIRE(s.state() == CalSweep::State::Idle);
  return t - 1;
}

TEST(done_repeats_from_100_ms_then_every_200_ms_until_the_next_command) {
  // SweepDone rides a lossy downlink, and a capacity sweep without it writes
  // nothing: it is re-armed 100 ms after the sweep ends -- half a period
  // off the GS's 200 ms resend grid -- then every 200 ms until the GS shows
  // it moved on.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  CalSweep s(CalSweepCfg{});
  const uint64_t end = finish_one_cell(s, tx, pwr, cap, 7, 0);
  const auto first = s.take_done();
  REQUIRE(first.has_value());
  for (uint64_t t = end + 1; t < end + 100; ++t) {
    s.pump(t, tx, pwr, cap);
    CHECK(!s.take_done().has_value());
  }
  s.pump(end + 100, tx, pwr, cap);
  const auto again = s.take_done();
  REQUIRE(again.has_value());
  CHECK(again->nonce == 7 && again->seq == 0 && again->offered == first->offered);
  s.pump(end + 299, tx, pwr, cap);
  CHECK(!s.take_done().has_value());
  s.pump(end + 300, tx, pwr, cap);
  CHECK(s.take_done().has_value());
  s.pump(end + 499, tx, pwr, cap);
  CHECK(!s.take_done().has_value());
  s.pump(end + 500, tx, pwr, cap);
  CHECK(s.take_done().has_value());

  // The next command of the run -- here the APPLY -- stops it.
  cal::ApplyCmd a;
  a.nonce = 7;
  a.seq = 0;
  s.on_apply(a, end + 550);
  REQUIRE(s.take_pending_apply().has_value());
  for (uint64_t t = end + 550; t < end + 2000; t += 10) {
    s.pump(t, tx, pwr, cap);
    CHECK(!s.take_done().has_value());
  }
}

TEST(done_repeats_stop_at_a_sweep_an_abort_or_the_close) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  auto silent_after = [&](CalSweep& s, uint64_t from) {
    bool quiet = true;
    for (uint64_t t = from; t < from + 1000; t += 10) {
      s.pump(t, tx, pwr, cap);
      if (s.take_done()) quiet = false;
    }
    return quiet;
  };
  {
    // Any SWEEP of the run, even a repeat of the finished one.
    CalSweep s(CalSweepCfg{});
    const uint64_t end = finish_one_cell(s, tx, pwr, cap, 8, 0);
    REQUIRE(s.take_done().has_value());
    s.on_cmd(power_cmd(8, 0, 0x01, 0, 0, 20, 3), end + 10, pwr);
    (void)s.take_ack();
    CHECK(silent_after(s, end + 10));
  }
  {
    // An ABORT of the run.
    CalSweep s(CalSweepCfg{});
    const uint64_t end = finish_one_cell(s, tx, pwr, cap, 9, 0);
    REQUIRE(s.take_done().has_value());
    cal::Abort ab;
    ab.nonce = 9;
    s.on_abort(ab, end + 10, pwr);
    CHECK(silent_after(s, end + 10));
  }
  {
    // The await timer closing the run.
    CalSweepCfg cfg;
    cfg.await_next_ms = 1000;
    CalSweep s(cfg);
    const uint64_t end = finish_one_cell(s, tx, pwr, cap, 10, 0);
    REQUIRE(s.take_done().has_value());
    int repeats = 0;
    for (uint64_t t = end + 1; t < end + 3000; ++t) {
      s.pump(t, tx, pwr, cap);
      if (s.take_done()) ++repeats;
    }
    CHECK(!s.active());
    CHECK(repeats == 5);  // +100, +300 .. +900; the run closed at +1000
  }
}

TEST(abort_restores_power_immediately) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);  // would never finish on its own
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  s.pump(0, tx, pwr, cap);
  s.pump(1, tx, pwr, cap);
  CHECK(s.state() == CalSweep::State::Sweeping);

  cal::Abort a{1};
  s.on_abort(a, 2, pwr);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
  CHECK(s.power_restored_for_test());
  CHECK(pwr.index_writes.back() == 53);
}

TEST(hard_cap_restores_power_with_the_gs_silent) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweepCfg cfg;
  cfg.hard_cap_ms = 500;
  CalSweep s(cfg);
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);
  s.on_cmd(c, 0, pwr);
  CHECK(s.state() == CalSweep::State::Sweeping);
  s.pump(600, tx, pwr, cap);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
  CHECK(s.power_restored_for_test());
}

TEST(unreadable_anchor_refuses_with_anchor_unreadable) {
  FakePowerCtl pwr;
  pwr.anchor = -1;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0);
  s.on_cmd(c, 0, pwr);
  CHECK(!s.active());
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(ack->nonce == 1);
  CHECK(ack->seq == 0);
  CHECK(s.take_refused());
  CHECK(!s.take_refused());               // drained: one refusal, one restore
  CHECK(pwr.zero_diff_calls == 1);         // flattened before the failed read
  CHECK(pwr.index_writes.empty());
}

TEST(arm_verify_skips_leave_entries) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  // Open a run first -- arm_verify is called after an apply within a live
  // run, never standing alone in production.
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  sink.frames.clear();

  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 10;
  park[3] = -5;
  s.arm_verify(park, 20, 1000);
  CHECK(s.state() == CalSweep::State::Sweeping);
  run_to_quiescence(s, tx, pwr, cap, 1000);

  CHECK(!sink.frames.empty());
  std::set<int> mcs_seen;
  for (const auto& f : sink.frames) {
    cal::CalFrameInfo info;
    REQUIRE(payload_of(f, &info));
    CHECK(info.kind == cal::Kind::Verify);
    CHECK((info.rate == 0 || info.rate == 3));
    if (info.rate == 0) CHECK(info.idx == 10);
    if (info.rate == 3) CHECK(info.idx == -5);
    mcs_seen.insert(info.rate);
  }
  CHECK(mcs_seen.size() == 2);        // only the two parked MCS, every other skipped
  CHECK(!s.take_done().has_value());  // Verify emits no SweepDone
}

// --- Fix round 1 ---------------------------------------------------------

TEST(start_delay_blocks_frames_and_set_point_before_t200) {
  // Power/Verify: no frame airs before the 200 ms start deadline.
  {
    CaptureSink sink;
    RadioTx tx(sink);
    FakePowerCtl pwr;
    FakeCapacityCtl cap;
    pwr.anchor = 53;
    CalSweep s(CalSweepCfg{});
    auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);
    s.on_cmd(c, 0, pwr);
    REQUIRE(s.take_ack().has_value());
    for (uint64_t t = 0; t < 200; t += 20) s.pump(t, tx, pwr, cap);
    CHECK(sink.frames.empty());
    CHECK(pwr.index_writes.empty());  // the cell was never entered either
    run_to_quiescence(s, tx, pwr, cap, 200);
    CHECK(!sink.frames.empty());
  }
  // Capacity: no set_point before the same deadline.
  {
    CaptureSink sink;
    RadioTx tx(sink);
    FakePowerCtl pwr;
    FakeCapacityCtl cap;
    pwr.anchor = 53;
    CalSweep s(CalSweepCfg{});
    auto c = capacity_cmd(1, 0, 0x01, 20, /*dwell*/ 0, /*settle*/ 0);
    s.on_cmd(c, 0, pwr);
    REQUIRE(s.take_ack().has_value());
    for (uint64_t t = 0; t < 200; t += 20) s.pump(t, tx, pwr, cap);
    CHECK(cap.points.empty());
    run_to_quiescence(s, tx, pwr, cap, 200);
    CHECK(!cap.points.empty());
  }
}

TEST(a_closed_nonce_does_not_reopen) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);  // never finishes on its own
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap, 0, 1, 300);  // past the 200 ms start delay, mid-sweep

  cal::Abort a{1};
  s.on_abort(a, 300, pwr);
  REQUIRE(s.power_restored_for_test());
  const size_t writes_before = pwr.index_writes.size();
  const size_t frames_before = sink.frames.size();

  // A straggler/resend of the aborted nonce+seq arrives after the close.
  s.on_cmd(c, 301, pwr);
  CHECK(!s.active());
  CHECK(!s.take_ack().has_value());
  CHECK(pwr.index_writes.size() == writes_before);  // power untouched by the attempt
  s.pump(301, tx, pwr, cap);
  s.pump(302, tx, pwr, cap);
  CHECK(sink.frames.size() == frames_before);  // nothing airs
  CHECK(pwr.index_writes.size() == writes_before);
  CHECK(s.power_restored_for_test());  // still restored -- no new run reopened
}

TEST(arm_verify_without_a_session_is_a_no_op) {
  CalSweep s(CalSweepCfg{});
  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 5;
  s.arm_verify(park, 20, 0);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
}

TEST(arm_verify_is_refused_while_a_sweep_is_sweeping) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);  // stays Sweeping
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(s.state() == CalSweep::State::Sweeping);

  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 5;
  s.arm_verify(park, 20, 0);
  CHECK(s.state() == CalSweep::State::Sweeping);  // unchanged -- still the Power sweep
  CHECK(s.active());

  // Prove it's still the Power sweep, not a Verify sweep that silently
  // replaced it: the next frame sent still carries kind == Power.
  run_to_quiescence(s, tx, pwr, cap, 0, 1, 500);
  REQUIRE(!sink.frames.empty());
  cal::CalFrameInfo info;
  REQUIRE(payload_of(sink.frames.back(), &info));
  CHECK(info.kind == cal::Kind::Power);
}

TEST(capacity_sweep_restores_power_to_the_anchor_before_its_first_point) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto power = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 63, /*hi*/ 63, /*bw*/ 20, /*fpc*/ 1);
  s.on_cmd(power, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  CHECK(pwr.index_writes.back() == 53 + 63);  // parked away from the anchor

  auto capacity = capacity_cmd(1, 1, 0x01, 20, /*dwell*/ 0, /*settle*/ 0);
  s.on_cmd(capacity, 1000, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(pwr.index_writes.back() == 53);  // restored to the anchor immediately on accept

  run_to_quiescence(s, tx, pwr, cap, 1000);
  REQUIRE(!cap.points.empty());
  CHECK(pwr.index_writes.back() == 53);  // still at the anchor through the point
}

TEST(repeated_apply_already_handed_out_does_not_disturb_a_running_verify) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);

  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  s.on_apply(a, 2000);
  CHECK(s.state() == CalSweep::State::Applying);
  REQUIRE(s.take_pending_apply().has_value());
  CHECK(s.state() == CalSweep::State::Idle);

  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 7;
  s.arm_verify(park, 20, 2100);
  CHECK(s.state() == CalSweep::State::Sweeping);

  // The GS resends the same APPLY (it hasn't heard APPLIED yet) while
  // the verify sweep it already triggered is running.
  s.on_apply(a, 2200);
  CHECK(s.state() == CalSweep::State::Sweeping);     // undisturbed
  CHECK(!s.take_pending_apply().has_value());        // not re-queued

  // And after the verify sweep finishes (back to Idle), a further
  // resend of that exact (nonce, bw) is still ignored on its own merits
  // -- not just because a sweep happened to be running.
  run_to_quiescence(s, tx, pwr, cap, 2200);
  CHECK(s.state() == CalSweep::State::Idle);
  s.on_apply(a, 5000);
  CHECK(s.state() == CalSweep::State::Idle);          // did not re-enter Applying
  CHECK(!s.take_pending_apply().has_value());
}

TEST(a_new_seq_at_the_same_width_is_a_new_apply) {
  // power20 then cap20 in one run: both apply at 20 MHz, so the handed-out
  // latch must key on the sweep's seq, not on the width.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(1, 0, 0x01, 0, 0, 20, 1), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);

  cal::ApplyCmd walls;
  walls.nonce = 1;
  walls.seq = 0;
  walls.bw = 20;
  s.on_apply(walls, 2000);
  REQUIRE(s.take_pending_apply().has_value());

  s.on_cmd(capacity_cmd(1, 1, 0x01, 20, /*dwell*/ 0, /*settle*/ 0), 3000, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap, 3000);
  REQUIRE(s.state() == CalSweep::State::Idle);

  cal::ApplyCmd eff;
  eff.nonce = 1;
  eff.seq = 1;
  eff.bw = 20;
  eff.has_eff = true;
  s.on_apply(eff, 6000);
  CHECK(s.state() == CalSweep::State::Applying);
  const auto got = s.take_pending_apply();
  REQUIRE(got.has_value());
  CHECK(got->seq == 1 && got->has_eff);

  // Its own repeat is then ignored, as the power apply's was.
  s.on_apply(eff, 6200);
  CHECK(!s.take_pending_apply().has_value());
}

TEST(await_timeout_while_applying_closes_the_run) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweepCfg cfg;
  cfg.await_next_ms = 100;
  CalSweep s(cfg);
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);

  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  s.on_apply(a, 1000);
  CHECK(s.state() == CalSweep::State::Applying);
  s.pump(1200, tx, pwr, cap);  // past await_deadline_ms_ (1000 + 100)
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
  CHECK(s.power_restored_for_test());
}

TEST(on_apply_is_ignored_while_a_sweep_is_sweeping) {
  FakePowerCtl pwr;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);  // stays Sweeping
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(s.state() == CalSweep::State::Sweeping);

  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  s.on_apply(a, 100);
  CHECK(s.state() == CalSweep::State::Sweeping);  // undisturbed
  CHECK(!s.take_pending_apply().has_value());     // not queued
}

TEST(an_ahead_seq_sweep_clears_a_stale_pending_apply) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);

  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  s.on_apply(a, 1000);
  CHECK(s.state() == CalSweep::State::Applying);

  auto c2 = power_cmd(1, 1, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);  // ahead seq
  s.on_cmd(c2, 1100, pwr);
  CHECK(s.state() == CalSweep::State::Sweeping);
  CHECK(!s.take_pending_apply().has_value());  // the stale apply was cleared, not handed out
}

TEST(power_restored_flag_resets_when_a_new_run_opens) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap, 0, 1, 300);

  cal::Abort a{1};
  s.on_abort(a, 300, pwr);
  CHECK(s.power_restored_for_test());

  auto c2 = power_cmd(2, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);  // a different, new nonce
  s.on_cmd(c2, 301, pwr);
  CHECK(!s.power_restored_for_test());  // reset for the new run
}

// starts_sweep() predicts on_cmd(): a new nonce (unless it is the one just
// closed) or a seq ahead of the running one starts a sweep; a repeat or a
// late duplicate does not. run_nonce() names the open run.
TEST(starts_sweep_predicts_on_cmd_and_run_nonce_names_the_run) {
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  CHECK(s.starts_sweep(power_cmd(5, 0)));
  s.on_cmd(power_cmd(5, 1), 1000, pwr);
  REQUIRE(s.active());
  CHECK(s.run_nonce() == 5);
  CHECK(!s.starts_sweep(power_cmd(5, 1)));  // repeat
  CHECK(!s.starts_sweep(power_cmd(5, 0)));  // late duplicate
  CHECK(s.starts_sweep(capacity_cmd(5, 2)));
  CHECK(s.starts_sweep(power_cmd(6, 0)));   // another run
  s.on_abort(cal::Abort{5}, 1100, pwr);
  REQUIRE(!s.active());
  CHECK(!s.starts_sweep(power_cmd(5, 3)));  // the closed nonce never reopens
  CHECK(s.starts_sweep(power_cmd(6, 0)));
}


// ---- floor sweep (cal_floor.h) -------------------------------------------

namespace {

void run_floor(CalSweep& s, RadioTx& tx, FakePowerCtl& pwr, test::FakeFloorUnit& unit) {
  for (uint64_t t = 0; t < 600000 && s.state() == CalSweep::State::Sweeping; t += 5)
    s.pump(t, tx, pwr, unit);
}

}  // namespace

TEST(floor_sweep_skips_a_marginal_value_and_reports_the_edge) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.edge = 0x24;
  unit.marginal = 0x22;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.ref_fps == 2484);
  CHECK(r.floor == 0x24);  // 0x22 passed its 4 s dwell and failed its 8 s one
  CHECK(r.tried == 3);
  CHECK(r.min_fps[0] == 0);
  CHECK(r.min_fps[1] == 2484);
  CHECK(r.min_fps[2] == 2484);
  CHECK(r.confirm_pct == 100);
  // Reference, 0x20, 0x22 + its confirmation, 0x24 + its confirmation.
  REQUIRE(unit.blasts.size() == 6);
  CHECK(unit.blasts[0] == std::make_pair(cal::kFloorRef, cal::kFloorRefDwellMs));
  CHECK(unit.blasts[3] == std::make_pair(uint8_t{0x22}, cal::kFloorConfirmMs));
  for (const auto& p : unit.points) CHECK(p == std::make_pair(cal::kFloorMcs, uint8_t{40}));
  // The sweep ends at the floor the drone flies.
  CHECK(unit.restores == 1);
  CHECK(unit.floor == 0x35);
}

TEST(floor_sweep_with_no_passing_value_reports_none) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.edge = 0x40;  // above every candidate
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0);
  CHECK(r.tried == 12);
  CHECK(!r.noisy);
  CHECK(unit.floor == 0x35);
}

// A blast's first second runs about 7 % above the rest. The reference is its
// dwell's worst second, as a candidate's verdict is, so a healthy floor
// still passes; a reference that read its first second would fail every
// candidate (2682 is 93.5 % of 2868).
TEST(floor_sweep_passes_a_healthy_floor_past_a_blasts_first_second) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.full_fps = 2682;
  unit.first_second_extra_fps = 186;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.ref_fps == 2682);
  CHECK(r.floor == 0x24);
  CHECK(r.confirm_pct == 100);
}

// Other traffic spoils the walk, so a busy channel shows no edge: the walk
// ends with nothing found and says what the drone heard. The reference
// point's receiver is deaf and is not asked; the first candidate ends it.
TEST(floor_sweep_ends_at_the_first_candidate_on_a_busy_channel) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.heard_per_s = 190;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.ref_fps == 2484);
  CHECK(r.floor == 0);
  CHECK(r.tried == 1);
  CHECK(r.heard_fps == 190);
  CHECK(r.noisy);
  REQUIRE(unit.blasts.size() == 2);
  CHECK(unit.floor == 0x35);
}

// Traffic that starts during the walk stops it at that point: the mean
// over the candidate points flown (400 frames in 8 s) is past the bound.
TEST(floor_sweep_stops_when_the_channel_turns_busy_mid_walk) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.heard_per_s = 100;
  unit.heard_from = 2;  // the reference and 0x20 in silence, then traffic
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0);
  CHECK(r.tried == 2);
  CHECK(r.heard_fps == 50);
  CHECK(r.noisy);
  REQUIRE(unit.blasts.size() == 3);
}

// One frame a second does not free a hold (measured) and does not end the
// walk.
TEST(floor_sweep_takes_a_trickle_of_frames_for_quiet) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.heard_per_s = 1;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0x24);
  CHECK(r.heard_fps == 1);
  CHECK(!r.noisy);
}

// Frames the receiver starts on and does not deliver free a held
// transmitter: 0x20 then sends the full rate and would be reported as the
// unit's edge. The callback counts none of them; the driver's carrier-sense
// count does, and the sweep ends with no result.
TEST(floor_sweep_counts_frames_the_receiver_does_not_deliver) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.seen_per_s = 40;
  unit.seen_frees_the_hold = true;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0);
  CHECK(r.tried == 1);
  CHECK(r.min_fps[0] == 2484);  // the held floor read as a healthy one
  CHECK(r.confirm_pct == 100);
  CHECK(r.heard_fps == 40);
  CHECK(r.noisy);
  // The reference, 0x20 and its confirmation.
  REQUIRE(unit.blasts.size() == 3);
  CHECK(unit.floor == 0x35);
}

// The bound on the stretch that decides the value: 5 events a second over
// the edge's dwell and confirmation still report it, 6 do not.
TEST(floor_sweep_reports_a_value_only_when_its_own_stretch_was_quiet) {
  for (uint32_t per_s : {5u, 6u}) {
    CaptureSink sink;
    RadioTx tx(sink);
    FakePowerCtl pwr;
    test::FakeFloorUnit unit;
    unit.seen_per_s = per_s;
    unit.seen_floor = 0x24;
    CalSweep s(CalSweepCfg{});
    s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
    REQUIRE(s.take_ack().has_value());
    run_floor(s, tx, pwr, unit);
    auto done = s.take_done();
    REQUIRE(done.has_value());
    const cal::FloorReport r = cal::unpack_floor_done(*done);
    CHECK(r.tried == 3);
    CHECK(r.confirm_pct == 100);
    CHECK(r.noisy == (per_s == 6));
    CHECK(r.floor == (per_s == 6 ? 0 : 0x24));
    // No frame was delivered; a noisy report carries the events.
    CHECK(r.heard_fps == (per_s == 6 ? 6 : 0));
  }
}

// Under the unit's edge the carrier-sense count is no measure of the
// channel: 0x20 in silence reads a steady 19 a second for seconds on end.
// It ends nothing; the walk goes on and reports the edge, whose own dwell
// and confirmation were quiet.
TEST(floor_sweep_lives_with_events_under_the_edge) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.seen_per_s = 19;
  unit.seen_floor = 0x20;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0x24);
  CHECK(r.tried == 3);
  CHECK(!r.noisy);
  CHECK(r.heard_fps == 0);
}

// Every candidate fails because a healthy transmitter defers to frames the
// receiver does not deliver (0x24 under 80 frames/s of 1.8 ms: 86 %). That
// is not a unit with no usable floor: the last candidate's dwell was not
// quiet, so the sweep has no result.
TEST(floor_sweep_with_no_passing_value_under_traffic_has_no_result) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  test::FakeFloorUnit unit;
  unit.edge = 0x20;
  unit.seen_per_s = 80;
  unit.seen_from = 1;  // the reference point is deaf to them and defers to none
  unit.seen_defers_fps = 348;  // 86 % of 2484 left
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0);
  CHECK(r.tried == 12);
  CHECK(r.min_fps[11] == 2136);
  CHECK(r.heard_fps == 80);
  CHECK(r.noisy);
}

namespace {

// 0x22 and 0x24 both marginal, the edge at 0x26.
struct TwoMarginal : test::FakeFloorUnit {
  BlastResult blast(const uint8_t* f, size_t len, uint32_t dwell_ms) override {
    if (floor == 0x22 || floor == 0x24) marginal = floor;
    return FakeFloorUnit::blast(f, len, dwell_ms);
  }
};

struct StampCapture : test::FakeFloorUnit {
  std::vector<int> idx;
  BlastResult blast(const uint8_t* f, size_t len, uint32_t dwell_ms) override {
    cal::CalFrameInfo info;
    if (cal::parse_cal_payload(f, len, &info)) {
      CHECK(info.kind == cal::Kind::Floor);
      CHECK(info.bw == 40);
      idx.push_back(info.idx);
    }
    return FakeFloorUnit::blast(f, len, dwell_ms);
  }
};

}  // namespace

TEST(floor_sweep_stops_after_two_failed_confirmations) {
  // The walk stops at the second marginal value rather than run on to the
  // edge, which bounds the GS's silent window.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  TwoMarginal unit;
  unit.edge = 0x26;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  const cal::FloorReport r = cal::unpack_floor_done(*done);
  CHECK(r.floor == 0);
  CHECK(r.tried == 3);
  CHECK(r.confirm_pct == 85);  // 2130 / 2484
}

TEST(floor_sweep_stamps_the_candidate_index) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  StampCapture unit;
  unit.edge = 0x22;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(7, 0), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_floor(s, tx, pwr, unit);
  CHECK(unit.idx == std::vector<int>({-1, 0, 1, 1}));
}

TEST(floor_sweep_is_refused_without_a_runtime_floor_or_at_20_mhz) {
  FakePowerCtl pwr;
  CalSweepCfg no_floor;
  no_floor.floor_ok = false;
  CalSweep s(no_floor);
  const auto c = cal::make_floor_sweep(7, 0);
  CHECK(!s.starts_sweep(c));
  s.on_cmd(c, 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Unsupported);
  CHECK(!s.active());
  CHECK(pwr.read_anchor_calls == 0);
  CHECK(pwr.zero_diff_calls == 0);

  CalSweep t(CalSweepCfg{});
  auto c20 = cal::make_floor_sweep(8, 0);
  c20.bw = 20;
  t.on_cmd(c20, 0, pwr);
  ack = t.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Unsupported);
  CHECK(!t.active());
}

TEST(an_unsupported_floor_leaves_an_open_run_running) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  CalSweepCfg no_floor;
  no_floor.floor_ok = false;
  CalSweep s(no_floor);
  s.on_cmd(power_cmd(9, 0, 0x01, 0, 0, 20, 3), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  REQUIRE(s.take_done().has_value());
  s.on_cmd(cal::make_floor_sweep(9, 1), 2000, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Unsupported);
  CHECK(s.active());
  CHECK(s.run_nonce() == 9);
}


// ---- the width a run flies --------------------------------------------------

TEST(a_40_mhz_run_on_a_20_mhz_drone_tunes_to_40_and_back_at_its_close) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.width = 20;
  pwr.anchor = 53;
  pwr.anchor40 = 47;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(1, 0, 0x01, 0, 0, /*bw*/ 40, 1), 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  CHECK(pwr.width == 40);
  // The anchor is the 40 MHz one: the only cell (rel 0) parks at it.
  run_to_quiescence(s, tx, pwr, cap);
  CHECK(pwr.index_writes.back() == 47);
  s.on_abort(cal::Abort{1}, 1000, pwr);
  CHECK(!s.active());
  CHECK(pwr.width == 20);
  CHECK(pwr.tunes == std::vector<uint8_t>({40, 20}));
}

TEST(a_20_mhz_sweep_never_retunes) {
  FakePowerCtl pwr;
  pwr.width = 40;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(2, 0, 0x01, 0, 0, /*bw*/ 20, 1), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  s.on_abort(cal::Abort{2}, 1000, pwr);
  CHECK(pwr.tunes.empty());
  CHECK(pwr.width == 40);
}

TEST(a_40_mhz_sweep_after_a_20_mhz_one_retunes_and_rereads_the_anchor) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.width = 20;
  pwr.anchor = 53;
  pwr.anchor40 = 47;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(3, 0, 0x01, 0, 0, 20, 1), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  CHECK(pwr.index_writes.back() == 53);
  REQUIRE(s.take_done().has_value());
  const int reads = pwr.read_anchor_calls;
  s.on_cmd(capacity_cmd(3, 1, 0x01, /*bw*/ 40), 2000, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  CHECK(pwr.width == 40);
  CHECK(pwr.read_anchor_calls == reads + 1);
  // Capacity puts the override at the anchor: the 40 MHz one.
  CHECK(pwr.index_writes.back() == 47);
  // A repeat of the running sweep touches nothing.
  s.on_cmd(capacity_cmd(3, 1, 0x01, 40), 2010, pwr);
  CHECK(pwr.tunes == std::vector<uint8_t>({40}));
  CHECK(pwr.read_anchor_calls == reads + 1);
}

TEST(a_channel_without_a_40_mhz_pair_acks_no_pair_and_touches_nothing) {
  FakePowerCtl pwr;
  pwr.width = 20;
  pwr.has_pair = false;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(cal::make_floor_sweep(4, 0), 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::NoPair);
  CHECK(!s.active());
  CHECK(pwr.index_writes.empty());
  CHECK(pwr.zero_diff_calls == 0);
  CHECK(pwr.read_anchor_calls == 0);
  CHECK(pwr.width == 20);
}

TEST(no_pair_mid_run_leaves_the_run_open_at_its_width) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.width = 20;
  pwr.has_pair = false;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(5, 0, 0x01, 0, 0, 20, 1), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  REQUIRE(s.take_done().has_value());
  s.on_cmd(power_cmd(5, 1, 0x01, 0, 0, 40, 1), 2000, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::NoPair);
  CHECK(s.active());
  CHECK(s.run_nonce() == 5);
  CHECK(pwr.width == 20);
  // The run carries on: a later 20 MHz sweep of it is accepted.
  s.on_cmd(capacity_cmd(5, 2, 0x01, 20), 2100, pwr);
  ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
}

TEST(an_unreadable_40_mhz_anchor_refuses_and_tunes_back) {
  FakePowerCtl pwr;
  pwr.width = 20;
  pwr.anchor40 = -1;
  pwr.anchor = -1;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(6, 0, 0x01, 0, 0, 40, 1), 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(!s.active());
  CHECK(pwr.width == 20);
  CHECK(pwr.tunes == std::vector<uint8_t>({40, 20}));
}

TEST(a_new_run_over_an_open_40_mhz_one_still_closes_on_the_flown_width) {
  FakePowerCtl pwr;
  pwr.width = 20;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(7, 0, 0x01, 0, 0, 40, 1), 0, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(pwr.width == 40);
  // A GS restart opens a new nonce while the old run is still open.
  s.on_cmd(power_cmd(8, 0, 0x01, 0, 0, 40, 1), 500, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(s.run_nonce() == 8);
  s.on_abort(cal::Abort{8}, 1000, pwr);
  CHECK(pwr.width == 20);
}

TEST(an_owner_that_does_not_retune_is_never_asked_to) {
  FakePowerCtl pwr;  // width 0
  CalSweep s(CalSweepCfg{});
  s.on_cmd(power_cmd(9, 0, 0x01, 0, 0, 40, 1), 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  s.on_abort(cal::Abort{9}, 1000, pwr);
  CHECK(pwr.tunes.empty());
}

MTEST_MAIN
