#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "mabur/maint_wire.h"

// The drone end of the maintenance lane (mabur/maint_wire.h): one shell
// session at a time and the video-off lease. Single-threaded (the maint
// thread); the process behind a session is behind MaintChild so tests run a
// fake.

// A running command: its stdin, its stdout and stderr merged, its exit.
// Every call is non-blocking.
struct MaintChild {
  virtual ~MaintChild() = default;
  virtual size_t write_stdin(const uint8_t* p, size_t n) = 0;  // bytes taken
  virtual void close_stdin() = 0;
  virtual size_t read_out(uint8_t* buf, size_t cap) = 0;  // 0 = nothing now
  virtual bool out_eof() const = 0;
  // The exit status once the process is reaped: the exit code, or 128 + the
  // signal that killed it.
  virtual std::optional<int> exited() = 0;
  virtual void kill() = 0;  // the whole process group
};
// nullptr = the command could not start.
using MaintSpawn = std::function<std::unique_ptr<MaintChild>(const std::string& cmd)>;

// /bin/sh -c cmd (an interactive /bin/sh -i for an empty cmd) in a process
// group of its own, default signal dispositions, SCHED_OTHER. A command that
// must outlive maburd (the update's swap script) detaches itself with setsid.
std::unique_ptr<MaintChild> spawn_shell(const std::string& cmd);

struct MaintServerCfg {
  int idle_ms = 30000;           // no frame of the session this long: killed
  uint16_t max_lease_ms = 10000;  // a video-off lease is clamped to this
};

struct MaintServerStats {
  uint64_t frames_in = 0, frames_out = 0, sessions = 0, refused = 0, bad = 0;
};

class MaintServer {
 public:
  MaintServer(MaintServerCfg cfg, MaintSpawn spawn) : cfg_(cfg), spawn_(std::move(spawn)) {}
  ~MaintServer();

  // A parsed frame off the air (one with kFlagFromDrone is ignored: another
  // drone's, or our own echo). `armed`: the FC reports ARMED right now; no
  // session opens and video stays on while it does.
  void on_frame(const mabur::maint::Frame& f, uint64_t now_ms, bool armed);
  // Moves bytes between the child and the stream; appends the frames to send,
  // each with kFlagFromDrone set.
  void poll(uint64_t now_ms, bool armed, std::vector<mabur::maint::Frame>* out);

  bool video_off(uint64_t now_ms) const { return now_ms < video_until_ms_; }
  bool session_open() const { return s_ != nullptr; }
  const MaintServerStats& stats() const { return stats_; }

 private:
  struct Session {
    explicit Session(uint32_t id) : stream(id) {}
    mabur::maint::Stream stream;
    std::unique_ptr<MaintChild> child;
    std::vector<uint8_t> stdin_pending;
    bool stdin_closed = false;
    uint64_t last_rx_ms = 0;
  };
  void close_session(int16_t status, bool tell);
  void reply_close(uint32_t session, int16_t status);

  MaintServerCfg cfg_;
  MaintSpawn spawn_;
  std::unique_ptr<Session> s_;
  std::vector<mabur::maint::Frame> replies_;
  uint64_t video_until_ms_ = 0;
  MaintServerStats stats_;
};
