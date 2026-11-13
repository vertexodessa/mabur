#include "maint_server.h"

#include <cerrno>
#include <csignal>
#include <ctime>
#include <fcntl.h>
#include <sched.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

using namespace mabur::maint;

namespace {

class PosixChild final : public MaintChild {
 public:
  PosixChild(pid_t pid, int in_fd, int out_fd) : pid_(pid), in_(in_fd), out_(out_fd) {}
  ~PosixChild() override {
    kill();
    if (in_ >= 0) ::close(in_);
    if (out_ >= 0) ::close(out_);
    if (!status_) {
      int st = 0;
      ::waitpid(pid_, &st, 0);  // SIGKILLed above: returns at once
    }
  }
  size_t write_stdin(const uint8_t* p, size_t n) override {
    if (in_ < 0 || n == 0) return 0;
    const ssize_t w = ::write(in_, p, n);
    if (w > 0) return static_cast<size_t>(w);
    if (w < 0 && errno == EPIPE) {
      // The command stopped reading. The maint thread blocks SIGPIPE, so the
      // signal is left pending on it: take it, and drop what is left.
      sigset_t s;
      sigemptyset(&s);
      sigaddset(&s, SIGPIPE);
      const timespec zero{0, 0};
      sigtimedwait(&s, nullptr, &zero);
      return n;
    }
    return 0;
  }
  void close_stdin() override {
    if (in_ >= 0) ::close(in_);
    in_ = -1;
  }
  size_t read_out(uint8_t* buf, size_t cap) override {
    if (out_ < 0 || cap == 0) return 0;
    const ssize_t r = ::read(out_, buf, cap);
    if (r > 0) return static_cast<size_t>(r);
    if (r == 0) {
      ::close(out_);
      out_ = -1;
    }
    return 0;
  }
  bool out_eof() const override { return out_ < 0; }
  std::optional<int> exited() override {
    if (!status_) {
      int st = 0;
      if (::waitpid(pid_, &st, WNOHANG) == pid_)
        status_ = WIFEXITED(st) ? WEXITSTATUS(st) : 128 + (WIFSIGNALED(st) ? WTERMSIG(st) : 0);
    }
    return status_;
  }
  void kill() override {
    if (!status_) ::kill(-pid_, SIGKILL);
  }

 private:
  pid_t pid_;
  int in_, out_;
  std::optional<int> status_;
};

}  // namespace

std::unique_ptr<MaintChild> spawn_shell(const std::string& cmd) {
  int in[2], out[2];
  if (::pipe2(in, O_CLOEXEC) != 0) return nullptr;
  if (::pipe2(out, O_CLOEXEC) != 0) {
    ::close(in[0]);
    ::close(in[1]);
    return nullptr;
  }
  posix_spawn_file_actions_t fa;
  posix_spawn_file_actions_init(&fa);
  posix_spawn_file_actions_adddup2(&fa, in[0], 0);
  posix_spawn_file_actions_adddup2(&fa, out[1], 1);
  posix_spawn_file_actions_adddup2(&fa, out[1], 2);
  posix_spawnattr_t at;
  posix_spawnattr_init(&at);
  sigset_t none, all;
  sigemptyset(&none);
  sigfillset(&all);
  posix_spawnattr_setsigmask(&at, &none);
  posix_spawnattr_setsigdefault(&at, &all);
  posix_spawnattr_setpgroup(&at, 0);
  sched_param sp{};
  sp.sched_priority = 0;
  posix_spawnattr_setschedpolicy(&at, SCHED_OTHER);
  posix_spawnattr_setschedparam(&at, &sp);
  posix_spawnattr_setflags(&at, POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF |
                                    POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSCHEDULER);
  std::string c = cmd;
  char sh[] = "/bin/sh", dash_c[] = "-c", dash_i[] = "-i";
  char* argv_c[] = {sh, dash_c, c.data(), nullptr};
  char* argv_i[] = {sh, dash_i, nullptr};
  pid_t pid = 0;
  const int rc = posix_spawn(&pid, "/bin/sh", &fa, &at, cmd.empty() ? argv_i : argv_c, environ);
  posix_spawn_file_actions_destroy(&fa);
  posix_spawnattr_destroy(&at);
  ::close(in[0]);
  ::close(out[1]);
  if (rc != 0) {
    ::close(in[1]);
    ::close(out[0]);
    return nullptr;
  }
  ::fcntl(in[1], F_SETFL, ::fcntl(in[1], F_GETFL) | O_NONBLOCK);
  ::fcntl(out[0], F_SETFL, ::fcntl(out[0], F_GETFL) | O_NONBLOCK);
  return std::make_unique<PosixChild>(pid, in[1], out[0]);
}

MaintServer::~MaintServer() = default;

void MaintServer::reply_close(uint32_t session, int16_t status) {
  Frame f;
  f.kind = kClose;
  f.session = session;
  f.status = status;
  replies_.push_back(std::move(f));
}

void MaintServer::close_session(int16_t status, bool tell) {
  if (!s_) return;
  if (s_->child) s_->child->kill();
  if (tell) reply_close(s_->stream.session(), status);
  s_.reset();
}

void MaintServer::on_frame(const Frame& f, uint64_t now_ms, bool armed) {
  if (f.flags & kFlagFromDrone) return;
  ++stats_.frames_in;
  switch (f.kind) {
    case kVideo: {
      uint16_t lease = f.data.size() >= 2 ? static_cast<uint16_t>(f.data[0] | (f.data[1] << 8)) : 0;
      if (armed) lease = 0;
      if (lease > cfg_.max_lease_ms) lease = cfg_.max_lease_ms;
      video_until_ms_ = lease ? now_ms + lease : 0;
      Frame r;
      r.kind = kVideo;
      r.session = f.session;
      r.data = {static_cast<uint8_t>(lease), static_cast<uint8_t>(lease >> 8)};
      replies_.push_back(std::move(r));
      return;
    }
    case kOpen: {
      if (armed) {
        ++stats_.refused;
        reply_close(f.session, kStatusArmed);
        return;
      }
      if (s_ && s_->stream.session() == f.session) {
        s_->last_rx_ms = now_ms;
        s_->stream.ack_now();  // the GS missed the answer
        return;
      }
      close_session(kStatusClosed, false);
      auto child = spawn_(std::string(f.data.begin(), f.data.end()));
      if (!child) {
        ++stats_.refused;
        reply_close(f.session, kStatusSpawnFailed);
        return;
      }
      s_ = std::make_unique<Session>(f.session);
      s_->child = std::move(child);
      s_->last_rx_ms = now_ms;
      s_->stream.ack_now();
      ++stats_.sessions;
      return;
    }
    case kData:
      if (!s_ || s_->stream.session() != f.session) {
        reply_close(f.session, kStatusUnknown);
        return;
      }
      s_->last_rx_ms = now_ms;
      s_->stream.on_frame(f, now_ms);
      return;
    case kClose:
      if (s_ && s_->stream.session() == f.session) close_session(kStatusClosed, true);
      else reply_close(f.session, kStatusUnknown);
      return;
    default:
      ++stats_.bad;
      reply_close(f.session, kStatusUnsupported);
      return;
  }
}

void MaintServer::poll(uint64_t now_ms, bool armed, std::vector<Frame>* out) {
  const size_t before = out->size();
  if (armed) {
    video_until_ms_ = 0;
    close_session(kStatusArmed, true);
  }
  if (s_ && now_ms - s_->last_rx_ms >= static_cast<uint64_t>(cfg_.idle_ms))
    close_session(kStatusIdle, true);
  if (s_) {
    Session& s = *s_;
    s.stream.read(&s.stdin_pending);
    while (!s.stdin_pending.empty()) {
      const size_t w = s.child->write_stdin(s.stdin_pending.data(), s.stdin_pending.size());
      if (w == 0) break;
      s.stdin_pending.erase(s.stdin_pending.begin(), s.stdin_pending.begin() + static_cast<std::ptrdiff_t>(w));
    }
    if (s.stream.peer_fin() && s.stdin_pending.empty() && !s.stdin_closed) {
      s.child->close_stdin();
      s.stdin_closed = true;
    }
    uint8_t buf[4096];
    while (!s.stream.finishing()) {
      const size_t room = std::min(s.stream.send_room(), sizeof buf);
      if (room == 0) break;
      const size_t n = s.child->read_out(buf, room);
      if (n == 0) break;
      s.stream.write(buf, n);
    }
    if (!s.stream.finishing() && s.child->out_eof()) {
      if (auto st = s.child->exited()) s.stream.finish(static_cast<int16_t>(*st));
    }
    s.stream.poll(now_ms, out);
  }
  for (auto& r : replies_) out->push_back(std::move(r));
  replies_.clear();
  for (size_t i = before; i < out->size(); ++i) (*out)[i].flags |= kFlagFromDrone;
  stats_.frames_out += out->size() - before;
}
