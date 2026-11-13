#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>
namespace mabur::maint {

// Maintenance lane: a shell on the drone, driven from the GS over the air.
//
// Its frames are NOT RC frames. RC frames carry RC_VERSION and both ends
// refuse every other version, which is right for the link but wrong here:
// the lane exists to update a drone whose maburd is older than the GS. So the
// lane has its own magic and its own envelope version, and a GS keeps
// speaking every envelope version a drone in the field may run. The envelope
// only moves bytes; what they mean is up to the shell at the far end, so new
// commands never need a new envelope. A kind this end does not know is
// answered with kClose / kStatusUnsupported, never dropped silently.
//
// Layout, little-endian, CRC16-CCITT over every byte before it (bytes past the
// CRC are ignored: on hardware the body still carries the 802.11 FCS):
//
//   u16 magic "MT" | u8 version | u8 kind | u32 session | u32 seq | u32 ack |
//   u8 flags | i16 status | u16 len | u8[len] data | u16 crc
//
// No tag: the lane assumes a trusted field (anyone on the channel can open a
// shell). Two drones answering the same GS are told apart by nothing but the
// session id the GS picks.
constexpr uint16_t kMagic = 0x544D;  // bytes 'M' 'T'
constexpr uint8_t kVersion = 1;
constexpr size_t kHeadLen = 2 + 1 + 1 + 4 + 4 + 4 + 1 + 2 + 2;  // 21
constexpr size_t kMaxData = 1200;

enum Kind : uint8_t {
  // GS -> drone: start a session running `data` with /bin/sh -c, or an
  // interactive shell when data is empty. A repeat of the current session's
  // OPEN is answered, not re-run; a new session id replaces the current one.
  kOpen = 1,
  // Both ways: stream bytes (GS -> drone stdin, drone -> GS stdout + stderr)
  // at stream offset `seq`, and `ack` = the next offset this end expects.
  // kFlagFin: the sender's stream ends after this frame's data; the drone's
  // FIN carries the command's exit status.
  kData = 2,
  // GS -> drone: end the session, kill the command. Drone -> GS: the session
  // is not (or no longer) there; `status` says why.
  kClose = 3,
  // GS -> drone: video off for data[0..1] (u16) ms from now, 0 = on now. The
  // GS renews the lease while it needs the air; video comes back by itself
  // when it stops. Drone -> GS: the lease left, same field.
  kVideo = 4,
};

constexpr uint8_t kFlagFin = 0x01;
// Set on every frame the drone sends, on none the GS sends: a GS with two
// cards hears its own frames on the second one and must not take them for
// the drone's.
constexpr uint8_t kFlagFromDrone = 0x80;

// kClose status from the drone.
constexpr int16_t kStatusClosed = 0;        // ended on request
constexpr int16_t kStatusArmed = -1;        // the FC is armed: no session
constexpr int16_t kStatusSpawnFailed = -2;  // the command could not start
constexpr int16_t kStatusUnknown = -3;      // no such session (a restarted drone)
constexpr int16_t kStatusUnsupported = -4;  // a kind this drone does not know
constexpr int16_t kStatusIdle = -5;         // nothing heard from the GS for too long

struct Frame {
  uint8_t kind = kData;
  uint32_t session = 0;
  uint32_t seq = 0;
  uint32_t ack = 0;
  uint8_t flags = 0;
  int16_t status = 0;
  std::vector<uint8_t> data;
};

// Caller keeps data.size() <= kMaxData.
std::vector<uint8_t> pack(const Frame& f);
// Any kind is returned (the caller answers the unknown ones); nullopt for a
// short buffer, another magic or version, len > kMaxData, or a bad CRC.
std::optional<Frame> parse(const uint8_t* buf, size_t len);
// Magic and version only, no CRC: routes a body on the RX path.
bool is_maint(const uint8_t* buf, size_t len);

// One direction pair of a session: a byte stream out with go-back-N
// retransmission, a byte stream in delivered in order. The same class runs
// on both ends. Offsets are u32 from 0; FIN takes one offset after the last
// byte, as in TCP, so an ack past it proves the FIN arrived.
class Stream {
 public:
  struct Cfg {
    size_t max_frame = kMaxData;
    size_t window = 16 * kMaxData;   // unacked bytes in flight
    size_t send_cap = 64 * 1024;     // bytes buffered for sending, acked or not
    size_t recv_cap = 64 * 1024;     // in-order bytes waiting for read()
    int rto_ms = 300;                // no ack progress this long: resend from the oldest
    int ack_delay_ms = 5;            // a pure ack waits this long for data to ride on
    int max_frames_per_poll = 8;
  };
  Stream(uint32_t session, const Cfg& cfg) : session_(session), cfg_(cfg) {}
  explicit Stream(uint32_t session) : Stream(session, Cfg{}) {}

  uint32_t session() const { return session_; }

  // Out. write() takes as much as fits in send_cap and returns that count.
  size_t write(const uint8_t* p, size_t n);
  size_t send_room() const;
  // No more writes; the FIN goes after the buffered bytes with `status`.
  void finish(int16_t status = 0);
  bool finishing() const { return fin_; }
  // Every byte and the FIN acked.
  bool out_done() const { return fin_ && snd_una_ == fin_off_ + 1; }
  size_t unacked() const { return buf_.size(); }

  // In. A kData frame of this session (acks and data both).
  void on_frame(const Frame& f, uint64_t now_ms);
  // Moves the in-order bytes received so far into *out (appends).
  size_t read(std::vector<uint8_t>* out);
  size_t readable() const { return rx_.size(); }
  bool peer_fin() const { return peer_fin_; }   // FIN received; read() has the rest
  int16_t peer_status() const { return peer_status_; }
  uint32_t rcv_nxt() const { return rcv_nxt_; }

  // The frames to send now: new data within the window, a resend after
  // rto_ms without ack progress, a FIN, or a pure ack when one is due.
  void poll(uint64_t now_ms, std::vector<Frame>* out);
  // Forces a pure ack on the next poll (an OPEN reply, a duplicate).
  void ack_now() { ack_due_ = true; ack_at_ = 0; }

  uint64_t resends() const { return resends_; }

 private:
  Frame make_data(uint32_t seq, size_t len, bool fin) const;
  uint32_t end_off() const { return snd_una_ + static_cast<uint32_t>(buf_.size()); }

  uint32_t session_;
  Cfg cfg_;
  // Out: buf_ holds the bytes from snd_una_ on (acked ones are popped).
  std::deque<uint8_t> buf_;
  // snd_max_: the highest offset ever sent; a go-back-N rewinds snd_nxt_
  // below it, and an ack is only progress up to it.
  uint32_t snd_una_ = 0, snd_nxt_ = 0, snd_max_ = 0;
  bool fin_ = false;
  uint32_t fin_off_ = 0;
  int16_t fin_status_ = 0;
  uint64_t progress_ms_ = 0;
  int dup_acks_ = 0;
  uint64_t resends_ = 0;
  // In.
  std::vector<uint8_t> rx_;
  uint32_t rcv_nxt_ = 0;
  bool peer_fin_ = false;
  int16_t peer_status_ = 0;
  bool ack_due_ = false;
  uint64_t ack_at_ = 0;
};

}  // namespace mabur::maint
