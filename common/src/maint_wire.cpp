#include "mabur/maint_wire.h"

#include <algorithm>

#include "mabur/crc16.h"

namespace mabur::maint {
namespace {
void put16(std::vector<uint8_t>& b, uint16_t v) {
  b.push_back(static_cast<uint8_t>(v));
  b.push_back(static_cast<uint8_t>(v >> 8));
}
void put32(std::vector<uint8_t>& b, uint32_t v) {
  for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}
uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t get32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace

std::vector<uint8_t> pack(const Frame& f) {
  std::vector<uint8_t> b;
  b.reserve(kHeadLen + f.data.size() + 2);
  put16(b, kMagic);
  b.push_back(kVersion);
  b.push_back(f.kind);
  put32(b, f.session);
  put32(b, f.seq);
  put32(b, f.ack);
  b.push_back(f.flags);
  put16(b, static_cast<uint16_t>(f.status));
  put16(b, static_cast<uint16_t>(f.data.size()));
  b.insert(b.end(), f.data.begin(), f.data.end());
  put16(b, crc16_ccitt(b.data(), b.size()));
  return b;
}

bool is_maint(const uint8_t* buf, size_t len) {
  return len >= 3 && get16(buf) == kMagic && buf[2] == kVersion;
}

std::optional<Frame> parse(const uint8_t* buf, size_t len) {
  if (len < kHeadLen + 2 || !is_maint(buf, len)) return std::nullopt;
  const size_t n = get16(buf + 19);
  if (n > kMaxData || len < kHeadLen + n + 2) return std::nullopt;
  if (crc16_ccitt(buf, kHeadLen + n) != get16(buf + kHeadLen + n)) return std::nullopt;
  Frame f;
  f.kind = buf[3];
  f.session = get32(buf + 4);
  f.seq = get32(buf + 8);
  f.ack = get32(buf + 12);
  f.flags = buf[16];
  f.status = static_cast<int16_t>(get16(buf + 17));
  f.data.assign(buf + kHeadLen, buf + kHeadLen + n);
  return f;
}

size_t Stream::send_room() const {
  if (fin_ || buf_.size() >= cfg_.send_cap) return 0;
  return cfg_.send_cap - buf_.size();
}

size_t Stream::write(const uint8_t* p, size_t n) {
  const size_t take = std::min(n, send_room());
  buf_.insert(buf_.end(), p, p + take);
  return take;
}

void Stream::finish(int16_t status) {
  if (fin_) return;
  fin_ = true;
  fin_off_ = end_off();
  fin_status_ = status;
}

Frame Stream::make_data(uint32_t seq, size_t len, bool fin) const {
  Frame f;
  f.kind = kData;
  f.session = session_;
  f.seq = seq;
  f.ack = rcv_nxt_;
  f.flags = fin ? kFlagFin : 0;
  f.status = fin ? fin_status_ : 0;
  const size_t from = seq - snd_una_;
  f.data.assign(buf_.begin() + static_cast<std::ptrdiff_t>(from),
                buf_.begin() + static_cast<std::ptrdiff_t>(from + len));
  return f;
}

void Stream::on_frame(const Frame& f, uint64_t now_ms) {
  if (f.kind != kData || f.session != session_) return;
  // Ack: anything in (snd_una_, snd_max_] is progress. An ack at snd_una_ on a
  // pure ack while bytes are in flight means the peer got something it could
  // not use (out of order or a duplicate); the second one resends at once
  // instead of waiting out rto_ms.
  const bool pure_ack = f.data.empty() && !(f.flags & kFlagFin);
  if (f.ack > snd_una_ && f.ack <= snd_max_) {
    const uint32_t data_acked = std::min(f.ack, end_off()) - snd_una_;
    buf_.erase(buf_.begin(), buf_.begin() + data_acked);
    snd_una_ = f.ack;
    if (snd_nxt_ < snd_una_) snd_nxt_ = snd_una_;
    progress_ms_ = now_ms;
    dup_acks_ = 0;
  } else if (pure_ack && f.ack == snd_una_ && snd_max_ > snd_una_) {
    if (++dup_acks_ == 2) {
      snd_nxt_ = snd_una_;
      progress_ms_ = now_ms;
      ++resends_;
    }
  }
  if (pure_ack) return;
  // Data: only the next bytes in order are taken; the rest is dropped and
  // acked at once, so the sender learns where the gap is.
  const uint32_t end = f.seq + static_cast<uint32_t>(f.data.size());
  const bool fin = (f.flags & kFlagFin) != 0;
  const bool new_bytes = f.seq <= rcv_nxt_ && end > rcv_nxt_;
  const bool new_fin = fin && f.seq <= rcv_nxt_ && end == rcv_nxt_;
  if (!peer_fin_ && (new_bytes || new_fin)) {
    const size_t skip = rcv_nxt_ - f.seq;
    const size_t take = f.data.size() - skip;
    if (rx_.size() + take <= cfg_.recv_cap) {
      rx_.insert(rx_.end(), f.data.begin() + static_cast<std::ptrdiff_t>(skip), f.data.end());
      rcv_nxt_ = end;
      if (fin) {
        rcv_nxt_ = end + 1;
        peer_fin_ = true;
        peer_status_ = f.status;
      }
      if (!ack_due_) {
        ack_due_ = true;
        ack_at_ = now_ms + static_cast<uint64_t>(cfg_.ack_delay_ms);
      }
      return;
    }
    // No room until read(): dropped, the sender resends.
  }
  ack_now();
}

size_t Stream::read(std::vector<uint8_t>* out) {
  const size_t n = rx_.size();
  out->insert(out->end(), rx_.begin(), rx_.end());
  rx_.clear();
  return n;
}

void Stream::poll(uint64_t now_ms, std::vector<Frame>* out) {
  if (snd_max_ > snd_una_ && now_ms - progress_ms_ >= static_cast<uint64_t>(cfg_.rto_ms)) {
    snd_nxt_ = snd_una_;  // go-back-N from the oldest unacked byte
    progress_ms_ = now_ms;
    ++resends_;
  }
  const uint32_t limit = end_off() + (fin_ ? 1u : 0u);
  int sent = 0;
  while (sent < cfg_.max_frames_per_poll && snd_nxt_ < limit &&
         snd_nxt_ - snd_una_ < cfg_.window) {
    const uint32_t data_end = end_off();
    const size_t len = snd_nxt_ < data_end ? std::min<size_t>(cfg_.max_frame, data_end - snd_nxt_) : 0;
    const bool fin_here = fin_ && snd_nxt_ + len == fin_off_;
    if (len == 0 && !fin_here) break;
    if (snd_nxt_ == snd_una_ && snd_max_ == snd_una_) progress_ms_ = now_ms;
    out->push_back(make_data(snd_nxt_, len, fin_here));
    snd_nxt_ += static_cast<uint32_t>(len) + (fin_here ? 1u : 0u);
    snd_max_ = std::max(snd_max_, snd_nxt_);
    ++sent;
  }
  if (sent > 0) {
    ack_due_ = false;  // every data frame carries the ack
  } else if (ack_due_ && now_ms >= ack_at_) {
    Frame f;
    f.kind = kData;
    f.session = session_;
    f.seq = snd_nxt_;
    f.ack = rcv_nxt_;
    out->push_back(std::move(f));
    ack_due_ = false;
  }
}

}  // namespace mabur::maint
