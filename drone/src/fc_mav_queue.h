#pragma once
// FcMavQueue: the flight controller's MAVLink packets (msp_telemetry.h, made
// on the MSP thread) waiting for the drone's T_MAV uplink (the MAV thread,
// docs/mavlink.md). Whole packets only; the MAV thread appends them to a
// frame's payload after the parameter component's own replies, up to
// rc::kMavMaxPayload, so the component's acks never wait behind telemetry.
//
// Bounded, drop-OLDEST: telemetry is a stream of samples, and while the link
// is down (no published session, a calibration sweep) the newest attitude
// and battery are the ones worth sending when it comes back.
//
// Thread-safe: push from the MSP thread, take from the MAV thread.
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace mabur {

class FcMavQueue {
 public:
  explicit FcMavQueue(size_t cap_packets = 32) : cap_(cap_packets) {}

  void push(std::vector<uint8_t>&& packet) {
    if (packet.empty()) return;
    std::lock_guard<std::mutex> lk(mu_);
    if (q_.size() >= cap_) {
      q_.pop_front();
      ++dropped_;
    }
    q_.push_back(std::move(packet));
  }

  // Appends whole entries to *out while out->size() stays within max_len; an
  // entry that does not fit stays queued for the next frame. An entry is one
  // or more whole MAVLink packets (MspTelemetry writes VFR_HUD with
  // GLOBAL_POSITION_INT, SYS_STATUS with BATTERY_STATUS). Returns the
  // MAVLink packets taken.
  int take(std::vector<uint8_t>* out, size_t max_len) {
    std::lock_guard<std::mutex> lk(mu_);
    int n = 0;
    while (!q_.empty() && out->size() + q_.front().size() <= max_len) {
      n += count_packets(q_.front());
      out->insert(out->end(), q_.front().begin(), q_.front().end());
      q_.pop_front();
    }
    return n;
  }

  // Whole MAVLink 2 (0xFD, len + 12 bytes unsigned) and v1 (0xFE, len + 8)
  // packets back to back; anything else counts as one.
  static int count_packets(const std::vector<uint8_t>& b) {
    size_t off = 0;
    int n = 0;
    while (off + 2 <= b.size()) {
      const size_t total = b[off] == 0xFD ? b[off + 1] + 12u : b[off] == 0xFE ? b[off + 1] + 8u : 0;
      if (total == 0 || off + total > b.size()) break;
      off += total;
      ++n;
    }
    return n > 0 ? n : 1;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(mu_);
    return q_.size();
  }
  uint64_t dropped() const {
    std::lock_guard<std::mutex> lk(mu_);
    return dropped_;
  }

 private:
  mutable std::mutex mu_;
  std::deque<std::vector<uint8_t>> q_;
  size_t cap_;
  uint64_t dropped_ = 0;
};

}  // namespace mabur
