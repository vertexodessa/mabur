#pragma once
// SimOverrides: bench-only simulated inputs for maburd (DEBUG, not for
// upstream). The debug HTTP endpoint (127.0.0.1 only, debug_http.h) sets
// them with POST /sim?<key>=<value>[&ttl=<s>] and reads them with GET /sim;
// the threads that own the real input consult sim().get(key) and use the
// simulated value while one is set. Nothing reaches the hardware: a
// simulated arm changes what maburd believes about the FC, never the FC.
//
// Every override expires (default kDefaultTtlS), so a forgotten one cannot
// leave a drone in a simulated state, and it lives in memory only: a maburd
// restart clears everything.
//
// Keys (one per scenario; add new ones to kKeys and to the reader):
//   arm   1 = FC armed, 0 = disarmed   (MSP thread: low_power's arm report
//                                       and the FC telemetry HEARTBEAT)
//
// Thread-safe.
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace mabur {

class SimOverrides {
 public:
  static constexpr long kDefaultTtlS = 60;
  static constexpr long kMaxTtlS = 3600;

  static bool known_key(const std::string& k) { return k == "arm"; }
  static bool valid(const std::string& k, long v) {
    if (k == "arm") return v == 0 || v == 1;
    return false;
  }

  // false for an unknown key, a bad value or a ttl outside 1..kMaxTtlS.
  bool set(const std::string& key, long value, long ttl_s, uint64_t now_ms) {
    if (!known_key(key) || !valid(key, value) || ttl_s < 1 || ttl_s > kMaxTtlS) return false;
    std::lock_guard<std::mutex> lk(mu_);
    map_[key] = Entry{value, now_ms + static_cast<uint64_t>(ttl_s) * 1000u};
    return true;
  }
  bool clear(const std::string& key) {
    if (!known_key(key)) return false;
    std::lock_guard<std::mutex> lk(mu_);
    map_.erase(key);
    return true;
  }
  std::optional<long> get(const std::string& key, uint64_t now_ms) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = map_.find(key);
    if (it == map_.end()) return std::nullopt;
    if (now_ms >= it->second.until_ms) {
      map_.erase(it);
      return std::nullopt;
    }
    return it->second.value;
  }
  // {"arm":{"value":1,"left_s":42}} -- the live overrides only.
  std::string json(uint64_t now_ms) {
    std::lock_guard<std::mutex> lk(mu_);
    std::string out = "{";
    bool first = true;
    for (auto it = map_.begin(); it != map_.end();) {
      if (now_ms >= it->second.until_ms) {
        it = map_.erase(it);
        continue;
      }
      if (!first) out += ",";
      first = false;
      out += "\"" + it->first + "\":{\"value\":" + std::to_string(it->second.value) +
             ",\"left_s\":" + std::to_string((it->second.until_ms - now_ms + 999) / 1000) + "}";
      ++it;
    }
    return out + "}";
  }

 private:
  struct Entry {
    long value;
    uint64_t until_ms;
  };
  std::mutex mu_;
  std::map<std::string, Entry> map_;
};

// The process's one instance.
inline SimOverrides& sim() {
  static SimOverrides s;
  return s;
}

inline uint64_t sim_now_ms() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

}  // namespace mabur
