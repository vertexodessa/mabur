// FcMavQueue: whole packets only, a packet that does not fit waits for the
// next frame, drop-oldest at the cap.
#include "mtest.h"

#include <vector>

#include "fc_mav_queue.h"

using mabur::FcMavQueue;

TEST(take_appends_whole_packets_up_to_the_limit) {
  FcMavQueue q(8);
  // Not MAVLink framing: each entry counts as one packet.
  q.push(std::vector<uint8_t>(100, 1));
  q.push(std::vector<uint8_t>(100, 2));
  q.push(std::vector<uint8_t>(100, 3));
  std::vector<uint8_t> payload(350, 0);   // the component's replies, already in
  CHECK(q.take(&payload, 600) == 2);
  CHECK(payload.size() == 550);
  CHECK(payload[350] == 1 && payload[450] == 2);
  CHECK(q.size() == 1);                   // the third waits for the next frame
  std::vector<uint8_t> next;
  CHECK(q.take(&next, 600) == 1);
  CHECK(next.size() == 100 && next[0] == 3);
  CHECK(q.take(&next, 600) == 0);
}

TEST(full_queue_drops_the_oldest) {
  FcMavQueue q(2);
  q.push({1});
  q.push({2});
  q.push({3});
  CHECK(q.size() == 2);
  CHECK(q.dropped() == 1);
  std::vector<uint8_t> out;
  CHECK(q.take(&out, 600) == 2);
  CHECK(out == std::vector<uint8_t>({2, 3}));
  q.push({});                             // empty: ignored
  CHECK(q.size() == 0);
}

TEST(take_counts_mavlink_packets_not_entries) {
  FcMavQueue q(8);
  // One entry holding two MAVLink 2 packets (payload 1 and 3: 13 + 15 bytes).
  std::vector<uint8_t> two(13, 0);
  two[0] = 0xFD; two[1] = 1;
  std::vector<uint8_t> b(15, 0);
  b[0] = 0xFD; b[1] = 3;
  two.insert(two.end(), b.begin(), b.end());
  q.push(std::move(two));
  std::vector<uint8_t> out;
  CHECK(q.take(&out, 600) == 2);
  CHECK(out.size() == 28);
}

MTEST_MAIN
