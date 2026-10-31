#include "mtest.h"
#include "debug_http.h"
#include "sim_overrides.h"
TEST(debug_http_routes) {
  CHECK(debug_http_parse("GET /snapshot.jpg HTTP/1.0").kind == DebugReq::SNAPSHOT);
  CHECK(debug_http_parse("GET /venc HTTP/1.0").kind == DebugReq::STATS);
  auto r = debug_http_parse("POST /venc/set?bitrate=4000 HTTP/1.0");
  CHECK(r.kind == DebugReq::SET); CHECK(r.key == "bitrate"); CHECK(r.val == 4000);
  CHECK(debug_http_parse("POST /venc/set?gop=1 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("GET /api/v1/set?video0.bitrate=1 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("POST /venc/set?bitrate=nope HTTP/1.0").kind == DebugReq::BAD);
}
// Bench simulation routes (sim_overrides.h).
TEST(sim_routes) {
  CHECK(debug_http_parse("GET /sim HTTP/1.0").kind == DebugReq::SIM_GET);
  auto r = debug_http_parse("POST /sim?arm=1 HTTP/1.0");
  CHECK(r.kind == DebugReq::SIM_SET); CHECK(r.key == "arm"); CHECK(r.val == 1);
  CHECK(!r.clear); CHECK(r.ttl == mabur::SimOverrides::kDefaultTtlS);
  r = debug_http_parse("POST /sim?arm=0&ttl=300 HTTP/1.0");
  CHECK(r.kind == DebugReq::SIM_SET); CHECK(r.val == 0); CHECK(r.ttl == 300);
  r = debug_http_parse("POST /sim?arm=off HTTP/1.0");
  CHECK(r.kind == DebugReq::SIM_SET); CHECK(r.clear);
  CHECK(debug_http_parse("POST /sim?arm=2 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("POST /sim?arm=1&ttl=0 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("POST /sim?arm=1&ttl=9999 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("POST /sim?arm=1&foo=2 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("POST /sim?motors=1 HTTP/1.0").kind == DebugReq::BAD);
  CHECK(debug_http_parse("GET /sim?arm=1 HTTP/1.0").kind == DebugReq::BAD);
}

TEST(sim_overrides_expire_and_clear) {
  mabur::SimOverrides s;
  CHECK(!s.get("arm", 0));
  CHECK(s.set("arm", 1, 60, 1000));
  CHECK(s.get("arm", 1000) == 1L);
  CHECK(s.json(1000) == "{\"arm\":{\"value\":1,\"left_s\":60}}");
  CHECK(s.get("arm", 60999) == 1L);
  CHECK(!s.get("arm", 61000));              // expired at 1000 + 60 s
  CHECK(s.json(61000) == "{}");
  CHECK(s.set("arm", 0, 10, 0));
  CHECK(s.clear("arm"));
  CHECK(!s.get("arm", 1));
  CHECK(!s.set("arm", 5, 10, 0));           // bad value
  CHECK(!s.set("nope", 1, 10, 0));          // unknown key
}

TEST(superframe_p_pct_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?superframe_p_pct=200 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "superframe_p_pct");
  CHECK(p.val == 200);
}
TEST(min_qp_key_rejected) {
  // Deleted with the venc.min_qp config key (bench refuted the
  // QP-floor hypothesis); a stale bench script must get BAD, not a silent ok.
  CHECK(debug_http_parse("POST /venc/set?min_qp=24 HTTP/1.0").kind == DebugReq::BAD);
}
TEST(min_iqp_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?min_iqp=40 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "min_iqp");
  CHECK(p.val == 40);
}

TEST(max_iqp_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?max_iqp=51 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "max_iqp");
  CHECK(p.val == 51);
}

TEST(max_ipprop_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?max_ipprop=2 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "max_ipprop");
  CHECK(p.val == 2);
}

TEST(fps_key_whitelisted) {
  auto p = debug_http_parse("POST /venc/set?fps=15 HTTP/1.0");
  CHECK(p.kind == DebugReq::SET);
  CHECK(p.key == "fps");
  CHECK(p.val == 15);
}
MTEST_MAIN
