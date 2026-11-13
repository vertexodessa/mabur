#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "mabur/uep_decoder.h"

namespace maburgs {

// {arrived, expected} for the ladder's block-4 instant-demote input --
// post-FEC (residual) loss on the current rung. Extracted from the control
// step in gs/src/main.cpp so the decoder -> LinkHealth wiring is testable:
// test_vrx_controller.cpp hand-builds LinkHealth, so it cannot cover which
// of the decoder's two loss measures the demote path actually reads. See
// tests/test_ladder_residual.cpp.
struct ResidualCounts {
  uint64_t arrived = 0;
  uint64_t expected = 0;
};

// One layer's post-FEC loss counters, from the FEC decoder's own abandonment
// count. cur = true reports the CURRENT RUNG only, excluding the
// pre-transition debris the attribution watermark booked as stale
// (SwDecoder::syms_abandoned_stale) -- attribution is preserved, it just
// lives on the symbol side now.
ResidualCounts residual_counts(const mabur::UepDecoder& dec, int sid, bool cur);

// Both layers summed. Observability only (sideport, ctl log, player OSD):
// "how much video did not survive FEC", base and enh together.
ResidualCounts residual_counts_pooled(const mabur::UepDecoder& dec, bool cur);

// Post-FEC delivery as a percent in [0,100]; 100 when nothing was expected.
int delivery_pct(ResidualCounts rc);

// What a base symbol a NACK retransmit filled counts as in the ladder's
// residual (link.nack_residual):
//   kFilled -- arrived (the default);
//   kAll    -- lost, as if no retransmit had come (what residual_counts books);
//   kTimely -- arrived when the fill beat the NACK tracker's deadline for its
//              seq, lost when it landed after (NackStats::filled_late).
enum class NackResidual : uint8_t { kFilled, kAll, kTimely };

// The config spelling: "filled", "all", "timely"; nullopt for anything else.
std::optional<NackResidual> parse_nack_residual(const std::string& s);
const char* nack_residual_name(NackResidual m);

// The ladder's block-4 instant-demote input: BASE, current rung, the
// decoder's abandonment plus the retx-filled symbols `mode` books as lost.
// late_fills is the NACK tracker's cumulative NackStats::filled_late, read
// only under kTimely. Named separately so the demote path's scope is
// greppable -- picking the wrong measure here is a bug this has had before.
ResidualCounts ladder_residual_counts(const mabur::UepDecoder& dec,
                                      NackResidual mode = NackResidual::kFilled,
                                      uint64_t late_fills = 0);

}  // namespace maburgs
