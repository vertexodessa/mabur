#pragma once

#include <cstdint>

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

// The ladder's block-4 instant-demote input: BASE, current rung, the
// decoder's abandonment alone (a retx-filled symbol counts as arrived, unlike
// in residual_counts). Named separately so the demote path's scope is
// greppable -- picking the wrong measure here is a bug this has had before.
ResidualCounts ladder_residual_counts(const mabur::UepDecoder& dec);

}  // namespace maburgs
