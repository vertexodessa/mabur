#include "ladder_residual.h"

#include <algorithm>

namespace maburgs {

// The BASE layer's post-FEC loss, from the FEC decoder's own abandonment
// count -- symbols that fell below the sliding-window horizon still unknown
// (SwDecoder::advance). Current-rung only: syms_abandoned_stale is the
// pre-transition debris the attribution watermark already accounted for.
//
// NOT the packet-level delivery window (window_counts_cur), which this
// replaced. That measure inferred loss from FRAG-seq gaps, and
// a gap is indistinguishable from a unit that merely has not completed yet:
// a sliding-window repair landing late completes an older unit after a newer
// one, and the next forward gap re-books the already-delivered unit as a
// fresh expectation. In a window holding 3-6 units that reads as 0.25 or
// 0.33 residual, and block 4 demotes on anything > 0 -- 31 spurious demotes
// in ~7 min on the bench with abn=0 the whole time. Symbol abandonment is
// pure seq arithmetic over a span and cannot be fooled by arrival order.
//
// The trade is detection lag: a lost symbol is booked only once the horizon
// passes it (seq_horizon 512 at ~6.4k sym/s is ~80 ms), where the packet
// measure fired on the next forward gap. The ENH layer's demote path has
// always accepted exactly this (see s3_resid_cur in gs/src/main.cpp).
//
// BASE only, deliberately: the packet measure pooled sid 0 and sid 1, a
// leftover from the 4-stream era when sid 0/1/2 were all base layers (see
// the "never-shed base layers" note on window_counts). Since the 2-stream
// flatten sid 1 is the SHED-ABLE enh layer, which has its own demote path
// in block 5a; pooling it here let enh loss demote through both.
//
// abandoned also folds in syms_retx: a symbol a NACK retransmit filled
// (or cascade-solved off a repair row) is video-whole but its direct copy
// never arrived, and the stats book that as loss. The ladder's own input
// books it as link.nack_residual says (ladder_residual_counts below; the
// default counts it arrived). syms_retx has no stale split: a
// retx filling a pre-transition seq counts as current residual loss even
// under `cur` (negligible -- the drone only answers seqs within ring_ms,
// and a transition gap is a handful of symbols).
ResidualCounts residual_counts(const mabur::UepDecoder& dec, int sid,
                               bool cur) {
  const auto s = dec.stats(sid);
  const uint64_t abandoned =
      (cur ? s.syms_abandoned - s.syms_abandoned_stale : s.syms_abandoned) +
      s.syms_retx;
  const uint64_t expected = s.syms_delivered + s.syms_recovered + abandoned;
  return ResidualCounts{expected - abandoned, expected};
}

ResidualCounts residual_counts_pooled(const mabur::UepDecoder& dec, bool cur) {
  const auto base = residual_counts(dec, 0, cur);
  const auto enh = residual_counts(dec, 1, cur);
  return ResidualCounts{base.arrived + enh.arrived,
                        base.expected + enh.expected};
}

int delivery_pct(ResidualCounts rc) {
  if (rc.expected == 0) return 100;  // idle layer is not a 0%-delivered layer
  const uint64_t pct = rc.arrived * 100 / rc.expected;
  return static_cast<int>(pct > 100 ? 100 : pct);
}

std::optional<NackResidual> parse_nack_residual(const std::string& s) {
  if (s == "filled") return NackResidual::kFilled;
  if (s == "all") return NackResidual::kAll;
  if (s == "timely") return NackResidual::kTimely;
  return std::nullopt;
}

const char* nack_residual_name(NackResidual m) {
  switch (m) {
    case NackResidual::kFilled: return "filled";
    case NackResidual::kAll: return "all";
    case NackResidual::kTimely: return "timely";
  }
  return "filled";
}

// The ladder demotes on what the decoder could not repair, and on the
// retransmit-filled symbols `mode` books as lost:
//   kFilled: none. The drone answers a request with a burst of resends that
//     race the frame's own FEC repairs, and the ones that land first fill
//     symbols the repairs would have solved a few milliseconds later. Booked
//     as loss, each of them cost a rung on a link that abandons nothing with
//     the NACK off.
//   kAll: every one, as residual_counts does. On a link with sustained loss
//     kFilled holds a rung whose holes only the retransmits fill, and the
//     retransmits spend air on that same failing rung.
//   kTimely: the ones that filled after the NACK tracker's deadline for
//     their seq (gap_timeout_ms after it was first seen missing, the frame
//     stream's own gap timeout): such a fill came too late to save its
//     frame. late_fills <= syms_retx by construction (a late fill is a seq
//     the decoder holds as kRetx, and every kRetx seq booked one syms_retx);
//     the min() only keeps a counter mismatch from wrapping.
// expected is the same under every mode, so a mode moves symbols between
// arrived and lost and never changes the window's denominator.
ResidualCounts ladder_residual_counts(const mabur::UepDecoder& dec,
                                      NackResidual mode, uint64_t late_fills) {
  const auto s = dec.stats(0);
  const uint64_t abandoned = s.syms_abandoned - s.syms_abandoned_stale;
  const uint64_t expected =
      s.syms_delivered + s.syms_recovered + s.syms_retx + abandoned;
  uint64_t lost = abandoned;
  switch (mode) {
    case NackResidual::kFilled: break;
    case NackResidual::kAll: lost += s.syms_retx; break;
    case NackResidual::kTimely: lost += std::min(late_fills, s.syms_retx); break;
  }
  return ResidualCounts{expected - lost, expected};
}

}  // namespace maburgs
