# A NACK-filled symbol must not cost a rung — 2026-11-11

**Status:** fixed; the ladder's residual input is the decoder's abandonment alone. Current: docs/fec-nack.md "Residual", docs/link-adaptation.md "Inputs"

Question: why does the link leave its top rung every few seconds on a
bench link that loses 0.01-0.04 % of its frames?

Setup: one GS (Palianytsia desktop, the Rust port of this ladder, RTL8812AU)
and the reference drone (RTL8822E) on the bench, ch 144, 20 MHz rungs,
aggregation off, 150 s per arm, NACK on and off interleaved.

## What the arms show

| ladder | NACK | on the top rung | rung changes / h | IDRs per arm | video per arm |
|---|---|---|---|---|---|
| mcs0-4 | on  | 82 %, 91 %       | 507, 254      | 26, 12 | 245, 249 MB |
| mcs0-4 | off | 92 %, 94 %       | 152, 101      | 9, 5   | 247, 245 MB |
| mcs0-5 | on  | 20 %, 43 %, 28 % | 838, 914, 888 | 36-40  | 264, 282, 269 MB |
| mcs0-5 | off | 92 %, 92 %       | 127, 178      | 13, 13 | 309, 313 MB |

Every fall from the top rung with the NACK on carries the reason
`residual` with a pre-FEC util of 0, half of them within 1.3 s of the
promote (the rung-change hold is 1.2 s). The base stream abandons no symbol in any arm.

## Why

The ladder demoted on any residual symbol, and the residual counted the
symbols a NACK retransmit filled next to the ones the decoder abandoned.
The drone answers a request with a burst of resends (`nack.burst_ms`),
and those race the frame's own FEC repairs. In one mcs0-5 arm the base
stream missed 145 symbols; the NACK delivered 253 that filled a hole, 28
that were wasted and 1697 that came after the symbol was already whole.
With the NACK off the same link repairs every missing symbol from FEC
alone. So a "fill" says which copy came first, not that FEC fell short,
and each one cost a rung.

Frame order on air is not the trigger: 0.02 % of frames arrive behind a
later sequence number, one frame behind, at mcs4 and mcs5 alike.

## Fix

`ladder_residual_counts` and the s3 path count a retx-filled symbol as
arrived. The stats (`residual_counts`, delivery percent, `syms_retx`) are
unchanged, and the pre-FEC util still sees the missing direct copy.

## After the fix

Same bench, NACK on, one or two arms of 95-120 s per setting, old rule
and new rule interleaved where both are listed:

| setting | old rule | new rule |
|---|---|---|
| 20 MHz, mcs0-4: on the top rung, rung changes / h | 82-91 %, 254-507 | 91 %, 234 |
| 20 MHz, mcs0-5: the same | 20-43 %, 838-914 | 89 %, 274 |
| 40 MHz, mcs0-4: the same | 65 %, 871 | 93 %, 129 |
| 40 MHz: capture-to-complete p50 / p99 | 24.5 / 49.3 ms | 24.1 / 36.5 ms |
| 20 MHz under simulated bursts (pre-FEC loss 1 %, rungs 0-2): truncated base AUs per arm | 2, 4 | 0, 3 |
| the same: on rung 0 | 52 %, 80 % | 50 %, 49 % |

Where frames are really lost the ladder moves on util and on abandoned
symbols with either rule, and the new one truncates no more base AUs.
The 40 MHz rows are from a drone whose transmitter does not stall (a
receive-gain floor of 0x28) and whose bitrate cap matches its send path.

## Open

A rung whose bitrate the drone cannot send in time: with the cap set
above the send path's reach the new rule held the top 40 MHz rung while
122 base AUs per arm were emitted truncated, whole but late. No ladder
input counts a late frame; the old rule covered it by accident.


The resends themselves: more than ten answers per missing symbol is air
the NACK budget (`nack.air_pct`) pays for nothing. Asking only once the frame's own
repairs have arrived would cut it, at the cost of the lead time the NACK
needs.
