# The wide-TX receive-gain floor is per unit — 2026-11-11

**Status:** `radio.wide_tx_igi_floor` is a config key with default 0x35; `maburcal`'s floor sweep measures each unit's edge and raises the key for a unit that needs more, never lowering it. Current: docs/calibration.md "The floor sweep", docs/deploy.md "Supported drone cards", docs/bw40.md

Question: why does 40 MHz video on the bench drone (RTL8822E) arrive
30 ms later than 20 MHz video and lose 1-2 % of its frames, on a link
that loses 0.02 % at 20 MHz?

## What was seen

Palianytsia desktop as the only GS (RTL8812AU), ch 144, five 40 MHz rungs,
120-180 s per arm, the drone on walls measured for this unit.

- Capture-to-complete delay of base AUs: p50 58.5-59.2 ms, p90 154-166,
  p99 238-281 at 40 MHz; p50 27 ms, p90 31, p99 57 at 20 MHz.
- The drone's own stamps: TX queue wait p50 1-2 ms at both widths, but
  p90 8-174 ms and p99 42-264 ms at 40 MHz (p90 5, p99 12 at 20 MHz);
  USB send calls up to 45-56 ms (6 ms at 20 MHz); silences on air longer
  than 30 ms 251-407 per minute (2 per minute at 20 MHz).
- Not the NACK, not the ladder's residual rule, not foreign traffic:
  `linkbench-tx` saturated at 40 MHz runs at the full rate with carrier
  sense on and off alike.

## Cause

`maburd` compiled `wide_tx_igi_floor = 0` for the 8822E image; linkbench
left the driver's default 0x35. With `linkbench-tx --wide-igi-floor`
(saturated single frames, mcs2 at 40 MHz, 8 s per value):

| floor | frames/s (min second - median) |
|---|---|
| 0, 0x1e | 0 |
| 0x20 | 0 - 99 |
| 0x22 | 2130 - 2484, repeat 2481 - 2484 |
| 0x24 to 0x35 | 2481 - 2838 (the full rate) |

At 20 MHz the floor changes nothing (2484 frames/s at mcs4 either way).
So this unit holds every 40 MHz frame below 0x22-0x24. Under `maburd`
the hold is intermittent rather than total, presumably because the
uplink frames the drone hears move its gain; not verified.

## Proof on the video path

A `maburd` with the floor at 0x28, the same arms:

| 40 MHz | floor off | floor 0x28 | floor 0x28, bitrate cap 14 Mbit/s |
|---|---|---|---|
| capture-to-complete p50 / p90 / p99 | 59 / 154-166 / 238-281 ms | 27 / 48 / 110 ms | 24 / 28 / 37 ms |
| union loss | 1.3-2.1 % | 0.11 % | 0.03-0.10 % |
| truncated AUs per arm | 24-52 | 3 | 0-1 |

The middle column sat on rung 2; on rungs 3-4 the 40 MHz ladder asks
18-20 Mbit/s, more than this drone's send path delivers in time, which is
what the last column removes.

## Open

- The uplink cost of the floor (about 1 dB per step by the driver's
  estimate) is not measured: the bench signal is too strong to see it.
- One channel, one temperature, 8 s per value. A calibration step should
  scan longer, on more than one channel, and store the lowest clean value
  plus a margin next to the power walls.
