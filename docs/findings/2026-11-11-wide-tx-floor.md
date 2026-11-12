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

## The hold and its release

Measured with `linkbench-tx` on the same unit, quiet channel, MCS 2 at
40 MHz, sends that wait on the chip without a timeout as `maburd`'s do
(`--no-cancel`), the floor moved under the running sender
(`--wide-igi-floor-at`):

| start | change at 2 s | result |
|---|---|---|
| 0x28 | none | full rate (about 2700 frames/s) |
| 0x20 | none | held: 0 frames/s after the first second |
| 0x20 | to 0x35, 0x50 or 0x77 | full rate within a second, 10 of 10 |
| 0x20 | to 0x7f | nothing more sent, 2 of 2 |

- The hold is not carrier sense: with `--no-cca` a start at 0x20 holds the
  same (2 of 2). The TX FIFO page counters and `TXDMA_STATUS` (0) read the
  same in a live and in a held sender.
- A raised floor releases the hold, unless the new floor is at the top of
  the field. A fresh sender at 0x7c-0x7f stops for good at the driver's
  first runtime tick in 9 of 17 starts (0x77-0x7b: 0 of 10): the driver's
  energy-detect threshold follows the receive gain (IGI + 8) and left its
  byte there. With the threshold held at 127 in the driver, 12 of 12
  starts at 0x7c-0x7f run. The floor sweep's reference point and its
  watchdog therefore use 0x50 (`cal_floor.h` `kFloorRef`).
- With sends that time out (the tool's default) a released sender can stay
  dead with `TXDMA_STATUS` 0x0000c000 or 0x00046004: the chip resumes on
  aggregates the timeout cut short. That state is the tool's, not
  `maburd`'s.
- A blast's first second runs above the rest while the send queue fills:
  under `maburd` 2859 to 2871 frames against 2679 to 2697 in every later
  second of 4 s and 8 s dwells, here 2764 against 2682 to 2694. No later
  second dips, so the driver's runtime tick costs the rate nothing that
  shows. A reference rate read from a first second is not what a longer
  dwell's worst second can reach (2682 is 93.5 % of 2868).
- The tool's own stats line (`frm`) is printed once per pacing burst of
  354 frames, so its seconds are up to an eighth long or short (2484 and
  2838 alternate at a steady 2686); the `sent ... heard` line is cut on the
  second.
- On a channel with other traffic the hold does not show: a floor sweep
  there read 1521 frames/s at 0x20 rising to 2652 at 0x36 with no held
  second, while the drone's receiver took 78 to 551 frames a second. On
  the quiet channel it took 0 to 2 a second and 0x20 held completely. An
  edge is measurable on a quiet channel only, so the floor sweep counts
  what the drone hears and ends without a result above 5 frames/s
  (`cal_floor.h` `kFloorQuietMaxHeardPerS`).

## Heard frames and the hold

Same sender, quiet channel, floors 0x20 (held), 0x22 (marginal) and 0x24
(the edge). A second card injects evenly spaced foreign 20 MHz MCS 0
frames (`linkbench-tx --inject-fps`, 0.3 ms each; "long" = 1400 bytes,
1.8 ms). 9 s per point, frames counted on the second; mean rate, and in
brackets the worst second, in percent of 0x24 in silence (2686 frames/s,
worst second 2682). The sender's own count of frames heard follows the
injected rate within 1 frame/s.

| heard, frames/s | 0x20 | 0x22 | 0x24 |
|---|---|---|---|
| 0 (the channel's own 1) | 0 (0) | 93 (89) | 100 (100) |
| 5 | 29 (8) | | 100 (99) |
| 10 | 41 (27) | 77 (68) | 100 (100) |
| 20 | 54 (49) | | 99 (99) |
| 40 | 71 (65) | 85 (80) | 99 (99) |
| 80 | 84 (83) | | 98 (97) |
| 160 | 89 (86) | | 95 (95) |
| 320 | 88 (87) | | 90 (90) |
| 20 long | 57 (47) | | 96 (96) |
| 80 long | 77 (73) | | 86 (86) |

- The hold has no rate at which it stops. Each heard frame frees the
  sender for a while and the rate climbs with the count: a third of the
  full rate at 5 frames/s, half at 20, five sixths at 80. At 160 and 320 a
  held floor still sends less than the edge does under the same traffic
  (89 against 95, 88 against 90).
- With frames the receiver delivers, a held floor never reaches the
  sweep's pass bar (no second under 95 % of the reference): no 4 s window
  of any of these 0x20 runs passes. Frames it does not deliver are
  another matter, below.
- Traffic makes the walk read the edge too high. Carrier sense defers to
  the foreign frames and a healthy floor loses about their share of the
  air: 0x24 is at the bar with 160 short frames/s (5 % of the air) and
  under it with 320 short or 80 long ones (10 % and 14 %). A deafer
  receiver defers less, which is the slow rise the sweep on the busy
  channel showed (1521 frames/s at 0x20 to 2652 at 0x36).
- The marginal floor loses rate when frames are heard (93 % in silence,
  77 % at 10 frames/s, 85 % at 40).
- At the sweep's bound of 5 frames/s heard a healthy floor reads 99 % or
  more; its first loss past 2 % is at 80 short frames/s or 20 long ones.

With the injector on the secondary 20 MHz channel only, the sender's
receiver delivers none of the frames (0.9 frames/s, as in silence):

| injected on the secondary | 0x20 | 0x24 |
|---|---|---|
| 80 frames/s | 19 (15) | 97 (97) |
| 320 frames/s | 0 (0) | 89 (89) |
| 80 frames/s long | | 86 (85) |

- The hold stays, so the walk still rejects a held floor.
- A healthy floor loses the same rate as with traffic on the primary, and
  the heard count does not see it: with a tenth of the air taken on the
  secondary half the walk reads the edge too high or finds none, and the
  sweep does not say that the channel is not quiet.

### Level, and frames the receiver does not deliver

The injector above reaches the sender at -33 dBm. Its power index moves
that to -47 dBm at most, and a decodable frame at -45 dBm does what one
at -33 dBm does (0x20: 52 % at 20 frames/s, 80 % at 80). A neighbour at
-60 to -80 dBm is not measured: the bench has no attenuator.

At power indexes 7 to 10 the bench card's frames arrive at about -45 dBm
and the saturated sender's receiver delivers about one in forty of them.
Those frames free the hold almost completely, 12 s per point:

| injected, frames/s | counted as heard | 0x20, mean (worst second) | 4 s windows at the pass bar |
|---|---|---|---|
| 20 | 1.0 | 92 (78) | 0 of 8 |
| 40 | 1.1 | 96 (85) | 6 of 8 |
| 80 | 7.6 | 95 (92) | 2 of 8 |
| 120 | 12.7 | 97 (95) | 5 of 8 |
| 160 | 8.0 | 97 (96) | 8 of 8 |

- 0x22 under 20 such frames/s: 97 % (94), 6 of 8 windows. 0x24 under 120:
  96 %, under 160: 95 %, so at 160 the held floor sends more than the
  edge does.
- A held floor passes the bar while the heard count reads what silence
  reads (1 frame/s). A quiet check by delivered frames does not see this
  traffic, and the walk then reports an edge under the real one or a unit
  that never stalled.

### The driver's carrier-sense count

The driver's gain-control pass reads the chip's OFDM carrier-sense
counter every 2 s. Summed over those windows (devourer `RxActivity`, no
extra register read) it gives events a second:

| on the channel | floor | events/s |
|---|---|---|
| silence | 0x22 (81 s), 0x24 (28 s), 0x28, 0x30, 0x36, 0x50 (8 s each) | 0.0 |
| 40 frames/s, delivered | 0x24 | 39.5 |
| 20 frames/s, not delivered | 0x20 | 20.2 |
| 40 frames/s, not delivered | 0x20, 0x24 | 40.0, 39.9 |
| 80 frames/s of 1.8 ms, not delivered | 0x24 | 79.8 |
| 80 and 320 frames/s on the secondary channel | 0x24 | 0.0 |

- It counts a frame start whether or not the frame is delivered, which
  is what the heard count misses. It is blind to the secondary channel.
- 0x20 in silence is not one state. Of ten runs of 20 s, three stayed
  held (0 frames/s but for one stray second, 0 to 3 events in all). Seven
  went, after 0 to 16 s, into a state of 18 to 21 events a second (31 to
  46 a window, every one a false alarm by the driver's count) in which
  the transmitter sends 700 to 1450 frames/s, at most 60 % of the full
  rate in any one second; one run left the state again after 12 s. 0x22 never shows it. What starts it is not
  identified. The 36 % once read at 0x20 with the driver's info log on is
  this state; it comes without the log too.
- So the low end of the 0x20 column in "Heard frames and the hold" (12 s
  a point, 29 % with 5 frames/s heard) cannot be told from this state.
  The higher points stand: 84 % with 80 delivered frames/s and 96 % with
  40 undelivered are above anything the state reaches.
- Under `maburd`'s own floor sweep on the quiet channel the counter reads
  0 at the reference, at 0x22 and over the confirmation. The windows lag
  a point by up to one: a sweep that took the counter's movement during a
  point as that point's count put 41 events of a 0x20 in this state into
  the 0x22 after it, so a point counts whole windows inside it only.
- The marginal 0x22 passed its dwell and its confirmation in one sweep
  of three (worst second 2607 of 2682, 96 %) and was reported as the
  edge; the other two read 0x24. The margin of 4 steps is for that.

## Open

- The floor sweep's quiet-channel check counts frames the receiver
  delivers and, over the stretch that decides its report, the driver's
  carrier-sense events. Not in it: traffic on the secondary 20 MHz
  channel alone (it fails healthy floors) and the air time of what is
  heard; both need a measure of the busy share of the whole 40 MHz.
- The check under real traffic is proven on the host only: the bench has
  one injector card and the sweep needs it as the GS.
- What puts 0x20 into its 19-events state in silence.
- What a neighbour at -60 to -80 dBm does to the hold.
- The uplink cost of the floor (about 1 dB per step by the driver's
  estimate) is not measured: the bench signal is too strong to see it.
- One channel, one temperature, 8 s per value. A calibration step should
  scan longer, on more than one channel, and store the lowest clean value
  plus a margin next to the power walls.
