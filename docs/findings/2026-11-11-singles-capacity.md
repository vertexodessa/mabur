# Air capacity with single frames — 2026-11-11

**Status:** measured; the bundle's `air_clock.efficiency_20/_40` are these numbers. Current: docs/airtime-model.md "7. Per-MCS capacity and per-rung A-MPDU"

Question: the bundle ships aggregation off (`ampdu.max_num = 0`), but its
efficiency tables held agg6 numbers at mcs4-7 (20 MHz) and at every MCS
(40 MHz). What does the air deliver with single frames at every rate?

Method: as `2026-09-17-bandwidth-sweep.md`, with one correction (finding
1): `linkbench-tx` sends QoS-Data frames, as `maburd` does. Drone
RTL8822E with `maburd` stopped for the sweep, `linkbench-rx` on a ground
card (RTL8812AU) on the bench, ch 144, prod FEC geometry (symbol 332,
bpb 4, window 64, overhead 0.5; 1435 B per frame as the receiver counts
them), `--ldpc --stbc` as the video flies, tx-threads 4, TX power lowered
for bench distance (`--pwr-mode offset`, 0 to -7.25 dB by rate), offered
load above capacity at every point (the sender reports `under target`),
15 s per point, median of the steady seconds. Efficiency = received
frame bytes per second / nominal HT rate. The sender's completed frames
per second agree with the receiver's within 1 % at every point but mcs0
(5 % at 20 MHz, 2 % at 40 MHz).

## Delivered air, QoS-Data frames, LDPC+STBC

| mcs | HT20 nominal | singles | eff | agg6 | eff | HT40 nominal | singles | eff | agg6 | eff |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 6.5  | 5.84  | **0.898** | —     | —     | 13.5  | 11.74 | **0.870** | —     | —     |
| 1 | 13   | 11.34 | **0.872** | —     | —     | 27    | 21.83 | **0.809** | —     | —     |
| 2 | 19.5 | 16.39 | **0.841** | —     | —     | 40.5  | 30.80 | **0.760** | 34.88 | 0.861 |
| 3 | 26   | 21.00 | **0.808** | —     | —     | 54    | 39.04 | **0.723** | 46.47 | 0.861 |
| 4 | 39   | 30.17 | **0.774** | 33.73 | 0.865 | 81    | 53.01 | **0.654** | 69.75 | 0.861 |
| 5 | 52   | 37.81 | **0.727** | 44.92 | 0.864 | 108   | 62.92 | **0.583** | 92.06 | 0.852 |
| 6 | 58.5 | 41.34 | **0.707** | 49.80 | 0.851 | 121.5 | 69.56 | **0.573** | 87.81 | 0.723 |
| 7 | 65   | 44.12 | **0.679** | 56.15 | 0.864 | 135   | 73.42 | **0.544** | 95.12 | 0.705 |

Frame loss at the receiver 0.00-0.06 % (one mcs4 run at 20 MHz 1.24 %,
its repeat 0.02 % with the same rate). Repeats at 20 MHz: mcs0 5.84 /
5.84, mcs4 30.17 / 30.16 / 30.14, mcs5 37.81 / 37.82, mcs7 44.12 / 44.13.

## Findings

1. **The frame type decides the result.** linkbench used to send its
   bodies behind a probe-request header (frame control 0x40). The radio
   driver routes data frames to the data queue and aggregates only those;
   a management frame rides the management queue. With probe-request
   frames the same sweep reads 0.929 / 0.870 / 0.831 / 0.787 / 0.729 /
   0.670 / 0.643 / 0.612 at 20 MHz and 0.870 / 0.782 / 0.717 / 0.663 /
   0.575 / 0.498 / 0.480 / 0.450 at 40 MHz, and `--ampdu 6` changes
   nothing (mcs4 28.43 against 28.42 Mbit/s, mcs7 39.80 against 39.80)
   although the tool logs `A-MPDU ON`. Those numbers describe the
   management queue, not the video. The single-frame points of
   `2026-09-17-bandwidth-sweep.md` and `2026-09-23-bw40-sweep.md` are
   probe-request numbers too (and without STBC: mcs5 0.678 and mcs7 0.630
   reproduce that way).
2. **It agrees with what maburcal measures through maburd.** On this
   unit maburcal's capacity sweep had written 0.88 / 0.85 / 0.82 / 0.80
   with singles at mcs0-3 and 0.86 / 0.86 / 0.84 / 0.86 with agg6 at
   mcs4-7; linkbench with data frames reads 0.90 / 0.87 / 0.84 / 0.81 and
   0.87 / 0.86 / 0.85 / 0.86.
3. **A single data frame costs 70-90 µs beyond its payload time at
   mcs4-7** (frame period minus 1435 B at the nominal rate: 86 / 83 / 81 /
   84 µs at 20 MHz, 75 / 76 / 71 / 71 µs at 40 MHz) and more at the slow
   rates (199 / 129 / 112 / 105 µs at mcs0-3 at 20 MHz). On the management
   queue it is 100-135 µs at every rate.
4. **What aggregation off costs in capacity.** At 20 MHz 11 % at mcs4,
   16 % at mcs5, 17 % at mcs6, 21 % at mcs7. At 40 MHz 12 % at mcs2, 16 %
   at mcs3, 24 % at mcs4, 32 % at mcs5.
5. **The sender is the limit above about 90 Mbit/s.** agg6 at 40 MHz
   delivers less at mcs6 (87.8) than at mcs5 (92.1) and 95.1 at mcs7, so
   those two points are bound by the drone's USB or CPU, not by the air.
   Every other point follows the rate.
6. **Tables for the bundle** (two digits): 20 MHz
   `[0.90, 0.87, 0.84, 0.81, 0.77, 0.73, 0.71, 0.68]`, 40 MHz
   `[0.87, 0.81, 0.76, 0.72, 0.65, 0.58, 0.57, 0.54]`.
