# Maintenance lane

A shell on the drone, driven from the GS over the radio link, and a lease
that turns the drone's video off while the GS needs the air. A GS updates
`maburd` through it with no PC on the drone's network.

## Why not an RC frame

RC frames carry `RC_VERSION` and both ends refuse every other version: a
half-deployed pair has no link at all, by design. The lane exists for the
opposite case, a GS that is newer than the drone, so it has its own magic
and its own envelope version. The envelope only moves bytes; what they mean
is up to the shell on the drone, so a new command never needs a new
envelope. A GS keeps speaking every envelope version a drone in the field
may run; a drone answers a kind it does not know with a close, never
silence.

The lane has no tag: anyone on the channel can open a shell. It assumes a
trusted field.

## The wire

`common/include/mabur/maint_wire.h`, little-endian, CRC16-CCITT over every
byte before it (bytes after the CRC, the 802.11 FCS, are ignored):

```
u16 magic "MT" | u8 version = 1 | u8 kind | u32 session | u32 seq | u32 ack |
u8 flags | i16 status | u16 len | u8[len] data (at most 1200) | u16 crc
```

| kind | direction | meaning |
|---|---|---|
| 1 OPEN | GS -> drone | run `data` with `/bin/sh -c`; empty `data` = `/bin/sh -i`. A repeat for the current session is answered, not run again; another session id replaces the current session |
| 2 DATA | both | stream bytes at offset `seq`, `ack` = next offset expected; GS -> drone is the command's stdin, drone -> GS its stdout and stderr merged |
| 3 CLOSE | both | GS: end the session, kill the command. Drone: no such session, `status` says why |
| 4 VIDEO | both | GS: video off for `data[0..1]` (u16) ms from now, 0 = on now. Drone: the lease it took |

- `flags` bit0 FIN: the sender's stream ends after this frame's data. The
  drone's FIN carries the command's exit status (the exit code, or 128 + the
  signal). FIN takes one offset after the last byte, so an ack past it
  proves it arrived.
- `flags` bit7: set on every frame the drone sends, on none the GS sends. A
  GS with two cards hears its own frames on the second one; the drone
  ignores frames that carry the bit.
- Drone CLOSE status: 0 closed on request, -1 the FC is armed, -2 the
  command did not start, -3 no such session (a restarted drone), -4 a kind
  this drone does not know, -5 nothing heard from the GS for 30 s.

Both ends run the same stream machine (`maint::Stream`): go-back-N with at
most 16 frames of 1200 bytes in flight, a resend from the oldest unacked
byte after 300 ms without ack progress or on the second duplicate pure ack,
in-order delivery only, a pure ack 5 ms after data when no data rides on the
way back. A receiver whose reader has stopped drops new data (64 kB held),
and the sender's resends carry it later.

## The drone end

`drone/src/maint_server.h`, run by maburd's `mbr-maint` thread:

- one session at a time; the command runs in a process group of its own
  with default signal dispositions and `SCHED_OTHER`, its stdin and output
  on non-blocking pipes. A command that must outlive maburd (the update's
  swap script) detaches itself with `setsid`.
- no session opens while the FC reports ARMED (a report at most 3 s old);
  arming closes an open session and ends the video lease.
- the video lease is clamped to 10 s; while it runs the hot thread stops
  encoding and sending, as it does for a calibration session. The GS renews
  it; when the GS goes away video comes back by itself.
- the lane works with or without a published session: a GS at another
  `RC_VERSION` reaches it too.
- frames go out on the Telem path (control radiotap, the shared Telem
  802.11 sequence). Log lines: `maburd maint: session N opened`,
  `maburd maint: video off (lease)` / `on`.

## Updating maburd over the lane

The GS uploads maburd, the config, the init script and
`mabur-swap.sh` to `/tmp/mabur-up` and starts the script detached. maburd is
what carries the shell, so each step is driven by a maburd that runs:

1. trial: the script stops the init loop and the old maburd and runs the new
   one from RAM on the new config. Flash is not touched. Without a commit
   within 120 s, or when the new maburd exits, it starts the old one again
   from flash (`state` = `restored`).
2. commit: the GS reaches the drone through the new maburd (the shell's
   parent is `/tmp/mabur-up/maburd`) and creates `/tmp/mabur-up/commit`. The
   script deletes the old binary (the overlay has no room for two), copies
   the new files to flash, checks them by sha256 and starts the init loop
   (`state` = `committed`). A power cut during these seconds can leave the
   drone without maburd; a PC install recovers it.

The script and the GS side live in Palianytsia
(`release/drone/mabur-swap.sh`, `crates/runtime/src/drone_update.rs`).
