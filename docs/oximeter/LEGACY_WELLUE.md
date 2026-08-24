# Legacy Wellue Ring Protocol (`0xAA`) — Byte-Level Reference

**Scope:** the proprietary BLE protocol spoken by legacy Wellue
pulse-oximeter rings — Wellue P02 / O2Ring / Checkme-O2 family. This is a
protocol specification only; how SomnoTrace integrates it (sync engine,
storage, presence) is described in [`OXIMETERS.md`](OXIMETERS.md).

Implemented by `main/oximeter_legacy.c`. Clean-room implementation:
studied from published reverse-engineering documentation only (see
References); no third-party source copied.

---

## 1. GATT

| Role | UUID |
|---|---|
| Service | `14839ac4-7d7e-415c-9a42-167340cf2339` |
| Notify (ring → us) | `0734594a-a8e7-4b1a-a6b1-cd5243059a57` |
| Write (us → ring) | `8b00ace7-eb0b-49b0-bbe9-9aee0a26e1a3` |

Subscribe by enabling notifications (`0x0001`) on the CCCD of the notify
characteristic. Writes are issued as ≤20-byte WRITE_REQUEST chunks, awaited
sequentially (ordering guaranteed without inter-chunk delays).

## 2. Frames

Request:

```
byte:     0        1        2         3 – 4           5 – 6        7 … 7+N−1    last
       +--------+--------+---------+---------------+------------+-------------+-------+
       |  0xAA  |  CMD   | CMD^FF  | BLOCK (u16 LE)| LEN(u16 LE)|   DATA[N]   |  CRC  |
       +--------+--------+---------+---------------+------------+-------------+-------+
```

Response (note: no command echo — byte 1 is a status code):

```
byte:     0        1          2         3 – 4           5 – 6        7 … 7+N−1    last
       +--------+----------+---------+---------------+------------+-------------+-------+
       |  0x55  |  STATUS  |STATUS^FF| BLOCK (u16 LE)| LEN(u16 LE)|   DATA[N]   |  CRC  |
       +--------+----------+---------+---------------+------------+-------------+-------+
```

* Total frame length = `LEN + 8`.
* `CRC` covers bytes `0 .. LEN+6` (everything except the CRC itself).
* STATUS `0x00` = success; non-zero is a device error (documented example:
  opening a file whose name lacks the terminating NUL returns error `9`).
* Responses echo the BLOCK field of the request; the implementation rejects
  mismatches as stream desynchronisation rather than guessing.

CRC-8: poly `0x07`, init `0x00`, MSB-first / no reflection, computed over every
byte of the frame except the final CRC byte itself. (The community reference
implements this as a per-bit XOR table; it is bit-for-bit equivalent to the
MSB-first shift form shown here — verified against 20 000 random buffers.)

## 3. Commands

| CMD | Name | Request payload | Response DATA |
|---|---|---|---|
| `0x14` | INFO | empty | ASCII JSON (see §4) |
| `0x03` | FILE_OPEN | filename ASCII **including trailing NUL** | `[0..3]` = file size, uint32 LE |
| `0x04` | FILE_READ | empty (block number goes in BLOCK field, from 0) | next chunk of file data |
| `0x05` | FILE_CLOSE | empty | empty (status only) |
| `0x17` | READ_SENSORS | — | **intentionally unimplemented** in SomnoTrace (live streaming; layout not required and not guessed) |

## 4. INFO response

ASCII JSON object. All fields optional except `SN` (required for identity):

```json
{"CurBAT":"99%","FileList":"20260821101616,20260823162711","Model":"1652","SN":"22012C5328"}
```

Parser tolerances implemented: unknown fields ignored, any field order,
missing/empty `FileList` (⇒ zero recordings), whitespace-tolerant splitting,
filenames validated against `[A-Za-z0-9._-]`, length ≤ 31, no leading `.`.
Observed hardware quirk: this P02 reports names **without** the `.vld`
extension; they are stored verbatim either way.

## 5. Worked examples (verified vectors)

```
INFO request
  aa 14 eb 00 00 00 00 c6
        │  │  └──┴── block=0, len=0
        │  └─── cmd^ff
        └────── cmd 0x14

FILE_OPEN "20260116233312.vld\0"          (payload includes the NUL!)
  aa 03 fc 00 00 13 00 32 30 32 36 30 31 31 36 32 33 33 33 31 32 2e 76 6c 64 00 f9
                          └── len=0x0013=19 ──┘

FILE_READ block 17
  aa 04 fb 11 00 00 00 1b
             └─┴─ block=17

FILE_CLOSE
  aa 05 fa 00 00 00 00 21

INFO response (status=0, len=0x004d)
  55 00 ff 00 00 4d 00 7b 22 43 75 72 42 41 54 22 ... 7d 45

FILE_OPEN success, size = 2280 (0x000008e8)
  55 00 ff 00 00 04 00 e8 08 00 00 6f

FILE_OPEN failure, status 9 (name missing NUL terminator)
  55 09 f6 00 00 00 00 18

FILE_READ chunk (20 bytes shown)
  55 00 ff 00 00 14 00 00 01 02 ... 13 cf
```

Download loop: `FILE_OPEN` → repeat `FILE_READ` with block 0,1,2,… appending
DATA until `received == declared_size` (final block truncated defensively;
duplicate/out-of-order blocks rejected; ≤3 retries per block) → `FILE_CLOSE`
(best-effort) → exact-size promotion. Never guesses EOF from an empty or short
block.

Note on timekeeping: this protocol has **no clock-set command**. Recording
filenames carry the ring's own RTC timestamps; keeping them UTC-aligned is up
to the companion app that set the ring's clock.

## 6. Hardware observations (P02, fw as of 2026-08)

* Advertises as `O2Ring XXXX` with mfg `0xF34E`; connectable only in standby
  (off-finger, countdown finished).
* Radio-silent while worn and while screen-on after a recording — only short
  standby windows advertise.
* `FileList` names arrive **without** the `.vld` extension.
* `Model` reported as numeric string (e.g. `"1652"`), `SN` like
  `22012C5328`, `CurBAT` as `"NN%"`.
* Negotiated MTU stays 23 → all notification payloads ≤20 B; frames up to the
  ~512 B read-block size are reassembled from many fragments.
* Docked/charging rings keep advertising indefinitely.

## 7. Deliberately deferred

| Item | Why |
|---|---|
| `READ_SENSORS (0x17)` live SpO₂/HR streaming | Not needed for stored-recording sync; response layout intentionally left undefined rather than guessed |
| VLD decoding / analysis | Recordings are preserved byte-for-byte; decoding belongs to a separate future parser |

## 8. References

* farolone/wellue-o2ring-protocol — protocol documentation (frames, commands,
  CRC, VLD header)
* MackeyStingray/o2r (+ bleak fork) — original protocol research client
