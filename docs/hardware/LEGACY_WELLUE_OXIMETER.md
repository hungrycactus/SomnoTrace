# SomnoTrace Dual-Protocol Oximeter Architecture

**Scope:** the oximeter subsystem added to SomnoTrace for pulse-oximeter ring
synchronisation. Two incompatible Wellue/Viatom BLE protocols are supported as
pluggable backends behind one shared sync/storage layer:

| Backend | Devices | Protocol | Code |
|---|---|---|---|
| **OxyII** | O2 Ring S, SleepHQ O2 Ring Pro (T8520, SHQO2Pro, `S8-AW`) | `0xA5` framing (`oximeter_oxyii.c`) |
| **Legacy** | Wellue P02 / O2Ring / Checkme-O2 family | `0xAA` framing (`oximeter_legacy.c`) |

This document is the byte-level reference for both protocols and describes how
the backends plug into the common layer. Clean-room implementations: studied
from published reverse-engineering documentation only (see References); no
third-party source copied.

---

## 1. Architecture

```
                       ┌──────────────────────────────┐
                       │        net_provision         │  HTTP API + portal UI
                       │  /api/ox/scan /pair /forget  │
                       └───────────────┬──────────────┘
                                       │ oximeter_* public API (oximeter.h)
                       ┌───────────────▼──────────────────────────────┐
                       │      COMMON LAYER (oximeter_oxyii.c)         │
                       │  • NimBLE watch task (passive scan, 15 s)    │
                       │  • advert classification → proto tag         │
                       │  • pair_task / pull_task orchestration       │
                       │  • served-curfew sleep model                 │
                       │  • NVS persistence ("oximeter" namespace)    │
                       └───────┬──────────────────────┬───────────────┘
                               │                      │
              OX_PROTO_OXYII   │                      │  OX_PROTO_LEGACY
              ("oxyii")        │                      │  ("legacy")
               ┌───────────────▼────────┐   ┌─────────▼─────────────────┐
               │  oximeter_oxyii.c      │   │  oximeter_legacy.c        │
               │  0xA5 frame codec      │   │  0xAA frame codec         │
               │  AUTH/SETUP session    │   │  INFO JSON                │
               │  F1/F2/F3/F4 file ops  │   │  FILE_OPEN/READ/CLOSE     │
               │  own GAP/GATT plumbing │   │  own GAP/GATT plumbing    │
               └───────────┬────────────┘   └─────────┬─────────────────┘
                           │  ox_store_* (oximeter_store.c)              │
               ┌───────────▼─────────────────────────────▼───────────────┐
               │ inbox/<name>.part → validate → files/<serial>/<name>    │
               │ index.json (dedupe) · paired.json · NVS mirror          │
               └───────────┬─────────────────────────────────────────────┘
                           ▼
                     SD card (FatFS)
```

### File map

| File | Owns |
|---|---|
| `main/oximeter.h` | Public API, state strings, `OX_PROTO_*` identifiers |
| `main/oximeter_oxyii.c` | Common layer (scan/classify/watch/pair/NVS) **and** the OxyII backend |
| `main/oximeter_legacy.h/.c` | Legacy backend only (codec, transport, INFO, downloads, adv matchers) |
| `main/oximeter_store.h/.c` | SD persistence shared by both backends |
| `ADD_LEGACY_OXIMETER.md` | This document |

Backends never call each other and never share statics. Each owns its GAP event
handler, GATT handles and semaphores; exactly one session runs at a time under
the common ops mutex.

---

## 2. Runtime lifecycle

### 2.1 Pairing (UI → `/api/ox/pair`)

```
UI scan → results carry {"proto":"legacy"|"oxyii"}
   ↓ user taps Pair
pair_task: address in scan cache?
   ├─ no  → active re-scan (4 s) → classify again
   ↓
dispatch on classified protocol
   ├─ legacy: legacy_sync_session(download=false)
   │          connect → INFO → SN → disconnect          (identify-only)
   └─ oxyii : connect → AUTH/SETUP → GET_INFO → SN → disconnect
   ↓
persist SN + firmware + name_prefix + addr + protocol
(NVS keys + paired.json) → state = paired
```

Pairing is deliberately read-only so it finishes in seconds; recordings are
pulled by the background watch immediately afterwards.

### 2.2 Watch loop & sleep guarantee

The ring powers off ≈120 s after entering standby **if no BLE connection
resets its timer**. Every connection restarts that countdown, so the watch
treats connections as scarce:

```
watch cycle every ~15 s: passive scan (listen-only, invisible to ring)
   ├─ no paired-proto hit for 4 consecutive scans (~60 s radio silence)
   │     ⇒ ring considered gone (slept / worn / moved);
   │        served flag cleared → next appearance is a fresh window
   ├─ hit while served
   │     ⇒ CURFEW: no connections at all while visibility continues
   │        (a docked/charging ring advertises forever and must be left
   │         alone so its power-off timer can finally expire)
   │     ⇒ exception: after 30 min of continuous visibility one
   │        scheduled re-probe runs (recording-made-while-visible valve)
   └─ hit while not served
         ⇒ connect → identify (SN must match pairing) → download every
            recording absent from index.json → disconnect → serve
```

Worn rings stop matching the scan (OxyII switches to recording-mode mfg
`0x036F`; legacy rings leave standby), which naturally builds the absence
streak while recording.

---

## 3. Protocol comparison at a glance

| Property | OxyII (`0xA5`) | Legacy (`0xAA`) |
|---|---|---|
| Request lead | `0xA5` | `0xAA` |
| Response lead | echoes command frame style | `0x55`, byte 1 is a **STATUS** (0 = OK) |
| Header size | 7 B | 7 B |
| Complement byte | `~CMD` (bitwise NOT) | `CMD ^ 0xFF` (request) / `STATUS ^ 0xFF` (response) |
| Sequence/block | 1-byte SEQ (+1-byte FLAG) | 2-byte BLOCK, little-endian |
| Length field | 2-byte LE | 2-byte LE; total frame = LEN + 8 |
| CRC-8 | poly `0x07`, init `0x00`, MSB-first, over all bytes except trailing CRC | identical polynomial |
| Encryption/auth | AUTH handshake (lepucloud-derived XOR key) | none |
| Write transport | single write-without-response after MTU exchange | ≤20-byte chunks, sequential WRITE_REQUESTs |
| Response matching | opcode echo | strict lockstep (one outstanding request) |
| File listing | `F1`: count + fixed 16-byte names | `INFO` JSON `FileList`, comma-separated names |
| File open size | `F2` reply, LE32 at payload[0..3] | `FILE_OPEN` reply DATA[0..3] LE32 |
| File read addressing | absolute byte offset in payload | block number in BLOCK header field |
| Close | `F4` (also clears recording-handle wedge) | `FILE_CLOSE` |

Both use **CRC-8 / poly 0x07 / init 0x00 / no reflection**, computed over every
byte of the frame except the final CRC byte itself. (The legacy reference
implements this as a per-bit XOR table; it is bit-for-bit equivalent to the
MSB-first shift form shown here — verified against 20 000 random buffers.)

---

## 4. Legacy Wellue protocol (`0xAA`) — byte level

### 4.1 GATT

| Role | UUID |
|---|---|
| Service | `14839ac4-7d7e-415c-9a42-167340cf2339` |
| Notify (ring → us) | `0734594a-a8e7-4b1a-a6b1-cd5243059a57` |
| Write (us → ring) | `8b00ace7-eb0b-49b0-bbe9-9aee0a26e1a3` |

Subscribe by enabling notifications (`0x0001`) on the CCCD of the notify
characteristic. Writes are issued as ≤20-byte WRITE_REQUEST chunks, awaited
sequentially (ordering guaranteed without inter-chunk delays).

### 4.2 Frames

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

### 4.3 Commands

| CMD | Name | Request payload | Response DATA |
|---|---|---|---|
| `0x14` | INFO | empty | ASCII JSON (see 4.4) |
| `0x03` | FILE_OPEN | filename ASCII **including trailing NUL** | `[0..3]` = file size, uint32 LE |
| `0x04` | FILE_READ | empty (block number goes in BLOCK field, from 0) | next chunk of file data |
| `0x05` | FILE_CLOSE | empty | empty (status only) |
| `0x17` | READ_SENSORS | — | **intentionally unimplemented** (live streaming; layout not required and not guessed) |

### 4.4 INFO response

ASCII JSON object. All fields optional except `SN` (required for identity):

```json
{"CurBAT":"99%","FileList":"20260821101616,20260823162711","Model":"1652","SN":"22012C5328"}
```

Parser tolerances implemented: unknown fields ignored, any field order,
missing/empty `FileList` (⇒ zero recordings), whitespace-tolerant splitting,
filenames validated against `[A-Za-z0-9._-]`, length ≤ 31, no leading `.`.
Observed hardware quirk: this P02 reports names **without** the `.vld`
extension; they are stored verbatim either way.

### 4.5 Worked examples (verified vectors)

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

---

## 5. OxyII protocol (`0xA5`) — byte level

Existing backend; summarised here for completeness. Full study:
`spec/0003-o2ring-ble-sync.md`.

### 5.1 GATT

| Role | UUID |
|---|---|
| Service | `e8fb0001-a14b-98f9-831b-4e2941d01248` |
| Write | `e8fb0002-a14b-98f9-831b-4e2941d01248` |
| Notify | `e8fb0003-a14b-98f9-831b-4e2941d01248` |

### 5.2 Frame

```
byte:     0        1        2        3      4       5 – 6       7 … 7+N−1    last
       +--------+--------+--------+------+-------+------------+-------------+-------+
       |  0xA5  |  CMD   |  ~CMD  | FLAG |  SEQ  | LEN(u16 LE)|   DATA[N]   |  CRC  |
       +--------+--------+--------+------+-------+------------+-------------+-------+
```

Same CRC-8 as the legacy protocol. Session bootstrap sends `AUTH (0xFF)` with a
16-byte payload derived from `MD5("lepucloud")` (even-indexed bytes +
serial-prefix `"0000"` + timestamp nibbles, XORed with the MD5), followed by
`SETUP (0x10)`.

### 5.3 Commands used

| CMD | Name | Notes |
|---|---|---|
| `0x00` | GET_CONFIG | part of file-session prep |
| `0x04` | LIVE_B | contact probe; payload[5]: `0x00` off-finger, `0x01` worn, `0x03` stale file handle ("F1 wedge", cleared with F4) |
| `0x10` | SETUP | after AUTH |
| `0xC0` | SET_UTC_TIME | 8-byte local time struct |
| `0xE1` | GET_INFO | firmware @payload[9..16], serial len @[37] + string |
| `0xE4` | GET_BATTERY | available, unused today |
| `0xF1` | GET_FILE_LIST | count + 16-byte fixed-width names |
| `0xF2` | READ_FILE_START | payload = 20 B: name (≤16, NUL-padded) + type word; reply LE32 size |
| `0xF3` | READ_FILE_DATA | payload = LE32 absolute offset; reply = chunk |
| `0xF4` | READ_FILE_END | closes handle |

Pull loop: `SET_UTC → GET_CONFIG → F4 → F1 → (F2 → F3[offset]… → F4) × N`,
skipping names already finalised in index.json.

---

## 6. Advertisement classification

Both families are Viatom-made and may advertise manufacturer id **0xF34E**, so
the classifier applies matches in priority order (first hit wins):

| Order | Test | Result |
|---|---|---|
| 1 | name contains (case-insensitive): `O2RING, CHECKME_O2, CHECKO2, SLEEPU, SLEEPO2, WEARO2, KIDSO2, BABYO2, OXYLINK` | legacy |
| 2 | name prefix `S8-AW` / `SHQO2Pro`, **or** mfg id `0xF34E` | oxyii |
| 3 | raw AD (types 0x06/0x07) contains service UUID `14839ac4-…` | legacy |

Additional guards: mfg `0x036F` (OxyII worn/recording) is skipped entirely —
never a pull candidate, and counts as *absence* for the served model. Every
classified advert logs `name/rssi/addr/mfg/proto` at INFO; unmatched adverts
log at DEBUG.

---

## 7. Storage layout (SD card)

```
/somnotrace/.somnotrace/oximetry/
├── inbox/
│   └── <name>.part            ← streamed download target (append-only)
├── files/
│   └── <serial>/
│       ├── <name>.bin         ← OxyII Format-A recordings
│       └── <name>             ← legacy native .vld (verbatim, no suffix)
├── paired.json                ← {"serial","firmware","name_prefix",
│                                   "last_addr","protocol"}
└── index.json                 ← [{"serial","name","bytes","finalised"}, …]
```

Promotion rules (all-or-nothing; partial files are never exposed):

| Backend | Validator | Destination | On failure |
|---|---|---|---|
| OxyII | Format-A trailer magic `48 12 5A DA` present at `size−44` | `files/<serial>/<name>.bin` | kept as `.part`, indexed `finalised:false` (retry later) |
| Legacy | `part size == FILE_OPEN declared size` (exact) | `files/<serial>/<name>` verbatim | `.part` deleted, nothing indexed (full retry later) |

Duplicate suppression: before downloading, each backend checks
`ox_store_index_check(serial, name)`; entries marked `finalised:true` are
skipped. Persistence: NVS namespace `oximeter` mirrors paired.json
(`serial`, `firmware`, `name_prefix`, `last_addr`, `protocol`). Records written
before the backend rename carry `"protocol":"o2ring"` — still accepted and
mapped to `legacy`.

---

## 8. Deliberately deferred

| Item | Why |
|---|---|
| `READ_SENSORS (0x17)` live SpO₂/HR streaming | Not needed for stored-recording sync; response layout intentionally left undefined rather than guessed |
| VLD decoding / analysis | Recordings are preserved byte-for-byte; decoding belongs to a separate future parser |
| OxyII `GET_BATTERY (0xE4)` | Available in firmware, not surfaced yet |

## 9. Hardware observations (P02, fw as of 2026-08)

* Advertises as `O2Ring XXXX` with mfg `0xF34E`; connectable only in standby
  (off-finger, countdown finished).
* `FileList` names arrive **without** the `.vld` extension.
* `Model` reported as numeric string (e.g. `"1652"`), `SN` like
  `22012C5328`, `CurBAT` as `"NN%"`.
* Negotiated MTU stays 23 → all notification payloads ≤20 B; frames up to the
  ~512 B read-block size are reassembled from many fragments.
* Docked/charging rings keep advertising indefinitely — hence the
  visibility-based curfew (§2.2) instead of a fixed window.

## 10. References

* farolone/wellue-o2ring-protocol — protocol documentation (frames, commands,
  CRC, VLD header)
* MackeyStingray/o2r (+ bleak fork) — original protocol research client
* `spec/0003-o2ring-ble-sync.md` — OxyII protocol study
