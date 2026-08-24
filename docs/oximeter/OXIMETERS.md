# SomnoTrace Oximeter Subsystem

**Scope:** the pulse-oximeter ring synchronisation subsystem. Two
incompatible Wellue/Viatom BLE protocols are supported as pluggable
backends behind one shared sync/storage layer:

| Backend | Devices | Protocol | Code |
|---|---|---|---|
| **OxyII** | O2 Ring S, SleepHQ O2 Ring Pro (T8520, SHQO2Pro, `S8-AW`) | `0xA5` framing (`oximeter_oxyii.c`) |
| **Legacy** | Wellue P02 / O2Ring / Checkme-O2 family | `0xAA` framing (`oximeter_legacy.c`) — byte-level spec: [`LEGACY_WELLUE.md`](LEGACY_WELLUE.md) |

Clean-room implementations: studied from published reverse-engineering
documentation only; no third-party source copied.

---

## 1. Architecture

```
                       ┌──────────────────────────────┐
                       │        net_provision         │  HTTP API + portal UI
                       │  /api/ox/scan /pair /forget  │
                       └───────────────┬──────────────┘
                                       │ oximeter_* public API (oximeter.h)
                       ┌───────────────▼──────────────────────────────┐
                       │    COMMON LAYER (oximeter_common.c)          │
                       │  • NimBLE watch task (passive scan, ~15 s)   │
                       │  • advert classification → proto tag         │
                       │  • pair_task / pull_task orchestration       │
                       │  • served-curfew sleep model                 │
                       │  • NVS persistence ("oximeter" namespace)    │
                       │  registry: oximeter_backend_t (backend .h)   │
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
| `main/oximeter_backend.h` | `oximeter_backend_t` registry interface |
| `main/oximeter_common.c` | Shared orchestration: scan/classify/watch/pair/pull, curfew, NVS |
| `main/oximeter_oxyii.c` | OxyII backend only (codec, transport, file ops) |
| `main/oximeter_legacy.h/.c` | Legacy backend only (codec, transport, INFO, downloads, adv matchers) |
| `main/oximeter_store.h/.c` | SD persistence shared by both backends |

Backends never call each other and never share statics. Each owns its GAP event
handler, GATT handles and semaphores; exactly one session runs at a time under
the common ops mutex. New families are added by registering one
`oximeter_backend_t`.

## 2. Runtime lifecycle

### 2.1 Pairing (UI → `/api/ox/pair`)

```
scan → results carry {"addr","name","rssi","proto":"legacy"|"oxyii"}
   ↓ user picks a device
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
   ├─ no paired-proto hit for 4 consecutive scans (~60 s radio silence,
   │  OX_ABSENT_STRIKES)
   │     ⇒ ring considered gone (slept / worn / moved);
   │        served flag cleared → next appearance is a fresh window
   ├─ hit while served
   │     ⇒ CURFEW: no connections at all while visibility continues
   │        (a docked/charging ring advertises forever and must be left
   │         alone so its power-off timer can finally expire)
   │     ⇒ exception: after 30 min of continuous visibility
   │        (OX_SERVED_REPROBE_MS) one scheduled re-probe runs
   │        (recording-made-while-visible valve)
   └─ hit while not served
         ⇒ connect → identify (SN must match pairing) → download every
            recording absent from index.json → disconnect → serve
```

Worn rings stop matching the scan (OxyII switches to recording-mode mfg
`0x036F`; legacy rings leave standby), which naturally builds the absence
streak while recording.

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
| Clock sync | `SET_UTC_TIME` before every pull | none — ring's own RTC timestamps filenames |

OxyII byte-level details live in `spec/0003-o2ring-ble-sync.md`; the
legacy protocol is fully specified in [`LEGACY_WELLUE.md`](LEGACY_WELLUE.md).

## 4. Advertisement classification

Both families are Viatom-made and may advertise manufacturer id **0xF34E**, so
the classifier applies matches in priority order (first hit wins):

| Order | Test | Result |
|---|---|---|
| 1 | name contains (case-insensitive): `O2RING, CHECKME_O2, CHECKO2, SLEEPU, SLEEPO2, WEARO2, KIDSO2, BABYO2, OXYLINK` | legacy |
| 2 | name prefix `S8-AW` / `SHQO2Pro`, **or** mfg id `0xF34E` | oxyii |
| 3 | raw AD (types 0x06/0x07) contains service UUID `14839ac4-…` | legacy |

Additional guards: mfg `0x036F` (OxyII worn/recording) is skipped entirely —
never connected, never offered for pairing, never a pull candidate, but
surfaced as presence `recording` so a worn ring is not mistaken for a vanished
one. Every classified advert logs `name/rssi/addr/mfg/proto` at INFO;
unrecognised adverts are summarised per scan for field diagnosis.

## 5. Presence & identity

Presence is tracked by the sync engine (`oximeter_get_presence()`):
`syncing`
(BLE session now), `ready` (visible, transfer due), `detected` (visible,
curfew), `recording` (worn & recording via its recording-mode advert),
`offline` (no adverts for several scans). `/api/status` exposure of this
field is deferred.

Family behaviour differs sharply:

- **Legacy**: radio-silent while worn and while screen-on after a
  recording — a worn ring looks like an off one. Transfers happen at the
  next appearance (take-off window or dock).
- **OxyII**: keeps advertising on-finger and through END; probes get
  LIVE_B and back off without consuming the sync window.

Identity matching prefers the **exact paired address** (hint confirmed
on first sighting each boot); same-family adverts from unknown addresses
are ignored afterwards so a neighbour's Viatom device cannot fake
presence, with automatic adoption after sustained loss (~10 scans) to
recover from MAC rotation / factory resets. The GET_INFO serial remains
the ultimate gate before any file transfer.

Live RF presence is deliberately not surfaced in the portal: legacy
rings are RF-silent while worn, so any UI label would be misleading half
the night.

## 6. Storage layout (SD card)

```
/somnotrace/.somnotrace/oximetry/
├── inbox/
│   └── <name>.part            ← streamed download target (append-only)
├── files/
│   └── <serial>/
│       ├── <name>.bin         ← OxyII Format-A recordings
│       └── <name>             ← legacy native .vld (verbatim, no suffix)
├── paired.json                ← {"serial","firmware","name_prefix",
│                                  "last_addr","protocol"}
└── index.json                 ← [{"serial","name","bytes","finalised"}, …]
```

Promotion rules (all-or-nothing; partial files are never exposed):

| Backend | Validator | Destination | On failure |
|---|---|---|---|
| OxyII | Format-A trailer magic `48 12 5A DA` present at `size−44` | `files/<serial>/<name>.bin` | kept as `.part`, indexed `finalised:false` (retry later) |
| Legacy | `part size == FILE_OPEN declared size` (exact) | `files/<serial>/<name>` verbatim | `.part` deleted, nothing indexed (full retry later) |

Duplicate suppression: before downloading, each backend checks
`ox_store_index_check(serial, name)`; entries marked `finalised:true` are
skipped. Persistence: NVS namespace `oximeter` mirrors paired.json.
Records written before the backend rename carry `"protocol":"o2ring"` —
still accepted and mapped to `legacy`. Legacy filenames are the ring's own
RTC-based UTC start timestamps; OxyII pulls additionally re-align the
clock via `SET_UTC_TIME` first.

## 7. HTTP API surface

| Endpoint / field | Notes |
|---|---|
| `/api/status` → `oximeter.state` | basic state machine string (upstream card renders this) |
| `/api/status` → `oximeter.presence` | *planned* — tracked internally today, see §5 |
| `/api/status` → `oximeter.device` | serial, firmware, addr, protocol; backend extras via `report_status()` (legacy reports model/battery/files-on-ring; OxyII does not yet); `last_seen` epoch; `last_sync`/`last_pulled` |
| `GET /api/ox/scan` | discovery across both families; elements carry `proto` |
| `POST /api/ox/scan/stop` | *planned* — cancel early, keep partial results |
| `POST /api/ox/pair` / `/api/ox/forget` | pair by address / unpair (clears RAM, NVS, `paired.json`) |

Radio-busy scan starts currently surface as a plain failure to the stock
UI ("Scan failed"); the module state recovers cleanly — retrying works.

## 8. Deliberately deferred

| Item | Why |
|---|---|
| Legacy `READ_SENSORS (0x17)` live SpO₂/HR streaming | Not needed for stored-recording sync; response layout intentionally left undefined rather than guessed |
| VLD decoding / analysis | Recordings are preserved byte-for-byte; decoding belongs to a separate future parser |
| OxyII `GET_BATTERY (0xE4)` surfacing | Command available in firmware, not reported yet |

## 9. Planned uses for ring data

1. **FTP documentation** — free, works today
   (`/somnotrace/.somnotrace/oximetry/…`; dot-folder hidden in some clients).
2. **SMB inclusion** — extend the upload scanner to the oximetry tree,
   mirrored under its own remote folder.
3. **Dashboard graphing** — piggyback on `session_graph.c` +
   uPlot channels; join recordings against CPAP MaskOn windows. OxyII
   Format-A decoding is documented in-repo (no research needed); legacy
   `.vld` decoding is the real research item.
4. **SleepHQ upload** — plausible but unverified against their API;
   fallback is decoding and wrapping as EDF (`edf_gen.c`).

## 10. References

* `spec/0003-o2ring-ble-sync.md` — OxyII protocol study & sync architecture
* [`LEGACY_WELLUE.md`](LEGACY_WELLUE.md) — legacy Wellue `0xAA` protocol reference
