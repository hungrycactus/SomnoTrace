# SomnoTrace Oximeter Subsystem — Architecture

> Dual-protocol BLE sync for Wellue/Viatom pulse-oximeter rings.
> Clean-room implementations based on published reverse-engineering docs.
> No third-party source copied.

---

## 1. File Map

| File | Role |
|------|------|
| `main/oximeter.h` | Public API, state strings, protocol IDs (`OX_PROTO_OXYII`, `OX_PROTO_LEGACY`), driver enum, `oximeter_get_scanned_name()` |
| `main/oximeter.c` | **Dispatcher**: initializes both drivers, runs merged scan, dedupes results, routes pairing to active driver, exposes `oximeter_get_scanned_name()`, safeguards `get_status()` |
| `main/oximeter_internal.h` | Driver vtable (`ox_driver_ops_t`), scanned-name lookup declarations |
| `main/oximeter_backend.h` | (Legacy) Backend interface — unused in current dispatcher model |
| `main/oximeter_oxyii.c` | **OxyII backend** (Gen2: O2 Ring S, SHQO2Pro, S8-AW) — 0xA5 framing, AUTH/SETUP, F1–F4 file ops, scanned name persistence |
| `main/oximeter_legacy.c` | **Legacy backend** (Gen1: P02, O2Ring, Checkme-O2) — 0xAA/0x55 framing, INFO JSON, FILE_OPEN/READ/CLOSE, scanned name persistence |
| `main/oximeter_store.c` | **Shared storage**: SD card (FatFS) — inbox/.part → files/<serial>/<name> (Legacy) or .bin (OxyII), index.json, paired.json, NVS mirror |
| `main/oximetry_canonical.c` | Canonical SNT v3 conversion (Format-A / VLD3) for upload pipeline |
| `main/net_provision.c` | HTTP API (`/api/ox/scan`, `/api/ox/pair`, `/api/ox/forget`) + `/api/status` composition |
| `main/portal.html` | Web UI: scan, pair, forget, status display |

---

## 2. Component Diagram

```
                           ┌────────────────────────────────────┐
                           │        net_provision (HTTP)        │
                           │  /api/ox/scan  /pair  /forget      │
                           │  /api/status (composes JSON)       │
                           └──────────────┬─────────────────────┘
                                          │
                                          ▼
                           ┌────────────────────────────────────┐
                           │        oximeter.c (Dispatcher)     │
                           │  • oximeter_init()  — inits BOTH   │
                           │  • oximeter_scan()   — runs BOTH   │
                           │  • oximeter_get_scan_results()     │
                           │      — merges + dedupes (Legacy 1st)│
                           │  • oximeter_pair(addr, driver)     │
                           │      — switches active driver      │
                           │  • oximeter_get_scanned_name()     │
                           │  • oximeter_get_status() safeguard │
                           └──────────────┬─────────────────────┘
                                          │
               ┌──────────────────────────┼──────────────────────────┐
               ▼                          ▼                          ▼
    ┌───────────────────────┐   ┌───────────────────────┐   ┌───────────────────────┐
    │  oxyii_driver_ops     │   │  legacy_driver_ops    │   │  oximeter_store.c     │
    │  (OxyII backend)      │   │  (Legacy backend)     │   │  (Shared SD storage)  │
    │                       │   │                       │   │                       │
    │ • init()              │   │ • init()              │   │ • inbox/.part         │
    │ • scan()              │   │ • scan()              │   │ • files/<serial>/     │
    │ • get_scan_results()  │   │ • get_scan_results()  │   │ • paired.json         │
    │ • pair()              │   │ • pair()              │   │ • index.json          │
    │ • forget()            │   │ • forget()            │   │ • NVS mirror          │
    │ • oxyii_get_          │   │ • legacy_get_         │   │ • ox_store_promote    │
    │   scanned_name()      │   │   scanned_name()      │   │   (trailer)           │
    │                       │   │                       │   │ • ox_store_promote    │
    └───────────┬───────────┘   └───────────┬───────────┘   │   _exact (exact size) │
                │                           │               └───────────┬───────────┘
                │                           │                           │
                ▼                           ▼                           ▼
    ┌───────────────────────┐   ┌───────────────────────┐   ┌───────────────────────┐
    │  OxyII Protocol       │   │  Legacy Protocol      │   │  SD Card (FatFS)      │
    │  0xA5 framing         │   │  0xAA / 0x55 framing  │   │  /somnotrace/.        │
    │  GATT: e8fb0001...    │   │  GATT: 14839ac4...    │   │   somnotrace/         │
    │  AUTH → SETUP → F1/F2 │   │  INFO (0x14) JSON     │   │   oximetry/           │
    │  /F3/F4               │   │  FILE_OPEN/READ/CLOSE │   │     inbox/            │
    │  CRC8 poly 0x07       │   │  CRC8 poly 0x07       │   │     files/<serial>/   │
    └───────────────────────┘   └───────────────────────┘   └───────────────────────┘
```

---

## 3. Runtime Lifecycle

### 3.1 Boot & Initialization
```
oximeter_init()
  ├─ load_driver_type()  ← NVS/paired.json → s_driver_type, s_active
  ├─ oxyii_driver_ops.init()   ← allocs s_scan, semaphores, starts pull_task
  ├─ legacy_driver_ops.init()  ← allocs s_scan, semaphores, starts pull_task
  └─ s_active->init()          ← starts watch task for paired protocol
```

### 3.2 User-Initiated Scan
```
oximeter_scan(6s)
  ├─ ensure_both_inited()
  ├─ oxyii_driver_ops.scan(6s)   ← passive GAP discovery, classifies by name/mfg_id
  ├─ legacy_driver_ops.scan(6s)  ← passive GAP discovery, classifies by name/service UUID
  └─ returns ESP_OK
```

### 3.3 Scan Results (Merged + Deduped)
```
oximeter_get_scan_results()
  ├─ oxyii_results  = [{"addr","name","rssi","type":"oxyii"}, ...]
  ├─ legacy_results = [{"addr","name","rssi","type":"legacy"}, ...]
  ├─ merged = []
  ├─ add Legacy results first (Legacy wins on shared mfg_id 0xF34E)
  ├─ add OxyII results, skip duplicate MACs
  └─ return merged JSON array
```

### 3.4 Pairing Flow
```
POST /api/ox/pair {"addr":"...", "type":"legacy|oxyii"}
  → oximeter_pair(addr, driver)
     ├─ if driver != s_driver_type: s_active->forget(); switch s_active
     ├─ s_active->pair(addr)  ← starts pair_task (identify-only)
     │    ├─ connect_and_discover()
     │    ├─ legacy_get_info() / oxyii_session_open() + oxyii_get_info()
     │    ├─ looked-up scanned_name from current scan results
     │    ├─ save to NVS: serial, firmware/model, **scanned_name**, last_addr, protocol
     │    ├─ save to paired.json: serial, firmware/model, **scanned_name**, last_addr, driver
     │    ├─ update in-RAM state
     │    └─ set_state(OX_STATUS_PAIRED)
     └─ returns ESP_OK (async)
```

### 3.5 Background Watch (pull_task)
```
Every 15s:
  ├─ passive scan (4s)
  ├─ classify → pick hit for paired protocol
  ├─ address-hint tracking (exact MAC > same-family > MAC rotation adopt)
  ├─ served curfew: no reconnect while visible (30 min safety valve)
  ├─ sync window open:
  │    ├─ connect_and_discover()
  │    ├─ legacy_get_info() / oxyii_session_open() + oxyii_get_info()
  │    ├─ verify serial matches
  │    ├─ if worn → back off (NOT_READY)
  │    ├─ download missing files (FILE_OPEN/READ/CLOSE or F1/F2/F3/F4)
  │    ├─ exact-size promotion (Legacy) / trailer validation (OxyII)
  │    └─ disconnect
  ├─ on success: served=true, curfew until ring disappears (4 absent strikes)
  ├─ on fail: count failures; 3 consecutive → served=true (fail valve)
  └─ on absence (4 strikes): served=false, next appearance = new window
```

---

## 4. Protocol Comparison at a Glance

| Property | OxyII (Gen2) | Legacy (Gen1) |
|----------|--------------|---------------|
| **Devices** | O2 Ring S, SHQO2Pro, S8-AW | P02, O2Ring, Checkme-O2 |
| **Request lead** | `0xA5` | `0xAA` |
| **Response lead** | echoes command | `0x55` + STATUS byte |
| **Header size** | 7 bytes | 7 bytes |
| **Complement** | `~CMD` (bitwise NOT) | `CMD ^ 0xFF` |
| **Sequence/Block** | 1-byte SEQ + 1-byte FLAG | 2-byte BLOCK (LE) |
| **Length field** | 2-byte LE | 2-byte LE (total = LEN + 8) |
| **CRC-8** | poly 0x07, init 0, MSB-first | **identical** |
| **Auth** | AUTH (0xFF) + SETUP (0x10) | **none** |
| **Write transport** | WRITE_CMD (no-rsp) | WRITE_REQ (all chunks) |
| **Response matching** | opcode echo | strict lockstep (STATUS) |
| **File listing** | F1: count + 16-byte names | INFO JSON: `FileList` CSV |
| **File open size** | F2 reply: LE32 at payload[0..3] | FILE_OPEN reply: DATA[0..3] LE32 |
| **File read addressing** | absolute byte offset (F3) | block number in BLOCK field |
| **Close** | F4 (also clears F1 wedge) | FILE_CLOSE (best-effort) |
| **GATT Service** | `e8fb0001-a14b-98f9-831b-4e2941d01248` | `14839ac4-7d7e-415c-9a42-167340cf2339` |
| **Mfg ID (standby)** | `0xF34E` | `0xF34E` (shared!) |
| **Mfg ID (recording)** | `0x036F` (visible, never connect) | N/A (leaves standby) |
| **Name match (scan)** | prefix `S8-AW` / `SHQO2Pro` | fragments: `O2RING`, `CHECKME_O2`, `OXYLINK`… |
| **Service UUID (scan)** | N/A | `14839ac4-7d7e-415c-9a42-167340cf2339` in AD |

---

## 5. Legacy Wellue Protocol (0xAA/0x55) — Byte Level

### 5.1 GATT
| Role | UUID |
|------|------|
| Service | `14839ac4-7d7e-415c-9a42-167340cf2339` |
| Notify (ring→host) | `0734594a-a8e7-4b1a-a6b1-cd5243059a57` |
| Write (host→ring) | `8b00ace7-eb0b-49b0-bbe9-9aee0a26e1a3` |

### 5.2 Frames
**Request** (host → ring):
```
AA  CMD  CMD^FF  BLOCK_LO  BLOCK_HI  LEN_LO  LEN_HI  DATA...  CRC8
```
**Response** (ring → host):
```
55  STATUS  STATUS^FF  BLOCK_LO  BLOCK_HI  LEN_LO  LEN_HI  DATA...  CRC8
```
- Total frame = `LEN + 8`
- CRC8 covers all bytes except trailing CRC (poly 0x07, init 0, MSB-first)

### 5.3 Commands
| CMD | Name | Request Payload | Response DATA |
|-----|------|-----------------|---------------|
| `0x14` | INFO | empty | ASCII JSON (`SN`, `Model`, `CurBAT`, `FileList`) |
| `0x03` | FILE_OPEN | filename + NUL | `[0..3]` = file size (LE32) |
| `0x04` | FILE_READ | empty (block in BLOCK) | chunk |
| `0x05` | FILE_CLOSE | empty | status only |

### 5.4 INFO JSON Example
```json
{"CurBAT":"99%","FileList":"20260821101616,20260823162711","Model":"1652","SN":"22012C5328"}
```

### 5.5 Transport
- All writes: **WRITE_REQUEST** (20-byte chunks, sequential, awaited)
- Notifications fragment frames → reassembled in `s_acc[]`
- Strict lockstep: one outstanding request, response carries STATUS (not command echo)
- Block numbers must match exactly (out-of-order = desync)

### 5.6 Download Loop
```
FILE_OPEN(name + '\0') → size
for block = 0..:
    FILE_READ(block) → DATA chunk
    append to inbox/<name>.part
    stop when received == size
FILE_CLOSE (best effort)
promote_exact(serial, name, size)  ← validates exact byte count
```

---

## 6. OxyII Protocol (0xA5) — Byte Level

### 6.1 GATT
| Role | UUID |
|------|------|
| Service | `e8fb0001-a14b-98f9-831b-4e2941d01248` |
| Write | `e8fb0002-a14b-98f9-831b-4e2941d01248` |
| Notify | `e8fb0003-a14b-98f9-831b-4e2941d01248` |

### 6.2 Frame
```
A5  OP  ~OP  FLAG  SEQ  LEN_LO  LEN_HI  DATA...  CRC8
```
- Total = `LEN + 8`
- Same CRC8 as Legacy

### 6.3 Commands Used
| OP | Name | Notes |
|----|------|-------|
| `0xFF` | AUTH | 16-byte payload from `MD5("lepucloud")` derivation |
| `0x10` | SETUP | 1 byte `0x00` after AUTH |
| `0xC0` | SET_UTC_TIME | 8-byte local time |
| `0xE1` | GET_INFO | firmware @9..16, serial len @37 + string |
| `0x04` | LIVE_B | contact probe: `[5]=0x00` off-finger, `0x01` on-finger, `0x03` F1 wedge |
| `0x00` | GET_CONFIG | part of file-session prep |
| `0xF1` | GET_FILE_LIST | count + 16-byte fixed names |
| `0xF2` | READ_FILE_START | 20B: name (16) + type (4) → reply LE32 size |
| `0xF3` | READ_FILE_DATA | 4B: absolute offset LE32 → chunk |
| `0xF4` | READ_FILE_END | closes handle |

### 6.4 Transport
- Writes: **WRITE_CMD** (no-rsp, single packet)
- Notifications reassembled
- Response matched by opcode echo

### 6.5 Pull Loop
```
SET_UTC_TIME → GET_CONFIG → F4 (clear wedge) → F1 (list)
for each name not in index:
    F2(name) → size
    F3(offset)… until offset == size
    F4
promote(serial, name)  ← validates Format-A trailer magic at size-44
```

---

## 7. Storage Layer (Shared)

### 7.1 Directory Layout
```
/somnotrace/.somnotrace/oximetry/
├── inbox/
│   └── <name>.part          ← streaming download target
├── files/
│   └── <serial>/
│       ├── <name>.bin       ← OxyII Format-A recordings
│       └── <name>           ← Legacy native .vld (verbatim, no suffix)
├── paired.json              ← {"serial","firmware","name_prefix","last_addr","protocol"}
└── index.json               ← [{"serial","name","bytes","finalised"}, ...]
```

### 7.2 Promotion Rules
| Backend | Validator | Destination | On Failure |
|---------|-----------|-------------|------------|
| OxyII | Format-A trailer magic `48 12 5A DA` at `size-44` | `files/<serial>/<name>.bin` | keep `.part`, index `finalised:false` |
| Legacy | `part size == FILE_OPEN declared size` (exact) | `files/<serial>/<name>` verbatim | delete `.part`, nothing indexed |

### 7.3 NVS Mirror (namespace `oximeter`)
| Key | Value |
|-----|-------|
| `serial` | device serial |
| `firmware` | firmware/model string |
| `name_prefix` | **scanned advertisement name** (e.g., "O2Ring 5328") |
| `last_addr` | BLE MAC |
| `protocol` | `"oxyii"` or `"legacy"` |
| `driver` | `0` (OxyII) or `1` (Legacy) |
| `probe_mode` | `0` (legacy) / `1` (persistent) |

---

## 8. Advertisement Classification

Shared GAP event handler → each backend scores:

| Backend | Tier 3 (explicit name) | Tier 2 (mfg heuristic) | Tier 1 (service UUID) | Tier -1 (visible-only) |
|---------|------------------------|------------------------|----------------------|------------------------|
| **OxyII** | `S8-AW*`, `SHQO2Pro*` | mfg `0xF34E` | — | mfg `0x036F` (recording) |
| **Legacy** | `O2RING`, `CHECKME_O2`, `OXYLINK`… | — | `14839ac4...` in AD | — |

**Dispatcher merge**: Legacy results added **first** → Legacy wins on shared mfg_id `0xF34E` for Gen1 devices. OxyII only matches on explicit name prefixes for true Gen2.

---

## 9. Key Invariants

1. **Scan results survive driver init** — `init()` guards against double-allocation (`if (!s_scan) malloc`)
2. **Scanned name persists** — saved to NVS/paired.json at pairing, used for UI after reboot
3. **Legacy-first merge** — Gen1 devices never misrouted to OxyII despite shared mfg_id
4. **Served curfew** — no reconnect while ring visible post-sync (connection resets power-off timer)
5. **Failure valve** — 3 consecutive sync failures → window served (prevents hammering dead ring)
6. **Exact-size promotion** — Legacy never promotes partial/mismatched files
7. **Canonical conversion** — runs async after promotion, produces SNT v3 for upload
8. **State safeguard** — `oximeter_get_status()` returns "paired" if `paired=true` but driver reports "idle"

---

## 10. References

- `spec/0003-o2ring-ble-sync.md` — OxyII protocol study
- `docs/oximeter/LEGACY_WELLUE.md` — Legacy protocol reference
- `farolone/wellue-o2ring-protocol` — frame/command docs
- `MackeyStingray/o2r` — original RE research