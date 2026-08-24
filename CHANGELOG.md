# Changelog

All notable changes to SomnoTrace are documented here.

## [Unreleased]

Local work carried on top of upstream `v1.0.8-alpha-experimental`
(2c8d801). Theme: **dual-protocol oximeter support (Wellue legacy +
OxyII) and a pluggable oximeter backend — firmware side only.** Web
UI / status-page integration is deliberately deferred; the stock
upstream portal is used unchanged.

### Added

- **Legacy Wellue O2Ring support** — second oximeter protocol family
  alongside the OxyII/SleepHQ Pro rings:
  - Full BLE protocol implementation (INFO JSON, FILE_OPEN/READ/CLOSE
    with per-block retry limits and strict response lockstep).
  - Recordings archived byte-for-byte under
    `/somnotrace/.somnotrace/oximetry/files/<serial>/`; per-serial dedupe
    ledger prevents re-pulling finalised recordings.
  - Protocol auto-detected from BLE adverts; scan results include both
    families; GET_INFO serial remains the identity gate.
- **Pluggable oximeter backend architecture** — shared orchestration
  (passive watch, sync windows, curfew, persistence) moved to
  `oximeter_common.c` behind an `oximeter_backend_t` registry
  (`oximeter_backend.h`); each protocol family lives in its own backend
  file. New families are added by registering one struct.
- **Presence model in the sync engine** (`oximeter_get_presence()`):
  `syncing`, `ready`, `detected`, `recording`, `offline`.
  - Recording-mode adverts (OxyII mfg `0x036F`) are detected as
    *visible-only*: never connected, never offered for pairing, never used
    to start a file session — but surfaced so a worn ring is not mistaken
    for a vanished one.
  - `/api/status` exposure of presence is deferred.
- **Address-hint identity matching** — presence and sync prefer the exact
  paired address; same-family adverts from unknown addresses can no longer
  fake presence or stall absence detection, with automatic adoption after
  sustained signal loss (MAC rotation / factory-reset recovery).
- **`last_seen` telemetry** added to the oximeter paired-device object of
  `/api/status` (epoch of the last background-watch match; flows through
  the paired-device info upstream already embeds). Not rendered in the
  UI yet.
- **Documentation**: `docs/oximeter/OXIMETERS.md` (sync engine, storage,
  presence, API surface, planned uses of ring data),
  `docs/oximeter/LEGACY_WELLUE.md` (legacy protocol reference).

### Fixed

- **Stuck "ring present"**: nearby foreign devices sharing the Viatom
  manufacturer id could reset the absence counter indefinitely, keeping
  the ring shown as available long after it disappeared.
- **Sticky oximeter error state** after a transient scan-start collision
  with CPAP reconnects (`BLE_HS_EBUSY`); the error no longer poisons the
  module state and the serial log gains unrecognised-advert summaries for
  field diagnosis.

### Known limitations

- The web UI is the stock upstream portal: the oximeter card shows only
  the basic state string (`scanning` / `connecting` / `pulling` /
  serial), with no family labels, no Last Seen / Last Transfer rows, no
  scan stop button, and no resilience against half-open WebSocket
  connections (the settings page can freeze on a stale label until
  reloaded). `last_seen` is available via the API; presence reporting is
  computed internally, and scan-cancel exists as firmware plumbing
  (`oximeter_scan_cancel()`), but neither is exposed via HTTP yet.
- Radio-busy scan starts surface in the stock settings page as a generic
  "Scan failed" message; retrying works (the module state recovers).
- O2S/OxyII rings do not yet report battery/model/recording-count via
  the API (legacy rings do).
- Legacy rings have no remote clock-set command (unlike OxyII's
  `SET_UTC_TIME`): recording timestamps follow the ring's own RTC, so
  the clock must be kept correct via the Wellue companion app for clean
  alignment with CPAP sessions.
- Legacy transfers are protocol-limited to ~20-byte notification chunks
  (MTU stays 23), so large recordings pull proportionally slowly.
- Legacy presence is only detectable during the ring's radio-visible
  standby windows (worn/screen-on rings are silent), so appearance-based
  sync can wait for a take-off/dock event.
- Large day-graph downloads still occupy the single HTTP worker for
  several seconds on slow Wi-Fi links; the durable fix is async-handler
  offloading (see `spec/archive/http-server-concurrency-and-socket-exhaustion.md`).
