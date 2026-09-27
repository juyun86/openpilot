# sp-dev-egpu to sunnypilot dev migration plan

## 2026-09-19 applicability review

This document retains the historical migration baselines, scope, and test-build
policy. It is not a fresh inventory of the running device or an instruction to
repeat completed migration work. The current City NOA scope is defined by the
[master plan](../../C3_CITY_NOA_MASTER_PLAN.md) and the
[consolidated lane-change plan](../../C3_LANE_CHANGE_CONSOLIDATED_PLAN.md).

Minimum intrusion is the highest engineering constraint: reuse existing
implementations and change only the locations needed for a confirmed issue.
This NOA/design review does not reopen hardware/boot migration, replace planner
backends, duplicate radar processing, add services, or enable the historical
Shadow mode listed below. Do not port whole files or move working features merely
to match the old slice boundaries. Recheck the relevant current code before
treating a historical pending item as unfinished work; retain unrelated changes.
This review edits documentation only and authorizes no device writes.

## Verified baselines

| Role | Commit | Use |
| --- | --- | --- |
| sunnypilot source | `f403b3a5e334cd7cf49dedd876a9d6ed8419bd37` | Source Baseline named by the current `dev` release |
| sunnypilot release | `6384047adee5d82dc8b87db9c33134146fd4260e` | Prebuilt Snapshot; not a merge/rebase base |
| local final tree | `6108f38` | Evidence of features that still exist |
| local history bridge | `629eda5` | Intent and commit provenance only |
| mr-one C3XL reference | `16c99e6b0bdc37f31b1c9c4c1f5ffad52545a452` | Behavioral reference for C3XL/Panda/AGNOS |

The fast startup premise needed one correction: sunnypilot `dev` is fast
because it contains the `prebuilt` marker and compiled artifacts. It is not a
non-precompiled development branch. Development therefore starts from the
source commit named by that snapshot. The maintained source/device channel is
`dev-sp-egpu`; only that exact published channel is classified as TICI-
compatible, so unrelated development branches remain blocked without a global
hardware-check bypass.

## Boot and Panda policy

The C3XL and current upstream manifests agree on `xbl`, `xbl_config`, `aop`,
`devcfg`, and `system`. They differ on `abl` and `boot`. The C3XL profile uses
the mr-one ABL and boot images and validates exact hash, raw hash, size, full
check, and A/B attributes before flashing.

This is an allowlist, not a blanket flash ban: every matching image, including
matching boot-chain images, may update automatically. An unknown changed
boot-chain image fails before any partition write. Panda reset/recover and raw
type interpretation are profile-scoped; raw and effective types remain
observable. The old unconditional `Panda.get_type() == 9` override is excluded.

## Feature inventory and migration slices

| Priority | Module | Content to port | Upstream intrusion after refactor | Status |
| --- | --- | --- | --- | --- |
| P0 | C3XL Profile | AGNOS allowlist/manifest, hardware identity Adapter, Panda Startup, read-only probe | build profile define; AGNOS validator call; Panda startup call | Host-tested |
| P0 | Tesla Control | DBC/HW4 decoding, MADS/coop steering, manual longitudinal selection, Dynamic Auto Stock, AP Hybrid, auto speed, safety validation | Tesla Control Profile at car init; Tesla Control Runtime at selfdrived | Core/opendbc ported; device test pending |
| P0 | Radar Backend | OEM/ARS408/Off selector, ARS RX parser, tracker, diagnostics, bounded motion TX, Panda safety | one backend selector in opendbc; one enum Param/UI control | Host-tested; device test pending |
| P1 | Planner Backend | Upstream Official plus restored legacy Experimental and TN-NoDEC; session latch; independent live profiles/tuning; TN stopping policy | one planner factory; isolated backend registry; one shared legacy MPC equation/build module | Darwin and C3XL route differentials, host/device convergence, and device timing pass; on-road behavior pending |
| P1 | Traffic Radar / Stop Profile | Off/Observe/Shadow/StopOnly/StopGo, current-lane Tesla CAN event controller, independent typed Traffic target, jerk-limited stop ownership, bounded eight-metre-gated departure, causal HUD diagnostics | one `trafficcontrold` publisher; one common post-planner arbitrator | Host-tested with route-derived and deterministic regression fixtures; on-road behavior pending |
| P2 | Device Query/Command | default-on fully unauthenticated settings/commands, Tesla/HW4 diagnostics and validation, hotspot, opt-in offroad terminal; no driving-information page/API | one managed service; query/command boundary | Device HTTP verified; physical command tests pending |
| P2 | Update reliability | proxy Adapter, current-tree LFS hydrate, last-known-good clock | narrow updater/time hooks | LFS and clock host-tested; proxy pending |
| P3 | Local Defaults/UX | complete Simplified Chinese catalog, legacy onroad-alert localization and glyph coverage; legacy C3XL GPIO42 buzzer; brightness controls; functional one-minute shutdown choice; speed offset cap; eGPU telemetry/safe eject | separate defaults policy, alert adapter, and isolated UI rows | Translation/font/alert/power host checks pass; device rendering, audible, and display checks pending |

### Tesla safety unit

The following move together and are not independently versioned:

1. Tesla DBCs and HW4 shadow decoding.
2. `CarState` ownership state machine and `CarController` output.
3. Panda Tesla safety flags, TX allowlists, and validation tests.
4. Tesla Control Profile parameters and the opendbc submodule commit.
5. Tesla Control Runtime event policy and same-cycle `carStateSP` publication.

Turning all local Tesla switches off must result in upstream behavior and no
new CAN transmission. ARS408 additionally requires its safety flag before any
motion frame is accepted.

### Planner migration rule

Do not keep a forked Official planner. The Official Adapter constructs the
Source Baseline planner directly. The registry exposes three session-latched
selections: Official (`0`), Experimental (`1`), and TN-NoDEC (`2`). The user
confirmed that Experimental and TN-NoDEC must reproduce the final old driving
logic, not approximate it through the new upstream lead-only MPC. They therefore
restore the old planner flow and share one reproducibly generated eight-
parameter cruise-obstacle equation source. It builds the legacy primary solver
and an active-only numerical recovery variant. Platform binaries and the two
untracked old generated trees remain excluded.

Route regression uses old-tree numerical baselines recorded separately for
Darwin/arm64 and Linux/aarch64; acados output from one CPU is not treated as the
oracle for another. On the C3XL, old `long_official` generated C and the migrated
primary matched all 236 cycles exactly, including every source/state field. The
112-case C3XL grid (both backends, Default/CrazyMax) had zero final failures and
a worst solve time of 0.731 ms. Host grids also exercise the recovery path.
Recovered primary failures are explicitly rate-limited and logged.

Each backend has an independent Default/CrazyMax/Custom profile. Validated live
values are revisioned in one configuration Param, polled at a bounded rate, and
ramped before use. The configuration format has an explicit schema identity and
migrates both the old semantic layout and the interim per-backend layout without
overwriting unknown input, including TN's native acceleration enable/profile.
Official remains the upstream implementation; its tuning is applied only
through narrow MPC hooks and Default takes an exact no-op path. Experimental
retains the legacy DEC/Experimental Mode behavior, while TN-NoDEC ignores DEC and follows
Experimental Mode directly, matching the final old trees.

`trafficcontrold` is the only Traffic state-machine owner and publishes
`trafficRadarState`. Tesla bus-2 `0x25D` supplies the authoritative current-lane
color and distance inside 200 metres; no CP/model stop point, fake `lead2`, MPC
target setter, turn-intent veto, or navigation signal is used. The selected
Official, Experimental, or TN-NoDEC backend first publishes its normal base
plan, then one common `FinalPlanArbitrator` applies a more conservative STOP.
Observe and disabled modes atomically clear Traffic ownership and are output-
transparent.

Confirmed same-session green removes a moving STOP immediately. A stationary
bounded START is deduplicated per session, capped at 1.6 m/s², 2.5 m/s, and
three seconds, and is fail-closed when visual lead health is unknown. Any
current lead within eight metres blocks the request; a selected near lead must
persist for 0.5 seconds before the session is delegated to the base lead
planner, while transient unselected targets clear after 0.4 seconds. The
request never mutates vehicle state, CAN, or backend persistent state.

### Local console boundary

The current test build deliberately starts the console by default and performs
no authentication: reads, settings writes, validation commands, and the
terminal endpoint do not require a password or token. Requests are still
accepted only from loopback/private/link-local addresses. The page exposes
settings, turn-signal validation, and the opt-in terminal; driving information
and `/api/driving-status` are intentionally absent. The arbitrary terminal
requires its own opt-in Param, runs only offroad, is killed on an onroad
transition, has a 20-second/64-KiB bound, and passes commands as a Bash argument
without Python `shell=True`. This is a user-confirmed temporary test policy and
must be reconsidered before broader deployment.

## Explicitly excluded

- New eGPU/UT3G model routing or model assets. The Source Baseline's existing
  USB-GPU path is unchanged. Chestnut FPS/VRAM observability, the left-side
  status panel, and recoverable offroad safe eject are retained locally.
- The old offline-Panda-wake series: its intermediate commits are not present
  as a functional net change in final `6108f38`.
- Unconditional Panda type spoofing.
- Default browser password `123456`, token authentication, and Python
  `shell=True`. The explicitly requested console is fully unauthenticated in
  this test build, subject only to the network and vehicle-state boundaries
  described above.
- Disabled audio-feedback code, disabled loggerd/DM/micd changes, hard-coded
  IMEI/private hosts, and Params with no consumer.
- Duplicate GPS time-sync process; only last-known-good clock persistence may
  be added to upstream `timed`.
- Duplicate Experimental/TN equation/build trees or copied platform binaries.
  The two custom backends share one source-generated legacy MPC family.
- A fake Traffic vehicle in `radarState`, model input, `leadOne`, or `leadTwo`.
- Whole-file copies of old `updated.py`, `selfdrived.py`, UI settings pages, or
  planner files.

## Acceptance gates

1. Host lint, schema generation, unit tests, and native C++ profile tests.
2. Tesla/opendbc Python and Panda safety suites with each capability disabled
   and enabled.
3. C3XL read-only probe records raw Panda type, effective type, boot slot,
   AGNOS version, and bootstub/application state before any flash.
4. Offroad dry-run validates all AGNOS allowlist entries without writes.
5. Bench CAN replay verifies no extra TX when a Module is disabled, followed by
   bounded ARS408 and Tesla handoff tests.
6. Only after the above, build a prebuilt artifact from the verified
   `dev-sp-egpu` source commit; retain the previous known-good device commit for
   rollback.
