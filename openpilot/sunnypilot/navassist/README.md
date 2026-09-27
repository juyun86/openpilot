# NavAssist closed-course P0

This module is an isolated, closed-course-only ingress path for high-level
mobile navigation observations. It does not accept actuator, curvature,
acceleration, CarState, or CAN commands.

## NOA data-use principle

NOA follows **maximize useful data, fuse by authority, and fail closed**. Every
semantically verified, fresh, valid, and plausible signal already available
from C3 perception/localization/radar/control state or from the OEM vehicle and
head-unit buses should be reused and cross-checked instead of duplicated or
discarded. AMap supplies route intent and map priors; C3 perception supplies
continuous physical geometry and objects; OEM messages supply actual vehicle,
driver, blind-spot, radar, lane-position, and vehicle-permission state.

Safety and actuation decisions remain subordinate to fresh C3 control state and
verified OEM feedback. No navigation, vision, or CAN source by itself may
bypass solid-line, blind-spot, driver-override, controller-state, or vehicle
permission gates. New bus signals require replay or vehicle validation plus
freshness, validity, range, continuity, conflict, and diagnostic handling.
Unknown, stale, implausible, or conflicting inputs fail closed; absence is never
interpreted as permission.

## Arming

The manager keeps the network-only `navassistd` receiver available on a Tesla
C3XL so an installed TesNav App can reconnect without a manually copied token.
`lane_topologyd` still runs only onroad. Network availability grants no control
authority. Active use separately requires fresh matched realtime phone
guidance and SP control authority. No per-drive Track Mode, token, or geofence
configuration step exists. Phone observation quality, unchanged-guidance age,
and C3XL `liveLocationKalman` remain visible diagnostics, but they do not veto
an unexpired matched phone route.

On first use, C3XL trusts a bounded set of up to four self-signed App identities,
and it adds each new identity only while offroad. Those P-256 public keys are
persisted as `NavAssistPairedApp`; unknown App keys cannot replace existing
entries while onroad. The C3XL device private key
is stored in the `PERSISTENT | DONT_LOG` `NavAssistDevicePrivateKey` Param and never transmitted. Clearing or
changing an App identity requires an explicit offroad maintenance action; it is
not a network command. First-use TOFU removes shared-secret configuration, but
it cannot identify the owner before the first pin: pair only on a controlled
private LAN.

## Transport

The closed-course phone default is a zero-configuration, unauthenticated UDP
4213 broadcast carrying the canonical v3 snapshot itself. `navassistd` accepts
only 1..8192 byte datagrams, limits each source to 20 packets/second, applies
the same strict schema, source-wall freshness, session/sequence/revision and
short TTL rules, then acknowledges the accepted session and sequence. No
command field or generic command dispatcher exists. Any host on the same LAN
can inject navigation observations, so this mode is suitable only for the
authorized closed-course setup described here.

The signed discovery/HTTP path below remains available as a compatibility and
explicit-URL override; it is not the Android/iOS default when the URL is blank.

The phone discovers C3XL without scanning every HTTP address. It sends a signed
IPv4 UDP broadcast to port 7765, then constructs the HTTP URL
from the source IP of a verified unicast offer. The offer never supplies a host
name or IP address. Both datagrams are compact JSON, at most 512 bytes, with
exact field sets:

```json
{"messageType":"navassist_discovery_request","schemaVersion":3,"nonce":"<32-lower-hex>","appKeyId":"<32-lower-hex>","appPublicKey":"<P-256-X.509-SPKI-base64url>","signature":"<ECDSA-DER-base64url>"}
{"messageType":"navassist_discovery_offer","schemaVersion":3,"nonce":"<same-nonce>","appKeyId":"<same-app-key-id>","deviceId":"<32-lower-hex>","devicePublicKey":"<P-256-X.509-SPKI-base64url>","port":7766,"path":"/v3/snapshot","signature":"<ECDSA-DER-base64url>"}
```

Both keys use P-256. `appKeyId` and `deviceId` are the lower-case hexadecimal
first 16 bytes of SHA-256 over the canonical 91-byte X.509 SubjectPublicKeyInfo
DER. Public keys and standard DER ECDSA signatures use unpadded base64url. The
request is signed using ECDSA with SHA-256 over the exact UTF-8 bytes below
(newline separators, no trailing newline):

```text
navassist_discovery_request
3
<nonce>
<appKeyId>
<appPublicKey>
```

The offer signature covers:

```text
navassist_discovery_offer
3
<nonce>
<appKeyId>
<deviceId>
<devicePublicKey>
7766
/v3/snapshot
```

With the longest valid P-256 DER signature, the compact request is at most 403
bytes and the offer is at most 484 bytes. Parsers reject noncanonical keys,
invalid curves, duplicate or unknown fields, incorrect primitive types, and
datagrams over 512 bytes. C3XL replies only to RFC1918 IPv4 sources and silently
drops malformed, unauthenticated, unpaired, or rate-limited requests. A bounded
two-second `(source, appKeyId, nonce)` cache suppresses duplicate bursts. The
outstanding random nonce on the phone prevents an old offer from matching a new
round, while HTTP retains separate TTL and persistent sequence replay controls.
A verified offer identifies a candidate; the phone reports C3XL as online only
after an HTTP exchange also succeeds.

POST compact v3 navigation JSON to:

```text
http://<c3xl-private-address>:7766/v3/snapshot
```

Set `X-NavAssist-Key-Id` to the pinned App key ID. Set
`X-NavAssist-Signature` to the App's unpadded base64url DER ECDSA/SHA-256
signature over these prefix bytes followed immediately by the exact HTTP body:

```text
navassist_snapshot
3
POST
/v3/snapshot
<deviceId>
<appKeyId>
<body-byte-length>
<raw-body-bytes>
```

Binding both identities, method, path, byte length, and raw body prevents a
valid snapshot signature from being redirected to another C3XL or request.
Signatures authenticate and protect integrity but do not encrypt location data;
use only a controlled private network.

The receiver bounds request size, rate, and concurrent connections, rejects malformed/non-finite values,
requires a strictly increasing sequence and non-decreasing route revision, and
assigns the control-relevant TTL on C3XL receipt only after also bounding the
signed source-wall timestamp. A receiver may join a
new authenticated session above sequence 1 after earlier HTTP attempts failed;
once replaced, that retired session cannot become active again. A source-wall
freshness bound rejects old captures, and an atomic high-water checkpoint in
`/dev/shm` preserves active/retired session state if only `navassistd` crashes
and restarts. A manager restart leaves the receiver available for the
already-paired Apps; transport availability never grants control authority. If
one paired App owns a fresh active navigation session, snapshots from another
paired App cannot preempt it until the active session expires.

Transport freshness cannot make a stopped App or an expired route active.
Active use still requires an accepted unexpired snapshot, realtime mode,
route matching, received guidance, a nonzero maneuver event, and phone-provided
route progress. Guidance and location callback ages remain diagnostics because
AMap may leave both timestamps unchanged while an instruction is unchanged;
the receiver-assigned snapshot TTL is the liveness boundary. Phone accuracy and
C3XL localization are also diagnostics rather than independent planner vetoes.
The phone SDK's `gpsWeak` flag is diagnostic-only.
Only Android/iOS `realtime` navigation can become control-valid; simulation and
generic track sources remain diagnostic-only.

## Active scope

`NavigationSpeedController` can only lower the common cruise speed ceiling for
an admitted maneuver below 60 km/h. It never requests a stop or acceleration,
and it leaves `controlsd`, lateral curvature, CarState, CarController, and Panda
unchanged. On Tesla it additionally requires fresh `carStateSP` proof that SP,
not stock longitudinal control, owns the vehicle. Official, Experimental and
TN-NoDEC consume the same navigation ceiling at the existing common speed-target
seam. Their solvers, acceleration limits, following and stopping policies remain
unchanged; the navigation module does not write acceleration commands. The
1.2 m/s² comfort value sizes the approach window, not a new actuator limit.
Lead, FCW, or other existing safety sources may independently request stronger
deceleration. An experimental typed `navLaneIntentSP` path can also request one
Tesla physical turn signal. A linked left/right route maneuver may request a
bounded pre-turn lamp with `targetLaneIndex = -1`; DesireHelper suppresses both
its physical feedback and cancellation tail at the ALC entrance. Navigation
publishes a target and diagnostic readiness only; SP's configured
AutoLaneChangeController remains the sole start authority and requires fresh
OEM `0x239` lane-position/neighbor state, fresh direction-specific `0x399`
permission, clear `0x399` BSM, active SP lateral, Panda TX echo, and physical
lamp feedback. A fresh visual solid line or road edge remains an additional
veto, but vision does not invent a neighbor or grant permission. `OFF`/`NUDGE`
remain authoritative. A same-direction
pre-turn lamp transfers to the lane-change request without blinking off. This path never
accepts phone curvature or bypasses Panda safety, and every lane is re-observed
before another request. AMap complete-road indices are never equated with local
OEM lane-position categories: only a direction-consistent recommendation touching the road
edge can qualify relative edge alignment; an unanchored middle recommendation
is display-only. While LaneInfo is absent, ordinary/sharp/U-turns can target the OEM-reported leftmost/rightmost lane
inside 1 km, and directional exit/ramp/merge events can do so inside 2 km;
`slightLeft`/`slightRight` never force an extreme-lane fallback.
Relative alignment adopts CP's temporal consistency without its global
fail-open behavior: three stable, counter-contiguous `0x239` samples with a
plausible virtual-lane width, at least 20 m view range, and direction-specific
`AVAILABLE`/`FUSED` line evidence; two permissive `0x399`
samples, 0.5-second neighbor stability, 5-second edge confirmation,
3-second new-lane stability after an edge, 2-second post-change cooldown, pause
during lateral transition/driver steering, and a configured change budget.
As of the 2026-09-22 highway/elevated-road closure fix, relative alignment ends
after one SP cycle with `laneChangeCompletionUnconfirmed`: local/OEM indices
cannot independently prove a crossing. It does not count success or request
another step for that manoeuvre. Readiness is withdrawn while waiting for the
cycle-end confirmation, including observation gaps. Cancelled and unconfirmed
events stay latched across inactive/event-zero input and revision-only changes;
only a valid nonzero new manoeuvre or a valid new navigation session releases
the latch. This latch is process-local, not persisted across daemon restarts.
Continuous relative alignment still requires independent completion evidence;
the configured count alone does not enable it. Existing absolute-index plans
retain their stable-index completion check.
The `0x239` center polynomial and fork state remain C3-local diagnostics for
now; they do not create steering requests or bypass any gate. The App-facing
UDP acknowledgement retains its existing schema.
Navigation samples record raw `can239` alongside the existing `can399` list,
including event timestamps, validity, source bus and DLC. Eight-byte 239 rows
also contain values from the same decoder used by OEM feedback. Decoded values
are observations, not permission: line usage is not solid/dashed paint, and
view range is not a boundary endpoint. Offroad 239 frames are buffered until
the existing sample/write trigger, with a 32-frame cap and an explicit
`can239_dropped` count on overflow. Disabling recording clears pending frames.
Onroad samples also store bounded 33-point model trajectories, lane lines and
road edges, with frame IDs, camera EOF timestamps and prediction time axes.
MADS state, measured steering/torque/yaw rate, requested actuators and
`carOutput.actuatorsOutput` are recorded separately at the existing 20 Hz loop.
`driverIntervention.madsSteeringPressed` means the MADS feature is available
and the vehicle reports steering pressed; it is not a label of pure human
control. Use the recorded MADS state and lateral-active state to distinguish
active overrides from ordinary manual driving. Missing/stale input is null.
Lane alignment requests the physical lamp at the navigation-owned timing, then
waits in the signaling phase while `0x399`, BSM, or a visual boundary veto is
blocked. It marks the request ready only after all C3-local gates and physical
lamp feedback are stable, then holds that lamp through the existing SP cycle. A
navigation lane-change lamp is excluded from LaneTurnDesire; only a true
turn-only request or a driver lamp may create turnLeft/turnRight desire.
When a new same-direction turn request takes ownership of a still-lit lamp,
the previous lane-change tail no longer masks that turn from the model. The
model's original rising-edge desire input is preserved. Entering the existing
ordinary-turn approach window stops new edge-alignment attempts; a started SP
lane-change cycle completes before handing ownership to the turn request.
The navigation action identity uses route/session/maneuver identity, independent
of changes in visible lane count. A short topology observation gap does not
pretend that actual lateral control became inactive. The existing Tesla signal
controller honors navigation's hold policy for starting-to-pre transitions as
well as finishing/off, so the coordinator owns normal completion cancellation.
No navigation request, including an imminent exit or ramp, may bypass stale or
negative `0x239`/`0x399` state. Confirmed visual solid paint or a road edge,
BSM, pedals, lateral-authority loss, and missing physical-lamp feedback also
block the manoeuvre.
Lane positioning itself never creates a speed target. A supported turn/exit
maneuver may still activate its comfort-distance speed ceiling while a final
lane change is in progress. Once the admitted ceiling activates, it remains as
distance is consumed, including zero; initial braking admission is not repeated
while the planner ramps deceleration. Source/authority loss and driver overrides
still cancel the event. SCC-V and navigation compete through the existing minimum
speed-target selection every cycle, so a lower SCC-V target wins and a transient
SCC-V release cannot discard the navigation ceiling for the rest of the turn.
The deceleration is attributed to the approaching maneuver, not to the lateral request.
Within one continuous route revision, maneuver-event advancement retains the
lamp until model-derived turn geometry has stayed clear for 0.5 seconds.

Follow the stationary gates and speed progression in
[`docs/NAVASSIST_CLOSED_COURSE_P0.md`](../../../docs/NAVASSIST_CLOSED_COURSE_P0.md)
before any active test.

Android does not infer a directional `exit_left/right` or `ramp_left/right`
event from road text alone. Android and iOS can both carry explicit directional
maneuvers; those events may request one-lane-at-a-time positioning through the
same OEM permission, neighbor, visual-veto, and BSM gates.
