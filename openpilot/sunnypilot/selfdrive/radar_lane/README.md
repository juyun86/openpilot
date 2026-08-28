# Radar lane occupancy

`radarlanesd` publishes the read-only `radarLaneStateSP` service from
`modelV2` lane geometry and `radarTracks` points. It reports the closest
forward radar target in the left-adjacent, current, and right-adjacent lane.
Navigation route data is intentionally not an input: it describes route intent,
not the live lane boundaries around the car.

This service intentionally does not modify or replace `radarState` and is not
consumed by longitudinal planning, FCW, lane-change control, or vehicle CAN.

## Semantics

- `occupied`: at least one radar target is inside the evaluated forward lane
  corridor.
- `clear`: trustworthy model lane boundaries cover the reported distance and
  the current accepted front-radar tracks contain no target in that corridor.
- `unknown`: the geometry or radar data cannot prove the corridor is clear.

When an outer lane line is unavailable, the classifier can still report
positive evidence inside a nominal 3.6 m path-relative corridor. Absence in an
estimated corridor remains `unknown`, never `clear`.

The standard `radarTracks` contract does not include ARS408 object class or
existence probability. Therefore `occupied` means "radar target present", not
"camera-confirmed vehicle present". A front radar also cannot establish rear
blind-spot clearance; consumers must not use this message as permission to
change lanes.

The three lanes are relative to the current `modelV2` frame, not stable map
lane identifiers. Consumers must require a live service and valid event, treat
`unknown` as unconfirmed, and must not carry a lane identity across a lane
change. `clear` does not prove complete sensor coverage and is not a substitute
for side/rear blind-spot monitoring.
