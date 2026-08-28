# Radar lane occupancy

`radarlanesd` publishes the read-only `radarLaneStateSP` service from
`modelV2` lane geometry and `radarTracks` points. It reports occupancy and the
closest forward radar target in the left-adjacent, current, and right-adjacent
lane, plus a bounded global list of unique targets.
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

All accepted targets participate in occupancy and cut-in-candidate selection.
The message publishes up to 24 unique target details in `targets`; `laneMask`
identifies left/center/right membership and boundary targets can set two bits.
`uniqueTargetCount` remains the full count, while `targetsTruncated` explicitly
reports that the bounded list omitted lower-priority details. Predicted cut-in
candidates are prioritized before distance, so a farther overtaking target is
not hidden by a closer target in the same lane.

Cut-in motion is estimated from the change in path-relative `dPath` across two
fresh radar timestamps. Repeated model ticks do not reapply the same radar
sample. `yvRel` is exposed only as raw diagnostic data because its ARS408 sign
has not yet been validated for this installation; it is not used to decide
cut-in direction. `cutInCandidate` means a measured lateral trend would reach
the center-lane boundary within the published three-second horizon. It is a
read-only candidate, not a collision decision or control command.

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

The standard C3XL on-road renderer subscribes to this service and displays at
most one unique representative target per lane. A predicted cut-in target has
priority over the closest target. Each marker shows distance and estimated
longitudinal target speed (`vEgo + vRel`); this is a radar-relative estimate,
not a wheel-speed measurement from the other vehicle. Red marks a cut-in
candidate, orange a closing target, and green a non-closing target.
