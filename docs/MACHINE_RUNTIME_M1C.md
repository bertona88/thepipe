# Machine runtime M1c — simple manipulation contract

Status: implemented simulation baseline; not hardware-qualified

M1c is the first runtime in this repository where a commanded arm changes the pose and ownership
of an assembly body. It is deliberately a calibration-coupon gate between M1b point motion and M2
two-arm handoff. The legacy gearbox executive remains separate and retains its F1-reduced label.

## Acceptance sequence

Manipulator 1 executes this fixed action order:

1. open the parallel jaws;
2. approach the pickup datum from a 2 mm world-X offset;
3. move the tool centre to the calibration peg;
4. close to 12 µm modeled pad compression on the 0.40 mm peg;
5. acquire the peg only after the geometric bilateral-contact test passes;
6. retract to the pickup approach datum;
7. translate to a datum 2 mm along the socket's local -Z axis while retaining plant ownership;
8. insert along the nominal socket axis;
9. open until geometric jaw contact is lost and the plant releases the peg; and
10. retreat to the socket approach datum while the released peg remains in the socket.

Every Cartesian leg uses the M1b synchronized, limit-derived trajectory and sampled preflight.
Preflight now removes the attached body from the static obstacle set, reconstructs its pose at
every path sample, and tests it against enabled obstacles and upstream links. A rejected carried
part path does not increment the command sequence or change actuator targets.

## Geometry and units

All persistent values use SI units. The compiled coupon baseline is:

| Item | Value |
| --- | ---: |
| Peg diameter | 0.400 mm |
| Peg cylindrical half-segment | 0.350 mm |
| Commanded pad compression | 0.012 mm |
| Pickup tool centre | `[20, 0, -6]` mm |
| Socket tool centre | `[20, 0, 6]` mm |
| Pickup world-X approach offset | 2.000 mm |
| Socket axial approach offset | 2.000 mm along local -Z |
| Socket radial clearance | 0.125 mm |
| Socket half-depth | 0.800 mm |
| Fixed simulation step | 0.001 s |

The peg is a dynamic capsule in a zero-gravity coupon plant. Four oriented static boxes form the
socket. Coupon collision groups intentionally test peg/socket proximity while excluding the
current unmodeled jaw and terminal-tool solids. Other arms remain present at the same separated
parking datums used by M1b.

## Grasp and release semantics

A grasp is rejected when any of these conditions is true:

- the body is static, disabled, already held, outside the finger depth, or outside jaw travel;
- the jaw faces still have positive clearance to the body;
- modeled compression exceeds twice the configured pad compliance; or
- the body centre is displaced along the closing axis by more than the configured pad compliance.

While attached, the plant recomputes reduced grip force from pad compression each fixed step. It
automatically clears ownership when opening creates positive geometric clearance. The held body
pose is always `world_from_tool * tool_from_body`; neither the task adapter nor renderer writes it.

## Report and fail-closed behavior

The native and WASM adapters emit report schema version 1 with phase-boundary replay records,
command sequence, tool and peg positions, grasp ownership, reduced grip force, final socket errors,
planned socket penetration, and unplanned penetration. Expected failures produce an `aborted`
report with a reason. They never produce a partial `complete` result.

Acceptance requires:

- deterministic equality of two fresh cycle reports;
- observed grasp ownership throughout retract, transfer, and insertion;
- observed release before retreat;
- no unplanned contact;
- zero planned socket penetration in the nominal coupon;
- final lateral, axial, and axis errors below 1 nm in this ideal kinematic baseline; and
- final peg/socket clearance greater than the 0.100 mm sampled-preflight threshold.

The nanometre numerical gate checks deterministic ideal kinematics; it is not a hardware accuracy
claim.

## Fidelity boundary and next gate

M1c is **F0 geometry with a reduced compliant grasp**, not normative F1. It does not yet model jaw
or terminal-tool collision solids, gravity-sensitive fixturing, breakable grasp dynamics,
contact-derived insertion force, constrained-orientation IK, estimator input, calibration drift,
or hardware timing. `SceneFrame::truth` is available for evaluation and `SceneFrame::estimate`
remains `null`; the controller uses configured coupon datums rather than hidden truth.

M2 may begin after this cycle remains green in native tests, strict clippy, the WASM release build,
and the headless CI acceptance run. M2 must add explicit dual ownership/handoff semantics and
space-time collision reservation before the optical estimator is allowed to drive control.
