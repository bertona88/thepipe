# Engineering replay and the stationary M1g handoff gate

This change provides an offline inspector of executed M1e/M1f peg manipulation
and a separate stationary two-arm ownership-exchange coupon. The latter is a
protocol/mechanics gate, **not completion of fixed-head observed two-arm motion**.
No site, hosted frontend, synthetic preview mode, or generated animation is used.

## Inspect the actual M1f task

From a clean checkout with stable Rust, Python 3 and a browser:

```sh
python3 scripts/record_observed_replay.py
```

Open `out/observed-inspection/inspector.html`. The script builds the current clean
Git revision, records the fixed-head M1f scenario, validates the result, and
embeds it in a self-contained offline HTML inspector. No web server, package
installation, network resource or JavaScript physics loop is needed. An explicit
`--scenario PATH` selects M1e or another admitted M1f scenario; `--fault NAME`
records the existing injected faults. Failed tasks retain their terminal reasons.

The low-level `pipe-observed-replay` binary accepts `--source-revision FULL_SHA`
and writes JSON to stdout. Its revision is caller-supplied provenance; use the
build-and-record script or CI to tie that value to the built source. It samples
every 100 fixed ticks and at decisions. At a tick shared by several decisions,
the recorded geometry is the state after the last decision at that tick; all
decisions remain individually ordered in the report. The final sample is always
included. Playback advances exact samples at 10 samples/s, **not real-time**.
There is no interpolation, extrapolation, or use of playback time for control.

| Display | Authoritative input and interpretation |
| --- | --- |
| Grey geometry | Actual mechanics body shapes/poses and FK collision capsules; reduced envelopes, not manufacturing CAD |
| Blue peg and jaws | Peg body and distal jaw geometry from the plant, including the modeled physical tool mounting tilt |
| FK flange | Retained separately in the scene; it is not substituted for the tilted physical tool |
| Green estimates | Latest report estimator update available by the sample tick; invalid, rejected, future or over-age updates show unavailable |
| Green uncertainty | Three times the reported marginal centre standard deviations, in metres; axis uncertainties are also printed in mrad |
| Green axis glyph | Reported 5-DoF axis direction with a fixed screen-length glyph, not a part length or an estimate of roll |
| Amber cross | Commanded TCP position from the runtime, not an executed pose |
| Contact/force text | The plant's timestamped reduced contact packets; forces remain uncalibrated proxies |
| Ownership | Physical held-body ownership from the simulation, visibly separate from controller decisions |
| Phase and reason | Latest ordered controller decision at or before the sample tick; end-of-run status is labelled separately |

The three panels are orthographic XY, XZ and YZ projections in millimetres; the
source schema uses metres, radians, seconds and newtons. Exact scenario and
machine-configuration source JSON is embedded; admission verifies both SHA-256
values against the report. World coordinates and
source/configuration identities are preserved. In the versioned M1e/M1f report,
object 10001 is the peg, 10002 is the socket and 10010 is the tool. Estimated roll
is unavailable. Estimated states remain at their published state tick.

The recorder is opt-in and plant-owned. It does not supply controller inputs,
modify the existing report or its hash, or populate `SceneFrame.estimate` with
fabricated full-state poses. Nonterminal exports and recording overflow are
errors. Overflow does not change control or stop the plant; it makes the
incomplete recording unavailable. The inspector rejects unsupported schemas,
bad units/timestamps, absent geometry, nonfinite values and broken state mapping.
Tool-jaw geometry has string identifiers rather than invented mechanics body IDs.

The exported geometry currently includes mechanics bodies, arm envelopes and
physical jaws. The terminal tube's detailed optical target meshes and rendered
camera views are **not exported**. Core contact snapshots do not replace the
separate raw insertion/pad proxy packets. Visibility decisions remain in the
observation-burst report; the inspector does not synthesize sensor images.

## Stationary observed handoff protocol

```sh
cargo run --locked --release -p pipe_sim_cli --bin pipe-handoff
cargo run --locked --release -p pipe_sim_cli --bin pipe-handoff -- --fault observation_lost_after_transfer
```

The frozen `scenarios/stationary_handoff_m1g_v1.json` coupon uses the M1e machine
configuration, opposing arms 1 and 2, and a 0.40 mm diameter, 5.40 mm overall
length capsule. The other two arms remain parked. Both active arms are
prepositioned by position/axis IK, with their TCPs 2.40 mm from the peg centre.
The donor jaws start preclosed. This is an explicit initialization condition,
not a claim that pickup or two-arm approach was executed.

The runtime executes:

1. Fresh estimates and donor pad evidence authorize donor acquisition.
2. The donor retains ownership while the receiver closes through the normal
   gripper scheduler.
3. New observations and bilateral pad evidence from both arms authorize the
   stationary ownership transaction.
4. The plant rechecks idle arms, donor retention and receiver capture. Every
   fallible check precedes mutation. It transfers exactly one kinematic owner
   without moving the body or advancing the simulation clock.
5. A new observation confirms receiver retention before the donor is commanded
   open. Final fresh observations and loss of donor pad contact complete the
   stationary exchange.

The controller accepts measurement-derived 5-DoF estimates and timestamped pad
packets. It has no physical scene input. Age, availability, uncertainty,
relative position/axis error, contact, force limits, message ordering and command
acknowledgements guard the protocol. Rejected transfers retain the donor;
observation loss after a successful transfer retains the receiver. Both arms
receive Stop on terminal failure. No blind retreat or automatic release is added.

The physical fixture checks inter-arm link/jaw envelopes and both recessed palm
clearances throughout jaw motion. Force proxies have per-tick interlocks. The
core transaction API deliberately leaves collision admission to the calling
fixture/controller; it is not a general motion or collision-planning API.

**Metrology boundary:** this coupon injects deterministic noisy labelled feature
packets into the existing weighted-least-squares estimator. It does not execute
camera/projector projection, occlusion, physical fiducial visibility, or image
detection. Ray-count fields describe the assumed packet interface, not measured
optical performance. This report is labelled F0 protocol-coupon fidelity and
must not inherit M1f's fixed-head feasibility or precision claims.

**Physics boundary:** the exchange uses a single kinematic attachment owner.
Concurrent bilateral pad evidence does not model dual-grasp load sharing.
Gravity, friction, slip, breakaway and calibrated forces remain absent.

## Verification and next gate

`observed_replay` tests compare full controller reports with recording enabled
and disabled, verify geometry/state mapping, terminal failure capture, deterministic
report/replay equality and recording-capacity rejection. CI also requires two
independent M1f replay recordings to be byte-identical. The Python and actual
inspector JavaScript are exercised against a real Rust recording, including
malformed input, finite projections and stale-estimate refusal.

`stationary_handoff` tests cover deterministic nominal exchange and seven fault
profiles: observation loss before receiver closure, before transfer and after
transfer; stale observations; missing receiver pad contact; inconsistent pose;
and physical transaction rejection. Core transaction tests verify exact state
preservation on rejection, pending-motion refusal, one owner and unchanged body
pose on success. The donor retains its original acquisition overlap requirement;
the receiver cannot weaken it. Missing attachment state and nonfinite carriage
state also reject before mutation. Replay regression checks cover missing or
duplicate jaws, nonunit axes, incomplete uncertainty, inconsistent estimate
timestamps and unordered report events in both Python and JavaScript.
Evaluation frames and scores are excluded from the handoff
controller trace hash.

The next gate is a physical fixed-head two-arm coupon: design visible target
geometry, observe both tools and the peg through the actual optics boundary,
execute bounded collision-checked approach and separation trajectories, and
retain these ownership and visibility-loss guarantees. This stationary coupon
does not close that milestone or execute the gearbox assembly.
