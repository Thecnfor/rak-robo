# Drone Flight Replan Decision Map

Goal: replace the current SIH/Isaac hybrid demonstration with measured closed-loop
control of the Visible Competition Drone, while keeping EGO upstream of PX4
Offboard control.

## #1: Which vehicle is the acceptance target?

Blocked by: none
Type: Research

### Question

Do the supplied video, ROS state and PX4 state describe the same vehicle?

### Answer

No. The video identifies the Visible Competition Drone, but the moving
`/drone/navigation/odometry` currently comes from PX4 `sihsim_quadx`. Isaac
`/drone0/state/pose` and `/drone_0_ego_odom` remain at zero. Mission-phase changes
and SIH motion are therefore not evidence of competition-drone flight.

## #2: How will Pegasus own the existing scene vehicle?

Blocked by: #1
Type: Prototype

### Question

What is the smallest safe adapter that makes Pegasus actuate the existing Visible
Competition Drone without creating `/World/quadrotor_01`, an empty USD reference,
or invalid rigid-body callbacks?

### Answer

Resolved on 2026-07-21.

- `Vehicle`/`Multirotor` now have an explicit `attach_existing` path. It wraps
  exactly `/World/quadrotor`, does not call `get_stage_next_free_path`, does not
  define another prim, and does not add an empty USD reference.
- `scene_app.py` captures the composed scene pose, validates the articulation
  root plus body/rotor rigid-body schemas before attachment, then validates all
  dynamic-control handles after `World.reset_async()`. Reset or handle failures
  now abort setup instead of continuing with invalid callbacks.
- Non-zero motor input wakes a sleeping articulation before applying force.
- `direct_rotor_motion_probe` waits for both state and the selected rotor
  subscriber, pulses only one rotor, measures rigid-body response, and always
  zeros all four outputs in `finally`.

Runtime evidence in direct mode, with the probe waiting before the scene
timeline started: rotor 0 at 600 rad/s for 0.5 s moved from
`(4.5513,-0.3819,1.1299)` to `(4.5533,-0.3852,1.1297)`, with 0.0039 m maximum
position delta, 0.2088 rad maximum attitude delta, and `passed=true`. Scene
bringup logged exact attachment and valid physics handles; no
`/World/quadrotor_01` was created.

## #3: Which PX4 estimator inputs are valid for Pegasus flight?

Blocked by: #2, #7
Type: Research

### Question

Which simulated IMU, magnetometer, barometer, GNSS or external-vision streams are
actually supplied by Pegasus, and which PX4 health checks can pass without broad
gate bypasses?

### Answer

Resolved on 2026-07-22.

- Pegasus supplies IMU, magnetometer, barometer and GNSS through Simulator
  MAVLink. The same run exposed raw ROS observer streams and PX4
  `sensor_combined`/`vehicle_odometry` at about 5.2 Hz while `/clock` ran at about
  3.2 Hz wall rate.
- The Docker startup no longer edits generated PX4 startup files. Its only PX4
  parameter override is `UXRCE_DDS_SYNCT=0`, which is a simulation-time policy.
  Live PX4 values were the normal gates (`EKF2_BARO_GATE=5`,
  `EKF2_GPS_P_GATE=5`, `EKF2_MAG_GATE=3`, `EKF2_REQ_EPH=3`,
  `EKF2_REQ_EPV=5`); no `99999` estimator gate remained.
- PX4 reported `pre_flight_checks_pass=true` and `failsafe=false` through two
  visible-drone flights. Isaac pose, PX4 odometry, Offboard setpoints, actuator
  output and motor-driven motion were recorded together.
- The successful-run ULog had `filter_fault_flags=0`, `time_slip=0`; maximum
  heading/velocity/position/height test ratios were respectively
  0.0762/0.3606/0.0111/0.0447. GNSS position/velocity, barometer height and
  magnetometer aid sources had zero rejected samples. Evidence includes SHA-256
  hashes and all five PX4 command ACK results (`result=0`).

The measured simulator real-time factor was only 0.0513. Flight trajectory
progress therefore uses `/clock`; transport freshness remains on steady wall
time so a stalled simulator still enters HOLD/LAND.

## #4: What is the flight-control contract after EGO?

Blocked by: #3
Type: Discuss

### Question

Which trajectory fields, frames, freshness rules and transition guarantees must
the Offboard executor enforce before it can arm or replace an active trajectory?

### Answer

Partially resolved: EGO remains the planner and PX4 Offboard remains the control
boundary. Position, velocity, acceleration and yaw stay in the existing trajectory
contract. The executor now owns an explicit prestream/active/hold/land/complete
lifecycle, enforces a single writer for the three contracted Offboard inputs, samples trajectories in simulation
time, and prevents the rolling planner from restarting the slow beginning of a
trajectory. A live 2026-07-31 trial showed that accepting every normal replan
after 4 simulated seconds repeatedly restarts the zero-speed segment and stalls
progress; it was safely aborted and reverted. Normal same-goal updates therefore
remain pinned through the endpoint, while NO_PATH recovery, phase changes and
unsafe return-altitude corrections may preempt immediately. The polyline
fallback now stops at each A* corner with a triangular/trapezoidal speed profile
constrained by both maximum velocity and acceleration. A dedicated
`ACTIVE_OBSTACLE_REPLAN` signal and dynamic-obstacle real-scene validation
remain open.

## #5: What does HOLD mean operationally?

Blocked by: #3, #4
Type: Prototype

### Question

How will the executor capture and maintain a hold target, and when must it hand
control to PX4 Land instead of continuing Offboard?

### Answer

Resolved for the controlled-hover slice on 2026-07-22. HOLD captures a world-frame
position and keeps the Offboard stream alive. LAND is terminal until PX4 reports
AUTO_LAND, landed and disarmed. Because PX4 did not auto-disarm reliably, the sole
executor now sends DISARM only after both LAND_LATCHED and landed have remained
true for two steady-wall seconds. The successful run showed HOLD while PX4 was
Offboard, followed by LAND_LATCHED while PX4 was AUTO_LAND; there was no longer a
HOLD/AUTO_LAND truth mismatch.

## #6: What evidence closes the flight-control milestone?

Blocked by: #2, #3, #4, #5
Type: Discuss

### Question

Which staged motor, hover, trajectory, hold, failsafe and landing runs prove that
the Visible Competition Drone is ready to reconnect to the full mission?

### Answer

Partially resolved. Synchronized visible-drone runs now prove physical takeoff,
hover intent, AUTO_LAND, touchdown and automatic disarm. The latest `round1i` used
the final safety build and returned `success=true`; exact state/command sequences,
errors, bag and ULog paths
are recorded in
[`drone_hover_evidence_2026-07-22.json`](drone_hover_evidence_2026-07-22.json).

M9.4 remains open for the formal 1.8 m / 30 s hover because the current runs were
deliberately limited to z=1.30/1.45 m. `round1i` held within 0.0901 m horizontally
but still reached 0.0827 m/s and displaced 0.2756 m during landing. The next ticket
is speed-settling/controller and landing calibration, then 1.8 m / 30 s and repeat trials.

The 2026-07-26 fixed-setpoint ladder supersedes that altitude limitation. One
single-round run passed +0.10 through +0.30 m and returned to the cradle. A
subsequent run passed +0.10/+0.20/+0.30/+0.40/+0.50/+0.60/+0.67 m, reached
world z=1.80 m, held for five simulated seconds, returned through the 8 mm
Isaac-truth capture gate, landed in the verified support and disarmed. All five
command ACKs succeeded. ULog `2026-07-26/12_57_18.ulg` recorded actual
roll/pitch maxima 0.289/0.694 degrees, motor maxima 0.584--0.602, and no
saturation, failsafe or failure-detector flag. The exact probe artifact is
`/tmp/drone_stage2_fullheight_180m_return_20260726.json`.

The subsequent formal 30-second run also passed the complete ladder, hold,
truth-guided return, landing and disarm. Its hold maxima were 0.0347 m
horizontal error, 0.0154 m altitude error and 0.0267 m/s speed. ULog
`2026-07-26/13_32_30.ulg` recorded actual roll/pitch maxima 0.240/0.888
degrees, motor maxima 0.504--0.518, and no saturation, failsafe or
failure-detector flag. The checksummed combined record is
[`drone_fullheight_hold30_evidence_2026-07-26.json`](drone_fullheight_hold30_evidence_2026-07-26.json).
M9.4 fixed-setpoint hover is resolved; repeatability and EGO navigation remain
open.

The 2026-07-22 `round2` rerun did not pass: it reached z=1.325 m but failed to
converge within the 0.15 m horizontal takeoff envelope before the 15 s simulated
timeout. The safety chain requested NAV_LAND, received all five command ACKs with
`result=0`, disarmed and reset, but the vehicle landed off the table at z=0.263 m.
Evidence is in `bags/drone_flight_test_20260722_round2`,
`/tmp/drone_flight_test_round2_20260722.json` and PX4 ULog
`2026-07-22/11_04_46.ulg`. This run reopens the tracking and landing portions of
the milestone; it must not be counted as a successful hover.

## #7: What keeps the drone valid before arming?

Blocked by: #2
Type: Prototype

### Question

How will the Visible Competition Drone remain at its calibrated spawn pose while
PX4 is booting, and how will the cargo/payload joints attach without snapping or
destabilizing the vehicle at the first physics step?

### Answer

Resolved on 2026-07-21.

- The launch table now carries two static collision pads under the closed bottom
  door panel, split around its handle. They have no rigid-body API and no joint
  to the vehicle, so they support zero-thrust startup and separate naturally
  when thrust lifts the aircraft.
- The payload joint now uses USD row-vector composition
  (`pencil_world * cargo_world.GetInverse()`). Its cargo-local position is
  `(0,0,0.072)` and scene setup rejects translation/rotation anchor errors above
  `1e-4` before PhysX can snap the bodies together.
- A 12 s zero-motor run stayed within 0.000172 m position, 0.000974 rad
  roll/pitch and 0.0144 m/s. Closing the side door during the run left the
  payload within 0.000019 m of its locked pose. No disjoint-joint, duplicate
  vehicle or invalid-handle warning occurred.
- A post-start rotor-1 pulse at 600 rad/s for 1.0 s produced 0.00229 rad attitude
  and 0.0968 rad/s angular-rate response, proving the support does not tether
  the vehicle. Full measurements are in
  [`prearm_support_evidence_2026-07-21.json`](prearm_support_evidence_2026-07-21.json).

Ticket #3 is now unblocked. PX4 estimator work must use this supported, settled
initial state and must not restore broad EKF/GPS gate bypasses.

## #8: How is a valid prearm pose restored and enforced after an off-table landing?

Blocked by: #7
Type: Prototype

### Question

How will scene reset restore the official spawn pose, and how will preflight prove
that the vehicle is physically above the support rather than merely observing that
the support prim exists?

### Answer

Resolved on 2026-07-22.

- The probe and supervisor now share a calibrated Isaac raw-pose envelope around
  `(4.55,-0.38,1.13)`: position error <=0.02 m, speed <=0.05 m/s and absolute
  roll/pitch <=3 degrees. Missing or stale raw pose/twist also fails closed.
- Ground arming requires the envelope, while an already armed Offboard flight
  bypasses it so natural departure from the support cannot interrupt setpoints.
  `/drone/navigation/state` exposes `prearm_pose_valid=true|false`.
- Before scene reset, the off-table pose `(5.1876,-0.3052,0.2630)` had 1.078791 m
  position error. After cargo setup, the pose envelope was the only failed gate;
  executor lifecycle remained `DISABLED` and PX4 remained disarmed.
- After a controlled Isaac/PX4 restart, raw pose returned to
  `(4.549878,-0.379997,1.129992)`. The successful preflight-only sample had
  0.000094 m error, 0.003547 m/s speed and 0.0395 degree maximum tilt. It exited
  successfully without entering ARM.

The compact evidence is
[`prearm_pose_gate_evidence_2026-07-22.json`](prearm_pose_gate_evidence_2026-07-22.json).
`prearm_support=true` remains an existence check and never substitutes for this
physical pose gate. The repair frontier advances to #9.

## #9: When is the map/lidar transform and local map actually ready?

Blocked by: #3
Type: Prototype

### Question

How will bringup distinguish a transient TF discovery race from a missing transform,
and prevent arming until point clouds have been transformed and ingested?

### Answer

Resolved on 2026-07-22.

- The planner now reports `map_ready`, `map_age` and `tf_age` only from a
  successfully transformed cloud that was ingested into the rolling voxel map.
  Merely receiving `/avoidance/lidar/pointcloud` cannot satisfy the gate.
- Ground arming requires a fresh positive planner state no older than 0.60 s in
  both the supervisor and the independent hover probe. Missing, malformed,
  negative or stale evidence fails closed. Once PX4 is armed, this ground-only
  gate is bypassed so a transient planning update cannot stop the Offboard stream.
- The same `planner_map_timeout` parameter drives planner publication,
  supervisor gating and probe acceptance. During the one-second Offboard
  prestream, an unarmed vehicle continuously rechecks readiness; loss of the gate
  returns the state machine to `PREFLIGHT` and disables the executor instead of
  completing ARM. `map_ready` is parsed as an exact boolean token, so malformed or
  conflicting tokens fail closed.
- The live chain eventually provides `map -> avoidance_base_link` and the static
  `avoidance_base_link -> avoidance_lidar` transform. The initial warning that
  `map` does not exist is a discovery-order race: the planner retries and does
  not claim readiness until the transform succeeds.
- Runtime validation reported `map_ready=true map_age=0.000000 tf_age=0.000000`;
  the preflight-only probe then passed with a planner-state age of about 0.045 s
  and an observed rate of about 4.99 Hz, without entering ARM.

The repair frontier advances to #10. This closes unsafe map readiness, but does
not explain the horizontal tracking oscillation observed during flight.

## #10: What produces the horizontal tracking oscillation?

Blocked by: #8, #9
Type: Prototype

### Question

Can PX4 hold a fixed position without EGO replanning, and which vehicle-model or
PX4 controller parameters must change to meet the 0.05 m/s settling requirement?

### Answer

In progress. Bag analysis rules out a gross frame/origin error: PX4-derived navigation
odometry tracked Isaac ground truth with roughly centimetre-scale mean error, and
all EGO trajectories ended at the fixed target `(5.004, -0.222, 1.300)`. Run a
fixed-setpoint step ladder with EGO updates frozen: supported idle, +0.10 m,
+0.20 m and 10 s HOLD. Record command, PX4 odometry, Isaac truth, attitude,
actuator saturation and EKF innovations. Tune/validate mass, inertia, rotor thrust
curve, hover thrust and PX4 MPC gains one variable group at a time. Exit only when
XY error <=0.10 m, speed <=0.05 m/s and no motor saturation are maintained for
10 simulated seconds.

Fixed-setpoint rounds on 2026-07-22 isolated model mismatches before gain
tuning. PX4's Iris control allocation used 0.15--0.245 m rotor arms while the
assembled USD rotor centers are at `(±0.065,±0.065)` m; the Docker runtime now
overrides `CA_ROTOR{0..3}_{PX,PY,KM}` with measured FRD geometry and the Pegasus
thrust/rolling-moment ratio. Pegasus's nominally visual rotor animation also
drove massive revolute rotor joints at 5/100 rad/s while the thrust model
separately applied rolling moment, so the competition scene now disables that
physical animation. The narrow contact footprint is now surrounded by a
non-jointed launch cradle: two support pads plus four 0.05 m high zero-friction
lateral guides with 15 mm clearance. The first repeat exposed constrained-control
windup, so the diagnostic executor now has an explicit vertical-only mode (Z
position, zero XY velocity, yaw unset), and the probe can descend onto the
support before requesting PX4 LAND. Normal EGO/trajectory setpoints do not use
this diagnostic mode.

The final +0.10 m run passed the 0.02 m horizontal, 0.015 m altitude and
0.05 m/s settling gates, reached 0.1171 m maximum clearance, held with
0.0381 m/s maximum speed, returned to the support and disarmed 0.01243 m from
the verified centre. All five PX4 command ACKs succeeded. ULog showed balanced
per-motor maxima of 0.5345--0.5364, zero samples above the 0.95 saturation
threshold, no failsafe, no EKF filter/GPS-check fault, and maximum
velocity/position/height/heading innovation test ratios of
0.0283/0.0052/0.0025/0.0218. Evidence is in
[`drone_cradle_takeoff_evidence_2026-07-22.json`](drone_cradle_takeoff_evidence_2026-07-22.json).
The cradle/+0.10 m milestone is resolved; ticket #10 remains open until the
+0.20 m and 10 simulated-second free-hold criteria pass outside the guides.

A 2026-07-23 regression with the shortened guide exposed a second handoff bug.
With the side and bottom doors closed before PX4 startup, preflight attitude
agreement was within 0.03 degrees and ULog showed balanced motor peaks
0.549--0.560, no saturation, roll/pitch rate-integral maxima 0.0216/0.0089,
and attitude below about 1.3 degrees. Nevertheless the executor kept
`fixed_vertical_active=true` until +0.08 m while the physical guide ended at
+0.05 m. The airframe therefore accumulated 0.20 m of nearly level lateral
motion without an XY position loop. The abort path then incorrectly attempted
an XY return while XY control was still disabled. PX4 NAV_LAND completed safely
on the table and disarmed, but outside the cradle.

The repaired contract makes the guide height explicit, validates that the
handoff is at least 5 mm below it, and now releases at +0.04 m for the +0.05 m
guide. The probe treats `fixed_vertical_active=false` as the only evidence that
XY recovery is available. The first repair used a 0.05 m pre-handoff bound;
the later K=0.5 regression proved that this still extended beyond the physical
15 mm guide clearance. The final bound is 12 mm, leaving 3 mm margin, and
causes an immediate LAND instead of `ABORT_RETURN`. Evidence for the failed regression
is `bags/drone_low_guide_closed_first_010m_20260723`,
`/tmp/drone_low_guide_closed_first_010m_20260723.json`, and PX4 ULog
`docker/px4/ulog/2026-07-23/05_59_42.ulg`. Ticket #10 remains open until the
corrected +0.10 m rerun and then +0.20 m/10 s free hold pass.

The corrected +0.10 m rerun then passed. Full XY control released at PX4/raw
clearances 0.0403/0.0442 m with only 0.0073 m horizontal displacement, and
re-engaged during return at 0.0292/0.0295 m. The 5 s hold stayed within
0.0215 m horizontal error, 0.0222 m altitude error and 0.018 m/s. The vehicle
returned to the verified cradle rectangle, landed, disarmed and completed all
five command ACKs. ULog motor maxima were 0.5006--0.5132 with no saturation,
failsafe, estimator fault or failure-detector flag. Evidence is in
[`drone_low_guide_handoff_evidence_2026-07-23.json`](drone_low_guide_handoff_evidence_2026-07-23.json).
Ticket #10 now advances to the +0.20 m/10 s free-hold rerun; it remains open.

The first +0.20 m attempt on 2026-07-23 did not pass. The +0.10 m step was
stable, but the next step developed a pitch oscillation and the probe tripped
its envelope at 20.4 degrees and 0.219 m/s. It selected `ABORT_RETURN` only
after full XY handoff, then escalated to PX4 LAND when yaw rate exceeded the
0.5 rad/s limit. The vehicle landed and disarmed on the table. ULog
`docker/px4/ulog/2026-07-23/06_49_20.ulg` showed no motor saturation or PX4
failsafe: commanded pitch stayed below 5.921 degrees while measured pitch
reached 21.599 degrees and the pitch rate integral reached its 0.05 limit.
This localizes the open issue to attitude/rate dynamics or the simulated
airframe inertia rather than insufficient thrust.

A single-variable follow-up temporarily reduced `MC_ROLLRATE_K` and
`MC_PITCHRATE_K` from 1.0 to 0.5. It was rejected: before full XY handoff the
airframe exceeded the guided 0.05 m horizontal envelope, LAND could not keep
it on the table, and it fell to the floor. ULog
`docker/px4/ulog/2026-07-23/07_08_32.ulg` recorded motor peaks up to 0.999,
about 90 degrees roll and a failure-detector flag. PX4 was disarmed and
stopped, and both temporary gain overrides were removed from Docker Compose.
Do not repeat this gain setting. Before another powered test, reset the scene,
measure the PhysX inertia tensor against the PX4 plant model, and investigate
outer attitude gains while keeping the previously stable rate gain at 1.0.
The pre-handoff probe bound is now 12 mm rather than the unsafe 50 mm. Ticket
#10 remains open and no higher-altitude flight is authorized.

Three subsequent +0.10 m diagnostic attempts kept the stable rate gains at
1.0 and changed only the pitch outer-loop gain to `MC_PITCH_P=4.0`. All three
were terminated by the pre-handoff safety envelope and completed PX4 LAND and
disarm without a crash. Their ULogs show actual pitch below 1.66 degrees,
actual roll below 1.17 degrees, balanced motor peaks below 0.557, no
saturation and no failsafe. The first trace also showed why a handoff at
+0.04 m was too late: horizontal displacement was only 2.5 mm at +0.03 m but
10.2 mm at +0.04 m, followed by one executor cycle before finite XY setpoints.
The executor therefore now releases at +0.03 m and re-engages at +0.02 m.

The remaining pre-handoff movement was not a PX4 attitude failure. Offline USD
inspection found that the old guide envelope came from
`transparent_cargo_bay`, which is a visual Xform with no collision or rigid
body API. The actual main collision
`/World/quadrotor/body/body_collision` remained 15--17.5 mm from those walls,
so the 12 mm abort gate fired before any lateral contact could occur. The
scene now generates the four guide walls directly around the main collision;
the initial 5 mm clearance was subsequently calibrated to 10 mm. It filters
the wider cargo body, doors, locked payload and
rotors from those walls, and closes the left cargo door before PX4 estimator
startup. Without the cargo filter, the initial wall overlap drove the
articulation to NaN and Isaac Sim exited with SIGSEGV. Nine offline geometry
tests now pass. A 10-second unpowered live capture received 136 poses; its
retained one-second samples had 1.23e-7 m maximum position drift, zero
roll/pitch and 0.000139 m/s maximum speed, while the scene remained active
beyond the former crash point. The persisted sample and hash are referenced by
the consolidated evidence. This is not yet a
powered flight ticket: the next gate is a PhysX contact-report or low-speed push proof,
followed by exactly one +0.10 m/5 s run. No +0.20 m attempt is authorized
until that run, the probe, and the ULog attitude audit all pass. Consolidated
evidence is in
[`drone_pitch_handoff_evidence_2026-07-23.json`](drone_pitch_handoff_evidence_2026-07-23.json).

The diagnostic probe now treats the 10 s HOLD criteria as hard per-sample gates,
observes normalized motor output for saturation, and rechecks the measured
touchdown after disarm. A numerically valid rectangle is no longer sufficient:
arming requires an explicitly supplied, scene-verified landing region. EKF
innovation extraction and the five-command-ACK acceptance summary remain required
evidence for the next successful run.

On 2026-07-26 the corrected rotor allocation, inertia-matched scene and bounded
PX4 envelope completed a new +0.10 m flight. Two intentionally failed return
trials isolated a stale guide contract: the current scene has 10 mm clearance
around `body_collision`, while the probe still enforced the older 4 mm return
radius and coupled it to the independent prearm tolerance. PX4 local position
entered that old radius for 4.50 s, but GPS/EKF versus Isaac truth accumulated
about 20 mm of dynamic horizontal bias, so further MPC gain reduction could not
solve millimetre cradle insertion. The rejected single-variable
`MPC_XY_P=0.50` trial was reverted to 0.95.

The return gate now uses Isaac truth and an 8 mm radius, preserving 2 mm physical
clearance inside the measured 10 mm guide; prearm remains 4 mm. The subsequent
ticket completed `FIXED_STEP -> FIXED_HOLD -> RETURN_HOME -> LAND -> RESET`
without timeout or abort. Touchdown was
`(4.549843,-0.380000,1.130000)`, 0.16 mm from the cradle centre. ULog
`docker/px4/ulog/2026-07-26/01_32_29.ulg` recorded maximum actual
roll/pitch 3.73/2.39 degrees, motor maxima
0.588/0.503/0.562/0.774, zero samples at or above 0.95, no failsafe,
failure-detector or EKF fault, and all five command ACKs succeeded. The exact
probe artifact is `/tmp/drone_stage2_guide_aligned_010m_20260726.json`.
The +0.10 m gate is resolved again. Later on 2026-07-26, +0.20 and +0.30 m
steps passed in one run, followed by the complete +0.67 m/world-z=1.80 m
ladder and a five-second hold. Ticket #10 is therefore resolved for bounded
climb and short hold; the formal 10/30-second endurance and EGO trajectory
continuity remain separate acceptance tickets.

## #11: How are EGO replans handed to an active trajectory continuously?

Blocked by: #4, #10
Type: Prototype

### Question

What continuity and priority checks replace the current time-only four-second
replan acceptance rule?

### Answer

Partially resolved. The planner published about 4.6 trajectories/s during
`round2`; endpoints were fixed, but each plan restarted from the current state.
The executor now rejects ordinary same-goal updates until the active endpoint
and has explicit immediate preemption for phase changes, NO_PATH recovery and
unsafe return-altitude corrections. A 2026-07-31 live attempt to accept every
ordinary update after 4 simulated seconds confirmed the restart-stall failure
and was safely landed, so that policy was removed. Remaining work is a dedicated
collision-risk signal (`ACTIVE_OBSTACLE_REPLAN`) with splice continuity checks
before live dynamic-obstacle acceptance.

## #12: Where may normal and emergency landing occur?

Blocked by: #8, #10
Type: Discuss

### Question

How will normal LAND return to a verified touchdown region without weakening the
immediate failsafe path?

### Answer

Resolved for normal fixed-setpoint return; emergency-site selection remains
open. The verified support polygon is centered at `(4.55,-0.38)` with
`(0.02,0.02)` half extents. Normal return follows a gain-2 Isaac-truth outer
loop until it is inside the 8 mm capture radius below 0.01 m/s, uses 12 mm
hysteresis to reject a bad capture, and hands PX4 Land over 0.055 m above the
support. The full-height run touched down about 10.2 mm from the center, inside
the physical guide/support, then disarmed.

Normal mission completion must return above home, settle inside the polygon, then
request NAV_LAND. Tracking timeout outside the polygon must select a separately
verified emergency landing policy; transport loss or PX4 failsafe still requests
NAV_LAND immediately. Arming is forbidden when no valid touchdown policy is
available. The probe must record touchdown position and fail if it leaves the
selected region.

## #13: Which staged reruns close M9.4 after the repair?

Blocked by: #8, #9, #10, #11, #12
Type: Discuss

### Question

What sequence proves the repaired chain before returning to obstacle navigation?

### Answer

Partially resolved. Offline/unit tests, reset/support gates, transformed map
readiness, bounded fixed-setpoint climbs through world z=1.80 m, the formal
30-second hold, safe return and landing have passed. Remaining order: an
empty-space EGO goal/return; then three consecutive EGO
takeoff/return/landing runs before obstacle and mission rehearsal.
Every flight must preserve the unique `/fmu/in/*` writer, five successful command
ACKs, no failsafe, no actuator saturation, speed <=0.05 m/s in the acceptance
window, and touchdown inside the selected polygon. Only then resume obstacle and
mission work.
