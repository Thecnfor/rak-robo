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

Open. The current `99999` EKF/GPS gates and disabled GPS position failsafe are not
accepted as a solution; they hide estimator-health failures and must be replaced
by a documented sensor/estimator configuration.

## #4: What is the flight-control contract after EGO?

Blocked by: #3
Type: Discuss

### Question

Which trajectory fields, frames, freshness rules and transition guarantees must
the Offboard executor enforce before it can arm or replace an active trajectory?

### Answer

Partially resolved: EGO remains the planner and PX4 Offboard remains the control
boundary. Position, velocity, acceleration and yaw stay in the existing trajectory
contract. Validation, interpolation, command ACK handling and restart semantics
remain to be fixed after the estimator timebase is known.

## #5: What does HOLD mean operationally?

Blocked by: #3, #4
Type: Prototype

### Question

How will the executor capture and maintain a hold target, and when must it hand
control to PX4 Land instead of continuing Offboard?

### Answer

Open. Acceptance must use measured position/velocity bounds; a supervisor `HOLD`
string is not sufficient. The current run is PX4 `AUTO_LAND` while the supervisor
reports HOLD, so mode truth and mission intent must be reconciled.

## #6: What evidence closes the flight-control milestone?

Blocked by: #2, #3, #4, #5
Type: Discuss

### Question

Which staged motor, hover, trajectory, hold, failsafe and landing runs prove that
the Visible Competition Drone is ready to reconnect to the full mission?

### Answer

Open. The final ticket will define synchronized Isaac pose, PX4 state, setpoint,
actuator and video evidence plus numeric tolerances and repeat counts. M9.4 remains
open until those checks pass.

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
