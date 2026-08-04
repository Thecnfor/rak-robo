# Air–Ground Competition Context

This glossary separates mission intent from physical evidence in the competition's
drone workflow. A phase label is never treated as proof that the corresponding
flight behaviour occurred.

## Language

**Visible Competition Drone**:
The official scene vehicle shown in the supplied video. Only its observed motion
can satisfy a competition flight requirement.
_Avoid_: Drone model, Isaac marker, visible SIH drone

**SIH Vehicle**:
An invisible PX4-internal vehicle used to validate flight-control protocol without
the competition scene dynamics.
_Avoid_: Competition drone, Isaac drone

**Closed-Loop Flight**:
A run in which commands actuate the Visible Competition Drone and all control
feedback describes that same vehicle.
_Avoid_: Topic chain online, state-machine pass

**Mission Phase**:
The supervisor's current intended task stage, such as TAKEOFF or HOLD. It describes
intent, not achieved physical behaviour.
_Avoid_: Flight state, flight result

**Flight State**:
The jointly observed arming, control-mode, failsafe, pose, velocity and actuator
condition of one vehicle.
_Avoid_: Mission phase

**Position Hold**:
A measured behaviour in which the vehicle remains inside an agreed position and
velocity tolerance for a specified duration.
_Avoid_: HOLD label, search-pose reached

**Flight Ticket**:
A reproducible run with synchronized logs and explicit numeric acceptance results.
_Avoid_: Walkthrough, screenshot proof