"""Small public policy object for the ground-to-air handoff."""


def _phase(drone_state: str) -> str:
    return drone_state.strip().split(maxsplit=1)[0].upper()


def flight_action_allowed(
    command: int, drone_state: str, allow_manual_actions: bool
) -> bool:
    """Keep public actions from racing the autonomous supervisor."""
    if command in {4, 5}:  # LAND and ABORT are always legal safety requests.
        return True
    if command == 0:  # TAKEOFF only starts a new gated mission.
        return _phase(drone_state) in {'UNKNOWN', 'IDLE', 'COMPLETE'}
    return allow_manual_actions and command in {1, 2, 3}


def door_action_allowed(drone_state: str, allow_manual_actions: bool) -> bool:
    """Let the supervisor own cargo commands throughout an autonomous mission."""
    return allow_manual_actions or _phase(drone_state) in {
        'UNKNOWN',
        'IDLE',
        'COMPLETE',
    }


class MissionGate:
    """Emit exactly one air-mission request after a successful ground phase."""

    def __init__(self) -> None:
        self._dispatched = False

    def observe(self, ground_state: str) -> bool:
        if self._dispatched or ground_state.strip().upper() not in {
            'COMPLETE',
            'SUCCESS',
            'GROUND_DONE',
        }:
            return False
        self._dispatched = True
        return True
