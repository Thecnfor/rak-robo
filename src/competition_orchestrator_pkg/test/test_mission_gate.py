import unittest

from competition_orchestrator_pkg.cargo_contract import cargo_command_and_expected
from competition_orchestrator_pkg.mission_gate import (
    door_action_allowed,
    flight_action_allowed,
    MissionGate,
)


class MissionGateTest(unittest.TestCase):
    def test_air_mission_dispatches_once_after_ground_completion(self):
        gate = MissionGate()
        self.assertFalse(gate.observe('RUNNING'))
        self.assertTrue(gate.observe('COMPLETE'))
        self.assertFalse(gate.observe('COMPLETE'))

    def test_air_mission_accepts_success_alias_but_not_failure(self):
        self.assertTrue(MissionGate().observe('SUCCESS'))
        self.assertFalse(MissionGate().observe('FAILED'))

    def test_air_mission_accepts_existing_ground_done_state(self):
        self.assertTrue(MissionGate().observe('GROUND_DONE'))

    def test_cargo_close_uses_closed_acknowledgement(self):
        self.assertEqual(
            cargo_command_and_expected('left', 'close'),
            ('left_close', 'left_closed'),
        )
        self.assertEqual(
            cargo_command_and_expected('bottom', 'open'),
            ('bottom_open', 'bottom_opened'),
        )

    def test_competition_mode_only_accepts_safe_flight_actions(self):
        self.assertTrue(flight_action_allowed(0, 'IDLE', False))
        self.assertFalse(flight_action_allowed(1, 'IDLE', False))
        self.assertFalse(flight_action_allowed(3, 'EGO_TRANSIT', False))
        self.assertTrue(flight_action_allowed(4, 'EGO_TRANSIT', False))
        self.assertTrue(flight_action_allowed(5, 'VISUAL_ALIGN', False))
        self.assertFalse(flight_action_allowed(0, 'TAKEOFF', False))

    def test_manual_actions_require_explicit_opt_in(self):
        self.assertTrue(flight_action_allowed(1, 'IDLE', True))
        self.assertTrue(flight_action_allowed(3, 'EGO_TRANSIT', True))
        self.assertFalse(door_action_allowed('RETURN', False))
        self.assertTrue(door_action_allowed('RETURN', True))
        self.assertTrue(door_action_allowed('COMPLETE', False))
