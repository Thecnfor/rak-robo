import unittest

from bridge_competition_pkg.interface_contract import (
    direct_rotor_output_allowed,
    evaluate_interface,
    observed_frequency_hz,
)


class InterfaceContractTest(unittest.TestCase):
    def test_accepts_complete_graph_with_one_executor(self):
        required = [
            '/sensor',
            '/fmu/in/offboard_control_mode',
            '/fmu/in/trajectory_setpoint',
            '/fmu/in/vehicle_command',
        ]
        result = evaluate_interface(
            required,
            {name: ['example/msg/Type'] for name in required},
            {
                name: (
                    ['/trajectory_executor']
                    if name.startswith('/fmu/in/') else ['/simulator']
                )
                for name in required
            },
            {
                name: ['/px4']
                for name in required if name.startswith('/fmu/in/')
            },
        )

        self.assertTrue(result['ok'])
        self.assertEqual(result['missing'], [])
        self.assertTrue(result['unique_fmu_writer'])

    def test_reports_missing_topic_and_multiple_fmu_writers(self):
        result = evaluate_interface(
            ['/missing'],
            {},
            {
                '/fmu/in/trajectory_setpoint': [
                    '/trajectory_executor',
                    '/rogue_writer',
                ],
            },
            {'/fmu/in/trajectory_setpoint': ['/px4']},
        )

        self.assertFalse(result['ok'])
        self.assertEqual(result['missing'], ['/missing'])
        self.assertEqual(
            result['multiple_fmu_writers'],
            ['/fmu/in/trajectory_setpoint'],
        )

    def test_rejects_missing_or_impersonating_fmu_writer(self):
        required = [
            '/fmu/in/offboard_control_mode',
            '/fmu/in/trajectory_setpoint',
            '/fmu/in/vehicle_command',
        ]
        result = evaluate_interface(
            required,
            {name: ['example/msg/Type'] for name in required},
            {
                '/fmu/in/offboard_control_mode': ['/trajectory_executor'],
                '/fmu/in/trajectory_setpoint': ['/other_node'],
            },
            {
                name: ['/px4']
                for name in required
            },
        )

        self.assertFalse(result['ok'])
        self.assertFalse(result['unique_fmu_writer'])
        self.assertEqual(
            result['invalid_fmu_writers'],
            {
                '/fmu/in/trajectory_setpoint': ['/other_node'],
                '/fmu/in/vehicle_command': [],
            },
        )

    def test_direct_mode_does_not_require_fmu_writer(self):
        result = evaluate_interface(
            ['/drone0/control/rotor0/ref'],
            {'/drone0/control/rotor0/ref': ['std_msgs/msg/Float64']},
            {'/drone0/control/rotor0/ref': ['/direct_rotor_smoke_test']},
            {'/drone0/control/rotor0/ref': ['/pegasus']},
            require_fmu_writer=False,
        )

        self.assertTrue(result['ok'])

    def test_direct_rotor_output_requires_both_arming_and_backend(self):
        self.assertTrue(direct_rotor_output_allowed(True, 'direct_rotor'))
        self.assertFalse(direct_rotor_output_allowed(False, 'direct_rotor'))
        self.assertFalse(direct_rotor_output_allowed(True, 'px4'))

    def test_observed_frequency_uses_sample_window(self):
        self.assertEqual(observed_frequency_hz([]), 0.0)
        self.assertEqual(observed_frequency_hz([1.0]), 0.0)
        self.assertAlmostEqual(observed_frequency_hz([1.0, 1.1, 1.2]), 10.0)

    def test_rejects_topic_without_publisher_or_command_subscriber(self):
        required = ['/sensor', '/cargo_bay/command']
        result = evaluate_interface(
            required,
            {name: ['example/msg/Type'] for name in required},
            {'/cargo_bay/command': ['/flight_supervisor']},
            {},
            require_fmu_writer=False,
        )

        self.assertFalse(result['ok'])
        self.assertEqual(result['unpublished'], ['/sensor'])
        self.assertEqual(result['disconnected_commands'], ['/cargo_bay/command'])
