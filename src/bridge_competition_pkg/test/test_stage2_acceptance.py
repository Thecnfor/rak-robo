import unittest

from bridge_competition_pkg.stage2_acceptance import (
    REQUIRED_PHASES,
    aggregate_acceptance,
    evaluate_run,
)


class Stage2AcceptanceTest(unittest.TestCase):
    def successful_run(self, error=0.01):
        return evaluate_run(
            phase_messages=list(REQUIRED_PHASES),
            px4_status_messages=["ready=true failsafe=false armed=true"],
            landed_values=[False, True],
            final_payload_xy=(5.5 + error, -3.5),
            target_xy=(5.5, -3.5),
            maximum_truth_speed_mps=0.5,
        )

    def test_run_requires_complete_ordered_physical_result(self):
        passed = self.successful_run()
        self.assertTrue(passed["passed"])
        self.assertLess(passed["drop_error_m"], 0.2)

        missing_return = evaluate_run(
            phase_messages=[phase for phase in REQUIRED_PHASES if phase != "RETURN"],
            px4_status_messages=["failsafe=false"],
            landed_values=[True],
            final_payload_xy=(5.5, -3.5),
            target_xy=(5.5, -3.5),
            maximum_truth_speed_mps=0.5,
        )
        self.assertFalse(missing_return["passed"])
        self.assertIn("phase_sequence", missing_return["failed_checks"])

    def test_ten_run_gate_allows_exactly_one_inaccurate_drop_but_no_failed_flight(self):
        nine_accurate = [self.successful_run() for _ in range(9)]
        one_miss = self.successful_run(error=0.25)
        summary = aggregate_acceptance([*nine_accurate, one_miss])
        self.assertTrue(summary["passed"])
        self.assertEqual(summary["successful_runs"], 10)
        self.assertEqual(summary["drop_within_0_2_m"], 9)
        self.assertEqual(summary["drop_success_rate"], 0.9)

        ten_passes = aggregate_acceptance([self.successful_run() for _ in range(10)])
        self.assertTrue(ten_passes["passed"])
        self.assertEqual(ten_passes["drop_within_0_2_m"], 10)
        self.assertEqual(ten_passes["drop_success_rate"], 1.0)

    def test_failsafe_or_missing_landed_proof_fails_the_ticket(self):
        failsafe = evaluate_run(
            phase_messages=list(REQUIRED_PHASES),
            px4_status_messages=["failsafe=true"],
            landed_values=[True],
            final_payload_xy=(5.5, -3.5),
            target_xy=(5.5, -3.5),
            maximum_truth_speed_mps=0.5,
        )
        self.assertFalse(failsafe["passed"])
        self.assertIn("px4_failsafe", failsafe["failed_checks"])

        not_landed = evaluate_run(
            phase_messages=list(REQUIRED_PHASES),
            px4_status_messages=["failsafe=false"],
            landed_values=[False],
            final_payload_xy=(5.5, -3.5),
            target_xy=(5.5, -3.5),
            maximum_truth_speed_mps=0.5,
        )
        self.assertFalse(not_landed["passed"])
        self.assertFalse(aggregate_acceptance([not_landed] * 10)["passed"])

    def test_consecutive_gate_uses_only_the_latest_unbroken_streak(self):
        failed = self.successful_run()
        failed["passed"] = False
        summary = aggregate_acceptance(
            [*([self.successful_run()] * 9), failed,
             *([self.successful_run()] * 10)]
        )
        self.assertTrue(summary["ten_consecutive_flights_ok"])
        self.assertEqual(summary["consecutive_successful_runs"], 10)

        reset = aggregate_acceptance(
            [*([self.successful_run()] * 10), failed]
        )
        self.assertFalse(reset["ten_consecutive_flights_ok"])
        self.assertEqual(reset["consecutive_successful_runs"], 0)

    def test_moving_payload_is_not_accepted_as_a_static_drop(self):
        moving = evaluate_run(
            phase_messages=list(REQUIRED_PHASES),
            px4_status_messages=["failsafe=false"],
            landed_values=[True],
            final_payload_xy=(5.5, -3.5),
            target_xy=(5.5, -3.5),
            maximum_truth_speed_mps=0.5,
            payload_stationary=False,
        )
        self.assertFalse(moving["passed"])
        self.assertFalse(moving["payload_stationary"])

    def test_locked_payload_position_is_not_counted_as_a_drop_attempt(self):
        aborted_before_drop = evaluate_run(
            phase_messages=list(REQUIRED_PHASES[:6]),
            px4_status_messages=["failsafe=false"],
            landed_values=[False],
            final_payload_xy=(5.5, -3.5),
            target_xy=(5.5, -3.5),
            maximum_truth_speed_mps=0.5,
        )

        self.assertFalse(aborted_before_drop["drop_attempted"])
        self.assertIsNone(aborted_before_drop["drop_error_m"])
        summary = aggregate_acceptance([aborted_before_drop])
        self.assertEqual(summary["measured_drops"], 0)


if __name__ == "__main__":
    unittest.main()
