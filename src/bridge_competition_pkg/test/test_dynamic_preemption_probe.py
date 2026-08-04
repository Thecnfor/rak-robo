import unittest

from bridge_competition_pkg.dynamic_preemption_probe import (
    choose_obstacle_center,
    obstacle_cluster,
)


class DynamicPreemptionProbeTest(unittest.TestCase):
    def test_selects_interior_path_point(self):
        path = [(index * 0.1, 0.0, 1.0) for index in range(21)]
        center = choose_obstacle_center(path)
        self.assertAlmostEqual(center[0], 1.0)

    def test_rejects_path_without_safe_interior_point(self):
        with self.assertRaises(ValueError):
            choose_obstacle_center([(0.0, 0.0, 1.0), (0.1, 0.0, 1.0), (0.2, 0.0, 1.0)])

    def test_cluster_surrounds_center(self):
        points = obstacle_cluster((1.0, 2.0, 3.0))
        self.assertEqual(len(points), 343)
        self.assertIn((1.0, 2.0, 3.0), points)


if __name__ == '__main__':
    unittest.main()
