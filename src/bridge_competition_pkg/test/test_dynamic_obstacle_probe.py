import math
import unittest

from bridge_competition_pkg.dynamic_obstacle_probe import (
    count_transformed_points_in_box,
    obstacle_visibility_passes,
    status_bounds_match,
)


class DynamicObstacleProbeTest(unittest.TestCase):
    def test_counts_lidar_points_after_map_transform(self):
        half_turn_z = math.sqrt(0.5)
        transform = ((5.5, -2.75, 1.8), (0.0, 0.0, half_turn_z, half_turn_z))
        lidar_points = [
            (0.0, 0.0, 0.0),
            (0.04, 0.03, 0.2),
            (0.5, 0.5, 0.0),
            (math.nan, 0.0, 0.0),
        ]

        count = count_transformed_points_in_box(
            lidar_points,
            transform=transform,
            center=(5.5, -2.75, 1.8),
            size=(0.18, 0.18, 0.8),
        )

        self.assertEqual(count, 2)

    def test_visibility_requires_absolute_and_incremental_hits(self):
        self.assertTrue(
            obstacle_visibility_passes(
                baseline_count=0,
                obstacle_count=4,
                minimum_points=3,
                minimum_increment=2,
            )
        )
        self.assertFalse(
            obstacle_visibility_passes(
                baseline_count=3,
                obstacle_count=4,
                minimum_points=3,
                minimum_increment=2,
            )
        )

    def test_status_bounds_must_match_requested_geometry(self):
        status = (
            'spawned center=(5.5, -2.75, 1.8) size=(0.6, 0.6, 1.0) '
            'bounds_min=(5.200,-3.050,1.300) '
            'bounds_max=(5.800,-2.450,2.300)'
        )
        self.assertTrue(
            status_bounds_match(
                status, center=(5.5, -2.75, 1.8), size=(0.6, 0.6, 1.0)
            )
        )
        self.assertFalse(
            status_bounds_match(
                status, center=(5.5, -2.75, 1.8), size=(0.18, 0.18, 0.8)
            )
        )


if __name__ == '__main__':
    unittest.main()
