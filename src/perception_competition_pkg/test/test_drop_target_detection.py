import unittest

import cv2
import numpy as np

from perception_competition_pkg.drop_target_detection import (
    detect_drop_target,
    DetectionConfig,
)


class DropTargetDetectionTest(unittest.TestCase):
    def test_detects_red_circle_and_reports_normalized_offset(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(image, (240, 80), 30, (0, 0, 255), thickness=-1)

        result = detect_drop_target(image, DetectionConfig(min_area=500.0))

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.nx, 0.5, delta=0.03)
        self.assertAlmostEqual(result.ny, -1.0 / 3.0, delta=0.03)
        self.assertAlmostEqual(result.radius, 30.0, delta=2.0)
        self.assertGreater(result.area_fraction, 0.03)

    def test_rejects_non_circular_red_region(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(image, (20, 100), (300, 115), (0, 0, 255), thickness=-1)

        result = detect_drop_target(
            image,
            DetectionConfig(min_area=500.0, min_circularity=0.75),
        )

        self.assertIsNone(result)
