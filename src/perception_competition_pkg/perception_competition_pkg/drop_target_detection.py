"""Classical color/circle detector for the competition drop target."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectionConfig:
    """Parameters kept independent of ROS so detection is easy to replay/test."""

    hsv_low_1: Tuple[int, int, int] = (0, 90, 70)
    hsv_high_1: Tuple[int, int, int] = (12, 255, 255)
    hsv_low_2: Tuple[int, int, int] = (168, 90, 70)
    hsv_high_2: Tuple[int, int, int] = (179, 255, 255)
    min_area: float = 300.0
    min_circularity: float = 0.70
    morphology_kernel: int = 5


@dataclass(frozen=True)
class DropTargetDetection:
    """Target location normalized about the image center, plus size evidence."""

    nx: float
    ny: float
    area_fraction: float
    radius: float
    center_x: float
    center_y: float
    circularity: float


def _target_mask(image_bgr: np.ndarray, config: DetectionConfig) -> np.ndarray:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, config.hsv_low_1, config.hsv_high_1)
    mask |= cv2.inRange(hsv, config.hsv_low_2, config.hsv_high_2)
    kernel_size = max(1, config.morphology_kernel | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def detect_drop_target(
    image_bgr: np.ndarray,
    config: DetectionConfig = DetectionConfig(),
) -> Optional[DropTargetDetection]:
    """Return the strongest unencoded circular color target in a BGR image."""
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('image must be a non-empty BGR array')
    height, width = image_bgr.shape[:2]
    if width == 0 or height == 0:
        raise ValueError('image must have non-zero dimensions')

    contours, _ = cv2.findContours(
        _target_mask(image_bgr, config), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.min_area:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if circularity < config.min_circularity:
            continue
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        candidates.append((area, center_x, center_y, radius, circularity))
    if not candidates:
        return None

    area, center_x, center_y, radius, circularity = max(candidates, key=lambda item: item[0])
    return DropTargetDetection(
        nx=(center_x - width * 0.5) / (width * 0.5),
        ny=(center_y - height * 0.5) / (height * 0.5),
        area_fraction=area / float(width * height),
        radius=radius,
        center_x=center_x,
        center_y=center_y,
        circularity=circularity,
    )


def annotate_detection(
    image_bgr: np.ndarray, detection: Optional[DropTargetDetection]
) -> np.ndarray:
    """Create a debug image without mutating the source frame."""
    debug = image_bgr.copy()
    if detection is None:
        cv2.putText(
            debug,
            'TARGET: NONE',
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        return debug
    center = (round(detection.center_x), round(detection.center_y))
    cv2.circle(debug, center, round(detection.radius), (0, 255, 0), 2)
    cv2.drawMarker(debug, center, (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
    cv2.putText(
        debug,
        f'offset=({detection.nx:+.3f},{detection.ny:+.3f})',
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )
    return debug
