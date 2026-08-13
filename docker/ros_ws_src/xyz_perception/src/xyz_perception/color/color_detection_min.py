#!/usr/bin/env python3
# encoding: utf-8
# Minimal color detection: LAB threshold + erosion/dilation + contour.
# Supports 'circle' and 'rect' detection only.
# Detects a single color and shape configured at construction time.

import cv2
import math
import numpy as np


class ColorDetectionMin:
    """Single-color, single-shape detector using LAB thresholding.

    CONFIG_DEFAULTS:
        detect_color     : str   = 'white'
        detect_type      : str   = 'circle'   ('circle' or 'rect')
        min_area         : int   = 300        (original image pixels)
        max_area         : int   = 10000      (original image pixels)
        image_process_size : tuple = (160, 120)   (proc_w, proc_h)

        # rect-only tuning:
        min_aspect_ratio : float = 1.0    (long_side/short_side; 1.0 = any incl. square)
        max_aspect_ratio : float = 10.0   (rejects very thin slivers)
        min_solidity     : float = 0.7    (contour_area/hull_area; raise for clean targets)
        #
        # cv2.minAreaRect angle convention: [-90, 0)
        #   0 or -90 = axis-aligned;  ~-45 = 45-degree tilt
        # Tuning guide:
        #   min_aspect_ratio  raise (e.g. 1.5) to reject near-square blobs
        #   max_aspect_ratio  lower (e.g. 5.0) to reject very thin slivers
        #   min_solidity      raise (e.g. 0.85) for solid/clean targets
        #                     lower (e.g. 0.5)  for partially occluded targets
    """

    def __init__(self, lab_config, detect_color='white', detect_type='circle',
                 min_area=300, max_area=10000, image_process_size=(160, 120),
                 min_aspect_ratio=1.0, max_aspect_ratio=10.0, min_solidity=0.7):
        """
        lab_config : dict
            Already-loaded color thresholds: {color: {'min': [...], 'max': [...]}}
        """
        self.lab_data = lab_config
        self.detect_color = detect_color
        self.detect_type = detect_type
        self.min_area = min_area
        self.max_area = max_area
        self.image_process_size = image_process_size  # (w, h)
        self._min_ar = min_aspect_ratio
        self._max_ar = max_aspect_ratio
        self._min_solidity = min_solidity
        self._kernel = np.ones((3, 3), np.uint8)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, bgr_image):
        """Run detection on one BGR frame.

        Returns
        -------
        annotated_bgr : np.ndarray
            Copy of bgr_image with result drawn on it.
        result : dict or None
            circle -> {'x', 'y', 'radius', 'width', 'height'}
            rect   -> {'x', 'y', 'width', 'height', 'angle'}
            None if nothing detected or color not in lab_data.
        """
        img_h, img_w = bgr_image.shape[:2]
        image_draw = bgr_image.copy()

        if self.detect_color not in self.lab_data:
            return image_draw, None

        proc_w, proc_h = self.image_process_size
        scale_x = img_w / proc_w
        scale_y = img_h / proc_h
        # Area thresholds are in original-image space; convert to processing space
        area_scale = scale_x * scale_y

        # Blur → resize → LAB
        image_gb = cv2.GaussianBlur(bgr_image, (3, 3), 3)
        image_small = cv2.resize(image_gb, (proc_w, proc_h), interpolation=cv2.INTER_NEAREST)
        image_lab = cv2.cvtColor(image_small, cv2.COLOR_BGR2LAB)

        lower = tuple(self.lab_data[self.detect_color]['min'])
        upper = tuple(self.lab_data[self.detect_color]['max'])

        # LAB threshold + erode + dilate
        binary = cv2.inRange(image_lab, lower, upper)
        binary = cv2.erode(binary, self._kernel)
        binary = cv2.dilate(binary, self._kernel)

        # Find and filter contours
        raw_contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        scaled_min = self.min_area / area_scale
        scaled_max = self.max_area / area_scale

        filtered = []
        for c in raw_contours:
            a = math.fabs(cv2.contourArea(c))
            if scaled_min <= a <= scaled_max:
                filtered.append((a, c))
        filtered.sort(key=lambda t: t[0], reverse=True)

        if not filtered:
            return image_draw, None

        if self.detect_type == 'circle':
            return self._detect_circle(filtered, image_draw, scale_x, scale_y)
        else:
            return self._detect_rect(filtered, image_draw, scale_x, scale_y)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _remap(x, y, scale_x, scale_y):
        """Scale coordinates from processing space to original image space."""
        return int(x * scale_x), int(y * scale_y)

    def _detect_circle(self, area_contours, image_draw, scale_x, scale_y):
        """Return the first contour that passes circularity and area-ratio checks."""
        for _, contour in area_contours:
            area = math.fabs(cv2.contourArea(contour))
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * area / (perimeter ** 2)
            if circularity < 0.65:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            circle_area = math.pi * radius ** 2
            if circle_area == 0 or area / circle_area < 0.45:
                continue

            # Remap to original image coordinates
            ox, oy = self._remap(cx, cy, scale_x, scale_y)
            oradius = max(1, int(radius * scale_x))
            x1 = ox - oradius
            y1 = oy - oradius
            x2 = ox + oradius
            y2 = oy + oradius

            # Draw enclosing circle + bounding box + center dot
            cv2.circle(image_draw, (ox, oy), oradius, (0, 255, 255), 2)
            cv2.rectangle(image_draw, (x1, y1), (x2, y2), (0, 200, 255), 1)
            cv2.circle(image_draw, (ox, oy), 3, (0, 0, 255), -1)

            result = {
                'x': ox,
                'y': oy,
                'radius': oradius,
                'width': oradius * 2,
                'height': oradius * 2,
            }
            return image_draw, result

        return image_draw, None

    def _detect_rect(self, area_contours, image_draw, scale_x, scale_y):
        """Iterate candidates in area order; return first passing solidity + aspect ratio."""
        for _, contour in area_contours:
            # Solidity: contour_area / convex_hull_area
            hull = cv2.convexHull(contour)
            hull_area = math.fabs(cv2.contourArea(hull))
            if hull_area == 0:
                continue
            solidity = math.fabs(cv2.contourArea(contour)) / hull_area
            if solidity < self._min_solidity:
                continue

            rect = cv2.minAreaRect(contour)
            rw, rh = rect[1]
            if rw == 0 or rh == 0:
                continue
            long_side = max(rw, rh)
            short_side = min(rw, rh)
            aspect = long_side / short_side
            if not (self._min_ar <= aspect <= self._max_ar):
                continue

            # Passed all checks — remap and draw
            corners = np.int64(cv2.boxPoints(rect))
            orig_corners = np.array(
                [[int(corners[i][0] * scale_x), int(corners[i][1] * scale_y)] for i in range(4)],
                dtype=np.int64,
            )

            cx, cy = rect[0]
            angle = rect[2]
            ox, oy = self._remap(cx, cy, scale_x, scale_y)
            ow = int(long_side * scale_x)
            oh = int(short_side * scale_y)

            cv2.drawContours(image_draw, [orig_corners], -1, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(image_draw, (ox, oy), 3, (0, 0, 255), -1)

            result = {
                'x': ox,
                'y': oy,
                'width': ow,
                'height': oh,
                'angle': int(angle),
            }
            return image_draw, result

        return image_draw, None
