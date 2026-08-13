#!/usr/bin/env python3
# encoding: utf-8
# Minimal color detection: LAB threshold + erosion/dilation + contour.
# Supports 'circle', 'rect' and 'line' detection.
# Detects a single color and shape configured at construction time.

import cv2
import math
import numpy as np


#: Three scan bands for 'line' mode, as (y_min, y_max, x_min, x_max) in
#: processing space. Defaults assume the 160x120 processing size.
DEFAULT_LINE_ROI = (
    (20, 30, 40, 120),
    (40, 50, 40, 120),
    (60, 70, 40, 120),
)


class ColorDetectionMin:
    """Single-color, single-shape detector using LAB thresholding.

    CONFIG_DEFAULTS:
        detect_color     : str   = 'white'
        detect_type      : str   = 'circle'   ('circle', 'rect' or 'line')
        min_area         : int   = 300        (original image pixels)
        max_area         : int   = 10000      (original image pixels)
        image_process_size : tuple = (160, 120)   (proc_w, proc_h)

        # line-only tuning:
        line_roi         : tuple = DEFAULT_LINE_ROI
        #   Three (y_min, y_max, x_min, x_max) bands in PROCESSING space, ordered
        #   near-to-far as the camera sees them (up, center, down). Each band is
        #   thresholded on its own and the FIRST band with a hit wins — so the
        #   bands are a priority list, not a set to average. Widen a band to
        #   tolerate a wandering line; move them down the frame to look closer to
        #   the feet.

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
                 min_aspect_ratio=1.0, max_aspect_ratio=10.0, min_solidity=0.7,
                 line_roi=DEFAULT_LINE_ROI):
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
        self._line_roi = tuple(tuple(band) for band in line_roi)
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
            line   -> {'x', 'y', 'width', 'height', 'angle'}
            None if nothing detected or color not in lab_data.

        For 'line' the width/height describe the band segment that matched, not
        a whole object — consumers treat a line as having no meaningful area
        (ObjectDetectionAdapter writes size=None for it) and steer on x alone.
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

        scaled_min = self.min_area / area_scale
        scaled_max = self.max_area / area_scale

        # 'line' thresholds each band separately, so it cannot reuse a single
        # whole-frame mask — dispatch before doing that work.
        if self.detect_type == 'line':
            return self._detect_line(image_lab, image_draw, lower, upper,
                                     scaled_min, scaled_max, scale_x, scale_y)

        # LAB threshold + erode + dilate
        binary = self._mask(image_lab, lower, upper)

        # Find and filter contours
        filtered = self._contours_by_area(binary, scaled_min, scaled_max)

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

    def _mask(self, image_lab, lower, upper):
        """LAB threshold, then erode+dilate to drop speckle."""
        binary = cv2.inRange(image_lab, lower, upper)
        binary = cv2.erode(binary, self._kernel)
        return cv2.dilate(binary, self._kernel)

    @staticmethod
    def _contours_by_area(binary, scaled_min, scaled_max):
        """Contours within the area window, largest first, as (area, contour)."""
        raw_contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        filtered = []
        for c in raw_contours:
            a = math.fabs(cv2.contourArea(c))
            if scaled_min <= a <= scaled_max:
                filtered.append((a, c))
        filtered.sort(key=lambda t: t[0], reverse=True)
        return filtered

    def _detect_line(self, image_lab, image_draw, lower, upper,
                     scaled_min, scaled_max, scale_x, scale_y):
        """Scan the three bands in order and return the first one that hits.

        Bands are a priority list: the earliest band with a qualifying contour
        decides the steering point and the rest are not examined. That is what
        makes the near/far ordering meaningful — put the band you most want to
        follow first.
        """
        proc_w, proc_h = self.image_process_size

        for y_min, y_max, x_min, x_max in self._line_roi:
            # Clamp so a mistuned ROI cannot silently produce an empty slice
            y0 = max(0, min(int(y_min), proc_h))
            y1 = max(0, min(int(y_max), proc_h))
            x0 = max(0, min(int(x_min), proc_w))
            x1 = max(0, min(int(x_max), proc_w))
            if y1 <= y0 or x1 <= x0:
                continue

            # Draw the band being searched — invaluable when tuning ROIs
            bx0, by0 = self._remap(x0, y0, scale_x, scale_y)
            bx1, by1 = self._remap(x1, y1, scale_x, scale_y)
            cv2.rectangle(image_draw, (bx0, by0), (bx1, by1), (255, 200, 0), 1)

            band = image_lab[y0:y1, x0:x1]
            filtered = self._contours_by_area(
                self._mask(band, lower, upper), scaled_min, scaled_max)
            if not filtered:
                continue

            _, contour = filtered[0]
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), angle = rect
            if rw == 0 or rh == 0:
                continue

            # Band coordinates are relative to the crop — shift back into
            # processing space before remapping to the original image
            ox, oy = self._remap(cx + x0, cy + y0, scale_x, scale_y)

            corners = cv2.boxPoints(rect)
            orig_corners = np.array(
                [[int((px + x0) * scale_x), int((py + y0) * scale_y)]
                 for px, py in corners],
                dtype=np.int64,
            )
            cv2.drawContours(image_draw, [orig_corners], -1, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(image_draw, (ox, oy), 3, (0, 0, 255), -1)

            return image_draw, {
                'x': ox,
                'y': oy,
                'width': int(max(rw, rh) * scale_x),
                'height': int(min(rw, rh) * scale_y),
                'angle': int(angle),
            }

        return image_draw, None

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
