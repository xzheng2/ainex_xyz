#!/usr/bin/env python3
import cv2


class AprilTagDetector:
    def __init__(self, tag_family='DICT_APRILTAG_36H11'):
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, tag_family))
        parameters = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        results = []
        if ids is not None:
            for i, corner in enumerate(corners):
                pts = corner[0]
                results.append({
                    'id': int(ids[i][0]),
                    'center_x': float(pts[:, 0].mean()),
                    'center_y': float(pts[:, 1].mean()),
                    'corners': pts.tolist(),
                })
        return results

    def draw(self, frame, results):
        for r in results:
            cx, cy = int(r['center_x']), int(r['center_y'])
            pts = [[int(p[0]), int(p[1])] for p in r['corners']]
            for j in range(4):
                cv2.line(frame, pts[j], pts[(j + 1) % 4], (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{r['id']}", (cx - 20, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame
