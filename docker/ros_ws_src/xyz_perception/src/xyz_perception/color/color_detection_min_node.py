#!/usr/bin/env python3
# encoding: utf-8
# Minimal color detection ROS node.
# Detects a single color and shape configured via ROS parameters.
# No runtime update services; config is fixed at startup.

import os
import math
import queue
import signal
import numpy as np
import rospy
from sensor_msgs.msg import Image
from ainex_interfaces.msg import ObjectInfo, ObjectsInfo
from ainex_sdk import common

from xyz_perception.color.color_detection_min import ColorDetectionMin, DEFAULT_LINE_ROI

LAB_CONFIG_PATH = '/home/ubuntu/software/lab_tool/lab_config.yaml'


class ColorDetectionMinNode:
    def __init__(self):
        rospy.init_node('color_detection_min_node', log_level=rospy.INFO)

        # --- Parameters ---
        self.detect_color = rospy.get_param('~detect_color', 'white')
        self.detect_type = rospy.get_param('~detect_type', 'circle')
        self.min_area = rospy.get_param('~min_area', 300)
        self.max_area = rospy.get_param('~max_area', 250000)
        self.enable_display = rospy.get_param('~enable_display', False)
        proc_w = rospy.get_param('~image_process_width', 160)
        proc_h = rospy.get_param('~image_process_height', 120)
        min_aspect_ratio = rospy.get_param('~min_aspect_ratio', 1.0)
        max_aspect_ratio = rospy.get_param('~max_aspect_ratio', 4.0)
        min_solidity = rospy.get_param('~min_solidity', 0.6)
        self._detection_hz = rospy.get_param('~detection_hz', 30)

        # line-only: three [y_min, y_max, x_min, x_max] bands in processing space
        line_roi = rospy.get_param('~line_roi', None)
        if line_roi is None:
            line_roi = DEFAULT_LINE_ROI
        elif len(line_roi) != 3 or any(len(b) != 4 for b in line_roi):
            rospy.logerr('ColorDetectionMinNode: line_roi must be 3 bands of '
                         '[y_min, y_max, x_min, x_max], got: %s', line_roi)
            rospy.signal_shutdown('bad line_roi')
            return

        # --- LAB config ---
        if not os.path.exists(LAB_CONFIG_PATH):
            rospy.logerr('ColorDetectionMinNode: LAB config not found: %s', LAB_CONFIG_PATH)
            rospy.signal_shutdown('missing lab config')
            return

        yaml_data = common.get_yaml_data(LAB_CONFIG_PATH)

        # ainex lab_config.yaml stores thresholds under lab.Mono
        if 'lab' in yaml_data and 'Mono' in yaml_data['lab']:
            lab_config = yaml_data['lab']['Mono']
        else:
            lab_config = yaml_data

        if self.detect_color not in lab_config:
            rospy.logerr(
                "ColorDetectionMinNode: color '%s' not in lab config. Available: %s",
                self.detect_color, list(lab_config.keys()),
            )
            rospy.signal_shutdown('unknown detect_color')
            return

        # --- Detector ---
        self.detector = ColorDetectionMin(
            lab_config=lab_config,
            detect_color=self.detect_color,
            detect_type=self.detect_type,
            min_area=self.min_area,
            max_area=self.max_area,
            image_process_size=(proc_w, proc_h),
            min_aspect_ratio=min_aspect_ratio,
            max_aspect_ratio=max_aspect_ratio,
            min_solidity=min_solidity,
            line_roi=line_roi,
        )

        # --- Camera topic ---
        camera = rospy.get_param('/camera')
        camera_name = camera['camera_name']
        image_topic = camera['image_topic']
        cam_topic = '/{}/{}'.format(camera_name, image_topic)

        # --- Publishers ---
        self.pub_objects = rospy.Publisher('/object/pixel_coords', ObjectsInfo, queue_size=1)
        self.pub_image = rospy.Publisher('~image_result', Image, queue_size=1)

        # --- Image queue (size=1: always process the latest frame) ---
        self.image_queue = queue.Queue(maxsize=1)
        self.running = True

        # --- Subscriber ---
        rospy.Subscriber(cam_topic, Image, self._image_callback, queue_size=1)

        signal.signal(signal.SIGINT, self._shutdown)

        common.loginfo('ColorDetectionMinNode started')
        common.loginfo('  detect_color : %s' % self.detect_color)
        common.loginfo('  detect_type  : %s' % self.detect_type)
        common.loginfo('  min_area     : %s' % self.min_area)
        common.loginfo('  max_area     : %s' % self.max_area)
        common.loginfo('  proc_size    : %dx%d' % (proc_w, proc_h))
        common.loginfo('  camera_topic : %s' % cam_topic)
        common.loginfo('  detection_hz : %s' % self._detection_hz)
        if self.detect_type == 'rect':
            common.loginfo('  min_aspect_ratio: %.1f' % min_aspect_ratio)
            common.loginfo('  max_aspect_ratio: %.1f' % max_aspect_ratio)
            common.loginfo('  min_solidity    : %.2f' % min_solidity)
        if self.detect_type == 'line':
            common.loginfo('  line_roi        : %s' % (list(line_roi),))

    # ------------------------------------------------------------------

    def _shutdown(self, signum, frame):
        self.running = False

    def _image_callback(self, ros_image):
        bgr = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data,
        ).copy()
        # Drop old frame if queue is full so we always process the latest
        if not self.image_queue.empty():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self.image_queue.put_nowait(bgr)
        except queue.Full:
            pass

    def run(self):
        rate = rospy.Rate(self._detection_hz)
        while self.running and not rospy.is_shutdown():
            try:
                bgr = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            annotated, result = self.detector.detect(bgr)

            # Log result at 1 Hz
            if result is not None:
                if self.detect_type == 'circle':
                    r = result.get('radius', 0)
                    pixels = int(math.pi * r * r)
                    detail = 'r=%d pixels=%d' % (r, pixels)
                else:
                    w = result.get('width', 0)
                    h = result.get('height', 0)
                    pixels = w * h
                    detail = 'w=%d h=%d angle=%d pixels=%d' % (
                        w, h, result.get('angle', 0), pixels)
                rospy.loginfo_throttle(1.0, '[%s/%s] detected  x=%d y=%d %s',
                    self.detect_color, self.detect_type,
                    result.get('x', 0), result.get('y', 0), detail)
            else:
                rospy.loginfo_throttle(1.0, '[%s/%s] not detected',
                    self.detect_color, self.detect_type)

            # Publish ObjectsInfo (empty data list when nothing detected)
            msg = ObjectsInfo()
            if result is not None:
                obj = ObjectInfo()
                obj.label = self.detect_color
                obj.type = self.detect_type
                obj.x = result.get('x', 0)
                obj.y = result.get('y', 0)
                obj.width = result.get('width', 0)
                obj.height = result.get('height', 0)
                obj.radius = result.get('radius', 0)
                obj.angle = result.get('angle', 0)
                msg.data.append(obj)
            self.pub_objects.publish(msg)

            # Publish annotated image — bgr8 to match /camera/image_raw encoding
            img_msg = Image()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.height       = annotated.shape[0]
            img_msg.width        = annotated.shape[1]
            img_msg.encoding     = 'bgr8'
            img_msg.step         = annotated.shape[1] * 3
            img_msg.data         = annotated.tobytes()
            self.pub_image.publish(img_msg)

            if self.enable_display:
                import cv2
                cv2.imshow('color_detection_min', annotated)
                cv2.waitKey(1)

            rate.sleep()

        if self.enable_display:
            import cv2
            cv2.destroyAllWindows()


if __name__ == '__main__':
    node = ColorDetectionMinNode()
    node.run()
