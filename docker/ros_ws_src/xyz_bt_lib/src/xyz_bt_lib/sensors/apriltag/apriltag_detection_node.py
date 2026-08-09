#!/usr/bin/env python3
"""AprilTag detection node — receives colour frames from gemini305_bridge via ZMQ.

Connects SUB to gemini305_bridge's BIND PUB on port 5552 (same wire format as
colour_ros_bridge_node.py):
  Part 0: struct.pack('HHB', H, W, C)  — C=3 for BGR
  Part 1: BGR uint8 bytes, H*W*3 bytes

Publishes:
  /apriltag/ids    (std_msgs/Int32MultiArray)  — IDs of all detected tags
  /apriltag/image  (sensor_msgs/Image)         — annotated BGR frame (optional)
"""
import struct

import cv2
import numpy as np
import zmq
import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from xyz_bt_lib.sensors.apriltag.apriltag_detection import AprilTagDetector


def main():
    rospy.init_node('apriltag_detection')
    zmq_host     = rospy.get_param('~zmq_host',     '172.17.0.1')
    zmq_port     = rospy.get_param('~zmq_port',     5552)
    publish_image = rospy.get_param('~publish_image', True)

    pub_ids = rospy.Publisher('/apriltag/ids',   Int32MultiArray, queue_size=1)
    pub_img = rospy.Publisher('/apriltag/image', Image,           queue_size=1)
    bridge  = CvBridge()
    detector = AprilTagDetector()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 2)
    sub.connect(f"tcp://{zmq_host}:{zmq_port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"")

    rospy.loginfo('apriltag_detection: CONNECT SUB → tcp://%s:%d', zmq_host, zmq_port)

    while not rospy.is_shutdown():
        try:
            hdr_bytes, img_bytes = sub.recv_multipart(zmq.NOBLOCK)
        except zmq.Again:
            rospy.sleep(0.002)
            continue

        H, W, _C = struct.unpack('HHB', hdr_bytes)
        frame = np.frombuffer(img_bytes, dtype=np.uint8).reshape(H, W, 3)

        results = detector.detect(frame)

        msg = Int32MultiArray()
        msg.data = [r['id'] for r in results]
        pub_ids.publish(msg)

        if publish_image:
            annotated = detector.draw(frame.copy(), results)
            pub_img.publish(bridge.cv2_to_imgmsg(annotated, 'bgr8'))

    sub.close()
    ctx.term()


if __name__ == '__main__':
    main()
