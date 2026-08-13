#!/usr/bin/env python3
import json
import signal
import threading

import cv2
import zmq
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ainex_interfaces.msg import ObjectInfo, ObjectsInfo


class YoloDetectionNode:
    def __init__(self):
        rospy.init_node('yolo_detection_node', log_level=rospy.INFO)
        zmq_host    = rospy.get_param('~zmq_host', 'localhost')
        zmq_port    = rospy.get_param('~zmq_port', 5551)
        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')

        self.pub = rospy.Publisher('/yolo/detections', ObjectsInfo, queue_size=1)

        # Annotated image publisher — overlaid event-driven on each new detection
        # (not a fixed-rate timer), so fresh bboxes pair with the freshest frame and
        # stale boxes are never repainted onto newer frames.
        self._bridge        = CvBridge()
        self._frame_lock    = threading.Lock()
        self._latest_frame  = None  # (cv_img, header)
        self.image_pub = rospy.Publisher('/yolo/image_annotated', Image, queue_size=1)
        rospy.Subscriber(image_topic, Image, self._on_image)

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PULL)
        self.sock.setsockopt(zmq.RCVHWM, 2)
        self.sock.setsockopt(zmq.RCVTIMEO, 100)   # 100 ms poll interval
        self.sock.connect("tcp://{}:{}".format(zmq_host, zmq_port))

        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)
        rospy.loginfo('YoloDetectionNode ready — zmq tcp://{}:{}, image {}'.format(
            zmq_host, zmq_port, image_topic))

    def _shutdown(self, sig, frame):
        self.running = False

    def _on_image(self, msg):
        """Store latest raw camera frame for annotation on the next detection."""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        with self._frame_lock:
            self._latest_frame = (cv_img, msg.header)

    def _publish_annotated(self, bboxes):
        """Overlay bboxes on the latest frame and publish once (event-driven)."""
        with self._frame_lock:
            frame_data = self._latest_frame
        if frame_data is None:
            return
        cv_img, header = frame_data
        img = cv_img.copy()
        for x1, y1, x2, y2, label in bboxes:
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, label, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        try:
            out = self._bridge.cv2_to_imgmsg(img, 'bgr8')
            out.header = header
            self.image_pub.publish(out)
        except Exception:
            pass

    def run(self):
        while self.running and not rospy.is_shutdown():
            try:
                data = self.sock.recv_string()
            except zmq.Again:
                continue

            detections = json.loads(data)

            bboxes = []
            msg = ObjectsInfo()
            for d in detections:
                x1, y1, x2, y2 = d['bbox']
                bboxes.append([x1, y1, x2, y2, d['class']])
                obj = ObjectInfo()
                obj.label  = d['class']
                obj.type   = 'rect'
                obj.x      = (x1 + x2) // 2
                obj.y      = (y1 + y2) // 2
                obj.width  = x2 - x1
                obj.height = y2 - y1
                obj.radius = 0
                obj.angle  = 0
                msg.data.append(obj)
            self.pub.publish(msg)

            # Annotated frame republishes at detection rate, paired with the
            # freshest camera frame — no fixed-rate repaint of stale boxes.
            self._publish_annotated(bboxes)

        self.sock.close()
        self.ctx.term()


if __name__ == '__main__':
    node = YoloDetectionNode()
    node.run()
