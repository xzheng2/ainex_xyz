#!/usr/bin/env python3
"""ROS camera topic frame viewer — measures FPS, no YOLO, no ZMQ."""
import os
import sys
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

IMAGE_TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/camera/image_raw"
SHOW_WINDOW = bool(os.environ.get("DISPLAY"))

cv_bridge   = CvBridge()
last_frame  = None
fps         = 0.0
frame_count = 0
t_prev      = None


def image_cb(msg):
    global last_frame, fps, frame_count, t_prev
    img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    now = time.time()
    if t_prev is not None:
        elapsed = now - t_prev
        fps = 0.7 * fps + 0.3 / elapsed if elapsed > 0 else fps
    t_prev = now
    frame_count += 1
    last_frame = img
    print("[{:04d}] {:.1f} FPS  {}x{}".format(
        frame_count, fps, img.shape[1], img.shape[0]))


def main():
    rospy.init_node("frame_viewer", anonymous=True)
    rospy.Subscriber(IMAGE_TOPIC, Image, image_cb, queue_size=1)
    rospy.loginfo("Subscribing to %s  display=%s", IMAGE_TOPIC, SHOW_WINDOW)

    if SHOW_WINDOW:
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if last_frame is not None:
                disp = last_frame.copy()
                cv2.putText(disp, "{:.1f} FPS".format(fps), (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow("frame viewer", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            rate.sleep()
        cv2.destroyAllWindows()
    else:
        rospy.spin()


if __name__ == "__main__":
    main()
