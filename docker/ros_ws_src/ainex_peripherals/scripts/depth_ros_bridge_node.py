#!/usr/bin/env python3
"""Shared-memory depth → ROS /camera/depth/image_raw + /camera/depth/camera_info bridge
(Python 3.8, Docker).

Reads depth frames zero-copy from the /dev/shm double buffer written by
gemini305_bridge.py (host), driven by a tiny ZMQ "frame ready" notification on
port 5553 — no bulk frame data crosses the host/container boundary. Requires the
container to run with `--ipc=host` so this /dev/shm is the host's.
Also connects SUB to port 5554 for periodic depth intrinsics JSON so that
CameraInfo can be published with matching stamp and frame_id every frame.

Notify wire format (single ZMQ part):
  struct.pack('QIHHBB', seq, idx, H, W, C, dtype_code)   # 18 bytes; dtype 1=uint16

Wire format (intrinsics, port 5554):
  Single JSON message (no multipart), fields:
    fx, fy, cx, cy, width, height,
    k1, k2, k3, k4, k5, k6, p1, p2
  These are rgb_intrinsic values because depth is D2C-aligned to colour space.

Publishes:
  /camera/depth/image_raw    (sensor_msgs/Image, 16UC1, millimetres)
  /camera/depth/camera_info  (sensor_msgs/CameraInfo, rational_polynomial)
"""
import json
import os
import struct
import sys

import rospy
import zmq
from sensor_msgs.msg import CameraInfo, Image

# shm_frame.py lives on the shared bind mount (host /home/pi/docker/tmp)
sys.path.insert(0, "/home/ubuntu/share/tmp")
from shm_frame import DoubleBuffer  # noqa: E402

DEPTH_SHM  = "gemini_depth"
NOTIFY_FMT = "QIHHBB"


def _build_camera_info(d: dict) -> CameraInfo:
    """Build a CameraInfo message from a depth intrinsics JSON dict.

    Uses rational_polynomial distortion model (8 coefficients: k1-k6, p1, p2).
    Intrinsics are rgb_intrinsic values because depth is D2C-aligned to colour space.
    """
    msg = CameraInfo()
    msg.width  = d['width']
    msg.height = d['height']
    msg.distortion_model = 'rational_polynomial'
    msg.D = [d['k1'], d['k2'], d['p1'], d['p2'],
             d['k3'], d['k4'], d['k5'], d['k6']]
    fx, fy, cx, cy = d['fx'], d['fy'], d['cx'], d['cy']
    msg.K = [fx, 0.0, cx,  0.0, fy, cy,  0.0, 0.0, 1.0]
    msg.R = [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
    msg.P = [fx, 0.0, cx, 0.0,  0.0, fy, cy, 0.0,  0.0, 0.0, 1.0, 0.0]
    return msg


def attach_with_retry(name):
    """Attach to the shm buffer, waiting for the producer to create it."""
    while not rospy.is_shutdown():
        try:
            return DoubleBuffer.attach(name)
        except (FileNotFoundError, ValueError) as e:
            rospy.loginfo_throttle(5.0, "depth_ros_bridge: waiting for /dev/shm/%s (%s)", name, e)
            rospy.sleep(0.2)
    return None


def shm_inode(name):
    """Current inode of /dev/shm/<name>, or None if the producer recreated/removed it.

    The producer unlinks + recreates the segment (new inode) on every restart; an
    inode change is how a long-lived consumer detects it must re-attach.
    """
    try:
        return os.stat(f"/dev/shm/{name}").st_ino
    except OSError:
        return None


def main():
    rospy.init_node('depth_ros_bridge')

    zmq_host        = rospy.get_param('~zmq_host',        '172.17.0.1')
    zmq_port        = rospy.get_param('~zmq_port',        5553)
    depth_info_port = rospy.get_param('~depth_info_port', 5554)
    frame_id        = rospy.get_param('~frame_id',        'camera_depth')

    img_pub  = rospy.Publisher('/camera/depth/image_raw',   Image,      queue_size=1)
    info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)

    db = attach_with_retry(DEPTH_SHM)
    if db is None:
        return
    attached_ino = shm_inode(DEPTH_SHM)

    ctx = zmq.Context()

    # Depth frame-ready notify: CONNECT SUB — host PUB is the stable BIND server
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 2)
    sub.connect(f"tcp://{zmq_host}:{zmq_port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"")

    # Intrinsics JSON: CONNECT SUB — published every ~10 s by gemini305_bridge
    intr_sub = ctx.socket(zmq.SUB)
    intr_sub.setsockopt(zmq.RCVHWM, 2)
    intr_sub.connect(f"tcp://{zmq_host}:{depth_info_port}")
    intr_sub.setsockopt(zmq.SUBSCRIBE, b"")

    rospy.loginfo(
        "depth_ros_bridge: shm /dev/shm/%s  notify SUB → tcp://%s:%d  intrinsics SUB → tcp://%s:%d",
        DEPTH_SHM, zmq_host, zmq_port, zmq_host, depth_info_port,
    )

    cam_info_template = None   # built once on first intrinsics message; reused each frame

    while not rospy.is_shutdown():
        # Check for updated intrinsics (non-blocking; published ~every 10 s by bridge).
        try:
            raw_intr = intr_sub.recv(zmq.NOBLOCK)
            cam_info_template = _build_camera_info(json.loads(raw_intr))
            rospy.loginfo("depth_ros_bridge: received depth CameraInfo (fx=%.1f)", cam_info_template.K[0])
        except zmq.Again:
            pass

        # Receive next frame-ready notify (non-blocking; sleep briefly if nothing ready).
        try:
            note = sub.recv(zmq.NOBLOCK)
        except zmq.Again:
            rospy.sleep(0.002)
            continue
        # Drain to the most recent notification (drop stale frame-ready notices).
        while True:
            try:
                note = sub.recv(zmq.NOBLOCK)
            except zmq.Again:
                break

        seq, idx, H, W, _C, _dt = struct.unpack(NOTIFY_FMT, note)

        # Producer restart recreates /dev/shm (new inode); our old mmap would be a
        # now-deleted segment whose seq can never match → every frame dropped. Detect
        # the inode swap and re-attach so we follow the live producer forever.
        cur_ino = shm_inode(DEPTH_SHM)
        if cur_ino is not None and cur_ino != attached_ino:
            rospy.logwarn("depth_ros_bridge: producer recreated shm — re-attaching")
            db.close()
            db = attach_with_retry(DEPTH_SHM)
            attached_ino = shm_inode(DEPTH_SHM)
            continue

        # Zero-copy view → copy once into the ROS message via tobytes(); then verify
        # the producer didn't wrap onto this buffer mid-read (drop the frame if it did).
        data = db.view(idx).tobytes()
        if db.buf_seq(idx) != seq:
            continue

        # Snapshot stamp once — Image and CameraInfo share the same header values.
        stamp = rospy.Time.now()

        img_msg = Image()
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = frame_id
        img_msg.height   = H
        img_msg.width    = W
        img_msg.encoding = '16UC1'
        img_msg.step     = W * 2
        img_msg.data     = data
        img_pub.publish(img_msg)

        if cam_info_template is not None:
            cam_info_template.header.stamp    = stamp
            cam_info_template.header.frame_id = frame_id
            info_pub.publish(cam_info_template)


if __name__ == '__main__':
    main()
