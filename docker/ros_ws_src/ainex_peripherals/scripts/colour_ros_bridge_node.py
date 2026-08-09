#!/usr/bin/env python3
"""Shared-memory colour → ROS /camera/image_raw bridge (Python 3.8, Docker).

Reads colour frames zero-copy from the /dev/shm double buffer written by
gemini305_bridge.py (host), driven by a tiny ZMQ "frame ready" notification on
port 5552 — no bulk frame data crosses the host/container boundary. Requires the
container to run with `--ipc=host` so this /dev/shm is the host's.

Notify wire format (single ZMQ part):
  struct.pack('QIHHBB', seq, idx, H, W, C, dtype_code)   # 18 bytes; dtype 0=uint8

Publishes:
  /camera/image_raw     (sensor_msgs/Image, bgr8)
  /camera/camera_info   (sensor_msgs/CameraInfo, from calibration YAML)

Note: camera_info_manager is C++-only in ROS Noetic; calibration YAML is loaded
directly with the stdlib yaml module.
"""
import os
import struct
import sys

import yaml

import rospy
import zmq
from sensor_msgs.msg import CameraInfo, Image

# shm_frame.py lives on the shared bind mount (host /home/pi/docker/tmp)
sys.path.insert(0, "/home/ubuntu/share/tmp")
from shm_frame import DoubleBuffer  # noqa: E402

COLOR_SHM  = "gemini_color"
NOTIFY_FMT = "QIHHBB"


def load_camera_info(url):
    """Parse a ROS camera calibration YAML (file:// URL or plain path) into CameraInfo."""
    path = url.replace('file://', '')
    with open(path) as f:
        d = yaml.safe_load(f)
    msg = CameraInfo()
    msg.width            = d['image_width']
    msg.height           = d['image_height']
    msg.distortion_model = d['distortion_model']
    msg.D = d['distortion_coefficients']['data']
    msg.K = d['camera_matrix']['data']
    msg.R = d['rectification_matrix']['data']
    msg.P = d['projection_matrix']['data']
    return msg


def attach_with_retry(name):
    """Attach to the shm buffer, waiting for the producer to create it."""
    while not rospy.is_shutdown():
        try:
            return DoubleBuffer.attach(name)
        except (FileNotFoundError, ValueError) as e:
            rospy.loginfo_throttle(5.0, "colour_ros_bridge: waiting for /dev/shm/%s (%s)", name, e)
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
    rospy.init_node('colour_ros_bridge')

    camera_name  = rospy.get_param('~camera_name',    'camera')
    zmq_host     = rospy.get_param('~zmq_host',       '172.17.0.1')
    zmq_port     = rospy.get_param('~zmq_port',       5552)
    cam_info_url = rospy.get_param(
        '~camera_info_url',
        'file:///home/ubuntu/.ros/camera_info/head_camera.yaml',
    )

    img_pub  = rospy.Publisher(f'/{camera_name}/image_raw',   Image,      queue_size=1)
    info_pub = rospy.Publisher(f'/{camera_name}/camera_info', CameraInfo, queue_size=1)

    cam_info_template = load_camera_info(cam_info_url)

    db = attach_with_retry(COLOR_SHM)
    if db is None:
        return
    attached_ino = shm_inode(COLOR_SHM)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)  # CONNECT SUB — host PUB is the stable BIND server
    sub.setsockopt(zmq.RCVHWM, 2)
    sub.connect(f"tcp://{zmq_host}:{zmq_port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"")

    rospy.loginfo(
        "colour_ros_bridge: shm /dev/shm/%s  notify SUB → tcp://%s:%d → /%s/image_raw",
        COLOR_SHM, zmq_host, zmq_port, camera_name,
    )

    while not rospy.is_shutdown():
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
        cur_ino = shm_inode(COLOR_SHM)
        if cur_ino is not None and cur_ino != attached_ino:
            rospy.logwarn("colour_ros_bridge: producer recreated shm — re-attaching")
            db.close()
            db = attach_with_retry(COLOR_SHM)
            attached_ino = shm_inode(COLOR_SHM)
            continue

        # Zero-copy view → copy once into the ROS message via tobytes(); then verify
        # the producer didn't wrap onto this buffer mid-read (drop the frame if it did).
        data = db.view(idx).tobytes()
        if db.buf_seq(idx) != seq:
            continue

        now = rospy.Time.now()
        img_msg = Image()
        img_msg.header.stamp    = now
        img_msg.header.frame_id = camera_name
        img_msg.height   = H
        img_msg.width    = W
        img_msg.encoding = 'bgr8'
        img_msg.step     = W * 3
        img_msg.data     = data

        cam_info = cam_info_template
        cam_info.header.stamp    = now
        cam_info.header.frame_id = camera_name

        img_pub.publish(img_msg)
        info_pub.publish(cam_info)


if __name__ == '__main__':
    main()
