#!/usr/bin/env python3
"""Servo-state → ZMQ bridge (Python 3.8, Docker / ainex container).

Republishes the bus-servo state topic to the host over ZMQ so host-side processes
(e.g. gemini305_view_bridge.py, which tags camera frames by head-tilt servo 24) can
read servo positions without a ROS install or rosbridge on the host.

Mirrors the camera ZMQ bridges' addressing: the container CONNECTs out to the host
at 172.17.0.1 (works whether the container is host- or bridge-networked). Here the
direction is reversed vs the camera bridges — this node is the PRODUCER (PUB), and
the host binds the matching SUB.

Subscribes:
  /ros_robot_controller/bus_servo/state   (ros_robot_controller/SetBusServosPosition)

Publishes (ZMQ PUB, connect):
  tcp://172.17.0.1:5555   JSON string  {"ts": <float>, "pos": {"<id>": <position>, ...}}
"""
import json

import rospy
import zmq

from ros_robot_controller.msg import SetBusServosPosition

ZMQ_HOST = "172.17.0.1"   # host (docker0 gateway); reachable in either network mode
ZMQ_PORT = 5555


class ServoStateZmqBridge:
    def __init__(self):
        zmq_host = rospy.get_param("~zmq_host", ZMQ_HOST)
        zmq_port = rospy.get_param("~zmq_port", ZMQ_PORT)

        self._ctx = zmq.Context()
        # PUB connects OUT to the host-bound SUB (host = stable server).
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 2)
        self._sock.connect(f"tcp://{zmq_host}:{zmq_port}")

        rospy.Subscriber(
            "/ros_robot_controller/bus_servo/state",
            SetBusServosPosition,
            self._callback,
            queue_size=1,
        )
        rospy.loginfo(
            "servo_state_zmq_bridge: /ros_robot_controller/bus_servo/state → "
            "PUB tcp://%s:%d", zmq_host, zmq_port,
        )

    def _callback(self, msg):
        pos = {str(entry.id): int(entry.position) for entry in msg.position}
        payload = json.dumps({"ts": rospy.get_time(), "pos": pos})
        try:
            self._sock.send_string(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass  # no consumer connected yet — drop

    def close(self):
        self._sock.close()
        self._ctx.destroy()


def main():
    rospy.init_node("servo_state_zmq_bridge")
    bridge = ServoStateZmqBridge()
    try:
        rospy.spin()
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
