#!/usr/bin/env python3
"""IMU dead-reckoning odometry node for the Ainex humanoid.

Subscribes to the complementary-filtered ``/imu`` topic and integrates it into a
relative pose ``(x, y, yaw)`` published as ``nav_msgs/Odometry`` on ``/odom`` plus
an ``odom -> base_link`` TF.

Accuracy note (read before trusting the numbers):
  * yaw / turn  -> reliable. Taken straight from the fused quaternion that the
    complementary filter already produces (gyro integrated + accel/mag corrected),
    so there is no runaway drift.
  * x / y       -> best-effort only. Position comes from DOUBLE integration of the
    linear acceleration, whose error grows ~t^2 and reaches metres within seconds
    on a consumer IMU. A zero-velocity update (ZUPT) clamps velocity while the robot
    is still to delay drift, but it cannot remove it. Do NOT use x/y for long-range
    navigation without fusing gait commands or vision.
"""

import math

import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse

# Reuse the project's quaternion <-> rpy helpers (single source of truth).
from ainex_sdk.common import qua2rpy, rpy2qua


def qua2rot(x, y, z, w):
    """Rotation matrix mapping a body-frame vector into the world frame.

    Same form as ainex_peripherals/scripts/tf_broadcaster_imu.py:qua2rot.
    """
    return np.array(
        [[1.0 - 2 * (y * y + z * z), 2 * (x * y - w * z),       2 * (w * y + x * z)],
         [2 * (x * y + w * z),       1.0 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y),       2 * (y * z + w * x),       1.0 - 2 * (x * x + y * y)]])


class ImuOdometry:
    def __init__(self):
        # --- parameters (all have defaults; mirror imu.launch style) ----------
        self.imu_topic    = rospy.get_param('~imu_topic', '/imu')
        self.odom_frame   = rospy.get_param('~odom_frame', 'odom')
        self.base_frame   = rospy.get_param('~base_frame', 'base_link')
        self.publish_tf   = rospy.get_param('~publish_tf', True)
        self.gravity      = rospy.get_param('~gravity', 9.80665)
        self.accel_deadband = rospy.get_param('~accel_deadband', 0.3)   # m/s^2
        self.gyro_deadband  = rospy.get_param('~gyro_deadband', 0.05)   # rad/s
        self.use_orientation_for_yaw = rospy.get_param('~use_orientation_for_yaw', True)

        # --- integrated state -------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0          # used only in the angular-velocity fallback path
        self.vx = 0.0
        self.vy = 0.0
        self.last_stamp = None
        self._warned_raw = False

        # --- ROS I/O ----------------------------------------------------------
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Service('~reset', Empty, self._reset_srv)
        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=50)

        rospy.loginfo("imu_odometry: subscribing %s -> publishing /odom (publish_tf=%s)",
                      self.imu_topic, self.publish_tf)

    # ------------------------------------------------------------------ reset
    def _reset_srv(self, _req):
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = 0.0
        self.last_stamp = None
        rospy.loginfo("imu_odometry: pose reset to origin")
        return EmptyResponse()

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def _orientation_is_zero(o):
        return o.x == 0.0 and o.y == 0.0 and o.z == 0.0 and o.w == 0.0

    # -------------------------------------------------------------- imu callback
    def _imu_cb(self, msg):
        stamp = msg.header.stamp
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = (stamp - self.last_stamp).to_sec()
        self.last_stamp = stamp
        # Skip absurd / non-monotonic gaps (clock jumps, first frames).
        if dt <= 0.0 or dt > 0.5:
            return

        ori = msg.orientation
        have_orientation = (self.use_orientation_for_yaw
                            and not self._orientation_is_zero(ori))

        # --- yaw / turn -------------------------------------------------------
        if have_orientation:
            _, _, yaw = qua2rpy(ori)
        else:
            # Fallback: raw IMU has no orientation -> integrate gyro z.
            if not self._warned_raw:
                rospy.logwarn("imu_odometry: '%s' has zero orientation; integrating "
                              "angular_velocity.z instead. Subscribe to the filtered "
                              "/imu for accurate yaw.", self.imu_topic)
                self._warned_raw = True
            self.yaw += msg.angular_velocity.z * dt
            yaw = self.yaw

        # --- x / y dead-reckoning (best-effort) -------------------------------
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
        a_body = np.array([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z])

        if have_orientation:
            # Rotate body accel into world frame, then remove gravity.
            a_world = qua2rot(ori.x, ori.y, ori.z, ori.w).dot(a_body)
            a_world[2] -= self.gravity
            ax_w, ay_w = a_world[0], a_world[1]
        else:
            # Without orientation we cannot reliably project out gravity; use the
            # horizontal body axes directly (rough; yaw fallback path only).
            ax_w, ay_w = a_body[0], a_body[1]

        # Zero-velocity update: if the robot looks still, clamp velocity.
        if (math.hypot(ax_w, ay_w) < self.accel_deadband
                and np.linalg.norm(gyro) < self.gyro_deadband):
            self.vx = 0.0
            self.vy = 0.0
        else:
            self.vx += ax_w * dt
            self.vy += ay_w * dt
            self.x += self.vx * dt
            self.y += self.vy * dt

        self._publish(stamp, yaw, msg.angular_velocity.z)

    # ----------------------------------------------------------------- publish
    def _publish(self, stamp, yaw, yaw_rate):
        q = rpy2qua(0.0, 0.0, yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = yaw_rate
        # Covariance: x/y unreliable (large), yaw trustworthy (small).
        # Order is [x, y, z, roll, pitch, yaw] on the 6x6 diagonal.
        odom.pose.covariance[0]  = 1.0    # x
        odom.pose.covariance[7]  = 1.0    # y
        odom.pose.covariance[14] = 1e6    # z (unused)
        odom.pose.covariance[21] = 1e6    # roll (unused)
        odom.pose.covariance[28] = 1e6    # pitch (unused)
        odom.pose.covariance[35] = 0.05   # yaw
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('imu_odometry')
    ImuOdometry()
    rospy.spin()
