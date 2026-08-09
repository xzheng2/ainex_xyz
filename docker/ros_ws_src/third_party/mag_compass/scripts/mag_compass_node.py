#!/usr/bin/env python3
"""Standalone magnetic compass node for the Ainex humanoid.

Subscribes ONLY to the calibrated magnetometer topic (default
``/ros_robot_controller/mag``, ``sensor_msgs/MagneticField``) and publishes a
magnetic heading in degrees as ``std_msgs/Float64``. This deliberately does NOT
use the complementary-filter ``/imu`` orientation, so you can observe how the
magnetometer behaves *on its own* as a compass.

Accuracy note (read before trusting the numbers):
  * The flat (mag-only) heading ``atan2(my, mx)`` is correct only when the robot
    is level. A biped sways while walking, so the reading will jump under tilt.
  * Set ``~tilt_compensation:=true`` to additionally subscribe to an IMU topic
    (``~imu_topic``, default ``/imu``), read roll/pitch from its quaternion, and
    project the field back to horizontal. Use the on/off comparison to judge how
    much the robot's motion corrupts a pure magnetic heading.
  * Output is MAGNETIC heading, not true north. Calibrate ``~declination_deg``
    empirically: face a known direction and add the offset that zeroes it there.
  * The robot's own servo currents add dynamic magnetic noise that the static
    ``mag_calib.yaml`` cannot remove -- expect jitter during motion.
"""

import math

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import MagneticField, Imu


def quat_to_roll_pitch(x, y, z, w):
    """Roll and pitch (rad) from a quaternion. Yaw is intentionally ignored."""
    # roll (rotation about X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (rotation about Y)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))   # clamp for numerical safety
    pitch = math.asin(sinp)
    return roll, pitch


class MagCompass:
    def __init__(self):
        # --- parameters (all have defaults; mirror imu_odometry_node.py style) ---
        self.mag_topic         = rospy.get_param('~mag_topic', '/ros_robot_controller/mag')
        self.heading_topic     = rospy.get_param('~heading_topic', '~heading_deg')
        self.declination_deg   = rospy.get_param('~declination_deg', 0.0)
        self.invert            = rospy.get_param('~invert', False)
        self.tilt_compensation = rospy.get_param('~tilt_compensation', False)
        self.imu_topic         = rospy.get_param('~imu_topic', '/imu')
        self.log_throttle      = rospy.get_param('~log_throttle', 1.0)

        # latest roll/pitch from the IMU (only used when tilt_compensation is on)
        self.roll = 0.0
        self.pitch = 0.0
        self.have_orientation = False

        self.heading_pub = rospy.Publisher(self.heading_topic, Float64, queue_size=1)

        if self.tilt_compensation:
            rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=1)
            rospy.loginfo('mag_compass: tilt compensation ON, using IMU orientation from %s',
                          self.imu_topic)
        else:
            rospy.loginfo('mag_compass: tilt compensation OFF (pure magnetometer heading)')

        rospy.Subscriber(self.mag_topic, MagneticField, self.mag_cb, queue_size=1)
        rospy.loginfo('mag_compass: subscribed to %s, publishing heading on %s',
                      self.mag_topic, self.heading_pub.resolved_name)

    def imu_cb(self, msg):
        q = msg.orientation
        # complementary filter leaves orientation = (0,0,0,0) until it converges;
        # ignore the null quaternion so we don't feed garbage roll/pitch.
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return
        self.roll, self.pitch = quat_to_roll_pitch(q.x, q.y, q.z, q.w)
        self.have_orientation = True

    def mag_cb(self, msg):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        if self.tilt_compensation and self.have_orientation:
            # Standard tilt-compensated compass: rotate the field into the
            # horizontal plane using the IMU roll/pitch, then take the heading.
            cr, sr = math.cos(self.roll), math.sin(self.roll)
            cp, sp = math.cos(self.pitch), math.sin(self.pitch)
            xh = mx * cp + my * sr * sp + mz * cr * sp
            yh = my * cr - mz * sr
        else:
            xh, yh = mx, my

        heading = math.degrees(math.atan2(-yh, xh))
        if self.invert:
            heading = -heading
        heading += self.declination_deg
        heading %= 360.0   # normalize to [0, 360)

        self.heading_pub.publish(Float64(data=heading))
        rospy.loginfo_throttle(self.log_throttle, 'mag_compass heading: %6.1f deg', heading)


if __name__ == '__main__':
    rospy.init_node('mag_compass')
    MagCompass()
    rospy.spin()
