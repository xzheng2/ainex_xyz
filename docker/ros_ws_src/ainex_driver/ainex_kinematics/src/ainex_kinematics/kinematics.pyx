# kinematics.pyx
# cython: language_level=3
# Recovered from kinematics.so (ARM aarch64, Cython 0.29.14, Python 3.8)
# Original file: kinematics.pyx
# Classes: LegIK
#
# Leg joint ordering (returned by leg_ik methods):
#   [0] hip_yaw
#   [1] hip_roll
#   [2] hip_pitch
#   [3] knee
#   [4] ank_pitch
#   [5] ank_roll
#
# Default leg geometry (metres):
#   thigh_length  = 0.097  (hip to knee)
#   calf_length   = 0.089  (knee to ankle joint)
#   ankle_length  = 0.024  (ankle joint to foot contact)
#
# Default joint_axis_direction (14 elements, +1/-1, applied by servo controller):
#   [r_hip_yaw, r_hip_roll, r_hip_pitch, r_knee, r_ank_pitch, r_ank_roll,
#    l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee, l_ank_pitch, l_ank_roll,
#    r_sho_pitch, l_sho_pitch]
#   = [-1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1]

from libc.math cimport acos, atan2, sqrt, sin, cos, fabs


cdef class LegIK:
    """
    6-DOF analytical inverse kinematics for a humanoid leg.

    Leg topology (per side):
        hip_yaw → hip_roll → hip_pitch → knee → ank_pitch → ank_roll → foot

    The foot target (x, y, z) is expressed in the hip-joint frame:
        x = forward, y = lateral (left positive), z = vertical (up positive).
    Orientation (roll, pitch, yaw) is the desired foot orientation in world frame.
    All angles are in radians.
    """

    cdef public double thigh_length
    cdef public double calf_length
    cdef public double ankle_length
    cdef public list joint_axis_direction

    def __init__(self):
        self.thigh_length = 0.097
        self.calf_length  = 0.089
        self.ankle_length = 0.024
        self.joint_axis_direction = [-1, -1, -1, -1, 1, 1,
                                     -1, -1,  1,  1, -1, 1,
                                     -1, 1]

    # ------------------------------------------------------------------
    # Property accessors
    # ------------------------------------------------------------------

    def get_leg_length(self):
        """Return [thigh_length, calf_length, ankle_length]."""
        return [self.thigh_length, self.calf_length, self.ankle_length]

    def set_leg_length(self, double thigh, double calf, double ankle):
        """Set individual link lengths (metres)."""
        self.thigh_length = thigh
        self.calf_length  = calf
        self.ankle_length = ankle

    def get_joint_axis_direction(self):
        """Return the 14-element joint axis direction list."""
        return self.joint_axis_direction

    def set_joint_axis_direction(self, directions):
        """Set the 14-element joint axis direction list."""
        self.joint_axis_direction = list(directions)

    # ------------------------------------------------------------------
    # Core IK helpers (C-level)
    # ------------------------------------------------------------------

    cdef int _solve_2d(self,
                       double ankle_x, double ankle_y, double ankle_z,
                       double roll, double pitch, double yaw,
                       double* hip_yaw_out,
                       double* hip_roll_out,
                       double* hip_pitch_out,
                       double* knee_out,
                       double* ank_pitch_out,
                       double* ank_roll_out,
                       int left_sign):
        """
        Core 2-D IK solver shared by all three public methods.

        left_sign = +1  → left_leg_ik convention  (hip_roll = atan2(-ankle_y, -ankle_z))
        left_sign = -1  → leg_ik convention        (hip_roll = atan2(+ankle_y, -ankle_z))

        Returns 1 on success, 0 on failure (error message already printed).
        """
        cdef double thigh = self.thigh_length
        cdef double calf  = self.calf_length
        cdef double hip_roll, z_eff, D
        cdef double cos_knee, knee
        cdef double alpha, denom, cos_beta_hip, beta_hip
        cdef double hip_pitch, beta_ankle, ank_pitch, ank_roll

        hip_roll_out[0] = <double>left_sign * atan2(
            -<double>left_sign * ankle_y, -ankle_z)

        # Effective z in the sagittal plane (after hip_roll compensation)
        z_eff = -sqrt(ankle_y * ankle_y + ankle_z * ankle_z)

        D = sqrt(ankle_x * ankle_x + z_eff * z_eff)

        # Knee angle via law of cosines
        cos_knee = (D * D - thigh * thigh - calf * calf) / (2.0 * thigh * calf)
        if cos_knee > 1.0 or cos_knee < -1.0:
            print("knee error")
            return 0
        knee = acos(cos_knee)

        # Forward lean of the hip-to-ankle vector from the downward vertical
        alpha = atan2(ankle_x, -z_eff)

        # Angle at the hip vertex (law of cosines)
        denom = 2.0 * D * thigh
        if denom == 0.0:
            print("hip_pitch error")
            return 0
        cos_beta_hip = (D * D + thigh * thigh - calf * calf) / denom
        if cos_beta_hip > 1.0 or cos_beta_hip < -1.0:
            print("hip_pitch error")
            return 0
        beta_hip = acos(cos_beta_hip)

        hip_pitch = -(alpha + beta_hip)

        # Angle at the ankle vertex
        beta_ankle = knee - beta_hip

        # ank_pitch: left_leg_ik uses +sign, leg_ik uses -sign
        ank_pitch = <double>left_sign * (beta_ankle - alpha - pitch)

        # ank_roll always equals (left-convention hip_roll) + desired roll
        # = atan2(-ankle_y, -ankle_z) + roll  (independent of left_sign)
        ank_roll = atan2(-ankle_y, -ankle_z) + roll

        hip_yaw_out[0]   = -yaw
        hip_pitch_out[0] = hip_pitch
        knee_out[0]      = knee
        ank_pitch_out[0] = ank_pitch
        ank_roll_out[0]  = ank_roll
        return 1

    # ------------------------------------------------------------------
    # Public IK methods
    # ------------------------------------------------------------------

    def leg_ik(self, double x, double y, double z,
               double roll, double pitch, double yaw):
        """
        Generic leg IK (canonical joint-space convention).

        hip_roll  is positive when the foot is displaced to the positive-y side.
        ank_pitch is positive for plantarflexion (toe pointing down).
        """
        cdef double ankle_x, ankle_y, ankle_z
        cdef double hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll

        # Ankle position from foot contact + rotation by (roll, pitch)
        ankle_x = x + self.ankle_length * sin(pitch)
        ankle_y = y - self.ankle_length * sin(roll) * cos(pitch)
        ankle_z = z + self.ankle_length * cos(roll) * cos(pitch)

        if not self._solve_2d(ankle_x, ankle_y, ankle_z,
                              roll, pitch, yaw,
                              &hip_yaw, &hip_roll, &hip_pitch,
                              &knee, &ank_pitch, &ank_roll,
                              -1):
            return []

        return [hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll]

    def left_leg_ik(self, double x, double y, double z,
                    double roll, double pitch, double yaw):
        """
        Left-leg IK.

        hip_roll  is negative when the foot is displaced laterally outward.
        ank_pitch is positive for dorsiflexion (toe pointing up).
        Angles are in servo-command sign convention for the left leg.
        """
        cdef double ankle_x, ankle_y, ankle_z
        cdef double hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll

        ankle_x = x + self.ankle_length * sin(pitch)
        ankle_y = y - self.ankle_length * sin(roll) * cos(pitch)
        ankle_z = z + self.ankle_length * cos(roll) * cos(pitch)

        if not self._solve_2d(ankle_x, ankle_y, ankle_z,
                              roll, pitch, yaw,
                              &hip_yaw, &hip_roll, &hip_pitch,
                              &knee, &ank_pitch, &ank_roll,
                              1):
            return []

        return [hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll]

    def right_leg_ik(self, double x, double y, double z,
                     double roll, double pitch, double yaw):
        """
        Right-leg IK.

        Same as left_leg_ik with hip_pitch, knee, and ank_pitch negated to
        account for the mirrored joint mounting of the right leg.
        """
        cdef double ankle_x, ankle_y, ankle_z
        cdef double hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll

        ankle_x = x + self.ankle_length * sin(pitch)
        ankle_y = y - self.ankle_length * sin(roll) * cos(pitch)
        ankle_z = z + self.ankle_length * cos(roll) * cos(pitch)

        if not self._solve_2d(ankle_x, ankle_y, ankle_z,
                              roll, pitch, yaw,
                              &hip_yaw, &hip_roll, &hip_pitch,
                              &knee, &ank_pitch, &ank_roll,
                              1):
            return []

        return [hip_yaw, hip_roll, -hip_pitch, -knee, -ank_pitch, ank_roll]
