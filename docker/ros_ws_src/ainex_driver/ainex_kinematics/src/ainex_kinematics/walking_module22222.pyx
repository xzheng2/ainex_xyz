# walking_module.pyx
# cython: language_level=3
# Recovered from walking_module.so (ARM aarch64, Cython 0.29.x, Python 3.8)
# Original file: walking_module_without_balance.pyx
# Classes: JointPosition, WalkingModule
#
# Merge basis: walking_module_without_balance.pyx (File B, 65% accuracy)
# Targeted fixes applied from ground-truth DWARF / symbol analysis (md doc):
#   - radians/degrees: cpdef + correct parameter names (DWARF confirmed)
#   - wSin: cpdef double + radians-form formula
#   - walk_finish / init_pose_finish: cpdef bint + correct logic
#   - get_time_: cpdef double, returns self.time_
#   - __init__ param: ik -> leg_ik
#   - init_pose_tra: add present_joint_state param + print('move_to_init')
#   - IK calls: leg_ik.leg_ik() -> right_leg_ik() / left_leg_ik()
#   - import numpy as np  (confirmed by __pyx_k_np)
#   - All internal _ fields are cdef public (pickle checksum requirement)
#   - Method order matches __pyx_pw_ numbering from original .so
#
# Ground-truth DWARF confirms:
#   radians(double degrees)   — argument named "degrees"
#   degrees(double randians)  — argument named "randians"
#
# Walking state machine states (confirmed by __pyx_k_* constants):
#   'WalkingEnable', 'WalkingInitPose', 'WalkingReady'
#
# Joint ordering (14 controlled joints, 0-indexed):
#   [0]  r_hip_yaw   [1]  r_hip_roll  [2]  r_hip_pitch
#   [3]  r_knee      [4]  r_ank_pitch [5]  r_ank_roll
#   [6]  l_hip_yaw   [7]  l_hip_roll  [8]  l_hip_pitch
#   [9]  l_knee      [10] l_ank_pitch [11] l_ank_roll
#   [12] r_sho_pitch [13] l_sho_pitch

import numpy as np
from libc.math cimport sin, cos, sqrt, fabs, acos, atan2, floor, M_PI as pi


# ---------------------------------------------------------------------------
# Module-level helpers
# DWARF confirms: radians arg = 'degrees', degrees arg = 'randians'
# __pyx_f_ symbols confirm: both are cpdef (C-level callable)
# ---------------------------------------------------------------------------

cpdef double radians(double degrees):
    """Convert degrees to radians."""
    return degrees * pi / 180.0


cpdef double degrees(double randians):
    """Convert radians to degrees."""
    return randians * 180.0 / pi


# ---------------------------------------------------------------------------
# JointPosition
# Pickle checksum 0xf4c6245 confirms goal_position and present_position
# are cdef public double (scalar), NOT object/numpy.
# ---------------------------------------------------------------------------

cdef class JointPosition:
    """Holds present and goal position (radians) for a single joint."""

    cdef public double goal_position
    cdef public double present_position

    def __init__(self):
        self.goal_position    = 0.0
        self.present_position = 0.0


# ---------------------------------------------------------------------------
# WalkingModule
# Pickle checksum 0x8ef86ba confirms all 69 cdef fields listed below.
# All internal _ fields are cdef public (required by pickle protocol).
# ---------------------------------------------------------------------------

cdef class WalkingModule:
    """
    Sinusoidal open-loop walking engine for Ainex humanoid robot.

    Implements the Robotis OP2/DARwIn gait algorithm without balance
    (no IMU feedback). All computations are in SI units (metres, radians,
    seconds) unless noted.

    Parameters
    ----------
    leg_ik : LegIK
        Leg inverse-kinematics instance (from kinematics module).
    walking_param : dict
        Gait parameters (period_time in ms, amplitudes in m/deg).
    joint_index : dict
        Mapping joint_name -> list index, e.g. {'r_hip_yaw': 0, ...}.
    init_position : list
        14-element list of initial joint angles in radians.
    smp_time : double
        Trajectory / control time step in seconds (e.g. 0.008).
    """

    # ------------------------------------------------------------------ #
    # Python-object / state attributes                                     #
    # ------------------------------------------------------------------ #
    cdef public object leg_ik
    cdef public object init_pose_count
    cdef public object real_running
    cdef public object ctrl_running
    cdef public object DEBUG
    cdef public object walking_state
    cdef public object calc_joint_tra
    cdef public double time_
    cdef public double time_unit
    cdef public double smp_time
    cdef public double leg_length
    cdef public object goal_position
    cdef public object present_position
    cdef public object init_position
    cdef public object leg_data
    cdef public object joint_axis_direction
    cdef public object walking_param
    cdef public object joint_index
    cdef public object predict_joint_state

    # ------------------------------------------------------------------ #
    # Pose offset parameters (metres / radians)                            #
    # ------------------------------------------------------------------ #
    cdef public double x_offset_
    cdef public double y_offset_
    cdef public double z_offset_
    cdef public double r_offset_
    cdef public double p_offset_
    cdef public double a_offset_
    cdef public double hit_pitch_offset_

    # ------------------------------------------------------------------ #
    # Movement state                                                        #
    # ------------------------------------------------------------------ #
    cdef public double previous_x_move_amplitude_
    cdef public double y_move_amplitude_shift_
    cdef public double arm_swing_gain_

    # ------------------------------------------------------------------ #
    # Phase shift constants (radians)                                       #
    # wSin formula: mag * sin(2π * t/T - phase_shift) + mag_shift          #
    # ------------------------------------------------------------------ #
    cdef public double x_swap_phase_shift_
    cdef public double x_swap_amplitude_shift_
    cdef public double x_move_phase_shift_
    cdef public double x_move_amplitude_shift_
    cdef public double y_swap_phase_shift_
    cdef public double y_swap_amplitude_shift_
    cdef public double y_move_phase_shift_
    cdef public double z_swap_phase_shift_
    cdef public double z_move_phase_shift_
    cdef public double a_move_phase_shift_

    # ------------------------------------------------------------------ #
    # Period / timing (seconds)                                             #
    # ------------------------------------------------------------------ #
    cdef public double period_time_
    cdef public double dsp_ratio_
    cdef public double ssp_ratio_
    cdef public double x_swap_period_time_
    cdef public double x_move_period_time_
    cdef public double y_swap_period_time_
    cdef public double y_move_period_time_
    cdef public double z_swap_period_time_
    cdef public double z_move_period_time_
    cdef public double a_move_period_time_
    cdef public double ssp_time_
    cdef public double l_ssp_start_time_
    cdef public double l_ssp_end_time_
    cdef public double r_ssp_start_time_
    cdef public double r_ssp_end_time_
    cdef public double phase1_time_
    cdef public double phase2_time_
    cdef public double phase3_time_

    # ------------------------------------------------------------------ #
    # Pelvis compensation                                                   #
    # ------------------------------------------------------------------ #
    cdef public double pelvis_offset_
    cdef public double pelvis_swing_

    # ------------------------------------------------------------------ #
    # Motion amplitudes (metres / radians)                                  #
    # ------------------------------------------------------------------ #
    cdef public double x_move_amplitude_
    cdef public double x_swap_amplitude_
    cdef public double y_move_amplitude_
    cdef public double y_swap_amplitude_
    cdef public double z_move_amplitude_
    cdef public double z_swap_amplitude_
    cdef public double z_move_amplitude_shift_
    cdef public double z_swap_amplitude_shift_
    cdef public double a_move_amplitude_
    cdef public double a_move_amplitude_shift_

    # ================================================================== #

    def __init__(self, leg_ik, walking_param, joint_index, init_position, smp_time,
                 control_cycle=None):
        # leg_ik: DWARF confirms field name is 'leg_ik' (public, not _leg_ik)
        self.leg_ik        = leg_ik
        self.walking_param = dict(walking_param)
        self.joint_index   = dict(joint_index)
        self.init_position = list(init_position)
        # control_cycle, if provided, overrides smp_time as trajectory step
        self.smp_time      = control_cycle if control_cycle is not None else smp_time
        self.time_unit     = self.smp_time

        self.real_running  = False
        self.ctrl_running  = False
        self.DEBUG         = False
        self.walking_state = 'WalkingEnable'
        self.calc_joint_tra = None
        self.init_pose_count = 0

        self.time_                       = 0.0
        self.previous_x_move_amplitude_  = 0.0

        # Retrieve leg geometry from IK object
        leg_data = leg_ik.get_leg_length()
        self.leg_data    = leg_data
        self.leg_length  = leg_data[0] + leg_data[1] + leg_data[2]
        self.joint_axis_direction = list(leg_ik.get_joint_axis_direction())

        # Initialise per-joint position dicts
        # Seed goal_position and present_position from init_position so that
        # init_pose_tra() (called below) starts its trajectory from the robot's
        # actual calibration positions, NOT from zero.
        # Without this, calc_joint_tra[i][0] = 0 for every joint, causing the
        # robot to snap all joints to 0 on the first run() call.
        self.goal_position        = {}
        self.present_position     = {}
        self.predict_joint_state  = {}
        # Build sorted joint-name list once for the seed loop
        _seed_names = [''] * 14
        for _jn, _idx in joint_index.items():
            _seed_names[_idx] = _jn
        for joint_name in self.joint_index:
            _i = self.joint_index[joint_name]
            _jp_goal = JointPosition()
            _jp_goal.goal_position    = self.init_position[_i]
            _jp_goal.present_position = self.init_position[_i]
            self.goal_position[joint_name] = _jp_goal

            _jp_pres = JointPosition()
            _jp_pres.goal_position    = self.init_position[_i]
            _jp_pres.present_position = self.init_position[_i]
            self.present_position[joint_name] = _jp_pres

            self.predict_joint_state[joint_name] = JointPosition()

        # Load all parameter groups
        self.update_time_param()
        self.update_movement_param()
        self.update_pose_param()

        # Phase shift constants (fixed at construction; updated on param change)
        self.x_swap_phase_shift_     = pi
        self.x_swap_amplitude_shift_ = 0.0
        self.x_move_phase_shift_     = pi / 2.0
        self.x_move_amplitude_shift_ = self.x_move_amplitude_
        self.y_swap_phase_shift_     = 0.0
        self.y_swap_amplitude_shift_ = walking_param['y_swap_amplitude']
        self.y_move_phase_shift_     = pi / 2.0
        self.z_swap_phase_shift_     = pi
        self.z_move_phase_shift_     = pi / 2.0
        self.a_move_phase_shift_     = pi / 2.0

        # Build init-pose trajectory so the module is ready to move immediately
        self.init_pose_tra()

    # ================================================================== #
    # Parameter update helpers                                             #
    # Method order: set_walking_param(2), get_walking_param(4),            #
    #               update_time_param(6), update_movement_param(8),        #
    #               update_pose_param(10)  — per __pyx_pw_ numbering       #
    # ================================================================== #

    cpdef set_walking_param(self, walking_param):
        """Replace the walking parameter dict and recompute all derived values."""
        self.walking_param = dict(walking_param)
        self.update_time_param()
        self.update_movement_param()
        self.update_pose_param()
        # Update phase shift constants that depend on movement params
        self.x_move_amplitude_shift_  = self.x_move_amplitude_
        self.y_swap_amplitude_shift_  = walking_param['y_swap_amplitude']

    cpdef get_walking_param(self):
        """Return a copy of the current walking parameter dict."""
        return dict(self.walking_param)

    cpdef update_time_param(self):
        """
        Recompute all timing variables from walking_param.

        period_time in the dict is in milliseconds; all internal _ fields
        are in seconds.  phase1/2/3_time_ use the midpoint formula from
        the Robotis DARwIn original algorithm.
        """
        cdef double pt, dsp, ssp, ssp_t

        pt  = self.walking_param['period_time'] / 1000.0   # ms → s
        dsp = self.walking_param['dsp_ratio']
        ssp = 1.0 - dsp

        self.period_time_ = pt
        self.dsp_ratio_   = dsp
        self.ssp_ratio_   = ssp

        self.x_swap_period_time_ = pt / 2.0
        self.x_move_period_time_ = pt * ssp
        self.y_swap_period_time_ = pt
        self.y_move_period_time_ = pt * ssp
        self.z_swap_period_time_ = pt / 2.0
        self.z_move_period_time_ = pt * ssp / 2.0
        self.a_move_period_time_ = pt * ssp

        ssp_t            = pt * ssp
        self.ssp_time_   = ssp_t

        self.l_ssp_start_time_ = (pt - ssp_t) / 4.0
        self.l_ssp_end_time_   = self.l_ssp_start_time_ + ssp_t
        self.r_ssp_start_time_ = pt / 2.0 + self.l_ssp_start_time_
        self.r_ssp_end_time_   = self.r_ssp_start_time_ + ssp_t

        # Midpoint formula (Robotis DARwIn original)
        self.phase1_time_ = (self.l_ssp_start_time_ + self.l_ssp_end_time_)   / 2.0
        self.phase2_time_ = (self.l_ssp_end_time_   + self.r_ssp_start_time_) / 2.0
        self.phase3_time_ = (self.r_ssp_start_time_ + self.r_ssp_end_time_)   / 2.0

    cpdef update_movement_param(self):
        """
        Recompute motion amplitudes from walking_param.

        move_aim_on affects the y lateral shift convention:
          False (normal): both feet swing symmetrically around zero
          True  (aim):    shift is absorbed into amplitude_shift
        """
        cdef double y_amp

        self.x_move_amplitude_ = self.walking_param['x_move_amplitude']
        self.x_swap_amplitude_ = (self.walking_param['x_move_amplitude']
                                  * self.walking_param['step_fb_ratio'])

        y_amp = self.walking_param['y_move_amplitude'] / 2.0
        if self.walking_param['move_aim_on']:
            self.y_move_amplitude_shift_ = y_amp
            self.y_move_amplitude_       = -y_amp
        else:
            self.y_move_amplitude_       = y_amp
            self.y_move_amplitude_shift_ = 0.0

        self.z_move_amplitude_       = self.walking_param['z_move_amplitude'] / 2.0
        self.z_swap_amplitude_       = self.walking_param['z_swap_amplitude']
        self.z_move_amplitude_shift_ = self.z_move_amplitude_
        self.z_swap_amplitude_shift_ = self.z_swap_amplitude_

        self.a_move_amplitude_       = radians(self.walking_param['angle_move_amplitude']) / 2.0
        self.a_move_amplitude_shift_ = 0.0

        self.arm_swing_gain_         = self.walking_param['arm_swing_gain']

    cpdef update_pose_param(self):
        """
        Recompute pose-offset parameters from walking_param.

        All angular offsets in the YAML are in degrees; convert to radians here.
        """
        self.x_offset_         = self.walking_param['init_x_offset']
        self.y_offset_         = self.walking_param['init_y_offset']
        self.z_offset_         = self.walking_param['init_z_offset']
        self.r_offset_         = radians(self.walking_param['init_roll_offset'])
        self.p_offset_         = radians(self.walking_param['init_pitch_offset'])
        self.a_offset_         = radians(self.walking_param['init_yaw_offset'])
        self.hit_pitch_offset_ = radians(self.walking_param['hip_pitch_offset'])
        self.pelvis_offset_    = radians(self.walking_param['pelvis_offset'])
        self.pelvis_swing_     = self.pelvis_offset_ * 0.35
        self.y_swap_amplitude_ = self.walking_param['y_swap_amplitude']

    # ================================================================== #
    # Walking state machine                                                #
    # ================================================================== #

    cpdef start(self):
        """Enable walking (begin sinusoidal gait)."""
        self.ctrl_running  = True
        self.real_running  = True
        self.walking_state = 'WalkingEnable'

    cpdef stop(self):
        """Request graceful stop (finish current gait period then halt)."""
        self.ctrl_running = False

    cpdef process_phase(self):
        """
        Advance time_ by one control step and manage walking-state transitions.

        State machine:
          WalkingInitPose  — not entered here; handled in run()
          WalkingEnable    — active walking; on period wrap, check ctrl_running
          WalkingReady     — stopping in progress; one more period then halts
        """
        cdef double t, pt

        t  = self.time_
        pt = self.period_time_

        t += self.time_unit
        if t >= pt:
            t = 0.0
            if not self.ctrl_running:
                if self.walking_state == 'WalkingEnable':
                    self.walking_state = 'WalkingReady'
                elif self.walking_state == 'WalkingReady':
                    self.real_running  = False
                    self.walking_state = 'WalkingEnable'

        self.time_ = t

    # ================================================================== #
    # Core motion primitives                                               #
    # ================================================================== #

    cpdef cal_tra(self, double pos_start, double vel_start, double accel_start,
                double pos_end,   double vel_end,   double accel_end,
                double smp_time,  double mov_time):
        """
        Generate a minimum-jerk (5th-order polynomial) scalar trajectory.

        Argument order confirmed by __pyx_k_* string constants in original .so:
            pos_start, vel_start, accel_start,
            pos_end,   vel_end,   accel_end,
            smp_time,  mov_time

        Original .so confirmed (via __pyx_v_ symbols):
          - poly_matrix / poly_vector / poly_coeff  → numpy linalg solve
          - np.zeros(n+1)                           → returns numpy array
          - n = round(mov_time / smp_time)          → uses round(), NOT int()

        Using round() is critical: int(1.0/0.008) = 124 (float rounding error),
        round(1.0/0.008) = 125 → trajectory reaches the exact target position.

        Returns
        -------
        numpy.ndarray  — length round(mov_time / smp_time) + 1
        """
        cdef int    n, i
        cdef double T, T2, T3, T4, T5, t

        # MUST use round(): int(1.0/0.008) = 124 due to float rounding,
        # round(1.0/0.008) = 125; only round() guarantees the last point == pos_end
        n  = int(round(mov_time / smp_time))
        T  = mov_time
        T2 = T * T;  T3 = T2 * T;  T4 = T3 * T;  T5 = T4 * T

        # Original uses poly_matrix / poly_vector / poly_coeff (numpy linalg)
        # poly_matrix * [a3, a4, a5]^T = [D, W, Z]^T
        poly_matrix = np.array([
            [T3,      T4,       T5      ],
            [3.0*T2,  4.0*T3,   5.0*T4 ],
            [6.0*T,  12.0*T2,  20.0*T3 ],
        ])
        D  = pos_end   - pos_start   - vel_start   * T - accel_start * T2 * 0.5
        W  = vel_end   - vel_start   - accel_start * T
        Z  = accel_end - accel_start
        poly_vector = np.array([[D], [W], [Z]])
        poly_coeff  = np.dot(np.linalg.inv(poly_matrix), poly_vector).flatten()

        a3 = poly_coeff[0]
        a4 = poly_coeff[1]
        a5 = poly_coeff[2]

        # Returns numpy array (original uses np.zeros(n+1) + index assignment)
        minimum_jerk_tra = np.zeros(n + 1)
        for i in range(n + 1):
            t = <double>i * smp_time
            minimum_jerk_tra[i] = (pos_start + vel_start * t + accel_start * t * t * 0.5
                                   + a3 * t * t * t
                                   + a4 * t * t * t * t
                                   + a5 * t * t * t * t * t)
        return minimum_jerk_tra

    cpdef init_pose_tra(self, present_joint_state=None):
        """
        Compute minimum-jerk trajectories from each joint's current position
        to the upright standing pose derived from IK.

        The result is stored in self.calc_joint_tra (list of lists, one per
        joint in joint_index order).  self.init_pose_count is reset to 0.
        self.walking_state is set to 'WalkingInitPose'.

        Parameters
        ----------
        present_joint_state : dict or None
            Mapping joint_name -> JointPosition.  When None the present
            positions already stored in self.present_position are used.
            __pyx_kp_u_move_to_init confirms this method prints 'move_to_init'.
        """
        cdef double thigh, calf, ankle
        cdef double x_r, y_r, z_r, x_l, y_l, z_l
        cdef int    n_joints, idx
        cdef double mov_time

        print('move_to_init')

        if present_joint_state is not None:
            for jname in self.present_position:
                if jname in present_joint_state:
                    self.present_position[jname].present_position = (
                        present_joint_state[jname].present_position)
                    # Sync goal_position too: on the first run() frame after this
                    # call, delta = |goal - calc_joint_tra[i][0]|. Since
                    # calc_joint_tra[i][0] == present_position (the new start),
                    # if goal_position still holds a stale value the delta is
                    # spuriously large, which can trigger false "Cal Err" warnings
                    # or trip safety checks in the robot's control loop.
                    self.goal_position[jname].goal_position = (
                        present_joint_state[jname].present_position)

        thigh = self.leg_data[0]
        calf  = self.leg_data[1]
        ankle = self.leg_data[2]

        # Standing foot targets in hip frame (metres)
        x_r =  self.x_offset_
        y_r = -(self.y_offset_ - thigh * sin(self.hit_pitch_offset_))
        z_r = -(self.leg_length - self.z_offset_)

        x_l =  self.x_offset_
        y_l =  self.y_offset_ + thigh * sin(self.hit_pitch_offset_)
        z_l = -(self.leg_length - self.z_offset_)

        # IK for standing pose — confirmed method names right_leg_ik / left_leg_ik
        # pitch=0.0: command foot to be level (horizontal); the ankle compensates
        # for any body lean.  Do NOT pass p_offset_+hit_pitch_offset_ here —
        # that would double-count the lean and tilt the foot incorrectly.
        r_angles = self.leg_ik.right_leg_ik(x_r, y_r, z_r,
                                             self.r_offset_,
                                             0.0,
                                             self.a_offset_)
        l_angles = self.leg_ik.left_leg_ik(x_l, y_l, z_l,
                                            self.r_offset_,
                                            0.0,
                                            self.a_offset_)

        if not r_angles:
            print('Right IK not Solved: ')
            r_angles = [0.0] * 6
        if not l_angles:
            print('Left IK not Solved: ')
            l_angles = [0.0] * 6

        # Build 14-element target array in joint_index order
        # NOTE: IK already returns angles in servo goal_position convention —
        # joint_axis_direction must NOT be applied here (OP3 reference: target = IK_angle directly).
        target = [0.0] * 14
        for i in range(6):
            target[i]     = r_angles[i]
        for i in range(6):
            target[6 + i] = l_angles[i]
        target[12] = self.init_position[12]
        target[13] = self.init_position[13]

        mov_time   = 1.0
        n_joints   = 14
        # Build joint_names sorted by index — no lambda/comprehension (cpdef closure restriction)
        joint_names = [''] * n_joints
        for _jn, _idx in self.joint_index.items():
            joint_names[_idx] = _jn

        # Dynamic mov_time: proportional to the largest joint displacement,
        # minimum 1.0 s.  Matches zheng.pyx: mov_time = max(1.0, max_disp/1.0).
        cdef double max_disp, disp, start_angle
        max_disp = 0.0
        start_angles = []
        for i in range(n_joints):
            jname = joint_names[i]
            start_angle = (self.present_position[jname].present_position
                           if jname in self.present_position
                           else self.init_position[i])
            start_angles.append(start_angle)
            disp = fabs(target[i] - start_angle)
            if disp > max_disp:
                max_disp = disp
        if max_disp > 1.0:
            mov_time = max_disp

        print('Generate Trajecotry : {}s [{}]'.format(mov_time, n_joints))

        self.calc_joint_tra = []
        for i in range(n_joints):
            jname  = joint_names[i]
            start  = start_angles[i]
            end    = target[i]
            traj   = self.cal_tra(start, 0.0, 0.0, end, 0.0, 0.0,
                                  self.smp_time, mov_time)
            self.calc_joint_tra.append(traj)

        self.init_pose_count = 0
        self.walking_state   = 'WalkingInitPose'

    # ================================================================== #
    # State accessor methods  (cpdef confirmed by __pyx_f_ symbols)        #
    # ================================================================== #

    cpdef double get_time_(self):
        """Return the current time within the gait period (seconds)."""
        return self.time_

    # ================================================================== #
    # Sinusoidal helper  (cpdef confirmed by __pyx_f_ symbol)              #
    # Formula: radians form — mag * sin(2π * t/T - phase_shift) + shift    #
    # ================================================================== #

    cpdef double wSin(self, double time, double period, double period_shift,
                      double mag, double mag_shift):
        """
        Evaluate one period of a sin wave at *time*.

            wSin(t) = mag * sin(2π * t / period - period_shift) + mag_shift

        period_shift is in radians (e.g. pi, pi/2, 0).
        This is the Robotis DARwIn original formula; phase_shift values stored
        as radian constants (pi, pi/2, etc.) confirm this form.
        """
        return mag * sin(2.0 * pi * time / period - period_shift) + mag_shift

    # ================================================================== #
    # Leg and arm angle computation                                         #
    # ================================================================== #

    cpdef get_leg_angle(self):
        """
        Compute goal joint angles for all 12 leg joints using the sinusoidal
        gait engine and store them in self.goal_position.

        Implements 5-branch phase clamping: during double-support (DSP) the
        foot position is held fixed (clamped to the SSP boundary value).
        This is the Robotis DARwIn core algorithm; confirmed decisive win
        over the simple unclamped wSin approach.
        """
        cdef double time
        cdef double x_swap
        cdef double r_x_move, l_x_move
        cdef double y_swap
        cdef double r_y_move, l_y_move
        cdef double r_z_swap, l_z_swap, r_z_move, l_z_move
        cdef double r_a_move, l_a_move
        cdef double r_pelvis, l_pelvis
        cdef double r_x, r_y, r_z, r_roll, r_pitch, r_a
        cdef double l_x, l_y, l_z, l_roll, l_pitch, l_a

        time = self.time_

        # Local aliases for speed (avoids repeated attribute lookup)
        cdef double x_swap_period_time_     = self.x_swap_period_time_
        cdef double x_swap_phase_shift_     = self.x_swap_phase_shift_
        cdef double x_swap_amplitude_       = self.x_swap_amplitude_
        cdef double x_swap_amplitude_shift_ = self.x_swap_amplitude_shift_
        cdef double x_move_period_time_     = self.x_move_period_time_
        cdef double x_move_phase_shift_     = self.x_move_phase_shift_
        cdef double x_move_amplitude_       = self.x_move_amplitude_
        cdef double x_move_amplitude_shift_ = self.x_move_amplitude_shift_
        cdef double y_swap_period_time_     = self.y_swap_period_time_
        cdef double y_swap_phase_shift_     = self.y_swap_phase_shift_
        cdef double y_swap_amplitude_       = self.y_swap_amplitude_
        cdef double y_swap_amplitude_shift_ = self.y_swap_amplitude_shift_
        cdef double y_move_period_time_     = self.y_move_period_time_
        cdef double y_move_phase_shift_     = self.y_move_phase_shift_
        cdef double y_move_amplitude_       = self.y_move_amplitude_
        cdef double y_move_amplitude_shift_ = self.y_move_amplitude_shift_
        cdef double z_swap_period_time_     = self.z_swap_period_time_
        cdef double z_swap_phase_shift_     = self.z_swap_phase_shift_
        cdef double z_swap_amplitude_       = self.z_swap_amplitude_
        cdef double z_swap_amplitude_shift_ = self.z_swap_amplitude_shift_
        cdef double z_move_period_time_     = self.z_move_period_time_
        cdef double z_move_phase_shift_     = self.z_move_phase_shift_
        cdef double z_move_amplitude_       = self.z_move_amplitude_
        cdef double z_move_amplitude_shift_ = self.z_move_amplitude_shift_
        cdef double a_move_period_time_     = self.a_move_period_time_
        cdef double a_move_phase_shift_     = self.a_move_phase_shift_
        cdef double a_move_amplitude_       = self.a_move_amplitude_
        cdef double a_move_amplitude_shift_ = self.a_move_amplitude_shift_
        cdef double l_ssp_start_time_       = self.l_ssp_start_time_
        cdef double l_ssp_end_time_         = self.l_ssp_end_time_
        cdef double r_ssp_start_time_       = self.r_ssp_start_time_
        cdef double r_ssp_end_time_         = self.r_ssp_end_time_
        cdef double pelvis_offset_          = self.pelvis_offset_
        cdef double pelvis_swing_           = self.pelvis_swing_
        cdef double hit_pitch_offset_       = self.hit_pitch_offset_
        cdef double x_offset_               = self.x_offset_
        cdef double y_offset_               = self.y_offset_
        cdef double z_offset_               = self.z_offset_
        cdef double r_offset_               = self.r_offset_
        cdef double p_offset_               = self.p_offset_
        cdef double a_offset_               = self.a_offset_
        cdef double thigh                   = self.leg_data[0]

        # ---------------------------------------------------------- #
        # X (forward/backward) — 5-branch phase clamping             #
        # ---------------------------------------------------------- #
        x_swap = (x_swap_amplitude_
                  * sin(2.0 * pi * time / x_swap_period_time_
                        - x_swap_phase_shift_)
                  + x_swap_amplitude_shift_)

        if time < l_ssp_start_time_:
            r_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * l_ssp_start_time_ / x_move_period_time_
                              - x_move_phase_shift_)
                        * (-1.0) + x_move_amplitude_shift_)
            l_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * l_ssp_start_time_ / x_move_period_time_
                              - (x_move_phase_shift_ + pi))
                        + x_move_amplitude_shift_)
        elif time < l_ssp_end_time_:
            r_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * time / x_move_period_time_
                              - x_move_phase_shift_)
                        * (-1.0) + x_move_amplitude_shift_)
            l_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * time / x_move_period_time_
                              - (x_move_phase_shift_ + pi))
                        + x_move_amplitude_shift_)
        elif time < r_ssp_start_time_:
            r_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * l_ssp_end_time_ / x_move_period_time_
                              - x_move_phase_shift_)
                        * (-1.0) + x_move_amplitude_shift_)
            l_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * l_ssp_end_time_ / x_move_period_time_
                              - (x_move_phase_shift_ + pi))
                        + x_move_amplitude_shift_)
        elif time < r_ssp_end_time_:
            r_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * time / x_move_period_time_
                              - (x_move_phase_shift_ + pi))
                        * (-1.0) + x_move_amplitude_shift_)
            l_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * time / x_move_period_time_
                              - x_move_phase_shift_)
                        + x_move_amplitude_shift_)
        else:
            r_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * r_ssp_end_time_ / x_move_period_time_
                              - (x_move_phase_shift_ + pi))
                        * (-1.0) + x_move_amplitude_shift_)
            l_x_move = (x_move_amplitude_
                        * sin(2.0 * pi * r_ssp_end_time_ / x_move_period_time_
                              - x_move_phase_shift_)
                        + x_move_amplitude_shift_)

        # ---------------------------------------------------------- #
        # Y (lateral) — 5-branch phase clamping                      #
        # ---------------------------------------------------------- #
        y_swap = (y_swap_amplitude_
                  * sin(2.0 * pi * time / y_swap_period_time_
                        - y_swap_phase_shift_)
                  + y_swap_amplitude_shift_)

        if time < l_ssp_start_time_:
            r_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * l_ssp_start_time_ / y_move_period_time_
                              - y_move_phase_shift_)
                        * (-1.0) + y_move_amplitude_shift_)
            l_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * l_ssp_start_time_ / y_move_period_time_
                              - (y_move_phase_shift_ + pi))
                        + y_move_amplitude_shift_)
        elif time < l_ssp_end_time_:
            r_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * time / y_move_period_time_
                              - y_move_phase_shift_)
                        * (-1.0) + y_move_amplitude_shift_)
            l_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * time / y_move_period_time_
                              - (y_move_phase_shift_ + pi))
                        + y_move_amplitude_shift_)
        elif time < r_ssp_start_time_:
            r_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * l_ssp_end_time_ / y_move_period_time_
                              - y_move_phase_shift_)
                        * (-1.0) + y_move_amplitude_shift_)
            l_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * l_ssp_end_time_ / y_move_period_time_
                              - (y_move_phase_shift_ + pi))
                        + y_move_amplitude_shift_)
        elif time < r_ssp_end_time_:
            r_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * time / y_move_period_time_
                              - (y_move_phase_shift_ + pi))
                        * (-1.0) + y_move_amplitude_shift_)
            l_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * time / y_move_period_time_
                              - y_move_phase_shift_)
                        + y_move_amplitude_shift_)
        else:
            r_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * r_ssp_end_time_ / y_move_period_time_
                              - (y_move_phase_shift_ + pi))
                        * (-1.0) + y_move_amplitude_shift_)
            l_y_move = (y_move_amplitude_
                        * sin(2.0 * pi * r_ssp_end_time_ / y_move_period_time_
                              - y_move_phase_shift_)
                        + y_move_amplitude_shift_)

        # ---------------------------------------------------------- #
        # Z (height / lift) — full-period continuous sinusoid                #
        # Right and left feet are 180° out of phase (right peaks at T/2,     #
        # left peaks at t=0/T).  Matches zheng.pyx confirmed implementation. #
        # z_swap is shared body-COM bob (same for both legs).                 #
        # ---------------------------------------------------------- #
        r_z_swap = (z_swap_amplitude_
                    * sin(2.0 * pi * time / z_swap_period_time_
                          - z_swap_phase_shift_)
                    + z_swap_amplitude_shift_)
        l_z_swap = r_z_swap

        cdef double period_time_ = self.period_time_
        r_z_move = (z_move_amplitude_
                    * sin(2.0 * pi * time / period_time_ - pi)
                    + z_move_amplitude_shift_)
        l_z_move = (z_move_amplitude_
                    * sin(2.0 * pi * time / period_time_)
                    + z_move_amplitude_shift_)

        # ---------------------------------------------------------- #
        # A (yaw) — 5-branch phase clamping                          #
        # ---------------------------------------------------------- #
        if time < l_ssp_start_time_:
            r_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * l_ssp_start_time_ / a_move_period_time_
                              - (a_move_phase_shift_ + pi))
                        * (-1.0) + a_move_amplitude_shift_)
            l_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * l_ssp_start_time_ / a_move_period_time_
                              - a_move_phase_shift_)
                        + a_move_amplitude_shift_)
        elif time < l_ssp_end_time_:
            r_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * time / a_move_period_time_
                              - (a_move_phase_shift_ + pi))
                        * (-1.0) + a_move_amplitude_shift_)
            l_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * time / a_move_period_time_
                              - a_move_phase_shift_)
                        + a_move_amplitude_shift_)
        elif time < r_ssp_start_time_:
            r_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * l_ssp_end_time_ / a_move_period_time_
                              - (a_move_phase_shift_ + pi))
                        * (-1.0) + a_move_amplitude_shift_)
            l_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * l_ssp_end_time_ / a_move_period_time_
                              - a_move_phase_shift_)
                        + a_move_amplitude_shift_)
        elif time < r_ssp_end_time_:
            r_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * time / a_move_period_time_
                              - a_move_phase_shift_)
                        * (-1.0) + a_move_amplitude_shift_)
            l_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * time / a_move_period_time_
                              - (a_move_phase_shift_ + pi))
                        + a_move_amplitude_shift_)
        else:
            r_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * r_ssp_end_time_ / a_move_period_time_
                              - a_move_phase_shift_)
                        * (-1.0) + a_move_amplitude_shift_)
            l_a_move = (a_move_amplitude_
                        * sin(2.0 * pi * r_ssp_end_time_ / a_move_period_time_
                              - (a_move_phase_shift_ + pi))
                        + a_move_amplitude_shift_)

        # ---------------------------------------------------------- #
        # Pelvis roll compensation (during SSP only)                  #
        # ---------------------------------------------------------- #
        if time < l_ssp_start_time_ or time > r_ssp_end_time_:
            r_pelvis = 0.0
            l_pelvis = 0.0
        elif time < l_ssp_end_time_:
            r_pelvis = pelvis_swing_
            l_pelvis = pelvis_offset_
        elif time < r_ssp_start_time_:
            r_pelvis = 0.0
            l_pelvis = 0.0
        else:
            r_pelvis = pelvis_offset_
            l_pelvis = pelvis_swing_

        # ---------------------------------------------------------- #
        # Foot target positions in hip frame                          #
        # ---------------------------------------------------------- #
        r_x     =  x_offset_ + r_x_move + x_swap
        r_y     = -(y_offset_ - thigh * sin(hit_pitch_offset_) + r_y_move + y_swap)
        r_z     = -(self.leg_length - z_offset_ - r_z_swap - r_z_move)
        r_roll  =  r_offset_ - r_pelvis
        # pitch=0.0: foot commanded level (horizontal); ankle compensates body lean.
        # Passing p_offset_+hit_pitch_offset_ here double-counts the lean offset.
        r_pitch =  0.0
        r_a     =  a_offset_ + r_a_move

        l_x     =  x_offset_ + l_x_move + x_swap
        l_y     =  (y_offset_ + thigh * sin(hit_pitch_offset_) + l_y_move + y_swap)
        l_z     = -(self.leg_length - z_offset_ - l_z_swap - l_z_move)
        l_roll  =  r_offset_ + l_pelvis
        l_pitch =  0.0
        l_a     =  a_offset_ + l_a_move

        # ---------------------------------------------------------- #
        # IK — confirmed names: right_leg_ik / left_leg_ik           #
        # ---------------------------------------------------------- #
        r_angles = self.leg_ik.right_leg_ik(r_x, r_y, r_z, r_roll, r_pitch, r_a)
        l_angles = self.leg_ik.left_leg_ik( l_x, l_y, l_z, l_roll, l_pitch, l_a)

        if not r_angles:
            print('Right IK not Solved: ')
            return
        if not l_angles:
            print('Left IK not Solved: ')
            return

        # Store raw IK angles directly — IK already returns servo-convention angles.
        # joint_axis_direction must NOT be multiplied here (OP3 reference pattern).
        joint_names_r = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch',
                         'r_knee',    'r_ank_pitch', 'r_ank_roll']
        for i, jname in enumerate(joint_names_r):
            self.goal_position[jname].goal_position = r_angles[i]

        joint_names_l = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch',
                         'l_knee',    'l_ank_pitch', 'l_ank_roll']
        for i, jname in enumerate(joint_names_l):
            self.goal_position[jname].goal_position = l_angles[i]

    cpdef get_arm_angle(self):
        """
        Compute goal angles for the two shoulder-pitch joints and store in
        self.goal_position.  Arms swing in opposition to the legs.
        """
        cdef double r_sho_pitch, l_sho_pitch
        cdef double hip_pitch_r, hip_pitch_l

        hip_pitch_r = self.goal_position['r_hip_pitch'].goal_position
        hip_pitch_l = self.goal_position['l_hip_pitch'].goal_position

        r_sho_pitch = (-hip_pitch_r
                       - self.goal_position['r_knee'].goal_position / 2.0
                       ) * self.arm_swing_gain_
        l_sho_pitch = (-hip_pitch_l
                       - self.goal_position['l_knee'].goal_position / 2.0
                       ) * self.arm_swing_gain_

        self.goal_position['r_sho_pitch'].goal_position = (
            r_sho_pitch * self.joint_axis_direction[12]
            + self.init_position[12])
        self.goal_position['l_sho_pitch'].goal_position = (
            l_sho_pitch * self.joint_axis_direction[13]
            + self.init_position[13])

    # ================================================================== #
    # Completion status methods (cpdef, confirmed by __pyx_f_ symbols)    #
    # ================================================================== #

    cpdef bint init_pose_finish(self):
        """Return True when the init-pose trajectory has completed."""
        return self.walking_state != 'WalkingInitPose'

    cpdef bint walk_finish(self):
        """Return True once both ctrl_running and real_running are False."""
        return (not self.ctrl_running) and (not self.real_running)

    # ================================================================== #
    # Main control loop step                                               #
    # ================================================================== #

    cpdef run(self, present_joint_state):
        """
        Execute one control step and return updated joint goals.

        Parameters
        ----------
        present_joint_state : dict
            Mapping joint_name -> JointPosition (present_position field used).

        Returns
        -------
        tuple : (joint_max_move, time_, goal_position)
            joint_max_move : double  — max |Δangle| this step (degrees)
            time_          : double  — current time in gait period (s)
            goal_position  : dict    — joint_name -> JointPosition
        """
        cdef double joint_max_move, delta
        cdef int    count

        # Update present positions from sensor feedback
        for jname in self.present_position:
            if jname in present_joint_state:
                self.present_position[jname].present_position = (
                    present_joint_state[jname].present_position)

        joint_max_move = 0.0

        if self.walking_state == 'WalkingInitPose':
            # Execute one frame of the init-pose trajectory
            if self.calc_joint_tra is not None:
                count   = self.init_pose_count
                # Build joint_names sorted by index — no lambda/comprehension
                joint_names = [''] * 14
                for _jn, _idx in self.joint_index.items():
                    joint_names[_idx] = _jn
                tra_len = len(self.calc_joint_tra[0]) if self.calc_joint_tra else 0
                if count < tra_len:
                    for i, jname in enumerate(joint_names):
                        prev = self.goal_position[jname].goal_position
                        self.goal_position[jname].goal_position = (
                            self.calc_joint_tra[i][count])
                        delta = fabs(self.goal_position[jname].goal_position - prev) * 180.0 / pi
                        if delta > joint_max_move:
                            joint_max_move = delta
                    self.init_pose_count = count + 1
                else:
                    # Trajectory complete; transition to WalkingReady
                    self.walking_state   = 'WalkingReady'
                    self.init_pose_count = 0

        elif self.walking_state in ('WalkingEnable', 'WalkingReady'):
            # Normal gait cycle — snapshot previous goals (no dict comprehension)
            prev_goals = {}
            for jname in self.goal_position:
                prev_goals[jname] = self.goal_position[jname].goal_position

            self.get_leg_angle()
            self.get_arm_angle()
            self.process_phase()

            for jname in self.goal_position:
                delta = fabs(self.goal_position[jname].goal_position
                             - prev_goals[jname]) * 180.0 / pi
                if delta > joint_max_move:
                    joint_max_move = delta

            if self.DEBUG:
                if joint_max_move > 0.1:
                    print('Cal Err : ', joint_max_move)
                else:
                    print('GOOD')

        return joint_max_move, self.time_, self.goal_position
