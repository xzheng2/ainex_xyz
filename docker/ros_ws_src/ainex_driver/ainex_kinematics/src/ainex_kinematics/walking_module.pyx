# walking_module_without_balance.pyx
# cython: language_level=3
# Recovered from walking_module.so (ARM aarch64, Cython 0.29.16, Python 3.8.10)
# Compiler: GCC 9.4.0 (Ubuntu 20.04, aarch64)
# Build dir: /home/ubuntu/0610
# Module name override: walking_module (via setup.py Extension name)

import numpy as np
from libc.math cimport sin, cos, sqrt, fabs, acos, atan2, floor, M_PI as pi


cpdef double radians(double degrees):
    return degrees * pi / 180.0


cpdef double degrees(double randians):
    return randians * 180.0 / pi


cdef class JointPosition:

    cdef public double goal_position
    cdef public double present_position

    def __init__(self):
        self.goal_position    = 0.0
        self.present_position = 0.0


cdef class WalkingModule:

    # Python-object attributes
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

    # Pose offset parameters
    cdef public double x_offset_
    cdef public double y_offset_
    cdef public double z_offset_
    cdef public double r_offset_
    cdef public double p_offset_
    cdef public double a_offset_
    cdef public double hit_pitch_offset_

    # Movement state
    cdef public double previous_x_move_amplitude_
    cdef public double y_move_amplitude_shift_
    cdef public double arm_swing_gain_

    # Phase shift constants
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

    # Period / timing
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

    # Pelvis compensation
    cdef public double pelvis_offset_
    cdef public double pelvis_swing_

    # Motion amplitudes
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

    def __init__(self, leg_ik, walking_param, joint_index, init_position, smp_time,
                 control_cycle=None):
        self.leg_ik        = leg_ik
        self.walking_param = dict(walking_param)
        self.joint_index   = dict(joint_index)
        self.init_position = list(init_position)
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

        leg_data = leg_ik.get_leg_length()
        self.leg_data    = leg_data
        self.leg_length  = leg_data[0] + leg_data[1] + leg_data[2]
        self.joint_axis_direction = list(leg_ik.get_joint_axis_direction())

        self.goal_position        = {}
        self.present_position     = {}
        self.predict_joint_state  = {}
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

        self.update_time_param()
        self.update_movement_param()
        self.update_pose_param()

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

        self.init_pose_tra()

    cpdef set_walking_param(self, walking_param):
        self.walking_param = dict(walking_param)
        self.update_time_param()
        self.update_movement_param()
        self.update_pose_param()
        self.x_move_amplitude_shift_  = self.x_move_amplitude_
        self.y_swap_amplitude_shift_  = walking_param['y_swap_amplitude']

    cpdef get_walking_param(self):
        return dict(self.walking_param)

    cpdef update_time_param(self):
        cdef double pt, dsp, ssp, ssp_t

        pt  = self.walking_param['period_time'] / 1000.0
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

        self.phase1_time_ = (self.l_ssp_start_time_ + self.l_ssp_end_time_)   / 2.0
        self.phase2_time_ = (self.l_ssp_end_time_   + self.r_ssp_start_time_) / 2.0
        self.phase3_time_ = (self.r_ssp_start_time_ + self.r_ssp_end_time_)   / 2.0

    cpdef update_movement_param(self):
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

    cpdef start(self):
        self.ctrl_running  = True
        self.real_running  = True
        self.walking_state = 'WalkingEnable'

    cpdef stop(self):
        self.ctrl_running = False

    cpdef process_phase(self):
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

    cpdef cal_tra(self, double pos_start, double vel_start, double accel_start,
                double pos_end,   double vel_end,   double accel_end,
                double smp_time,  double mov_time):
        cdef int    n, i
        cdef double T, T2, T3, T4, T5, t

        n  = int(round(mov_time / smp_time))
        T  = mov_time
        T2 = T * T;  T3 = T2 * T;  T4 = T3 * T;  T5 = T4 * T

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

        minimum_jerk_tra = np.zeros(n + 1)
        for i in range(n + 1):
            t = <double>i * smp_time
            minimum_jerk_tra[i] = (pos_start + vel_start * t + accel_start * t * t * 0.5
                                   + a3 * t * t * t
                                   + a4 * t * t * t * t
                                   + a5 * t * t * t * t * t)
        return minimum_jerk_tra

    cpdef init_pose_tra(self, present_joint_state=None):
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
                    self.goal_position[jname].goal_position = (
                        present_joint_state[jname].present_position)

        thigh = self.leg_data[0]
        calf  = self.leg_data[1]
        ankle = self.leg_data[2]

        x_r =  self.x_offset_
        y_r = -(self.y_offset_ - thigh * sin(self.hit_pitch_offset_))
        z_r = -(self.leg_length - self.z_offset_)

        x_l =  self.x_offset_
        y_l =  self.y_offset_ + thigh * sin(self.hit_pitch_offset_)
        z_l = -(self.leg_length - self.z_offset_)

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

        target = [0.0] * 14
        for i in range(6):
            target[i]     = r_angles[i]
        for i in range(6):
            target[6 + i] = l_angles[i]

        # Post-IK hip_pitch_offset correction (same as get_leg_angle)
        target[2] -= self.joint_axis_direction[2] * self.hit_pitch_offset_
        target[8] -= self.joint_axis_direction[8] * self.hit_pitch_offset_

        target[12] = self.init_position[12]
        target[13] = self.init_position[13]

        mov_time   = 1.0
        n_joints   = 14
        joint_names = [''] * n_joints
        for _jn, _idx in self.joint_index.items():
            joint_names[_idx] = _jn

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

    cpdef double get_time_(self):
        return self.time_

    cpdef double wSin(self, double time, double period, double period_shift,
                      double mag, double mag_shift):
        return mag * sin(2.0 * pi * time / period - period_shift) + mag_shift

    cpdef get_leg_angle(self):
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

        # X (forward/backward)
        x_swap = self.wSin(time, self.x_swap_period_time_,
                           self.x_swap_phase_shift_,
                           self.x_swap_amplitude_,
                           self.x_swap_amplitude_shift_)

        if time < self.l_ssp_start_time_:
            r_x_move = self.wSin(self.l_ssp_start_time_, self.x_move_period_time_,
                                 self.x_move_phase_shift_,
                                 -self.x_move_amplitude_, self.x_move_amplitude_shift_)
            l_x_move = self.wSin(self.l_ssp_start_time_, self.x_move_period_time_,
                                 self.x_move_phase_shift_ + pi,
                                 self.x_move_amplitude_, self.x_move_amplitude_shift_)
        elif time < self.l_ssp_end_time_:
            r_x_move = self.wSin(time, self.x_move_period_time_,
                                 self.x_move_phase_shift_,
                                 -self.x_move_amplitude_, self.x_move_amplitude_shift_)
            l_x_move = self.wSin(time, self.x_move_period_time_,
                                 self.x_move_phase_shift_ + pi,
                                 self.x_move_amplitude_, self.x_move_amplitude_shift_)
        elif time < self.r_ssp_start_time_:
            r_x_move = self.wSin(self.l_ssp_end_time_, self.x_move_period_time_,
                                 self.x_move_phase_shift_,
                                 -self.x_move_amplitude_, self.x_move_amplitude_shift_)
            l_x_move = self.wSin(self.l_ssp_end_time_, self.x_move_period_time_,
                                 self.x_move_phase_shift_ + pi,
                                 self.x_move_amplitude_, self.x_move_amplitude_shift_)
        elif time < self.r_ssp_end_time_:
            r_x_move = self.wSin(time, self.x_move_period_time_,
                                 self.x_move_phase_shift_ + pi,
                                 -self.x_move_amplitude_, self.x_move_amplitude_shift_)
            l_x_move = self.wSin(time, self.x_move_period_time_,
                                 self.x_move_phase_shift_,
                                 self.x_move_amplitude_, self.x_move_amplitude_shift_)
        else:
            r_x_move = self.wSin(self.r_ssp_end_time_, self.x_move_period_time_,
                                 self.x_move_phase_shift_ + pi,
                                 -self.x_move_amplitude_, self.x_move_amplitude_shift_)
            l_x_move = self.wSin(self.r_ssp_end_time_, self.x_move_period_time_,
                                 self.x_move_phase_shift_,
                                 self.x_move_amplitude_, self.x_move_amplitude_shift_)

        # Y (lateral)
        y_swap = self.wSin(time, self.y_swap_period_time_,
                           self.y_swap_phase_shift_,
                           self.y_swap_amplitude_,
                           self.y_swap_amplitude_shift_)

        if time < self.l_ssp_start_time_:
            r_y_move = self.wSin(self.l_ssp_start_time_, self.y_move_period_time_,
                                 self.y_move_phase_shift_,
                                 -self.y_move_amplitude_, self.y_move_amplitude_shift_)
            l_y_move = self.wSin(self.l_ssp_start_time_, self.y_move_period_time_,
                                 self.y_move_phase_shift_ + pi,
                                 self.y_move_amplitude_, self.y_move_amplitude_shift_)
        elif time < self.l_ssp_end_time_:
            r_y_move = self.wSin(time, self.y_move_period_time_,
                                 self.y_move_phase_shift_,
                                 -self.y_move_amplitude_, self.y_move_amplitude_shift_)
            l_y_move = self.wSin(time, self.y_move_period_time_,
                                 self.y_move_phase_shift_ + pi,
                                 self.y_move_amplitude_, self.y_move_amplitude_shift_)
        elif time < self.r_ssp_start_time_:
            r_y_move = self.wSin(self.l_ssp_end_time_, self.y_move_period_time_,
                                 self.y_move_phase_shift_,
                                 -self.y_move_amplitude_, self.y_move_amplitude_shift_)
            l_y_move = self.wSin(self.l_ssp_end_time_, self.y_move_period_time_,
                                 self.y_move_phase_shift_ + pi,
                                 self.y_move_amplitude_, self.y_move_amplitude_shift_)
        elif time < self.r_ssp_end_time_:
            r_y_move = self.wSin(time, self.y_move_period_time_,
                                 self.y_move_phase_shift_ + pi,
                                 -self.y_move_amplitude_, self.y_move_amplitude_shift_)
            l_y_move = self.wSin(time, self.y_move_period_time_,
                                 self.y_move_phase_shift_,
                                 self.y_move_amplitude_, self.y_move_amplitude_shift_)
        else:
            r_y_move = self.wSin(self.r_ssp_end_time_, self.y_move_period_time_,
                                 self.y_move_phase_shift_ + pi,
                                 -self.y_move_amplitude_, self.y_move_amplitude_shift_)
            l_y_move = self.wSin(self.r_ssp_end_time_, self.y_move_period_time_,
                                 self.y_move_phase_shift_,
                                 self.y_move_amplitude_, self.y_move_amplitude_shift_)

        # Z (height)
        r_z_swap = self.wSin(time, self.z_swap_period_time_,
                             self.z_swap_phase_shift_,
                             self.z_swap_amplitude_,
                             self.z_swap_amplitude_shift_)
        l_z_swap = r_z_swap

        r_z_move = self.wSin(time, self.period_time_, pi,
                             self.z_move_amplitude_, self.z_move_amplitude_shift_)
        l_z_move = self.wSin(time, self.period_time_, 0.0,
                             self.z_move_amplitude_, self.z_move_amplitude_shift_)

        # A (yaw)
        if time < self.l_ssp_start_time_:
            r_a_move = self.wSin(self.l_ssp_start_time_, self.a_move_period_time_,
                                 self.a_move_phase_shift_ + pi,
                                 -self.a_move_amplitude_, self.a_move_amplitude_shift_)
            l_a_move = self.wSin(self.l_ssp_start_time_, self.a_move_period_time_,
                                 self.a_move_phase_shift_,
                                 self.a_move_amplitude_, self.a_move_amplitude_shift_)
        elif time < self.l_ssp_end_time_:
            r_a_move = self.wSin(time, self.a_move_period_time_,
                                 self.a_move_phase_shift_ + pi,
                                 -self.a_move_amplitude_, self.a_move_amplitude_shift_)
            l_a_move = self.wSin(time, self.a_move_period_time_,
                                 self.a_move_phase_shift_,
                                 self.a_move_amplitude_, self.a_move_amplitude_shift_)
        elif time < self.r_ssp_start_time_:
            r_a_move = self.wSin(self.l_ssp_end_time_, self.a_move_period_time_,
                                 self.a_move_phase_shift_ + pi,
                                 -self.a_move_amplitude_, self.a_move_amplitude_shift_)
            l_a_move = self.wSin(self.l_ssp_end_time_, self.a_move_period_time_,
                                 self.a_move_phase_shift_,
                                 self.a_move_amplitude_, self.a_move_amplitude_shift_)
        elif time < self.r_ssp_end_time_:
            r_a_move = self.wSin(time, self.a_move_period_time_,
                                 self.a_move_phase_shift_,
                                 -self.a_move_amplitude_, self.a_move_amplitude_shift_)
            l_a_move = self.wSin(time, self.a_move_period_time_,
                                 self.a_move_phase_shift_ + pi,
                                 self.a_move_amplitude_, self.a_move_amplitude_shift_)
        else:
            r_a_move = self.wSin(self.r_ssp_end_time_, self.a_move_period_time_,
                                 self.a_move_phase_shift_,
                                 -self.a_move_amplitude_, self.a_move_amplitude_shift_)
            l_a_move = self.wSin(self.r_ssp_end_time_, self.a_move_period_time_,
                                 self.a_move_phase_shift_ + pi,
                                 self.a_move_amplitude_, self.a_move_amplitude_shift_)

        # Pelvis roll compensation
        if time < self.l_ssp_start_time_ or time > self.r_ssp_end_time_:
            r_pelvis = 0.0
            l_pelvis = 0.0
        elif time < self.l_ssp_end_time_:
            r_pelvis = self.pelvis_swing_
            l_pelvis = self.pelvis_offset_
        elif time < self.r_ssp_start_time_:
            r_pelvis = 0.0
            l_pelvis = 0.0
        else:
            r_pelvis = self.pelvis_offset_
            l_pelvis = self.pelvis_swing_

        # Foot target positions in hip frame
        r_x     =  self.x_offset_ + r_x_move + x_swap
        r_y     = -(self.y_offset_ - self.leg_data[0] * sin(self.hit_pitch_offset_) + r_y_move + y_swap)
        r_z     = -(self.leg_length - self.z_offset_ - r_z_swap - r_z_move)
        r_roll  =  self.r_offset_
        r_pitch =  0.0
        r_a     =  self.a_offset_ + r_a_move

        l_x     =  self.x_offset_ + l_x_move + x_swap
        l_y     =  (self.y_offset_ + self.leg_data[0] * sin(self.hit_pitch_offset_) + l_y_move + y_swap)
        l_z     = -(self.leg_length - self.z_offset_ - l_z_swap - l_z_move)
        l_roll  =  self.r_offset_
        l_pitch =  0.0
        l_a     =  self.a_offset_ + l_a_move

        # IK
        r_angles = self.leg_ik.right_leg_ik(r_x, r_y, r_z, r_roll, r_pitch, r_a)
        l_angles = self.leg_ik.left_leg_ik( l_x, l_y, l_z, l_roll, l_pitch, l_a)

        if not r_angles:
            print('Right IK not Solved: ')
            return
        if not l_angles:
            print('Left IK not Solved: ')
            return

        # Store IK results via joint_index mapping
        for jname, idx in self.joint_index.items():
            if idx < 6:
                self.goal_position[jname].goal_position = r_angles[idx]
            elif idx < 12:
                self.goal_position[jname].goal_position = l_angles[idx - 6]

        # Post-IK pelvis roll compensation (with joint_axis_direction)
        self.goal_position['r_hip_roll'].goal_position += self.joint_axis_direction[1] * r_pelvis
        self.goal_position['l_hip_roll'].goal_position += self.joint_axis_direction[7] * l_pelvis

        # Post-IK hip_pitch_offset correction (OP3 reference: offset -= direction * hit_pitch_offset_)
        self.goal_position['r_hip_pitch'].goal_position -= self.joint_axis_direction[2] * self.hit_pitch_offset_
        self.goal_position['l_hip_pitch'].goal_position -= self.joint_axis_direction[8] * self.hit_pitch_offset_

    cpdef get_arm_angle(self):
        cdef double r_sho_pitch, l_sho_pitch
        cdef double hip_pitch_r, hip_pitch_l

        hip_pitch_r = self.goal_position['r_hip_pitch'].goal_position
        hip_pitch_l = self.goal_position['l_hip_pitch'].goal_position

        r_sho_pitch = -hip_pitch_r * self.arm_swing_gain_
        l_sho_pitch = -hip_pitch_l * self.arm_swing_gain_

        self.goal_position['r_sho_pitch'].goal_position = (
            r_sho_pitch * self.joint_axis_direction[12]
            + self.init_position[12])
        self.goal_position['l_sho_pitch'].goal_position = (
            l_sho_pitch * self.joint_axis_direction[13]
            + self.init_position[13])

    cpdef bint init_pose_finish(self):
        return self.walking_state != 'WalkingInitPose'

    cpdef bint walk_finish(self):
        return (not self.ctrl_running) and (not self.real_running)

    cpdef run(self, present_joint_state):
        cdef double joint_max_move, delta
        cdef int    count

        for jname in self.present_position:
            if jname in present_joint_state:
                self.present_position[jname].present_position = (
                    present_joint_state[jname].present_position)

        joint_max_move = 0.0

        if self.walking_state == 'WalkingInitPose':
            if self.calc_joint_tra is not None:
                count   = self.init_pose_count
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
                    self.walking_state   = 'WalkingReady'
                    self.init_pose_count = 0

        elif self.walking_state in ('WalkingEnable', 'WalkingReady'):
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
