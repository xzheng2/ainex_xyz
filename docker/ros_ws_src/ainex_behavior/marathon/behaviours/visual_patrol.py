#!/usr/bin/env python3
# encoding: utf-8
# Pure-computation visual patrol algorithm component.
# Calculates gait control commands from line detection data; does NOT issue
# any ROS commands directly.  The caller (MarathonSemanticFacade) is
# responsible for passing the returned command dict to the Generic ROS Facade.
import math
from ainex_sdk import misc, common


class VisualPatrol:
    def __init__(self, gait_manager):
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        self.x_range = [0, 0.015]
        self.yaw_range = [-8, 10]
        # gait_manager used only for initial parameter loading (one-time read)
        self.go_gait_param = gait_manager.get_gait_param()
        self.go_gait_param['body_height'] = 0.025
        self.go_gait_param['step_height'] = 0.015
        self.go_gait_param['hip_pitch_offset'] = 25
        self.go_gait_param['z_swap_amplitude'] = 0.006
        self.go_dsp = [300, 0.2, 0.02]
        self.go_arm_swap = 30

        self.turn_gait_param = gait_manager.get_gait_param()
        self.turn_gait_param['body_height'] = 0.025
        self.turn_gait_param['step_height'] = 0.015
        self.turn_gait_param['hip_pitch_offset'] = 25
        self.turn_gait_param['z_swap_amplitude'] = 0.006
        self.turn_gait_param['pelvis_offset'] = 8
        self.turn_dsp = [400, 0.2, 0.02]
        self.turn_arm_swap = 30

        self.x_max = self.x_range[1]

    def update_turn_gait(self, dsp=None, x_max=None, arm_swap=None, walking_param=None):
        if walking_param is not None:
            self.turn_gait_param = walking_param
        if dsp is not None:
            self.turn_dsp = dsp
        if x_max is not None:
            self.x_max = x_max
        if arm_swap is not None:
            self.turn_arm_swap = arm_swap

    def update_go_gait(self, dsp=None, x_max=None, arm_swap=None, walking_param=None):
        if walking_param is not None:
            self.go_gait_param = walking_param
        if dsp is not None:
            self.go_dsp = dsp
        if x_max is not None:
            self.x_max = x_max
        if arm_swap is not None:
            self.go_arm_swap = arm_swap

    def compute_follow_command(self, x, width):
        """Compute gait control values from line position.

        Returns a structured command dict to be passed to the Generic ROS Facade.
        Does NOT call any manager or issue any ROS communication.

        Returns:
            dict with keys: dsp, x, y, yaw, gait_param, arm_swap, step_num
        """
        center_x = width / 2 + self.calib_config['center_x_offset']
        if abs(x - center_x) < 10:
            yaw_output = 0
        elif abs(x - center_x) < width / 6:
            yaw_output = math.copysign(1, x - center_x) + misc.val_map(
                x - center_x, -width / 6, width / 6,
                self.yaw_range[0] + 1, self.yaw_range[1] - 1)
        else:
            yaw_output = math.copysign(self.yaw_range[1], x - center_x)

        if abs(yaw_output) > 6:
            x_output = 0.008
        else:
            x_output = self.x_max

        if abs(yaw_output) < 4:
            return {
                "dsp":        self.go_dsp,
                "x":          x_output,
                "y":          0,
                "yaw":        int(-yaw_output),
                "gait_param": self.go_gait_param,
                "arm_swap":   self.go_arm_swap,
                "step_num":   0,
            }
        else:
            return {
                "dsp":        self.turn_dsp,
                "x":          x_output,
                "y":          0,
                "yaw":        int(-yaw_output),
                "gait_param": self.turn_gait_param,
                "arm_swap":   self.turn_arm_swap,
                "step_num":   0,
            }
