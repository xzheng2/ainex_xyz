#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/11
# @author:aiden
# 自动踢球(autonomous ball kicking)
import time
import math
import rospy
import signal
import numpy as np

from std_msgs.msg import Float64, String
from ainex_sdk import pid, misc, common
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect
from ainex_kinematics.motion_manager import MotionManager


class PenaltyKickBallNode(Common):
    def __init__(self, name):
        # 初始化父类(initialize parent class)
        # 初始化头部位置(initialize head position)
        self.head_pan_init = 450  # 左右舵机的初始值(initial value of left-right servo)
        self.head_tilt_init = 500 # 上下舵机的初始值(initial value of up-down servo)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        # 初始化接近控制(initialize approaching control)
        self.gait_param = self.gait_manager.get_gait_param()

print(PenaltyKickBallNode("123").gait_param)
