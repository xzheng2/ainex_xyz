#!/usr/bin/env python3
# encoding: utf-8
# @data:2025/03/09
# Marathon BT Node - 行为树架构主节点
#
# 替代原有的两节点方案（marathon_node.py + fall_rise_node.py），
# 将跌倒检测、起立恢复、视觉巡线统一在一个节点内，
# 用行为树管理控制逻辑和优先级。
#
# 原架构：
#   fall_rise_node (进程A) ──/fall_rise/state──► marathon_node (进程B)
#
# BT架构：
#   marathon_bt_node (单进程)
#   ├── ROS 订阅者（IMU、视觉）→ 写黑板
#   └── 行为树（30Hz tick）   ← 读黑板执行动作

import sys
import math
import time
import signal
import rospy
import py_trees

from threading import RLock
from std_msgs.msg import String
from sensor_msgs.msg import Imu

from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect
from ros_robot_controller.msg import BuzzerState

# 将本目录加入 Python 路径（供 import marathon_bt / behaviours 使用）
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from visual_patrol import VisualPatrol  # type: ignore
from marathon_bt import bootstrap


class MarathonBTNode(Common):
    """
    马拉松行为树主节点。

    职责：
      1. 订阅 /imu → 检测跌倒，更新黑板 robot_state
      2. 订阅 /object/pixel_coords → 更新黑板 line_data
      3. 以 30Hz 驱动行为树 tick
      4. 行为树负责决策：恢复站立 or 跟线行走 or 停步
    """

    # 三段 ROI（占图像比例）：上/中/下
    line_roi = [
        (5 / 12, 6 / 12, 1 / 4, 3 / 4),
        (6 / 12, 7 / 12, 1 / 4, 3 / 4),
        (7 / 12, 8 / 12, 1 / 4, 3 / 4),
    ]
    image_process_size = [160, 120]

    # 跌倒判断阈值（连续 N 帧才触发，防抖动）
    FALL_COUNT_THRESHOLD = 100

    def __init__(self, name: str):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.lock = RLock()

        # 头部初始位置
        self.head_pan_init = 500
        self.head_tilt_init = 340
        # Common 提供：gait_manager, motion_manager, detect_pub,
        #              start/stop/enter_func 等
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        # 巡线逻辑（使用 Common 提供的 gait_manager）
        self.visual_patrol = VisualPatrol(self.gait_manager)

        # 蜂鸣器发布者
        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1
        )

        # ── 黑板初始化 ──────────────────────────────────────────────────────
        py_trees.blackboard.Blackboard.enable_activity_stream()
        self._bb = py_trees.blackboard.Client(name="MarathonBTNode")
        self._bb.register_key("robot_state", access=py_trees.common.Access.WRITE)
        self._bb.register_key("line_data",   access=py_trees.common.Access.WRITE)
        self._bb.robot_state = 'stand'
        self._bb.line_data   = None

        # 跌倒计数（防抖）
        self._count_lie     = 0
        self._count_recline = 0

        # ── 构建并初始化行为树 ───────────────────────────────────────────────
        bt_root = bootstrap(
            self.motion_manager,
            self.gait_manager,
            self.visual_patrol,
            self.buzzer_pub,
        )
        self.bt = py_trees.trees.BehaviourTree(root=bt_root)
        self.bt.setup(timeout=15)
        common.loginfo(f"{name}: behaviour tree ready")

        # ── ROS 订阅与服务 ──────────────────────────────────────────────────
        rospy.Subscriber('/imu', Imu, self._imu_callback)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self._objects_callback)
        rospy.Service('~set_color', SetString, self._set_color_srv_callback)

        signal.signal(signal.SIGINT, self._shutdown)

        # 准备姿势
        self.motion_manager.run_action('walk_ready')

        # 自动启动
        if rospy.get_param('~start', True):
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self._set_color_srv_callback(String(target_color))
            self.start_srv_callback(None)
            common.loginfo(f"{name}: start marathon BT, tracking [{target_color}] lane")

    # ── IMU 回调：跌倒检测 ────────────────────────────────────────────────────
    def _imu_callback(self, msg: Imu):
        """
        将 IMU 加速度转化为倾斜角，连续超阈值帧后判定跌倒，
        写入黑板 robot_state，供行为树 RecoverFromFall 节点使用。
        """
        angle = abs(int(math.degrees(
            math.atan2(msg.linear_acceleration.y, msg.linear_acceleration.z)
        )))

        with self.lock:
            # 只在站立状态下做跌倒检测（恢复中不重复触发）
            if self._bb.robot_state != 'stand':
                return

            # 向前倒（angle 接近 0°）
            if angle < 30:
                self._count_lie += 1
            else:
                self._count_lie = 0

            # 向后倒（angle 接近 180°）
            if angle > 150:
                self._count_recline += 1
            else:
                self._count_recline = 0

            if self._count_lie >= self.FALL_COUNT_THRESHOLD:
                self._count_lie = 0
                self._bb.robot_state = 'lie_to_stand'
                rospy.logwarn(f"{self.name}: forward fall detected → lie_to_stand")

            elif self._count_recline >= self.FALL_COUNT_THRESHOLD:
                self._count_recline = 0
                self._bb.robot_state = 'recline_to_stand'
                rospy.logwarn(f"{self.name}: backward fall detected → recline_to_stand")

    # ── 视觉回调：巡线数据 ────────────────────────────────────────────────────
    def _objects_callback(self, msg: ObjectsInfo):
        """
        从颜色识别结果中提取线条数据，写入黑板 line_data。
        """
        line_data = None
        for obj in msg.data:
            if obj.type == 'line':
                line_data = obj
                break
        self._bb.line_data = line_data

    # ── 颜色设置服务 ──────────────────────────────────────────────────────────
    def _set_color_srv_callback(self, msg):
        """配置颜色识别节点的检测参数（ROI、颜色、检测类型）。"""
        param = ColorDetect()
        param.color_name        = msg.data
        param.use_name          = True
        param.detect_type       = 'line'
        param.image_process_size = self.image_process_size

        for roi_attr, roi_range in zip(
            ['up', 'center', 'down'], self.line_roi
        ):
            roi = getattr(param.line_roi, roi_attr)
            roi.y_min = int(roi_range[0] * self.image_process_size[1])
            roi.y_max = int(roi_range[1] * self.image_process_size[1])
            roi.x_min = int(roi_range[2] * self.image_process_size[0])
            roi.x_max = int(roi_range[3] * self.image_process_size[0])

        param.min_area = 1
        param.max_area = self.image_process_size[0] * self.image_process_size[1]
        self.detect_pub.publish([param])
        common.loginfo(f"{self.name}: set_color → {msg.data}")
        return [True, 'set_color']

    # ── 关闭处理 ──────────────────────────────────────────────────────────────
    def _shutdown(self, signum, frame):
        with self.lock:
            self.running = False
        common.loginfo(f"{self.name}: shutdown signal received")

    # ── 主循环（行为树 tick） ──────────────────────────────────────────────────
    def run(self):
        """
        以 30Hz 驱动行为树 tick。
        每次 tick 后打印树状态（throttle 2s，避免刷屏）。
        """
        tick_rate = rospy.Rate(30)  # 30 Hz

        while self.running and not rospy.is_shutdown():
            if self.start:
                try:
                    self.bt.tick(post_tick_handler=self._print_tree)
                except Exception as e:
                    rospy.logerr(f"{self.name}: BT tick error: {e}")
            tick_rate.sleep()

        # 退出前：恢复初始姿势，关闭颜色识别
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

    def _print_tree(self, tree: py_trees.trees.BehaviourTree):
        """每 2 秒打印一次行为树状态，方便调试观察。"""
        rospy.loginfo_throttle(
            2,
            '\n' + py_trees.display.unicode_tree(root=tree.root, show_status=True)
        )


if __name__ == '__main__':
    MarathonBTNode('marathon_bt').run()
