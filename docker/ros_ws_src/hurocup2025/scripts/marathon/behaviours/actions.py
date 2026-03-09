#!/usr/bin/env python3
# encoding: utf-8
# Marathon BT - Action Nodes
# 动作节点：执行具体行为，返回 SUCCESS/FAILURE/RUNNING

import time
import py_trees
from ros_robot_controller.msg import BuzzerState


class RecoverFromFall(py_trees.behaviour.Behaviour):
    """
    跌倒恢复动作节点。

    从黑板读取 robot_state（'lie_to_stand' 或 'recline_to_stand'），
    执行对应起立动作后将 robot_state 重置为 'stand'。

    流程：
      1. 蜂鸣器提示
      2. 停止步态（gait_manager.disable）
      3. 播放起立动作（motion_manager.run_action）
      4. 写回 robot_state = 'stand'
    """

    def __init__(self, name: str, motion_manager, gait_manager, buzzer_pub):
        super().__init__(name)
        self.motion_manager = motion_manager
        self.gait_manager = gait_manager
        self.buzzer_pub = buzzer_pub

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "robot_state", access=py_trees.common.Access.READ_WRITE
        )

    def update(self) -> py_trees.common.Status:
        state = self.blackboard.robot_state
        self.feedback_message = f"recovering from {state}"

        # 蜂鸣提示
        self.buzzer_pub.publish(
            BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1)
        )
        time.sleep(2)

        # 停止步态，播放起立动作
        self.gait_manager.disable()
        if state == 'lie_to_stand':
            self.motion_manager.run_action('lie_to_stand')
        elif state == 'recline_to_stand':
            self.motion_manager.run_action('recline_to_stand')

        time.sleep(0.5)

        # 恢复完成，写回站立状态
        self.blackboard.robot_state = 'stand'
        self.feedback_message = "recovery complete"
        return py_trees.common.Status.SUCCESS


class FollowLine(py_trees.behaviour.Behaviour):
    """
    视觉巡线动作节点。

    从黑板读取 line_data（ObjectInfo），调用 VisualPatrol.process()
    计算偏差并驱动步态前进/转向。
    """

    def __init__(self, name: str, visual_patrol):
        super().__init__(name)
        self.visual_patrol = visual_patrol

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "line_data", access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        line_data = self.blackboard.line_data
        if line_data is None:
            self.feedback_message = "line_data lost mid-tick"
            return py_trees.common.Status.FAILURE

        self.visual_patrol.process(line_data.x, line_data.width)
        self.feedback_message = f"following line x={line_data.x:.1f}"
        return py_trees.common.Status.SUCCESS


class StopWalking(py_trees.behaviour.Behaviour):
    """
    停止步态动作节点。

    调用 gait_manager.disable() 停止当前步态，始终返回 SUCCESS。
    用于两个场景：
      - 跌倒检测后，Recovery 序列中先停步再起立
      - 无线可循时，PatrolControl 的 fallback
    """

    def __init__(self, name: str, gait_manager):
        super().__init__(name)
        self.gait_manager = gait_manager

    def update(self) -> py_trees.common.Status:
        self.gait_manager.disable()
        self.feedback_message = "walking stopped"
        return py_trees.common.Status.SUCCESS
