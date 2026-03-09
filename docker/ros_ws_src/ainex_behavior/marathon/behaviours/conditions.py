#!/usr/bin/env python3
# encoding: utf-8
# Marathon BT - Condition Nodes
# 条件节点：从黑板读取状态，返回 SUCCESS/FAILURE

import py_trees


class IsRobotStanding(py_trees.behaviour.Behaviour):
    """
    检查机器人是否处于站立状态。
    读取黑板上的 robot_state，若为 'stand' 则返回 SUCCESS，否则 FAILURE。

    在行为树中作为安全门控的第一个子节点：
        Selector(SafetyGate)
        ├── IsRobotStanding   ← 站立时直接通过
        └── Recovery(Sequence)
    """

    def __init__(self, name: str = "IsRobotStanding"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "robot_state", access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        if self.blackboard.robot_state == 'stand':
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"robot_state={self.blackboard.robot_state}"
        return py_trees.common.Status.FAILURE


class IsLineDetected(py_trees.behaviour.Behaviour):
    """
    检查视觉是否检测到巡线数据。
    读取黑板上的 line_data，若不为 None 则返回 SUCCESS，否则 FAILURE。

    在行为树中作为巡线序列的前置条件：
        Selector(PatrolControl)
        ├── Sequence(LineFollowing)
        │   ├── IsLineDetected   ← 有线才执行 FollowLine
        │   └── FollowLine
        └── StopWalking          ← 无线则停步
    """

    def __init__(self, name: str = "IsLineDetected"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "line_data", access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        if self.blackboard.line_data is not None:
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "no line detected"
        return py_trees.common.Status.FAILURE
