#!/usr/bin/env python3
# encoding: utf-8
# Marathon Behavior Tree - 树结构定义
#
# 树形结构示意图：
#
# [-] MarathonBT [Sequence]
#     [o] SafetyGate [Selector]          ← 安全门控：站立才继续，否则恢复
#         --> IsRobotStanding            ← 条件：机器人是否站立
#         [-] Recovery [Sequence]        ← 恢复序列：先停步再起立
#             --> StopWalking
#             --> RecoverFromFall
#     [o] PatrolControl [Selector]       ← 巡线控制：有线则跟，无线则停
#         [-] LineFollowing [Sequence]
#             --> IsLineDetected         ← 条件：是否检测到线
#             --> FollowLine             ← 动作：跟线行走
#         --> StopWalking               ← fallback：无线时停步

import py_trees

from behaviours.conditions import IsRobotStanding, IsLineDetected
from behaviours.actions import RecoverFromFall, FollowLine, StopWalking


class MarathonBT(py_trees.composites.Sequence):
    """
    HuroCup 马拉松赛项行为树根节点。

    将原来分散在两个 ROS 节点（marathon_node + fall_rise_node）中的
    状态机逻辑，统一用行为树表达：

    原架构：
        fall_rise_node  ──/fall_rise/state──►  marathon_node
        (独立进程，状态机)                      (if/else 主循环)

    BT 架构：
        单节点 + 行为树，通过共享黑板传递状态，
        优先级和安全约束直接由树结构保证。
    """

    def __init__(self, motion_manager, gait_manager, visual_patrol, buzzer_pub):
        super().__init__("MarathonBT", memory=False)
        self._motion_manager = motion_manager
        self._gait_manager = gait_manager
        self._visual_patrol = visual_patrol
        self._buzzer_pub = buzzer_pub
        self._build_tree()

    def _build_tree(self):
        # ── 安全门控 ─────────────────────────────────────────────────────────
        # Selector：子节点依次尝试，任一 SUCCESS 则整体 SUCCESS
        # - IsRobotStanding: 站立 → 直接 SUCCESS，跳过 Recovery
        # - Recovery:        跌倒 → 先停步，再执行起立动作
        recovery = py_trees.composites.Sequence(
            "Recovery",
            memory=False,
            children=[
                StopWalking("StopWalkingOnFall", self._gait_manager),
                RecoverFromFall(
                    "RecoverFromFall",
                    self._motion_manager,
                    self._gait_manager,
                    self._buzzer_pub,
                ),
            ],
        )

        safety_gate = py_trees.composites.Selector(
            "SafetyGate",
            memory=False,
            children=[
                IsRobotStanding("IsRobotStanding"),
                recovery,
            ],
        )

        # ── 巡线控制 ──────────────────────────────────────────────────────────
        # Selector：
        # - LineFollowing: 有线 → 跟线行走（两步都 SUCCESS 才算成功）
        # - StopWalking:   无线 → 停步并返回 SUCCESS（防止树整体失败）
        line_following = py_trees.composites.Sequence(
            "LineFollowing",
            memory=False,
            children=[
                IsLineDetected("IsLineDetected"),
                FollowLine("FollowLine", self._visual_patrol),
            ],
        )

        patrol_control = py_trees.composites.Selector(
            "PatrolControl",
            memory=False,
            children=[
                line_following,
                StopWalking("StopWalkingNoLine", self._gait_manager),
            ],
        )

        self.add_children([safety_gate, patrol_control])


def bootstrap(motion_manager, gait_manager, visual_patrol, buzzer_pub) -> py_trees.behaviour.Behaviour:
    """工厂函数，供主节点调用。"""
    return MarathonBT(motion_manager, gait_manager, visual_patrol, buzzer_pub)
