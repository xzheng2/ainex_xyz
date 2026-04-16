#!/usr/bin/env python3
"""Project Semantic Facade for fall_recovery BT.

Translates leaf-node business intent into project-semantic actions, then
delegates all ROS communication to the Generic ROS Facade (CommFacade).

Only stop_walking() and recover_from_fall() are used in this project.
The remaining four AinexBTFacade abstract methods raise NotImplementedError
as they are not part of the fall-recovery scope.
"""
import time
import rospy
from ros_robot_controller.msg import BuzzerState
from ainex_bt_edu.base_facade import AinexBTFacade


class FallRecoverySemanticFacade(AinexBTFacade):
    """Translates fall_recovery leaf-node intent to project-semantic ROS commands."""

    # ── Fall recovery actions ─────────────────────────────────────────────
    LIE_ACTION     = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, comm_facade, tick_id_getter=None):
        """
        Args:
            comm_facade:    CommFacade instance (Generic ROS Facade).
            tick_id_getter: callable → current tick_id (fallback when leaf
                            nodes do not pass tick_id explicitly).
        """
        self._comm    = comm_facade
        self._tick_id = tick_id_getter or (lambda: -1)

    def _tid(self, tick_id):
        """Resolve tick_id: use caller-supplied value if given, else own getter."""
        return tick_id if tick_id is not None else self._tick_id()

    # ── Implemented methods ───────────────────────────────────────────────

    def stop_walking(self, bt_node, tick_id=None):
        """Disable the gait controller."""
        self._comm.disable_gait(
            bt_node=bt_node,
            semantic_source='stop_walking',
            tick_id=self._tid(tick_id),
        )

    def recover_from_fall(self, robot_state, bt_node, tick_id=None):
        """Full fall-recovery sequence: alert buzzer → disable gait → stand-up action.

        Does NOT write to the blackboard — the calling BT node handles that.
        """
        tid = self._tid(tick_id)
        buzzer_msg = BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1)
        self._comm.publish_buzzer(
            buzzer_msg,
            bt_node=bt_node,
            semantic_source='recover_from_fall',
            tick_id=tid,
            reason='fall detected alert',
        )
        time.sleep(2)

        self._comm.disable_gait(
            bt_node=bt_node,
            semantic_source='recover_from_fall',
            tick_id=tid,
        )

        if robot_state == 'lie_to_stand':
            rospy.loginfo('[FallRecoverySemanticFacade.recover_from_fall] lie_to_stand')
            action_name = self.LIE_ACTION
        elif robot_state == 'recline_to_stand':
            rospy.loginfo('[FallRecoverySemanticFacade.recover_from_fall] recline_to_stand')
            action_name = self.RECLINE_ACTION
        else:
            rospy.logwarn('[FallRecoverySemanticFacade.recover_from_fall] unknown state: %s',
                          robot_state)
            action_name = None

        if action_name:
            self._comm.run_action(
                action_name,
                bt_node=bt_node,
                semantic_source='recover_from_fall',
                tick_id=tid,
            )

        time.sleep(0.5)

    # ── Not used in fall_recovery ─────────────────────────────────────────

    def follow_line(self, line_data, bt_node, tick_id=None):
        raise NotImplementedError('not used in fall_recovery')

    def search_line(self, last_line_x, lost_count, bt_node, tick_id=None):
        raise NotImplementedError('not used in fall_recovery')

    def move_head(self, pan_pos, bt_node, tick_id=None):
        raise NotImplementedError('not used in fall_recovery')

    def head_sweep_align(self, head_offset, bt_node, tick_id=None):
        raise NotImplementedError('not used in fall_recovery')
