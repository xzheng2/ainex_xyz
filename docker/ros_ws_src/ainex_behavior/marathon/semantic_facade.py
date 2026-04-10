#!/usr/bin/env python3
"""Project Semantic Facade for marathon BT.

Translates leaf-node business intent into project-semantic actions, then
delegates all ROS communication to the Generic ROS Facade (CommFacade).

Layer responsibilities
----------------------
Leaf nodes     → express WHAT they want ("follow line", "search line")
SemanticFacade → translate to HOW in project terms (which gait config,
                 which servo, which action file)
CommFacade     → handle actual ROS communication + emit attributed logs

Usage in leaf nodes
-------------------
    class FollowLine(py_trees.behaviour.Behaviour):
        def __init__(self, name, semantic_facade, tick_id_getter=None):
            self._semantic = semantic_facade
            self._tick_id_getter = tick_id_getter or (lambda: -1)

        def update(self):
            line_data = self.bb.line_data
            self._semantic.follow_line(
                line_data=line_data,
                bt_node=self.name,
                tick_id=self._tick_id_getter(),
            )
            return py_trees.common.Status.SUCCESS
"""
import time
import rospy
from ros_robot_controller.msg import BuzzerState


class MarathonSemanticFacade:
    """Translates marathon leaf-node intent to project-semantic ROS commands."""

    # ── Line-search tuning (moved from FindLine) ──────────────────────────
    IMAGE_CENTER_X = 80       # half of 160-px detection frame
    BASE_TURN_DEG  = 3        # conservative turn angle right after losing line
    MAX_TURN_DEG   = 7        # upper cap
    COUNT_SCALE_AT = 30       # lost_count at which turn reaches MAX_TURN_DEG
    DEFAULT_TURN_DEG = 3      # fallback when no last_line_x history

    # ── Head-sweep alignment tuning (moved from FindLineHeadSweep) ────────
    HEAD_PAN_CENTER    = 500
    SWEEP_LEFT_POS     = 700  # must match FindLineHeadSweep.SWEEP_LEFT_POS
    ALIGN_TURN_MAX_DEG = 7
    ALIGN_TURN_SCALE   = ALIGN_TURN_MAX_DEG / (SWEEP_LEFT_POS - HEAD_PAN_CENTER)
    PAN_INVERT         = 1    # +1: increasing servo value = head pans left

    # ── Head servo config ─────────────────────────────────────────────────
    HEAD_PAN_SERVO     = 23
    HEAD_MOVE_DELAY_MS = 50

    # ── Fall recovery actions ─────────────────────────────────────────────
    LIE_ACTION     = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, visual_patrol, comm_facade, tick_id_getter=None):
        """
        Args:
            visual_patrol: VisualPatrol instance (pure algorithm component).
            comm_facade:   CommFacade instance (Generic ROS Facade).
            tick_id_getter: callable → current tick_id (fallback when leaf
                            nodes do not pass tick_id explicitly).
        """
        self._vp     = visual_patrol
        self._comm   = comm_facade
        self._tick_id = tick_id_getter or (lambda: -1)

    def _tid(self, tick_id):
        """Resolve tick_id: use caller-supplied value if given, else own getter."""
        return tick_id if tick_id is not None else self._tick_id()

    # ── Public API ────────────────────────────────────────────────────────

    def follow_line(self, line_data, bt_node, tick_id=None):
        """Compute follow-line gait command and send via Generic ROS Facade."""
        cmd = self._vp.compute_follow_command(line_data.x, line_data.width)
        self._comm.set_step(
            bt_node=bt_node,
            semantic_source='follow_line',
            tick_id=self._tid(tick_id),
            dsp=cmd['dsp'],
            x=cmd['x'],
            y=cmd['y'],
            yaw=cmd['yaw'],
            gait_param=cmd['gait_param'],
            arm_swap=cmd['arm_swap'],
            step_num=cmd['step_num'],
        )

    def search_line(self, last_line_x, lost_count, bt_node, tick_id=None):
        """Turn in-place to recover a lost line.

        Direction is biased toward where the line was last seen.
        Turn magnitude scales linearly with lost_count.

        Returns gait_yaw (int) for the caller to use in loginfo if desired.
        """
        scale    = min(1.0, lost_count / max(self.COUNT_SCALE_AT, 1))
        turn_deg = int(self.BASE_TURN_DEG
                       + scale * (self.MAX_TURN_DEG - self.BASE_TURN_DEG))

        if last_line_x is None:
            gait_yaw = self.DEFAULT_TURN_DEG
        elif last_line_x < self.IMAGE_CENTER_X:
            gait_yaw = turn_deg      # line was left → turn left
        else:
            gait_yaw = -5            # line was right → turn right

        rospy.loginfo('[SemanticFacade.search_line] lost_count=%d  gait_yaw=%+d',
                      lost_count, gait_yaw)
        self._comm.set_step(
            bt_node=bt_node,
            semantic_source='search_line',
            tick_id=self._tid(tick_id),
            dsp=self._vp.turn_dsp,
            x=0,
            y=0,
            yaw=gait_yaw,
            gait_param=self._vp.turn_gait_param,
            arm_swap=self._vp.turn_arm_swap,
            step_num=0,
        )
        return gait_yaw

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
            rospy.loginfo('[SemanticFacade.recover_from_fall] lie_to_stand')
            action_name = self.LIE_ACTION
        elif robot_state == 'recline_to_stand':
            rospy.loginfo('[SemanticFacade.recover_from_fall] recline_to_stand')
            action_name = self.RECLINE_ACTION
        else:
            rospy.logwarn('[SemanticFacade.recover_from_fall] unknown state: %s',
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

    def move_head(self, pan_pos, bt_node, tick_id=None):
        """Command the head-pan servo to the given position."""
        self._comm.set_servos_position(
            self.HEAD_MOVE_DELAY_MS,
            [[self.HEAD_PAN_SERVO, int(pan_pos)]],
            bt_node=bt_node,
            semantic_source='move_head',
            tick_id=self._tid(tick_id),
        )

    def head_sweep_align(self, head_offset, bt_node, tick_id=None):
        """Command body-turn proportional to head_offset during alignment.

        Uses go_gait when |yaw| < 2 deg, turn_gait otherwise.
        Returns gait_yaw (int) for the caller to use in loginfo.
        """
        gait_yaw = int(max(-self.ALIGN_TURN_MAX_DEG,
                           min(self.ALIGN_TURN_MAX_DEG,
                               head_offset * self.ALIGN_TURN_SCALE * self.PAN_INVERT)))
        if abs(gait_yaw) < 2:
            self._comm.set_step(
                bt_node=bt_node,
                semantic_source='head_sweep_align',
                tick_id=self._tid(tick_id),
                dsp=self._vp.go_dsp,
                x=0, y=0, yaw=gait_yaw,
                gait_param=self._vp.go_gait_param,
                arm_swap=self._vp.go_arm_swap,
                step_num=0,
            )
        else:
            self._comm.set_step(
                bt_node=bt_node,
                semantic_source='head_sweep_align',
                tick_id=self._tid(tick_id),
                dsp=self._vp.turn_dsp,
                x=0, y=0, yaw=gait_yaw,
                gait_param=self._vp.turn_gait_param,
                arm_swap=self._vp.turn_arm_swap,
                step_num=0,
            )
        return gait_yaw
