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

Gait profile dispatch
---------------------
gait_step(profile='go'|'turn', x, y, yaw, ...)
    Selects go_cfg or turn_cfg and calls CommFacade.set_step with
    motion_profile forwarded into the JSONL payload.

go_step / turn_step are thin wrappers for the common cases.

Usage in leaf nodes
-------------------
    # FollowLine: algorithm lives in L2_Gait_FollowLine.update()
    self._facade.go_step(x=x_out, y=0, yaw=yaw, ..., semantic_source='follow_line')

    # FindLine: algorithm lives in L2_Gait_FindLine.update()
    self._facade.turn_step(x=0, y=0, yaw=gait_yaw, ..., semantic_source='search_line')

    # HeadSweepAlign: algorithm lives in L2_Head_FindLineSweep._update_align()
    self._facade.go_step(x=0, y=0, yaw=yaw, ..., semantic_source='head_sweep_align')
"""
import time
import rospy
from ros_robot_controller.msg import BuzzerState
from ainex_bt_edu.base_facade import AinexBTFacade


class MarathonSemanticFacade(AinexBTFacade):
    """Translates marathon leaf-node intent to project-semantic ROS commands."""

    # ── Head servo config ─────────────────────────────────────────────────
    HEAD_PAN_SERVO     = 23
    HEAD_MOVE_DELAY_MS = 50

    # ── Fall recovery actions ─────────────────────────────────────────────
    LIE_ACTION     = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, go_cfg: dict, turn_cfg: dict, comm_facade,
                 tick_id_getter=None):
        """
        Args:
            go_cfg:     Gait config for straight walking.
                        Dict with keys: 'dsp' (list), 'gait_param' (dict),
                        'arm_swap' (int).
            turn_cfg:   Gait config for rotation-biased walking (same keys).
            comm_facade:    CommFacade instance (Generic ROS Facade).
            tick_id_getter: callable → current tick_id (fallback when leaf
                            nodes do not pass tick_id explicitly).
        """
        self._go    = go_cfg
        self._turn  = turn_cfg
        self._comm  = comm_facade
        self._tick_id = tick_id_getter or (lambda: -1)

    def _tid(self, tick_id):
        """Resolve tick_id: use caller-supplied value if given, else own getter."""
        return tick_id if tick_id is not None else self._tick_id()

    # ── Gait profile dispatch ─────────────────────────────────────────────

    def gait_step(self, profile, x, y, yaw, step_num=0,
                  bt_node=None, tick_id=None, semantic_source='gait_step'):
        """Send one gait step using the named motion profile.

        Args:
            profile: 'go' (straight walking) or 'turn' (rotation-biased).
                     Raises ValueError for any other value.
        """
        if profile not in ('go', 'turn'):
            raise ValueError(
                f"[MarathonSemanticFacade.gait_step] invalid profile: {profile!r}; "
                "must be 'go' or 'turn'")
        cfg = self._go if profile == 'go' else self._turn
        self._comm.set_step(
            bt_node=bt_node,
            semantic_source=semantic_source,
            tick_id=self._tid(tick_id),
            dsp=cfg['dsp'],
            x=x, y=y, yaw=yaw,
            gait_param=cfg['gait_param'],
            arm_swap=cfg['arm_swap'],
            step_num=step_num,
            motion_profile=profile,
        )

    def go_step(self, x, y, yaw, step_num=0,
                bt_node=None, tick_id=None, semantic_source='gait_step'):
        """Thin wrapper: gait_step('go', ...)."""
        self.gait_step('go', x, y, yaw, step_num, bt_node, tick_id, semantic_source)

    def turn_step(self, x, y, yaw, step_num=0,
                  bt_node=None, tick_id=None, semantic_source='gait_step'):
        """Thin wrapper: gait_step('turn', ...)."""
        self.gait_step('turn', x, y, yaw, step_num, bt_node, tick_id, semantic_source)

    # ── Deprecated locomotion methods ─────────────────────────────────────

    def follow_line(self, line_data, bt_node, tick_id=None):
        """Deprecated: algorithm moved to L2_Gait_FollowLine.update().

        Kept only to satisfy the AinexBTFacade abstract contract.
        Not called by any BT node in the current code path.
        """
        rospy.logwarn_once(
            '[MarathonSemanticFacade] follow_line() is deprecated; '
            'algorithm lives in L2_Gait_FollowLine.update()')

    def search_line(self, *args, **kwargs):
        """Removed: logic moved to L2_Gait_FindLine.update() → turn_step()."""
        raise AttributeError(
            '[MarathonSemanticFacade] search_line() has been removed; '
            'L2_Gait_FindLine now calls facade.turn_step() directly')

    def head_sweep_align(self, *args, **kwargs):
        """Removed: logic moved to L2_Head_FindLineSweep._update_align()."""
        raise AttributeError(
            '[MarathonSemanticFacade] head_sweep_align() has been removed; '
            'L2_Head_FindLineSweep now calls facade.go_step()/turn_step() directly')

    # ── Other locomotion ──────────────────────────────────────────────────

    def stop_walking(self, bt_node, tick_id=None):
        """Disable the gait controller."""
        self._comm.disable_gait(
            bt_node=bt_node,
            semantic_source='stop_walking',
            tick_id=self._tid(tick_id),
        )

    # ── Recovery ──────────────────────────────────────────────────────────

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

    # ── Head ──────────────────────────────────────────────────────────────

    def move_head(self, pan_pos, bt_node, tick_id=None):
        """Command the head-pan servo to the given position."""
        self._comm.set_servos_position(
            self.HEAD_MOVE_DELAY_MS,
            [[self.HEAD_PAN_SERVO, int(pan_pos)]],
            bt_node=bt_node,
            semantic_source='move_head',
            tick_id=self._tid(tick_id),
        )
