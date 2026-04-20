#!/usr/bin/env python3
"""Project Semantic Facade for {{PROJECT_CLASS}} BT.

Translates leaf-node business intent into project-semantic actions, then
delegates all ROS communication to CommFacade.

Layer responsibilities
----------------------
Leaf nodes     → express WHAT they want ("follow line", "search line")
SemanticFacade → translate to HOW in project terms (which gait config, params)
CommFacade     → handle actual ROS communication + emit attributed logs
"""
from ainex_bt_edu.base_facade import AinexBTFacade


class {{PROJECT_CLASS}}SemanticFacade(AinexBTFacade):
    """Translates {{PROJECT_CLASS}} leaf-node intent to project-semantic ROS commands."""

    # ── Tuning constants ─────────────────────────────────────────────────
    HEAD_PAN_CENTER    = 500
    HEAD_PAN_SERVO     = 23
    HEAD_MOVE_DELAY_MS = 50

    LIE_ACTION     = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, comm_facade, tick_id_getter=None):
        """
        Args:
            comm_facade:   CommFacade instance (sole ROS I/O exit).
            tick_id_getter: callable → current tick_id.
        """
        self._comm    = comm_facade
        self._tick_id = tick_id_getter or (lambda: -1)

    def _tid(self, tick_id):
        return tick_id if tick_id is not None else self._tick_id()

    # ── AinexBTFacade interface ───────────────────────────────────────────

    def stop_walking(self, bt_node=None, tick_id=None):
        self._comm.disable_gait(
            bt_node=bt_node,
            semantic_source='stop_walking',
            tick_id=self._tid(tick_id),
        )

    def follow_line(self, line_data, bt_node=None, tick_id=None):
        """Deprecated — algorithm now lives in L2_Gait_FollowLine.update()."""
        pass  # no-op; not called on the current code path

    def gait_step(self, profile, x, y, yaw, step_num=0,
                  bt_node=None, tick_id=None, semantic_source='gait_step'):
        """Send one gait step. profile must be 'go' or 'turn'; raises ValueError otherwise."""
        if profile not in ('go', 'turn'):
            raise ValueError(
                '[{{PROJECT_CLASS}}SemanticFacade.gait_step] invalid profile: '
                f'{profile!r}; must be \'go\' or \'turn\'')
        # TODO: select go_cfg vs turn_cfg and call self._comm.set_step(...)
        raise NotImplementedError(
            'gait_step not implemented for {{PROJECT_CLASS}}')

    def go_step(self, x, y, yaw, step_num=0, bt_node=None, tick_id=None,
                semantic_source='gait_step'):
        """Thin wrapper: gait_step('go', ...)."""
        self.gait_step('go', x, y, yaw, step_num, bt_node, tick_id, semantic_source)

    def turn_step(self, x, y, yaw, step_num=0, bt_node=None, tick_id=None,
                  semantic_source='gait_step'):
        """Thin wrapper: gait_step('turn', ...)."""
        self.gait_step('turn', x, y, yaw, step_num, bt_node, tick_id, semantic_source)

    def recover_from_fall(self, robot_state, bt_node=None, tick_id=None):
        import time
        from ros_robot_controller.msg import BuzzerState
        tid = self._tid(tick_id)
        buzzer_msg = BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1)
        self._comm.publish_buzzer(buzzer_msg, bt_node=bt_node,
                                  semantic_source='recover_from_fall',
                                  tick_id=tid, reason='fall detected alert')
        time.sleep(2)
        self._comm.disable_gait(bt_node=bt_node,
                                semantic_source='recover_from_fall', tick_id=tid)
        if robot_state == 'lie_to_stand':
            self._comm.run_action(self.LIE_ACTION, bt_node=bt_node,
                                  semantic_source='recover_from_fall', tick_id=tid)
        elif robot_state == 'recline_to_stand':
            self._comm.run_action(self.RECLINE_ACTION, bt_node=bt_node,
                                  semantic_source='recover_from_fall', tick_id=tid)
        time.sleep(0.5)

    def move_head(self, pan_pos, bt_node=None, tick_id=None):
        self._comm.set_servos_position(
            self.HEAD_MOVE_DELAY_MS,
            [[self.HEAD_PAN_SERVO, int(pan_pos)]],
            bt_node=bt_node,
            semantic_source='move_head',
            tick_id=self._tid(tick_id),
        )

