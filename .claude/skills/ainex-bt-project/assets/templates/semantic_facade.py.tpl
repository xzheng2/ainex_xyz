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
    IMAGE_CENTER_X  = 80
    BASE_TURN_DEG   = 3
    MAX_TURN_DEG    = 7
    COUNT_SCALE_AT  = 30
    DEFAULT_TURN_DEG = 3

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
        # TODO: compute follow command (copy VisualPatrol logic or adapt)
        # Example using VisualPatrol:
        # cmd = self._vp.compute_follow_command(line_data.x, line_data.width)
        # self._comm.set_step(bt_node=bt_node, semantic_source='follow_line',
        #                     tick_id=self._tid(tick_id), **cmd)
        raise NotImplementedError('follow_line not implemented for {{PROJECT_CLASS}}')

    def search_line(self, last_line_x=None, lost_count=0, bt_node=None, tick_id=None):
        scale    = min(1.0, lost_count / max(self.COUNT_SCALE_AT, 1))
        turn_deg = int(self.BASE_TURN_DEG
                       + scale * (self.MAX_TURN_DEG - self.BASE_TURN_DEG))
        if last_line_x is None:
            gait_yaw = self.DEFAULT_TURN_DEG
        elif last_line_x < self.IMAGE_CENTER_X:
            gait_yaw = turn_deg
        else:
            gait_yaw = -5
        # TODO: replace with project-specific gait params
        self._comm.set_step(
            bt_node=bt_node,
            semantic_source='search_line',
            tick_id=self._tid(tick_id),
            dsp=0, x=0, y=0, yaw=gait_yaw,
        )

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

    def head_sweep_align(self, head_offset=0, bt_node=None, tick_id=None):
        # TODO: implement if using head-sweep mode; otherwise can be no-op
        pass
