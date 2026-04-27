#!/usr/bin/env python3
"""L2 Action: sweep head to find a lost line, then align body before handing off.

Phase 1 — SWEEP
    Robot stands still.  Head pans between sweep_left_pos and sweep_right_pos,
    pausing sweep_pause_ticks ticks at each end.
    Transitions to ALIGN as soon as the camera sees the line.

Phase 2 — ALIGN
    Head steps back toward centre (align_step units/tick).  Each tick the body
    turns via turn_step with yaw derived from head servo direction — NOT from
    camera pixels.  Yaw magnitude is linearly scaled from align_turn_min_deg
    (head near centre) to align_turn_max_deg (head at full sweep range), so the
    body always turns at a meaningful rate throughout ALIGN.
    Returns SUCCESS when head_pan is within center_threshold of head_pan_center
    so the Selector re-evaluates: IsLineDetected + IsHeadCentered both pass →
    FollowLine takes over.

BB reads:
  BB.LINE_DATA          (/latched/line_data)         — triggers SWEEP→ALIGN transition
  BB.LAST_LINE_ERROR_X  (/latched/last_line_error_x) — seeds initial sweep direction

BB writes:
  BB.HEAD_PAN_POS  (/head_pan_pos) — read by companion IsHeadCentered condition

Facade:    stop_walking(), move_head(pan_pos), turn_step(x, y, yaw)
Strategy:  _compute_align_turn(head_offset) → gait_yaw (int)

Returns:
    RUNNING → sweep or align in progress
    SUCCESS → ALIGN complete (head within center_threshold of centre)

CONFIG_DEFAULTS:
    sweep_left_pos:      700   leftmost  servo position (500=centre, 875=hard left)
    sweep_right_pos:     300   rightmost servo position (500=centre, 125=hard right)
    sweep_step:          10    servo units per 30 Hz tick (must NOT be a factor of 200)
    sweep_pause_ticks:   0     ticks to hold at each endpoint before reversing
    center_threshold:    30    ±units around head_pan_center that counts as 'centred'
    align_step:          7     servo units per tick the head moves toward centre
    align_turn_min_deg:  3     minimum body-turn yaw (°) applied during ALIGN
    align_turn_max_deg:  8     maximum body-turn yaw (°) at full sweep deflection
    head_pan_center:     500   servo centre position
"""
from py_trees.common import Access, Status
from xyz_bt_edu.base_node import XyzL2ActionNode
from xyz_bt_edu.base_facade import XyzBTFacade
from xyz_bt_edu.blackboard_keys import BB


class L2_Head_FindLineSweep(XyzL2ActionNode):
    """Sweep-then-align head search. RUNNING during search; SUCCESS when aligned."""

    LEVEL        = 'L2'
    BB_READS     = [BB.LINE_DATA, BB.LAST_LINE_ERROR_X]
    BB_WRITES    = [BB.HEAD_PAN_POS]
    FACADE_CALLS = ['stop_walking', 'move_head', 'turn_step']
    CONFIG_DEFAULTS = {
        'sweep_left_pos':     700,
        'sweep_right_pos':    300,
        'sweep_step':         10,
        'sweep_pause_ticks':  0,
        'center_threshold':   30,
        'align_step':         7,
        'align_turn_min_deg': 3,
        'align_turn_max_deg': 8,
        'head_pan_center':    500,
    }

    # ── Direction convention ───────────────────────────────────────────────
    # +1 → increasing servo value = head pans LEFT (confirmed hardware)
    PAN_INVERT = 1

    # ── States ─────────────────────────────────────────────────────────────
    _ST_SWEEP = 'sweep'
    _ST_ALIGN = 'align'

    def __init__(self, name: str = 'L2_Head_FindLineSweep',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 sweep_left_pos: int = 700,
                 sweep_right_pos: int = 300,
                 sweep_step: int = 10,
                 sweep_pause_ticks: int = 0,
                 center_threshold: int = 30,
                 align_step: int = 7,
                 align_turn_min_deg: int = 3,
                 align_turn_max_deg: int = 8,
                 head_pan_center: int = 500):
        """
        Args:
            sweep_left_pos:     leftmost servo position (hw max 875).
            sweep_right_pos:    rightmost servo position (hw min 125).
            sweep_step:         servo units per tick while sweeping.
                                Must NOT be a factor of 200 (half sweep range),
                                otherwise the head will never reach exact centre
                                and the sweep will overshoot endpoints.
            sweep_pause_ticks:  ticks to hold at each endpoint before reversing.
            center_threshold:   ±units around head_pan_center that counts as centred.
            align_step:         servo units per tick moving toward centre.
            align_turn_min_deg: minimum body-turn yaw (°) at CENTER_THRESHOLD.
            align_turn_max_deg: maximum body-turn yaw (°) at full sweep range.
            head_pan_center:    servo centre position.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._sweep_left_pos     = sweep_left_pos
        self._sweep_right_pos    = sweep_right_pos
        self._sweep_step         = sweep_step
        self._sweep_pause_ticks  = sweep_pause_ticks
        self._center_threshold   = center_threshold
        self._align_step         = align_step
        self._align_turn_min_deg = align_turn_min_deg
        self._align_turn_max_deg = align_turn_max_deg
        self._head_pan_center    = head_pan_center
        self._bb                 = None
        self._bb_pan             = None
        self._head_pan           = self._head_pan_center
        self._sweep_dir          = +self.PAN_INVERT   # initial: sweep left
        self._pause_ticks        = 0
        self._state              = self._ST_SWEEP
        self._fresh_start        = True  # True on first run and after each ALIGN success

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=f'{self.name}_latched', namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY,          access=Access.READ)
        self._bb.register_key(key=BB.LAST_LINE_ERROR_X_KEY,  access=Access.READ)

        self._bb_pan = self.attach_blackboard_client(name=self.name)
        self._bb_pan.register_key(key=BB.HEAD_PAN_POS, access=Access.WRITE)
        self._bb_pan.head_pan_pos = self._head_pan_center  # initialise for IsHeadCentered

    def initialise(self):
        """Called on first activation and after each SUCCESS.  Reset to SWEEP.

        _sweep_dir is only updated from last_x on a genuine fresh start (first
        run, or just completed an ALIGN).  On mid-sweep re-activations caused by
        the memory=False Selector restarting the node every tick, _sweep_dir is
        preserved so the sweep continues in the same direction.

        stop_walking() is only called when line_data is None (SWEEP mode — robot
        stands still while scanning).  When line_data is present the node will
        immediately re-enter ALIGN, so the gait must not be stopped mid-turn.
        """
        if self._bb.line_data is None:            # SWEEP: stand still while scanning
            self.call_facade('stop_walking')
        self._state       = self._ST_SWEEP
        self._pause_ticks = 0

        if self._fresh_start:
            # Genuine fresh start: pick sweep direction from last known error sign.
            # last_line_error_x < 0 → line was to the LEFT  → look left first.
            # last_line_error_x >= 0 or None → look right (or unknown).
            self._fresh_start = False
            last_err = self._bb.last_line_error_x
            if last_err is not None and last_err < 0:
                self._sweep_dir = +self.PAN_INVERT   # line was left  → look left first
            else:
                self._sweep_dir = -self.PAN_INVERT   # line was right or unknown → look right
        # else: mid-sweep reinit (memory=False tick) — preserve _sweep_dir

        self._command_head(self._head_pan)
        self.emit_action_intent(
            action='head_sweep_start',
            inputs={'head_pan': self._head_pan, 'sweep_dir': self._sweep_dir},
        )

    def update(self) -> Status:
        if self._state == self._ST_SWEEP:
            return self._update_sweep()
        return self._update_align()

    # ── Phase 1 ────────────────────────────────────────────────────────────

    def _update_sweep(self) -> Status:
        if self._bb.line_data is not None:
            self._state = self._ST_ALIGN
            self.emit_decision(
                inputs={'head_pan': self._head_pan, 'transition': 'ALIGN'},
                status=Status.RUNNING,
                reason='line detected, transitioning to ALIGN',
            )
            return self._update_align()

        # Hold at endpoint before reversing
        if self._pause_ticks > 0:
            self._pause_ticks -= 1
            return Status.RUNNING

        # Step head
        self._head_pan = int(self._head_pan + self._sweep_dir * self._sweep_step)

        if self._head_pan >= self._sweep_left_pos:
            self._head_pan    = self._sweep_left_pos
            self._sweep_dir   = -self.PAN_INVERT
            self._pause_ticks = self._sweep_pause_ticks
        elif self._head_pan <= self._sweep_right_pos:
            self._head_pan    = self._sweep_right_pos
            self._sweep_dir   = +self.PAN_INVERT
            self._pause_ticks = self._sweep_pause_ticks

        self._command_head(self._head_pan)
        self._write_head_pan()
        self.emit_decision(
            inputs={'head_pan': self._head_pan, 'sweep_dir': self._sweep_dir},
            status=Status.RUNNING,
            reason='sweeping',
            bb_writes={BB.HEAD_PAN_POS: self._head_pan},
        )
        return Status.RUNNING

    # ── Phase 2 ────────────────────────────────────────────────────────────

    def _update_align(self) -> Status:
        head_offset = self._head_pan - self._head_pan_center

        if abs(head_offset) <= self._center_threshold:
            # Head is centred — snap to exact centre and signal done
            self._head_pan = self._head_pan_center
            self._command_head(self._head_pan_center)
            self._write_head_pan()
            self.call_facade('stop_walking')
            self._fresh_start = True   # next initialise() is a genuine restart
            self.emit_decision(
                inputs={'head_pan': self._head_pan},
                status=Status.SUCCESS,
                reason='head centred, ALIGN complete',
                bb_writes={BB.HEAD_PAN_POS: self._head_pan},
            )
            return Status.SUCCESS

        # Move head one step toward centre (don't overshoot)
        step = -self._align_step if head_offset > 0 else self._align_step
        new_pan = self._head_pan + step
        self._head_pan = (min(new_pan, self._head_pan_center) if step > 0
                          else max(new_pan, self._head_pan_center))
        self._command_head(self._head_pan)
        self._write_head_pan()

        # Turn body based on head direction (servo position, not camera pixels).
        # Always calls turn_step regardless of magnitude.
        gait_yaw = self._compute_align_turn(head_offset)
        self.call_facade('turn_step', x=0, y=0, yaw=gait_yaw,
                         semantic_source='head_sweep_align')
        self.emit_decision(
            inputs={'head_pan': self._head_pan, 'head_offset': head_offset,
                    'gait_yaw': gait_yaw},
            status=Status.RUNNING,
            reason='aligning body to head direction',
            bb_writes={BB.HEAD_PAN_POS: self._head_pan},
        )
        return Status.RUNNING

    # ── Helpers ────────────────────────────────────────────────────────────

    def _compute_align_turn(self, head_offset: int) -> int:
        """Return signed gait yaw for body alignment from head servo offset.

        Yaw = lerp(align_turn_min_deg, align_turn_max_deg, norm), where norm
        is head_offset normalised over the half sweep range
        [0, sweep_left_pos − head_pan_center].
        No BB/ROS/facade calls here.
        """
        norm = min(1.0, abs(head_offset) / float(self._sweep_left_pos - self._head_pan_center))
        magnitude = self._align_turn_min_deg + (self._align_turn_max_deg - self._align_turn_min_deg) * norm
        return int(magnitude) * (1 if head_offset * self.PAN_INVERT > 0 else -1)

    def _command_head(self, pan_pos: int):
        self.call_facade('move_head', pan_pos=int(pan_pos))

    def _write_head_pan(self):
        self._bb_pan.head_pan_pos = self._head_pan
