#!/usr/bin/env python3
# encoding: utf-8
"""{{CLASS_NAME}} - {{ROS_TOPIC}} -> {{DESCRIPTION}}

Input adapter.

ROS topic subscribed:
  {{ROS_TOPIC}}

ROS message type:
  {{MSG_TYPE}}

BB keys written:
  {{BB_KEY_LIST}}

Per-BB-value traceability:
  TODO: for every BB write, document:
    - source ROS message field(s)
    - extraction/classification helper
    - rule/threshold
    - snapshot field
    - final BB.* key

Threshold/calibration source:
  TODO: CONFIG_DEFAULTS or ainex_bt_edu/config/*.yaml.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on the main thread.

Observability:
  write_snapshot() may emit only, via AinexInputAdapter helpers:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import threading

import rospy
{{MSG_TYPE_IMPORT}}
from ainex_bt_edu.blackboard_keys import BB
from ainex_bt_edu.base_adapter import AinexInputAdapter


class {{CLASS_NAME}}(AinexInputAdapter):
    """Adapter: {{ROS_TOPIC}} -> {{DESCRIPTION}}"""

    ROS_TOPIC = '{{ROS_TOPIC}}'
    ADAPTER_NAME = '{{CLASS_NAME}}'
    BB_READS = []
    BB_WRITES = [
        # TODO: BB.SOME_VALUE,
    ]
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'threshold': 0,             # example: sensor-level detection threshold
        'center_x_offset': 0,      # example: camera center x offset (pixels)
        # TODO: add all thresholds and calibration values here.
        # These must match __init__ default args. No hard-coded constants in helpers.
    }

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None,
                 threshold: int = 0,
                 center_x_offset: int = 0):
        """
        Args:
            lock: Shared threading.Lock (same object as bt_node.lock).
            logger: DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
            threshold: Sensor-level detection threshold.
            center_x_offset: Camera center x offset (pixels).

        Every CONFIG_DEFAULTS entry must have a matching __init__ arg stored on self._.
        Classification helpers must use self._ fields, not raw literals.
        """
        super().__init__(
            lock=lock,
            logger=logger,
            tick_id_getter=tick_id_getter,
        )
        self._threshold = threshold
        self._center_x_offset = center_x_offset

        # Live async state, written only by _callback under self._lock.
        {{BB_KEY_INITS}}

        # Latched BB client, written only by write_snapshot() on the main thread.
        self._bb = self.make_latched_bb_client(name="{{CLASS_NAME}}")
        {{BB_REGISTER_KEYS}}
        # Initialise BB before first tick so BT nodes never hit KeyError.
        {{BB_INIT_WRITES}}

        rospy.Subscriber('{{ROS_TOPIC}}', {{MSG_TYPE}}, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _extract_xxx(self, msg: {{MSG_TYPE}}):
        """Extract relevant raw data from the ROS message.

        No BB writes, ROS calls, logger calls, or live-state mutation here.
        """
        raise NotImplementedError("Fill in ROS message extraction")

    def _classify_xxx(self, extracted):
        """Apply documented sensor-level rules/thresholds.

        No BT decisions or action strategy here.
        Use self._ instance fields (self._threshold, self._center_x_offset, etc.),
        not raw literals. Hard-coded thresholds are a conformance violation.
        """
        raise NotImplementedError("Fill in sensor-level classification")

    def _make_bb_writes(self, classified) -> dict:
        """Return {BB.KEY: value} for every BB key this adapter writes."""
        raise NotImplementedError("Fill in BB write mapping")

    def _apply_live_writes(self, bb_writes: dict) -> None:
        """Copy bb_writes into self._live_* fields.

        Called by _callback() while holding self._lock. This is the only helper
        in the conversion path that mutates live state.
        """
        {{CALLBACK_LOGIC}}

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: {{MSG_TYPE}}) -> None:
        """Receive ROS message, compute snapshot fields, update live state.

        This is the only place that may:
        - update callback-thread-only accumulators (no lock needed)
        - call rospy.loginfo/logwarn on detected state changes

        Side-effect-free helpers (_extract_xxx, _classify_xxx, _make_bb_writes)
        must not mutate self state or call rospy logging.
        If _classify_xxx manages counters, it should accept and return them:
          new_count, result = self._classify_xxx(extracted, self._count)
        """
        extracted = self._extract_xxx(msg)
        classified = self._classify_xxx(extracted)
        # TODO: if _classify_xxx returns counters, apply them here (before lock):
        #   self._count_xxx = new_count
        # TODO: if state change warrants logging:
        #   if classified is not None:
        #       rospy.loginfo('[{{CLASS_NAME}}] state: %s', classified)
        bb_writes = self._make_bb_writes(classified)

        with self._lock:
            self._apply_live_writes(bb_writes)
            self._received_count += 1

    # ------------------------------------------------------------------
    # Public two-phase latch API
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        """Return current live-state snapshot and reset received_count.

        Must be called while the caller holds self._lock.
        """
        snap = {
            {{SNAP_FIELDS}}
            'received_count': self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write pre-captured snapshot to BB and emit business-in events.

        No lock is needed; this is called only from the main thread after the
        shared lock has been released.
        """
        self.emit_ros_in(
            tick_id=tick_id,
            received_count=snap['received_count'],
        )

        # Write to BB before input_state so logs reflect what BT will read.
        {{BB_WRITES}}

        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={
                {{INPUT_STATE_BB_WRITES}}
            },
        )
