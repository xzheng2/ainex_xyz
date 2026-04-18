#!/usr/bin/env python3
"""BT execution mode controller: RUN / PAUSE / STEP.

When the node is named {{PROJECT}}_bt, the resolved service names are:
  rosservice call /{{PROJECT}}_bt/bt/run    # → mode RUN (continuous tick)
  rosservice call /{{PROJECT}}_bt/bt/pause  # → mode PAUSE (freeze tree)
  rosservice call /{{PROJECT}}_bt/bt/step   # → one tick then auto-PAUSE
  rostopic echo  /{{PROJECT}}_bt/bt/mode    # → current mode string (latched)

Note: `/<project>_bt/stop` (from Common base class) clears self.start flag entirely.
Use `bt/pause` to freeze the tree while keeping the node alive.
"""
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse


class BTExecController:
    """
    Controls whether and how the BT tree ticks each loop iteration.

    RUN   — tick on every iteration (default after ~start)
    PAUSE — no ticking; tree state is frozen
    STEP  — tick exactly once, then revert to PAUSE automatically

    Services (std_srvs/Empty):
        ~bt/run    set mode to RUN
        ~bt/pause  set mode to PAUSE
        ~bt/step   queue one tick, then PAUSE

    Topic (std_msgs/String, latched):
        ~bt/mode   current mode string: 'run' or 'pause'
    """

    MODE_RUN   = 'run'
    MODE_PAUSE = 'pause'

    def __init__(self, lock, initial_mode=MODE_RUN):
        self._lock         = lock
        self._mode         = initial_mode
        self._step_pending = False

        self._pub = rospy.Publisher('~bt/mode', String, queue_size=1, latch=True)
        self._pub.publish(String(data=self._mode))

        rospy.Service('~bt/run',   Empty, self._cb_run)
        rospy.Service('~bt/pause', Empty, self._cb_pause)
        rospy.Service('~bt/step',  Empty, self._cb_step)

        rospy.loginfo('[BTExecController] init: mode=%s', self._mode)

    # ── Service callbacks ──────────────────────────────────────────────────

    def _cb_run(self, _req):
        with self._lock:
            self._mode         = self.MODE_RUN
            self._step_pending = False
        self._pub.publish(String(data=self.MODE_RUN))
        rospy.loginfo('[BTExecController] → RUN')
        return EmptyResponse()

    def _cb_pause(self, _req):
        with self._lock:
            self._mode         = self.MODE_PAUSE
            self._step_pending = False
        self._pub.publish(String(data=self.MODE_PAUSE))
        rospy.loginfo('[BTExecController] → PAUSE')
        return EmptyResponse()

    def _cb_step(self, _req):
        with self._lock:
            self._mode         = self.MODE_PAUSE   # ensure PAUSE after tick
            self._step_pending = True
        rospy.loginfo('[BTExecController] → STEP (one tick queued)')
        return EmptyResponse()

    # ── Main-loop gate ─────────────────────────────────────────────────────

    def should_tick(self):
        """Call once per loop iteration. Returns True if tree should tick.
        Consumes the step flag and publishes PAUSE after a STEP."""
        with self._lock:
            if self._mode == self.MODE_RUN:
                return True
            if self._step_pending:
                self._step_pending = False
                self._pub.publish(String(data=self.MODE_PAUSE))
                return True
            return False
