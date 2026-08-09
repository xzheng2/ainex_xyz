#!/usr/bin/env python3
"""ProcessManager — start/stop a fixed set of background child processes.

Purpose
-------
Some BT projects need to start and stop *other* programs on demand while the BT
node's own launch stays up — for example a stand-alone servo demo script that is
NOT a ROS node (so `rosnode kill` does not apply). This manager owns each target
as a `subprocess.Popen` handle so it can be started idempotently and stopped
gracefully.

Layer contract
--------------
Process management is NOT ROS egress, so it does not go through _RuntimeIO. A
project RuntimeFacade holds a ProcessManager instance and exposes it to L2 nodes
via the XyzBTFacade.start_process / stop_process contract. L2 nodes never touch
this class directly.

Registry
--------
`registry` maps a stable target name to the argv list used to launch it, e.g.::

    ProcessManager({
        'serial_servo_move': [
            'python3',
            '/home/ubuntu/ros_ws/src/ainex_tutorial/'
            'scripts/serial_servo/serial_servo_move_demo.py',
        ],
    })

The child process is spawned in the same container/host as the BT node, so it
inherits hardware access; use absolute paths valid inside that environment.

Stop semantics
--------------
stop() sends SIGINT first (not SIGTERM) so a target whose cleanup lives in an
`except KeyboardInterrupt:` block gets a chance to run (e.g. return a servo to a
safe pose). If the process is still alive after `sigint_timeout` seconds it is
escalated to SIGKILL.

Observability
-------------
An optional BT observability `logger` (DebugEventLogger) records a `ros_out`
event with `comm_type='process'` on each start/stop, mirroring the _RuntimeIO
`_emit` pattern. `logger=None` is a zero-cost no-op.
"""
import os
import signal
import subprocess
import time as _time


class ProcessManager:
    """Start/stop fixed-registry background processes via Popen handles."""

    def __init__(self, registry: dict, logger=None, tick_id_getter=None):
        """
        Args:
            registry:        {name -> argv list}. Only these names may be started.
            logger:          BT observability logger (None = zero-cost no-op).
            tick_id_getter:  callable → current tick_id (int).
        """
        self._registry = dict(registry or {})
        self._procs = {}                       # name -> subprocess.Popen
        self._logger = logger
        self._tick_id = tick_id_getter or (lambda: -1)

    # ── Queries ─────────────────────────────────────────────────────────────

    def is_running(self, name: str) -> bool:
        """True if the named process was started and has not yet exited."""
        proc = self._procs.get(name)
        return proc is not None and proc.poll() is None

    # ── Start / stop ────────────────────────────────────────────────────────

    def start(self, name: str, bt_node=None, tick_id=None) -> bool:
        """Start a registered process. Idempotent.

        Returns:
            True  — a new process was spawned.
            False — already running (no-op).

        Raises:
            KeyError — name is not in the registry.
        """
        if name not in self._registry:
            raise KeyError(
                'ProcessManager: unknown target {!r} (registry: {})'.format(
                    name, sorted(self._registry)))
        if self.is_running(name):
            return False
        argv = self._registry[name]
        proc = subprocess.Popen(argv)
        self._procs[name] = proc
        self._emit(name, 'start', bt_node, tick_id,
                   summary='pid={} argv={}'.format(proc.pid, argv))
        return True

    def stop(self, name: str, sigint_timeout: float = 2.0,
             bt_node=None, tick_id=None) -> bool:
        """Stop a running process: SIGINT, then SIGKILL after sigint_timeout.

        Returns:
            True  — a running process was signalled.
            False — not running (no-op).

        Raises:
            KeyError — name is not in the registry.
        """
        if name not in self._registry:
            raise KeyError(
                'ProcessManager: unknown target {!r} (registry: {})'.format(
                    name, sorted(self._registry)))
        proc = self._procs.get(name)
        if proc is None or proc.poll() is not None:
            self._procs.pop(name, None)
            return False

        escalated = False
        proc.send_signal(signal.SIGINT)
        deadline = _time.time() + sigint_timeout
        while _time.time() < deadline:
            if proc.poll() is not None:
                break
            _time.sleep(0.05)
        if proc.poll() is None:
            proc.kill()
            escalated = True
        try:
            proc.wait(timeout=1.0)
        except Exception:
            pass
        self._procs.pop(name, None)
        self._emit(name, 'stop', bt_node, tick_id,
                   summary='pid={} escalated_to_sigkill={}'.format(
                       proc.pid, escalated))
        return True

    def stop_all(self):
        """Stop every tracked process. Intended for rospy.on_shutdown cleanup."""
        for name in list(self._procs):
            try:
                self.stop(name)
            except Exception:
                pass

    # ── Logging helper ──────────────────────────────────────────────────────

    def _emit(self, name, semantic_source, bt_node, tick_id, summary=''):
        """Emit a process ros_out log entry. No-op when logger is None."""
        if not self._logger:
            return
        tid = tick_id if tick_id is not None else self._tick_id()
        self._logger.emit_comm({
            'event':                  'ros_out',
            'ts':                     _time.time(),
            'tick_id':                tid,
            'phase':                  'tick',
            'bt_node':                bt_node or '',
            'ros_node':               '',
            'semantic_source':        semantic_source,
            'target':                 name,
            'comm_type':              'process',
            'direction':              'out',
            'payload':                {'target': name, 'pid': os.getpid()},
            'summary':                summary,
            'attribution_confidence': 'high',
            'node':                   bt_node or '',
        })
