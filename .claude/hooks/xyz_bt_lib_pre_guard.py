#!/usr/bin/env python3
"""PreToolUse hook: inject xyz-bt-lib-node / xyz-bt-lib-adapter skill reminders before writing to xyz_bt_lib source."""
import json
import re
import sys

_HOOK = 'xyz_bt_lib_pre_guard'

_NODE_PATTERN    = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L[123]_\w+/(?!__init__\.py$)[^/]+\.py$')
_ADAPTER_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/adapters/(?!__init__\.py$)[^/]+\.py$')

_NODE_REMINDER = (
    "[xyz_bt_lib node guard · pre-write reminder]\n"
    "Detected write to xyz_bt_lib behaviours/L*/... BT node file.\n"
    "Ensure content complies with xyz_bt_lib node conventions (xyz-bt-lib-node skill):\n"
    "  1. class L1_Foo(XyzL1ConditionNode) / L2_Foo(XyzL2ActionNode) / L3_Foo(XyzL3ActionNode)  — correct base\n"
    "  2. LEVEL = 'L1'|'L2'|'L3'          — class variable declaring node level\n"
    "  3. BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS declared\n"
    "  4. setup() calls super().setup(**kwargs)\n"
    "  5. Do not import rospy or call rospy.*()  — use self._logger.emit_bt() instead\n"
    "Run the xyz-bt-lib-node skill first if you need a template."
)

_ADAPTER_REMINDER = (
    "[xyz_bt_lib adapter guard · pre-write reminder]\n"
    "Detected write to xyz_bt_lib adapters/... adapter file.\n"
    "Ensure content complies with the two-phase lock protocol (xyz-bt-lib-adapter skill):\n"
    "  1. snapshot_and_reset()          — Phase 1: call inside with lock:, return snapshot and reset received_count\n"
    "  2. write_snapshot(snap, tick_id) — Phase 2: call after releasing lock, write BB + emit ros_in/input_state event\n"
    "  3. rospy.Subscriber created only in __init__()\n"
    "  4. Initial BB write in __init__() (ensures keys exist before first tick, prevents KeyError in BT nodes)\n"
    "Run the xyz-bt-lib-adapter skill first if you need a template."
)


def _log(file_path, rule, data):
    """Record that guidance was injected. Not a `pass`: this guard evaluates no
    content, so calling it a passed check would claim a check that never ran."""
    try:
        import guard_log
        guard_log.log_reminder(_HOOK, file_path, rule, data)
    except Exception:
        pass


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    if data.get("tool_name", "") not in ("Write", "Edit"):
        sys.exit(0)

    file_path = data.get("tool_input", {}).get("file_path", "")

    if _NODE_PATTERN.search(file_path):
        _log(file_path, 'bt_lib_pre.node_reminder', data)
        print(json.dumps({"additionalContext": _NODE_REMINDER}))
    elif _ADAPTER_PATTERN.search(file_path):
        _log(file_path, 'bt_lib_pre.adapter_reminder', data)
        print(json.dumps({"additionalContext": _ADAPTER_REMINDER}))

    sys.exit(0)


if __name__ == '__main__':
    main()
