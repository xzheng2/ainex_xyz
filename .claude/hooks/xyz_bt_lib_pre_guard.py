#!/usr/bin/env python3
"""PreToolUse hook: inject xyz-bt-lib-node / xyz-bt-lib-adapter skill reminders before writing to xyz_bt_lib source.

Thin wrapper. The reminder texts live in guardlib.hints, the shapes in
guardlib.path_spec.

This guard evaluates NO content -- it only says "you are about to write this kind of
file, here are its conventions". That is why its events are logged as `reminder` and
not as `pass`: a pass would claim a check that never ran.
"""
import json
import sys

_HOOK = 'xyz_bt_lib_pre_guard'


def _log(file_path, rule, data):
    """Record that guidance was injected. Not a `pass`: this guard evaluates no
    content, so calling it a passed check would claim a check that never ran."""
    try:
        import guard_log
        guard_log.log_reminder(_HOOK, file_path, rule, data)
    except Exception:
        pass


def _degraded(file_path, data):
    try:
        import guard_log
        guard_log.log_degraded(_HOOK, file_path, data=data)
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

    try:
        from guardlib import hints, path_spec
    except Exception:
        _degraded(file_path, data)
        sys.exit(0)

    if path_spec.NODE_PATTERN.search(file_path):
        _log(file_path, 'bt_lib_pre.node_reminder', data)
        print(json.dumps({"additionalContext": hints.NODE_REMINDER}))
    elif path_spec.ADAPTER_PATTERN.search(file_path):
        _log(file_path, 'bt_lib_pre.adapter_reminder', data)
        print(json.dumps({"additionalContext": hints.ADAPTER_REMINDER}))

    sys.exit(0)


if __name__ == '__main__':
    main()
