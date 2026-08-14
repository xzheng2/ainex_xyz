#!/usr/bin/env python3
"""PreToolUse hook: inject conventions reminder before writing to xyz_behavior project files.

Thin wrapper. The hint texts live in guardlib.hints; which hint fires for a path lives
in guardlib.path_spec.hint_key().

The hint selection order is NOT the same as the category order -- `_groot.xml` is
tested before `tree/`, and `launch/` before `scripts/`. Both orders are declared in
path_spec and both are preserved deliberately; see its module docstring.

Evaluates no content, so its events are `reminder`, not `pass`.
"""
import json
import sys

_HOOK = 'xyz_behavior_pre_guard'


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

    tool_name = data.get("tool_name", "")
    if tool_name not in ("Write", "Edit"):
        sys.exit(0)

    file_path = data.get("tool_input", {}).get("file_path", "")

    try:
        from guardlib import hints, path_spec
    except Exception:
        _degraded(file_path, data)
        sys.exit(0)

    if not path_spec.BEHAVIOR_PATTERN.search(file_path):
        sys.exit(0)

    matched_key = path_spec.hint_key(file_path)
    hint = hints.FILE_TYPE_HINTS.get(matched_key) or hints.DEFAULT_BEHAVIOR_HINT

    # The hint key is already a stable identifier for which reminder fired.
    _log(file_path, 'behavior_pre.' + (matched_key or '<default>'), data)
    print(json.dumps({"additionalContext": hint}))
    sys.exit(0)


if __name__ == '__main__':
    main()
