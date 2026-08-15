#!/usr/bin/env python3
"""PostToolUse hook: check node/adapter compliance and blackboard_keys.py ROSA_TOPIC_MAP coverage.

Thin wrapper. The rules live in guardlib.bt_lib_rules / guardlib.behavior_rules and the
shape-to-rule binding in guardlib.registry; this file only turns a tool event into a
call and the result into advice.

Re-exported below for external callers: validate_engine.py's library_sweep and the
xyz-bt-lib-node / xyz-bt-lib-adapter skill validators import check_node, check_adapter,
check_bb_keys and the path patterns from this module by name.
"""
import json
import sys

_HOOK = 'xyz_bt_lib_guard'

# Re-exports for external callers (validate_engine.py's library_sweep and the skill
# validators import these by name). Wrapped because this module must remain IMPORTABLE
# when guardlib is missing: main() then fails open and records a `degraded` event. An
# unguarded import here would raise at module scope, and the hook would die with a
# traceback instead of stepping aside -- exit 1 with no event, which is the one outcome
# the fail-open rule exists to prevent. validate_engine still fails LOUD on the same
# breakage, because it imports these names and gets AttributeError.
try:
    from guardlib.bt_lib_rules import check_node, check_adapter, check_bb_keys  # noqa: E402,F401
    from guardlib.path_spec import (  # noqa: E402,F401
        NODE_PATTERN as _NODE_PATTERN,
        ADAPTER_PATTERN as _ADAPTER_PATTERN,
        BB_KEYS_PATTERN as _BB_KEYS_PATTERN,
    )
except Exception:  # pragma: no cover - guardlib absent
    pass


def _log(action, file_path, violations, data):
    """Record the event. Advisory only, and the record must never affect the advice."""
    try:
        import guard_log
        if violations:
            guard_log.log_violations(_HOOK, action, file_path, violations, data)
        else:
            guard_log.log_clean(_HOOK, file_path, data)
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

    # Imported inside main() so a broken guardlib fails open at runtime rather than
    # taking the process down. The push gate fails LOUD on the same breakage.
    try:
        from guardlib import registry
    except Exception:
        _degraded(file_path, data)
        sys.exit(0)

    specs = registry.lib_rules_for(file_path)
    if not specs:
        sys.exit(0)

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    violations = registry.run_all(specs, content, file_path)
    name = specs[0][0]

    if not violations:
        _log('pass', file_path, (), data)
        sys.exit(0)

    _log('warned', file_path, violations, data)

    lines = "\n".join(f"  {v}" for v in violations)
    if name == 'check_bb_keys':
        # check_bb_keys returns pre-formatted warning strings
        context = "\n".join(violations)
    else:
        kind = 'node' if name == 'check_node' else 'adapter'
        context = (
            f"[xyz_bt_lib {kind} guard · compliance check] {file_path} has {len(violations)} violation(s):\n"
            f"{lines}\n"
            "Please fix the issues above before proceeding — must comply with xyz_bt_lib conventions."
        )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


if __name__ == '__main__':
    main()
