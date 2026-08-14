#!/usr/bin/env python3
"""PreToolUse guard: enforce tree-wiring conventions in tree/*_bt.py.

Thin wrapper. The rules live in guardlib.tree_pre_rules.

THIS IS THE ONLY GUARD THAT BLOCKS. It prints to stderr and exits 2, which denies the
write; every other guard is advisory and exits 0. Keep it that way -- a raw composite
that reaches disk is a memory-semantics bug the whole composites contract exists to
prevent, and an advisory here would be a suggestion the agent may decline.

It is also the only guard that reads the PENDING content out of the tool payload
rather than the file on disk, which it must: PreToolUse runs before the write lands.
"""
import json
import sys

_HOOK = 'xyz_bt_tree_pre_guard'

# Re-exports for external callers (validate_engine.py's library_sweep and the skill
# validators import these by name). Wrapped because this module must remain IMPORTABLE
# when guardlib is missing: main() then fails open and records a `degraded` event. An
# unguarded import here would raise at module scope, and the hook would die with a
# traceback instead of stepping aside -- exit 1 with no event, which is the one outcome
# the fail-open rule exists to prevent. validate_engine still fails LOUD on the same
# breakage, because it imports these names and gets AttributeError.
try:
    from guardlib.path_spec import TREE_PATTERN as _TREE_PATTERN  # noqa: E402,F401
    from guardlib.tree_pre_rules import _violations  # noqa: E402,F401
except Exception:  # pragma: no cover - guardlib absent
    pass


def _log(action, file_path, violations, data):
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

    tool_input = data.get("tool_input", {})
    file_path = tool_input.get("file_path", "")

    try:
        from guardlib import registry
    except Exception:
        _degraded(file_path, data)
        sys.exit(0)

    specs = registry.tree_rules_for(file_path)
    if not specs:
        sys.exit(0)

    # Write → content; Edit → new_string.
    content = tool_input.get("content") or tool_input.get("new_string") or ""
    if not content:
        sys.exit(0)

    violations = registry.run_all(specs, content, file_path)
    if not violations:
        _log('pass', file_path, (), data)
        sys.exit(0)

    _log('blocked', file_path, violations, data)

    body = "\n".join(f"  - {v}" for v in violations)
    sys.stderr.write(
        "[xyz_bt tree guard] blocked write to " + file_path + ":\n" + body + "\n")
    sys.exit(2)


if __name__ == '__main__':
    main()
