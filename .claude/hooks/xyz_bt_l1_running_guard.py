#!/usr/bin/env python3
"""PostToolUse hook: L1 condition nodes must be pure predicates (no Status.RUNNING).

Thin wrapper. The rule and its per-tier scoping live in guardlib.bt_lib_rules; which
files it claims lives in guardlib.path_spec.applies_l1.

Re-exported below: validate_engine.py's library_sweep and the xyz-bt-lib-node skill
validator import check_l1_running and applies_to from this module by name.
"""
import json
import sys

_HOOK = 'xyz_bt_l1_running_guard'

# Re-exports for external callers (validate_engine.py's library_sweep and the skill
# validators import these by name). Wrapped because this module must remain IMPORTABLE
# when guardlib is missing: main() then fails open and records a `degraded` event. An
# unguarded import here would raise at module scope, and the hook would die with a
# traceback instead of stepping aside -- exit 1 with no event, which is the one outcome
# the fail-open rule exists to prevent. validate_engine still fails LOUD on the same
# breakage, because it imports these names and gets AttributeError.
try:
    from guardlib.bt_lib_rules import check_l1_running  # noqa: E402,F401
    from guardlib.path_spec import applies_l1 as applies_to  # noqa: E402,F401
except Exception:  # pragma: no cover - guardlib absent
    pass


def _log(action, file_path, violations, data):
    try:
        import guard_log
        if violations:
            # collapse=True: check_l1_running returns two strings (diagnosis + "→"
            # remedy) for the single rule it has, so one event, not two.
            guard_log.log_violations(_HOOK, action, file_path, violations, data,
                                     collapse=True)
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

    try:
        from guardlib import registry
    except Exception:
        _degraded(file_path, data)
        sys.exit(0)

    specs = registry.l1_rules_for(file_path)
    if not specs:
        sys.exit(0)

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    violations = registry.run_all(specs, content, file_path)
    if not violations:
        _log('pass', file_path, (), data)
        sys.exit(0)

    _log('warned', file_path, violations, data)

    body = "\n".join(f"  {v}" for v in violations)
    context = (
        f"[xyz_bt_lib L1 guard · compliance check] {file_path} returns/uses "
        f"Status.RUNNING.\n{body}"
    )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


if __name__ == '__main__':
    main()
