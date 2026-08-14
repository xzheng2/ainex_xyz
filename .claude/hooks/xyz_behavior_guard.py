#!/usr/bin/env python3
"""PostToolUse hook: check xyz_behavior project file compliance.

Thin wrapper. The rules live in guardlib.behavior_rules, the shape-to-rule binding in
guardlib.registry.

Unlike the library guard this one runs EVERY matching rule, not the first: a file at
tree/foo_groot.xml matches both the `tree/` and `_groot.xml` prefixes and is genuinely
subject to both. registry.behavior_rules_for() preserves that.
"""
import json
import sys

_HOOK = 'xyz_behavior_guard'

# Re-exports for external callers (validate_engine.py's library_sweep and the skill
# validators import these by name). Wrapped because this module must remain IMPORTABLE
# when guardlib is missing: main() then fails open and records a `degraded` event. An
# unguarded import here would raise at module scope, and the hook would die with a
# traceback instead of stepping aside -- exit 1 with no event, which is the one outcome
# the fail-open rule exists to prevent. validate_engine still fails LOUD on the same
# breakage, because it imports these names and gets AttributeError.
try:
    from guardlib.behavior_rules import (  # noqa: E402,F401
        check_runtime_io, check_runtime_facade, check_tree_bt, check_groot_xml,
        check_app_bt_node, check_script_bt_node, check_launch, check_behaviours,
    )
    from guardlib.path_spec import BEHAVIOR_PATTERN as _PATTERN  # noqa: E402,F401
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

    file_path = data.get("tool_input", {}).get("file_path", "")

    # Coarse prefilter, deliberately NOT the real gate: it exists only so a broken
    # guardlib does not emit a `degraded` event for every edit anywhere in the repo.
    # A strictly weaker necessary condition than BEHAVIOR_PATTERN, so it can never
    # admit a file the authoritative check below would reject, nor reject one it
    # would admit.
    if 'xyz_behavior/' not in file_path:
        sys.exit(0)

    try:
        from guardlib import path_spec, registry
    except Exception:
        _degraded(file_path, data)
        sys.exit(0)

    if not path_spec.BEHAVIOR_PATTERN.search(file_path):
        sys.exit(0)

    specs = registry.behavior_rules_for(file_path)
    # Inside xyz_behavior/ but matching no rule shape: exit WITHOUT logging. That file
    # is xyz_coverage_guard's subject, not this hook's -- logging a pass here would
    # claim a check that never ran.
    if not specs:
        sys.exit(0)

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    all_violations = registry.run_all(specs, content, file_path)

    if not all_violations:
        _log('pass', file_path, (), data)
        sys.exit(0)

    _log('warned', file_path, all_violations, data)

    lines = "\n".join(f"  {v}" for v in all_violations)
    context = (
        f"[xyz_behavior guard · compliance check] {file_path} has {len(all_violations)} violation(s):\n"
        f"{lines}\n"
        "Please fix the issues above before proceeding — must comply with the xyz-bt-facade-project skill."
    )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


if __name__ == '__main__':
    main()
