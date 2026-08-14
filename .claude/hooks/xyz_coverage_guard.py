#!/usr/bin/env python3
"""PostToolUse guard: every file in a managed package must fall to some guard.

The other six guards check CONTENT. This one checks only that a written path is
classifiable at all: if path_spec.classify() returns 'UNKNOWN', the file sits
inside xyz_bt_lib / xyz_behavior, is not exempt, and matches no guard's pattern
— so nothing inspected it, and its silence in guard_events.jsonl means "unseen",
not "clean". That distinction is invisible without this hook.

Deliberately never opens the file. Reading content would make it a seventh
content guard; its whole subject is the classification, and a file that no guard
recognises has no rules to read it against yet.

Reports via additionalContext (same advisory style as xyz_bt_l1_running_guard.py).
Fail-open (exit 0) on any error.
"""
import json
import sys

_HOOK = 'xyz_coverage_guard'


def _degraded(file_path, data):
    try:
        import guard_log
        guard_log.log_degraded(_HOOK, file_path, data=data)
    except Exception:
        pass


def _log(action, file_path, violations, data, category=None):
    try:
        import guard_log
        if violations:
            guard_log.log_violations(_HOOK, action, file_path, violations, data,
                                     collapse=True)
        else:
            guard_log.log_clean(_HOOK, file_path, data,
                                rule='coverage.' + category if category else None)
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
        from guardlib import path_spec
        category = path_spec.classify(file_path)
    except Exception:
        # Fail open: a coverage guard that cannot classify must not block an edit.
        # But RECORD it: an unlogged failure here is the exact blind spot this hook
        # exists to remove -- silence would read as "everything was classifiable".
        _degraded(file_path, data)
        sys.exit(0)

    if category != 'UNKNOWN':
        # 'unmanaged' is every edit made anywhere else in the repo; logging it would
        # bury the real signal. 'exempt' and every real category (including 'build',
        # which currently has zero rules) ARE logged, with the category as the rule --
        # that attribution is what lets a coverage matrix separate "checked and clean"
        # from "claimed but unchecked".
        if category != 'unmanaged':
            _log('pass', file_path, (), data, category=category)
        sys.exit(0)

    violations = [
        "Unclassified file in a managed package: no content guard's path pattern "
        "matches it, so nothing checked what you just wrote.",
        "→  Either move it under a shape an existing guard claims (behaviours/L[123]_*/, "
        "adapters/, tree/, runtime/, app/, scripts/, launch/), or — if it is a "
        "genuinely new kind of module — add its pattern to "
        ".claude/hooks/guardlib/path_spec.py and give it a guard.",
    ]
    _log('warned', file_path, violations, data)

    body = "\n".join(f"  {v}" for v in violations)
    context = (
        f"[xyz coverage guard · classification check] {file_path} is in a managed "
        f"package but belongs to no guarded category.\n{body}"
    )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


if __name__ == '__main__':
    main()
