#!/usr/bin/env python3
"""PostToolUse guard: L1 condition nodes must be pure predicates (no Status.RUNNING).

Fires after a write to behaviours/L1_*/... . L1 nodes answer a yes/no question and
must return only SUCCESS or FAILURE — never RUNNING, and hold no cross-tick state.
A `Status.RUNNING` in an L1 file is the tell-tale of an "L1 in name, L2 in behaviour"
node (a dwell/hysteresis smuggled into a condition). Stability confirmation belongs
to a tree-layer LatchedDwellDecorator instead.

Reports via additionalContext (same advisory style as xyz_bt_lib_guard.py).
Fail-open (exit 0) on any error.
"""
import json
import re
import sys

_HOOK = 'xyz_bt_l1_running_guard'

#: The project-level behaviours file. The scaffold defines it as L2 territory
#: (actions.py.tpl: "Must inherit XyzL2ActionNode"), and an L2 node returning
#: RUNNING is correct — 8 of the library's 15 L2 nodes do. But nothing stops an
#: L1 condition being written here too, and that one would otherwise be checked
#: by nobody. So the file is in scope while only its L1_* classes are scanned:
#: scanning it whole would fire this guard on correct L2 code, and a guard that
#: cries wolf on the canonical content of a file gets ignored on the day it is right.
_PROJECT_SRC     = r'xyz_behavior/.+/behaviours/actions\.py$'
_PROJECT_PATTERN = re.compile(_PROJECT_SRC)

_L1_PATTERN   = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L1_\w+/(?!__init__\.py$)[^/]+\.py$'
                           r'|' + _PROJECT_SRC)
_BASE_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/core/base_node\.py$')

_STRING_RE  = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')
_COMMENT_RE = re.compile(r'#[^\n]*')
_RUNNING_RE = re.compile(r'(?<![A-Za-z0-9_.])Status\.RUNNING(?![A-Za-z0-9_])')


def _strip_noise(src: str) -> str:
    return _COMMENT_RE.sub('', _STRING_RE.sub('""', src))


def _l1_class_slice(src: str) -> str:
    """Return only the XyzL1ConditionNode class body from base_node.py.

    The base file also defines XyzL2ActionNode / XyzL3ActionNode; slicing to the
    L1 class avoids flagging RUNNING that legitimately belongs to another tier.
    """
    m = re.search(r'^class\s+XyzL1ConditionNode\b', src, re.MULTILINE)
    if not m:
        return ''
    rest = src[m.end():]
    nxt = re.search(r'^class\s+\w+', rest, re.MULTILINE)
    return rest[:nxt.start()] if nxt else rest


#: What makes a class L1 is its declared LEVEL, not its name. The other guards agree:
#: both xyz_bt_lib_guard and xyz_behavior_guard locate the tier with `^\s{4}LEVEL\s*=`,
#: so this matches the same declaration and only narrows it to the L1 value. Slicing by
#: class name instead would skip a node called CheckBallVisible(XyzL1ConditionNode) --
#: correctly declared LEVEL = 'L1', wrong prefix -- which is precisely the sloppily-named
#: node most likely to have an L2 dwell smuggled into it.
_LEVEL_L1_RE = re.compile(r'^\s{4}LEVEL\s*=\s*[\'"]?L?1[\'"]?\s*$', re.MULTILINE)


def _l1_classes(src: str) -> str:
    """Return the bodies of top-level classes declaring LEVEL = 'L1', concatenated.

    Applied to a file that mixes tiers: a project actions.py holds L2 nodes by
    design, so everything outside an L1-declared class is out of jurisdiction.
    Each block runs from its `^class` to the next one at column 0.
    """
    starts = [m.start() for m in re.finditer(r'^class\s+\w+', src, re.MULTILINE)]
    out = []
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(src)
        block = src[start:end]
        if _LEVEL_L1_RE.search(block):
            out.append(block)
    return '\n'.join(out)


def _l1_scope(src: str, file_path: str) -> str:
    """The slice of `src` this guard is entitled to judge."""
    if _BASE_PATTERN.search(file_path):
        # L2/L3 base classes may legitimately reference RUNNING.
        return _l1_class_slice(src)
    if _PROJECT_PATTERN.search(file_path):
        return _l1_classes(src)
    # A file under behaviours/L1_*/ is L1 all the way down.
    return src


def applies_to(file_path: str) -> bool:
    """True if `file_path` is L1 territory this guard is responsible for."""
    return bool(_L1_PATTERN.search(file_path) or _BASE_PATTERN.search(file_path))


def check_l1_running(content: str, file_path: str = '') -> list:
    """Return violation strings for L1 purity. Empty list means clean.

    Exposed as a function (mirroring xyz_bt_lib_guard.check_node) so the same
    rule can run outside the hook path — hooks only fire when the AGENT writes,
    so a human edit is otherwise unchecked. tools/validate_engine.py sweeps the
    whole library through this.
    """
    scan = _l1_scope(content, file_path)
    if not _RUNNING_RE.search(_strip_noise(scan)):
        return []
    return [
        "❌ L1 condition nodes are PURE PREDICATES: SUCCESS/FAILURE only, never "
        "RUNNING, no cross-tick state.",
        "→  Remove the RUNNING path. For \"condition stable for N ticks\", wrap "
        "this node at the tree layer in "
        "xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator (BB-backed, "
        "reactive-safe) — do not dwell inside the L1 node.",
    ]


def _log(action, file_path, violations, data):
    try:
        import guard_log
        if violations:
            guard_log.log_violations(_HOOK, action, file_path, violations, data,
                                     collapse=True)
        else:
            guard_log.log_clean(_HOOK, file_path, data)
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
    if not applies_to(file_path):
        sys.exit(0)

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    violations = check_l1_running(content, file_path)
    if not violations:
        _log('pass', file_path, (), data)
        sys.exit(0)

    # collapse=True: check_l1_running returns two strings (diagnosis + "→" remedy) for
    # the single rule it has, so one event, not two.
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
