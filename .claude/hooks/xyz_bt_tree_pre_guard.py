#!/usr/bin/env python3
"""PreToolUse guard: enforce tree-wiring conventions in tree/*_bt.py.

Blocks two mistakes when writing a BT wiring file (…/tree/*_bt.py):
  1. Raw py_trees composites — bare `Sequence(` / `Selector(` (incl.
     `py_trees.composites.Sequence(`). Trees must use the semantic factories
     ReactiveSequence / CommittedSequence / PrioritySelector / CommittedSelector
     from xyz_bt_lib.core.composites (the factory name states the memory
     contract; a typo becomes a NameError). The factory names themselves
     (ReactiveSequence(, PrioritySelector(, …) are allowed.
  2. A non-literal `state_key=` for LatchedDwellDecorator. state_key is a state
     identity: it MUST be a hardcoded string literal (greppable; a duplicate must
     fail loudly at construction), never a variable, arg, rosparam, or f-string.

Block = print reason to stderr + exit(2) (PreToolUse deny). Fail-open (exit 0)
on any error so the hook can never wedge editing.
"""
import json
import re
import sys

_HOOK = 'xyz_bt_tree_pre_guard'

_TREE_PATTERN = re.compile(r'/tree/[^/]*_bt\.py$')

# bare Sequence(/Selector(/Parallel( not preceded by an identifier char (so the
# ReactiveSequence/CommittedSequence/PrioritySelector/CommittedSelector/ParallelAll/
# ParallelAny factories do NOT match) — `.Sequence(` and `Sequence(` do match.
# (ParallelAll( / ParallelAny( are not matched: `Parallel` there is followed by
# `A`, not `(` or whitespace.)
_BARE_SEQ = re.compile(r'(?<![A-Za-z0-9_])(Sequence|Selector|Parallel)\s*\(')

_STATE_KEY = re.compile(r'state_key\s*=\s*([^\s,)]+)')

_STRING_RE  = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')
_COMMENT_RE = re.compile(r'#[^\n]*')


def _strip_strings_and_comments(src: str) -> str:
    return _COMMENT_RE.sub('', _STRING_RE.sub('""', src))


def _strip_comments(src: str) -> str:
    return _COMMENT_RE.sub('', src)


def _violations(content: str) -> list:
    out = []

    # 1. bare composites (ignore strings/comments so docstrings that mention the
    #    rule do not self-trigger)
    clean = _strip_strings_and_comments(content)
    hits = sorted(set(m.group(1) for m in _BARE_SEQ.finditer(clean)))
    if hits:
        out.append(
            "Raw py_trees composite(s) used: " + ", ".join(h + "(" for h in hits) + "\n"
            "  Use the semantic factories instead (xyz_bt_lib.core.composites):\n"
            "    Sequence(memory=False) -> ReactiveSequence   Sequence(memory=True) -> CommittedSequence\n"
            "    Selector(memory=False) -> PrioritySelector   Selector(memory=True) -> CommittedSelector\n"
            "    Parallel(SuccessOnAll) -> ParallelAll        Parallel(SuccessOnOne) -> ParallelAny")

    # 2. non-literal state_key (keep strings so a literal passes; drop comments so
    #    a commented-out example does not trigger)
    for m in _STATE_KEY.finditer(_strip_comments(content)):
        val = m.group(1)
        if not (val[:1] in ('"', "'")):
            out.append(
                f"Non-literal state_key={val} for LatchedDwellDecorator.\n"
                "  state_key MUST be a hardcoded string literal (e.g. state_key='grab_confirmed'),\n"
                "  never a variable / arg / rosparam / f-string — it is a greppable state identity\n"
                "  and a duplicate must fail at construction.")
            break
        if val[:2] in ('f"', "f'") or val[:2].lower() in ('rf', 'fr'):
            out.append(
                f"f-string state_key={val} for LatchedDwellDecorator.\n"
                "  state_key MUST be a plain hardcoded literal (no interpolation), so it stays greppable.")
            break
    return out


def _log(action, file_path, violations, data):
    """Record the event. Import is local and failure is swallowed: the block/allow
    decision above must not depend on the log being writable."""
    try:
        import guard_log
        if violations:
            guard_log.log_violations(_HOOK, action, file_path, violations, data)
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

    tool_input = data.get("tool_input", {})
    file_path = tool_input.get("file_path", "")
    if not _TREE_PATTERN.search(file_path):
        sys.exit(0)

    # Write → content; Edit → new_string.
    content = tool_input.get("content") or tool_input.get("new_string") or ""
    if not content:
        sys.exit(0)

    violations = _violations(content)
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
