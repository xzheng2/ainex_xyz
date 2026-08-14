"""Rules for tree wiring files, the only ones that BLOCK a write.

Function body lifted verbatim from xyz_bt_tree_pre_guard. This module keeps the
blocking semantics: its caller exits 2, it is not advisory. Downgrading it would let a
raw composite reach a file that the whole composites contract exists to keep it out of.

`_strip_comments` is used instead of `strip_noise` in the state_key rule ON PURPOSE:
that rule has to SEE string literals, because a literal `state_key='grab_confirmed'` is
what passing looks like. Blanking strings first would make every literal read as a
violation.
"""
from __future__ import annotations

import re

from guardlib.text_utils import strip_comments as _strip_comments
from guardlib.text_utils import strip_noise as _strip_strings_and_comments

# bare Sequence(/Selector(/Parallel( not preceded by an identifier char (so the
# ReactiveSequence/CommittedSequence/PrioritySelector/CommittedSelector/ParallelAll/
# ParallelAny factories do NOT match) — `.Sequence(` and `Sequence(` do match.
# (ParallelAll( / ParallelAny( are not matched: `Parallel` there is followed by
# `A`, not `(` or whitespace.)
_BARE_SEQ = re.compile(r'(?<![A-Za-z0-9_])(Sequence|Selector|Parallel)\s*\(')

_STATE_KEY = re.compile(r'state_key\s*=\s*([^\s,)]+)')

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
