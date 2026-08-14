"""How a BT node declares its tier, and how to select classes by it.

WHAT WAS MERGED HERE
    Three regexes that all locate the same `LEVEL = ...` class variable:

      xyz_bt_lib_guard        `^\\s{4}LEVEL\\s*=`                       (node must declare one)
      xyz_behavior_guard      `^\\s{4}LEVEL\\s*=`                       (byte-identical)
      xyz_bt_l1_running_guard `^\\s{4}LEVEL\\s*=\\s*['"]?L?1['"]?\\s*$`   (narrowed to L1)

    The third is a strict extension of the first two, so `LEVEL_DECL` is the shared
    stem and `level_value_re()` adds the value matcher. Editing the stem now changes
    all three at once, which is the point.

WHY THE VALUE MATCHER ACCEPTS BOTH QUOTED AND BARE
    The convention is `LEVEL = 'L1'`, but the regex has always tolerated `L1`, `'1'`
    and `1`. That tolerance is kept verbatim: tightening it would newly flag files
    that pass today, which is a rule change wearing a refactor's clothes.

THE FOUR-SPACE INDENT IS LOAD-BEARING
    `^\\s{4}` anchors to a class-body attribute. A module-level `LEVEL = 'L1'` does not
    match, and neither does one nested two levels deep. Every caller relies on that.
"""
import re

#: The shared stem: a class-body `LEVEL =` declaration, any value.
LEVEL_DECL = re.compile(r'^\s{4}LEVEL\s*=', re.MULTILINE)

#: Top-level class statements, used to cut a mixed-tier file into per-class blocks.
_CLASS_START = re.compile(r'^class\s+\w+', re.MULTILINE)

_LEVEL_VALUE_CACHE = {}


def _normalise(level):
    """'L1' / 'l1' / 1 / '1' all mean tier 1."""
    return str(level).strip().upper().lstrip('L')


def level_value_re(level):
    """Regex matching a class-body declaration of exactly `level`.

    For level 1 this is byte-equivalent to the pattern it replaces in
    xyz_bt_l1_running_guard.
    """
    n = _normalise(level)
    if n not in _LEVEL_VALUE_CACHE:
        _LEVEL_VALUE_CACHE[n] = re.compile(
            r'^\s{4}LEVEL\s*=\s*[\'"]?L?' + re.escape(n) + r'[\'"]?\s*$',
            re.MULTILINE)
    return _LEVEL_VALUE_CACHE[n]


def declares_level(block):
    """True if `block` declares a LEVEL class variable at all (any value)."""
    return bool(LEVEL_DECL.search(block))


def is_level(block, level):
    """True if `block` declares LEVEL of exactly `level`."""
    return bool(level_value_re(level).search(block))


def slice_by_level(src, level):
    """Concatenate the bodies of top-level classes declaring `level`.

    For a file that mixes tiers -- a project actions.py holds L2 nodes by design -- a
    whole-file scan would flag the L2 nodes for behaviour that is legitimate at their
    tier. Each block runs from its `^class` to the next one at column 0.

    Selection is by DECLARED LEVEL, not by class name: a condition called
    CheckBallVisible with `LEVEL = 'L1'` is L1 despite the missing prefix, and it is
    precisely the sloppily-named node most likely to hide an L2 dwell.
    """
    starts = [m.start() for m in _CLASS_START.finditer(src)]
    out = []
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(src)
        block = src[start:end]
        if is_level(block, level):
            out.append(block)
    return '\n'.join(out)
