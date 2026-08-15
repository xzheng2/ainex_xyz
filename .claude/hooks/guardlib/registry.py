"""Which rules run on which file, for which hook.

This is the last of the duplicated tables. The dispatch used to live in three places
that had to agree: xyz_behavior_guard._CHECK_MAP, xyz_paths._CHECK_PREFIXES, and the
if/elif chain in xyz_bt_lib_guard.main(). The prefixes moved to path_spec; the binding
of shape -> rule function lives here.

TWO DISPATCH MODES, BOTH PRESERVED
    The hooks did not agree on what "which rules apply" means, and the difference is
    observable:

      xyz_bt_lib_guard    an if/elif chain -- exactly ONE rule runs, first match wins
      xyz_behavior_guard  a for loop with extend() -- EVERY matching rule runs

    A file at `.../tree/foo_groot.xml` matches both the `tree/` and `_groot.xml`
    prefixes and genuinely gets both checks today. Collapsing to first-match would
    silently drop check_groot_xml's four rules on every groot file. So `rules_for()`
    returns a list and the mode is declared per family: FIRST for the library, ALL for
    the project.

ARITY IS NOT UNIFORM, AND IS NOT UNIFIED HERE
    check_adapter(content) and check_bb_keys(content) take no file_path; the others
    take (content, file_path). Adding a defaulted parameter would edit signatures that
    validate_engine.py and two skill validators call positionally. The table records
    `takes_path` instead, so the rule functions stay exactly as their callers expect.

Module scope holds imports, constants and tuple assembly only -- no calls. See
validate_engine.hooks_import_safe.
"""
from __future__ import annotations

from guardlib import behavior_rules as _beh
from guardlib import bt_lib_rules as _lib
from guardlib import path_spec as _ps
from guardlib import tree_pre_rules as _tree

#: A rule: (name, applies-predicate, function, takes_path).
#: `name` is diagnostic only -- the recorded rule id comes from the violation text via
#: guard_log.rule_id(), never from here.

#: xyz_bt_lib_guard: FIRST match only. Order is the original if/elif order --
#: node, adapter, then bb_keys as the fallback.
#:
#: There used to be a fourth entry borrowing behavior_rules.check_runtime_io for
#: xyz_bt_lib/core/default_runtime_io.py. That file moved to xyz_behavior/bt_ros_io/,
#: where xyz_behavior_guard reaches it through BEHAVIOR_RULES with the same function
#: and the same violation strings -- so the rule ids are unchanged, only the hook
#: that reports them.
LIB_RULES = (
    ('check_node',    _ps.NODE_PATTERN.search,    _lib.check_node,    True),
    ('check_adapter', _ps.ADAPTER_PATTERN.search, _lib.check_adapter, False),
    ('check_bb_keys', _ps.BB_KEYS_PATTERN.search, _lib.check_bb_keys, False),
)

#: xyz_behavior_guard: EVERY match. Prefix order comes from path_spec, which is the
#: same order _CHECK_MAP declared -- irrelevant to the result here (all matches run)
#: but kept so the two tables stay readable side by side.
_BEHAVIOR_FNS = {
    'behavior.runtime_io':     _beh.check_runtime_io,
    'behavior.runtime_facade': _beh.check_runtime_facade,
    'behavior.tree':           _beh.check_tree_bt,
    'behavior.groot':          _beh.check_groot_xml,
    'behavior.app':            _beh.check_app_bt_node,
    'behavior.script':         _beh.check_script_bt_node,
    'behavior.launch':         _beh.check_launch,
    'behavior.behaviours':     _beh.check_behaviours,
}

BEHAVIOR_RULES = tuple(
    (fn.__name__, prefix, fn, True)
    for prefix, category in _ps._PREFIX_TABLE
    for fn in (_BEHAVIOR_FNS[category],)
)

#: xyz_bt_l1_running_guard: one rule, spanning three categories. Its two returned
#: strings are one logical violation -- the caller collapses them into one event.
L1_RULES = (
    ('check_l1_running', _ps.applies_l1, _lib.check_l1_running, True),
)

#: xyz_bt_tree_pre_guard: one rule, and the only one that BLOCKS.
TREE_RULES = (
    ('tree_violations', _ps.TREE_PATTERN.search, _tree._violations, False),
)


def _run(spec, content, file_path):
    _name, _match, fn, takes_path = spec
    return fn(content, file_path) if takes_path else fn(content)


def lib_rules_for(file_path):
    """The single library rule that claims `file_path`, or []. First match wins."""
    for spec in LIB_RULES:
        if spec[1](file_path):
            return [spec]
    return []


def behavior_rules_for(file_path):
    """EVERY project rule whose prefix is in `file_path`, in table order.

    An empty list means no prefix matched. That is distinct from "matched and clean":
    xyz_behavior_guard exits without logging in that case, because the file is inside
    xyz_behavior/ but is not a shape any rule claims -- which is xyz_coverage_guard's
    subject, not this hook's.
    """
    return [spec for spec in BEHAVIOR_RULES if spec[1] in file_path]


def l1_rules_for(file_path):
    return [spec for spec in L1_RULES if spec[1](file_path)]


def tree_rules_for(file_path):
    return [spec for spec in TREE_RULES if spec[1](file_path)]


def run_all(specs, content, file_path):
    """Run `specs` in order and concatenate their violations."""
    out = []
    for spec in specs:
        out.extend(_run(spec, content, file_path))
    return out


#: Categories that classify() can return but no rule covers. Recorded so a coverage
#: matrix can print "claimed, zero rules" rather than leaving them indistinguishable
#: from unchecked files.
RULELESS_CATEGORIES = ('build', 'exempt')


def categories_with_rules():
    """Every category some rule family covers. Used by the coverage matrix."""
    names = {'lib.node', 'lib.adapter', 'lib.bb_keys', 'lib.base_node', 'tree'}
    names.update(category for _prefix, category in _ps._PREFIX_TABLE)
    return frozenset(names)
