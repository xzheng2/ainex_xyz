#!/usr/bin/env python3
"""One place that answers "what kind of file is this?" for the xyz guards.

The six content guards each carry their own path regex, and each one silently
decides what it is NOT responsible for. Nothing ever asked whether the union of
those decisions covers the managed packages -- so a file in a shape no guard
recognises (a new subpackage, a helper module beside the nodes) is checked by
nobody, and looks exactly like a file that passed every check.

This module makes that union nameable. `classify()` returns the category a path
belongs to, or 'UNKNOWN' for a managed-but-unrecognised file, which is the case
xyz_coverage_guard.py reports.

The patterns here are COPIES of the ones in the guards, deliberately: importing
them would make every guard a dependency of every other, and a hook that fails
to import fails open and silently stops guarding. The copies are kept honest by
a sweep in xyz_bt_lib/tools/validate_engine.py that asserts each pattern string
here is byte-identical to the guard's own. That assertion is the only thing
standing between two copies and two different answers -- if it is ever removed,
these constants become a liability rather than a map.

Module scope holds imports, re.compile and constant assignments only: validate_engine's
hooks_import_safe sweep AST-scans every hook and rejects a module-level call statement,
because importing a hook that runs main() takes the whole validator down.
"""
import re

# ── Scope ─────────────────────────────────────────────────────────────────
#: The packages whose contents the guards are responsible for. Everything else
#: (xyz_perception, xyz_run_lab, ainex_*, the repo root) is 'unmanaged': not
#: unguarded by oversight, but out of scope by design.
_MANAGED = re.compile(r'(?:^|/)(?:xyz_bt_lib|xyz_behavior)/')

#: Managed paths that no content guard should be expected to cover.
#: __init__.py is packaging, /log/ is runtime output, /tools/ is developer
#: scripts that answer to nothing, and .md is prose.
_EXEMPT = re.compile(r'(?:/__init__\.py$|/log/|/tools/|\.md$)')

#: Catkin build files. These get a REAL category rather than an _EXEMPT entry,
#: because exemption is a silent pass -- the same fail-open hole this module was
#: written to close. 'build' says "claimed, and the rule count under this
#: category is currently zero", which a coverage matrix can print; exemption says
#: nothing and disappears from the statistics.
#:
#: Not optional noise-reduction: a lane rebuilding xyz_bt_lib from seed MUST write
#: these three files, so they are mainline work, not an occasional edit. And a
#: PostToolUse additionalContext lands directly in that agent's context -- every
#: false report spends its tokens and derails it, which shows up in the data as a
#: worse engine rather than as a noisy guard.
_BUILD = re.compile(r'/(?:package\.xml|CMakeLists\.txt|setup\.py)$')

# ── Copies of the guards' own patterns (see module docstring) ─────────────
#: from xyz_bt_lib_guard.py
_NODE_PATTERN    = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L[123]_\w+/(?!__init__\.py$)[^/]+\.py$')
_ADAPTER_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/adapters/(?!__init__\.py$)[^/]+\.py$')
_BB_KEYS_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/blackboard/blackboard_keys\.py$')

#: from xyz_bt_tree_pre_guard.py
_TREE_PATTERN    = re.compile(r'/tree/[^/]*_bt\.py$')

#: from xyz_behavior_guard.py
_BEHAVIOR_PATTERN = re.compile(r'xyz_behavior/(?!log/).+\.(py|launch|xml)$')

#: from xyz_behavior_guard.py _CHECK_MAP -- substring prefixes, in its order.
#: Order is load-bearing there (first match wins per file) and is preserved here
#: so the two agree on a path that matches more than one entry.
_CHECK_PREFIXES = (
    ('runtime/_runtime_io.py',    'behavior.runtime_io'),
    ('runtime/runtime_facade.py', 'behavior.runtime_facade'),
    ('infra/infra_manifest.py',   'behavior.infra_manifest'),
    ('infra/bb_ros_bridge.py',    'behavior.bb_ros_bridge'),
    ('tree/',                     'behavior.tree'),
    ('_groot.xml',                'behavior.groot'),
    ('app/',                      'behavior.app'),
    ('scripts/',                  'behavior.script'),
    ('launch/',                   'behavior.launch'),
    ('behaviours/',               'behavior.behaviours'),
)

#: The bare prefix strings, for the validate_engine equality assertion.
_CHECK_PREFIX_STRINGS = tuple(p for p, _ in _CHECK_PREFIXES)

#: Library patterns paired with their category, most specific first.
_LIB_CATEGORIES = (
    ('lib.node',    _NODE_PATTERN),
    ('lib.adapter', _ADAPTER_PATTERN),
    ('lib.bb_keys', _BB_KEYS_PATTERN),
)


def classify(path):
    """Return the category of `path`.

    'unmanaged' -- outside xyz_bt_lib / xyz_behavior; no guard claims it.
    'exempt'    -- managed, but deliberately outside every guard's remit.
    'build'     -- catkin build files: claimed, with zero rules under the category.
    'UNKNOWN'   -- managed, not exempt, and matching no guard's pattern. This is
                   the answer that matters: a file nothing is checking.
    anything else -- the name of the category whose guard covers it.

    Total and side-effect free: every input gets an answer, nothing raises.
    """
    if not path or not _MANAGED.search(path):
        return 'unmanaged'
    if _EXEMPT.search(path):
        return 'exempt'
    if _BUILD.search(path):
        return 'build'
    for name, pattern in _LIB_CATEGORIES:
        if pattern.search(path):
            return name
    # tree/*_bt.py is claimed by xyz_bt_tree_pre_guard before xyz_behavior_guard's
    # broader 'tree/' prefix, matching which guard actually inspects it first.
    if _TREE_PATTERN.search(path):
        return 'tree'
    if _BEHAVIOR_PATTERN.search(path):
        for prefix, name in _CHECK_PREFIXES:
            if prefix in path:
                return name
    return 'UNKNOWN'
