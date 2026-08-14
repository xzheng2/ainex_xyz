"""The single answer to "what kind of file is this?".

WHAT WAS MERGED HERE
    Five tables that each classified paths, and disagreed:

      xyz_bt_lib_guard        4 patterns (node / adapter / bb_keys / runtime_io)
      xyz_bt_lib_pre_guard    the node + adapter patterns again, byte-identical
      xyz_behavior_guard      _PATTERN + _CHECK_MAP's 8 prefixes
      xyz_behavior_pre_guard  _PATTERN again + _FILE_TYPE_HINTS' 8 keys
      xyz_paths               copies of all of the above, kept honest by an assertion

    Plus one the copies never covered: xyz_bt_l1_running_guard's `_BASE_PATTERN`
    (`core/base_node.py`). The L1 rule genuinely inspects that file, but classify()
    answered UNKNOWN for it -- the coverage guard was reporting "nothing checks this"
    about a file a guard does check. It now has a category (`lib.base_node`).

WHY TWO ORDERINGS ARE KEPT, NOT ONE
    The three prefix tables were not in the same order, and their match semantics
    differed:

      _CHECK_MAP        runs EVERY matching entry  -> order does not matter
      _CHECK_PREFIXES   first match wins           -> order matters
      _FILE_TYPE_HINTS  first match wins           -> order matters, AND two pairs
                        were transposed against the others:
                        `_groot.xml` before `tree/`, `launch/` before `scripts/`

    So for `tree/foo_groot.xml` the pre-guard emitted the groot hint while classify()
    said `behavior.tree`. Collapsing to one order would silently change one of them:
    taking _CHECK_MAP's order flips the hint's rule id from `behavior_pre._groot.xml`
    to `behavior_pre.tree/` (breaking event-id continuity in guard_events.jsonl);
    taking the hint order flips classify() from `behavior.tree` to `behavior.groot`.

    Both are behaviour changes disguised as cleanup. One table, two declared traversal
    orders -- `_PREFIX_TABLE` for categories, `_HINT_ORDER` for hints -- keeps a single
    source of truth while preserving both observable behaviours exactly.

Module scope holds imports, re.compile and constant assignments only: validate_engine's
hooks_import_safe sweep AST-rejects a module-level call statement.
"""
import re

# ── Scope ─────────────────────────────────────────────────────────────────
#: The packages the guards are responsible for. Everything else (xyz_perception,
#: xyz_run_lab, ainex_*, the repo root) is 'unmanaged': out of scope by design, not
#: by oversight.
MANAGED = re.compile(r'(?:^|/)(?:xyz_bt_lib|xyz_behavior)/')

#: Managed paths no content guard should be expected to cover. __init__.py is
#: packaging, /log/ is runtime output, /tools/ is developer scripts that answer to
#: nothing, .md is prose.
EXEMPT = re.compile(r'(?:/__init__\.py$|/log/|/tools/|\.md$)')

#: Catkin build files get a REAL category rather than an exemption, because exemption
#: is a silent pass -- the same hole this module exists to close. 'build' says
#: "claimed, zero rules under it", which a coverage matrix can print; exemption says
#: nothing and vanishes from the statistics.
BUILD = re.compile(r'/(?:package\.xml|CMakeLists\.txt|setup\.py)$')

# ── Library shapes ────────────────────────────────────────────────────────
NODE_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L[123]_\w+/(?!__init__\.py$)[^/]+\.py$')
ADAPTER_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/adapters/(?!__init__\.py$)[^/]+\.py$')
BB_KEYS_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/blackboard/blackboard_keys\.py$')

#: The shared _RuntimeIO, formerly scaffolded per project. Without an entry it would
#: classify UNKNOWN like the rest of core/, and the BuzzerState rule would stop
#: covering the file that actually holds the code.
RUNTIME_IO_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/core/default_runtime_io\.py$')

#: L1 territory. The lib half is narrower than NODE_PATTERN (L1 only, not L[123]) --
#: L2/L3 nodes return RUNNING legitimately, so the L1 purity rule must not see them.
L1_LIB_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L1_\w+/(?!__init__\.py$)[^/]+\.py$')

#: A project's actions.py mixes tiers by design; the rule slices it per class.
L1_PROJECT_PATTERN = re.compile(r'xyz_behavior/.+/behaviours/actions\.py$')

#: base_node.py defines all three tier bases, so only its L1 class body is L1 territory.
BASE_NODE_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/core/base_node\.py$')

# ── Project shapes ────────────────────────────────────────────────────────
TREE_PATTERN = re.compile(r'/tree/[^/]*_bt\.py$')
BEHAVIOR_PATTERN = re.compile(r'xyz_behavior/(?!log/).+\.(py|launch|xml)$')

#: One table, in _CHECK_MAP's order. See the module docstring for why the hint
#: traversal order below is different rather than this being reordered.
_PREFIX_TABLE = (
    ('runtime/_runtime_io.py',    'behavior.runtime_io'),
    ('runtime/runtime_facade.py', 'behavior.runtime_facade'),
    ('tree/',                     'behavior.tree'),
    ('_groot.xml',                'behavior.groot'),
    ('app/',                      'behavior.app'),
    ('scripts/',                  'behavior.script'),
    ('launch/',                   'behavior.launch'),
    ('behaviours/',               'behavior.behaviours'),
)

#: The order xyz_behavior_pre_guard's _FILE_TYPE_HINTS declared its keys in. Differs
#: from _PREFIX_TABLE for exactly two pairs (_groot.xml/tree/ and launch/scripts/),
#: and both tables are first-match-wins, so the difference is observable.
_HINT_ORDER = (
    'runtime/_runtime_io.py',
    'runtime/runtime_facade.py',
    '_groot.xml',
    'tree/',
    'app/',
    'launch/',
    'scripts/',
    'behaviours/',
)

#: Library patterns paired with their category, most specific first.
_LIB_CATEGORIES = (
    ('lib.node',      NODE_PATTERN),
    ('lib.adapter',   ADAPTER_PATTERN),
    ('lib.bb_keys',   BB_KEYS_PATTERN),
    ('lib.runtime_io', RUNTIME_IO_PATTERN),
    ('lib.base_node', BASE_NODE_PATTERN),
)


def applies_l1(path):
    """True if the L1 purity rule is responsible for `path`.

    Spans three categories (lib.node's L1 subset, behavior.behaviours, lib.base_node),
    which is why rule selection cannot be a plain category lookup.
    """
    return bool(L1_LIB_PATTERN.search(path)
                or L1_PROJECT_PATTERN.search(path)
                or BASE_NODE_PATTERN.search(path))


def classify(path):
    """Return the category of `path`.

    'unmanaged' -- outside xyz_bt_lib / xyz_behavior; no guard claims it.
    'exempt'    -- managed, but deliberately outside every guard's remit.
    'build'     -- catkin build files: claimed, zero rules under the category.
    'UNKNOWN'   -- managed, not exempt, matching no guard's shape. The answer that
                   matters: a file nothing is checking.
    anything else -- the category whose guard covers it.

    Total and side-effect free: every input gets an answer, nothing raises.
    """
    if not path or not MANAGED.search(path):
        return 'unmanaged'
    if EXEMPT.search(path):
        return 'exempt'
    if BUILD.search(path):
        return 'build'
    for name, pattern in _LIB_CATEGORIES:
        if pattern.search(path):
            return name
    # tree/*_bt.py is claimed by xyz_bt_tree_pre_guard before xyz_behavior_guard's
    # broader 'tree/' prefix, matching which guard actually inspects it first.
    if TREE_PATTERN.search(path):
        return 'tree'
    if BEHAVIOR_PATTERN.search(path):
        for prefix, name in _PREFIX_TABLE:
            if prefix in path:
                return name
    return 'UNKNOWN'


def hint_key(path):
    """Return the pre-write hint key for `path`, or None if no hint applies.

    Traverses _HINT_ORDER, not _PREFIX_TABLE -- see the module docstring. The caller
    turns this into the rule id `behavior_pre.<key>`, so the key strings are part of
    the recorded event vocabulary and cannot be renamed.
    """
    if not BEHAVIOR_PATTERN.search(path):
        return None
    for prefix in _HINT_ORDER:
        if prefix in path:
            return prefix
    return None


def matchers():
    """Ordered (name, predicate) pairs for offline file selection.

    Used by validate_engine's sweeps, which need "every file this rule applies to"
    rather than "the one category this file is in". The two questions differ: the L1
    rule spans three categories, and lib.node covers L2/L3 files the L1 rule must not
    see.
    """
    return (
        ('lib.node',       NODE_PATTERN.search),
        ('lib.adapter',    ADAPTER_PATTERN.search),
        ('lib.bb_keys',    BB_KEYS_PATTERN.search),
        ('lib.runtime_io', RUNTIME_IO_PATTERN.search),
        ('l1',             applies_l1),
    )
