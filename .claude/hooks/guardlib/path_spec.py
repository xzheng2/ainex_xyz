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

    The `runtime_io` entry in that first row is gone: `core/default_runtime_io.py`
    moved to `xyz_behavior/bt_ros_io/`, so the file it selected is no longer in
    xyz_bt_lib and the library guard no longer borrows the project rule for it. The
    rule function itself never moved -- it has always lived in behavior_rules.

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
#:
#: The bt_ros_io/ pair is the SHARED binding layer (the concrete _RuntimeIO and
#: RuntimeFacade, moved out of xyz_bt_lib/core so core/ stops at the facade ABC).
#:
#: `runtime/_runtime_io.py` stays alongside it: _RuntimeIO's knobs are list constants
#: (_GO_STEP_VEL / _TURN_STEP_VEL), so a project retunes them by subclassing, and that
#: subclass is genuinely a behavior.runtime_io file. check_runtime_io inspects imports,
#: not base classes, so it passes a subclass cleanly -- verified.
#:
#: `runtime/runtime_facade.py` was RETIRED, not kept. The facade's only per-project
#: knobs are constructor arguments now, so no project has any reason to write that
#: file, and the entry would have been a mapping with no legal content behind it.
#: Worse than idle: check_runtime_facade's pattern demands XyzBTFacade itself in the
#: base list, so the one thing anybody would plausibly have put there --
#: `class ProjRuntimeFacade(XyzRuntimeFacade)` -- was reported as a violation. A path
#: key whose only reachable content is a false positive is worse than no key: if
#: someone writes that file anyway, classify() now answers UNKNOWN and
#: xyz_coverage_guard says so out loud, which is the honest answer.
_PREFIX_TABLE = (
    ('bt_ros_io/default_runtime_io.py', 'behavior.runtime_io'),
    ('bt_ros_io/runtime_facade.py',     'behavior.runtime_facade'),
    ('runtime/_runtime_io.py',          'behavior.runtime_io'),
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
    'bt_ros_io/default_runtime_io.py',
    'bt_ros_io/runtime_facade.py',
    'runtime/_runtime_io.py',
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
    ('lib.base_node', BASE_NODE_PATTERN),
)

#: Categories classify() can return that no content rule covers, and that none is
#: expected to. A SEPARATE table from _PREFIX_TABLE, for two reasons: registry.py feeds
#: _PREFIX_TABLE straight into _BEHAVIOR_FNS[category], so an entry there without a rule
#: function is a KeyError at import -- and one WITH a stub function would make
#: categories_with_rules() count these as `checked`, which is a lie and exactly the
#: "zero means both" collapse the coverage matrix exists to stop. Landing here instead
#: reports them as `declared but unchecked`: named, counted, and visibly ruleless.
#:
#: WHY THESE ARE RULELESS BY DESIGN, not pending work. Every rule in this repo checks a
#: REPEATED SHAPE -- 27 nodes must each declare LEVEL, 6 adapters must each implement
#: the two-phase latch. That works because the shape gets copied, so forgetting a part
#: of it is a real and recurring mistake. Each file below is a one-off implementation
#: with no second instance to be consistent with, so a "rule" for it could only restate
#: it: finding nothing, and false-positiving the moment it legitimately evolves. This is
#: not hypothetical -- the facade-subclass rule retired in the same commit as this table
#: was added had exactly one reachable match, and that match was a false positive.
#:
#: They are not untested, either. Their coverage is executional and stronger than any
#: pattern match: seed closure imports core/ file by file under a blocking meta_path
#: finder, RUNNABLE_DEMOS runs the demos on every push, catkin builds msg/.
#:
#: Revisit when a SECOND instance of one of these shapes appears -- not because the
#: count looks untidy.
#:
#: DIRECTORY PREFIXES, AND THAT IS A DELIBERATE TRADE-OFF. A file added to one of these
#: directories later lands here silently too, instead of surfacing as UNKNOWN the way it
#: would today. Chosen over enumerating the 23 paths because these directories are
#: homogeneous and the table would otherwise need an edit per file. The cost is real:
#: core/ is where the framework grows, so a genuinely new engine module arrives
#: unannounced. If that ever bites, split core/ out into an explicit file list and leave
#: the rest as prefixes.
#:
#: NOT listed, on purpose: xyz_behavior/bt_ros_io/. Both its files have real rules
#: (check_runtime_io / check_runtime_facade) and count as `checked`; adding the directory
#: here would retire them. A new, un-ruled file in bt_ros_io/ SHOULD read UNKNOWN -- that
#: is the binding layer, and something unclaimed there is worth being told about.
_RULELESS_PREFIX_TABLE = (
    ('xyz_behavior/bt_observability/',        'behavior.observability'),
    ('xyz_bt_lib/examples/',                  'lib.example'),
    ('xyz_bt_lib/msg/',                       'lib.msg'),
    ('xyz_bt_lib/src/xyz_bt_lib/core/',       'lib.core'),
    ('xyz_bt_lib/src/xyz_bt_lib/blackboard/', 'lib.blackboard'),
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
                   matters: a file nothing is checking, and nobody has said whether
                   that is intended.
    anything else -- a named category. It may still have zero rules: the ones from
                   _RULELESS_PREFIX_TABLE are ruleless ON PURPOSE, and the coverage
                   matrix reports them as "declared but unchecked". That is the whole
                   distinction this function exists to draw -- "nobody looked" and
                   "we decided nobody needs to" must not produce the same output.

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
    # Last, so a real rule always wins: every ruled shape above has already had its
    # chance, and base_node.py / blackboard_keys.py in particular are claimed by
    # _LIB_CATEGORIES before the broader core/ and blackboard/ prefixes here.
    for prefix, name in _RULELESS_PREFIX_TABLE:
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
        ('lib.node',    NODE_PATTERN.search),
        ('lib.adapter', ADAPTER_PATTERN.search),
        ('lib.bb_keys', BB_KEYS_PATTERN.search),
        ('l1',          applies_l1),
    )
