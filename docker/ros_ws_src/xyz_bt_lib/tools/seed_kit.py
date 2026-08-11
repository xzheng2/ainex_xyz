#!/usr/bin/env python3
# encoding: utf-8
"""seed_kit.py - portable-seed manifest and import-isolation harness.

WHAT THE SEED IS
    The "seed" is the minimum set of xyz_bt_lib files that must travel with the
    skill templates so that the templates remain usable when the node library
    (behaviours/, adapters/, sensors/) does not exist -- a fresh robot, or this
    repository stripped back to its engine layer.

    Templates REFERENCE these base classes; they never copy them. That is why
    the seed and the templates are one deployable unit.

WHY THIS FILE LIVES IN THE LIBRARY, NOT IN A SKILL
    "Which files constitute the seed" is knowledge about the library, and it has
    to move to a new robot together with core/. The skill validators that consume
    it (.claude/skills/xyz-bt-lib-{node,adapter}/validate_templates.py) own the
    other half: "do MY templates still work against that seed".

TIER A -- BT contract layer, import-time ROS-free
    core/base_facade.py       abc only
    core/base_node.py         py_trees
    core/composites.py        py_trees
    core/latched_dwell.py     py_trees
    core/hysteresis.py        py_trees
    core/stub_facade.py       base_facade
    core/tree_render.py       py_trees
    core/process_manager.py   stdlib
    core/base_adapter.py      py_trees + blackboard_keys.  ROS-free AT IMPORT
                              TIME: rospy/numpy/sensor_msgs are lazy-imported
                              inside the depth-sampling class, so the adapter
                              base contract itself is portable.
    blackboard/blackboard_keys.py

TIER B -- ROS1-bound
    core/bt_runner.py         rospy + xyz_bt_lib.msg (a catkin build product,
                              so it cannot be copied as a plain source file)

    Tier B also covers the GENERATED adapter: input_adapter.py.tpl owns
    rospy.Subscriber by design, because an input adapter IS the ROS boundary.
    That binding is honest and deliberate -- see the adapter skill.

WHY THE SEED SHIPS ITS OWN __init__.py
    src/xyz_bt_lib/__init__.py is an eager aggregator: it imports bt_runner and
    bb_ros_bridge, both of which need rospy. Importing ANY submodule of the
    installed package therefore drags in ROS. build_seed_tree() writes EMPTY
    package __init__.py files instead, which is what makes Tier A genuinely
    ROS-free rather than only nominally so.

ISOLATION MECHANISM
    seed_isolation() does not merely "not import" the node library -- it makes
    the library unreachable and lets CPython's own import machinery be the
    oracle:
      1. purge every already-imported xyz_bt_lib* module from sys.modules
      2. drop every real library source dir from sys.path
      3. prepend the temporary seed dir, so `xyz_bt_lib` resolves to the seed
         and its __path__ contains nothing else
      4. install a sys.meta_path finder that raises for forbidden modules
    Anything the rendered template references outside the seed then fails with
    ModuleNotFoundError. That is proof, not inspection.

TYPICAL USE

    import seed_kit
    with tempfile.TemporaryDirectory() as tmp:
        seed = seed_kit.build_seed_tree(tmp, tier='A')
        src = seed_kit.render(tpl_text, {'CLASS_NAME': 'L1_Probe', ...})
        open(os.path.join(seed, 'probe.py'), 'w').write(src)
        with seed_kit.seed_isolation(seed, tier='A'):
            mod = importlib.import_module('probe')
            ...
"""
import contextlib
import importlib.abc
import os
import re
import shutil
import sys
import types

# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------

#: xyz_bt_lib/src, derived from this file so the kit still works after the
#: whole tree is relocated to another robot.
LIB_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')

#: Tier A: core modules that import cleanly with ROS blocked.
TIER_A_CORE = (
    'base_facade.py',
    'base_node.py',
    'composites.py',
    'latched_dwell.py',
    'hysteresis.py',
    'stub_facade.py',
    'tree_render.py',
    'process_manager.py',
    'base_adapter.py',
)

#: Tier B: ROS1-bound core modules. bt_runner additionally depends on
#: xyz_bt_lib.msg, a catkin build product, so it is never copied into a seed
#: tree; it is listed here to document the boundary, not to be shipped.
TIER_B_CORE = (
    'bt_runner.py',
)

#: Blackboard key constants. Part of Tier A: the BB namespace is contract, not
#: transport, and both node and adapter templates import it.
TIER_A_BLACKBOARD = (
    'blackboard_keys.py',
)

#: Templates each tier is responsible for, relative to a skill's
#: assets/templates/ dir. Recorded here so the manifest describes the whole
#: deployable unit, not just its library half.
TIER_A_TEMPLATES = (
    'xyz-bt-lib-node/assets/templates/l1_node.py.tpl',
    'xyz-bt-lib-node/assets/templates/l2_node.py.tpl',
    'xyz-bt-lib-node/assets/templates/l3_node.py.tpl',
)
TIER_B_TEMPLATES = (
    'xyz-bt-lib-adapter/assets/templates/input_adapter.py.tpl',
)

#: Never importable inside a seed test, at any tier. These are exactly the
#: parts of xyz_bt_lib that a portable template must not depend on.
_ALWAYS_BLOCKED = (
    'xyz_bt_lib.behaviours',
    'xyz_bt_lib.adapters',
    'xyz_bt_lib.sensors',
    'xyz_bt_lib.msg',
)

#: Additionally blocked at Tier A, to prove import-time ROS independence.
_TIER_A_BLOCKED = (
    'rospy',
    'rosgraph',
    'roslib',
    'sensor_msgs',
    'std_msgs',
    'geometry_msgs',
    'cv_bridge',
)


def blocked_modules(tier='A'):
    """Return the module prefixes that must be unimportable for `tier`."""
    tier = tier.upper()
    if tier == 'A':
        return _ALWAYS_BLOCKED + _TIER_A_BLOCKED
    if tier == 'B':
        # Tier B may touch ROS, but through injected stubs (install_ros_stubs),
        # never through a live ROS install -- the test must not need a master.
        return _ALWAYS_BLOCKED
    raise ValueError("tier must be 'A' or 'B', got %r" % (tier,))


def seed_manifest(tier='A', lib_src=None):
    """Return [(relative_dest, absolute_source), ...] for the given tier.

    Tier B is cumulative: it contains everything Tier A contains, because an
    adapter still needs the BT contract layer.
    """
    lib_src = lib_src or LIB_SRC
    tier = tier.upper()
    if tier not in ('A', 'B'):
        raise ValueError("tier must be 'A' or 'B', got %r" % (tier,))

    items = []
    for fname in TIER_A_CORE:
        items.append((os.path.join('xyz_bt_lib', 'core', fname),
                      os.path.join(lib_src, 'xyz_bt_lib', 'core', fname)))
    for fname in TIER_A_BLACKBOARD:
        items.append((os.path.join('xyz_bt_lib', 'blackboard', fname),
                      os.path.join(lib_src, 'xyz_bt_lib', 'blackboard', fname)))
    # Tier B adds no further copyable core files: bt_runner needs the catkin
    # msg package and is excluded on purpose (see TIER_B_CORE).
    return items


def build_seed_tree(dest, tier='A', lib_src=None):
    """Materialise the seed under `dest` and return the package root dir.

    Package structure is preserved so that
    `from xyz_bt_lib.core.base_node import ...` resolves inside the seed. Every
    package gets an EMPTY __init__.py -- see the module docstring for why the
    real aggregator __init__ cannot be used.

    Raises IOError if a manifest entry is missing, which is the intended
    behaviour for the counter-test that removes a file from the seed.
    """
    missing = [src for _, src in seed_manifest(tier, lib_src)
               if not os.path.isfile(src)]
    if missing:
        raise IOError('seed manifest references missing files:\n  ' +
                      '\n  '.join(missing))

    for rel, src in seed_manifest(tier, lib_src):
        dst = os.path.join(dest, rel)
        pkg_dir = os.path.dirname(dst)
        if not os.path.isdir(pkg_dir):
            os.makedirs(pkg_dir)
        shutil.copyfile(src, dst)

    # Empty package markers, top-down.
    for rel_pkg in ('xyz_bt_lib',
                    os.path.join('xyz_bt_lib', 'core'),
                    os.path.join('xyz_bt_lib', 'blackboard')):
        pkg_dir = os.path.join(dest, rel_pkg)
        if not os.path.isdir(pkg_dir):
            os.makedirs(pkg_dir)
        init_py = os.path.join(pkg_dir, '__init__.py')
        if not os.path.exists(init_py):
            open(init_py, 'w').close()

    return dest


# ---------------------------------------------------------------------------
# Template rendering
# ---------------------------------------------------------------------------

_PLACEHOLDER_RE = re.compile(r'\{\{([A-Z0-9_]+)\}\}')


def render(text, mapping):
    """Substitute every {{PLACEHOLDER}} and refuse to return a partial render.

    A leftover placeholder is a hard error: it would become a SyntaxError far
    away from its cause, or -- worse -- survive into a file a human then edits.
    """
    out = _PLACEHOLDER_RE.sub(
        lambda m: str(mapping[m.group(1)]) if m.group(1) in mapping
        else m.group(0), text)
    leftover = sorted(set(_PLACEHOLDER_RE.findall(out)))
    if leftover:
        raise KeyError('unrendered placeholders: %s' % ', '.join(leftover))
    return out


def template_placeholders(text):
    """Return the sorted set of placeholder names appearing in `text`."""
    return sorted(set(_PLACEHOLDER_RE.findall(text)))


# ---------------------------------------------------------------------------
# Import isolation
# ---------------------------------------------------------------------------

class _BlockingFinder(importlib.abc.MetaPathFinder):
    """Raise ModuleNotFoundError for a set of module prefixes.

    Raising (rather than returning None) is deliberate: returning None would let
    a later finder resolve the module, which is exactly the leak this guards
    against.
    """

    def __init__(self, prefixes):
        self._prefixes = tuple(prefixes)

    def find_spec(self, fullname, path=None, target=None):
        for prefix in self._prefixes:
            if fullname == prefix or fullname.startswith(prefix + '.'):
                raise ModuleNotFoundError(
                    'seed isolation blocked %r (not part of the seed)'
                    % fullname, name=fullname)
        return None


def _hosts_real_library(path_entry, seed_dir):
    """True if `path_entry` is a sys.path dir holding a real xyz_bt_lib package."""
    if not path_entry:
        return False
    try:
        if os.path.samefile(path_entry, seed_dir):
            return False
    except OSError:
        pass
    return os.path.isdir(os.path.join(path_entry, 'xyz_bt_lib'))


@contextlib.contextmanager
def seed_isolation(seed_dir, tier='A', extra_blocked=()):
    """Make `seed_dir` the ONLY source of xyz_bt_lib for the duration.

    Restores sys.path, sys.modules and sys.meta_path on exit, so a validator can
    run several isolated cases in one process.
    """
    saved_path = list(sys.path)
    saved_modules = dict(sys.modules)
    saved_meta = list(sys.meta_path)

    for name in list(sys.modules):
        if name == 'xyz_bt_lib' or name.startswith('xyz_bt_lib.'):
            del sys.modules[name]

    sys.path[:] = [p for p in sys.path if not _hosts_real_library(p, seed_dir)]
    sys.path.insert(0, seed_dir)

    finder = _BlockingFinder(tuple(blocked_modules(tier)) + tuple(extra_blocked))
    sys.meta_path.insert(0, finder)
    try:
        yield seed_dir
    finally:
        sys.meta_path[:] = saved_meta
        sys.path[:] = saved_path
        sys.modules.clear()
        sys.modules.update(saved_modules)


def visible_library_paths(seed_dir):
    """Return sys.path entries that would expose a real xyz_bt_lib.

    A validator prints this to prove the node library is out of reach rather
    than asserting it.
    """
    return [p for p in sys.path if _hosts_real_library(p, seed_dir)]


# ---------------------------------------------------------------------------
# ROS stubs (Tier B)
# ---------------------------------------------------------------------------

class SubscriberRecord(object):
    """One recorded rospy.Subscriber(...) call."""

    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

    def __repr__(self):
        return 'SubscriberRecord(topic=%r, msg_type=%r)' % (
            self.topic, getattr(self.msg_type, '__name__', self.msg_type))

    def unregister(self):
        pass


class _RospyStub(types.ModuleType):
    """Minimal rospy that records wiring instead of talking to a master.

    Enough for an adapter's __init__ to run and for the validator to assert
    which topic/type it subscribed to, with no roscore and no ROS install.
    """

    def __init__(self):
        types.ModuleType.__init__(self, 'rospy')
        self.subscriptions = []
        self.publications = []

    def Subscriber(self, name, data_class, callback=None, **kwargs):
        rec = SubscriberRecord(name, data_class, callback)
        self.subscriptions.append(rec)
        return rec

    def Publisher(self, name, data_class, **kwargs):
        rec = SubscriberRecord(name, data_class, None)
        self.publications.append(rec)
        return rec

    # Logging is a no-op: adapters may log on state changes, and a validator
    # must not have that turn into noise or a crash.
    def loginfo(self, *args, **kwargs):
        pass

    logwarn = logerr = logdebug = logfatal = loginfo

    def get_time(self):
        return 0.0

    def is_shutdown(self):
        return False

    def init_node(self, *args, **kwargs):
        pass


def install_ros_stubs(msg_modules=None):
    """Insert a stub `rospy` (and optional message modules) into sys.modules.

    `msg_modules` maps module name -> iterable of message class names, e.g.
    {'ainex_interfaces.msg': ['ObjectsInfo']}. Each class is a permissive
    placeholder that accepts any constructor kwargs, which is all a template's
    type annotation and isinstance-free code needs.

    Returns the stub rospy module so a caller can inspect .subscriptions.
    Only valid inside seed_isolation(), which restores sys.modules on exit.
    """
    rospy_stub = _RospyStub()
    sys.modules['rospy'] = rospy_stub

    for mod_name, class_names in (msg_modules or {}).items():
        parts = mod_name.split('.')
        for i in range(1, len(parts) + 1):
            sub = '.'.join(parts[:i])
            if sub not in sys.modules:
                sys.modules[sub] = types.ModuleType(sub)
        mod = sys.modules[mod_name]
        for cls_name in class_names:
            setattr(mod, cls_name, _make_msg_stub(cls_name))
        # Make `import a.b` expose attribute `b` on `a`.
        for i in range(1, len(parts)):
            setattr(sys.modules['.'.join(parts[:i])], parts[i],
                    sys.modules['.'.join(parts[:i + 1])])

    return rospy_stub


def _make_msg_stub(name):
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

    return type(str(name), (object,), {'__init__': __init__})
