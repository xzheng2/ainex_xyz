#!/usr/bin/env python3
"""Prove the xyz-bt-facade-project templates still produce a working project.

Renders every .tpl into a throwaway package, imports it, instantiates the runtime
layers against fakes, builds the tree via bootstrap() and ticks it. Nothing here
touches the robot or ROS master — the managers and publishers are stubs.

WHY THIS EXISTS
    In Aug 2026 an audit found five defects that each crashed a freshly scaffolded
    project before its first tick: _RuntimeIO missing buzzer_pub, RuntimeFacade
    missing go_cfg/turn_cfg, a double-wrapped BehaviourTree, a facade move_head()
    without tilt_pos, and an entry point importing infra modules no template made.
    They survived because nothing ever ran the template output — SKILL.md and the
    templates simply agreed with each other. Python's ABC machinery did not help:
    it checks method NAMES, never signatures, so the broken facade satisfied
    XyzBTFacade and instantiated cleanly, then raised TypeError at runtime.

    Run this after ANY change to the templates or to the library contracts they
    depend on (core/base_facade.py above all).

USAGE (must run inside the ainex container, where py_trees and the ROS message
packages live):

    docker exec -u ubuntu -w /home/ubuntu/ros_ws ainex bash -lc \\
      "source devel/setup.bash && python3 <path>/validate_templates.py"

Exit code 0 = every check passed; 1 = at least one failed (details on stdout).
"""
import os
import re
import shutil
import sys
import tempfile
import traceback

TEMPLATE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'assets', 'templates')
SUBSTITUTIONS = {
    '{{PROJECT}}':       'demoproj',
    '{{PROJECT_CLASS}}': 'DemoProj',
    '{{TASK_DESC}}':     'template validation',
}

# tpl basename -> path inside the generated package (None = not importable on its
# own, rendered for syntax checking only).
LAYOUT = {
    'project_bt.py.tpl':          'tree/demoproj_bt.py',
    'runtime_facade.py.tpl':      'runtime/runtime_facade.py',
    'actions.py.tpl':             'behaviours/actions.py',
    'bt_node.py.tpl':             'app/demoproj_bt_node.py',
    'project_bt_groot.xml.tpl':   'tree/demoproj_groot.xml',
    'project.launch.tpl':         'launch/demoproj.launch',
}

results = []


def check(name, fn):
    try:
        detail = fn()
        results.append((True, name, detail or ''))
    except Exception as exc:                      # noqa: BLE001
        results.append((False, name, '{}: {}\n{}'.format(
            type(exc).__name__, exc, traceback.format_exc(limit=3))))


def render(out_root):
    """Render every template; return {tpl_name: rendered_path}."""
    rendered = {}
    for tpl, rel in LAYOUT.items():
        src = os.path.join(TEMPLATE_DIR, tpl)
        text = open(src, encoding='utf-8').read()
        for k, v in SUBSTITUTIONS.items():
            text = text.replace(k, v)
        left = re.findall(r'\{\{[A-Z_]+\}\}', text)
        if left:
            raise AssertionError('{}: unsubstituted placeholders {}'.format(
                tpl, sorted(set(left))))
        dst = os.path.join(out_root, 'demoproj', rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        open(dst, 'w', encoding='utf-8').write(text)
        rendered[tpl] = dst
    # package markers
    for d in ('', 'tree', 'runtime', 'behaviours', 'app'):
        open(os.path.join(out_root, 'demoproj', d, '__init__.py'), 'w').close()
    return rendered


class FakeGait:
    """Stands in for ainex_kinematics.gait_manager.GaitManager."""
    def __init__(self):
        self.calls = []
    def get_gait_param(self):
        return {'step_height': 0.02, 'hip_pitch_offset': 15}
    def __getattr__(self, n):
        def f(*a, **k):
            self.calls.append((n, a, k))
        return f


class FakeMotion:
    def __init__(self):
        self.calls = []
    def __getattr__(self, n):
        def f(*a, **k):
            self.calls.append((n, a, k))
        return f


class FakePub:
    def __init__(self):
        self.msgs = []
    def publish(self, msg):
        self.msgs.append(msg)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--lib-root', default='/home/pi/docker/ros_ws_src/xyz_bt_lib',
                    help='xyz_bt_lib package root; used to locate tools/seed_kit.py')
    ap.add_argument('--hooks-dir', default='/home/pi/.claude/hooks',
                    help='accepted for symmetry with the other skill validators')
    args = ap.parse_args(argv)

    tmp = tempfile.mkdtemp(prefix='tplcheck_')
    try:
        check('render all templates (no leftover placeholders)',
              lambda: 'rendered {} files'.format(len(render(tmp))))
        if not results[-1][0]:
            return report()

        sys.path.insert(0, tmp)
        import py_trees
        from py_trees.common import Access, Status

        # ── syntax: every rendered .py must compile ───────────────────────
        def compile_all():
            n = 0
            for rel in LAYOUT.values():
                if not rel.endswith('.py'):
                    continue
                p = os.path.join(tmp, 'demoproj', rel)
                compile(open(p, encoding='utf-8').read(), p, 'exec')
                n += 1
            return '{} python templates compile'.format(n)
        check('python syntax', compile_all)

        # ── the runtime layers actually construct ─────────────────────────
        state = {}

        def build_runtime():
            from xyz_bt_lib.core.default_runtime_io import _RuntimeIO
            from demoproj.runtime.runtime_facade import DemoProjRuntimeFacade
            from xyz_bt_lib.core.base_facade import XyzBTFacade
            go_cfg = {'dsp': 0.1, 'gait_param': None, 'arm_swap': 30, 'step_num': 0}
            io = _RuntimeIO(FakeGait(), FakeMotion(), FakePub(), tick_id_getter=lambda: 1)
            facade = DemoProjRuntimeFacade(io, go_cfg, dict(go_cfg),
                                           tick_id_getter=lambda: 1)
            assert isinstance(facade, XyzBTFacade), 'facade must be an XyzBTFacade'
            state['facade'] = facade
            return 'IO + facade constructed with the real signatures'
        check('_RuntimeIO / RuntimeFacade construction', build_runtime)

        # ── facade signatures match the ABC (ABCs only check names!) ──────
        def signature_parity():
            import inspect
            from xyz_bt_lib.core.base_facade import XyzBTFacade
            facade = state['facade']
            bad = []
            for mname in dir(XyzBTFacade):
                if mname.startswith('_'):
                    continue
                base = getattr(XyzBTFacade, mname, None)
                impl = getattr(type(facade), mname, None)
                if not callable(base) or impl is base:
                    continue
                bp = list(inspect.signature(base).parameters)
                ip = list(inspect.signature(impl).parameters)
                missing = [p for p in bp if p not in ip]
                if missing:
                    bad.append('{}() missing {}'.format(mname, missing))
            assert not bad, ('facade signature drift (an ABC will NOT catch this): '
                             + '; '.join(bad))
            return 'all overridden methods accept every base parameter'
        check('facade signature parity with XyzBTFacade', signature_parity)

        # ── bt_node's OWN construction calls satisfy the real signatures ──
        # Constructing these correctly inside this script proves nothing about
        # bt_node.py.tpl: the shipped defects were all at ITS call sites. Parse
        # them and bind against the true signatures.
        def bt_node_call_sites():
            import ast
            import inspect
            from xyz_bt_lib.core.default_runtime_io import _RuntimeIO
            from demoproj.runtime.runtime_facade import DemoProjRuntimeFacade

            src_path = os.path.join(tmp, 'demoproj', 'app', 'demoproj_bt_node.py')
            tree_ast = ast.parse(open(src_path, encoding='utf-8').read(), src_path)
            targets = {'_RuntimeIO': _RuntimeIO,
                       'DemoProjRuntimeFacade': DemoProjRuntimeFacade}
            seen = {}
            for node in ast.walk(tree_ast):
                if not isinstance(node, ast.Call):
                    continue
                fn = node.func
                nm = fn.id if isinstance(fn, ast.Name) else getattr(fn, 'attr', None)
                if nm in targets:
                    seen[nm] = node
            missing_calls = [t for t in targets if t not in seen]
            assert not missing_calls, 'bt_node never constructs {}'.format(missing_calls)

            problems = []
            for nm, call in seen.items():
                sig = inspect.signature(targets[nm].__init__)
                params = [p for p in sig.parameters.values() if p.name != 'self']
                supplied_kw = {k.arg for k in call.keywords if k.arg}
                n_pos = len(call.args)
                for i, p in enumerate(params):
                    if p.default is not inspect.Parameter.empty:
                        continue
                    if i < n_pos or p.name in supplied_kw:
                        continue
                    problems.append('{}(): required arg {!r} not supplied'.format(nm, p.name))
            assert not problems, '; '.join(problems)
            return 'bt_node supplies every required arg to {}'.format(
                ', '.join(sorted(seen)))
        check('bt_node construction call sites', bt_node_call_sites)


        # ── the tree builds and ticks ─────────────────────────────────────
        def build_and_tick():
            from demoproj.tree.demoproj_bt import bootstrap
            py_trees.blackboard.Blackboard.storage.clear()
            seed = py_trees.blackboard.Client(name='seed', namespace='/latched')
            for k, v in [('robot_state', 'stand'), ('tracked_objects', {})]:
                seed.register_key(key=k, access=Access.WRITE)
                seed.set('/latched/' + k, v)

            tick = {'n': 0}
            tree = bootstrap(state['facade'], tick_id_getter=lambda: tick['n'])
            assert isinstance(tree, py_trees.trees.BehaviourTree), (
                'bootstrap() must return a BehaviourTree — do not re-wrap it in bt_node')
            tree.setup(timeout=15)

            # The SafetyGate wraps IsStanding in a LatchedDwellDecorator, which
            # returns RUNNING while it counts. Tick past the dwell first,
            # otherwise the task branches are never reached and this check would
            # silently stop demonstrating what its name claims.
            storage = py_trees.blackboard.Blackboard.storage
            latch_path = '/node_state/safety_stand_confirmed'
            for _ in range(12):                     # target absent -> Search branch
                tick['n'] += 1
                tree.tick()
                if (storage.get(latch_path) or {}).get('latched'):
                    break
            latch = storage.get(latch_path)
            assert latch is not None, (
                'the SafetyGate dwell wrote no {} state — is the '
                'LatchedDwellDecorator still wired in bootstrap()?'.format(latch_path))
            assert latch.get('latched'), (
                'dwell never latched after 12 ticks with robot_state=stand: '
                '{}'.format(latch))

            for _ in range(2):                      # settle on the Search branch
                tick['n'] += 1
                tree.tick()
            searching = tree.root.status

            seed.set('/latched/tracked_objects',
                     {'ball': {'x': 320.0, 'y': 360.0, 'error_x': 0.0, 'error_y': 0.0,
                               'size': 1500.0, 'lost_count': 0, 'depth_mm': None,
                               'displacement': 0.0}})
            tick['n'] += 1
            tree.tick()                             # target visible -> Approach branch
            return ('dwell latched after {} ticks, then search ({}) / approach ({})'
                    .format(latch.get('stable_ticks'), searching.name,
                            tree.root.status.name))
        check('bootstrap() builds a tickable tree (both branches)', build_and_tick)

        # ── Groot XML parity with the .py wiring ──────────────────────────
        def groot_parity():
            import xml.etree.ElementTree as ET
            xml_path = os.path.join(tmp, 'demoproj', 'tree', 'demoproj_groot.xml')
            root = ET.parse(xml_path).getroot()
            # Only the <BehaviorTree> half carries node INSTANCE names; the
            # name= inside <TreeNodesModel> is a port name, not a node.
            bt = root.find('BehaviorTree')
            assert bt is not None, 'no <BehaviorTree> section'
            names = {e.get('name') for e in bt.iter() if e.get('name')}
            py_src = open(os.path.join(tmp, 'demoproj', 'tree', 'demoproj_bt.py'),
                          encoding='utf-8').read()
            missing = [n for n in sorted(names)
                       if n and not n.endswith('BT') and "'{}'".format(n) not in py_src]
            assert not missing, 'Groot node name(s) with no bt.py counterpart: {}'.format(missing)

            # Every node TYPE used in the tree must have a TreeNodesModel entry,
            # or Groot renders the box with no port fields.
            model = root.find('TreeNodesModel')
            assert model is not None, 'no <TreeNodesModel> section'
            modelled = {e.get('ID') for e in model}
            # bt itself carries ID="<tree name>", which is not a node type.
            inner = [e for e in bt.iter() if e is not bt]
            used = {e.get('ID') for e in inner if e.get('ID')}
            used |= {e.tag for e in inner if e.tag in ('Sequence', 'Fallback')}
            unmodelled = sorted(u for u in used if u and u not in modelled)
            assert not unmodelled, 'node types missing from TreeNodesModel: {}'.format(unmodelled)
            return '{} node instances match bt.py; {} node types modelled'.format(
                len(names), len(modelled))
        check('Groot XML <-> bt.py name parity', groot_parity)

        # ── the whole rendered project imports, with nothing stubbed ──────
        # Compiling is not importing. bt_node.py.tpl spent months importing
        # {{PROJECT}}.infra.tree_publisher and .bt_exec_controller, for which no
        # template existed and no project file was generated -- every scaffold died
        # on its first import. Eight checks were green throughout, because the only
        # ones that touched app/*_bt_node.py were compile() and ast.parse(), and
        # neither resolves an import.
        #
        # Deliberately NOT inside seed_isolation() and with no install_ros_stubs():
        # a stub would re-create exactly the blind spot this check exists to close.
        # It therefore needs the real container environment (rospy, py_trees,
        # rospkg, xyz_bt_lib, ros_robot_controller.msg) plus whatever sys.path
        # surgery the rendered entry point performs on its own behalf.
        # Derived from LAYOUT, never hand-listed: a hand-written list would omit the
        # next template added, which is precisely the drift that produced the gap this
        # check exists to catch.
        _RENDERED_MODULES = tuple(
            'demoproj.' + rel[:-len('.py')].replace('/', '.')
            for rel in LAYOUT.values() if rel.endswith('.py'))

        def _purge_demoproj():
            """Drop every demoproj module so an import is actually exercised.

            Checks 3, 6 and 7 already imported several of these; without this the
            new check would re-read sys.modules and prove nothing about them.
            """
            for name in [m for m in sys.modules if m == 'demoproj'
                         or m.startswith('demoproj.')]:
                del sys.modules[name]

        def rendered_imports():
            import importlib
            _purge_demoproj()
            for name in _RENDERED_MODULES:
                importlib.import_module(name)
            return '{} rendered modules import unstubbed'.format(len(_RENDERED_MODULES))
        check('rendered project imports with no stubs', rendered_imports)

        # ── counter-test: the check above must be able to fail ────────────
        # An import check that cannot go red is indistinguishable from no check at
        # all -- which is the failure mode that let the missing modules survive.
        # Block one shared dependency and require the entry point to break.
        def missing_shared_module_breaks_import():
            import importlib
            sys.path.insert(0, os.path.join(args.lib_root, 'tools'))
            import seed_kit

            blocked = 'bt_observability.tree_publisher'
            _purge_demoproj()
            for name in [m for m in sys.modules if m == 'bt_observability'
                         or m.startswith('bt_observability.')]:
                del sys.modules[name]

            finder = seed_kit._BlockingFinder([blocked])
            sys.meta_path.insert(0, finder)
            try:
                importlib.import_module('demoproj.app.demoproj_bt_node')
            except ModuleNotFoundError:
                return 'blocking {} breaks the entry point, as it must'.format(blocked)
            finally:
                sys.meta_path.remove(finder)
                _purge_demoproj()
            raise AssertionError(
                'the entry point imported with {} blocked — either it no longer '
                'depends on the shared module, or the block is not in effect, and '
                'in both cases the import check above proves nothing'.format(blocked))
        check('counter-test: a missing shared infra module breaks the import',
              missing_shared_module_breaks_import)

        # ── the example tree's dependency on the node library ─────────────
        # project_bt.py.tpl imports concrete behaviours/ classes. That is a real
        # dependency: on a robot whose node library differs, the rendered file
        # will not import. Both template headers say so; this check makes it a
        # tested fact instead of a claim, mirroring the adapter skill's
        # "Tier B is a real boundary" assertion.
        def example_tree_dependency():
            import xml.etree.ElementTree as ET
            py_path = os.path.join(tmp, 'demoproj', 'tree', 'demoproj_bt.py')
            py_src = open(py_path, encoding='utf-8').read()

            # Derived, never hardcoded — a second copy of the node list is
            # exactly the kind of thing that drifts.
            imported = set(re.findall(
                r'from\s+xyz_bt_lib\.behaviours\.[\w.]+\s+import\s+(\w+)', py_src))
            assert imported, ('project_bt.py.tpl no longer imports any '
                              'xyz_bt_lib.behaviours node — update this check and '
                              'both template headers')

            xml_path = os.path.join(tmp, 'demoproj', 'tree', 'demoproj_groot.xml')
            bt = ET.parse(xml_path).getroot().find('BehaviorTree')
            xml_types = {e.get('ID') for e in bt.iter()
                         if (e.get('ID') or '').startswith(('L1_', 'L2_', 'L3_'))}
            if xml_types != imported:
                raise AssertionError(
                    'tree templates disagree on library nodes — bt.py imports {}, '
                    'Groot XML uses {}'.format(sorted(imported), sorted(xml_types)))

            # Now prove the dependency is real: with behaviours/ unreachable the
            # rendered module MUST fail to import.
            sys.path.insert(0, os.path.join(args.lib_root, 'tools'))
            import seed_kit
            seed_dir = seed_kit.build_seed_tree(
                os.path.join(tmp, 'seed'), tier='A',
                lib_src=os.path.join(args.lib_root, 'src'))
            shutil.copyfile(py_path, os.path.join(seed_dir, 'demoproj_bt_probe.py'))
            with seed_kit.seed_isolation(seed_dir, tier='A'):
                try:
                    __import__('demoproj_bt_probe')
                except ModuleNotFoundError as exc:
                    return ('{} library nodes, consistent across bt.py and Groot; '
                            'without behaviours/ the tree fails to import ({})'
                            .format(len(imported), exc))
            raise AssertionError(
                'the rendered tree imported with behaviours/ blocked — either it '
                'no longer depends on the node library, or the isolation is not '
                'in effect')
        check('example tree declares its node-library dependency',
              example_tree_dependency)

    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return report()


def report():
    print('\n=== xyz-bt-facade-project template validation ===')
    for ok, name, detail in results:
        print('  {} {}'.format('PASS' if ok else 'FAIL', name))
        if detail:
            head = detail if ok else detail.rstrip()
            for line in str(head).splitlines():
                print('        ' + line)
    failed = [r for r in results if not r[0]]
    print('\n{} passed, {} failed'.format(len(results) - len(failed), len(failed)))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
