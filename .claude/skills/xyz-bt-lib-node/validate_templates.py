#!/usr/bin/env python3
# encoding: utf-8
"""Prove the xyz-bt-lib-node templates work WITHOUT the node library present.

WHAT IS BEING CLAIMED
    The l1/l2/l3 templates plus the Tier A seed (xyz_bt_lib/core + blackboard
    keys, see tools/seed_kit.py) are sufficient to build a real BT node on a
    machine where behaviours/, adapters/ and xyz_perception do not exist -- a fresh
    robot, or this repo stripped to its engine layer.

HOW IT IS PROVED, NOT INSPECTED
    The seed is copied to a temp dir, the templates are rendered into it, and
    sys.path is reduced to that dir alone. CPython's import machinery is then the
    oracle: any reference the templates make outside the seed raises
    ModuleNotFoundError. Rendered nodes are additionally instantiated against
    StubFacade, setup() and ticked, which catches contract drift that imports
    cannot -- wrong __init__ signature, missing super().setup(), Access misuse,
    a status that is not a py_trees Status.

    Counter-tests run alongside the positive ones. Removing base_node.py from the
    seed MUST break the suite; a template importing behaviours MUST fail; the
    guard MUST reject a wrong base class. Without those, "all green" would prove
    nothing -- which is exactly how check_bb_keys stayed dead for months while
    reporting success.

USAGE (inside the ainex container, which has py_trees; ROS is NOT required):

    docker exec -u ubuntu ainex python3 \\
      /home/ubuntu/.claude-skills/xyz-bt-lib-node/validate_templates.py

    In practice run it through the unified entry point:
      xyz_bt_lib/tools/validate_engine.py

Exit code 0 = every check passed; 1 = at least one failed.
"""
import argparse
import importlib
import os
import shutil
import sys
import tempfile
import traceback

SKILL_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(SKILL_DIR, 'assets', 'templates')

DEFAULT_LIB_ROOT = '/home/pi/docker/ros_ws_src/xyz_bt_lib'
DEFAULT_HOOKS_DIR = '/home/pi/.claude/hooks'

#: One rendered node per template. `kwargs` mirrors how a project tree would
#: construct the node; `allowed` is the status contract the tier must honour.
CASES = {
    'l1_node.py.tpl': {
        'class': 'L1_Seed_IsProbeTrue',
        'kwargs': {},
        'facade': False,
        'allowed': ('SUCCESS', 'FAILURE'),   # L1 purity: never RUNNING
    },
    'l2_node.py.tpl': {
        'class': 'L2_Seed_ProbeAction',
        'kwargs': {},
        'facade': True,
        'allowed': ('SUCCESS', 'FAILURE', 'RUNNING'),
    },
    'l3_node.py.tpl': {
        'class': 'L3_Seed_ProbeProcess',
        'kwargs': {'target': 'seed_probe_process'},
        'facade': True,
        'allowed': ('SUCCESS', 'FAILURE', 'RUNNING'),
    },
}

results = []


def check(name, fn):
    try:
        detail = fn()
        results.append((True, name, detail or ''))
    except Exception as exc:                      # noqa: BLE001
        results.append((False, name, '{}: {}\n{}'.format(
            type(exc).__name__, exc, traceback.format_exc(limit=3))))


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def substitutions(class_name):
    return {
        'CLASS_NAME':   class_name,
        'NODE_NAME':    class_name,
        'DESCRIPTION':  'seed-closure probe node',
        'BB_READS':     '[]',
        'DEFAULT_NAME': repr(class_name),
    }


def render_cases(seed_kit, out_dir):
    """Render every template into `out_dir`; return {tpl: (path, class)}."""
    rendered = {}
    for tpl, spec in CASES.items():
        text = open(os.path.join(TEMPLATE_DIR, tpl), encoding='utf-8').read()
        src = seed_kit.render(text, substitutions(spec['class']))
        path = os.path.join(out_dir, spec['class'] + '.py')
        open(path, 'w', encoding='utf-8').write(src)
        rendered[tpl] = (path, spec['class'])
    return rendered


def extract_gait_variant(seed_kit, tpl_text, class_name):
    """Return the l2 template's commented gait skeleton as runnable source.

    The skeleton is documentation, and documentation drifts. Rendering it for
    real is the only way to know the L2_Gait_ contract it teaches is still the
    contract the guard enforces.
    """
    marker = '# class {{CLASS_NAME}}(XyzL2GaitActionNode):'
    idx = tpl_text.find(marker)
    if idx < 0:
        raise AssertionError('l2 template no longer carries the gait skeleton')
    header = tpl_text.split('\nclass {{CLASS_NAME}}(XyzL2ActionNode):')[0]
    body = []
    for line in tpl_text[idx:].splitlines():
        if line.startswith('# '):
            body.append(line[2:])
        elif line.strip() == '#':
            body.append('')
        else:
            body.append(line)
    text = header + '\n\n\n' + '\n'.join(body) + '\n'
    return seed_kit.render(text, substitutions(class_name))


# ---------------------------------------------------------------------------
# Execution inside the seed
# ---------------------------------------------------------------------------

def tick_case(spec, module_name):
    """Import, construct, setup and tick one rendered node. Return its status."""
    from xyz_bt_lib.core.stub_facade import StubFacade
    from py_trees.common import Status

    mod = importlib.import_module(module_name)
    cls = getattr(mod, spec['class'])

    kwargs = dict(spec['kwargs'])
    if spec['facade']:
        kwargs['facade'] = StubFacade()
    node = cls(name=spec['class'], **kwargs)

    node.setup()
    node.tick_once()

    if not isinstance(node.status, Status):
        raise AssertionError('tick returned {!r}, not a py_trees Status'
                             .format(node.status))
    if node.status.name not in spec['allowed']:
        raise AssertionError('{} returned {}, contract allows {}'
                             .format(spec['class'], node.status.name,
                                     '/'.join(spec['allowed'])))
    return node.status.name


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--lib-root', default=DEFAULT_LIB_ROOT,
                    help='xyz_bt_lib package root (holds tools/ and src/)')
    ap.add_argument('--hooks-dir', default=DEFAULT_HOOKS_DIR,
                    help='directory holding xyz_bt_lib_guard.py')
    args = ap.parse_args(argv)

    sys.path.insert(0, os.path.join(args.lib_root, 'tools'))
    import seed_kit                                    # noqa: E402
    sys.path.insert(0, args.hooks_dir)
    from xyz_bt_lib_guard import check_node            # noqa: E402
    from xyz_bt_l1_running_guard import check_l1_running   # noqa: E402

    lib_src = os.path.join(args.lib_root, 'src')
    tmp = tempfile.mkdtemp(prefix='seed_node_')
    try:
        seed = None

        def build_seed():
            nonlocal seed
            seed = seed_kit.build_seed_tree(
                os.path.join(tmp, 'seed'), tier='A', lib_src=lib_src)
            files = seed_kit.seed_manifest('A', lib_src)
            return '{} seed files under {}'.format(len(files), seed)
        check('Tier A seed builds', build_seed)

        if seed is None:
            return report()

        rendered = {}

        def do_render():
            rendered.update(render_cases(seed_kit, seed))
            return 'rendered {} templates, no placeholders left'.format(len(rendered))
        check('templates render completely', do_render)

        def syntax():
            for tpl, (path, _) in sorted(rendered.items()):
                compile(open(path, encoding='utf-8').read(), path, 'exec')
            return '{} files compile'.format(len(rendered))
        check('rendered output compiles', syntax)

        # ---- the central claim -------------------------------------------
        def isolation_holds():
            with seed_kit.seed_isolation(seed, tier='A'):
                visible = seed_kit.visible_library_paths(seed)
                if visible:
                    raise AssertionError(
                        'real xyz_bt_lib still on sys.path: %s' % visible)
                for forbidden in ('xyz_bt_lib.behaviours', 'xyz_bt_lib.adapters',
                                  'xyz_perception', 'rospy'):
                    try:
                        importlib.import_module(forbidden)
                    except ModuleNotFoundError:
                        continue
                    raise AssertionError('%s was importable inside the seed'
                                         % forbidden)
            return ('behaviours/ adapters/ xyz_perception rospy all unreachable; '
                    'sys.path holds no real library')
        check('node library is out of reach', isolation_holds)

        def tick_all():
            out = []
            with seed_kit.seed_isolation(seed, tier='A'):
                for tpl, (path, cls_name) in sorted(rendered.items()):
                    status = tick_case(CASES[tpl], cls_name)
                    out.append('{} -> {}'.format(cls_name, status))
            return '; '.join(out)
        check('import + construct + setup + tick inside seed', tick_all)

        # ---- guard conformance -------------------------------------------
        def guard_clean():
            for tpl, (path, _) in sorted(rendered.items()):
                content = open(path, encoding='utf-8').read()
                violations = check_node(content, path)
                if violations:
                    raise AssertionError('{}: {}'.format(
                        os.path.basename(path), violations))
            return '{} rendered nodes pass check_node'.format(len(rendered))
        check('guard accepts rendered nodes', guard_clean)

        def l1_purity():
            path, _ = rendered['l1_node.py.tpl']
            content = open(path, encoding='utf-8').read()
            violations = check_l1_running(content, path)
            if violations:
                raise AssertionError('rendered L1 is not a pure predicate: '
                                     '{}'.format(violations))
            return 'rendered L1 holds no Status.RUNNING path'
        check('L1 purity rule accepts rendered L1', l1_purity)

        def gait_skeleton():
            tpl_text = open(os.path.join(TEMPLATE_DIR, 'l2_node.py.tpl'),
                            encoding='utf-8').read()
            name = 'L2_Gait_SeedProbe'
            src = extract_gait_variant(seed_kit, tpl_text, name)
            path = os.path.join(seed, name + '.py')
            open(path, 'w', encoding='utf-8').write(src)
            compile(src, path, 'exec')
            violations = check_node(src, path)
            if violations:
                raise AssertionError('gait skeleton violates its own contract: '
                                     '{}'.format(violations))
            with seed_kit.seed_isolation(seed, tier='A'):
                status = tick_case(
                    {'class': name, 'kwargs': {}, 'facade': True,
                     'allowed': ('SUCCESS', 'FAILURE', 'RUNNING')}, name)
            return 'L2_Gait_ skeleton compiles, passes guard, ticks -> ' + status
        check('l2 gait skeleton honours the L2_Gait_ contract', gait_skeleton)

        # ---- counter-tests: these MUST fail ------------------------------
        def counter_missing_base():
            broken = seed_kit.build_seed_tree(
                os.path.join(tmp, 'broken'), tier='A', lib_src=lib_src)
            for tpl, (path, cls_name) in rendered.items():
                shutil.copyfile(path, os.path.join(broken, cls_name + '.py'))
            os.remove(os.path.join(broken, 'xyz_bt_lib', 'core', 'base_node.py'))
            with seed_kit.seed_isolation(broken, tier='A'):
                try:
                    importlib.import_module(CASES['l1_node.py.tpl']['class'])
                except ModuleNotFoundError as exc:
                    return 'removing base_node.py breaks the seed: {}'.format(exc)
            raise AssertionError(
                'seed still worked without base_node.py -- the test is not '
                'actually importing from the seed')
        check('counter-test: seed without base_node.py fails', counter_missing_base)

        def counter_upward_import():
            path = os.path.join(seed, 'seed_upward_probe.py')
            open(path, 'w', encoding='utf-8').write(
                'from xyz_bt_lib.behaviours.L1_perception import anything\n')
            with seed_kit.seed_isolation(seed, tier='A'):
                try:
                    importlib.import_module('seed_upward_probe')
                except ModuleNotFoundError as exc:
                    os.remove(path)
                    return 'upward import rejected: {}'.format(exc)
            os.remove(path)
            raise AssertionError('a template could import behaviours/ undetected')
        check('counter-test: importing behaviours/ fails', counter_upward_import)

        def counter_guard_blind():
            path, cls_name = rendered['l1_node.py.tpl']
            content = open(path, encoding='utf-8').read()
            broken = content.replace(
                'class {}(XyzL1ConditionNode):'.format(cls_name),
                'class {}(object):'.format(cls_name))
            if broken == content:
                raise AssertionError('could not mutate the base class')
            if not check_node(broken, path):
                raise AssertionError(
                    'guard accepted an L1 node not inheriting '
                    'XyzL1ConditionNode -- check_node is blind here')
            return 'guard rejects a wrong base class'
        check('counter-test: guard catches wrong base class', counter_guard_blind)

        def counter_l1_running():
            path, cls_name = rendered['l1_node.py.tpl']
            content = open(path, encoding='utf-8').read()
            broken = content.replace('        return status',
                                     '        return Status.RUNNING')
            if broken == content:
                raise AssertionError('could not inject a RUNNING return')
            if not check_l1_running(broken, path):
                raise AssertionError(
                    'L1 purity rule accepted an L1 node returning RUNNING')
            return 'L1 purity rule rejects RUNNING in an L1 node'
        check('counter-test: guard catches L1 returning RUNNING', counter_l1_running)

    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return report()


def report():
    print('\n=== xyz-bt-lib-node seed-closure validation ===')
    for ok, name, detail in results:
        print('  {} {}'.format('PASS' if ok else 'FAIL', name))
        if detail:
            for line in str(detail).rstrip().splitlines():
                print('        ' + line)
    failed = [r for r in results if not r[0]]
    print('\n{} passed, {} failed'.format(len(results) - len(failed), len(failed)))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
