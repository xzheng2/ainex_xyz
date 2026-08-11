#!/usr/bin/env python3
# encoding: utf-8
"""Prove the xyz-bt-lib-adapter template works against the seed alone.

WHAT IS BEING CLAIMED
    input_adapter.py.tpl plus the seed (xyz_bt_lib/core + blackboard keys, see
    tools/seed_kit.py) is sufficient to build a working ROS input adapter on a
    machine where the existing adapters/ do not exist.

    An adapter is Tier B: it owns rospy.Subscriber, because an input adapter IS
    the ROS boundary. That binding is deliberate, and this script makes it
    TESTABLE rather than merely documented -- one check asserts the rendered
    adapter cannot import under Tier A isolation, which is what "Tier B" means.

HOW IT IS PROVED
    Seed -> temp dir, template rendered into it, sys.path reduced to that dir.
    rospy and the message package are injected as stubs (seed_kit.install_ros_stubs),
    so the adapter's __init__ runs with no roscore and no ROS install, while the
    subscription it creates is recorded and asserted: right topic, right message
    type, bound to _callback.

    The two-phase latch protocol is then exercised for real --
    snapshot_and_reset() under the lock, write_snapshot() after release -- and the
    blackboard is read back to confirm the value actually landed.

    Counter-tests run alongside: removing base_adapter.py from the seed MUST
    break the import, and removing the emit_input_state / rospy.Subscriber calls
    MUST be caught by check_adapter. "All green" with no negative control proves
    nothing.

USAGE (inside the ainex container, which has py_trees; ROS is NOT required):
    run it through the unified entry point, xyz_bt_lib/tools/validate_engine.py

Exit code 0 = every check passed; 1 = at least one failed.
"""
import argparse
import importlib
import os
import shutil
import sys
import tempfile
import threading
import traceback

SKILL_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(SKILL_DIR, 'assets', 'templates', 'input_adapter.py.tpl')

DEFAULT_LIB_ROOT = '/home/pi/docker/ros_ws_src/xyz_bt_lib'
DEFAULT_HOOKS_DIR = '/home/pi/.claude/hooks'

CLASS_NAME = 'SeedProbeAdapter'
ROS_TOPIC = '/seed/probe'
MSG_MODULE = 'seed_probe_msgs.msg'
MSG_TYPE = 'ProbeMsg'
INITIAL_STATE = 'stand'

#: A minimal but REAL adapter: one existing BB key, written through the two-phase
#: latch. Using BB.ROBOT_STATE rather than an invented key keeps the rendered
#: output something check_adapter and the blackboard both accept as genuine.
SUBSTITUTIONS = {
    'CLASS_NAME':       CLASS_NAME,
    'ROS_TOPIC':        ROS_TOPIC,
    'DESCRIPTION':      'seed-closure probe adapter',
    'MSG_TYPE':         MSG_TYPE,
    'MSG_TYPE_IMPORT':  'from {} import {}'.format(MSG_MODULE, MSG_TYPE),
    'BB_KEY_LIST':      'BB.ROBOT_STATE  (/latched/robot_state)',
    'BB_KEY_INITS':     "self._live_robot_state = {!r}".format(INITIAL_STATE),
    'BB_REGISTER_KEYS': 'self._bb.register_key(key=BB.ROBOT_STATE_KEY, '
                        'access=Access.WRITE)',
    'BB_INIT_WRITES':   "self._bb.robot_state = {!r}".format(INITIAL_STATE),
    'CALLBACK_LOGIC':   'self._live_robot_state = bb_writes[BB.ROBOT_STATE]',
    'SNAP_FIELDS':      "'robot_state': self._live_robot_state,",
    'BB_WRITES':        "self._bb.robot_state = snap['robot_state']",
    'INPUT_STATE_BB_WRITES': "BB.ROBOT_STATE: snap['robot_state'],",
}

results = []


def check(name, fn):
    try:
        detail = fn()
        results.append((True, name, detail or ''))
    except Exception as exc:                      # noqa: BLE001
        results.append((False, name, '{}: {}\n{}'.format(
            type(exc).__name__, exc, traceback.format_exc(limit=3))))


def exercise_adapter(seed_kit, module_name):
    """Construct the rendered adapter and drive the two-phase latch protocol."""
    import py_trees
    from xyz_bt_lib.blackboard.blackboard_keys import BB

    rospy_stub = seed_kit.install_ros_stubs({MSG_MODULE: [MSG_TYPE]})
    mod = importlib.import_module(module_name)
    cls = getattr(mod, CLASS_NAME)

    lock = threading.Lock()
    adapter = cls(lock=lock, logger=None, tick_id_getter=lambda: 7)

    # --- ROS wiring -------------------------------------------------------
    if len(rospy_stub.subscriptions) != 1:
        raise AssertionError('expected exactly 1 subscription, got {}'
                             .format(rospy_stub.subscriptions))
    sub = rospy_stub.subscriptions[0]
    if sub.topic != ROS_TOPIC:
        raise AssertionError('subscribed to {!r}, expected {!r}'
                             .format(sub.topic, ROS_TOPIC))
    if getattr(sub.msg_type, '__name__', None) != MSG_TYPE:
        raise AssertionError('subscribed with message type {!r}'
                             .format(sub.msg_type))
    if sub.callback != adapter._callback:
        raise AssertionError('subscription is not bound to _callback')

    # --- two-phase latch --------------------------------------------------
    with lock:
        snap = adapter.snapshot_and_reset()
    for field in ('received_count', 'robot_state'):
        if field not in snap:
            raise AssertionError('snapshot missing {!r}: {}'.format(field, snap))
    adapter.write_snapshot(snap, tick_id=7)

    written = py_trees.blackboard.Blackboard.storage.get(BB.ROBOT_STATE)
    if written != INITIAL_STATE:
        raise AssertionError('{} = {!r}, expected {!r}'
                             .format(BB.ROBOT_STATE, written, INITIAL_STATE))

    # --- placeholder contract still intact --------------------------------
    try:
        adapter._extract_xxx(None)
    except NotImplementedError:
        pass
    else:
        raise AssertionError('_extract_xxx no longer raises NotImplementedError '
                             '-- the "you must fill this in" contract is gone')

    return ('subscribed {} ({}), latch wrote {}={!r}'
            .format(ROS_TOPIC, MSG_TYPE, BB.ROBOT_STATE, written))


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
    from xyz_bt_lib_guard import check_adapter         # noqa: E402

    lib_src = os.path.join(args.lib_root, 'src')
    tmp = tempfile.mkdtemp(prefix='seed_adapter_')
    fake_path = ('xyz_bt_lib/src/xyz_bt_lib/adapters/'
                 'seed_probe_adapter.py')
    try:
        state = {}

        def build_seed():
            state['seed'] = seed_kit.build_seed_tree(
                os.path.join(tmp, 'seed'), tier='B', lib_src=lib_src)
            return '{} seed files under {}'.format(
                len(seed_kit.seed_manifest('B', lib_src)), state['seed'])
        check('Tier B seed builds', build_seed)

        if 'seed' not in state:
            return report()
        seed = state['seed']

        def do_render():
            text = open(TEMPLATE, encoding='utf-8').read()
            src = seed_kit.render(text, SUBSTITUTIONS)
            path = os.path.join(seed, CLASS_NAME + '.py')
            open(path, 'w', encoding='utf-8').write(src)
            compile(src, path, 'exec')
            state['path'] = path
            state['src'] = src
            return 'rendered and compiled {}'.format(os.path.basename(path))
        check('template renders and compiles', do_render)

        if 'src' not in state:
            return report()

        def isolation_holds():
            with seed_kit.seed_isolation(seed, tier='B'):
                visible = seed_kit.visible_library_paths(seed)
                if visible:
                    raise AssertionError('real xyz_bt_lib still on sys.path: %s'
                                         % visible)
                for forbidden in ('xyz_bt_lib.adapters', 'xyz_bt_lib.behaviours'):
                    try:
                        importlib.import_module(forbidden)
                    except ModuleNotFoundError:
                        continue
                    raise AssertionError('%s was importable' % forbidden)
            return 'existing adapters/ and behaviours/ unreachable'
        check('adapter library is out of reach', isolation_holds)

        def run_adapter():
            with seed_kit.seed_isolation(seed, tier='B'):
                return exercise_adapter(seed_kit, CLASS_NAME)
        check('construct + subscribe + two-phase latch inside seed', run_adapter)

        def guard_clean():
            violations = check_adapter(state['src'])
            if violations:
                raise AssertionError(violations)
            return 'rendered adapter passes check_adapter'
        check('guard accepts rendered adapter', guard_clean)

        # ---- the Tier B boundary, asserted rather than merely documented --
        def tier_b_is_real():
            with seed_kit.seed_isolation(seed, tier='A'):
                try:
                    importlib.import_module(CLASS_NAME)
                except ModuleNotFoundError as exc:
                    return ('adapter cannot import with ROS blocked ({}) -- '
                            'Tier B is a real boundary, not a label'.format(exc))
            raise AssertionError(
                'adapter imported under Tier A isolation: either it no longer '
                'owns rospy.Subscriber, or Tier A is not blocking ROS')
        check('adapter is genuinely Tier B (ROS-bound)', tier_b_is_real)

        # ---- counter-tests: these MUST fail ------------------------------
        def counter_missing_base_adapter():
            broken = seed_kit.build_seed_tree(
                os.path.join(tmp, 'broken'), tier='B', lib_src=lib_src)
            shutil.copyfile(state['path'],
                            os.path.join(broken, CLASS_NAME + '.py'))
            os.remove(os.path.join(broken, 'xyz_bt_lib', 'core',
                                   'base_adapter.py'))
            with seed_kit.seed_isolation(broken, tier='B'):
                seed_kit.install_ros_stubs({MSG_MODULE: [MSG_TYPE]})
                try:
                    importlib.import_module(CLASS_NAME)
                except ModuleNotFoundError as exc:
                    return 'removing base_adapter.py breaks the seed: {}'.format(exc)
            raise AssertionError(
                'seed still worked without base_adapter.py -- the test is not '
                'actually importing from the seed')
        check('counter-test: seed without base_adapter.py fails',
              counter_missing_base_adapter)

        def counter_guard_blind():
            # The replacement must share NO substring with the needle: a naive
            # '_removed_' + label prefix still contains the token the guard
            # greps for, so the mutation silently does nothing and the
            # counter-test "passes" for the wrong reason.
            probes = [
                ('self.emit_input_state(', 'self.gutted_a(', 'emit_input_state'),
                ('rospy.Subscriber(', 'gutted_b(', 'rospy.Subscriber'),
                ('def snapshot_and_reset(', 'def gutted_c(', 'snapshot_and_reset'),
            ]
            for needle, replacement, label in probes:
                if needle not in state['src']:
                    raise AssertionError('rendered adapter lacks {}'.format(label))
                broken = state['src'].replace(needle, replacement)
                if needle in broken:
                    raise AssertionError(
                        'mutation for {} did not remove the token'.format(label))
                if not check_adapter(broken):
                    raise AssertionError(
                        'check_adapter accepted an adapter with {} removed'
                        .format(label))
            return 'guard rejects removal of {} required calls'.format(len(probes))
        check('counter-test: guard catches a gutted adapter', counter_guard_blind)

    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return report()


def report():
    print('\n=== xyz-bt-lib-adapter seed-closure validation ===')
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
