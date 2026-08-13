#!/usr/bin/env python3
# encoding: utf-8
"""validate_engine.py - one command that checks the whole BT engine.

WHAT THIS COVERS
    0. HOOK IMPORT SAFETY (host)      The sweep below imports the hooks. Any
       hook that calls main() at module scope runs on import and sys.exit()s,
       which killed this whole process before it printed anything -- and
       .githooks/pre-push then passed on exit 0. Checked first, statically.

    1. LIBRARY SWEEP (host, static)   Every L1/L2/L3 node, every adapter and
       blackboard_keys.py run through the SAME rule functions the Claude Code
       hooks use. This is the point: hooks only fire when the AGENT writes a
       file, so a human edit -- or any change made before the hooks existed --
       is otherwise never checked. The sweep closes that hole.

    2. SEED CLOSURE (container)       Each skill's validate_templates.py proves
       its templates still build working artefacts against the portable seed
       alone (see seed_kit.py). Run for xyz-bt-lib-node, xyz-bt-lib-adapter and
       xyz-bt-facade-project.

    3. LIBRARY SMOKE (container)      demo_composites_tree.build_tree() is
       actually built and ticked against StubFacade -- it has no __main__, so
       merely executing the file proves nothing -- plus the three runnable
       demos.

WHY IT RUNS ON THE HOST
    .claude/skills and .claude/hooks are NOT bind-mounted into the ainex
    container, while xyz_bt_lib is. So this script stages the engine layer into
    the shared mount (/home/pi/docker/tmp by default), then invokes the
    container for anything that needs py_trees or ROS. The static sweep needs
    neither and runs here directly.

    Host<->container path translation is read from `docker inspect`, not
    hardcoded, so the script survives a container recreated with different
    mounts.

USAGE
    python3 /home/pi/docker/ros_ws_src/xyz_bt_lib/tools/validate_engine.py
    python3 .../validate_engine.py --skip-container    # fast static-only pass

Exit code 0 = everything passed; 1 = at least one failure. Advisory findings
(blackboard_keys ROSA mirroring) are reported as WARN and do not fail the run.
"""
import argparse
import ast
import os
import re
import shutil
import subprocess
import sys

LIB_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB_SRC = os.path.join(LIB_ROOT, 'src', 'xyz_bt_lib')

DEFAULT_SKILLS_DIR = '/home/pi/.claude/skills'
DEFAULT_HOOKS_DIR = '/home/pi/.claude/hooks'
DEFAULT_STAGE = '/home/pi/docker/tmp/.engine_validate'
DEFAULT_CONTAINER = 'ainex'

#: Skills that carry a seed-closure validator, in dependency order (node
#: templates first: everything else builds on the node contract).
SKILL_VALIDATORS = (
    'xyz-bt-lib-node',
    'xyz-bt-lib-adapter',
    'xyz-bt-facade-project',
)

#: SKILL.md files that make checkable claims about the facade contract. None of
#: them restates the signatures — the copy in xyz-bt-lib-adapter that did was only
#: actually deleted Aug 13 2026, having drifted to the point of missing move_head's
#: tilt_pos and every go_step/turn_step override param. This list had excluded that
#: file, so neither its counts nor its "removed from contract" list were ever checked;
#: it is included now. Those two claims drift just as silently as signatures did.
FACADE_CLAIM_SKILLS = (
    'xyz-bt-lib-node',
    'xyz-bt-lib-adapter',
    'xyz-bt-facade-project',
)

_FACADE_PROBE_SRC = '''\
import importlib.util, json, sys
spec = importlib.util.spec_from_file_location("bf", sys.argv[1])
bf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bf)
F = bf.XyzBTFacade
abstract = sorted(F.__abstractmethods__)
public = sorted(n for n, v in vars(F).items() if callable(v) and not n.startswith("_"))
print(json.dumps({"abstract": abstract,
                  "concrete": [m for m in public if m not in abstract]}))
'''

#: Demos with a __main__ that exercises real behaviour. demo_composites_tree is
#: absent on purpose -- it only defines build_tree(), so it is driven by
#: _SMOKE_SRC below instead.
RUNNABLE_DEMOS = (
    'demo_hysteresis.py',
    'demo_latched_dwell.py',
    'demo_memory_interaction.py',
)

#: Blackboard seed values for the smoke tick. Only keys whose type matters need
#: an entry; anything else a node declares in BB_READS is seeded as None, which
#: is what a real tick sees before the first sensor message arrives.
_SMOKE_SRC = '''\
import sys
sys.path.insert(0, sys.argv[1])
import py_trees
import demo_composites_tree as demo
from xyz_bt_lib.core.stub_facade import StubFacade

TYPED_DEFAULTS = {
    '/latched/robot_state':     'stand',
    '/latched/tracked_objects': {},
    '/latched/detection_source': '',
}

tree = demo.build_tree(facade=StubFacade())

# setup() is where nodes attach their blackboard client; py_trees does NOT
# recurse it for a bare composite, so ticking without this leaves _bb None.
tree.setup_with_descendants()

storage = py_trees.blackboard.Blackboard.storage
seeded = 0
for node in tree.iterate():
    for key in (getattr(node, 'BB_READS', None) or []):
        if key not in storage:
            storage[key] = TYPED_DEFAULTS.get(key)
            seeded += 1

for _ in range(3):
    tree.tick_once()

nodes = len(list(tree.iterate()))
assert tree.status is not None, 'tree never produced a status'
print('built {} nodes, seeded {} BB keys, ticked 3x, root -> {}'.format(
    nodes, seeded, tree.status.name))
'''

results = []


def record(state, name, detail=''):
    results.append((state, name, detail))


def run(cmd):
    """Return (returncode, combined output)."""
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return proc.returncode, proc.stdout.decode('utf-8', 'replace')


# ---------------------------------------------------------------------------
# 1. Library sweep (host, static)
# ---------------------------------------------------------------------------

def all_py_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d != '__pycache__']
        for fn in sorted(filenames):
            if fn.endswith('.py'):
                yield os.path.join(dirpath, fn)


#: How many checks library_sweep() must record. A sweep that silently stops
#: happening is indistinguishable from a clean run in the report, so the count
#: is asserted rather than trusted.
#: 4 -> 5 when path_table_agrees() joined the sweep (the xyz_paths drift check).
EXPECTED_SWEEPS = 5


def _call_name(call):
    return getattr(call.func, 'id', None) or getattr(call.func, 'attr', '?')


def hooks_import_safe(hooks_dir):
    """Every hook must be importable without executing its body.

    This sweep imports the hooks to reuse their path patterns and check
    functions. A hook that calls main() at module scope therefore RUNS on
    import: json.load(sys.stdin) fails on non-hook stdin, the except branch
    calls sys.exit(0), and SystemExit is not an Exception -- so it tore the
    whole validator down before it printed anything, and .githooks/pre-push
    gated on exit 0. That is exactly what happened; this makes the next
    reintroduction name the file instead of taking the gate down with it.
    """
    label = 'sweep: hooks are import-safe'
    offenders = []
    count = 0
    for fn in sorted(os.listdir(hooks_dir)):
        if not fn.endswith('.py'):
            continue
        count += 1
        path = os.path.join(hooks_dir, fn)
        try:
            module = ast.parse(open(path, encoding='utf-8').read(), path)
        except Exception as exc:                  # noqa: BLE001
            offenders.append('{}: unparseable ({})'.format(fn, exc))
            continue
        # A bare call statement at column 0 -- `main()`. Module-level
        # assignments (_NODE_PATTERN = re.compile(...)) are not statements of
        # this shape and are left alone.
        for node in module.body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
                offenders.append(
                    '{}:{}: calls {}() at module scope -- importing this hook '
                    'executes it. Wrap it in `if __name__ == "__main__":`.'
                    .format(fn, node.lineno, _call_name(node.value)))
    if not count:
        record('FAIL', label, 'no .py files found in {}'.format(hooks_dir))
    elif offenders:
        record('FAIL', label, '\n'.join(offenders))
    else:
        record('PASS', label, '{} hooks clean'.format(count))


def path_table_agrees(hooks_dir):
    """xyz_paths.py's copies of the guards' path patterns must be byte-identical.

    xyz_paths deliberately COPIES rather than imports (a guard that imports another
    guard fails open together with it). Two copies of a regex are two answers to
    "is this a node?" the moment one is edited -- and the coverage guard would then
    report UNKNOWN for files a content guard is in fact checking, or worse, stay
    silent for files nothing checks. This assertion is the only thing preventing
    that; deleting it silently converts xyz_paths from a map into a liability.
    """
    label = 'sweep: xyz_paths mirrors guard patterns'
    try:
        import xyz_paths
        import xyz_bt_lib_guard as libguard
        import xyz_behavior_guard as behguard
        import xyz_bt_tree_pre_guard as treeguard
    except Exception as exc:                      # noqa: BLE001
        record('FAIL', label, 'cannot import: {}: {}'.format(type(exc).__name__, exc))
        return

    checks = (
        ('_NODE_PATTERN',     xyz_paths._NODE_PATTERN,     libguard._NODE_PATTERN),
        ('_ADAPTER_PATTERN',  xyz_paths._ADAPTER_PATTERN,  libguard._ADAPTER_PATTERN),
        ('_BB_KEYS_PATTERN',  xyz_paths._BB_KEYS_PATTERN,  libguard._BB_KEYS_PATTERN),
        ('_TREE_PATTERN',     xyz_paths._TREE_PATTERN,     treeguard._TREE_PATTERN),
        ('_BEHAVIOR_PATTERN', xyz_paths._BEHAVIOR_PATTERN, behguard._PATTERN),
    )
    problems = []
    for name, mine, theirs in checks:
        if mine.pattern != theirs.pattern:
            problems.append(
                '{}: xyz_paths has {!r}, the guard has {!r}'
                .format(name, mine.pattern, theirs.pattern))

    real_prefixes = tuple(p for p, _ in behguard._CHECK_MAP)
    if xyz_paths._CHECK_PREFIX_STRINGS != real_prefixes:
        problems.append(
            '_CHECK_MAP prefixes: xyz_paths has {}, xyz_behavior_guard has {}'
            .format(list(xyz_paths._CHECK_PREFIX_STRINGS), list(real_prefixes)))

    if problems:
        record('FAIL', label, '\n'.join(problems))
    else:
        record('PASS', label,
               '{} patterns + {} _CHECK_MAP prefixes identical'
               .format(len(checks), len(real_prefixes)))


def library_sweep(hooks_dir):
    sys.path.insert(0, hooks_dir)
    try:
        import xyz_bt_lib_guard as guard
        import xyz_bt_l1_running_guard as l1guard
        from xyz_bt_lib_guard import check_node, check_adapter, check_bb_keys
        from xyz_bt_l1_running_guard import check_l1_running
    except SystemExit as exc:
        # Not an Exception, so `except Exception` never saw it: a hook that runs
        # main() at import calls sys.exit() and kills this process silently.
        record('FAIL', 'library sweep: import guards',
               'a hook module called sys.exit({}) while being imported -- it '
               'executes its body at module scope. See the import-safe sweep '
               'above for which one.'.format(exc.code))
        return
    except Exception as exc:                      # noqa: BLE001
        record('FAIL', 'library sweep: import guards',
               '{}: {}'.format(type(exc).__name__, exc))
        return

    # File selection reuses the hooks' OWN path patterns rather than a parallel
    # copy. A second copy would drift -- and then the sweep and the hook would
    # disagree about what counts as a node, which is the whole failure mode this
    # tool exists to prevent.
    files = list(all_py_files(LIB_SRC))
    node_files = [p for p in files if guard._NODE_PATTERN.search(p)]
    adapter_files = [p for p in files if guard._ADAPTER_PATTERN.search(p)]
    bb_key_files = [p for p in files if guard._BB_KEYS_PATTERN.search(p)]
    l1_files = [p for p in files if l1guard.applies_to(p)]

    def sweep(label, paths, fn, advisory=False):
        offenders = []
        count = 0
        for path in paths:
            count += 1
            try:
                content = open(path, encoding='utf-8').read()
            except Exception as exc:              # noqa: BLE001
                offenders.append('{}: unreadable ({})'.format(path, exc))
                continue
            for v in fn(content, path):
                offenders.append('{}\n    {}'.format(
                    os.path.relpath(path, LIB_ROOT), v))
        if not count:
            # Not advisory even for an advisory sweep: `advisory` means the
            # findings do not block, not that the sweep may quietly vanish. A
            # pattern matching nothing reads exactly like a clean run.
            record('FAIL', label,
                   'no files matched -- the path pattern selects nothing, so '
                   'this check swept an empty set')
        elif offenders:
            record('WARN' if advisory else 'FAIL', label,
                   '\n'.join(offenders))
        else:
            record('PASS', label, '{} files clean'.format(count))

    path_table_agrees(hooks_dir)

    sweep('sweep: node conformance (check_node)', node_files,
          lambda c, p: check_node(c, p))
    sweep('sweep: L1 purity (check_l1_running)', l1_files,
          lambda c, p: check_l1_running(c, p))
    sweep('sweep: adapter protocol (check_adapter)', adapter_files,
          lambda c, p: check_adapter(c))
    sweep('sweep: blackboard keys (check_bb_keys, advisory)', bb_key_files,
          lambda c, p: check_bb_keys(c), advisory=True)


# ---------------------------------------------------------------------------
# 2 & 3. Container-side checks
# ---------------------------------------------------------------------------

def container_mounts(container):
    rc, out = run(['docker', 'inspect', container, '--format',
                   '{{range .Mounts}}{{.Source}}|{{.Destination}}\n{{end}}'])
    if rc != 0:
        raise RuntimeError('docker inspect {} failed:\n{}'.format(container, out))
    mounts = []
    for line in out.splitlines():
        if '|' in line:
            src, dst = line.split('|', 1)
            mounts.append((src.rstrip('/'), dst.rstrip('/')))
    # longest source first, so the most specific mount wins
    return sorted(mounts, key=lambda m: -len(m[0]))


def to_container_path(host_path, mounts):
    host_path = os.path.abspath(host_path).rstrip('/')
    for src, dst in mounts:
        if host_path == src:
            return dst
        if host_path.startswith(src + '/'):
            return dst + host_path[len(src):]
    raise RuntimeError(
        '{} is not inside any bind mount of the container -- pass --stage-dir '
        'somewhere under a mounted directory'.format(host_path))


def stage_engine(skills_dir, hooks_dir, stage_dir):
    """Copy skills+hooks into the shared mount so the container can see them."""
    if os.path.exists(stage_dir):
        shutil.rmtree(stage_dir)
    os.makedirs(stage_dir)
    shutil.copytree(skills_dir, os.path.join(stage_dir, 'skills'),
                    ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(hooks_dir, os.path.join(stage_dir, 'hooks'),
                    ignore=shutil.ignore_patterns('__pycache__'))
    with open(os.path.join(stage_dir, '_smoke.py'), 'w', encoding='utf-8') as fh:
        fh.write(_SMOKE_SRC)
    return stage_dir


def facade_claims(args, dexec, c_stage, c_lib_root):
    """Check what SKILL.md *claims* about XyzBTFacade against the real class.

    The claims are prose, so this deliberately checks only what prose can state
    unambiguously and what has actually gone wrong before:
      - the abstract / concrete method counts,
      - that every name listed as "removed from contract" really is gone.
    Signatures are not checked here because no SKILL.md restates them any more --
    they all point at base_facade.py, which is the fix, not a gap.
    """
    probe = os.path.join(args.stage_dir, '_facade_probe.py')
    with open(probe, 'w', encoding='utf-8') as fh:
        fh.write(_FACADE_PROBE_SRC)
    rc, out = dexec(['python3', '{}/_facade_probe.py'.format(c_stage),
                     '{}/src/xyz_bt_lib/core/base_facade.py'.format(c_lib_root)])
    if rc != 0:
        record('FAIL', 'facade claims in SKILL.md', out.strip())
        return

    import json
    real = json.loads(out.strip().splitlines()[-1])
    n_abstract, n_concrete = len(real['abstract']), len(real['concrete'])
    known = set(real['abstract']) | set(real['concrete'])

    problems = []

    # Constants that belong to a template, never to prose. A copy of _GO_CFG in
    # xyz-bt-facade-project/SKILL.md drifted to arm_swap=False against the
    # template's 30 and was deleted Aug 2026; this stops it coming back.
    for skill_dir in sorted(os.listdir(args.skills_dir)):
        path = os.path.join(args.skills_dir, skill_dir, 'SKILL.md')
        if not os.path.isfile(path):
            continue
        text = open(path, encoding='utf-8').read()
        for const in ('_GO_CFG', '_TURN_CFG', '_GO_STEP_VEL', '_TURN_STEP_VEL'):
            if re.search(r'^\s*' + const + r'\s*=', text, re.M):
                problems.append(
                    '{}: defines {} inline; that constant lives in a template '
                    '(bt_node.py.tpl / _runtime_io.py.tpl) — point at it instead'
                    .format(skill_dir, const))

    for skill in FACADE_CLAIM_SKILLS:
        path = os.path.join(args.skills_dir, skill, 'SKILL.md')
        if not os.path.isfile(path):
            problems.append('{}: SKILL.md missing'.format(skill))
            continue
        text = open(path, encoding='utf-8').read()

        m = re.search(r'(\d+)\s+abstract methods', text)
        if m and int(m.group(1)) != n_abstract:
            problems.append('{}: claims {} abstract methods, real count is {}'
                            .format(skill, m.group(1), n_abstract))
        m = re.search(r'(?:plus\s+)?(\d+)\s+concrete', text)
        if m and int(m.group(1)) != n_concrete:
            problems.append('{}: claims {} concrete methods, real count is {}'
                            .format(skill, m.group(1), n_concrete))

        block = re.search(r'^Removed from contract.*?$\n((?:^- .*$\n?)+)',
                          text, re.M | re.S)
        if block:
            for name in re.findall(r'^- `(\w+)`', block.group(1), re.M):
                if name in known:
                    problems.append(
                        '{}: lists `{}` as removed from the contract, but '
                        'XyzBTFacade still defines it'.format(skill, name))

    if problems:
        record('FAIL', 'facade claims in SKILL.md', '\n'.join(problems))
    else:
        record('PASS', 'facade claims in SKILL.md',
               '{} abstract + {} concrete methods; removed-method lists accurate'
               .format(n_abstract, n_concrete))


def container_checks(args, mounts):
    stage_dir = stage_engine(args.skills_dir, args.hooks_dir, args.stage_dir)
    c_stage = to_container_path(stage_dir, mounts)
    c_lib_root = to_container_path(LIB_ROOT, mounts)
    c_examples = c_lib_root + '/examples'

    def dexec(cmd):
        return run(['docker', 'exec', '-u', args.user, args.container] + cmd)

    # --- seed closure -----------------------------------------------------
    for skill in SKILL_VALIDATORS:
        script = '{}/skills/{}/validate_templates.py'.format(c_stage, skill)
        rc, out = dexec(['bash', '-lc',
                         'source /home/ubuntu/ros_ws/devel/setup.bash >/dev/null 2>&1; '
                         'python3 -u {} --lib-root {} --hooks-dir {}/hooks'
                         .format(script, c_lib_root, c_stage)])
        label = 'seed closure: {}'.format(skill)
        summary = ''
        for line in out.splitlines():
            if re.match(r'\s*\d+ passed', line):
                summary = line.strip()
        if rc == 0:
            record('PASS', label, summary or 'validator exited 0')
        else:
            record('FAIL', label, out.strip())

    # --- library smoke ----------------------------------------------------
    rc, out = dexec(['bash', '-lc',
                     'source /home/ubuntu/ros_ws/devel/setup.bash && '
                     'python3 -u {}/_smoke.py {}'.format(c_stage, c_examples)])
    if rc == 0:
        record('PASS', 'smoke: demo_composites_tree builds and ticks',
               out.strip().splitlines()[-1] if out.strip() else '')
    else:
        record('FAIL', 'smoke: demo_composites_tree builds and ticks', out.strip())

    # --- facade claims in SKILL.md ---------------------------------------
    facade_claims(args, dexec, c_stage, c_lib_root)

    for demo in RUNNABLE_DEMOS:
        rc, out = dexec(['bash', '-lc',
                         'source /home/ubuntu/ros_ws/devel/setup.bash && '
                         'python3 -u {}/{}'.format(c_examples, demo)])
        label = 'demo: {}'.format(demo)
        if rc == 0:
            record('PASS', label, '{} lines of output'.format(
                len(out.strip().splitlines())))
        else:
            record('FAIL', label, out.strip())


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Validate the xyz_bt_lib engine: guards, seed closure, demos.')
    ap.add_argument('--skills-dir', default=DEFAULT_SKILLS_DIR)
    ap.add_argument('--hooks-dir', default=DEFAULT_HOOKS_DIR)
    ap.add_argument('--stage-dir', default=DEFAULT_STAGE,
                    help='staging dir; MUST live under a container bind mount')
    ap.add_argument('--container', default=DEFAULT_CONTAINER)
    ap.add_argument('--user', default='ubuntu',
                    help='container user (pip --user packages live in its home)')
    ap.add_argument('--skip-container', action='store_true',
                    help='run only the host-side static sweep')
    args = ap.parse_args(argv)

    hooks_import_safe(args.hooks_dir)

    before = len(results)
    library_sweep(args.hooks_dir)
    swept = results[before:]
    if len(swept) != EXPECTED_SWEEPS and not any(r[0] == 'FAIL' for r in swept):
        record('FAIL', 'library sweep completeness',
               'recorded {} checks, expected {} -- a sweep stopped running '
               'without failing'.format(len(swept), EXPECTED_SWEEPS))

    if not args.skip_container:
        try:
            mounts = container_mounts(args.container)
            container_checks(args, mounts)
        except Exception as exc:                  # noqa: BLE001
            record('FAIL', 'container checks',
                   '{}: {}'.format(type(exc).__name__, exc))

    return report()


def report():
    print('\n=== xyz_bt_lib engine validation ===')
    if not results:
        # "0 passed, 0 failed" used to exit 0 and read like success. Nothing
        # having run is the worst outcome this tool can have, not the best.
        print('  FAIL no checks were recorded -- the validator never ran a '
              'single check')
        print('\n0 passed, 1 failed, 0 warnings')
        return 1
    for state, name, detail in results:
        print('  {:4} {}'.format(state, name))
        if detail:
            for line in str(detail).rstrip().splitlines():
                print('        ' + line)
    failed = [r for r in results if r[0] == 'FAIL']
    warned = [r for r in results if r[0] == 'WARN']
    print('\n{} passed, {} failed, {} warnings'.format(
        len(results) - len(failed) - len(warned), len(failed), len(warned)))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
