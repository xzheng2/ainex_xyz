#!/usr/bin/env python3
"""
Run identity, per-run log directories, and retention (stdlib only, Python 3.8+).

WHY THIS MODULE EXISTS
    The observability logger writes fixed filenames and truncates them at startup, so
    every run destroyed the previous one's output. An ablation experiment is N variants
    times M repeats, all of which have to survive side by side, and each of which has to
    be attributable to the exact code that produced it.

    This module gives every run its own directory under ``log/runs/<run_id>/`` and stamps
    it with ``run_meta.json``. The logger itself needs no change: it already takes
    explicit paths, so pointing it at a fresh directory makes its startup truncate hit a
    brand-new empty file.

WHY THE LEGACY FILENAMES SURVIVE AS SYMLINKS
    Several readers hardcode the old fixed paths in ``log/``:

      - ROSA: ``xyz_agent_tools/bt_analysis/raw_tick.py`` uses ``BT_OBS_DIR =
        "/opt/ainex_bt_log"`` plus fixed basenames, reached through the read-only mount
        declared in ``docker/docker-compose.yml``
      - the ``bt_log_read_guard.py`` Claude Code hook
      - the ``xyz-bt-lastrun-diagnose`` skill and ``lastrun_digest_cli.py``

    So ``open_run_dir()`` refreshes a symlink in ``log/`` for each legacy name, pointing
    at the newest run. Every one of those readers keeps working untouched.

    The links are **relative** on purpose. The same directory is
    ``/home/pi/docker/ros_ws_src/xyz_behavior/log`` on the host and ``/opt/ainex_bt_log``
    inside the ROSA container; an absolute link would resolve in exactly one of them.

    Symlinks are for readers only. ``BlackboardCurrentWriter.write_current()`` and the
    logger's ``_atomic_write()`` both finish with ``os.replace(tmp, path)``, which
    REPLACES a symlink with a regular file. Every writer must therefore be pointed at the
    run directory itself -- see ``bt_node.py.tpl``.

WHY BODY ID COMES FROM THE WIFI HOTSPOT
    Each robot runs an access point whose SSID already encodes the unit (``HW-...``), so
    it is an identifier that exists whether or not anyone remembered to configure one.
    ``hostname`` is not usable: every stock Raspberry Pi answers to ``raspberrypi``, and
    two bodies sharing an id would silently merge their experiment results.

    NetworkManager is only reachable from the host -- ``nmcli`` exists inside the ainex
    container but cannot connect to its D-Bus. So the host probes once and caches the
    answer in ``log/.body_id``, which is inside the bind mount and therefore readable by
    the containerised node.

WHY ``dirty`` HAS A SCOPE
    ``run_meta.json``'s ``git.dirty`` answers one question: can this run's code be
    recovered from a commit? It is therefore computed over ``RUNTIME_PATHS`` only, not
    the whole repository -- the repo root is ``/home/pi`` and also holds Claude Code
    hooks, skills and templates that no running BT node ever loads. See ``RUNTIME_PATHS``
    for the reasoning; ``repo_dirty`` keeps the unscoped signal as information.
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # xyz_run_lab
_WS_SRC = os.path.dirname(_PKG_DIR)                                      # ros_ws_src

#: The project package. This module owns run identity and body calibration, but not the
#: log tree: `log/` stays in xyz_behavior because ROSA's /opt/ainex_bt_log mount, the
#: bt_log_read_guard hook and the diagnose skill all address it by that path. Every
#: caller passes log_dir explicitly; this constant only backs the __main__ self-report
#: and error messages that point at tools living over there.
_BEHAVIOR_DIR = os.path.join(_WS_SRC, 'xyz_behavior')

#: Default number of run directories kept. Older ones are deleted unconditionally --
#: publish runs you care about (tools/publish_runs.py) before they age out.
DEFAULT_MAX_RUNS = 10

#: Filenames that readers outside this package expect to find directly in log/.
#: Kept byte-identical inside each run directory: renaming them would break ROSA, the
#: read guard and the diagnose skill all at once, and none of them gains anything.
LEGACY_LINKS = (
    'bt_debug_lastrun.jsonl',
    'bt_ros_comm_debug_lastrun.jsonl',
    'bt_debug_recent.jsonl',
    'bt_ros_comm_debug_recent.jsonl',
    'bb_current.json',
    'infra_comm_manifest_lastrun.json',
)

#: Of those, the ones created empty up front so a run that has not logged anything yet
#: reads as an empty run rather than a missing file. The two JSON state files are left to
#: their writers: an empty file is not valid JSON, which would be worse than absent.
_PRECREATED = tuple(n for n in LEGACY_LINKS if n.endswith('.jsonl'))

#: Repository paths whose state can change what the robot does, and therefore the only
#: ones that decide whether a run is reproducible. Everything else in this repository --
#: .claude/ hooks, skills, templates and memory, and docker/rosa-agent -- is development-
#: or analysis-time only: none of it is loaded by a running BT node.
#:
#: Scoping matters because `dirty` has a job: stopping irreproducible data from reaching
#: an ablation table. Computed over the whole repo, editing one hook would light it up
#: for every run afterwards, and a warning that is always on is a warning nobody reads.
#:
#: Templates are not a hole in this. Edit a .tpl AND regenerate a project, and the
#: generated files land under docker/ros_ws_src/ where they are caught anyway; edit a
#: .tpl without regenerating, and the robot is running the old code, which is exactly
#: the case that should not be flagged.
RUNTIME_PATHS = ('docker/ros_ws_src',)

#: How many dirty paths to keep in run_meta.json before truncating.
_DIRTY_SAMPLE = 20

#: Files whose contents change how the robot physically moves. Hashed into every run so a
#: result can be tied to the calibration in force at the time, not just to the code.
_CALIBRATION_FILES = (
    'ainex_driver/ainex_kinematics/config/walking_param.yaml',
    'ainex_driver/ainex_kinematics/config/walking_offset.yaml',
    'ainex_driver/ainex_kinematics/config/servo_controller.yaml',
    'ainex_driver/ainex_kinematics/config/init_pose.yaml',
)

#: Directories skipped by tree_digest(): build artifacts and VCS metadata, neither of
#: which is part of what a lane ships. __pycache__ in particular is written by whichever
#: Python happens to import first, so hashing it would make the fingerprint depend on
#: run order rather than on content.
_DIGEST_SKIP_DIRS = frozenset(('__pycache__', '.git'))

#: The trees that must be byte-identical across all four lanes, each fingerprinted into
#: every run. Two are packages the lanes share (xyz_run_lab is the measuring apparatus
#: itself; xyz_perception feeds every tree), and two are the agent's own configuration --
#: the variable actually under study, so a silent edit to it during a campaign would
#: invalidate the comparison it is meant to support.
#:
#: ActionGroups is deliberately NOT here. The motion assets are hand-tuned and expected
#: to change during a campaign; fingerprinting them would flag ordinary work as drift,
#: and a constant that is not actually constant teaches everyone to ignore the field.
_CONSTANT_TREES = (
    ('xyz_run_lab',    os.path.join(_WS_SRC, 'xyz_run_lab')),
    ('xyz_perception', os.path.join(_WS_SRC, 'xyz_perception')),
    ('claude_hooks',   os.path.expanduser('~/.claude/hooks')),
    ('claude_skills',  os.path.expanduser('~/.claude/skills')),
)

BODY_ID_CACHE = '.body_id'

#: Experiment arm ("swim lane"). Four SD cards give the coding agent four different
#: asset mixes, and all four run on the SAME robot -- so body_id is identical across
#: them and cannot separate the arms. Without this field every run lands in one
#: undifferentiated partition and the ablation is unanalysable.
LANE_CACHE = '.lane'
VALID_LANES = ('a', 'b', 'c', 'd')

#: Which experiment a run belongs to (exp1, exp2, ...). One level above lane: a lane is
#: fixed for the life of an SD card, but the same card takes part in successive studies,
#: so this one legitimately changes over time -- which is exactly why the two cannot be
#: the same field. Pooling two studies under one prefix would silently average across
#: experiments that were never meant to be compared.
STUDY_CACHE = '.study'

#: Deliberately a shape, not a fixed set. A closed tuple would mean editing xyz_run_lab
#: to start a third experiment -- and this package has to stay byte-identical across all
#: four lanes, so changing it mid-campaign is worse than the typo risk an open set
#: carries. The pattern still rejects anything that would misbehave as a path component;
#: use `new_run.py --print-study` to confirm what a card is set to.
STUDY_RE = re.compile(r'^[a-z0-9][a-z0-9_-]*$')


# --------------------------------------------------------------------------- #
# Body identity
# --------------------------------------------------------------------------- #

class BodyIdError(RuntimeError):
    """Raised when this robot cannot be identified. Never guessed around."""


def _nmcli(*args):
    try:
        proc = subprocess.run(['nmcli'] + list(args),
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except OSError:
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.decode('utf-8', 'replace')


def probe_hotspot_ssid():
    """Return the SSID of the access point this machine is running, or None.

    Only works on the host: NetworkManager's D-Bus is not exposed to the containers.
    A machine may hold several wireless connections at once -- typically one AP and one
    upstream client link -- so the AP is picked by mode, not by position.
    """
    active = _nmcli('-t', '-f', 'NAME,TYPE', 'connection', 'show', '--active')
    if not active:
        return None
    for line in active.splitlines():
        if not line.endswith(':802-11-wireless'):
            continue
        name = line.rsplit(':', 1)[0].replace('\\:', ':')
        mode = _nmcli('-t', '-f', '802-11-wireless.mode', 'connection', 'show', name)
        if mode and mode.strip().split(':', 1)[-1].strip() == 'ap':
            return name
    return None


def resolve_body_id(log_dir, probe=True, cache=True):
    """Identify this robot body.

    Order: ``AINEX_BODY_ID`` -> ``<log_dir>/.body_id`` cache -> hotspot SSID probe.
    There is deliberately no fallback beyond that: an unidentified body writing results
    under a guessed name is worse than a run that refuses to start.
    """
    env = (os.environ.get('AINEX_BODY_ID') or '').strip()
    if env:
        return env

    cache_path = os.path.join(log_dir, BODY_ID_CACHE)
    try:
        with open(cache_path, encoding='utf-8') as fh:
            cached = fh.read().strip()
        if cached:
            return cached
    except OSError:
        pass

    ssid = probe_hotspot_ssid() if probe else None
    if ssid:
        if cache:
            try:
                os.makedirs(log_dir, exist_ok=True)
                with open(cache_path, 'w', encoding='utf-8') as fh:
                    fh.write(ssid + '\n')
            except OSError:
                pass                      # a read-only mount is not a reason to fail
        return ssid

    raise BodyIdError(
        "cannot identify this robot body.\n"
        "  AINEX_BODY_ID is unset, {} holds nothing, and the hotspot SSID could not be\n"
        "  probed (nmcli is only usable on the host -- inside a container it cannot reach\n"
        "  NetworkManager's D-Bus).\n"
        "  Fix on the HOST:  python3 {}/tools/new_run.py --print-body-id\n"
        "  or set it by hand: export AINEX_BODY_ID=<hotspot-ssid>".format(
            cache_path, _BEHAVIOR_DIR))


# --------------------------------------------------------------------------- #
# Experiment identity: study, then arm
# --------------------------------------------------------------------------- #

class StudyError(RuntimeError):
    """Raised when the experiment a run belongs to is unknown. Never guessed around."""


def resolve_study(log_dir, study=None, cache=True):
    """Identify which experiment this card is currently running.

    Order: explicit argument -> ``AINEX_STUDY`` -> ``<log_dir>/.study`` cache, same shape
    as resolve_lane. Unlike the lane, this one is expected to be re-set: a card finishes
    exp1 and is handed to exp2, and ``--study exp2`` rewrites the cache.

    Hard-fails rather than defaulting to the most recent study. Silently filing exp2 runs
    under exp1 pools two experiments into one partition, and the resulting table averages
    across a boundary nobody can see afterwards.
    """
    for value, source in ((study, 'argument'),
                          (os.environ.get('AINEX_STUDY'), 'AINEX_STUDY'),
                          (_read_cache(log_dir, STUDY_CACHE), 'cache')):
        value = (value or '').strip().lower()
        if not value:
            continue
        if not STUDY_RE.match(value):
            raise StudyError(
                'study {!r} (from {}) is not a usable name: lowercase letters, digits, '
                '"_" and "-" only, starting with a letter or digit'.format(value, source))
        if cache and source == 'argument':
            _write_cache(log_dir, STUDY_CACHE, value)
        return value

    raise StudyError(
        'cannot identify which experiment this run belongs to.\n'
        '  AINEX_STUDY is unset and {} holds nothing.\n'
        '  Set it when a card starts a study:  python3 .../xyz_behavior/tools/new_run.py '
        '--study exp1\n'
        '  or by hand:                         export AINEX_STUDY=exp1'.format(
            os.path.join(log_dir, STUDY_CACHE)))


class LaneError(RuntimeError):
    """Raised when this card's experiment arm is unknown. Never guessed around."""


def resolve_lane(log_dir, lane=None, cache=True):
    """Identify which experiment arm this SD card is.

    Order: explicit argument -> ``AINEX_LANE`` -> ``<log_dir>/.lane`` cache. Mirrors
    resolve_body_id deliberately, minus the probe step: there is nothing to probe.
    All four cards boot the same robot, so nothing about the running system reveals
    which arm it is -- the answer has to be written down once when the card is imaged.

    The cache lives in ``log/`` next to ``.body_id``, NOT in this package: xyz_run_lab
    is the instrumentation every arm shares byte-for-byte, and a file that differs per
    card cannot live inside it. ``log/`` is gitignored machine state, which is also
    why writing it does not make every subsequent run read as dirty.

    Hard-fails rather than defaulting. A run that cannot be attributed to an arm is
    not evidence, and silently filing it under a guessed arm corrupts the table it
    lands in.
    """
    for value, source in ((lane, 'argument'),
                          (os.environ.get('AINEX_LANE'), 'AINEX_LANE'),
                          (_read_cache(log_dir, LANE_CACHE), 'cache')):
        value = (value or '').strip().lower()
        if not value:
            continue
        if value not in VALID_LANES:
            raise LaneError(
                'lane {!r} (from {}) is not one of {}'.format(
                    value, source, '/'.join(VALID_LANES)))
        if cache and source == 'argument':
            _write_cache(log_dir, LANE_CACHE, value)
        return value

    raise LaneError(
        "cannot identify this card's experiment lane.\n"
        "  AINEX_LANE is unset and {} holds nothing.\n"
        "  Set it once per card:  python3 .../xyz_behavior/tools/new_run.py --lane <{}>\n"
        "  or by hand:            export AINEX_LANE=<{}>".format(
            os.path.join(log_dir, LANE_CACHE),
            '|'.join(VALID_LANES), '|'.join(VALID_LANES)))


def _read_cache(log_dir, name):
    try:
        with open(os.path.join(log_dir, name), encoding='utf-8') as fh:
            return fh.read().strip()
    except OSError:
        return None


def _write_cache(log_dir, name, value):
    try:
        os.makedirs(log_dir, exist_ok=True)
        with open(os.path.join(log_dir, name), 'w', encoding='utf-8') as fh:
            fh.write(value + '\n')
    except OSError:
        pass                          # a read-only mount is not a reason to fail


# --------------------------------------------------------------------------- #
# Run directories
# --------------------------------------------------------------------------- #

def new_run_id(variant='run', trial=1):
    """Build a run id that sorts chronologically as plain text."""
    return '{}_{}_t{}'.format(time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()),
                              variant, trial)


def runs_root(log_dir):
    return os.path.join(log_dir, 'runs')


def open_run_dir(log_dir, run_id=None, max_runs=DEFAULT_MAX_RUNS, log=None):
    """Create (or adopt) this run's directory, republish the legacy links, prune.

    `run_id` defaults to ``AINEX_RUN_ID`` -- which is how a run opened on the host by
    tools/new_run.py is handed to the containerised node -- and then to a fresh
    timestamped id. An existing directory is adopted rather than replaced, so the meta
    the host already wrote survives.

    Returns the absolute run directory.
    """
    log = log or (lambda msg: None)
    run_id = run_id or (os.environ.get('AINEX_RUN_ID') or '').strip() or new_run_id()

    run_dir = os.path.join(runs_root(log_dir), run_id)
    os.makedirs(run_dir, exist_ok=True)

    for name in _PRECREATED:
        path = os.path.join(run_dir, name)
        if not os.path.exists(path):
            open(path, 'a').close()

    _refresh_links(log_dir, run_id)
    prune_runs(log_dir, max_runs=max_runs, keep=(run_id,), log=log)
    return run_dir


def _refresh_links(log_dir, run_id):
    """Point each legacy name in log_dir at this run, with a relative symlink."""
    for name in LEGACY_LINKS:
        link = os.path.join(log_dir, name)
        target = os.path.join('runs', run_id, name)
        if os.path.lexists(link):
            # A regular file here means an older layout wrote directly into log/, or a
            # writer was pointed at the link and os.replace() turned it into a file.
            if os.path.isdir(link) and not os.path.islink(link):
                shutil.rmtree(link)
            else:
                os.remove(link)
        os.symlink(target, link)


def prune_runs(log_dir, max_runs=DEFAULT_MAX_RUNS, keep=(), log=None):
    """Keep the newest `max_runs` run directories, delete the rest. Returns deleted ids.

    Deletion is unconditional -- an unpublished run is deleted like any other, and only
    noted on the way out. Publish what matters (tools/publish_runs.py) before it ages out.
    """
    log = log or (lambda msg: None)
    root = runs_root(log_dir)
    if max_runs is None or max_runs <= 0 or not os.path.isdir(root):
        return []

    entries = []
    for name in os.listdir(root):
        path = os.path.join(root, name)
        if not os.path.isdir(path):
            continue
        try:
            mtime = os.stat(path).st_mtime
        except OSError:
            continue
        entries.append((mtime, name))
    # mtime first so an unusually named run id cannot sort itself into deletion; the name
    # breaks ties for runs created inside the same second.
    entries.sort()

    doomed = [name for _, name in entries[:-max_runs]] if len(entries) > max_runs else []
    deleted = []
    for name in doomed:
        if name in keep:
            continue
        path = os.path.join(root, name)
        published = os.path.exists(os.path.join(path, '.published'))
        try:
            shutil.rmtree(path)
        except OSError as exc:
            log('could not prune {}: {}'.format(name, exc))
            continue
        deleted.append(name)
        log('pruned run {}{}'.format(
            name, '' if published else '  (NOT published to the results repo)'))
    return deleted


# --------------------------------------------------------------------------- #
# Provenance
# --------------------------------------------------------------------------- #

def _git(repo, *args):
    """Run a git command in `repo`; return stripped stdout, or None if git fails."""
    try:
        proc = subprocess.run(['git', '-C', repo] + list(args),
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except OSError:
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.decode('utf-8', 'replace').strip()


def code_provenance():
    """Describe the code repository this workspace belongs to.

    `dirty` is scoped to RUNTIME_PATHS, not the whole repository -- see that constant for
    why. `repo_dirty` keeps the whole-repository signal as information only.

    Inside the ROS container this reports unavailable: only the workspace source is
    bind-mounted, not the repository. That is why a run is normally opened on the host
    (tools/new_run.py) and its id handed over through AINEX_RUN_ID.
    """
    root = _git(os.path.dirname(os.path.abspath(__file__)), 'rev-parse', '--show-toplevel')
    if root is None:
        return {'available': False,
                'reason': 'workspace is not inside a git repository (expected inside the '
                          'ROS container: only the source tree is mounted, not the repo)'}
    # Untracked files are included (git status reports them by default): a node file that
    # was never committed changes behaviour just as much as an edited one.
    status = _git(root, 'status', '--porcelain', '--', *RUNTIME_PATHS) or ''
    paths = status.splitlines()
    repo_paths = (_git(root, 'status', '--porcelain') or '').splitlines()
    return {
        'available': True,
        'root':    root,
        'sha':     _git(root, 'rev-parse', 'HEAD'),
        'branch':  _git(root, 'rev-parse', '--abbrev-ref', 'HEAD'),
        'subject': _git(root, 'log', '-1', '--format=%s'),
        # Names the experiment baseline this run sits on, e.g. abl-20260811-gait-baseline.
        # A bare sha is unreadable in an ablation table and states no intent; a tag does
        # both. Falls back to the short sha when no tag is reachable.
        # Deliberately without --dirty: git's own dirty notion is whole-repo and would
        # contradict the scoped `dirty` below.
        'describe': _git(root, 'describe', '--tags', '--always'),
        'dirty':       bool(paths),
        'dirty_count': len(paths),
        # What `dirty` actually covers. Recorded rather than assumed: a reader years from
        # now must not have to guess which paths a `dirty: false` was claiming about.
        'dirty_scope': list(RUNTIME_PATHS),
        # A digest of the scoped status, so two dirty runs can be told apart -- equal
        # digests mean the same uncommitted state.
        'dirty_digest': hashlib.sha256(status.encode('utf-8')).hexdigest() if paths else None,
        # A sample only: the full list can run long and would drown the fields that
        # matter. The digest above is the identity; this is for eyeballing.
        'dirty_sample': paths[:_DIRTY_SAMPLE],
        # Whole-repository state, informational. Deliberately no path list: this repo is
        # published to a PUBLIC results repository, and paths outside RUNTIME_PATHS are
        # local working notes rather than something to broadcast.
        'repo_dirty':       bool(repo_paths),
        'repo_dirty_count': len(repo_paths),
    }


def sha256(path):
    h = hashlib.sha256()
    try:
        with open(path, 'rb') as fh:
            for chunk in iter(lambda: fh.read(65536), b''):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def calibration_digest(body_cfg=None):
    digest = {}
    for rel in _CALIBRATION_FILES:
        digest[rel] = sha256(os.path.join(_WS_SRC, rel))
    if body_cfg and os.path.isfile(body_cfg):
        digest[os.path.relpath(body_cfg, _WS_SRC)] = sha256(body_cfg)
    return digest


def tree_digest(root):
    """One hexdigest over every file under `root`, or None if `root` does not exist.

    Order is forced (`dirnames.sort()`, `sorted(filenames)`) because os.walk yields
    directory entries in filesystem order, which differs between two cards holding
    identical bytes -- an unordered walk would produce a digest that changes for no
    reason, which is worse than no digest at all.

    Each file contributes its relative path AND its content hash, so renaming a file
    changes the tree digest even though no byte of content moved.
    """
    if not os.path.isdir(root):
        return None
    h = hashlib.sha256()
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        dirnames[:] = [d for d in dirnames if d not in _DIGEST_SKIP_DIRS]
        for name in sorted(filenames):
            if name.endswith('.pyc'):
                continue
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, root).replace(os.sep, '/')
            content = sha256(full)
            # An unreadable file still counts as present: hashing a marker rather than
            # skipping it keeps a file that loses its read permission from looking
            # identical to a file that was deleted.
            h.update(rel.encode('utf-8') + b'\0'
                     + (content.encode('ascii') if content else b'<unreadable>'))
    return h.hexdigest()


def constants_digest():
    """Fingerprint of everything that is supposed to be IDENTICAL across the four lanes.

    The experiment only compares lanes if the instrument is the same in all of them.
    These five trees are the instrument: two packages plus the hand-tuned motion assets
    that the robot replays, and the agent's own hooks and skills. Recording their
    digests does not prevent an edit -- it makes one visible afterwards, in the run's own
    metadata, which is the only check that still works once the data is in the results
    repository and the working tree has moved on.

    The two `.claude/` entries resolve under the invoking user's home, so inside the
    `ainex` container (home /home/ubuntu) they simply do not exist and come back None.
    That is expected, not an error: a run started in the container legitimately cannot
    see the host's agent configuration.
    """
    return {name: tree_digest(path) for name, path in _CONSTANT_TREES}


def body_config_path(body_id):
    return os.path.join(_PKG_DIR, 'config', 'bodies', body_id + '.yaml')


def write_run_meta(run_dir, body_id, variant='', trial=0, params=None, note='',
                   lane='', study=''):
    """Stamp a run with what produced it. Returns the meta path.

    Written at the START of a run, before anything can drift.
    """
    # schema/2: git.dirty became scoped to RUNTIME_PATHS (it covered the whole repository
    # in /1) and gained dirty_scope / repo_dirty. The meaning of an existing field
    # changed, which is what a schema version is for.
    #
    # `study` and `lane` were added later WITHOUT a version bump, by that same rule: both
    # are purely additive and no existing field changed meaning. Nor is there anything to
    # tell apart -- not one /2 run had been published when they landed, so a "/2 without
    # study/lane" does not exist in any results repo.
    #
    # `constants_sha256` was added the same way and for the same reason: it is a new key
    # that no existing field's meaning depends on. Unlike study/lane, /2 runs WITHOUT it
    # do exist, so a reader must treat a missing key as "not recorded" -- which is what
    # absence already means, and is why this still does not need a version.
    meta = {
        'schema':      'ainex.run_meta/2',
        'run_id':      os.path.basename(run_dir.rstrip(os.sep)),
        'body_id':     body_id,
        'study':       study,
        'lane':        lane,
        'created_utc': time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()),
        'variant':     variant,
        'trial':       trial,
        'params':      params or {},
        'note':        note,
        'git':         code_provenance(),
        'calibration_sha256': calibration_digest(body_config_path(body_id)),
        'constants_sha256':   constants_digest(),
    }
    path = os.path.join(run_dir, 'run_meta.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, indent=2, sort_keys=True)
        fh.write('\n')
    return path


def read_run_meta(run_dir):
    try:
        with open(os.path.join(run_dir, 'run_meta.json'), encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


if __name__ == '__main__':                      # tiny self-report, not a CLI
    _log_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_BEHAVIOR_DIR, 'log')
    print('log_dir  :', _log_dir)
    print('body_id  :', resolve_body_id(_log_dir))
    print('runs     :', sorted(os.listdir(runs_root(_log_dir)))
          if os.path.isdir(runs_root(_log_dir)) else '(none)')
