#!/usr/bin/env python3
"""Append-only event log for the BT contract guards.

WHY THIS EXISTS
    "Intercepting at write time beats reporting afterwards" is a claim the ablation
    experiment has to evidence, and the two numbers that carry it -- how often a guard
    fired, and how often the very next write came back clean -- exist for about a
    millisecond each. A guard's whole output is a string handed to the agent's context;
    nothing is ever written down. Swap the SD card and the evidence never existed.

    This module is the one place that writes it down. It records; it never decides.

WHY `pass` EVENTS ARE LOGGED TOO
    Fix-on-first-retry needs to distinguish "the agent fixed it" from "the agent gave up
    and edited something else". Without an event for a clean write those two are the
    same absence. So a guard whose path pattern matched and which found nothing logs
    action="pass", and the metric becomes: for each blocked/warned event on a file, look
    at the next event for that same file.

WHY THE LOG LIVES WHERE IT DOES
    /home/pi/.claude/logs/ is picked by elimination, not preference:
      - NOT under .claude/hooks/ -- validate_engine.stage_engine() copytree()s that whole
        directory into the container staging area on every `git push`, so a log there
        would be duplicated without bound.
      - NOT a path that is simultaneously *.jsonl AND contains "/log/" AND contains
        "xyz_behavior/" -- bt_log_read_guard.py classifies that as a BT observability log
        and blocks the Read tool on it. ("/logs/" does not contain the substring "/log/",
        which is what makes this directory safe.)
      - NOT a filename starting bt_debug_ / bt_ros_comm_debug_ -- same guard, by regex.

WHY EVERYTHING IS LAZY
    tools/validate_engine.py genuinely imports xyz_bt_lib_guard and
    xyz_bt_l1_running_guard to reuse their patterns, and its hooks_import_safe sweep
    fails any hook with a bare call statement at module scope. So module scope here holds
    nothing but imports, compiled regexes and constants: no directory creation, no open().
    A module-level side effect would also fire on every pre-push run and inject phantom
    events with no session behind them.

    The AST sweep is syntactic and would not catch `_F = open(path, "a")` at module scope
    -- which is exactly why this file must not contain one.
"""
import json
import os
import re
import time

#: Overridable so a test can point the log somewhere disposable.
_ENV_PATH = 'AINEX_GUARD_LOG'
_DEFAULT_PATH = '/home/pi/.claude/logs/guard_events.jsonl'

#: One O_APPEND write below PIPE_BUF is atomic on Linux, and up to four guard
#: processes write concurrently on a single Write/Edit. Keep every line under this
#: by truncating the message; the rule id carries the identity anyway.
_MAX_LINE = 4000

#: Violation message -> stable rule id, scoped by the hook that produced it.
#:
#: Scoped rather than flat because two guards emit a byte-identical message with
#: different meanings: "Missing class variable LEVEL" is node.level in xyz_bt_lib_guard
#: and behavior.behaviours.level in xyz_behavior_guard. A flat table would silently
#: file every project behaviour violation under the library's rule.
#:
#: Kept here rather than in the guards so that adding an identifier never touches a
#: check function. Those functions still return list[str] and are still called with
#: their existing arities by validate_engine.py -- changing their return shape would
#: ripple into the library sweep.
#:
#: Messages that interpolate variable text (sampled rospy calls, XML parser errors,
#: missing key names) are matched on their fixed prefix only.
_RULES_BY_HOOK = {
    'xyz_bt_tree_pre_guard': (
        (re.compile(r'^Raw py_trees composite'),                'tree.raw_composite'),
        (re.compile(r'^Non-literal state_key'),                 'tree.state_key_nonliteral'),
        (re.compile(r'^f-string state_key'),                    'tree.state_key_fstring'),
    ),
    'xyz_bt_lib_guard': (
        # check_node
        (re.compile(r'L2_Gait_\* node does not inherit'),       'node.gait_base'),
        (re.compile(r'Does not inherit an XyzL1ConditionNode'), 'node.base_class'),
        (re.compile(r'Missing class variable LEVEL'),           'node.level'),
        (re.compile(r'Missing BB_READS'),                       'node.bb_reads'),
        (re.compile(r'setup\(\) does not call super'),          'node.super_setup'),
        (re.compile(r'BT node calls rospy'),                    'node.rospy_call'),
        # check_adapter
        (re.compile(r'Missing snapshot_and_reset'),             'adapter.snapshot_and_reset'),
        (re.compile(r'Missing write_snapshot'),                 'adapter.write_snapshot'),
        (re.compile(r'No rospy\.Subscriber created'),           'adapter.subscriber'),
        (re.compile(r'Missing received_count'),                 'adapter.received_count'),
        (re.compile(r'Missing input_state event'),              'adapter.input_state'),
        # check_bb_keys
        (re.compile(r'ROSA_TOPIC_MAP may be missing'),          'bb_keys.rosa_topic_map_missing'),
    ),
    'xyz_bt_l1_running_guard': (
        (re.compile(r'L1 condition nodes are PURE PREDICATES'), 'l1.status_running'),
    ),
    'xyz_coverage_guard': (
        (re.compile(r'^Unclassified'),                          'coverage.unclassified'),
    ),
    'xyz_behavior_guard': (
        # check_runtime_io
        (re.compile(r'BuzzerState imported from ainex_interfaces'), 'behavior.runtime_io.wrong_import'),
        (re.compile(r'BuzzerState not imported from'),          'behavior.runtime_io.missing_import'),
        # check_runtime_facade
        (re.compile(r'Facade class does not inherit'),          'behavior.facade.base_class'),
        # check_infra_manifest
        (re.compile(r'Missing build_infra_manifest'),           'behavior.infra_manifest.build'),
        (re.compile(r'Missing write_infra_manifest'),           'behavior.infra_manifest.write'),
        # check_bb_ros_bridge
        (re.compile(r'Missing XXXBBBridge class'),              'behavior.bb_bridge.class'),
        (re.compile(r'Missing start\(\) method'),               'behavior.bb_bridge.start'),
        # check_tree_bt / check_groot_xml
        (re.compile(r'Missing bootstrap\(\) function'),         'behavior.tree.bootstrap'),
        (re.compile(r'Missing companion Groot XML'),            'behavior.tree.missing_groot'),
        (re.compile(r'Missing companion tree file'),            'behavior.groot.missing_bt'),
        (re.compile(r'Missing main_tree_to_execute'),           'behavior.groot.main_tree'),
        (re.compile(r'No <BehaviorTree> element'),              'behavior.groot.no_behaviortree'),
        (re.compile(r'Missing <TreeNodesModel>'),               'behavior.groot.no_nodesmodel'),
        # check_app_bt_node / check_script_bt_node
        (re.compile(r'Missing main\(\) entry function'),        'behavior.app.main'),
        (re.compile(r'Missing adapter\.snapshot_and_reset'),    'behavior.app.snapshot_and_reset'),
        (re.compile(r'Missing adapter\.write_snapshot'),        'behavior.app.write_snapshot'),
        (re.compile(r'Missing rospkg\.RosPack'),                'behavior.app.rospack'),
        (re.compile(r'Script entry point does not import main'), 'behavior.script.import_main'),
        (re.compile(r'Script entry point missing rospkg'),      'behavior.script.rospack'),
        (re.compile(r'Script entry point missing sys\.path'),   'behavior.script.syspath'),
        # check_behaviours
        (re.compile(r'Behaviour class does not inherit'),       'behavior.behaviours.base_class'),
        (re.compile(r'Missing class variable LEVEL'),           'behavior.behaviours.level'),
        (re.compile(r'Missing class variable BB_LOG_KEYS'),     'behavior.behaviours.bb_log_keys'),
        (re.compile(r'Direct rospy\.\*\(\) call'),              'behavior.behaviours.rospy_call'),
        # check_launch and check_groot_xml both emit this
        (re.compile(r'Invalid XML'),                            'behavior.invalid_xml'),
    ),
}

_LEADING_MARK = re.compile(r'^[\W_]+')
_NON_SLUG = re.compile(r'[^a-z0-9]+')


def rule_id(message, hook=''):
    """Map a violation message to a stable identifier within `hook`'s namespace.

    Falls back to a deterministic slug of the message's first clause so a rule added
    to a guard without a matching entry above still groups correctly in analysis --
    it just gets an ugly name, which is a far better failure than silently collapsing
    every unknown violation into one bucket.
    """
    text = _LEADING_MARK.sub('', (message or '').strip())
    for pattern, name in _RULES_BY_HOOK.get(hook, ()):
        if pattern.search(text):
            return name
    head = re.split(r'\s{2,}→|→|\n|\.\s', text, 1)[0]
    return 'unmapped.' + _NON_SLUG.sub('_', head.lower()).strip('_')[:60]


def log_path():
    return os.environ.get(_ENV_PATH) or _DEFAULT_PATH


def log_event(hook, action, file_path, rule=None, message='',
              session_id='', event='', tool=''):
    """Append one event. Returns None always, raises never.

    Deliberately swallows every Exception: a guard that crashed while logging would
    either wedge editing or -- far worse for the experiment -- let a violating write
    through because the process died before its exit(2). It does NOT catch BaseException,
    so a guard's own SystemExit still propagates untouched.
    """
    try:
        record = {
            'ts': time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime()) +
                  '.%03dZ' % (int(time.time() * 1000) % 1000),
            'session_id': session_id or '',
            'hook': hook,
            'event': event or '',
            'tool': tool or '',
            'file': file_path or '',
            'action': action,
            'rule': rule if rule is not None else (rule_id(message, hook) if message else None),
            'message': (message or '').replace('\n', ' ').strip(),
        }
        line = json.dumps(record, sort_keys=True)
        if len(line) > _MAX_LINE:
            # Trim the message rather than drop the event: identity lives in `rule`.
            overflow = len(line) - _MAX_LINE
            record['message'] = record['message'][:max(0, len(record['message']) - overflow - 3)] + '...'
            line = json.dumps(record, sort_keys=True)
        path = log_path()
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        try:
            # One write() of one line: concurrent guard processes append without
            # interleaving as long as the line stays under PIPE_BUF.
            os.write(fd, (line + '\n').encode('utf-8'))
        finally:
            os.close(fd)
    except Exception:
        pass


def log_violations(hook, action, file_path, violations, data=None, collapse=False):
    """Log one line per violation, or a single line when `collapse` is set.

    `collapse` exists for xyz_bt_l1_running_guard: check_l1_running returns two strings
    for one logical violation (the diagnosis and the `→` remedy), so counting its list
    length would double-count the only rule it has.
    """
    ctx = _context(data)
    if not violations:
        return
    if collapse:
        log_event(hook, action, file_path, message=violations[0], **ctx)
        return
    for v in violations:
        log_event(hook, action, file_path, message=v, **ctx)


def log_clean(hook, file_path, data=None, rule=None):
    """Log that this guard's pattern matched and the content was clean.

    `rule` is optional and defaults to the historical None. xyz_coverage_guard passes
    the category it resolved ('coverage.build', 'coverage.lib.node', ...) so a pass can
    be attributed: a coverage matrix needs to show "build: claimed, 0 rules", and an
    unlabelled pass is indistinguishable from every other pass.
    """
    log_event(hook, 'pass', file_path, rule=rule, message='', **_context(data))


def log_reminder(hook, file_path, rule, data=None):
    """Log a pre-write convention reminder.

    Distinct from `pass`: the two PreToolUse reminder guards evaluate nothing, so
    recording them as a passed check would claim a check that never ran.
    """
    log_event(hook, 'reminder', file_path, rule=rule, message='', **_context(data))


def _context(data):
    """Pull the correlation fields out of the hook's stdin payload.

    session_id is the join key to the exported transcript, which is named for it.
    No guard read this field before; it costs nothing and collides with nothing.
    """
    data = data or {}
    return {
        'session_id': data.get('session_id', '') or '',
        'event': data.get('hook_event_name', '') or '',
        'tool': data.get('tool_name', '') or '',
    }
