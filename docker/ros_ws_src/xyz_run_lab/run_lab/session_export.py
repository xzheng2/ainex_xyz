#!/usr/bin/env python3
"""Distil Claude Code session transcripts into publishable process data (stdlib only).

WHY THIS MODULE EXISTS
    The ablation compares how a coding agent works under four asset mixes. Four of the
    metrics -- token consumption, tool-call count, rework, clarification questions --
    exist only inside the CLI's own session transcripts under
    ``~/.claude/projects/<project>/``. Those files are not in any repository and are
    scoped to one SD card: swap the card and the data is gone, unrecoverably. This
    module is what gets it off the card before that happens.

WHY DISTILLED AND NOT COPIED
    Two independent reasons, either sufficient on its own.

    Size: a single session measured 16.8 MB raw and 1.7 MB distilled -- 72 KB once
    gzipped. Four lanes x three projects of raw transcripts would dwarf the results
    they are meant to annotate.

    Disclosure: the results repository is PUBLIC, and a raw transcript contains the
    full text of every file Read, the complete stdout of every Bash command, and every
    prompt as typed. On this machine that tree also holds an OpenAI key in
    settings.json and a GitHub PAT in ~/.git-credentials. The distilled form carries no
    free text at all -- the only strings that survive are tool names and file paths --
    so it is safe by construction rather than by review. Nothing here should ever be
    changed to keep message text without revisiting that.

WHY `message.id` DEDUPLICATION IS NOT OPTIONAL
    The CLI writes one JSONL line per content block, and every line of one API response
    repeats that response's ``usage`` object byte for byte. One measured session held
    2661 assistant lines for 1263 distinct ``message.id``; summing per line inflated
    output tokens by 2.56x. Token totals are therefore computed over deduplicated
    message ids. Tool-call counts are NOT deduplicated -- those genuinely are per block.

WHY SUBAGENT FILES ARE READ TOO
    Subagent work is not inlined. It lives in
    ``<project>/<session_id>/subagents/agent-<agentId>.jsonl``, and in the main
    transcript the spawning ``Agent`` call returns only ``{"status":"async_launched"}``.
    A session that delegated heavily would report a small fraction of its true cost if
    only the main file were read.
"""
import glob
import json
import os
import re

SCHEMA = 'ainex.session_metrics/1'

DEFAULT_PROJECTS_DIR = '/home/pi/.claude/projects'
DEFAULT_SESSIONS_DIR = '/home/pi/.claude/sessions'

#: Only these two line types carry conversation. The rest (mode, permission-mode,
#: ai-title, agent-name, last-prompt, attachment, file-history-*, queue-operation,
#: system) are UI and bookkeeping records; several lack uuid/timestamp entirely.
_MESSAGE_TYPES = ('assistant', 'user')

#: Tools whose repeated use on one file within one human turn is the rework signal.
_EDIT_TOOLS = ('Edit', 'Write', 'NotebookEdit')

_AGENT_FILE = re.compile(r'agent-(.+)\.jsonl$')


# --------------------------------------------------------------------------- #
# Locating sessions
# --------------------------------------------------------------------------- #

def project_dir_name(cwd):
    """The CLI names a project directory after its cwd with separators replaced."""
    return cwd.replace('/', '-')


def live_session_ids(sessions_dir=DEFAULT_SESSIONS_DIR):
    """Session ids of currently running CLI processes, newest first.

    Authoritative, unlike file mtime: a bulk operation can touch several transcripts
    in the same second, and a resumed session spans CLI versions.
    """
    found = []
    for path in glob.glob(os.path.join(sessions_dir, '*.json')):
        try:
            with open(path, encoding='utf-8') as fh:
                info = json.load(fh)
        except (OSError, ValueError):
            continue
        if info.get('sessionId'):
            found.append((info.get('startedAt') or 0, info['sessionId'], info))
    found.sort(reverse=True)
    return [(sid, info) for _, sid, info in found]


def session_ids_in(project_path):
    """Every session id with a transcript in this project directory."""
    ids = []
    for path in sorted(glob.glob(os.path.join(project_path, '*.jsonl'))):
        ids.append(os.path.basename(path)[:-len('.jsonl')])
    return ids


def transcript_paths(project_path, session_id):
    """(main transcript, [subagent transcripts]) for one session.

    The main file may legitimately be absent while subagent files exist, and vice
    versa; both halves are optional so a partially written session still exports.
    """
    main = os.path.join(project_path, session_id + '.jsonl')
    subs = sorted(glob.glob(os.path.join(
        project_path, session_id, 'subagents', 'agent-*.jsonl')))
    return (main if os.path.isfile(main) else None), subs


def subagent_meta(path):
    """Read agent-<id>.meta.json beside a subagent transcript.

    ``toolUseId`` links the subagent back to the exact tool_use block in the parent
    that spawned it, which is the only reliable join between the two files.
    """
    meta_path = path[:-len('.jsonl')] + '.meta.json'
    try:
        with open(meta_path, encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


# --------------------------------------------------------------------------- #
# Distillation
# --------------------------------------------------------------------------- #

def _distil_block(block):
    """Reduce one content block to its countable shape. Returns None to drop it.

    Everything textual is dropped here, deliberately -- see the module docstring.
    ``n_questions`` survives because a single AskUserQuestion call can bundle several
    questions and the metric counts questions, not calls.
    """
    if not isinstance(block, dict):
        return None
    kind = block.get('type')
    if kind == 'tool_use':
        out = {'type': 'tool_use', 'name': block.get('name')}
        args = block.get('input')
        if isinstance(args, dict):
            if args.get('file_path'):
                out['fp'] = args['file_path']
            questions = args.get('questions')
            if isinstance(questions, list):
                out['nq'] = len(questions)
        return out
    if kind == 'tool_result':
        return {'type': 'tool_result', 'err': bool(block.get('is_error'))}
    if kind in ('text', 'thinking'):
        # Counted, never carried.
        return {'type': kind}
    return {'type': kind}


def distil_line(obj, agent_id=None):
    """Reduce one transcript line, or return None if it carries no conversation."""
    if not isinstance(obj, dict) or obj.get('type') not in _MESSAGE_TYPES:
        return None

    message = obj.get('message') or {}
    content = message.get('content')
    if isinstance(content, str):
        # A real human prompt: the string is the prompt text, which we do not keep.
        blocks = [{'type': 'text'}]
    elif isinstance(content, list):
        blocks = [b for b in (_distil_block(b) for b in content) if b]
    else:
        blocks = []

    out = {
        't':  obj.get('timestamp'),
        'ty': obj.get('type'),
        'c':  blocks,
    }
    for src, dst in (('uuid', 'uuid'), ('parentUuid', 'pu'), ('promptId', 'pid'),
                     ('requestId', 'rid')):
        if obj.get(src):
            out[dst] = obj[src]
    if message.get('id'):
        out['mid'] = message['id']
    if message.get('model'):
        out['model'] = message['model']
    usage = message.get('usage')
    if isinstance(usage, dict):
        out['u'] = {
            'in': usage.get('input_tokens') or 0,
            'out': usage.get('output_tokens') or 0,
            'cc': usage.get('cache_creation_input_tokens') or 0,
            'cr': usage.get('cache_read_input_tokens') or 0,
        }
    if agent_id:
        out['agent'] = agent_id
    return out


def _iter_jsonl(path):
    """Yield parsed objects, skipping unparseable lines.

    Tolerant on purpose: a live session's last line can be half-written, and
    ~/.claude/history.jsonl is known to contain at least one bare scalar.
    """
    try:
        with open(path, encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except ValueError:
                    continue
    except OSError:
        return


def distil_session(project_path, session_id):
    """Return (lines, info) for one session, main transcript plus every subagent."""
    main, subs = transcript_paths(project_path, session_id)
    lines = []
    info = {
        'session_id': session_id,
        'main_transcript': bool(main),
        'subagents': [],
        'versions': [],
        'slug': None,
        'cwd': None,
    }

    versions = []
    if main:
        for obj in _iter_jsonl(main):
            if obj.get('version') and obj['version'] not in versions:
                # A resumed session spans CLI upgrades; record all of them.
                versions.append(obj['version'])
            if info['slug'] is None and obj.get('slug'):
                info['slug'] = obj['slug']
            if info['cwd'] is None and obj.get('cwd'):
                info['cwd'] = obj['cwd']
            row = distil_line(obj)
            if row:
                lines.append(row)

    for sub in subs:
        agent_id = _AGENT_FILE.search(os.path.basename(sub))
        agent_id = agent_id.group(1) if agent_id else os.path.basename(sub)
        meta = subagent_meta(sub)
        count = 0
        for obj in _iter_jsonl(sub):
            row = distil_line(obj, agent_id=agent_id)
            if row:
                lines.append(row)
                count += 1
        info['subagents'].append({
            'agent_id': agent_id,
            'agent_type': meta.get('agentType'),
            'description': meta.get('description'),
            'tool_use_id': meta.get('toolUseId'),
            'spawn_depth': meta.get('spawnDepth'),
            'lines': count,
        })

    info['versions'] = versions
    return lines, info


# --------------------------------------------------------------------------- #
# Reduction
# --------------------------------------------------------------------------- #

def _empty_metrics():
    return {
        'schema': SCHEMA,
        'lines': 0,
        'api_responses': 0,
        'tokens': {'input': 0, 'output': 0, 'cache_creation': 0, 'cache_read': 0,
                   'total': 0},
        'tokens_subagent': {'input': 0, 'output': 0, 'cache_creation': 0,
                            'cache_read': 0, 'total': 0},
        'tool_calls_total': 0,
        'tool_calls_by_name': {},
        'file_path_calls': 0,
        'files_touched': 0,
        'rework_events': 0,
        'rework_files': 0,
        'clarification_calls': 0,
        'clarification_questions': 0,
        'human_turns': 0,
        'subagent_count': 0,
        'subagent_types': {},
        'tool_errors': 0,
        'duration_s': None,
        'first_ts': None,
        'last_ts': None,
    }


def reduce_session(lines, info=None):
    """Turn distilled lines into the metric set the ablation table needs.

    Never raises on malformed input: a session that cannot be measured should report
    zeros, not abort a publish run.
    """
    out = _empty_metrics()
    info = info or {}
    out['lines'] = len(lines)

    seen_ids = set()
    seen_ids_sub = set()
    by_name = {}
    edits = {}                     # (promptId, file_path) -> count
    files = set()
    turns = set()
    stamps = []

    for row in lines:
        is_sub = bool(row.get('agent'))
        if row.get('t'):
            stamps.append(row['t'])

        # Tokens: one API response is many lines, all repeating one usage object.
        usage, mid = row.get('u'), row.get('mid')
        if usage and mid:
            pool = seen_ids_sub if is_sub else seen_ids
            if mid not in pool:
                pool.add(mid)
                bucket = out['tokens_subagent'] if is_sub else out['tokens']
                bucket['input'] += usage.get('in', 0)
                bucket['output'] += usage.get('out', 0)
                bucket['cache_creation'] += usage.get('cc', 0)
                bucket['cache_read'] += usage.get('cr', 0)

        if row.get('ty') == 'user' and row.get('pid'):
            turns.add(row['pid'])

        for block in row.get('c') or []:
            kind = block.get('type')
            if kind == 'tool_result' and block.get('err'):
                out['tool_errors'] += 1
            if kind != 'tool_use':
                continue
            name = block.get('name') or '?'
            out['tool_calls_total'] += 1
            by_name[name] = by_name.get(name, 0) + 1
            path = block.get('fp')
            if path:
                out['file_path_calls'] += 1
                files.add(path)
                if name in _EDIT_TOOLS:
                    # Keyed by human turn: repeatedly editing one file inside a single
                    # turn is rework; edits spread across turns are ordinary progress.
                    key = (row.get('pid') or row.get('agent') or '', path)
                    edits[key] = edits.get(key, 0) + 1
            if name == 'AskUserQuestion':
                out['clarification_calls'] += 1
                out['clarification_questions'] += block.get('nq') or 1

    for bucket in (out['tokens'], out['tokens_subagent']):
        bucket['total'] = (bucket['input'] + bucket['output'] +
                           bucket['cache_creation'] + bucket['cache_read'])

    repeats = [n for n in edits.values() if n > 1]
    out['api_responses'] = len(seen_ids) + len(seen_ids_sub)
    out['tool_calls_by_name'] = dict(sorted(by_name.items()))
    out['files_touched'] = len(files)
    out['rework_events'] = sum(n - 1 for n in repeats)
    out['rework_files'] = len(repeats)
    out['human_turns'] = len(turns)

    subs = info.get('subagents') or []
    out['subagent_count'] = len(subs)
    types = {}
    for sub in subs:
        key = sub.get('agent_type') or '?'
        types[key] = types.get(key, 0) + 1
    out['subagent_types'] = dict(sorted(types.items()))

    if stamps:
        stamps.sort()
        out['first_ts'], out['last_ts'] = stamps[0], stamps[-1]
        out['duration_s'] = _span_seconds(stamps[0], stamps[-1])
    return out


def _span_seconds(first, last):
    import calendar
    import time as _time

    def parse(stamp):
        try:
            return calendar.timegm(_time.strptime(stamp[:19], '%Y-%m-%dT%H:%M:%S'))
        except (ValueError, TypeError):
            return None

    a, b = parse(first), parse(last)
    return None if a is None or b is None else round(b - a, 1)


# --------------------------------------------------------------------------- #
# Guard events
# --------------------------------------------------------------------------- #

DEFAULT_GUARD_LOG = '/home/pi/.claude/logs/guard_events.jsonl'


def guard_events(session_id, path=None):
    """This session's slice of the guard event log.

    Absent or empty is a RESULT, not an error: a lane without the skills+hooks engine
    has no guards to fire, and that zero is the control group's data point.
    """
    path = path or os.environ.get('AINEX_GUARD_LOG') or DEFAULT_GUARD_LOG
    return [e for e in _iter_jsonl(path) if e.get('session_id') == session_id]


def reduce_guard_events(events):
    """Counts per action/rule, plus fix-on-first-retry over the file timeline."""
    by_action, by_rule = {}, {}
    for e in events:
        a = e.get('action') or '?'
        by_action[a] = by_action.get(a, 0) + 1
        if e.get('rule'):
            by_rule[e['rule']] = by_rule.get(e['rule'], 0) + 1

    # For each blocked/warned event on a file, did the NEXT event on that same file
    # come back clean? `pass` events exist precisely so this is answerable.
    per_file = {}
    for e in events:
        per_file.setdefault(e.get('file') or '', []).append(e)

    fixed = not_fixed = dangling = 0
    for timeline in per_file.values():
        for i, e in enumerate(timeline):
            if e.get('action') not in ('blocked', 'warned'):
                continue
            nxt = next((n for n in timeline[i + 1:]
                        if n.get('action') in ('blocked', 'warned', 'pass')), None)
            if nxt is None:
                dangling += 1
            elif nxt.get('action') == 'pass':
                fixed += 1
            else:
                not_fixed += 1

    decided = fixed + not_fixed
    return {
        'events': len(events),
        'by_action': dict(sorted(by_action.items())),
        'by_rule': dict(sorted(by_rule.items())),
        'fixed_next_write': fixed,
        'not_fixed_next_write': not_fixed,
        'no_following_write': dangling,
        'fix_on_first_retry': round(fixed / decided, 4) if decided else None,
    }
