#!/usr/bin/env python3
"""
Summarize ROS logs from ~/.ros/log/latest/ into an LLM-friendly Markdown report.

Reads all .log files in the latest ROS log session directory, deduplicates
repetitive events (ROSA polling, camera restarts), and outputs a condensed
report targeting <20K tokens (~80KB text).

Usage:
    python3.9 summarize_ros_logs.py [--log-dir /path/to/log/latest]
                                     [--expected-nodes marathon_bringup]
                                     [--config /path/to/expected_nodes.yaml]

Default log dir: most recently modified session dir under /root/.ros/log/
                 (found by mtime — no reliance on the 'latest' symlink)
"""

import os
import re
import sys
import glob
import argparse
from collections import Counter, defaultdict, OrderedDict
from datetime import datetime

_LOG_BASE = "/root/.ros/log"
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "config", "expected_nodes.yaml")


def _find_latest_session_dir(log_base=_LOG_BASE):
    """Return the most recently modified ROS session directory under log_base."""
    if not os.path.isdir(log_base):
        return ""
    dirs = [
        os.path.join(log_base, d)
        for d in os.listdir(log_base)
        if d != "latest" and os.path.isdir(os.path.join(log_base, d))
    ]
    if not dirs:
        return ""
    return max(dirs, key=os.path.getmtime) + "/"


# ---------------------------------------------------------------------------
# master.log parser
# ---------------------------------------------------------------------------

# Pattern: [rosmaster.master][INFO] 2026-03-13 21:15:48,802: +SUB [/topic] /node uri
MASTER_EVENT_RE = re.compile(
    r'\[rosmaster\.\w+\]\[(\w+)\]\s+'
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),\d+:\s+'
    r'([+-](?:SUB|PUB|SERVICE|PARAM))\s+'
    r'\[([^\]]+)\]\s+'
    r'(/\S+)\s+'
    r'(.*)')

# Pattern for master startup line
MASTER_START_RE = re.compile(
    r'\[rosmaster\.main\]\[INFO\]\s+'
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),\d+:\s+'
    r'initialization complete')

# publisherUpdate lines (skip)
PUBLISHER_UPDATE_RE = re.compile(r'publisherUpdate')


def parse_master_log(filepath):
    """Parse master.log into unique events and repetitive event counts."""
    events = []          # (timestamp, action, topic, node)
    churn = Counter()    # (action_pair, topic, node_prefix) -> count
    churn_first = {}     # same key -> first timestamp
    churn_last = {}      # same key -> last timestamp
    session_start = None
    session_end = None

    # Track +SUB/-SUB pairs for dedup
    seen_events = set()  # (action, topic, node) for dedup of unique events
    all_nodes = set()
    all_topics_pub = set()   # topics with active +PUB
    all_topics_sub = set()   # topics with active +SUB
    all_services = set()

    with open(filepath, 'r', errors='replace') as f:
        for line in f:
            # Check for session start
            m = MASTER_START_RE.search(line)
            if m:
                session_start = m.group(1)
                continue

            # Skip publisherUpdate noise
            if PUBLISHER_UPDATE_RE.search(line):
                continue

            m = MASTER_EVENT_RE.match(line)
            if not m:
                continue

            _level, timestamp, action, topic, node, _uri = m.groups()
            session_end = timestamp

            # Extract node prefix (without unique suffixes like _1773450958591)
            node_clean = node.rstrip('/')
            # Detect ROSA polling nodes (contain timestamp-like suffix)
            is_rosa = 'rosa' in node_clean.lower()

            all_nodes.add(node_clean)

            # Track graph state
            if action == '+PUB':
                all_topics_pub.add((topic, node_clean))
            elif action == '-PUB':
                all_topics_pub.discard((topic, node_clean))
            elif action == '+SUB':
                all_topics_sub.add((topic, node_clean))
            elif action == '-SUB':
                all_topics_sub.discard((topic, node_clean))
            elif action == '+SERVICE':
                all_services.add((topic, node_clean))
            elif action == '-SERVICE':
                all_services.discard((topic, node_clean))

            # Dedup: ROSA polling creates rapid +SUB/-SUB pairs
            if is_rosa and action in ('+SUB', '-SUB'):
                key = ('ROSA_poll', topic)
                churn[key] += 1
                if key not in churn_first:
                    churn_first[key] = timestamp
                churn_last[key] = timestamp
                continue

            # Dedup: camera restart cycles (+PUB/-PUB on camera topics)
            if 'camera' in node_clean.lower() and action in ('+PUB', '-PUB'):
                key = ('camera_restart', topic)
                churn[key] += 1
                if key not in churn_first:
                    churn_first[key] = timestamp
                churn_last[key] = timestamp
                # Still record first occurrence as unique event
                event_key = (action, topic, node_clean)
                if event_key not in seen_events:
                    seen_events.add(event_key)
                    events.append((timestamp, action, topic, node_clean))
                continue

            # Unique event
            event_key = (action, topic, node_clean)
            if event_key not in seen_events:
                seen_events.add(event_key)
                events.append((timestamp, action, topic, node_clean))

    return {
        'session_start': session_start,
        'session_end': session_end,
        'events': events,
        'churn': churn,
        'churn_first': churn_first,
        'churn_last': churn_last,
        'all_nodes': all_nodes,
        'active_pubs': all_topics_pub,
        'active_subs': all_topics_sub,
        'active_services': all_services,
    }


# ---------------------------------------------------------------------------
# rosout.log parser
# ---------------------------------------------------------------------------

# Pattern: epoch LEVEL /node [source] [topics: ...] message
ROSOUT_RE = re.compile(
    r'(\d+\.\d+)\s+'
    r'(\w+)\s+'
    r'(/\S+)\s+'
    r'\[([^\]]*)\]\s+'
    r'\[([^\]]*)\]\s+'
    r'(.*)', re.DOTALL)


def parse_rosout_log(filepath):
    """Parse rosout.log, group by (level, node, message_template)."""
    groups = OrderedDict()  # (level, node, template) -> {count, first_ts, last_ts, sample}

    with open(filepath, 'r', errors='replace') as f:
        content = f.read()

    # Split on lines that start with an epoch timestamp
    entries = re.split(r'\n(?=\d+\.\d+\s+)', content)

    for entry in entries:
        entry = entry.strip()
        if not entry:
            continue
        m = ROSOUT_RE.match(entry)
        if not m:
            continue

        epoch, level, node, source, topics, message = m.groups()
        # Clean ANSI color codes from message
        message = re.sub(r'\x1b\[[0-9;]*m', '', message).strip()

        # Create template by replacing numbers with N
        template = re.sub(r'\d+\.\d+', 'N.N', message)
        template = re.sub(r'\b\d{3,}\b', 'N', template)  # long numbers only

        try:
            ts = datetime.fromtimestamp(float(epoch))
            ts_str = ts.strftime('%Y-%m-%d %H:%M:%S')
        except (ValueError, OSError):
            ts_str = epoch

        key = (level, node, template)
        if key not in groups:
            groups[key] = {
                'count': 0,
                'first_ts': ts_str,
                'last_ts': ts_str,
                'sample': message[:200],
            }
        groups[key]['count'] += 1
        groups[key]['last_ts'] = ts_str

    return groups


# ---------------------------------------------------------------------------
# roslaunch log parser
# ---------------------------------------------------------------------------

LAUNCH_NODE_RE = re.compile(
    r'\[roslaunch\]\[INFO\].*?Added node of type \[([^\]]+)\] in namespace \[([^\]]+)\]')
LAUNCH_FILE_RE = re.compile(
    r'loading config file\s+(\S+)')
LAUNCH_PROCESS_RE = re.compile(
    r'\[roslaunch\]\[INFO\].*?process\[([^\]]+)\].*?started with pid \[(\d+)\]')
LAUNCH_DEATH_RE = re.compile(
    r'process_died.*?name\[([^\]]+)\].*?exit code\[([^\]]*)\]')


def parse_roslaunch_log(filepath):
    """Extract launch file, nodes, PIDs, and failures from roslaunch log."""
    launch_files = []
    nodes_added = []
    nodes_started = []
    deaths = []

    with open(filepath, 'r', errors='replace') as f:
        for line in f:
            m = LAUNCH_FILE_RE.search(line)
            if m:
                launch_files.append(m.group(1))
                continue

            m = LAUNCH_NODE_RE.match(line)
            if m:
                nodes_added.append((m.group(1), m.group(2)))
                continue

            m = LAUNCH_PROCESS_RE.match(line)
            if m:
                nodes_started.append((m.group(1), m.group(2)))

            m = LAUNCH_DEATH_RE.search(line)
            if m:
                deaths.append((m.group(1), m.group(2)))

    return {
        'launch_files': launch_files,
        'nodes_added': nodes_added,
        'nodes_started': nodes_started,
        'deaths': deaths,
    }


# ---------------------------------------------------------------------------
# Per-node log parser (small files, include verbatim or dedup)
# ---------------------------------------------------------------------------

def parse_node_log(filepath, max_verbatim=50):
    """Read a per-node log file. Return (lines, is_truncated)."""
    with open(filepath, 'r', errors='replace') as f:
        lines = f.readlines()

    if len(lines) <= max_verbatim:
        return lines, False

    # For large files, keep first 20 and last 20 lines
    truncated = lines[:20] + [f'\n... ({len(lines) - 40} lines omitted) ...\n\n'] + lines[-20:]
    return truncated, True


# ---------------------------------------------------------------------------
# Expected nodes checker
# ---------------------------------------------------------------------------

def load_expected_nodes(config_path, launch_name):
    """Load expected node list from YAML config."""
    if not os.path.exists(config_path):
        return []
    try:
        import yaml
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        return data.get(launch_name, [])
    except ImportError:
        # Fallback: simple YAML parsing without PyYAML
        nodes = []
        in_section = False
        with open(config_path, 'r') as f:
            for line in f:
                line = line.rstrip()
                if line.startswith('#') or not line.strip():
                    continue
                if not line.startswith(' ') and not line.startswith('-'):
                    in_section = line.startswith(f'{launch_name}:')
                    continue
                if in_section and line.strip().startswith('- '):
                    nodes.append(line.strip()[2:].strip())
        return nodes


def check_missing_nodes(expected, found_nodes):
    """Return list of expected nodes not found in the log."""
    found_clean = set()
    for n in found_nodes:
        # Strip leading / and any unique suffixes
        clean = n.lstrip('/')
        found_clean.add(clean)
        # Also add without namespace prefix
        if '/' in clean:
            found_clean.add(clean.split('/')[-1])

    missing = []
    for exp in expected:
        # Check if expected node appears in any found node
        if not any(exp in f for f in found_clean):
            missing.append(exp)
    return missing


# ---------------------------------------------------------------------------
# Report generator
# ---------------------------------------------------------------------------

def generate_report(log_dir, launch_name=None, config_path=None):
    """Generate the full Markdown summary report."""
    sections = []

    # Discover log files
    log_files = sorted(glob.glob(os.path.join(log_dir, '*.log')))
    if not log_files:
        return f"No log files found in {log_dir}"

    # Categorize files
    master_log = None
    rosout_log = None
    roslaunch_logs = []
    node_logs = []

    for f in log_files:
        basename = os.path.basename(f)
        if basename == 'master.log':
            master_log = f
        elif basename == 'rosout.log':
            rosout_log = f
        elif basename.startswith('roslaunch-'):
            roslaunch_logs.append(f)
        elif os.path.getsize(f) > 0:  # skip empty stdout logs
            node_logs.append(f)

    # --- Parse master.log ---
    master_data = None
    if master_log:
        master_data = parse_master_log(master_log)

    # --- Parse roslaunch logs ---
    launch_data_list = []
    detected_launch = None
    for lf in roslaunch_logs:
        ld = parse_roslaunch_log(lf)
        launch_data_list.append(ld)
        for lfile in ld['launch_files']:
            if 'bringup' in lfile or 'launch' in lfile:
                detected_launch = os.path.basename(lfile).replace('.launch', '')

    # Auto-detect launch name for expected nodes
    if not launch_name and detected_launch:
        launch_name = detected_launch

    # --- Header ---
    start = master_data['session_start'] if master_data else 'unknown'
    end = master_data['session_end'] if master_data else 'unknown'
    sections.append(f"# ROS Log Summary — Session {start} to {end}\n")

    # --- Critical Issues ---
    issues = []

    # Check for missing expected nodes
    if launch_name and config_path:
        expected = load_expected_nodes(config_path, launch_name)
        if expected and master_data:
            missing = check_missing_nodes(expected, master_data['all_nodes'])
            for node in missing:
                issues.append(
                    f"**MISSING NODE**: `{node}` expected by `{launch_name}` "
                    f"but never appeared in the ROS graph")

    # Check rosout for errors
    rosout_data = None
    if rosout_log:
        rosout_data = parse_rosout_log(rosout_log)
        for (level, node, template), info in rosout_data.items():
            if level in ('ERROR', 'FATAL'):
                issues.append(
                    f"**{level}** `{node}`: {info['sample'][:120]} "
                    f"(x{info['count']}, {info['first_ts']} — {info['last_ts']})")

    # Check for process deaths
    for ld in launch_data_list:
        for name, exit_code in ld['deaths']:
            issues.append(f"**PROCESS DIED**: `{name}` exit code {exit_code}")

    sections.append("## Critical Issues\n")
    if issues:
        for i, issue in enumerate(issues, 1):
            sections.append(f"{i}. {issue}")
    else:
        sections.append("No critical issues detected.")
    sections.append("")

    # --- Launch Info ---
    sections.append("## Launch Configuration\n")
    for ld in launch_data_list:
        if ld['launch_files']:
            sections.append(f"- Launch file: `{ld['launch_files'][0]}`")
        if ld['nodes_added']:
            sections.append(f"- Nodes configured: {len(ld['nodes_added'])}")
            for ntype, ns in ld['nodes_added']:
                sections.append(f"  - `{ntype}` in `{ns}`")
    sections.append("")

    # --- Node Lifecycle Timeline ---
    if master_data and master_data['events']:
        sections.append("## Node Lifecycle Timeline\n")
        sections.append("| Time | Event | Topic/Service | Node |")
        sections.append("|------|-------|--------------|------|")

        # Show first 80 unique events (should cover all meaningful ones)
        for timestamp, action, topic, node in master_data['events'][:80]:
            time_short = timestamp.split(' ')[1] if ' ' in timestamp else timestamp
            sections.append(f"| {time_short} | {action} | `{topic}` | `{node}` |")

        remaining = len(master_data['events']) - 80
        if remaining > 0:
            sections.append(f"\n*... and {remaining} more unique events*")
        sections.append("")

    # --- Repetitive Event Counts ---
    if master_data and master_data['churn']:
        sections.append("## Repetitive Event Counts (deduplicated)\n")
        sections.append("| Pattern | Count | First | Last |")
        sections.append("|---------|-------|-------|------|")

        for key, count in master_data['churn'].most_common():
            category, topic = key
            first = master_data['churn_first'].get(key, '?')
            last = master_data['churn_last'].get(key, '?')
            first_short = first.split(' ')[1] if ' ' in first else first
            last_short = last.split(' ')[1] if ' ' in last else last

            if category == 'ROSA_poll':
                label = f"ROSA +SUB/-SUB `{topic}`"
            elif category == 'camera_restart':
                label = f"Camera +PUB/-PUB `{topic}`"
            else:
                label = f"{category} `{topic}`"

            sections.append(f"| {label} | {count} | {first_short} | {last_short} |")
        sections.append("")

    # --- Rosout Messages (deduplicated) ---
    if rosout_data:
        sections.append("## Log Messages (rosout, deduplicated)\n")
        sections.append("| Level | Node | Message | Count | First | Last |")
        sections.append("|-------|------|---------|-------|-------|------|")

        # Sort: FATAL > ERROR > WARN > INFO > DEBUG
        level_order = {'FATAL': 0, 'ERROR': 1, 'WARN': 2, 'INFO': 3, 'DEBUG': 4}
        sorted_groups = sorted(rosout_data.items(),
                               key=lambda x: (level_order.get(x[0][0], 5), x[0][1]))

        for (level, node, template), info in sorted_groups:
            msg_short = info['sample'][:100].replace('|', '\\|').replace('\n', ' ')
            sections.append(
                f"| {level} | `{node}` | {msg_short} | {info['count']} | "
                f"{info['first_ts'].split(' ')[1] if ' ' in info['first_ts'] else info['first_ts']} | "
                f"{info['last_ts'].split(' ')[1] if ' ' in info['last_ts'] else info['last_ts']} |")
        sections.append("")

    # --- Per-Node Logs ---
    if node_logs:
        sections.append("## Per-Node Logs\n")
        for nlog in node_logs:
            basename = os.path.basename(nlog)
            # Skip rosout-1-stdout.log (usually just "started core service")
            if 'stdout' in basename:
                continue
            lines, truncated = parse_node_log(nlog)
            if not lines:
                continue
            sections.append(f"### {basename} ({len(lines)} lines"
                            + (", truncated" if truncated else "") + ")\n")
            sections.append("```")
            for line in lines:
                sections.append(line.rstrip())
            sections.append("```\n")

    # --- Active Topics/Services at Session End ---
    if master_data:
        if master_data['active_pubs']:
            sections.append("## Active Publishers at Session End\n")
            sections.append("| Topic | Node |")
            sections.append("|-------|------|")
            for topic, node in sorted(master_data['active_pubs']):
                sections.append(f"| `{topic}` | `{node}` |")
            sections.append("")

        if master_data['active_services']:
            sections.append("## Active Services at Session End\n")
            sections.append("| Service | Node |")
            sections.append("|---------|------|")
            for svc, node in sorted(master_data['active_services']):
                sections.append(f"| `{svc}` | `{node}` |")
            sections.append("")

    return "\n".join(sections)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Summarize ROS logs into LLM-friendly Markdown report')
    parser.add_argument('--log-dir', default=None,
                        help='Path to ROS log session directory '
                             '(default: most recent under ' + _LOG_BASE + ')')
    parser.add_argument('--expected-nodes', default=None,
                        help='Launch name to check expected nodes against '
                             '(e.g. marathon_bringup). Auto-detected if not set.')
    parser.add_argument('--config', default=DEFAULT_CONFIG,
                        help='Path to expected_nodes.yaml (default: %(default)s)')
    args = parser.parse_args()

    if args.log_dir is None:
        args.log_dir = _find_latest_session_dir()
        if not args.log_dir:
            print(f"Error: no ROS session directories found under {_LOG_BASE}",
                  file=sys.stderr)
            sys.exit(1)
    elif not os.path.isdir(args.log_dir):
        print(f"Error: log directory not found: {args.log_dir}", file=sys.stderr)
        sys.exit(1)

    report = generate_report(args.log_dir, args.expected_nodes, args.config)
    print(report)


if __name__ == '__main__':
    main()
