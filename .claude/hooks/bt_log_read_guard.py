#!/usr/bin/env python3
"""PreToolUse guard: block raw reads of BT observability logs, redirect to the structured path.

The BT logs (recent + lastrun, both the decision and comm streams) are huge and flood the
context when read raw. This hook forces Claude Code through the bounded diagnosis path:
the `xyz-bt-lastrun-diagnose` skill, `lastrun_digest_cli.py`, ROSA's `cross_tick_analysis` /
`get_bt_tick_raw`, or a bounded tick-level extraction.

Matches (Read tool file_path OR Bash command):
  - basename like bt_debug_*.jsonl or bt_ros_comm_debug_*.jsonl (recent/lastrun, any project)
  - any *.jsonl under a BT log dir (path has '/log/' AND ('xyz_behavior/' or '/opt/ainex_bt_log'))

Allowed even when a match is referenced (Bash only): bounded/CLI tokens
(lastrun_digest_cli.py, session_correlate.py, tick_id, --tick-selection, --tick-range) and pure
metadata commands (wc/ls/stat/du/file, head -c). Small non-.jsonl state files in /log
(bb_current.json) are never matched.

Block = print reason to stderr + exit(2) (PreToolUse deny). Fail-open (exit 0) on any error.
"""
import json
import os
import re
import sys

# Filename patterns for the BT observability JSONL logs.
_BT_LOG_NAME = re.compile(r"bt_debug_.*\.jsonl$|bt_ros_comm_debug_.*\.jsonl$")

# Bounded / CLI tokens that indicate a safe, narrow access (Bash).
_ALLOW_TOKENS = (
    "lastrun_digest_cli.py",
    "session_correlate.py",
    "tick_id",
    "--tick-selection",
    "--tick-range",
)
# Metadata commands that inspect a file without dumping its contents.
_META_CMD = re.compile(r"\b(wc|ls|stat|du|file)\b|\bhead\s+-c\b")

_REDIRECT = (
    "[bt_log_read_guard] Blocked: don't read raw BT logs (recent or lastrun) directly — "
    "they flood context. Use the /xyz-bt-lastrun-diagnose skill, or run "
    "lastrun_digest_cli.py for the bounded digest, then drill into a tick range; for raw "
    "lines use jq 'select(.tick_id>=A and .tick_id<=B)' on the file. For a live session, use "
    "the ROSA cross_tick_analysis / get_bt_tick_raw tools instead of reading the file."
)


def _is_bt_log_path(path: str) -> bool:
    """True if `path` is a raw BT observability log file."""
    if not path:
        return False
    base = os.path.basename(path)
    if _BT_LOG_NAME.search(base):
        return True
    # Any .jsonl inside a BT log directory.
    if path.endswith(".jsonl") and "/log/" in path and (
        "xyz_behavior/" in path or "/opt/ainex_bt_log" in path
    ):
        return True
    return False


def _command_hits_bt_log(command: str) -> bool:
    """True if a Bash command references a raw BT log path/filename."""
    if not command:
        return False
    if _BT_LOG_NAME.search(command):
        return True
    # token-scan for a .jsonl under a BT log dir
    for tok in re.split(r"\s+", command):
        t = tok.strip("'\"")
        if t.endswith(".jsonl") and "/log/" in t and (
            "xyz_behavior/" in t or "/opt/ainex_bt_log" in t
        ):
            return True
    return False


def _block():
    sys.stderr.write(_REDIRECT + "\n")
    sys.exit(2)


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)  # fail-open

    tool = data.get("tool_name", "")
    ti = data.get("tool_input", {}) or {}

    if tool == "Read":
        if _is_bt_log_path(ti.get("file_path", "")):
            _block()
        sys.exit(0)

    if tool == "Bash":
        cmd = ti.get("command", "") or ""
        if not _command_hits_bt_log(cmd):
            sys.exit(0)
        # referenced a BT log — allow only bounded/CLI or metadata uses
        if any(tok in cmd for tok in _ALLOW_TOKENS) or _META_CMD.search(cmd):
            sys.exit(0)
        _block()

    sys.exit(0)


if __name__ == "__main__":
    main()
