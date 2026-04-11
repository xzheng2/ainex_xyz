#!/usr/bin/env python3
"""Deterministic raw-tick retrieval from BT observability recent files.

No LLM calls, no summarization.  Pure extraction: select a tick_id from the
rolling recent JSONL files, return every raw line belonging to that tick plus
optional neighbor ticks.

Only recent files are used.  There is NO fallback to lastrun files — one-tick
analysis is always a live/paused-session operation.
"""
import json
import os
import time

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BT_OBS_DIR         = "/opt/ainex_bt_log"
BT_RECENT_BT       = "bt_debug_recent.jsonl"
BT_RECENT_COMM     = "bt_ros_comm_debug_recent.jsonl"
STALE_WARN_SECONDS = 10.0


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _path(filename: str) -> str:
    return os.path.join(BT_OBS_DIR, filename)


def _file_status(filename: str) -> dict:
    """Return existence and age info for a recent file.

    Returns:
        {
            "exists": bool,
            "age_seconds": float | None,   # None if not found
            "stale": bool,                 # True if age > STALE_WARN_SECONDS
        }
    """
    p = _path(filename)
    try:
        mtime = os.path.getmtime(p)
        age = time.time() - mtime
        return {"exists": True, "age_seconds": round(age, 1), "stale": age > STALE_WARN_SECONDS}
    except OSError:
        return {"exists": False, "age_seconds": None, "stale": False}


def _read_jsonl_raw(path: str) -> list:
    """Read every non-empty line from a JSONL file.

    Returns list of entry dicts:
        {"raw_line": str, "parsed": dict|None, "parse_error": str|None}

    If the file is missing, returns a single entry with an "error" key instead
    (so callers can detect the missing-file case without an exception).
    """
    if not os.path.exists(path):
        return [{"error": f"file not found: {path}"}]
    try:
        entries = []
        with open(path, "r") as f:
            for raw in f:
                raw = raw.rstrip("\n")
                if not raw.strip():
                    continue
                try:
                    parsed = json.loads(raw)
                    entries.append({"raw_line": raw, "parsed": parsed, "parse_error": None})
                except json.JSONDecodeError as e:
                    entries.append({"raw_line": raw, "parsed": None, "parse_error": str(e)})
        return entries
    except OSError as e:
        return [{"error": str(e)}]


def _detect_order(entries: list) -> str:
    """Detect whether entries are newest-first or oldest-first by tick_id sequence.

    Looks at the first and last entries with a valid tick_id.

    Returns: "newest-first" | "oldest-first" | "single" | "unknown"
    """
    tick_ids = [
        e["parsed"]["tick_id"]
        for e in entries
        if e.get("parsed") and "tick_id" in e["parsed"]
    ]
    if len(tick_ids) < 2:
        return "single" if len(tick_ids) == 1 else "unknown"
    if tick_ids[0] > tick_ids[-1]:
        return "newest-first"
    if tick_ids[0] < tick_ids[-1]:
        return "oldest-first"
    return "unknown"


def _group_by_tick(entries: list) -> dict:
    """Group entries by tick_id.

    Entries without a tick_id (e.g. corrupted lines) go under key -1.

    Returns:
        dict mapping tick_id (int) → list of entry dicts
    """
    groups: dict = {}
    for e in entries:
        if e.get("error"):
            # file-level error sentinel — skip grouping
            continue
        parsed = e.get("parsed")
        tick_id = parsed.get("tick_id") if parsed else None
        if tick_id is None:
            tick_id = -1
        groups.setdefault(tick_id, []).append(e)
    return groups


def _resolve_tick_id(tick_id_param, grouped: dict) -> int:
    """Resolve the user-supplied tick_id parameter to an integer tick_id.

    "latest" → max available tick_id (ignoring -1 sentinel).
    Any other value → int(tick_id_param).

    Raises:
        ValueError: if tick_id_param cannot be converted or grouped is empty.
        KeyError:   if the resolved tick_id is not in grouped.
    """
    valid_ids = [k for k in grouped if k >= 0]
    if not valid_ids:
        raise ValueError("no valid tick_ids found in file")

    if str(tick_id_param).strip().lower() == "latest":
        return max(valid_ids)

    try:
        requested = int(tick_id_param)
    except (TypeError, ValueError):
        raise ValueError(f"tick_id must be 'latest' or an integer, got: {tick_id_param!r}")

    if requested not in grouped:
        raise KeyError(
            f"tick_id {requested} not in file. Available: {sorted(valid_ids)}"
        )
    return requested


def _select_neighbors(grouped: dict, selected_id: int, include_neighbors: int) -> dict:
    """Return neighbor tick entries within ±include_neighbors of selected_id.

    Returns dict mapping neighbor tick_id → list of entries.
    The selected_id itself is excluded.
    """
    if include_neighbors <= 0:
        return {}
    neighbors = {}
    for offset in range(-include_neighbors, include_neighbors + 1):
        if offset == 0:
            continue
        nid = selected_id + offset
        if nid in grouped:
            neighbors[nid] = grouped[nid]
    return neighbors


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def get_raw_tick_bundle(tick_id="latest", include_neighbors: int = 1) -> dict:
    """Extract raw log entries for one BT tick from the recent JSONL files.

    Args:
        tick_id:           "latest" (default) or an integer tick_id string/int.
        include_neighbors: how many ticks before/after to include as context (default 1).

    Returns:
        RawTickBundle dict — see module docstring for full schema.
        Always returns a dict; never raises.  Check ["ok"] first.
    """
    warnings: list = []

    # ── File status ─────────────────────────────────────────────────────────
    bt_status   = _file_status(BT_RECENT_BT)
    comm_status = _file_status(BT_RECENT_COMM)

    stale_warning = None
    if not bt_status["exists"]:
        return {
            "ok": False,
            "error": (
                f"bt_debug_recent.jsonl not found at {_path(BT_RECENT_BT)}. "
                "One-tick analysis requires recent files — ensure the BT node has run "
                "at least once in this session. No lastrun fallback for one-tick analysis."
            ),
            "selected_tick_id": None,
            "source_mode": "recent",
            "files": {"bt": BT_RECENT_BT, "comm": BT_RECENT_COMM},
            "recent_status": {
                "bt_age_seconds":   bt_status["age_seconds"],
                "comm_age_seconds": comm_status["age_seconds"],
                "stale_warning":    None,
            },
            "order_detected": {"bt": "unknown", "comm": "unknown"},
            "all_tick_ids": [],
            "selected_tick": {"bt_entries": [], "comm_entries": []},
            "neighbors": {},
            "warnings": [],
        }

    if bt_status["stale"]:
        stale_warning = (
            f"bt file stale ({bt_status['age_seconds']}s since last write) — "
            "BT node may be paused or stopped. Data reflects the last recorded ticks."
        )
        warnings.append(stale_warning)

    if comm_status["exists"] and comm_status["stale"]:
        warnings.append(
            f"comm file stale ({comm_status['age_seconds']}s since last write)."
        )

    # ── Read raw entries ─────────────────────────────────────────────────────
    bt_entries   = _read_jsonl_raw(_path(BT_RECENT_BT))
    comm_entries = _read_jsonl_raw(_path(BT_RECENT_COMM)) if comm_status["exists"] else []

    # Check for file-level errors
    if bt_entries and bt_entries[0].get("error"):
        return {
            "ok": False,
            "error": bt_entries[0]["error"],
            "selected_tick_id": None,
            "source_mode": "recent",
            "files": {"bt": BT_RECENT_BT, "comm": BT_RECENT_COMM},
            "recent_status": {
                "bt_age_seconds":   bt_status["age_seconds"],
                "comm_age_seconds": comm_status["age_seconds"],
                "stale_warning":    stale_warning,
            },
            "order_detected": {"bt": "unknown", "comm": "unknown"},
            "all_tick_ids": [],
            "selected_tick": {"bt_entries": [], "comm_entries": []},
            "neighbors": {},
            "warnings": warnings,
        }

    # ── Detect order per file ────────────────────────────────────────────────
    bt_order   = _detect_order(bt_entries)
    comm_order = _detect_order(comm_entries) if comm_entries else "unknown"

    # ── Group by tick ────────────────────────────────────────────────────────
    bt_grouped   = _group_by_tick(bt_entries)
    comm_grouped = _group_by_tick(comm_entries)

    # ── Resolve tick_id ──────────────────────────────────────────────────────
    try:
        selected_id = _resolve_tick_id(tick_id, bt_grouped)
    except (ValueError, KeyError) as e:
        valid_ids = sorted(k for k in bt_grouped if k >= 0)
        return {
            "ok": False,
            "error": str(e),
            "selected_tick_id": None,
            "source_mode": "recent",
            "files": {"bt": BT_RECENT_BT, "comm": BT_RECENT_COMM},
            "recent_status": {
                "bt_age_seconds":   bt_status["age_seconds"],
                "comm_age_seconds": comm_status["age_seconds"],
                "stale_warning":    stale_warning,
            },
            "order_detected": {"bt": bt_order, "comm": comm_order},
            "all_tick_ids": valid_ids,
            "selected_tick": {"bt_entries": [], "comm_entries": []},
            "neighbors": {},
            "warnings": warnings,
        }

    # ── Extract selected tick entries ────────────────────────────────────────
    selected_bt   = bt_grouped.get(selected_id, [])
    selected_comm = comm_grouped.get(selected_id, [])

    if not selected_bt:
        warnings.append(
            f"tick_id {selected_id} has no BT entries — "
            "tick may have been trimmed from the rolling window."
        )

    # ── Neighbors ───────────────────────────────────────────────────────────
    neighbor_ids = _select_neighbors(bt_grouped, selected_id, include_neighbors)
    neighbors = {}
    for nid, nbt in neighbor_ids.items():
        neighbors[nid] = {
            "bt_entries":   nbt,
            "comm_entries": comm_grouped.get(nid, []),
        }

    all_tick_ids = sorted(k for k in bt_grouped if k >= 0)

    return {
        "ok": True,
        "error": None,
        "selected_tick_id": selected_id,
        "source_mode": "recent",
        "files": {"bt": BT_RECENT_BT, "comm": BT_RECENT_COMM},
        "recent_status": {
            "bt_age_seconds":   bt_status["age_seconds"],
            "comm_age_seconds": comm_status["age_seconds"],
            "stale_warning":    stale_warning,
        },
        "order_detected": {"bt": bt_order, "comm": comm_order},
        "all_tick_ids": all_tick_ids,
        "selected_tick": {
            "bt_entries":   selected_bt,
            "comm_entries": selected_comm,
        },
        "neighbors": neighbors,
        "warnings": warnings,
    }
