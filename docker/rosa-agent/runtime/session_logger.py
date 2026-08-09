"""
session_logger.py — Lightweight session transcript logger for ROSA-Ainex agent.

Records user queries and final assistant responses to a JSONL file.
When the agent runs with return_intermediate_steps=True, each turn record
also carries the tool-selection/execution trace (tool, tool_input, observation)
for debugging — observations are truncated to keep the JSONL manageable.

Log directory: <repo>/docker/rosa-agent/logs  (container: /opt/rosa-agent/logs)
File naming:   session_<UTC timestamp>_<mode>.jsonl
               e.g. session_20260515T143012Z_cli.jsonl
"""

import json
import sys
import warnings
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

_DEFAULT_LOGS_DIR = Path(__file__).parent.parent / "logs"


class SessionLogger:
    """
    Minimal session transcript logger.

    Usage:
        logger = SessionLogger(mode="cli")
        logger.start()
        logger.log_turn("user query", "assistant response")
        logger.end()
    """

    def __init__(self, mode: str, logs_dir: Optional[Path] = None):
        self._mode = mode
        self._logs_dir = logs_dir or _DEFAULT_LOGS_DIR
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._session_id = f"session_{ts}_{mode}"
        self._log_path = self._logs_dir / f"{self._session_id}.jsonl"
        self._turn_index = 0
        self._fh = None  # file handle, opened in start()

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Create the log file and write the session_start record."""
        try:
            self._logs_dir.mkdir(parents=True, exist_ok=True)
            self._fh = open(self._log_path, "a", encoding="utf-8")
            self._write({
                "event": "session_start",
                "session_id": self._session_id,
                "started_at": _utcnow(),
                "mode": self._mode,
            })
        except Exception as exc:
            warnings.warn(
                f"[session_logger] Failed to start session log: {exc}",
                stacklevel=2,
            )
            self._fh = None

    def log_turn(
        self,
        user_query: str,
        assistant_response: str,
        status: str = "ok",
        error: Optional[str] = None,
        intermediate_steps: Optional[list] = None,
    ) -> None:
        """Append a turn record to the session log."""
        self._turn_index += 1
        record = {
            "event": "turn",
            "session_id": self._session_id,
            "turn_index": self._turn_index,
            "timestamp": _utcnow(),
            "user_query": user_query,
            "assistant_response": assistant_response,
            "status": status,
        }
        if error is not None:
            record["error"] = error
        if intermediate_steps:
            record["intermediate_steps"] = _serialize_steps(intermediate_steps)
        self._write(record)

    def end(self) -> None:
        """Write the session_end record and close the log file."""
        self._write({
            "event": "session_end",
            "session_id": self._session_id,
            "ended_at": _utcnow(),
            "turn_count": self._turn_index,
        })
        try:
            if self._fh is not None:
                self._fh.close()
                self._fh = None
        except Exception as exc:
            warnings.warn(
                f"[session_logger] Failed to close session log: {exc}",
                stacklevel=2,
            )

    # ── Internal ──────────────────────────────────────────────────────────────

    def _write(self, record: dict) -> None:
        """Write a single JSON record as one line. Silently warns on failure."""
        if self._fh is None:
            return
        try:
            self._fh.write(json.dumps(record, ensure_ascii=False) + "\n")
            self._fh.flush()
        except Exception as exc:
            warnings.warn(
                f"[session_logger] Failed to write log record: {exc}",
                stacklevel=2,
            )


def _utcnow() -> str:
    return datetime.now(timezone.utc).isoformat()


_MAX_OBSERVATION_CHARS = 2000


def _serialize_steps(steps: list) -> list:
    """
    Convert LangChain intermediate steps — a list of (AgentAction, observation)
    tuples — into JSON-safe dicts. Never raises: a malformed step is recorded
    as its repr() so the turn record is still written.
    """
    out = []
    for step in steps:
        try:
            action, observation = step
            obs = str(observation)
            if len(obs) > _MAX_OBSERVATION_CHARS:
                obs = obs[:_MAX_OBSERVATION_CHARS] + f"... [truncated, {len(obs)} chars total]"
            out.append({
                "tool": getattr(action, "tool", None),
                "tool_input": _json_safe(getattr(action, "tool_input", None)),
                "observation": obs,
            })
        except Exception:
            out.append({"unparsed_step": repr(step)[:_MAX_OBSERVATION_CHARS]})
    return out


def _json_safe(value):
    try:
        json.dumps(value)
        return value
    except (TypeError, ValueError):
        return str(value)
