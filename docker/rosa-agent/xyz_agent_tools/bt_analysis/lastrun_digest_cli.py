#!/usr/bin/env python3
"""Host-side CLI for whole-run (lastrun) BT diagnosis.

Thin stdlib-only wrapper around the deterministic reducer
(bt_analysis.raw_cross_tick.get_raw_cross_tick_bundle) and the langchain-free
formatter (bt_analysis.cross_tick_format). Designed to run under HOST python3
against a project log dir, so the Claude Code skill `xyz-bt-lastrun-diagnose`
can analyze a completed run without the rosa-agent container or langchain.

Two-step forensic workflow:
  1. Session digest first (bounded for any run length):
       lastrun_digest_cli.py --log-dir <proj>/log --detail digest
     → state occupancy profile + transition-type histogram. Locate the suspect span.
  2. Drill into that span (verbose per-tick scaffold):
       lastrun_digest_cli.py --log-dir <proj>/log --detail full --tick-selection 556-561

The same reducer also serves the ROSA `session_digest` tool in-container; this CLI
is just the host entry point onto it. No fallback magic — you pass the log dir.
"""
import argparse
import json
import os
import sys

# Make `bt_analysis` importable as a top-level package WITHOUT importing
# xyz_agent_tools/__init__.py (which eagerly imports langchain-based tools).
# This file is xyz_agent_tools/bt_analysis/lastrun_digest_cli.py, so the parent of
# bt_analysis/ is the xyz_agent_tools dir — put it on sys.path.
_PKG_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from bt_analysis.raw_cross_tick import get_raw_cross_tick_bundle  # noqa: E402
from bt_analysis.cross_tick_format import format_digest, format_scaffold  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="lastrun_digest_cli",
        description="Whole-run / windowed BT diagnosis from observability JSONL logs.",
    )
    ap.add_argument(
        "--log-dir", required=True,
        help="Directory holding the JSONL logs (e.g. xyz_behavior/<proj>/log).",
    )
    ap.add_argument(
        "--source", choices=["lastrun", "recent"], default="lastrun",
        help="Which file tier to read (default: lastrun = full completed run).",
    )
    ap.add_argument(
        "--detail", choices=["digest", "full"], default="digest",
        help="digest = coarse whole-run overview (default); full = verbose per-tick scaffold "
             "(use with a bounded --tick-selection).",
    )
    ap.add_argument(
        "--tick-selection", default="all",
        help='Ticks to analyze: "all" | "latest:N" | "A-B" | "A,B,C" (default: all).',
    )
    ap.add_argument(
        "--out", choices=["text", "json"], default="text",
        help="text = human/LLM-readable scaffold (default); json = raw bundle dict.",
    )
    ap.add_argument(
        "--top-n", type=int, default=15,
        help="digest: number of top execution states to show in the occupancy profile (default 15).",
    )
    ap.add_argument(
        "--user-observation", default="",
        help="Optional physical/UI observation to compare against the log evidence.",
    )
    args = ap.parse_args(argv)

    if not os.path.isdir(args.log_dir):
        print(f"error: --log-dir is not a directory: {args.log_dir}", file=sys.stderr)
        return 2

    bundle = get_raw_cross_tick_bundle(
        tick_selection=args.tick_selection,
        source=args.source,
        log_dir=args.log_dir,
        detail=args.detail,
    )

    if not bundle.get("ok"):
        # Still emit the bundle on --out json so callers can inspect the error context.
        if args.out == "json":
            print(json.dumps(bundle, default=str))
        else:
            print(f"lastrun_digest_cli error: {bundle.get('error')}", file=sys.stderr)
        return 1

    if args.out == "json":
        print(json.dumps(bundle, default=str))
        return 0

    if args.detail == "digest":
        print(format_digest(bundle, args.user_observation, top_n=args.top_n))
    else:
        print(format_scaffold(bundle, args.user_observation))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
