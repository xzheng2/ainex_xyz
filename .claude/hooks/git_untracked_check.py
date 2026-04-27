#!/usr/bin/env python3
"""PostToolUse hook: after git commit, warn about untracked files in ros_ws_src and rosa_agent."""
import json
import subprocess
import sys


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    if data.get("tool_name") != "Bash":
        sys.exit(0)

    command = data.get("tool_input", {}).get("command", "")
    if "git commit" not in command:
        sys.exit(0)

    try:
        result = subprocess.run(
            ["git", "-C", "/home/pi", "status", "--short", "--",
             "docker/ros_ws_src/", "docker/rosa_agent/"],
            capture_output=True, text=True, timeout=5
        )
        untracked = [l for l in result.stdout.splitlines() if l.startswith("??")]
    except Exception:
        sys.exit(0)

    if not untracked:
        sys.exit(0)

    files = "\n".join(f"  {l}" for l in untracked)
    context = (
        f"⚠️  Commit done but {len(untracked)} untracked file(s) remain "
        f"in docker/ros_ws_src/ or docker/rosa_agent/:\n"
        f"{files}\n"
        "→ Stage and commit these too if they belong in the repo."
    )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


main()
