#!/usr/bin/env python3
"""PreToolUse hook: inject this repo's git conventions when a git command is actually run.

WHY A HOOK AND NOT CLAUDE.md
    CLAUDE.md is loaded unconditionally every session, so it only earns space for rules
    that must shape *design discussions* — the monorepo rationale, trunk-based as the
    default. Everything here is operational: it matters only in the moment a git command
    is issued, and would otherwise be a permanent context tax on sessions that never
    touch git.

WHY NOT A SKILL
    Skills are pull-based — they load only if the agent chooses to invoke one. A rule that
    must not be routed around cannot be optional to read.

WHAT IT BLOCKS
    `git push --no-verify` skips .githooks/pre-push, i.e. pushes code that nothing has
    checked. That escape hatch belongs to the human, who can always run it directly; it
    should not be something the agent reaches for when the gate is inconvenient.
    Everything else is advisory context, never a block.

Exit 2 = deny (PreToolUse). Exit 0 = allow. Fail-open on any error.
"""
import json
import re
import sys

# Heredoc bodies are DATA, not commands. Without stripping them, a commit message that
# merely mentions `git push --no-verify` — such as the one that introduced this hook —
# gets read as an attempt to run it. The sibling guards strip strings and comments before
# matching for the same reason; a heredoc just needs its own pass.
_HEREDOC = re.compile(r"<<-?\s*(['\"]?)(\w+)\1.*?^\2$", re.S | re.M)

# Only match at a command boundary: start of input, or after a newline / ; / & / |,
# allowing leading VAR=value assignments. This keeps `echo "don't use git push"` and any
# other quoted prose from counting as an invocation.
_CMD_START = r"(?:^|[\n;&|])\s*(?:[A-Za-z_][A-Za-z0-9_]*=\S*\s+)*"
_GIT_CMD = re.compile(_CMD_START + r"git\s+(?:push|commit|tag)\b", re.M)
_NO_VERIFY = re.compile(_CMD_START + r"git\s+push\b[^\n;&|]*--no-verify\b", re.M)

_DENY = (
    "[git_workflow_guard] Blocked: `git push --no-verify` skips the pre-push gate, which "
    "means pushing code that validate_engine.py has never checked.\n"
    "→ Fix what the gate reported instead. If the gate itself is wrong, fix the gate.\n"
    "→ If this genuinely must be forced, that is the user's call: ask them to run it "
    "themselves with `!git push --no-verify`."
)

_CONTEXT = (
    "[git_workflow_guard] This repo's git conventions:\n"
    "  • Trunk-based: commit straight to `master`. No Git Flow, no release branches. A "
    "short-lived branch is for multi-session risky work, or for freezing master while an "
    "experiment campaign runs — delete it after merging.\n"
    "  • `git push` runs .githooks/pre-push → validate_engine.py (~4 s). If it rejects, "
    "fix the failure; do not reach for --no-verify.\n"
    "  • Experiment baselines are ANNOTATED TAGS, not branches: "
    "`git tag -a abl-<YYYYMMDD>-<campaign> -m '...'`. run_meta.json records "
    "`git.describe`, so a tag makes an ablation table readable where a bare sha does not. "
    "Ablation variants that are only parameter differences need no tag and no branch — "
    "they are already in run_meta.params.\n"
    "  • package.xml version numbers are decorative: nothing reads them, there is no CI "
    "and no release process. Do not 'fix' them or start bumping them.\n"
    "  • Before committing, stage everything that belongs in the repo: "
    "`git status --short -- docker/ros_ws_src/ docker/rosa-agent/`"
)


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:                                  # noqa: BLE001
        sys.exit(0)                                    # fail-open

    if data.get("tool_name") != "Bash":
        sys.exit(0)

    command = (data.get("tool_input", {}) or {}).get("command", "") or ""
    command = _HEREDOC.sub("<<HEREDOC", command)

    if _NO_VERIFY.search(command):
        sys.stderr.write(_DENY + "\n")
        sys.exit(2)

    if not _GIT_CMD.search(command):
        sys.exit(0)

    print(json.dumps({"additionalContext": _CONTEXT}))
    sys.exit(0)


if __name__ == '__main__':
    main()
