#!/usr/bin/env python3
"""
Claude Code PreToolUse Hook
- Calls OpenAI to explain the action in plain language (Bash, Write, Edit)
- Displays explanation in a desktop popup (zenity); Claude never sees it
- Always exits 0 so Claude proceeds normally without interruption

Toggle on/off:
  In ~/.claude/settings.json, under "env":
    "COMMAND_EXPLAINER": "1"   ← on  (default)
    "COMMAND_EXPLAINER": "0"   ← off
  Restart Claude Code after changing.
"""

import sys
import json
import urllib.request
import urllib.error
import os
import textwrap
import subprocess

# ─── Config ────────────────────────────────────────────────
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "sk-your-key-here")
MODEL = "gpt-4o-mini"
_PREVIEW_LEN = 200
# Set COMMAND_EXPLAINER=0 (or false/off/no) in settings.json env to disable
ENABLED = os.environ.get("COMMAND_EXPLAINER", "1").strip().lower() not in ("0", "false", "off", "no")
# ───────────────────────────────────────────────────────────

SYSTEM_PROMPT = """\
You are a plain-language assistant that helps non-technical users understand \
what an AI assistant is about to do on their computer.

You will be given a description of an action (running a terminal command, \
creating a file, or editing a file). Respond with exactly ONE sentence in \
simple, everyday English that a person with no computer background can \
instantly understand.

Rules:
- No technical jargon (no "directory", "process", "argument", "string", etc.)
- Use concrete everyday words ("folder" not "directory", "save" not "write", \
"update" not "edit")
- Mention the real-world effect and any risk of data loss if relevant
- Never start with "This command" or "This action" — start with the action verb directly
- Output the sentence only, no punctuation beyond the sentence itself
"""


def build_prompt(tool_name: str, tool_input: dict) -> str | None:
    if tool_name == "Bash":
        command = tool_input.get("command", "").strip()
        if not command:
            return None
        return f"Tool: Bash\nCommand: {command}"

    if tool_name == "Write":
        path = tool_input.get("file_path", "").strip()
        content = tool_input.get("content", "")
        if not path:
            return None
        preview = content[:_PREVIEW_LEN].replace("\n", "↵")
        ellipsis = "…" if len(content) > _PREVIEW_LEN else ""
        return (
            f"Tool: Write (create or overwrite a file)\n"
            f"File: {os.path.basename(path)}\n"
            f"Content preview: {preview}{ellipsis}"
        )

    if tool_name == "Edit":
        path = tool_input.get("file_path", "").strip()
        old = tool_input.get("old_string", "")
        new = tool_input.get("new_string", "")
        if not path:
            return None
        old_p = old[:_PREVIEW_LEN].replace("\n", "↵")
        new_p = new[:_PREVIEW_LEN].replace("\n", "↵")
        old_e = "…" if len(old) > _PREVIEW_LEN else ""
        new_e = "…" if len(new) > _PREVIEW_LEN else ""
        return (
            f"Tool: Edit (change part of a file)\n"
            f"File: {os.path.basename(path)}\n"
            f"Replacing: {old_p}{old_e}\n"
            f"With: {new_p}{new_e}"
        )

    return None


def ask_openai(prompt: str) -> str:
    payload = {
        "model": MODEL,
        "max_tokens": 80,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",   "content": prompt},
        ],
    }

    req = urllib.request.Request(
        "https://api.openai.com/v1/chat/completions",
        data=json.dumps(payload).encode(),
        headers={
            "Content-Type":  "application/json",
            "Authorization": f"Bearer {OPENAI_API_KEY}",
        },
        method="POST",
    )

    with urllib.request.urlopen(req, timeout=10) as resp:
        data = json.load(resp)
        return data["choices"][0]["message"]["content"].strip()


def show_to_user(text: str, header: str = "") -> None:
    """
    Show explanation in a non-blocking desktop popup.
    Spawned with start_new_session=True so the dialog outlives the hook
    process and never touches the Claude Code TTY or TUI.
    Falls back to stderr if no display is available.
    """
    label = f"{header}\n\n💬 explain hook:\n\n{text}" if header else f"💬 explain hook:\n\n{text}"

    # Try zenity (GTK dialog, auto-closes after 10 s)
    try:
        subprocess.Popen(
            ["zenity", "--info", "--title=Claude Action",
             f"--text={label}", "--timeout=20", "--width=350"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        return
    except Exception:
        pass

    # Fallback: xmessage (plain X11 dialog, auto-closes after 10 s)
    try:
        subprocess.Popen(
            ["xmessage", "-timeout", "20", "-center", label],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        return
    except Exception:
        pass

    # Last resort: stderr (no display available)
    prefix = "💬 explain hook: "
    indent = "                 "
    wrapped = textwrap.fill(text, width=60,
                            initial_indent=prefix, subsequent_indent=indent)
    print(f"\n{wrapped}\n", file=sys.stderr)


def main():
    if not ENABLED:
        sys.exit(0)

    try:
        hook_input = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    # Only explain in interactive modes; stay silent when accept-edits is on
    mode = hook_input.get("permission_mode", "default")
    if mode not in ("default", "plan"):
        sys.exit(0)

    tool_name  = hook_input.get("tool_name", "")
    tool_input = hook_input.get("tool_input", {})

    prompt = build_prompt(tool_name, tool_input)
    if not prompt:
        sys.exit(0)

    try:
        explanation = ask_openai(prompt)
        header = "\n".join(prompt.splitlines()[:2])
        show_to_user(explanation, header)
    except Exception:
        pass  # always fail silently — never block Claude on API errors

    sys.exit(0)  # Claude proceeds normally, unaware this hook ran


if __name__ == "__main__":
    main()
