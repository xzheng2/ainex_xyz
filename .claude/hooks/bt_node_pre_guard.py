#!/usr/bin/env python3
"""PreToolUse hook: inject xyz-bt-project skill reminder before writing a BT node file."""
import json
import re
import sys

_PATTERN = re.compile(r'(ainex|xyz)_behavior/.+/behaviours/[^/]+\.py$')

_REMINDER = (
    "【BT节点守护 · 事前提醒】\n"
    "检测到即将写入 ainex_behavior/.../behaviours/*.py 或 xyz_behavior/.../behaviours/*.py BT节点文件。\n"
    "请确保内容符合 AinexBTNode 规范（xyz-bt-project skill）：\n"
    "  1. class Foo(AinexBTNode) — 继承 AinexBTNode 基类\n"
    "  2. LEVEL = 'L1'|'L2'|'L3'  — 类变量声明节点等级\n"
    "  3. BB_LOG_KEYS = [...]       — 类变量声明 Blackboard 可观测键\n"
    "  4. 不直接调用 rospy.*()      — 通过 comm_facade / SemanticFacade 访问 ROS\n"
    "如需规范模板，请先调用 xyz-bt-project skill 再写代码。"
)


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    tool_name = data.get("tool_name", "")
    if tool_name not in ("Write", "Edit"):
        sys.exit(0)

    file_path = data.get("tool_input", {}).get("file_path", "")
    if not _PATTERN.search(file_path):
        sys.exit(0)

    print(json.dumps({"additionalContext": _REMINDER}))
    sys.exit(0)


main()
