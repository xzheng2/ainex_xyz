#!/usr/bin/env python3
"""PreToolUse hook: inject ainex-bt-edu-extend skill reminder before writing to ainex_bt_edu source."""
import json
import re
import sys

_NODE_PATTERN    = re.compile(r'ainex_bt_edu/src/ainex_bt_edu/behaviours/L[12]_\w+/[^/]+\.py$')
_ADAPTER_PATTERN = re.compile(r'ainex_bt_edu/src/ainex_bt_edu/input_adapters/[^/]+\.py$')

_NODE_REMINDER = (
    "【ainex_bt_edu 节点守护 · 事前提醒】\n"
    "检测到即将写入 ainex_bt_edu behaviours/L*/... BT节点文件。\n"
    "请确保内容符合 AinexBTNode 规范（ainex-bt-edu-extend skill）：\n"
    "  1. class Foo(AinexBTNode)   — 继承 AinexBTNode 基类\n"
    "  2. LEVEL = 'L1'|'L2'        — 类变量声明节点等级\n"
    "  3. BB_LOG_KEYS = [BB.*]      — 用 BB.* 常量列表\n"
    "  4. setup() 调用 super().setup(**kwargs)\n"
    "  5. 不 import rospy，不调用 rospy.*()  — 用 self._logger.emit_bt() 替代\n"
    "如需模板，请先调用 ainex-bt-edu-extend skill 再写代码。"
)

_ADAPTER_REMINDER = (
    "【ainex_bt_edu 适配器守护 · 事前提醒】\n"
    "检测到即将写入 ainex_bt_edu input_adapters/... 适配器文件。\n"
    "请确保内容符合两阶段锁协议（ainex-bt-edu-extend skill）：\n"
    "  1. snapshot_and_reset()          — Phase 1，在 with lock: 块内调用，返回快照并重置 received_count\n"
    "  2. write_snapshot(snap, tick_id) — Phase 2，释放锁后调用，写 BB + 发 ros_in/input_state 事件\n"
    "  3. rospy.Subscriber 只在 __init__() 中创建\n"
    "  4. BB 初始写在 __init__() 中（首 tick 前确保键存在，BT 节点不会 KeyError）\n"
    "如需模板，请先调用 ainex-bt-edu-extend skill 再写代码。"
)


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    if data.get("tool_name", "") not in ("Write", "Edit"):
        sys.exit(0)

    file_path = data.get("tool_input", {}).get("file_path", "")

    if _NODE_PATTERN.search(file_path):
        print(json.dumps({"additionalContext": _NODE_REMINDER}))
    elif _ADAPTER_PATTERN.search(file_path):
        print(json.dumps({"additionalContext": _ADAPTER_REMINDER}))

    sys.exit(0)


main()
