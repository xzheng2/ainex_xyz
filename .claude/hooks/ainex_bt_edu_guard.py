#!/usr/bin/env python3
"""PostToolUse hook: check AinexBTNode / adapter compliance after writing to ainex_bt_edu source."""
import json
import re
import sys

_NODE_PATTERN    = re.compile(r'ainex_bt_edu/src/ainex_bt_edu/behaviours/L[12]_\w+/[^/]+\.py$')
_ADAPTER_PATTERN = re.compile(r'ainex_bt_edu/src/ainex_bt_edu/input_adapters/[^/]+\.py$')

_COMMENT_RE = re.compile(r'#[^\n]*')
_STRING_RE  = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')


def _strip_noise(src: str) -> str:
    src = _STRING_RE.sub('""', src)
    src = _COMMENT_RE.sub('', src)
    return src


def check_node(content: str) -> list:
    violations = []
    clean = _strip_noise(content)

    if not re.search(r'class\s+\w+\s*\([^)]*AinexBTNode[^)]*\)', clean):
        violations.append("❌ 未继承 AinexBTNode 基类      →  class Foo(AinexBTNode):")
    if not re.search(r'^\s{4}LEVEL\s*=', clean, re.MULTILINE):
        violations.append("❌ 缺少类变量 LEVEL             →  LEVEL = 'L1'")
    if not re.search(r'^\s{4}BB_LOG_KEYS\s*=', clean, re.MULTILINE):
        violations.append("❌ 缺少类变量 BB_LOG_KEYS       →  BB_LOG_KEYS = [BB.SOME_KEY]")
    if not re.search(r'super\(\)\.setup\(', clean):
        violations.append("❌ setup() 未调用 super().setup()  →  必须调用以初始化 AinexBTNode")
    calls = re.findall(r'rospy\.\w+\s*\(', clean)
    if calls:
        samples = ', '.join(sorted(set(calls))[:3])
        violations.append(f"❌ BT节点调用 rospy.*()：{samples}  →  改用 self._logger.emit_bt()")

    return violations


def check_adapter(content: str) -> list:
    violations = []
    clean = _strip_noise(content)

    if not re.search(r'def snapshot_and_reset\(', clean):
        violations.append("❌ 缺少 snapshot_and_reset()          →  Phase 1 两阶段锁协议")
    if not re.search(r'def write_snapshot\(', clean):
        violations.append("❌ 缺少 write_snapshot()              →  Phase 2 两阶段锁协议")
    if not re.search(r'rospy\.Subscriber\(', clean):
        violations.append("❌ 未创建 rospy.Subscriber            →  在 __init__() 中订阅 ROS topic")
    if not re.search(r'received_count', clean):
        violations.append("❌ 缺少 received_count 计数器         →  两阶段锁协议需要此字段")
    if not re.search(r'"input_state"', clean):
        violations.append('❌ 缺少 input_state 事件              →  write_snapshot() 必须发 "input_state"')

    return violations


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    if data.get("tool_name", "") not in ("Write", "Edit"):
        sys.exit(0)

    file_path = data.get("tool_input", {}).get("file_path", "")
    is_node    = _NODE_PATTERN.search(file_path)
    is_adapter = _ADAPTER_PATTERN.search(file_path)

    if not is_node and not is_adapter:
        sys.exit(0)

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    violations = check_node(content) if is_node else check_adapter(content)
    if not violations:
        sys.exit(0)

    lines = "\n".join(f"  {v}" for v in violations)
    kind  = "BT节点" if is_node else "适配器"
    context = (
        f"【ainex_bt_edu {kind}守护 · 合规检查】{file_path} 存在 {len(violations)} 项违规：\n"
        f"{lines}\n"
        "请立即修正上述问题，确保符合 ainex_bt_edu 规范后再继续。"
    )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


main()
