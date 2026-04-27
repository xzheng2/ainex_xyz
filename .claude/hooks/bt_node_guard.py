#!/usr/bin/env python3
"""PostToolUse hook: check AinexBTNode compliance after writing a BT node file."""
import json
import re
import sys

_PATTERN = re.compile(r'(ainex|xyz)_behavior/.+/behaviours/[^/]+\.py$')

# Strip single-line comments and string literals for cleaner analysis
_COMMENT_RE = re.compile(r'#[^\n]*')
_STRING_RE = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')


def _strip_noise(src: str) -> str:
    """Remove string literals and comments to avoid false positives."""
    src = _STRING_RE.sub('""', src)
    src = _COMMENT_RE.sub('', src)
    return src


def check_compliance(content: str) -> list[str]:
    violations: list[str] = []
    clean = _strip_noise(content)

    # 1. Must inherit AinexBTNode
    if not re.search(r'class\s+\w+\s*\([^)]*AinexBTNode[^)]*\)', clean):
        violations.append("❌ 未继承 AinexBTNode 基类  →  class Foo(AinexBTNode):")

    # 2. Class-level LEVEL assignment
    if not re.search(r'^\s{4}LEVEL\s*=', clean, re.MULTILINE):
        violations.append("❌ 缺少类变量 LEVEL        →  LEVEL = 'L2'")

    # 3. Class-level BB_LOG_KEYS assignment
    if not re.search(r'^\s{4}BB_LOG_KEYS\s*=', clean, re.MULTILINE):
        violations.append("❌ 缺少类变量 BB_LOG_KEYS  →  BB_LOG_KEYS = ['key1']")

    # 4. No direct rospy API calls (allow bare `import rospy`)
    calls = re.findall(r'rospy\.\w+\s*\(', clean)
    if calls:
        samples = ', '.join(sorted(set(calls))[:3])
        violations.append(f"❌ 直接调用 rospy.*()：{samples}  →  改用 comm_facade / SemanticFacade")

    return violations


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

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    violations = check_compliance(content)
    if not violations:
        sys.exit(0)

    lines = "\n".join(f"  {v}" for v in violations)
    context = (
        f"【BT节点守护 · 合规检查】{file_path} 存在 {len(violations)} 项违规：\n"
        f"{lines}\n"
        "请立即修正上述问题，确保符合 AinexBTNode 规范后再继续。"
    )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


main()
