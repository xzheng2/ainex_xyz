#!/usr/bin/env python3
"""PostToolUse hook: check node/adapter compliance and blackboard_keys.py ROSA_TOPIC_MAP coverage."""
import json
import re
import sys

_NODE_PATTERN    = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L[123]_\w+/[^/]+\.py$')
_ADAPTER_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/adapters/[^/]+\.py$')
_BB_KEYS_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/blackboard/blackboard_keys\.py$')

_COMMENT_RE = re.compile(r'#[^\n]*')
_STRING_RE  = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')


def _strip_noise(src: str) -> str:
    src = _STRING_RE.sub('""', src)
    src = _COMMENT_RE.sub('', src)
    return src


def check_node(content: str) -> list:
    violations = []
    clean = _strip_noise(content)

    if not re.search(r'class\s+\w+\s*\([^)]*Xyz(?:L1ConditionNode|L2ActionNode|L3ActionNode)[^)]*\)', clean):
        violations.append("❌ Does not inherit XyzL1ConditionNode / XyzL2ActionNode / XyzL3ActionNode  →  class L1_Foo(XyzL1ConditionNode): / L2_Foo(XyzL2ActionNode): / L3_Foo(XyzL3ActionNode):")
    if not re.search(r'^\s{4}LEVEL\s*=', clean, re.MULTILINE):
        violations.append("❌ Missing class variable LEVEL               →  LEVEL = 'L1'")
    if not re.search(r'^\s{4}BB_READS\s*=', clean, re.MULTILINE):
        violations.append("❌ Missing BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS   →  declare all four at class level")
    if not re.search(r'super\(\)\.setup\(', clean):
        violations.append("❌ setup() does not call super().setup()      →  required to initialise XyzBTNode")
    calls = re.findall(r'rospy\.\w+\s*\(', clean)
    if calls:
        samples = ', '.join(sorted(set(calls))[:3])
        violations.append(f"❌ BT node calls rospy.*(): {samples}  →  use self._logger.emit_bt() instead")

    return violations


def check_adapter(content: str) -> list:
    violations = []
    clean = _strip_noise(content)

    if not re.search(r'def snapshot_and_reset\(', clean):
        violations.append("❌ Missing snapshot_and_reset()          →  two-phase lock protocol Phase 1")
    if not re.search(r'def write_snapshot\(', clean):
        violations.append("❌ Missing write_snapshot()              →  two-phase lock protocol Phase 2")
    if not re.search(r'rospy\.Subscriber\(', clean):
        violations.append("❌ No rospy.Subscriber created           →  subscribe to ROS topic in __init__()")
    if not re.search(r'received_count', clean):
        violations.append("❌ Missing received_count counter        →  required by two-phase lock protocol")
    if not re.search(r'"input_state"', clean):
        violations.append('❌ Missing input_state event             →  write_snapshot() must emit "input_state"')

    return violations


def check_bb_keys(content: str) -> list:
    """Warn if any constant defined as LATCHED_NS + '...' is absent from ROSA_TOPIC_MAP.

    Only matches the standard pattern:
        FOO_BAR = LATCHED_NS + '/' + FOO_BAR_KEY
    Raw string literals and comments are intentionally ignored to avoid false positives.
    """
    defined = re.findall(r'^(\w+)\s*=\s*LATCHED_NS\s*\+\s*', content, re.MULTILINE)
    if not defined:
        return []

    map_match = re.search(r'ROSA_TOPIC_MAP\s*=\s*\{(.+?)\}', content, re.DOTALL)
    mapped = set(re.findall(r'\b(\w+)\s*:', map_match.group(1))) if map_match else set()

    missing = [k for k in defined if k not in mapped]
    if not missing:
        return []

    items = '\n'.join(f'  {k}' for k in missing)
    return [
        f"[blackboard_keys.py guard] ROSA_TOPIC_MAP may be missing the following /latched/ key(s):\n{items}\n"
        "Add them to ROSA_TOPIC_MAP in the same edit, or confirm the key does not need ROSA mirroring."
    ]


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    if data.get("tool_name", "") not in ("Write", "Edit"):
        sys.exit(0)

    file_path  = data.get("tool_input", {}).get("file_path", "")
    is_node    = _NODE_PATTERN.search(file_path)
    is_adapter = _ADAPTER_PATTERN.search(file_path)
    is_bb_keys = _BB_KEYS_PATTERN.search(file_path)

    if not is_node and not is_adapter and not is_bb_keys:
        sys.exit(0)

    try:
        with open(file_path, "r", encoding="utf-8") as fh:
            content = fh.read()
    except Exception:
        sys.exit(0)

    if is_node:
        violations = check_node(content)
    elif is_adapter:
        violations = check_adapter(content)
    else:
        violations = check_bb_keys(content)

    if not violations:
        sys.exit(0)

    lines = "\n".join(f"  {v}" for v in violations)
    if is_bb_keys:
        # check_bb_keys returns pre-formatted warning strings
        context = "\n".join(violations)
    else:
        kind = "node" if is_node else "adapter"
        context = (
            f"[xyz_bt_lib {kind} guard · compliance check] {file_path} has {len(violations)} violation(s):\n"
            f"{lines}\n"
            "Please fix the issues above before proceeding — must comply with xyz_bt_lib conventions."
        )
    print(json.dumps({"additionalContext": context}))
    sys.exit(0)


main()
