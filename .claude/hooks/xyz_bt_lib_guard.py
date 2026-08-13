#!/usr/bin/env python3
"""PostToolUse hook: check node/adapter compliance and blackboard_keys.py ROSA_TOPIC_MAP coverage."""
import json
import re
import sys

_HOOK = 'xyz_bt_lib_guard'

_NODE_PATTERN    = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/behaviours/L[123]_\w+/(?!__init__\.py$)[^/]+\.py$')
_ADAPTER_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/adapters/(?!__init__\.py$)[^/]+\.py$')
_BB_KEYS_PATTERN = re.compile(r'xyz_bt_lib/src/xyz_bt_lib/blackboard/blackboard_keys\.py$')

_COMMENT_RE = re.compile(r'#[^\n]*')
_STRING_RE  = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')


def _strip_noise(src: str) -> str:
    src = _STRING_RE.sub('""', src)
    src = _COMMENT_RE.sub('', src)
    return src


def check_node(content: str, file_path: str = '') -> list:
    violations = []
    clean = _strip_noise(content)

    # Naming IS the contract: a file called L2_Gait_*.py must inherit the gait
    # base, so the rule is checkable from the filename alone with no ambiguity
    # about whether the node "really" dispatches a step. Nodes that only stop the
    # gait (L2_Motion_StopGait, L2_Motion_PauseAfterTicks) deliberately live
    # outside the L2_Gait_ namespace. Three nodes drifted from this rule in Aug
    # 2026 because it existed only in prose — hence this check.
    if re.search(r'/L2_Gait_[^/]*\.py$', file_path):
        if not re.search(r'class\s+\w+\s*\([^)]*XyzL2GaitActionNode[^)]*\)', clean):
            violations.append(
                "❌ L2_Gait_* node does not inherit XyzL2GaitActionNode  →  the "
                "L2_Gait_ prefix means 'dispatches gait steps'; inherit "
                "XyzL2GaitActionNode for the shared pass-through knobs + gait_kwargs(). "
                "If it never dispatches a step, rename it out of the L2_Gait_ namespace "
                "(e.g. L2_Motion_*).")

    # Matches any XyzL<n>...(Condition|Action)Node base, so adding a new
    # intermediate base (as XyzL2GaitActionNode was in Aug 2026) does not silently
    # turn every node that uses it into a false positive.
    if not re.search(r'class\s+\w+\s*\([^)]*XyzL[123]\w*(?:Condition|Action)Node[^)]*\)', clean):
        violations.append("❌ Does not inherit an XyzL1ConditionNode / XyzL2ActionNode / XyzL2GaitActionNode / XyzL3ActionNode base  →  class L1_Foo(XyzL1ConditionNode): / L2_Foo(XyzL2ActionNode):")
    if not re.search(r'^\s{4}LEVEL\s*=', clean, re.MULTILINE):
        violations.append("❌ Missing class variable LEVEL               →  LEVEL = 'L1'")
    if not re.search(r'^\s{4}BB_READS\s*=', clean, re.MULTILINE):
        violations.append("❌ Missing BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS   →  declare all four at class level")
    # Only meaningful when the node defines setup() at all: a node with no BB access
    # (L2_Gait_Stop, L2_Gait_StepNum, L2_Motion_RunAction, …) legitimately has none.
    if re.search(r'def setup\(', clean) and not re.search(r'super\(\)\.setup\(', clean):
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
    # Look for the emit_input_state() helper call, NOT a quoted "input_state"
    # literal: _strip_noise() blanks every string before we get here, so a
    # quoted-literal search could never match and fired on all six adapters.
    if not re.search(r'emit_input_state\s*\(', clean):
        violations.append('❌ Missing input_state event             →  write_snapshot() must call self.emit_input_state(...)')

    return violations


def check_bb_keys(content: str) -> list:
    """Warn if any constant defined as LATCHED_NS + '...' is absent from ROSA_TOPIC_MAP.

    Only matches the standard pattern:
        FOO_BAR = LATCHED_NS + '/' + FOO_BAR_KEY
    Raw string literals and comments are intentionally ignored to avoid false positives.

    The leading \\s* is load-bearing: these are class attributes indented by four
    spaces, so anchoring at column 0 (as this did until Aug 2026) made `defined`
    always empty and the whole check dead — it never once fired.
    """
    defined = re.findall(r'^\s*(\w+)\s*=\s*LATCHED_NS\s*\+\s*', content, re.MULTILINE)
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


def _log(action, file_path, violations, data):
    """Record the event. Advisory only, and the record must never affect the advice."""
    try:
        import guard_log
        if violations:
            guard_log.log_violations(_HOOK, action, file_path, violations, data)
        else:
            guard_log.log_clean(_HOOK, file_path, data)
    except Exception:
        pass


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
        violations = check_node(content, file_path)
    elif is_adapter:
        violations = check_adapter(content)
    else:
        violations = check_bb_keys(content)

    if not violations:
        _log('pass', file_path, (), data)
        sys.exit(0)

    _log('warned', file_path, violations, data)

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


if __name__ == '__main__':
    main()
