"""Rules for the xyz_bt_lib shared library: nodes, adapters, BB keys, L1 purity.

Function bodies are lifted verbatim from xyz_bt_lib_guard and xyz_bt_l1_running_guard.
The imports below are aliased to the names those bodies already call, so nothing inside
a rule changed -- which matters because every violation string here is a lookup key in
guard_log._RULES_BY_HOOK, and a single altered character silently demotes the rule to an
`unmapped.*` slug.

`from __future__ import annotations` is not decoration: the skill validators import these
rules INSIDE the ainex container, which runs Python 3.8. PEP 563 stops the annotations
being evaluated, so `list[str]` stays legal there.
"""
from __future__ import annotations

import re

from guardlib.path_spec import BASE_NODE_PATTERN as _BASE_PATTERN
from guardlib.path_spec import L1_PROJECT_PATTERN as _PROJECT_PATTERN
from guardlib.level_spec import slice_by_level
from guardlib.text_utils import strip_noise as _strip_noise

_RUNNING_RE = re.compile(r'(?<![A-Za-z0-9_.])Status\.RUNNING(?![A-Za-z0-9_])')

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

def _l1_class_slice(src: str) -> str:
    """Return only the XyzL1ConditionNode class body from base_node.py.

    The base file also defines XyzL2ActionNode / XyzL3ActionNode; slicing to the
    L1 class avoids flagging RUNNING that legitimately belongs to another tier.
    """
    m = re.search(r'^class\s+XyzL1ConditionNode\b', src, re.MULTILINE)
    if not m:
        return ''
    rest = src[m.end():]
    nxt = re.search(r'^class\s+\w+', rest, re.MULTILINE)
    return rest[:nxt.start()] if nxt else rest

def _l1_classes(src: str) -> str:
    """Return the bodies of top-level classes declaring LEVEL = 'L1', concatenated.

    Delegates to level_spec, which owns the LEVEL declaration regex shared with
    check_node and check_behaviours.
    """
    return slice_by_level(src, 'L1')

def _l1_scope(src: str, file_path: str) -> str:
    """The slice of `src` this guard is entitled to judge."""
    if _BASE_PATTERN.search(file_path):
        # L2/L3 base classes may legitimately reference RUNNING.
        return _l1_class_slice(src)
    if _PROJECT_PATTERN.search(file_path):
        return _l1_classes(src)
    # A file under behaviours/L1_*/ is L1 all the way down.
    return src

def check_l1_running(content: str, file_path: str = '') -> list:
    """Return violation strings for L1 purity. Empty list means clean.

    Exposed as a function (mirroring xyz_bt_lib_guard.check_node) so the same
    rule can run outside the hook path — hooks only fire when the AGENT writes,
    so a human edit is otherwise unchecked. tools/validate_engine.py sweeps the
    whole library through this.
    """
    scan = _l1_scope(content, file_path)
    if not _RUNNING_RE.search(_strip_noise(scan)):
        return []
    return [
        "❌ L1 condition nodes are PURE PREDICATES: SUCCESS/FAILURE only, never "
        "RUNNING, no cross-tick state.",
        "→  Remove the RUNNING path. For \"condition stable for N ticks\", wrap "
        "this node at the tree layer in "
        "xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator (BB-backed, "
        "reactive-safe) — do not dwell inside the L1 node.",
    ]
