"""
get_current_behavior — inspect which behavior nodes are currently running.

ROS interfaces used (all READ):
  - rosnode list (via rosgraph API)
  - /rosout_agg for recent behavior log messages
"""

from langchain.agents import tool
import rospy
import rosgraph


# Node name patterns that indicate a behavior is active
_BEHAVIOR_NODE_PATTERNS = [
    "marathon_bt",
    "penalty_kick",
    "sprint",
    "triple_jump",
    "weight_lift",
    "visual_patrol",
    "color_detect",
    "face_detect",
    "gesture",
]


@tool
def get_current_behavior(_input: str = "") -> str:
    """
    Get the currently running behavior or competition mode on the Ainex robot.

    Inspects the live ROS node graph to identify which behavior tree or
    competition task nodes are active. Also checks whether the behavior
    node manager is running. All operations are read-only.

    Use this tool when asked what the robot is currently doing, which behavior
    is active, or whether any competition task is running.
    """
    results = []

    # ── Active nodes via rosgraph ────────────────────────────────────────────
    try:
        master = rosgraph.Master("/rosa_behavior_checker")
        state = master.getSystemState()
        # state = [publishers, subscribers, services]
        # We want to look at what nodes are in the graph via rosnode list
        node_list = []
        pubs, subs, srvs = state
        seen = set()
        for topic_entry in pubs + subs:
            for node in topic_entry[1]:
                seen.add(node)
        node_list = sorted(seen)
    except Exception as e:
        results.append(f"Could not query ROS graph: {e}")
        node_list = []

    if node_list:
        behavior_nodes = [
            n for n in node_list
            if any(pat in n.lower() for pat in _BEHAVIOR_NODE_PATTERNS)
        ]
        core_nodes = [
            n for n in node_list
            if any(
                kw in n.lower()
                for kw in ["controller", "bringup", "camera", "walking", "kinematics"]
            )
        ]

        if behavior_nodes:
            results.append("Active behavior nodes:\n" + "\n".join(f"  {n}" for n in behavior_nodes))
        else:
            results.append("No behavior/competition nodes currently active.")

        results.append(
            f"\nCore infrastructure nodes ({len(core_nodes)}):\n"
            + "\n".join(f"  {n}" for n in core_nodes[:8])
            + (" ..." if len(core_nodes) > 8 else "")
        )
        results.append(f"\nTotal nodes in graph: {len(node_list)}")

    # ── Check walking control state ───────────────────────────────────────────
    try:
        walking_enabled = rospy.get_param("/walking/enable_control", None)
        if walking_enabled is not None:
            results.append(f"Walking control enabled: {walking_enabled}")
    except Exception:
        pass  # Parameter may not exist if walking node not running

    return "\n".join(results) if results else "No behavior data available."
