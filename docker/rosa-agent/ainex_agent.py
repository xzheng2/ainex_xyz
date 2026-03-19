#!/usr/bin/env python3
"""
ainex_agent.py — ROSA-based read-only diagnostic agent for the Ainex humanoid robot.

Usage:
  # Interactive CLI
  python3.9 ainex_agent.py

  # Single query
  python3.9 ainex_agent.py --query "Is the robot healthy?"

  # As ROS node (ROS must be running)
  python3.9 ainex_agent.py --ros-node
"""

import os
import sys
import argparse
import yaml
from pathlib import Path

# ── Load .env before any other imports ───────────────────────────────────────
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent / ".env")

# ── Read-only guard announcement ─────────────────────────────────────────────
_READONLY_CONFIG = Path(__file__).parent / "config" / "readonly.yaml"
_BLACKLIST_CONFIG = Path(__file__).parent / "config" / "blacklist.yaml"
_WRITE_ENABLED = os.environ.get("AINEX_WRITE_ENABLED", "false").lower() == "true"


def _load_blacklist() -> list:
    try:
        with open(_BLACKLIST_CONFIG) as f:
            cfg = yaml.safe_load(f)
            return cfg.get("blacklist", [])
    except FileNotFoundError:
        return []


def _announce_mode():
    from rich.console import Console
    from rich.panel import Panel
    console = Console()
    if _WRITE_ENABLED:
        console.print(Panel(
            "[bold yellow]WARNING: AINEX_WRITE_ENABLED=true[/bold yellow]\n"
            "Write operations are enabled. Proceed with caution.",
            title="ROSA-Ainex Agent", border_style="yellow"
        ))
    else:
        console.print(Panel(
            "[bold green]READ-ONLY MODE[/bold green]\n"
            "All write operations are disabled.\n"
            f"ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI', 'not set')}",
            title="ROSA-Ainex Agent", border_style="green"
        ))


class AinexAgent:
    """
    ROSA agent configured for Ainex humanoid robot diagnostics.

    Instantiation is deferred until first use so that ROS init can be called
    by the caller if running as a ROS node.
    """

    def __init__(self, verbose: bool = False, streaming: bool = False):
        self._verbose = verbose
        self._streaming = streaming
        self._agent = None

    def _build(self):
        """Lazy-build the ROSA agent on first use."""
        if self._agent is not None:
            return

        from rosa import ROSA
        from llm_config import get_llm
        from ainex_agent_tools import AINEX_TOOLS
        from ainex_agent_tools.prompts import get_ainex_prompts

        blacklist = _load_blacklist()

        self._agent = ROSA(
            ros_version=1,
            llm=get_llm(),
            tools=AINEX_TOOLS,
            tool_packages=[],       # all tools provided explicitly via tools=
            blacklist=blacklist,
            prompts=get_ainex_prompts(),
            verbose=self._verbose,
            streaming=self._streaming,
            accumulate_chat_history=True,
            show_token_usage=False,
            max_iterations=30,
        )

    def chat(self, query: str) -> str:
        self._build()
        return self._agent.invoke(query)


# ── CLI entry point ───────────────────────────────────────────────────────────

def _interactive_loop(agent: AinexAgent):
    from rich.console import Console
    from rich.markdown import Markdown
    console = Console()
    console.print("[dim]Type your query, or 'exit' / Ctrl-C to quit.[/dim]\n")
    while True:
        try:
            query = input("You: ").strip()
        except (EOFError, KeyboardInterrupt):
            console.print("\n[dim]Goodbye.[/dim]")
            break
        if not query:
            continue
        if query.lower() in ("exit", "quit", "q"):
            console.print("[dim]Goodbye.[/dim]")
            break
        try:
            response = agent.chat(query)
            console.print("\n[bold]ROSA-Ainex:[/bold]")
            console.print(Markdown(response))
            console.print()
        except Exception as e:
            console.print(f"[red]Error:[/red] {e}")


def _ros_node_mode(agent: AinexAgent):
    """Run as a ROS node, processing queries from a ROS topic."""
    import rospy
    from std_msgs.msg import String

    rospy.init_node("rosa_ainex_agent", anonymous=False, log_level=rospy.INFO)
    streaming = rospy.get_param("~streaming", False)
    agent._streaming = streaming

    response_pub = rospy.Publisher(
        "/rosa_ainex/response", String, queue_size=1
    )

    def query_cb(msg):
        rospy.loginfo(f"ROSA query: {msg.data}")
        try:
            resp = agent.chat(msg.data)
            response_pub.publish(resp)
        except Exception as e:
            response_pub.publish(f"Error: {e}")

    rospy.Subscriber("/rosa_ainex/query", String, query_cb, queue_size=1)
    rospy.loginfo("ROSA-Ainex agent ready. Publish queries to /rosa_ainex/query")
    rospy.spin()


def main():
    parser = argparse.ArgumentParser(
        description="ROSA-Ainex read-only diagnostic agent"
    )
    parser.add_argument("--query", "-q", help="Single query mode")
    parser.add_argument("--ros-node", action="store_true",
                        help="Run as ROS node (subscribe /rosa_ainex/query)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable ROSA verbose output")
    parser.add_argument("--streaming", "-s", action="store_true",
                        help="Enable streaming responses")
    args = parser.parse_args()

    _announce_mode()

    # ROS node mode needs rospy.init_node before building the agent
    if args.ros_node:
        agent = AinexAgent(verbose=args.verbose, streaming=args.streaming)
        _ros_node_mode(agent)
        return

    # Initialize rospy without ROS node if we just want CLI
    # (Many rospy calls need a node; init one anonymously)
    try:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node("rosa_ainex_cli", anonymous=True, disable_signals=True)
    except Exception as e:
        print(f"Warning: could not init ROS node: {e}", file=sys.stderr)
        print("ROS calls will fail if ainex master is not reachable.", file=sys.stderr)

    agent = AinexAgent(verbose=args.verbose, streaming=args.streaming)

    if args.query:
        response = agent.chat(args.query)
        print(response)
    else:
        _interactive_loop(agent)


if __name__ == "__main__":
    main()
