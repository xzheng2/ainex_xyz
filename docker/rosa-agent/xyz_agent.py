#!/usr/bin/env python3
"""
xyz_agent.py — ROSA-based read-only diagnostic agent for the Ainex humanoid robot.

Usage:
  # Interactive CLI
  python3.9 xyz_agent.py

  # Single query
  python3.9 xyz_agent.py --query "Is the robot healthy?"

  # As ROS node (ROS must be running)
  python3.9 xyz_agent.py --ros-node
"""

import os
import sys
import warnings
import argparse
import yaml
from pathlib import Path

# Suppress pydantic v2 noise from langchain using builtin 'any' as a type annotation
try:
    from pydantic.warnings import ArbitraryTypeWarning
    warnings.filterwarnings("ignore", category=ArbitraryTypeWarning)
except ImportError:
    pass

from runtime.session_logger import SessionLogger

# ── Load .env before any other imports ───────────────────────────────────────
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent / ".env")

# ── Read-only guard announcement ─────────────────────────────────────────────
_READONLY_CONFIG = Path(__file__).parent / "config" / "readonly.yaml"
_BLACKLIST_CONFIG = Path(__file__).parent / "config" / "blacklist.yaml"
_WRITE_ENABLED = os.environ.get("XYZ_WRITE_ENABLED", "false").lower() == "true"


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
            "[bold yellow]WARNING: XYZ_WRITE_ENABLED=true[/bold yellow]\n"
            "Write operations are enabled. Proceed with caution.",
            title="ROSA-XYZ Agent", border_style="yellow"
        ))
    else:
        console.print(Panel(
            "[bold green]READ-ONLY MODE[/bold green]\n"
            "All write operations are disabled.\n"
            f"ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI', 'not set')}",
            title="ROSA-XYZ Agent", border_style="green"
        ))


class XyzAgent:
    """
    ROSA agent configured for Ainex humanoid robot diagnostics.

    Instantiation is deferred until first use so that ROS init can be called
    by the caller if running as a ROS node.
    """

    def __init__(self, verbose: bool = False, streaming: bool = False, logger=None):
        self._verbose = verbose
        self._streaming = streaming
        self._agent = None
        self._logger = logger  # Optional[SessionLogger]
        # Most recent look-frame message kept in chat history so follow-up
        # questions ("in the last image you captured...") can still see it.
        # Evicted only when a new look frame replaces it — at most one image
        # in history (~85 tokens/turn at detail=low).
        self._last_img_msg = None

    def _build(self):
        """Lazy-build the ROSA agent on first use."""
        if self._agent is not None:
            return

        from rosa import ROSA
        from llm_config import get_llm
        from xyz_agent_tools import XYZ_TOOLS
        from xyz_agent_tools.prompts import get_xyz_prompts

        blacklist = _load_blacklist()

        self._agent = ROSA(
            ros_version=1,
            llm=get_llm(),
            tools=XYZ_TOOLS,
            tool_packages=[],       # all tools provided explicitly via tools=
            blacklist=blacklist,
            prompts=get_xyz_prompts(),
            verbose=self._verbose,
            streaming=self._streaming,
            accumulate_chat_history=True,
            show_token_usage=False,
            max_iterations=30,
            max_execution_time=60,
            return_intermediate_steps=True,
        )

    def chat(self, query: str, with_camera: bool = False) -> str:
        self._build()

        # Progressive knowledge loading: the base system prompt no longer carries
        # the full ROS reference; matching sections are prepended per query.
        from xyz_agent_tools.knowledge import retrieve_knowledge
        context = retrieve_knowledge(query)
        payload = query if not context else (
            "[Reference context — auto-retrieved for this query]\n"
            + context
            + "\n\n[User query]\n" + query
        )

        # Camera-frame vision: inject one frame as a multimodal message via
        # ROSA's public chat_history (vendor code untouched), remove it after
        # the turn so the base64 never persists into later requests.
        img_msg = None
        frame_note = ""
        if with_camera:
            try:
                from xyz_agent_tools.camera import capture_frame_b64
                from langchain_core.messages import HumanMessage
                b64, meta = capture_frame_b64()
                img_msg = HumanMessage(content=[
                    {"type": "text", "text": (
                        "[Most recent frame from my head camera. PRIMARY "
                        "evidence for vision questions; later questions about "
                        "'the last image' refer to this frame.]"
                    )},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{b64}",
                        "detail": os.environ.get("XYZ_VISION_DETAIL", "low"),
                    }},
                ])
                if self._last_img_msg is not None:
                    try:
                        self._agent.chat_history.remove(self._last_img_msg)
                    except ValueError:
                        pass
                self._agent.chat_history.append(img_msg)
                self._last_img_msg = img_msg
                # The latest user message carries the strongest instruction:
                # without this, ROSA's base prompt ("always check with tools
                # first") wins and the model answers from ROS state instead
                # of looking at the image.
                payload = (
                    "[VISION QUERY — a live camera frame is attached. Answer "
                    "by examining the IMAGE itself: describe what is actually "
                    "visible. Do NOT call ROS tools to answer what can be "
                    "seen; only use tools if the question also asks about ROS "
                    "or detection-node state, and clearly separate what I see "
                    "in the image from what ROS reports.]\n\n"
                ) + payload
                frame_note = (
                    f" [camera frame attached: {meta['width']}x{meta['height']}, "
                    f"{meta['jpeg_kb']} KB, {meta['topic']}]"
                )
                print(f"[camera frame attached: {meta['width']}x{meta['height']}, "
                      f"{meta['jpeg_kb']} KB]")
            except Exception as e:
                frame_note = " [camera frame capture FAILED]"
                payload = (
                    f"(Note: a camera frame was requested but capture failed: "
                    f"{e}. Answer from other data and tell the user.)\n\n"
                ) + payload

        try:
            response = self._agent.invoke(payload)
        except BaseException as e:
            if self._logger:
                self._logger.log_turn(query + frame_note, "", status="error",
                                      error=str(e))
            raise

        if self._logger:
            steps = getattr(self._agent, "last_intermediate_steps", [])
            if response.startswith("An error occurred: "):
                self._logger.log_turn(query + frame_note, response, status="error",
                                      error=response, intermediate_steps=steps)
            else:
                self._logger.log_turn(query + frame_note, response,
                                      intermediate_steps=steps)
        return response


# ── CLI entry point ───────────────────────────────────────────────────────────

def _interactive_loop(agent: XyzAgent):
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
        # 'look <question>' attaches one camera frame to the query
        with_camera = False
        if query.lower() == "look" or query.lower().startswith("look "):
            with_camera = True
            query = query[4:].strip() or "Describe what you see in the camera frame."
        try:
            response = agent.chat(query, with_camera=with_camera)
            console.print("\n[bold]ROSA-XYZ:[/bold]")
            console.print(Markdown(response))
            console.print()
        except Exception as e:
            console.print(f"[red]Error:[/red] {e}")


def _ros_node_mode(agent: XyzAgent, logger=None):
    """Run as a ROS node, processing queries from a ROS topic."""
    import rospy
    from std_msgs.msg import String

    rospy.init_node("rosa_xyz_agent", anonymous=False, log_level=rospy.INFO)
    streaming = rospy.get_param("~streaming", False)
    agent._streaming = streaming

    if logger:
        rospy.on_shutdown(logger.end)

    response_pub = rospy.Publisher(
        "/rosa_xyz/response", String, queue_size=1
    )

    def query_cb(msg):
        rospy.loginfo(f"ROSA query: {msg.data}")
        try:
            resp = agent.chat(msg.data)
            response_pub.publish(resp)
        except Exception as e:
            response_pub.publish(f"Error: {e}")

    rospy.Subscriber("/rosa_xyz/query", String, query_cb, queue_size=1)
    rospy.loginfo("ROSA-XYZ agent ready. Publish queries to /rosa_xyz/query")
    rospy.spin()


def main():
    parser = argparse.ArgumentParser(
        description="ROSA-XYZ read-only diagnostic agent"
    )
    parser.add_argument("--query", "-q", help="Single query mode")
    parser.add_argument("--look", "-l", metavar="QUERY",
                        help="Single query mode with one camera frame attached")
    parser.add_argument("--ros-node", action="store_true",
                        help="Run as ROS node (subscribe /rosa_xyz/query)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable ROSA verbose output")
    parser.add_argument("--streaming", "-s", action="store_true",
                        help="Enable streaming responses")
    args = parser.parse_args()

    _announce_mode()

    # Determine mode for session log naming
    if args.ros_node:
        mode = "ros_node"
    elif args.query or args.look:
        mode = "single_query"
    else:
        mode = "cli"

    logger = SessionLogger(mode=mode)
    logger.start()

    # ROS node mode needs rospy.init_node before building the agent
    if args.ros_node:
        agent = XyzAgent(verbose=args.verbose, streaming=args.streaming, logger=logger)
        _ros_node_mode(agent, logger)
        return

    # Initialize rospy without ROS node if we just want CLI
    # (Many rospy calls need a node; init one anonymously)
    try:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node("rosa_xyz_cli", anonymous=True, disable_signals=True)
    except Exception as e:
        print(f"Warning: could not init ROS node: {e}", file=sys.stderr)
        print("ROS calls will fail if the ainex master is not reachable.", file=sys.stderr)

    agent = XyzAgent(verbose=args.verbose, streaming=args.streaming, logger=logger)

    if args.query or args.look:
        try:
            response = agent.chat(args.query or args.look,
                                  with_camera=bool(args.look))
            print(response)
        finally:
            logger.end()
    else:
        try:
            _interactive_loop(agent)
        finally:
            logger.end()


if __name__ == "__main__":
    main()
