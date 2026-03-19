"""
get_latest_detections — read the most recent object/color detections.

ROS interfaces used (all READ):
  - /object_detect/objects     (ainex_interfaces/ObjectsInfo)
  - /color_detect/objects      (ainex_interfaces/ObjectsInfo)  [if active]
  - /apriltag_ros/tag_detections  (apriltag_ros/AprilTagDetectionArray) [if active]
"""

from langchain.agents import tool
import rospy


_TIMEOUT = 2.0  # seconds to wait for a single message


def _try_topic(topic: str, msg_type, timeout: float = _TIMEOUT):
    """Subscribe for one message, return it or None on timeout."""
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.ROSException:
        return None
    except Exception:
        return None


@tool
def get_latest_detections(_input: str = "") -> str:
    """
    Get the latest object detection results from the Ainex robot's vision system.

    Reads one message from each active detection topic:
    - /object_detect/objects  (generic object detections)
    - /color_detect/objects   (color-based detections)
    - /apriltag_ros/tag_detections (AprilTag fiducial markers)

    Returns object labels, positions, and confidence scores if available.
    All operations are read-only (subscribe, no publish).

    Use this tool when asked what the robot can see, what objects are detected,
    or what AprilTags are visible.
    """
    results = []

    # ── Generic object detections ─────────────────────────────────────────────
    try:
        from ainex_interfaces.msg import ObjectsInfo
        msg = _try_topic("/object_detect/objects", ObjectsInfo)
        if msg is not None and msg.objects:
            lines = []
            for obj in msg.objects[:10]:  # cap output
                lines.append(
                    f"  [{obj.label}] x={obj.x:.1f} y={obj.y:.1f} "
                    f"w={obj.width:.1f} h={obj.height:.1f}"
                )
            results.append(
                f"Object detections ({len(msg.objects)} objects):\n" + "\n".join(lines)
            )
        elif msg is not None:
            results.append("Object detections: topic active but no objects in frame")
        else:
            results.append("Object detections: /object_detect/objects not publishing")
    except Exception as e:
        results.append(f"Object detections: error — {e}")

    # ── Color detections ──────────────────────────────────────────────────────
    try:
        from ainex_interfaces.msg import ObjectsInfo
        msg = _try_topic("/color_detect/objects", ObjectsInfo, timeout=1.5)
        if msg is not None and msg.objects:
            colors = [obj.label for obj in msg.objects]
            results.append(f"Color detections: {colors}")
        elif msg is not None:
            results.append("Color detections: topic active but nothing detected")
        # If None, topic is simply not active — skip silently
    except Exception as e:
        results.append(f"Color detections: error — {e}")

    # ── AprilTag detections ───────────────────────────────────────────────────
    try:
        # apriltag_ros is a C++ package; its Python message import path
        from apriltag_ros.msg import AprilTagDetectionArray
        msg = _try_topic("/tag_detections", AprilTagDetectionArray, timeout=1.5)
        if msg is not None and msg.detections:
            tags = [f"id={d.id[0]}" for d in msg.detections[:5]]
            results.append(f"AprilTag detections: {tags}")
        elif msg is not None:
            results.append("AprilTag detections: topic active but no tags in view")
    except ImportError:
        pass  # apriltag_ros may not be in ainex workspace
    except Exception as e:
        results.append(f"AprilTag detections: error — {e}")

    return "\n".join(results) if results else "No detection topics currently active."
