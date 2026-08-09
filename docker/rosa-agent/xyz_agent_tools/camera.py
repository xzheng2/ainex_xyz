"""
camera.py — standalone camera-frame capture for vision queries.

Grabs ONE frame from the robot camera and returns it as base64 JPEG, ready to
embed in a multimodal LLM message. Pure xyz code: no changes to the vendored
ROSA library — xyz_agent.py injects the frame via ROSA's public chat_history.

Default topic: /camera/image_raw (raw bgr8, converted with numpy + Pillow).
Topics ending in /compressed (sensor_msgs/CompressedImage) are passed through
as-is — their payload is already JPEG.

Override with env XYZ_CAMERA_TOPIC, e.g. /camera/image_rect_color/compressed.
"""

import base64
import io
import os
from pathlib import Path

import rospy
from sensor_msgs.msg import CompressedImage, Image

_FALLBACK_TOPIC = "/camera/image_raw"

# The most recent captured frame is always saved here (single file, overwritten
# on every capture) so the operator can see what the robot saw on its last
# 'look' query. Host path: /home/pi/docker/rosa-agent/logs/last_look.jpg
_LAST_LOOK_PATH = Path(__file__).parent.parent / "logs" / "last_look.jpg"


def capture_frame_b64(
    topic: str = None,
    timeout: float = 3.0,
    max_width: int = 640,
    jpeg_quality: int = 80,
):
    """
    Capture one frame and return (b64_jpeg_str, meta).

    meta = {"topic", "width", "height", "jpeg_kb"}

    Raises rospy.ROSException on timeout (no frame / no publisher) and
    ValueError on an unsupported raw encoding — callers should catch and
    fall back to a text-only query.
    """
    from PIL import Image as PILImage

    topic = topic or os.environ.get("XYZ_CAMERA_TOPIC", _FALLBACK_TOPIC)

    if topic.endswith("/compressed"):
        msg = rospy.wait_for_message(topic, CompressedImage, timeout)
        jpeg = bytes(msg.data)
        with PILImage.open(io.BytesIO(jpeg)) as im:
            width, height = im.size
    else:
        import numpy as np

        msg = rospy.wait_for_message(topic, Image, timeout)
        if msg.encoding not in ("bgr8", "rgb8"):
            raise ValueError(
                f"Unsupported encoding '{msg.encoding}' on {topic} — "
                f"expected bgr8 or rgb8"
            )
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        if msg.encoding == "bgr8":
            arr = arr[..., ::-1]
        im = PILImage.fromarray(arr)
        if im.width > max_width:
            im = im.resize(
                (max_width, round(im.height * max_width / im.width))
            )
        buf = io.BytesIO()
        im.save(buf, format="JPEG", quality=jpeg_quality)
        jpeg = buf.getvalue()
        width, height = im.size

    meta = {
        "topic": topic,
        "width": width,
        "height": height,
        "jpeg_kb": round(len(jpeg) / 1024, 1),
    }

    # A failed save must never break the vision query.
    try:
        _LAST_LOOK_PATH.parent.mkdir(parents=True, exist_ok=True)
        _LAST_LOOK_PATH.write_bytes(jpeg)
        meta["saved_to"] = str(_LAST_LOOK_PATH)
    except OSError:
        pass

    return base64.b64encode(jpeg).decode("ascii"), meta
