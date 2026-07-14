#!/usr/bin/env python3
"""Probe live robot camera frames for QR decoding without starting navigation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
REACTIVE_NAV = REPO_ROOT / "ubuntu" / "reactive_nav"
if str(REACTIVE_NAV) not in sys.path:
    sys.path.insert(0, str(REACTIVE_NAV))
ROBOT_REACTIVE_NAV = REPO_ROOT / "reactive_nav"
if str(ROBOT_REACTIVE_NAV) not in sys.path:
    sys.path.insert(0, str(ROBOT_REACTIVE_NAV))

try:
    from ubuntu.reactive_nav.qr_detection import decode_qr_image  # noqa: E402
except ImportError:  # robot deployment layout: /home/ubuntu/reactive_nav_test/reactive_nav
    from qr_detection import decode_qr_image  # type: ignore  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/oakd/rgb/preview/image_raw")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--save-frame", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class Probe(Node):
        def __init__(self):
            super().__init__("qr_camera_probe")
            self.bridge = CvBridge()
            self.detector = cv2.QRCodeDetector()
            self.frames = 0
            self.last_status = ""
            self.saved = False
            self.sub = self.create_subscription(Image, args.image_topic, self.callback, qos_profile_sensor_data)

        def callback(self, msg):
            self.frames += 1
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if args.save_frame and not self.saved:
                Path(args.save_frame).parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(args.save_frame, cv_img)
                self.saved = True
            result = decode_qr_image(self.detector, cv_img)
            status = (
                f"frame={self.frames} status={result.status} variant={result.variant} "
                f"detected_count={result.detected_count} content={result.content!r} error={result.error}"
            )
            if result.content or status != self.last_status:
                self.get_logger().info(status)
                self.last_status = status

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + max(1.0, args.frames / 5.0 + 2.0)
    try:
        while rclpy.ok() and node.frames < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
