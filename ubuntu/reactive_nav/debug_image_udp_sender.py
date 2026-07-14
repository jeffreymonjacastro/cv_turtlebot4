#!/usr/bin/env python3
"""Debug-only UDP image sender for the laptop YOLO viewer.

This node is intentionally separate from ``reactive_navigator.py``. It only
subscribes to the robot camera, waits for the laptop viewer handshake, and sends
JPEG frames using the IMG packet format consumed by ``win/yolo/recibidor.py``.
It does not read LiDAR and never publishes motion commands.
"""

from __future__ import annotations

import base64
import os
import socket
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DebugImageUdpSender(Node):
    def __init__(self):
        super().__init__("debug_image_udp_sender")

        self.declare_parameter("port", int(os.environ.get("YOLO_TELEMETRY_PORT", "6610")))
        self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
        self.declare_parameter("pairing_code", "ROBOT_A_2")
        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter(
            "send_hz",
            5.0,
            ParameterDescriptor(
                description="Maximum debug image send rate in Hz.",
                dynamic_typing=True,
            ),
        )

        self.port = self._param_int("port")
        self.robot_name = self._param_str("robot_name")
        self.pairing_code = self._param_str("pairing_code")
        self.image_topic = self._param_str("image_topic")
        self.jpeg_quality = max(1, min(100, self._param_int("jpeg_quality")))
        self.min_send_interval_s = 1.0 / max(0.1, self._param_float("send_hz"))
        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))

        self.authorized_addr = None
        self.last_send_time = 0.0
        self.running = True
        self.bridge = CvBridge()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.udp_thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.udp_thread.start()

        self.get_logger().info(
            "debug image UDP sender started "
            f"port={self.port} image_topic={self.image_topic} "
            f"send_hz={1.0 / self.min_send_interval_s:.1f} jpeg_quality={self.jpeg_quality} "
            f"domain={self.ros_domain_id}"
        )
        self.get_logger().info("waiting for laptop HELLO from win/yolo/recibidor.py")

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def udp_loop(self) -> None:
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except OSError:
                break
            except Exception as exc:
                self.get_logger().warn(f"UDP receive error: {exc}")
                continue

            text = data.decode("utf-8", errors="ignore").strip()
            parts = text.split()
            if not parts:
                continue
            if parts[0] != "HELLO":
                self.get_logger().warn(f"unexpected UDP message from {addr}: {text!r}")
                continue
            self.handle_hello(parts, addr)

    def handle_hello(self, parts: list[str], addr) -> None:
        if len(parts) < 3:
            self.get_logger().warn(f"invalid HELLO from {addr}: {parts}")
            return
        try:
            desired_domain = int(parts[1])
        except ValueError:
            self.get_logger().warn(f"invalid HELLO domain from {addr}: {parts[1]!r}")
            return

        pairing_code = parts[2]
        if desired_domain != self.ros_domain_id:
            self.get_logger().warn(
                f"HELLO domain mismatch from {addr}: requested={desired_domain} local={self.ros_domain_id}"
            )
            return
        if pairing_code != self.pairing_code:
            self.get_logger().warn(f"HELLO pairing_code mismatch from {addr}")
            return

        if self.authorized_addr is None:
            self.authorized_addr = addr
            self.get_logger().info(f"laptop paired for debug images: {addr}")
        elif addr != self.authorized_addr:
            self.get_logger().warn(f"HELLO from {addr}, already paired with {self.authorized_addr}")
            return

        ack_msg = f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8")
        try:
            self.sock.sendto(ack_msg, addr)
        except OSError as exc:
            self.get_logger().warn(f"failed to send ACK to {addr}: {exc}")

    def image_callback(self, msg: Image) -> None:
        if self.authorized_addr is None:
            return

        now = time.monotonic()
        if now - self.last_send_time < self.min_send_interval_s:
            return
        self.last_send_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, jpeg = cv2.imencode(
                ".jpg",
                cv_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                self.get_logger().warn("failed to JPEG-encode camera frame")
                return

            b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
            header = (
                f"IMG {self.ros_domain_id} {self.robot_name} "
                f"{msg.header.stamp.sec} {msg.header.stamp.nanosec}"
            )
            self.sock.sendto(f"{header} {b64}".encode("utf-8"), self.authorized_addr)
        except Exception as exc:
            self.get_logger().warn(f"failed to send debug image frame: {exc}")

    def destroy_node(self) -> None:
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebugImageUdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
