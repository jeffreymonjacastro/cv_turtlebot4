#!/usr/bin/env python3
import base64
import os
import socket
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class UdpSavePhotosTelemetryNode(Node):
    def __init__(self):
        super().__init__("udp_save_photos_telemetry")

        self.declare_parameter("port", 6000)
        self.declare_parameter("robot_name", "turtlebot4")
        self.declare_parameter("pairing_code", "ROBOT_PAIRING_CODE")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("jpeg_quality", 95)

        port = self.get_parameter("port").get_parameter_value().integer_value
        self.robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.pairing_code = (
            self.get_parameter("pairing_code").get_parameter_value().string_value
        )
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.jpeg_quality = max(
            1,
            min(100, self.get_parameter("jpeg_quality").get_parameter_value().integer_value),
        )

        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))
        self.get_logger().info(f"ROS_DOMAIN_ID detectado: {self.ros_domain_id}")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.get_logger().info(f"Telemetria UDP escuchando en 0.0.0.0:{port}")

        self.authorized_addr = None
        self.get_logger().info("Esperando HELLO para emparejar PC de telemetria...")

        self.bridge = CvBridge()
        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.sub_img = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.udp_thread.start()

    def udp_loop(self):
        self.get_logger().info("Hilo UDP de telemetria iniciado.")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                text = data.decode("utf-8").strip()
                parts = text.split()
                if not parts:
                    continue

                if parts[0] == "HELLO":
                    self.handle_hello(parts, addr)
                else:
                    self.get_logger().warn(
                        f"Mensaje inesperado en telemetria desde {addr}: '{text}'"
                    )
            except Exception as e:
                self.get_logger().error(f"Error en udp_loop: {e}")
                break

        self.get_logger().info("Hilo UDP de telemetria finalizado.")

    def handle_hello(self, parts, addr):
        if len(parts) < 3:
            self.get_logger().warn(f"HELLO invalido desde {addr}: {parts}")
            return

        desired_domain_str = parts[1]
        pairing_code = parts[2]

        try:
            desired_domain = int(desired_domain_str)
        except ValueError:
            self.get_logger().warn(
                f"HELLO con domain_id invalido desde {addr}: '{desired_domain_str}'"
            )
            return

        if pairing_code != self.pairing_code:
            self.get_logger().warn(f"HELLO con pairing_code incorrecto desde {addr}")
            return

        if desired_domain != self.ros_domain_id:
            self.get_logger().warn(
                f"HELLO con domain_id {desired_domain} pero este robot tiene {self.ros_domain_id}"
            )
            return

        if self.authorized_addr is None:
            self.authorized_addr = addr
            self.get_logger().info(f"PC de telemetria emparejada: {addr}")
        elif addr != self.authorized_addr:
            self.get_logger().warn(
                f"HELLO desde {addr} pero ya hay PC emparejada: {self.authorized_addr}"
            )
            return

        ack_msg = f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8")
        self.sock.sendto(ack_msg, addr)

    def scan_callback(self, msg: LaserScan):
        if self.authorized_addr is None:
            return

        ranges = list(msg.ranges)
        header = (
            f"SCAN {self.ros_domain_id} {self.robot_name} "
            f"{msg.header.stamp.sec} {msg.header.stamp.nanosec} "
            f"{msg.angle_min} {msg.angle_increment} {len(ranges)}"
        )
        ranges_str = " ".join(f"{r:.3f}" for r in ranges)
        data = f"{header} {ranges_str}".encode("utf-8")

        try:
            self.sock.sendto(data, self.authorized_addr)
        except Exception as e:
            self.get_logger().error(
                f"Error enviando SCAN a {self.authorized_addr}: {e}"
            )

    def image_callback(self, msg: Image):
        if self.authorized_addr is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, jpeg = cv2.imencode(
                ".jpg",
                cv_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                return

            b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
            header = (
                f"IMG {self.ros_domain_id} {self.robot_name} "
                f"{msg.header.stamp.sec} {msg.header.stamp.nanosec}"
            )
            self.sock.sendto(f"{header} {b64}".encode("utf-8"), self.authorized_addr)

        except Exception as e:
            self.get_logger().error(f"Error en image_callback: {e}")

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpSavePhotosTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
