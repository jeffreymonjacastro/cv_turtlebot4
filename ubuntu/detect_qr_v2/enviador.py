#!/usr/bin/env python3
import os
import socket
import threading
import base64

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2


class UdpTelemetryNode(Node):
    def __init__(self):
        super().__init__("udp_telemetry_v2")

        # ========= Parametros =========
        self.declare_parameter("port", int(os.environ.get("QR_TELEMETRY_PORT", "6611")))
        self.declare_parameter("robot_name", "turtlebot4")
        self.declare_parameter("pairing_code", "ROBOT_PAIRING_CODE")  # debe coincidir con la PC
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")

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

        # ========= ROS_DOMAIN_ID =========
        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))
        self.get_logger().info(f"ROS_DOMAIN_ID detectado: {self.ros_domain_id}")

        # ========= Socket UDP =========
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.get_logger().info(f"Telemetria UDP escuchando en 0.0.0.0:{port}")

        # ========= Estado de emparejamiento =========
        self.authorized_addr = None  # (ip, puerto) de la PC emparejada
        self.get_logger().info("Esperando HELLO para emparejar PC de telemetria...")

        # ========= Subscripciones =========
        self.bridge = CvBridge()

        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.sub_img = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        # ========= Hilo UDP (para HELLO / ACK) =========
        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.udp_thread.start()

    # ================== Hilo UDP (HELLO/ACK) ==================
    def udp_loop(self):
        self.get_logger().info("Hilo UDP de telemetria iniciado.")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                text = data.decode("utf-8").strip()
                parts = text.split()

                if not parts:
                    continue

                cmd_type = parts[0]

                if cmd_type == "HELLO":
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
        # Formato: HELLO <desired_domain_id> <pairing_code>
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
        else:
            if addr != self.authorized_addr:
                self.get_logger().warn(
                    f"HELLO desde {addr} pero ya hay PC emparejada: {self.authorized_addr}"
                )
                return

        ack_msg = f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8")
        self.sock.sendto(ack_msg, addr)

    # ================== Callbacks de sensores ==================
    def scan_callback(self, msg: LaserScan):
        if self.authorized_addr is None:
            return

        ranges = list(msg.ranges)
        n = len(ranges)

        header = (
            f"SCAN {self.ros_domain_id} {self.robot_name} "
            f"{msg.header.stamp.sec} {msg.header.stamp.nanosec} "
            f"{msg.angle_min} {msg.angle_increment} {n}"
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

            ok, jpeg = cv2.imencode(".jpg", cv_img)
            if not ok:
                return

            b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")

            header = (
                f"IMG {self.ros_domain_id} {self.robot_name} "
                f"{msg.header.stamp.sec} {msg.header.stamp.nanosec}"
            )
            data = f"{header} {b64}".encode("utf-8")

            self.sock.sendto(data, self.authorized_addr)

        except Exception as e:
            self.get_logger().error(f"Error en image_callback: {e}")

    # ================== Cleanup ==================
    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
