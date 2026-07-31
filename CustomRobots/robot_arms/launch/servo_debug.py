#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class ServoDebug(Node):

    def __init__(self):
        super().__init__("servo_debug")

        self.get_logger().info("Servo Debug iniciado")


def main():

    rclpy.init()

    node = ServoDebug()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()