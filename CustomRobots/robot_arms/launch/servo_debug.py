#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class ServoDebug(Node):

    def __init__(self):

        super().__init__("servo_debug")

        self.get_logger().info("Servo Debug iniciado")

        self.twist_count = 0

        self.create_subscription(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            self.twist_callback,
            10,
        )

    def twist_callback(self, msg):

        self.twist_count += 1

        # Imprime los primeros 5 mensajes y luego uno cada 20
        if self.twist_count <= 5 or self.twist_count % 20 == 0:

            print("\n========== TWIST RECEIVED ==========")
            print(f"Command : {self.twist_count}")
            print(
                f"Time    : "
                f"{msg.header.stamp.sec}."
                f"{msg.header.stamp.nanosec:09d}"
            )
            print(f"Frame   : {msg.header.frame_id}")

            print(
                f"Linear  : "
                f"x={msg.twist.linear.x:+.4f} "
                f"y={msg.twist.linear.y:+.4f} "
                f"z={msg.twist.linear.z:+.4f}"
            )

            print(
                f"Angular : "
                f"x={msg.twist.angular.x:+.4f} "
                f"y={msg.twist.angular.y:+.4f} "
                f"z={msg.twist.angular.z:+.4f}"
            )

            print("====================================")


def main():

    rclpy.init()

    node = ServoDebug()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()