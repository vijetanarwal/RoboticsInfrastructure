#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory


class ServoDebug(Node):

    def __init__(self):

        super().__init__("servo_debug")

        self.get_logger().info("Servo Debug iniciado")

        self.twist_count = 0
        self.traj_count = 0

        # ==========================
        # Subscriber Twist
        # ==========================
        self.create_subscription(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            self.twist_callback,
            10,
        )

        # ==========================
        # Subscriber Trayectoria
        # ==========================
        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self.traj_callback,
            10,
        )

    def twist_callback(self, msg):

        self.twist_count += 1

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

    def traj_callback(self, msg):

        self.traj_count += 1

        if self.traj_count <= 5 or self.traj_count % 20 == 0:

            print("\n========== TRAJECTORY ==========")
            print(f"Trajectory : {self.traj_count}")

            if len(msg.points) == 0:
                print("No trajectory points.")
                print("===============================")
                return

            point = msg.points[0]

            print("Joint positions:")

            for joint, pos in zip(msg.joint_names, point.positions):
                print(f"  {joint:25s}: {pos:+.6f}")

            if len(point.velocities):

                print("\nJoint velocities:")

                for joint, vel in zip(msg.joint_names, point.velocities):
                    print(f"  {joint:25s}: {vel:+.6f}")

            else:

                print("\nJoint velocities: EMPTY")

            print("===============================")


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