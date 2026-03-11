#!/usr/bin/env python3
"""Broadcast a static map → odom transform so the planner can look up map → base_link."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class SetInitialPose(Node):
    def __init__(self):
        super().__init__("set_initial_pose")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("theta", 0.0)

        self.broadcaster = StaticTransformBroadcaster(self)

        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        theta = float(self.get_parameter("theta").value)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Published static map → odom: x={x:.2f} y={y:.2f} theta={theta:.2f}"
        )


def main():
    rclpy.init()
    node = SetInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
