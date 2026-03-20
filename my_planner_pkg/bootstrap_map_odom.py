#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros


class BootstrapMapOdom(Node):
    def __init__(self):
        super().__init__("bootstrap_map_odom")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("shutdown_delay_s", 2.0)

        self._shutdown_time = None
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            self.on_initial_pose,
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(publish_rate_hz, 1e-3)
        self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "Publishing temporary identity map->odom TF until /initialpose is received."
        )

    def on_initial_pose(self, _msg: PoseWithCovarianceStamped):
        if self._shutdown_time is not None:
            return

        delay = float(self.get_parameter("shutdown_delay_s").value)
        self._shutdown_time = self.get_clock().now() + rclpy.duration.Duration(seconds=delay)
        self.get_logger().info(
            f"/initialpose received. Stopping temporary map->odom TF in {delay:.1f}s."
        )

    def on_timer(self):
        now = self.get_clock().now()

        if self._shutdown_time is not None and now >= self._shutdown_time:
            self.get_logger().info("Bootstrap TF finished. Shutting down helper node.")
            rclpy.shutdown()
            return

        msg = TransformStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.child_frame_id = str(self.get_parameter("odom_frame").value)
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = BootstrapMapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
