#!/usr/bin/env python3
"""
Broadcast map → odom so the planner can look up map → base_link.

Accepts the robot's position in the MAP frame (x, y, theta parameters or
/initialpose clicks from RViz).  Reads the current odom → base_link from
TF and computes:  map→odom = map→base * inv(odom→base).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
)


def _se2_inverse(x, y, theta):
    """Inverse of an SE(2) transform."""
    c = math.cos(theta)
    s = math.sin(theta)
    return (-c * x - s * y, s * x - c * y, -theta)


def _se2_compose(x1, y1, th1, x2, y2, th2):
    """Compose two SE(2) transforms: T1 * T2."""
    c = math.cos(th1)
    s = math.sin(th1)
    x = x1 + c * x2 - s * y2
    y = y1 + s * x2 + c * y2
    return (x, y, th1 + th2)


class SetInitialPose(Node):
    def __init__(self):
        super().__init__("set_initial_pose")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("theta", 0.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Subscribe to /initialpose so RViz 2D Pose Estimate can update live
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.on_initialpose,
            10,
        )

        # Try to publish immediately from parameters; retry via timer if TF
        # is not yet available.
        self.initial_done = False
        self.timer = self.create_timer(1.0, self.try_initial_publish)

    # ── helpers ──────────────────────────────────────────────────────────

    def _get_odom_base(self):
        """Read current odom → base_link from TF."""
        odom = self.get_parameter("odom_frame").value
        base = self.get_parameter("base_frame").value
        tfm = self.tf_buffer.lookup_transform(odom, base, rclpy.time.Time())
        t = tfm.transform.translation
        q = tfm.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (t.x, t.y, yaw)

    def _broadcast_map_odom(self, map_base_x, map_base_y, map_base_theta):
        """Compute and broadcast map → odom from the desired map→base pose."""
        try:
            ob_x, ob_y, ob_th = self._get_odom_base()
        except TransformException:
            self.get_logger().warn("Cannot read odom → base_link yet.")
            return False

        inv_ob = _se2_inverse(ob_x, ob_y, ob_th)
        mo_x, mo_y, mo_th = _se2_compose(
            map_base_x, map_base_y, map_base_theta, *inv_ob
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.get_parameter("odom_frame").value
        t.transform.translation.x = mo_x
        t.transform.translation.y = mo_y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, mo_th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)
        self.get_logger().info(
            f"map→base: ({map_base_x:.2f}, {map_base_y:.2f}, {map_base_theta:.2f})  "
            f"odom→base: ({ob_x:.2f}, {ob_y:.2f}, {ob_th:.2f})  "
            f"=> map→odom: ({mo_x:.2f}, {mo_y:.2f}, {mo_th:.2f})"
        )
        return True

    # ── callbacks ────────────────────────────────────────────────────────

    def try_initial_publish(self):
        """Publish from parameters once odom → base_link is available."""
        if self.initial_done:
            self.timer.cancel()
            return
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        theta = float(self.get_parameter("theta").value)
        if self._broadcast_map_odom(x, y, theta):
            self.initial_done = True
            self.timer.cancel()
            self.get_logger().info(
                "Initial pose set. Use RViz '2D Pose Estimate' to adjust."
            )

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        """Handle RViz 2D Pose Estimate clicks to update pose live."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self._broadcast_map_odom(x, y, theta)


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
