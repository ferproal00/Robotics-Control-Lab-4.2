#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener


class TriangleMaker(Node):
    """
    Publishes a desired end-effector position (PointStamped) on /desired_position
    following a triangle in link_base.

    The PD controller (position_controller) should subscribe to /desired_position
    and convert error -> /controller/delta_twist_cmds.
    """

    def __init__(self):
        super().__init__("triangle_maker_xarm_lite6")

        # ---------- Params ----------
        self.target_topic = str(self.declare_parameter("target_topic", "/desired_position").value)

        self.plane = str(self.declare_parameter("plane", "xy").value).lower().strip()      # xy | xz | yz
        self.hold_z = bool(self.declare_parameter("hold_z", True).value)

        self.side = float(self.declare_parameter("side", 0.10).value)      # meters (triangle side length-ish)
        self.frequency = float(self.declare_parameter("frequency", 0.06).value)  # Hz (loops per second)

        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.02).value)  # 50 Hz
        self.softstart_s = float(self.declare_parameter("softstart_s", 2.0).value)  # ramp to full size

        # ---------- Pub ----------
        self.pub = self.create_publisher(PointStamped, self.target_topic, 10)

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # center from current EE pose
        self.center = None
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(self.publish_period_s, self._tick)

        self.get_logger().info(
            "✅ triangle_maker started\n"
            f"  target_topic={self.target_topic}\n"
            f"  plane={self.plane}, hold_z={self.hold_z}\n"
            f"  side={self.side} m, frequency={self.frequency} Hz\n"
            f"  publish_period_s={self.publish_period_s}\n"
        )

    def _read_ee_pos(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "link_base", "link_eef", rclpy.time.Time()
            )
            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ], dtype=float)
        except Exception:
            return None

    def _triangle_vertices(self, center, size):
        # Equilateral triangle in 2D around origin, then shifted to center
        # vertices in local 2D: angles 90, 210, 330 deg (nice orientation)
        angles = [math.radians(90), math.radians(210), math.radians(330)]
        r = size / math.sqrt(3)  # relates "side" to circumradius-ish
        pts2 = [(r * math.cos(a), r * math.sin(a)) for a in angles]

        cx, cy, cz = center
        if self.plane == "xy":
            verts = [np.array([cx + px, cy + py, cz], dtype=float) for (px, py) in pts2]
        elif self.plane == "xz":
            verts = [np.array([cx + px, cy, cz + py], dtype=float) for (px, py) in pts2]
        elif self.plane == "yz":
            verts = [np.array([cx, cy + px, cz + py], dtype=float) for (px, py) in pts2]
        else:
            verts = [np.array([cx + px, cy + py, cz], dtype=float) for (px, py) in pts2]
        return verts

    @staticmethod
    def _lerp(a, b, s):
        return (1.0 - s) * a + s * b

    def _triangle_target(self, t_sec):
        # ramp size in first softstart_s seconds
        ramp = min(max(t_sec / max(self.softstart_s, 1e-6), 0.0), 1.0)
        size = ramp * self.side

        v = self._triangle_vertices(self.center, size)

        # phase in [0,1)
        phase = (t_sec * self.frequency) % 1.0

        # split into 3 edges, each 1/3 of cycle
        edge = int(phase * 3.0)  # 0,1,2
        s = (phase * 3.0) - edge  # local progress 0..1

        p0 = v[edge]
        p1 = v[(edge + 1) % 3]
        target = self._lerp(p0, p1, s)

        # optionally hold Z exactly at center.z (useful if plane is xy)
        if self.hold_z:
            target[2] = self.center[2]

        return target

    def _tick(self):
        if self.center is None:
            p = self._read_ee_pos()
            if p is None:
                return
            self.center = p.copy()
            self.t0 = self.get_clock().now()
            self.get_logger().info(f"✅ Center set to {np.round(self.center, 3)}")
            return

        t_sec = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        target = self._triangle_target(t_sec)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.point.x = float(target[0])
        msg.point.y = float(target[1])
        msg.point.z = float(target[2])

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TriangleMaker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()