#!/usr/bin/env python3
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PerturbationInjector(Node):
    def __init__(self):
        super().__init__("xarm_perturbation_injector")

        self.input_topic = self.declare_parameter("input_topic", "/controller/delta_twist_cmds").value
        self.output_topic = self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds").value

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.mode = str(self.declare_parameter("mode", "sine").value).lower().strip()  # off|sine|gaussian

        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.02).value)
        self.max_lin = float(self.declare_parameter("max_linear_speed", 0.20).value)

        self.sine_freq_hz = float(self.declare_parameter("sine_freq_hz", 1.0).value)
        self.sine_amp_linear = float(self.declare_parameter("sine_amp_linear", 0.01).value)
        self.sine_axis = str(self.declare_parameter("sine_axis", "x").value).lower().strip()

        self.noise_std_linear = float(self.declare_parameter("gauss_std_linear", 0.01).value)

        base = self.declare_parameter("base_linear", [0.0, 0.0, 0.0]).value
        try:
            self.base_linear = np.array([float(base[0]), float(base[1]), float(base[2])], dtype=float)
        except Exception:
            self.base_linear = np.zeros(3, dtype=float)

        self.debug = bool(self.declare_parameter("debug", True).value)
        self.debug_period_s = float(self.declare_parameter("debug_period_s", 1.0).value)
        self._out_count = 0
        self._last_dbg_wall = time.time()

        qos_name = str(self.declare_parameter("pub_reliability", "reliable").value).lower().strip()
        reliability = ReliabilityPolicy.RELIABLE if qos_name in ("reliable", "r") else ReliabilityPolicy.BEST_EFFORT

        self.pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(TwistStamped, self.output_topic, self.pub_qos)
        self.sub = self.create_subscription(TwistStamped, self.input_topic, self._on_in, 10)

        self.last_in = np.zeros(3, dtype=float)
        self.t0 = time.time()
        self.rng = np.random.default_rng(7)

        self.timer = self.create_timer(self.publish_period_s, self.tick)

        self.get_logger().info(
            "✅ xarm_perturbation_injector (INJECTOR)\n"
            f"   IN : {self.input_topic}\n"
            f"   OUT: {self.output_topic}\n"
            f"   mode={self.mode}, enabled={self.enabled}\n"
            f"   PUB reliability={('RELIABLE' if reliability==ReliabilityPolicy.RELIABLE else 'BEST_EFFORT')}\n"
        )

    def _on_in(self, msg: TwistStamped):
        self.last_in = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float)

    def _dp(self):
        if self.mode == "off":
            return np.zeros(3, dtype=float)

        if self.mode == "gaussian":
            return self.rng.normal(0.0, self.noise_std_linear, size=3)

        s = math.sin(2.0 * math.pi * self.sine_freq_hz * (time.time() - self.t0))
        amp = self.sine_amp_linear * s
        dp = np.zeros(3, dtype=float)
        axis = {"x": 0, "y": 1, "z": 2}.get(self.sine_axis, 0)
        dp[axis] = amp
        return dp

    def tick(self):
        vin = self.last_in.copy()

        if not self.enabled:
            vout = np.clip(vin, -self.max_lin, self.max_lin)
            self._publish(vout, note="DISABLED(pass-through)")
            return

        vout = vin + self.base_linear + self._dp()
        vout = np.clip(vout, -self.max_lin, self.max_lin)
        self._publish(vout, note="RUN(add)")

    def _publish(self, v_xyz: np.ndarray, note=""):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "link_base"
        out.twist.linear.x = float(v_xyz[0])
        out.twist.linear.y = float(v_xyz[1])
        out.twist.linear.z = float(v_xyz[2])
        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0
        out.twist.angular.z = 0.0

        self.pub.publish(out)
        self._out_count += 1

        if self.debug:
            now = time.time()
            if now - self._last_dbg_wall >= self.debug_period_s:
                self.get_logger().info(f"[dbg] out={self._out_count} {note} v={np.round(v_xyz,3)}")
                self._last_dbg_wall = now


def main():
    rclpy.init()
    n = PerturbationInjector()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()