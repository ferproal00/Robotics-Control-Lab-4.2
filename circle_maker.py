#!/usr/bin/env python3
import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped
from tf2_ros import Buffer, TransformListener
from pynput import keyboard


class RobotState(Enum):
    RUNNING = 1
    PAUSED = 2
    HOME = 3


class CircleMakerXArmLite6(Node):
    """
    Circle trajectory generator for xArm Lite 6.

    - Always publishes desired Cartesian position as PointStamped to desired_topic (default: /desired_position).
    - Optionally publishes TwistStamped commands to MoveIt Servo (default disabled).
      Use publish_twist:=true ONLY if you want circle_maker to directly drive the robot.
      For the Nezih pipeline: keep publish_twist:=false and let position_controller drive.
    """

    def __init__(self):
        super().__init__("circle_maker_xarm_lite6")

        # ---------------------------
        # State machine
        # ---------------------------
        self.robot_state = RobotState.RUNNING

        # ---------------------------
        # Parameters (trajectory)
        # ---------------------------
        self.radius = float(self.declare_parameter("radius", 0.06).value)
        self.frequency = float(self.declare_parameter("frequency", 0.06).value)  # Hz
        self.plane = str(self.declare_parameter("plane", "xy").value).lower().strip()
        self.hold_z = bool(self.declare_parameter("hold_z", True).value)

        # Soft start (seconds)
        self.soft_start_s = float(self.declare_parameter("soft_start_s", 2.0).value)

        # Desired position topic
        self.desired_topic = str(self.declare_parameter("desired_topic", "/desired_position").value)

        # ---------------------------
        # Optional direct servo (Twist)
        # ---------------------------
        self.publish_twist = bool(self.declare_parameter("publish_twist", False).value)
        self.output_topic = str(self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds").value)

        # PD gains (ONLY used if publish_twist=True)
        kp = self.declare_parameter("kp", [2.5, 2.5, 2.5]).value
        kd = self.declare_parameter("kd", [0.6, 0.6, 0.6]).value
        self.kp = np.array([float(kp[0]), float(kp[1]), float(kp[2])], dtype=float)
        self.kd = np.array([float(kd[0]), float(kd[1]), float(kd[2])], dtype=float)

        # Deadband & saturation (ONLY used if publish_twist=True)
        self.deadband = float(self.declare_parameter("deadband", 0.002).value)  # meters
        self.max_speed = float(self.declare_parameter("max_speed", 0.12).value)  # m/s (magnitude clamp)

        # Home position (used by keyboard 'h')
        home = self.declare_parameter("home_position", [0.227, 0.00, 0.468]).value
        try:
            self.home_position = np.array([float(home[0]), float(home[1]), float(home[2])], dtype=float)
        except Exception:
            self.home_position = np.array([0.227, 0.00, 0.468], dtype=float)

        # ---------------------------
        # Publishers
        # ---------------------------
        self.desired_pub = self.create_publisher(PointStamped, self.desired_topic, 10)
        self.servo_pub = self.create_publisher(TwistStamped, self.output_topic, 10)

        # ---------------------------
        # TF for current EE pose
        # ---------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Center initialized from current EE pose when TF becomes available
        self.center = None

        # Timing / derivative memory (ONLY used if publish_twist=True)
        self.start_time = self.get_clock().now()
        self.prev_error = np.zeros(3)
        self.prev_time = self.get_clock().now()

        # Log throttles
        self.last_info_time = self.get_clock().now()
        self.last_tf_warn_time = self.get_clock().now()

        # Keyboard controls
        self._start_keyboard()

        # Control loop (50 Hz)
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info(
            "✅ circle_maker running\n"
            f"   desired_topic={self.desired_topic}\n"
            f"   publish_twist={self.publish_twist} (output_topic={self.output_topic})\n"
            "   Keys: 'p' pause/resume, 'h' home"
        )

    # ---------------------------
    # Keyboard controls
    # ---------------------------
    def _start_keyboard(self):
        def on_press(key):
            if hasattr(key, "char") and key.char == "p":
                if self.robot_state == RobotState.RUNNING:
                    self.robot_state = RobotState.PAUSED
                    if self.publish_twist:
                        self._publish_zero()
                    self.get_logger().warn("Paused.")
                elif self.robot_state == RobotState.PAUSED:
                    self.robot_state = RobotState.RUNNING
                    self.prev_time = self.get_clock().now()
                    self.prev_error = np.zeros(3)
                    self.get_logger().info("Resumed.")

            if hasattr(key, "char") and key.char == "h":
                self.robot_state = RobotState.HOME
                self.get_logger().info("Going HOME...")

        self.keyboard_listener = keyboard.Listener(on_press=on_press)
        self.keyboard_listener.start()

    # ---------------------------
    # TF pose read (position only)
    # ---------------------------
    def _read_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ],
                dtype=float,
            )
        except Exception as e:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time).nanoseconds > 2e9:
                self.get_logger().warn(f"TF not ready: {e}")
                self.last_tf_warn_time = now
            return None

    # ---------------------------
    # Circle target generator
    # ---------------------------
    def _circle_target(self, t_sec: float):
        cx, cy, cz = self.center
        w = 2.0 * math.pi * self.frequency

        ramp = 1.0
        if self.soft_start_s > 1e-6:
            ramp = min(max(t_sec / self.soft_start_s, 0.0), 1.0)

        a = ramp * self.radius * math.cos(w * t_sec)
        b = ramp * self.radius * math.sin(w * t_sec)

        if self.plane == "xy":
            x = cx + a
            y = cy + b
            z = cz if self.hold_z else cz
        elif self.plane == "xz":
            x = cx + a
            y = cy
            z = cz if self.hold_z else (cz + b)
        elif self.plane == "yz":
            x = cx
            y = cy + a
            z = cz if self.hold_z else (cz + b)
        else:
            x, y, z = cx + a, cy + b, cz

        return np.array([x, y, z], dtype=float)

    # ---------------------------
    # Publish desired PointStamped
    # ---------------------------
    def _publish_desired(self, target_pos: np.ndarray):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.point.x = float(target_pos[0])
        msg.point.y = float(target_pos[1])
        msg.point.z = float(target_pos[2])
        self.desired_pub.publish(msg)

    # ---------------------------
    # Publish TwistStamped (optional)
    # ---------------------------
    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x = float(v_xyz[0])
        cmd.twist.linear.y = float(v_xyz[1])
        cmd.twist.linear.z = float(v_xyz[2])
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.servo_pub.publish(cmd)

    def _publish_zero(self):
        self._publish_twist(np.zeros(3))

    # ---------------------------
    # PD step (only if publish_twist=True)
    # ---------------------------
    def _servo_to(self, target_pos: np.ndarray):
        current = self._read_pose()
        if current is None:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6

        error = target_pos - current
        error = np.where(np.abs(error) < self.deadband, 0.0, error)
        d_error = (error - self.prev_error) / dt

        v = self.kp * error + self.kd * d_error

        # magnitude clamp
        speed = float(np.linalg.norm(v))
        if speed > self.max_speed and speed > 1e-9:
            v = v * (self.max_speed / speed)

        self._publish_twist(v)

        self.prev_error = error
        self.prev_time = now

        # log @ 1 Hz
        if (now - self.last_info_time).nanoseconds > 1e9:
            self.get_logger().info(
                f"cur={current.round(3)} tgt={target_pos.round(3)} e={error.round(3)} |v|={np.linalg.norm(v):.3f}"
            )
            self.last_info_time = now

    # ---------------------------
    # Main loop / state machine
    # ---------------------------
    def _loop(self):
        # Initialize circle center from current EE pose
        if self.center is None:
            p = self._read_pose()
            if p is None:
                return
            self.center = p.copy()
            self.start_time = self.get_clock().now()
            self.prev_time = self.get_clock().now()
            self.prev_error = np.zeros(3)
            self.get_logger().info(f"✅ Center set to {self.center.round(3)}")
            return

        if self.robot_state == RobotState.PAUSED:
            # Still publish desired so controller can keep tracking a fixed point if desired
            # (we publish the current point on the circle for continuity)
            t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            target = self._circle_target(t)
            self._publish_desired(target)
            return

        if self.robot_state == RobotState.HOME:
            current = self._read_pose()
            if current is None:
                return

            # Publish desired home position
            self._publish_desired(self.home_position)

            if self.publish_twist:
                error = self.home_position - current
                dist = float(np.linalg.norm(error))
                if dist < 0.005:
                    self._publish_zero()
                    self.robot_state = RobotState.RUNNING
                    self.center = current.copy()
                    self.start_time = self.get_clock().now()
                    self.prev_time = self.get_clock().now()
                    self.prev_error = np.zeros(3)
                    self.get_logger().info("✅ Home reached. Circle re-centered and resumed.")
                    return

                # Simple homing velocity (directional)
                direction = np.where(np.abs(error) > 1e-4, np.sign(error), 0.0)
                v = direction * min(0.08, self.max_speed)
                self._publish_twist(v)
            return

        # RUNNING: generate and publish desired circle target
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        target = self._circle_target(t)
        self._publish_desired(target)

        # Optional direct servo control
        if self.publish_twist:
            self._servo_to(target)


def main(args=None):
    rclpy.init(args=args)
    node = CircleMakerXArmLite6()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()