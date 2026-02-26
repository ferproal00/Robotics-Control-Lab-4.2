#!/usr/bin/env python3
import os
import csv
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
import tf2_ros


class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')

        # -------- PARAMETERS --------
        self.declare_parameter('kp', [5.5, 5.5, 5.5])
        self.declare_parameter('kd', [0.15, 0.15, 0.15])
        self.declare_parameter('deadband', 0.003)
        self.declare_parameter('max_speed', 0.18)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('log_path', '/tmp/position_controller.csv')
        self.declare_parameter('save_after_samples', 3500)
        self.declare_parameter('flush_every_rows', 100)

        self.kp = np.array(self.get_parameter('kp').value, dtype=float)
        self.kd = np.array(self.get_parameter('kd').value, dtype=float)
        self.deadband = float(self.get_parameter('deadband').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.dt = float(self.get_parameter('dt').value)
        self.log_path = str(self.get_parameter('log_path').value)
        self.save_after = int(self.get_parameter('save_after_samples').value)
        self.flush_every = int(self.get_parameter('flush_every_rows').value)

        # -------- ROS --------
        self.pub = self.create_publisher(TwistStamped, '/controller/delta_twist_cmds', 10)
        self.sub = self.create_subscription(Point, '/desired_position', self.desired_callback, 10)

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- STATE --------
        self.desired = None
        self.prev_err = np.zeros(3)
        self.start_time = time.time()

        self.samples = 0
        self.saved_once = False

        # -------- LOG FILE (STREAMING) --------
        self.csv_file = None
        self.csv_writer = None
        self._open_log_file()

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f"✅ position_controller\n"
            f"  kp={self.kp}, kd={self.kd}\n"
            f"  deadband={self.deadband}, max_speed={self.max_speed}\n"
            f"  dt={self.dt}\n"
            f"  log_path={self.log_path}\n"
            f"  save_after_samples={self.save_after}, flush_every_rows={self.flush_every}"
        )

    def _open_log_file(self):
        # Crear carpeta y archivo desde el inicio (para que “aparezca” inmediato)
        try:
            folder = os.path.dirname(self.log_path)
            if folder:
                os.makedirs(folder, exist_ok=True)

            self.csv_file = open(self.log_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)

            header = [
                "t",
                "pos_x","pos_y","pos_z",
                "des_x","des_y","des_z",
                "err_x","err_y","err_z",
                "vx","vy","vz",
                "v_norm",
                "sat"
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())

            self.get_logger().info(f"📝 CSV creado y listo: {self.log_path}")
        except Exception as e:
            self.get_logger().error(f"❌ No pude abrir CSV en '{self.log_path}': {e}")
            self.csv_file = None
            self.csv_writer = None

    def destroy_node(self):
        # Cerrar archivo al apagar el nodo
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
        except:
            pass
        super().destroy_node()

    def desired_callback(self, msg):
        self.desired = np.array([msg.x, msg.y, msg.z], dtype=float)

    def get_current_position(self):
        try:
            tf = self.tf_buffer.lookup_transform('link_base', 'link_eef', rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z], dtype=float)
        except:
            return None

    def step(self):
        if self.desired is None:
            return

        pos = self.get_current_position()
        if pos is None:
            return

        err = self.desired - pos

        # Deadband
        err = np.where(np.abs(err) < self.deadband, 0.0, err)

        # Derivative
        d_err = (err - self.prev_err) / self.dt
        self.prev_err = err.copy()

        # PD
        v = self.kp * err + self.kd * d_err

        # Saturation
        sat = 0
        for i in range(3):
            if v[i] > self.max_speed:
                v[i] = self.max_speed
                sat = 1
            elif v[i] < -self.max_speed:
                v[i] = -self.max_speed
                sat = 1

        # Publish
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        self.pub.publish(msg)

        # Log streaming
        self.samples += 1
        if self.csv_writer is not None:
            t = time.time() - self.start_time
            row = [
                t,
                pos[0], pos[1], pos[2],
                self.desired[0], self.desired[1], self.desired[2],
                err[0], err[1], err[2],
                v[0], v[1], v[2],
                float(np.linalg.norm(v)),
                sat
            ]
            self.csv_writer.writerow(row)

            # Flush periódico (para que SIEMPRE se escriba)
            if (self.samples % self.flush_every) == 0:
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())

            # “Marcador” cuando llega a N muestras (pero NO detiene control)
            if (not self.saved_once) and (self.samples >= self.save_after):
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                self.saved_once = True
                self.get_logger().info(f"✅ Alcancé {self.save_after} muestras. CSV ya contiene ese bloque (y sigue creciendo).")

        # Print ligero
        if (self.samples % 50) == 0:
            self.get_logger().info(f"samples={self.samples} pos={np.round(pos,3)} des={np.round(self.desired,3)} err={np.round(err,3)}")


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()