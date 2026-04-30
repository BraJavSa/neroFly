#!/usr/bin/env python3
import math
import threading
import numpy as np
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, TransformStamped, PointStamped, Vector3
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import Imu, Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from controller import Supervisor

# ── Controlador PID embebido ──────────────────────────────────────────────────
class PIDChannel:
    def __init__(self, kp, ki, kd, e_int_max, tau_d=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.e_int_max = e_int_max
        self.tau_d     = tau_d
        self._e_int     = 0.0
        self._meas_prev = 0.0
        self._d_filt    = 0.0

    def reset(self):
        self._e_int = self._meas_prev = self._d_filt = 0.0

    def update(self, e: float, measurement: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        self._e_int  = np.clip(self._e_int + e * dt, -self.e_int_max, self.e_int_max)
        d_raw        = -(measurement - self._meas_prev) / dt
        alpha        = dt / (self.tau_d + dt)
        self._d_filt = (1.0 - alpha) * self._d_filt + alpha * d_raw
        self._meas_prev = measurement
        return self.kp * e + self.ki * self._e_int + self.kd * self._d_filt

# PID params: (kp, ki, kd, e_int_max)
# x/y ya no usan PID para el freno — el freno es proporcional en world frame
_PID_PARAMS = {
    'z':   (1.0,  0.05, 0.3, 0.3),
    'yaw': (0.8,  0.02, 0.2, 0.3),
}

_KU = np.array([0.8417,  0.8354,  3.966,  9.8524])
_KV = np.array([0.18227, 0.17095, 4.001,  4.7295])

CMD_TIMEOUT_S = 1.0 / 5.0  # 5 Hz → 0.2 s sin mensaje activa freno/cero interno

# ── Gimbal: parámetros (espejados de tf_cam.cpp) ─────────────────────────────
GIMBAL_MIN_DEG      = -90.0          # límite inferior de pitch comandado
GIMBAL_MAX_DEG      =  15.0          # límite superior de pitch comandado
GIMBAL_SPEED_DEG_S  =  22.0          # velocidad angular del servo (°/s)
GIMBAL_DELAY_S      =   0.5          # demora entre recepción y arranque
GIMBAL_TILT_OFFSET  = math.radians(-10.0)   # offset de montaje (no usado en Webots)


class BebopWebotsFullSim(Node):
    def __init__(self):
        super().__init__("neroFlyulator")
        self.declare_parameter('enable_brake', True)
        self._brake_enabled = self.get_parameter('enable_brake').get_parameter_value().bool_value
        if not self._brake_enabled:
            self.get_logger().info("Freno desactivado (enable_brake:=false)")
        else:
            self.get_logger().info("Freno activado (enable_brake:=true)")

        self.robot       = Supervisor()
        self.timestep    = int(self.robot.getBasicTimeStep())
        self.drone_node  = self.robot.getFromDef("DRONE_BODY")
        self.gimbal_node = self.robot.getFromDef("GIMBAL")
        self.camera_node = self.robot.getFromDef("DRONE_CAMERA")

        self.prop_nodes = [self.robot.getFromDef(f"PROP_{n}") for n in ["FR", "FL", "RR", "RL"]]
        self.prop_angle = 0.0

        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.timestep)
            w, h  = self.camera.getWidth(), self.camera.getHeight()
            f_px  = w / (2.0 * math.tan(self.camera.getFov() / 2.0))
            self._cam_info = CameraInfo()
            self._cam_info.header.frame_id  = "camera_gimbal"
            self._cam_info.width, self._cam_info.height = w, h
            self._cam_info.distortion_model = "plumb_bob"
            self._cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self._cam_info.k = [f_px, 0.0, w/2.0,  0.0, f_px, h/2.0,  0.0, 0.0, 1.0]
            self._cam_info.p = [f_px, 0.0, w/2.0, 0.0,  0.0, f_px, h/2.0, 0.0,  0.0, 0.0, 1.0, 0.0]
            self._cam_w, self._cam_h = w, h

        # ── Thread de cámara ──────────────────────────────────────────────────
        self._cam_lock  = threading.Lock()
        self._cam_frame = None
        self._cam_thread = threading.Thread(target=self._cam_publish_loop, daemon=True)
        self._cam_thread.start()

        self.tf_br = TransformBroadcaster(self)

        self.dt       = self.timestep / 1000.0
        self.tick     = 0
        self.z_ground = 0.05
        self.x        = np.array([0.0, 0.0, self.z_ground, 0.0])
        self.xdot     = np.zeros(4)
        self.u        = np.zeros(4)

        self.Ku = np.diag(_KU)
        self.Kv = np.diag(_KV)

        # ── Estado cmd externo ────────────────────────────────────────────────
        # cmd_x / cmd_y guardan el último valor recibido por tópico.
        # Si pasa el timeout, se tratan internamente como 0 sin alterar last_cmd.
        self.last_cmd      = None
        self.last_cmd_time = time.time()

        # ── PIDs internos ─────────────────────────────────────────────────────
        self.pid = {k: PIDChannel(*v) for k, v in _PID_PARAMS.items()}

        # Estado del freno por eje (independientes)
        self.brake_x = False
        self.brake_y = False

        # ── Gimbal: estado de la interpolación (igual que tf_cam.cpp) ─────────
        self.gimbal_current_deg = -10.0  # posición actual del servo (°) — igual que tf_cam.cpp
        self.gimbal_target_deg  = -10.0  # objetivo pedido
        self.gimbal_waiting     = False  # en la demora pre-movimiento
        self.gimbal_moving      = False  # en movimiento activo
        self.gimbal_wait_start  = time.time()

        self.mode             = "IDLE"
        self.last_logged_mode = ""
        self.max_tilt         = math.radians(5.0)

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_xy       = self.create_publisher(PointStamped, "/bebop/position",          10)
        self.pub_z        = self.create_publisher(Float64,      "/bebop/altitude",           10)
        self.pub_imu      = self.create_publisher(Imu,          "/bebop/imu",                10)
        self.pub_odom     = self.create_publisher(Odometry,     "/bebop/odom",               10)
        self.pub_cam      = self.create_publisher(Image,        "/bebop/camera/image_raw",   10)
        self.pub_cam_info = self.create_publisher(CameraInfo,   "/bebop/camera/camera_info", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Twist,   "/bebop/cmd_vel",      self.cmd_cb,        10)
        self.create_subscription(Empty,   "/bebop/takeoff",      self.takeoff_cb,    10)
        self.create_subscription(Empty,   "/bebop/land",         self.land_cb,       10)
        self.create_subscription(Empty,   "/bebop/emergency",    self.emergency_cb,  10)
        # ← GUI real publica Vector3 en /bebop/move_camera (msg.x = pitch en grados)
        self.create_subscription(Vector3, "/bebop/move_camera",  self.move_camera_cb, 10)

        self.create_timer(self.dt, self.timer_cb)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def cmd_cb(self, msg):
        self.last_cmd      = msg
        self.last_cmd_time = time.time()

    def takeoff_cb(self, _):
        if self.mode in ["IDLE", "LANDING"]:
            self.mode = "TAKING_OFF"
            self.u[:] = 0.0
            self.last_cmd_time = time.time()
            for p in self.pid.values():
                p.reset()

    def land_cb(self, _):
        if self.mode in ["FLYING", "TAKING_OFF"]:
            self.mode = "LANDING"
            self.u[:] = 0.0

    def emergency_cb(self, _):
        self.mode = "EMERGENCY"
        self.u[:] = 0.0

    def move_camera_cb(self, msg: Vector3):
        """
        Escucha /bebop/move_camera (Vector3).
        msg.x = pitch deseado en grados (igual que el GUI real y tf_cam.cpp).
        Clampea al rango del Bebop y arranca la secuencia demora→movimiento.
        """
        target = float(np.clip(msg.x, GIMBAL_MIN_DEG, GIMBAL_MAX_DEG))
        if abs(target - self.gimbal_target_deg) < 0.5:
            return  # ignorar si es prácticamente el mismo ángulo
        self.gimbal_target_deg = target
        self.gimbal_waiting    = True
        self.gimbal_moving     = True
        self.gimbal_wait_start = time.time()

    # ── Freno en world frame ──────────────────────────────────────────────────
    _BRAKE_DEAD_STOP = 0.01   # m/s — por debajo de esto se mata la velocidad directo

    def _compute_brake_u(self, raw_x: float, raw_y: float) -> tuple[bool, bool, float, float]:
        """
        Calcula u[0] y u[1] (body frame) para frenar xdot en world frame.

        El error conceptual del enfoque anterior era aplicar el PID sobre xdot
        directamente como si fuera body frame, pero xdot está en world frame.
        La fuerza u pasa por F (rotación yaw) antes de afectar xdot, así que
        hay que proyectar la fuerza deseada al body frame:

            F_world = -Kp * xdot_world   (lo que queremos aplicar)
            u_body  = F^-1 @ F_world     (F^-1 = F^T porque F es ortogonal)
        """
        if not self._brake_enabled:
            return False, False, raw_x, raw_y

        timeout = (time.time() - self.last_cmd_time) > CMD_TIMEOUT_S

        brake_x = timeout or (abs(raw_x) < 1e-4)
        brake_y = timeout or (abs(raw_y) < 1e-4)

        if not brake_x and not brake_y:
            return False, False, raw_x, raw_y

        yaw = self.x[3]
        c, s = math.cos(yaw), math.sin(yaw)

        # Velocidad en world frame que queremos frenar
        vx_w = self.xdot[0] if brake_x else 0.0
        vy_w = self.xdot[1] if brake_y else 0.0

        # Dead-stop: si ya está casi quieto matar directo
        if brake_x and abs(vx_w) < self._BRAKE_DEAD_STOP:
            self.xdot[0] = 0.0
            vx_w = 0.0
        if brake_y and abs(vy_w) < self._BRAKE_DEAD_STOP:
            self.xdot[1] = 0.0
            vy_w = 0.0

        # Fuerza deseada en world frame: proporcional a velocidad (P puro)
        # La integral la eliminamos — era la fuente del windup
        Kp = 1.5
        fx_w = -Kp * vx_w
        fy_w = -Kp * vy_w

        # Proyectar al body frame: u = F^T @ f_world
        # F = [[c,-s],[s,c]]  →  F^T = [[c,s],[-s,c]]
        u_x = c * fx_w + s * fy_w
        u_y = -s * fx_w + c * fy_w

        # Si un eje no frena, devolver el cmd original
        if not brake_x:
            u_x = raw_x
        if not brake_y:
            u_y = raw_y

        u_x = float(np.clip(u_x, -1.0, 1.0))
        u_y = float(np.clip(u_y, -1.0, 1.0))

        return brake_x, brake_y, u_x, u_y

    # ── Gimbal: step de interpolación ─────────────────────────────────────────
    def _step_gimbal(self):
        """
        Avanza la interpolación del servo de cámara (espeja tf_cam.cpp).
        Actualiza self.gimbal_current_deg.
        """
        now = time.time()

        if self.gimbal_waiting:
            if (now - self.gimbal_wait_start) >= GIMBAL_DELAY_S:
                self.gimbal_waiting = False

        if self.gimbal_moving and not self.gimbal_waiting:
            error  = self.gimbal_target_deg - self.gimbal_current_deg
            speed  = GIMBAL_SPEED_DEG_S * self.dt  # °/step
            if abs(error) <= speed:
                self.gimbal_current_deg = self.gimbal_target_deg
                self.gimbal_moving      = False
            else:
                self.gimbal_current_deg += math.copysign(speed, error)

    # ── Loop principal ────────────────────────────────────────────────────────
    def timer_cb(self):
        self.tick += 1

        # Transiciones de modo
        if self.mode == "TAKING_OFF":
            self.x[2] += 0.5 * self.dt
            if self.x[2] >= 1.2:
                self.mode = "FLYING"

        elif self.mode in ["LANDING", "EMERGENCY"]:
            speed        = 0.5 if self.mode == "LANDING" else 1.0
            self.xdot[:] = 0.0
            self.x[2]   -= speed * self.dt
            if self.x[2] <= self.z_ground:
                self.x[2]  = self.z_ground
                self.mode  = "IDLE"
                for p in self.pid.values():
                    p.reset()

        # Determinar u y propagar dinámica
        if self.mode == "FLYING":
            # Obtener valores de cmd (timeout → tratar como 0 internamente)
            timeout = (time.time() - self.last_cmd_time) > CMD_TIMEOUT_S
            if self.last_cmd is not None and not timeout:
                raw_x   = self.last_cmd.linear.x
                raw_y   = self.last_cmd.linear.y
                raw_z   = self.last_cmd.linear.z
                raw_yaw = self.last_cmd.angular.z
            else:
                # timeout: internamente como cero, no se modifica last_cmd
                raw_x = raw_y = raw_z = raw_yaw = 0.0

            # Freno: proyecta la fuerza correctamente world→body frame
            self.brake_x, self.brake_y, self.u[0], self.u[1] = self._compute_brake_u(raw_x, raw_y)

            # Z e YAW: sin freno, uso directo del cmd (o cero si timeout)
            self.u[2] = raw_z
            self.u[3] = raw_yaw

            # Integración Euler explícita
            yaw  = self.x[3]
            c, s = math.cos(yaw), math.sin(yaw)
            F    = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            xddot = F @ (self.Ku @ self.u) - self.Kv @ self.xdot

            xddot = np.clip(xddot, -20.0, 20.0)
            self.x    += self.xdot * self.dt
            self.xdot += xddot    * self.dt
            self.xdot[:2] = np.clip(self.xdot[:2], -15.0, 15.0)
            self.xdot[2]  = np.clip(self.xdot[2],  -5.0,  5.0)
            self.xdot[3]  = np.clip(self.xdot[3],  -6.0,  6.0)
            self.x[3] = math.remainder(self.x[3], 2.0 * math.pi)

        # Seguridad: reset silencioso si NaN/Inf
        if not np.all(np.isfinite(self.xdot)) or not np.all(np.isfinite(self.x)):
            self.xdot[:] = 0.0
            self.x[:3]   = [0.0, 0.0, 1.2]
            for p in self.pid.values():
                p.reset()

        if self.x[2] < self.z_ground:
            self.x[2]    = self.z_ground
            self.xdot[2] = 0.0

        # Visualización Webots
        roll  = float(self.u[1]) * self.max_tilt
        pitch = float(self.u[0]) * self.max_tilt
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, float(self.x[3]))

        if self.drone_node:
            self.drone_node.getField("translation").setSFVec3f(
                [float(self.x[0]), float(self.x[1]), float(self.x[2])])
            angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
            den   = math.sqrt(max(0.0, 1.0 - qw * qw))
            axis  = [qx/den, qy/den, qz/den] if den > 1e-6 else [0, 0, 1]
            self.drone_node.getField("rotation").setSFRotation(axis + [angle])

            if self.mode != "IDLE":
                self.prop_angle += 0.6
                for i, prop in enumerate(self.prop_nodes):
                    if prop:
                        prop.getField("rotation").setSFRotation(
                            [0, 0, 1, self.prop_angle * (1 if i % 2 == 0 else -1)])

        # ── Gimbal: interpolar pitch y aplicar estabilización ─────────────────
        self._step_gimbal()

        if self.gimbal_node:
            # En Webots pitch positivo = nariz sube. La convención Bebop es
            # negativo = cámara baja, por lo tanto hay que negar para que
            # -90° (abajo) produzca una rotación positiva en Webots.
            gimbal_pitch_rad = math.radians(-self.gimbal_current_deg)
            g_roll  = -roll
            g_pitch = -pitch + gimbal_pitch_rad
            gqx, gqy, gqz, gqw = quaternion_from_euler(g_roll, g_pitch, 0.0)
            g_angle = 2 * math.acos(max(-1.0, min(1.0, gqw)))
            g_den   = math.sqrt(max(0.0, 1.0 - gqw * gqw))
            g_axis  = ([gqx/g_den, gqy/g_den, gqz/g_den]
                       if g_den > 1e-6 else [0, 1, 0])
            self.gimbal_node.getField("rotation").setSFRotation(g_axis + [g_angle])

        if self.tick % 2 == 0:
            if self.camera:
                raw = self.camera.getImage()
                if raw:
                    with self._cam_lock:
                        self._cam_frame = (raw, self.get_clock().now().to_msg())
            self.publish_imu(qx, qy, qz, qw)
            self.publish_tfs(qx, qy, qz, qw)
            self.publish_odom(qx, qy, qz, qw)

        if self.tick % 4 == 0: self.publish_xy()
        if self.tick % 6 == 0: self.publish_z()
        if self.robot.step(self.timestep) == -1: rclpy.shutdown()

    # ── Publicadores ──────────────────────────────────────────────────────────
    def publish_tfs(self, qx, qy, qz, qw):
        now = self.get_clock().now().to_msg()
        t   = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "odom", "base_link"
        t.transform.translation.x = float(self.x[0])
        t.transform.translation.y = float(self.x[1])
        t.transform.translation.z = float(self.x[2])
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_br.sendTransform(t)

    def publish_imu(self, qx, qy, qz, qw):
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.orientation.x, msg.orientation.y = qx, qy
        msg.orientation.z, msg.orientation.w = qz, qw
        self.pub_imu.publish(msg)

    def publish_xy(self):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x, msg.point.y = float(self.x[0]), float(self.x[1])
        self.pub_xy.publish(msg)

    def publish_z(self):
        msg      = Float64()
        msg.data = float(self.x[2])
        self.pub_z.publish(msg)

    def publish_odom(self, qx, qy, qz, qw):
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id  = "base_link"
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = float(self.x[2])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x  = float(self.xdot[0])
        msg.twist.twist.linear.y  = float(self.xdot[1])
        msg.twist.twist.linear.z  = float(self.xdot[2])
        msg.twist.twist.angular.z = float(self.xdot[3])
        self.pub_odom.publish(msg)

    def _cam_publish_loop(self):
        while True:
            with self._cam_lock:
                frame = self._cam_frame
                self._cam_frame = None

            if frame is not None:
                raw, stamp = frame
                w, h = self._cam_w, self._cam_h
                img_bgr = np.frombuffer(raw, np.uint8).reshape((h, w, 4))[:, :, :3]
                self._cam_info.header.stamp = stamp
                self.pub_cam_info.publish(self._cam_info)
                msg = Image()
                msg.header.stamp    = stamp
                msg.header.frame_id = "camera_gimbal"
                msg.height, msg.width = h, w
                msg.encoding = "bgr8"
                msg.step     = w * 3
                msg.data     = img_bgr.tobytes()
                self.pub_cam.publish(msg)
            else:
                time.sleep(0.002)


def main():
    rclpy.init()
    rclpy.spin(BebopWebotsFullSim())
    rclpy.shutdown()

if __name__ == "__main__":
    main()