#!/usr/bin/env python3
import math
import os
import json
import threading
import time
import casadi as ca
import numpy as np
import rclpy
from rclpy.node import Node
from collections import deque
from scipy.signal import savgol_filter
from geometry_msgs.msg import Twist, TransformStamped, PointStamped, Vector3
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import Imu, Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from controller import Supervisor

SG_WINDOW        = 11
SG_POLYORD       = 3
GIMBAL_MIN_DEG   = -90.0
GIMBAL_MAX_DEG   = 15.0
GIMBAL_SPEED_DEG_S = 22.0
GIMBAL_DELAY_S   = 0.5
CMD_TIMEOUT_S    = 1.0 / 5.0
TILT_ALPHA       = 0.08


class OnlineSavgolFilter:

    def __init__(self, window_length: int = SG_WINDOW, polyorder: int = SG_POLYORD, n_signals: int = 4):
        assert window_length % 2 == 1, 'window_length debe ser impar'
        self.wl = window_length
        self.po = polyorder
        self.n  = n_signals
        self._buf = deque([np.zeros(n_signals)] * window_length, maxlen=window_length)

    def update(self, v_raw: np.ndarray) -> np.ndarray:
        self._buf.append(v_raw.copy())
        arr = np.array(self._buf)
        filtered = savgol_filter(arr, self.wl, self.po, axis=0)
        return filtered[-1]

    def get_window_filtered(self) -> np.ndarray:
        arr = np.array(self._buf)
        return savgol_filter(arr, self.wl, self.po, axis=0)

    def reset(self, v_init: np.ndarray | None = None):
        v0 = v_init if v_init is not None else np.zeros(self.n)
        self._buf = deque([v0.copy()] * self.wl, maxlen=self.wl)


def _load_mlp_model(package_name: str = 'neroFly'):
    from ament_index_python.packages import get_package_share_directory
    package_share_dir = get_package_share_directory(package_name)
    so_path   = os.path.join(package_share_dir, 'models', 'drone_model_v2.so')
    json_path = os.path.join(package_share_dir, 'models', 'drone_metadata_v2.json')
    if not os.path.exists(so_path):
        raise FileNotFoundError(f'No se encuentra el modelo: {so_path}')
    if not os.path.exists(json_path):
        raise FileNotFoundError(f'No se encuentra el metadata: {json_path}')
    with open(json_path, 'r') as f:
        meta = json.load(f)
    f_mlp = ca.external('f_mlp', so_path)
    return f_mlp, int(meta['window_size']), float(meta['dt'])


class BebopWebotsMLPSim(Node):

    def __init__(self):
        super().__init__('neroFlyulator')
        self.f_mlp, self.window_size, self.model_dt = _load_mlp_model()
        self.get_logger().info(
            f'MLP v2 cargado | window_size={self.window_size} | dt_model={self.model_dt:.4f}s | '
            f'input=v_now(4)+u_hist({self.window_size}x4)+u_now(4)={4*(self.window_size+2)} features'
        )

        self.robot      = Supervisor()
        self.timestep   = int(self.robot.getBasicTimeStep())
        self.dt         = self.timestep / 1000.0
        self.drone_node  = self.robot.getFromDef('DRONE_BODY')
        self.gimbal_node = self.robot.getFromDef('GIMBAL')
        self.camera_node = self.robot.getFromDef('DRONE_CAMERA')
        self.prop_nodes  = [self.robot.getFromDef(f'PROP_{n}') for n in ['FR', 'FL', 'RR', 'RL']]
        self.prop_angle  = 0.0

        self.camera = self.robot.getDevice('camera')
        if self.camera:
            self.camera.enable(self.timestep)
            w, h  = self.camera.getWidth(), self.camera.getHeight()
            f_px  = w / (2.0 * math.tan(self.camera.getFov() / 2.0))
            self._cam_info = CameraInfo()
            self._cam_info.header.frame_id = 'camera_gimbal'
            self._cam_info.width  = w
            self._cam_info.height = h
            self._cam_info.distortion_model = 'plumb_bob'
            self._cam_info.d = [0.0] * 5
            self._cam_info.k = [f_px, 0.0, w/2.0, 0.0, f_px, h/2.0, 0.0, 0.0, 1.0]
            self._cam_info.p = [f_px, 0.0, w/2.0, 0.0, 0.0, f_px, h/2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self._cam_w, self._cam_h = w, h

        self._cam_lock   = threading.Lock()
        self._cam_frame  = None
        self._cam_thread = threading.Thread(target=self._cam_publish_loop, daemon=True)
        self._cam_thread.start()

        self.tf_br = TransformBroadcaster(self)
        self.tick  = 0

        self.z_ground  = 0.05
        self.max_tilt  = math.radians(5.0)
        self.x         = np.array([0.0, 0.0, self.z_ground, 0.0])
        self.v_body_raw = np.zeros(4)
        self.xdot       = np.zeros(4)

        self.sg_filter  = OnlineSavgolFilter(window_length=SG_WINDOW, polyorder=SG_POLYORD, n_signals=4)

        # v2: ya no necesitamos v_history para el input de la red,
        # pero lo conservamos por si se usa en warm-up o logging.
        # El buffer relevante para la inferencia es solo u_history.
        self.u_history  = deque([np.zeros(4)] * self.window_size, maxlen=self.window_size)

        self.u_cmd      = np.zeros(4)
        self.vis_roll   = 0.0
        self.vis_pitch  = 0.0

        self.warmup_steps = 0
        self.warmup_done  = False
        self.last_cmd      = None
        self.last_cmd_time = time.time()

        self.mode             = 'IDLE'
        self.last_logged_mode = ''

        self.gimbal_pitch_current_deg = -10.0
        self.gimbal_pitch_target_deg  = -10.0
        self.gimbal_waiting      = False
        self.gimbal_moving_pitch = False
        self.gimbal_wait_start   = time.time()

        self.pub_xy       = self.create_publisher(PointStamped, '/bebop/position',           10)
        self.pub_z        = self.create_publisher(Float64,       '/bebop/altitude',           10)
        self.pub_imu      = self.create_publisher(Imu,           '/bebop/imu',               10)
        self.pub_odom     = self.create_publisher(Odometry,      '/bebop/odom',              10)
        self.pub_cam      = self.create_publisher(Image,         '/bebop/camera/image_raw',  10)
        self.pub_cam_info = self.create_publisher(CameraInfo,    '/bebop/camera/camera_info',10)

        self.create_subscription(Twist,   '/bebop/cmd_vel',    self.cmd_cb,        10)
        self.create_subscription(Empty,   '/bebop/takeoff',    self.takeoff_cb,    10)
        self.create_subscription(Empty,   '/bebop/land',       self.land_cb,       10)
        self.create_subscription(Empty,   '/bebop/emergency',  self.emergency_cb,  10)
        self.create_subscription(Vector3, '/bebop/move_camera',self.move_camera_cb,10)
        self.create_timer(self.dt, self.timer_cb)

    # ─── callbacks ────────────────────────────────────────────────────────────

    def cmd_cb(self, msg: Twist):
        self.last_cmd      = msg
        self.last_cmd_time = time.time()

    def takeoff_cb(self, _):
        if self.mode in ['IDLE', 'LANDING']:
            self.mode         = 'TAKING_OFF'
            self.u_cmd[:]     = 0.0
            self.warmup_steps = 0
            self.warmup_done  = False
            self._reset_history()
            self.last_cmd_time = time.time()
            self.get_logger().info('Takeoff iniciado')

    def land_cb(self, _):
        if self.mode in ['FLYING', 'TAKING_OFF']:
            self.mode     = 'LANDING'
            self.u_cmd[:] = 0.0
            self.get_logger().info('Landing iniciado')

    def emergency_cb(self, _):
        self.mode     = 'EMERGENCY'
        self.u_cmd[:] = 0.0
        self.get_logger().warn('¡EMERGENCIA! Motores cortados')

    def move_camera_cb(self, msg: Vector3):
        target = float(np.clip(msg.x, GIMBAL_MIN_DEG, GIMBAL_MAX_DEG))
        if abs(target - self.gimbal_pitch_target_deg) < 0.5:
            return
        self.gimbal_pitch_target_deg = target
        self.gimbal_moving_pitch     = True
        self.gimbal_waiting          = True
        self.gimbal_wait_start       = time.time()

    # ─── historia y filtro ────────────────────────────────────────────────────

    def _reset_history(self):
        self.sg_filter.reset(self.v_body_raw)
        self.u_history = deque([np.zeros(4)] * self.window_size, maxlen=self.window_size)
        self.v_body_raw[:] = 0.0
        self.xdot[:]       = 0.0

    def _update_velocity_buffer(self, v_raw: np.ndarray) -> np.ndarray:
        """Aplica Savitzky-Golay online y devuelve v suavizada en t."""
        return self.sg_filter.update(v_raw)

    # ─── inferencia MLP v2 ────────────────────────────────────────────────────

    def _build_mlp_input(self, v_smooth_now: np.ndarray, u_actual: np.ndarray) -> np.ndarray:
        """
        Formato v2: [ v_now(4) | u_hist(W×4) | u_now(4) ]
        v_smooth_now : velocidad suavizada en el instante actual (4,)
        u_actual     : comando de control a aplicar   (4,)
        """
        u_win   = np.array(self.u_history)                        # (W, 4)
        x_input = np.hstack((
            v_smooth_now.flatten(),   # 4
            u_win.flatten(),          # W*4
            u_actual.flatten()        # 4
        )).reshape(1, -1)             # total: 4*(W+2)
        return x_input

    def _mlp_derivative(self, v_smooth_now: np.ndarray, u_actual: np.ndarray) -> np.ndarray:
        x_input = self._build_mlp_input(v_smooth_now, u_actual)
        return self.f_mlp(x_input).full().flatten()

    def _euler_step(self, v_smooth: np.ndarray, u_actual: np.ndarray, dt: float) -> np.ndarray:
        dvdt = self._mlp_derivative(v_smooth, u_actual)
        return v_smooth + dt * dvdt

    # ─── cinemática inercial ──────────────────────────────────────────────────

    def _integrate_inertial(self, v_body: np.ndarray, dt: float):
        vx_b, vy_b, vz_b, vpsi = v_body
        psi    = self.x[3]
        x_dot  = vx_b * math.cos(psi) - vy_b * math.sin(psi)
        y_dot  = vx_b * math.sin(psi) + vy_b * math.cos(psi)
        self.x[0] += x_dot  * dt
        self.x[1] += y_dot  * dt
        self.x[2] += vz_b   * dt
        self.x[3] += vpsi   * dt
        self.x[3]  = math.remainder(self.x[3], 2.0 * math.pi)
        self.xdot  = np.array([x_dot, y_dot, vz_b, vpsi])

    # ─── gimbal ───────────────────────────────────────────────────────────────

    def _step_gimbal(self):
        now = time.time()
        if self.gimbal_waiting:
            if now - self.gimbal_wait_start >= GIMBAL_DELAY_S:
                self.gimbal_waiting = False
        if self.gimbal_moving_pitch and not self.gimbal_waiting:
            speed = GIMBAL_SPEED_DEG_S * self.dt
            error = self.gimbal_pitch_target_deg - self.gimbal_pitch_current_deg
            if abs(error) <= speed:
                self.gimbal_pitch_current_deg = self.gimbal_pitch_target_deg
                self.gimbal_moving_pitch      = False
            else:
                self.gimbal_pitch_current_deg += math.copysign(speed, error)

    # ─── timer principal ──────────────────────────────────────────────────────

    def timer_cb(self):
        self.tick += 1
        dt = self.dt

        # Actualizar comando
        timeout = time.time() - self.last_cmd_time > CMD_TIMEOUT_S
        if self.last_cmd is not None and not timeout:
            self.u_cmd[0] = float(np.clip(self.last_cmd.linear.x,  -1.0, 1.0))
            self.u_cmd[1] = float(np.clip(self.last_cmd.linear.y,  -1.0, 1.0))
            self.u_cmd[2] = float(np.clip(self.last_cmd.linear.z,  -1.0, 1.0))
            self.u_cmd[3] = float(np.clip(self.last_cmd.angular.z, -1.0, 1.0))
        elif self.mode == 'FLYING':
            self.u_cmd[:] = 0.0

        # ── Máquina de estados ────────────────────────────────────────────────
        if self.mode == 'TAKING_OFF':
            self.v_body_raw = np.array([0.0, 0.0, 0.5, 0.0])
            self.x[2]      += 0.5 * dt
            self._update_velocity_buffer(self.v_body_raw)
            self.u_history.append(np.zeros(4))
            if self.x[2] >= 1.2:
                self.mode = 'FLYING'
                self.get_logger().info('En vuelo — iniciando warm-up MLP')

        elif self.mode in ['LANDING', 'EMERGENCY']:
            speed           = 0.5 if self.mode == 'LANDING' else 1.0
            self.v_body_raw = np.array([0.0, 0.0, -speed, 0.0])
            self.x[2]      -= speed * dt
            self.xdot[:]    = 0.0
            self._update_velocity_buffer(self.v_body_raw)
            self.u_history.append(self.u_cmd.copy())
            if self.x[2] <= self.z_ground:
                self.x[2] = self.z_ground
                self.mode  = 'IDLE'
                self._reset_history()
                self.get_logger().info('En tierra')

        elif self.mode == 'FLYING':
            if not self.warmup_done:
                # Warm-up: acumular historial de U sin inferencia
                self._update_velocity_buffer(self.v_body_raw)
                self.u_history.append(self.u_cmd.copy())
                self._integrate_inertial(self.v_body_raw, dt)
                self.warmup_steps += 1
                if self.warmup_steps >= self.window_size:
                    self.warmup_done = True
                    self.get_logger().info(
                        f'Warm-up completado ({self.window_size} pasos) — MLP v2 activo'
                    )
            else:
                # Inferencia MLP v2
                v_smooth = self._update_velocity_buffer(self.v_body_raw)
                self.u_history.append(self.u_cmd.copy())

                # Euler: f(v_now, u_hist, u_now)
                v_new           = self._euler_step(v_smooth, self.u_cmd.copy(), dt)
                self.v_body_raw = v_new.copy()
                self._integrate_inertial(self.v_body_raw, dt)

                if self.x[2] < self.z_ground:
                    self.x[2]          = self.z_ground
                    self.v_body_raw[2] = 0.0

        # ── Guardia NaN/Inf ───────────────────────────────────────────────────
        if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.v_body_raw)):
            self.get_logger().error('Estado NaN/Inf detectado — reset')
            self.v_body_raw[:] = 0.0
            self.x[:3]         = [0.0, 0.0, 1.2]
            self._reset_history()
            self.warmup_steps  = 0
            self.warmup_done   = False

        if self.x[2] < self.z_ground:
            self.x[2]          = self.z_ground
            self.v_body_raw[2] = 0.0

        # ── Visual (tilt + props) ─────────────────────────────────────────────
        roll_target  = float(self.u_cmd[1]) * self.max_tilt
        pitch_target = float(self.u_cmd[0]) * self.max_tilt
        self.vis_roll  += TILT_ALPHA * (roll_target  - self.vis_roll)
        self.vis_pitch += TILT_ALPHA * (pitch_target - self.vis_pitch)
        roll, pitch = self.vis_roll, self.vis_pitch
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, float(self.x[3]))

        if self.drone_node:
            self.drone_node.getField('translation').setSFVec3f(
                [float(self.x[0]), float(self.x[1]), float(self.x[2])]
            )
            angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
            den   = math.sqrt(max(0.0, 1.0 - qw * qw))
            axis  = [qx/den, qy/den, qz/den] if den > 1e-6 else [0, 0, 1]
            self.drone_node.getField('rotation').setSFRotation(axis + [angle])
            if self.mode != 'IDLE':
                self.prop_angle += 0.6
                for i, prop in enumerate(self.prop_nodes):
                    if prop:
                        prop.getField('rotation').setSFRotation(
                            [0, 0, 1, self.prop_angle * (1 if i % 2 == 0 else -1)]
                        )

        self._step_gimbal()
        if self.gimbal_node:
            gimbal_pitch_rad = math.radians(-self.gimbal_pitch_current_deg)
            g_roll  = -self.vis_roll
            g_pitch = -self.vis_pitch + gimbal_pitch_rad
            gqx, gqy, gqz, gqw = quaternion_from_euler(g_roll, g_pitch, 0.0)
            g_angle = 2 * math.acos(max(-1.0, min(1.0, gqw)))
            g_den   = math.sqrt(max(0.0, 1.0 - gqw * gqw))
            g_axis  = [gqx/g_den, gqy/g_den, gqz/g_den] if g_den > 1e-6 else [0, 1, 0]
            self.gimbal_node.getField('rotation').setSFRotation(g_axis + [g_angle])

        # ── Cámara ────────────────────────────────────────────────────────────
        if self.camera:
            raw = self.camera.getImage()
            if raw:
                with self._cam_lock:
                    self._cam_frame = (raw, self.get_clock().now().to_msg())

        # ── Publicaciones ─────────────────────────────────────────────────────
        self.publish_imu(qx, qy, qz, qw)
        self.publish_tfs(qx, qy, qz, qw)
        if self.tick % 2 == 0:
            self.publish_xy()
        if self.tick % 3 == 0:
            self.publish_z()
        if self.tick % 6 == 0:
            self.publish_odom(qx, qy, qz, qw)

        if self.robot.step(self.timestep) == -1:
            rclpy.shutdown()

    # ─── publicadores ─────────────────────────────────────────────────────────

    def publish_tfs(self, qx, qy, qz, qw):
        now    = self.get_clock().now().to_msg()
        t_base = TransformStamped()
        t_base.header.stamp    = now
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id  = 'base_link'
        t_base.transform.translation.x = float(self.x[0])
        t_base.transform.translation.y = float(self.x[1])
        t_base.transform.translation.z = float(self.x[2])
        t_base.transform.rotation.x = qx
        t_base.transform.rotation.y = qy
        t_base.transform.rotation.z = qz
        t_base.transform.rotation.w = qw
        gimbal_pitch_rad = math.radians(-self.gimbal_pitch_current_deg)
        cgx, cgy, cgz, cgw = quaternion_from_euler(-self.vis_roll, -self.vis_pitch + gimbal_pitch_rad, 0.0)
        t_cam = TransformStamped()
        t_cam.header.stamp    = now
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id  = 'camera_gimbal'
        t_cam.transform.translation.x = 0.0
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 0.0
        t_cam.transform.rotation.x = cgx
        t_cam.transform.rotation.y = cgy
        t_cam.transform.rotation.z = cgz
        t_cam.transform.rotation.w = cgw
        self.tf_br.sendTransform([t_base, t_cam])

    def publish_imu(self, qx, qy, qz, qw):
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.orientation.x   = qx
        msg.orientation.y   = qy
        msg.orientation.z   = qz
        msg.orientation.w   = qw
        self.pub_imu.publish(msg)

    def publish_xy(self):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.point.x = float(self.x[0])
        msg.point.y = float(self.x[1])
        self.pub_xy.publish(msg)

    def publish_z(self):
        msg      = Float64()
        msg.data = float(self.x[2])
        self.pub_z.publish(msg)

    def publish_odom(self, qx, qy, qz, qw):
        msg = Odometry()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = 'odom'
        msg.child_frame_id    = 'base_link'
        msg.pose.pose.position.x    = float(self.x[0])
        msg.pose.pose.position.y    = float(self.x[1])
        msg.pose.pose.position.z    = float(self.x[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x  = float(self.xdot[0])
        msg.twist.twist.linear.y  = float(self.xdot[1])
        msg.twist.twist.linear.z  = float(self.xdot[2])
        msg.twist.twist.angular.z = float(self.xdot[3])
        self.pub_odom.publish(msg)

    def _cam_publish_loop(self):
        while True:
            with self._cam_lock:
                frame          = self._cam_frame
                self._cam_frame = None
            if frame is not None:
                raw, stamp = frame
                w, h       = self._cam_w, self._cam_h
                img_bgr    = np.frombuffer(raw, np.uint8).reshape((h, w, 4))[:, :, :3]
                self._cam_info.header.stamp = stamp
                self.pub_cam_info.publish(self._cam_info)
                msg             = Image()
                msg.header.stamp    = stamp
                msg.header.frame_id = 'camera_gimbal'
                msg.height   = h
                msg.width    = w
                msg.encoding = 'bgr8'
                msg.step     = w * 3
                msg.data     = img_bgr.tobytes()
                self.pub_cam.publish(msg)
            else:
                time.sleep(0.002)


def main():
    rclpy.init()
    rclpy.spin(BebopWebotsMLPSim())
    rclpy.shutdown()

if __name__ == '__main__':
    main()