#!/usr/bin/env python3
"""
BebopWebotsMLPSim
─────────────────
Simulador Webots/ROS 2 para el Bebop donde la dinámica interna
es reemplazada íntegramente por el MLP aprendido exportado como
librería CasADi (.so).

Diferencias respecto a la versión anterior
───────────────────────────────────────────
1. Filtro Savitzky-Golay ONLINE: la entrada al MLP siempre usa
   velocidades filtradas, igual que durante el entrenamiento.
   Se aplica sobre el buffer deslizante de tamaño window_size.

2. Fase de WARM-UP: durante los primeros window_size pasos en
   vuelo la dinámica se maneja manualmente (igual que el script
   de validación) y el MLP no se llama hasta que el historial
   está lleno con datos de vuelo real.

Rutas del modelo
─────────────────
  <package_share>/models/drone_model.so
  <package_share>/models/drone_metadata.json
"""

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


# ──────────────────────────────────────────────
# Parámetros del filtro (deben coincidir con el entrenamiento)
# ──────────────────────────────────────────────
SG_WINDOW   = 11   # window_length del Savitzky-Golay
SG_POLYORD  = 3    # polyorder del Savitzky-Golay

GIMBAL_MIN_DEG     = -90.0
GIMBAL_MAX_DEG     =  15.0
GIMBAL_SPEED_DEG_S =  22.0
GIMBAL_DELAY_S     =   0.5
CMD_TIMEOUT_S      =   1.0 / 5.0

# Filtro paso bajo para los ángulos visuales (roll/pitch del modelo 3D).
# Alpha ∈ (0, 1]: más cerca de 0 → más suave, más cerca de 1 → sin filtro.
# 0.08 ≈ constante de tiempo ~4 frames a 30 Hz → suaviza la vibración
# sin introducir un lag perceptible en los cambios de inclinación.
TILT_ALPHA = 0.08


# ──────────────────────────────────────────────
# Filtro SG online (causal, sobre buffer deslizante)
# ──────────────────────────────────────────────
class OnlineSavgolFilter:
    """
    Aplica Savitzky-Golay sobre un buffer deslizante de tamaño fijo.

    Durante el entrenamiento se usó savgol_filter(..., axis=0) sobre
    toda la señal. Online, replicamos eso aplicando el filtro a las
    últimas `window_length` muestras y devolviendo SOLO el último
    valor filtrado (el más reciente).

    Esto es equivalente al valor que el script de entrenamiento habría
    asignado a la muestra actual si la señal futura fuera constante
    (aproximación causal). El error de fase es mínimo para señales
    suaves típicas de drones.
    """

    def __init__(self, window_length: int = SG_WINDOW,
                 polyorder: int = SG_POLYORD,
                 n_signals: int = 4):
        assert window_length % 2 == 1, "window_length debe ser impar"
        self.wl         = window_length
        self.po         = polyorder
        self.n          = n_signals
        # Buffer de las últimas wl muestras crudas
        self._buf       = deque([np.zeros(n_signals)] * window_length,
                                maxlen=window_length)

    def update(self, v_raw: np.ndarray) -> np.ndarray:
        """
        Ingresa una nueva muestra cruda y devuelve la muestra filtrada
        correspondiente al instante actual.
        """
        self._buf.append(v_raw.copy())
        arr = np.array(self._buf)           # (wl, n_signals)
        filtered = savgol_filter(arr, self.wl, self.po, axis=0)
        return filtered[-1]                 # último valor = instante actual

    def get_window_filtered(self) -> np.ndarray:
        """
        Devuelve las `wl` muestras del buffer ya filtradas.
        Se usa para construir la ventana de entrada del MLP.
        """
        arr = np.array(self._buf)
        return savgol_filter(arr, self.wl, self.po, axis=0)  # (wl, n_signals)

    def reset(self, v_init: np.ndarray | None = None):
        v0 = v_init if v_init is not None else np.zeros(self.n)
        self._buf = deque([v0.copy()] * self.wl, maxlen=self.wl)


# ──────────────────────────────────────────────
# Carga del modelo MLP
# ──────────────────────────────────────────────
def _load_mlp_model(package_name: str = "neroFly"):
    from ament_index_python.packages import get_package_share_directory
    package_share_dir = get_package_share_directory(package_name)
    so_path   = os.path.join(package_share_dir, "models", "drone_model.so")
    json_path = os.path.join(package_share_dir, "models", "drone_metadata.json")

    if not os.path.exists(so_path):
        raise FileNotFoundError(f"No se encuentra el modelo: {so_path}")
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"No se encuentra el metadata: {json_path}")

    with open(json_path, "r") as f:
        meta = json.load(f)

    f_mlp = ca.external("f_mlp", so_path)
    return f_mlp, int(meta["window_size"]), float(meta["dt"])


# ──────────────────────────────────────────────
# Nodo principal
# ──────────────────────────────────────────────
class BebopWebotsMLPSim(Node):

    def __init__(self):
        super().__init__("neroFlyulator")

        # ── Cargar MLP ──────────────────────────────────────────────
        self.f_mlp, self.window_size, self.model_dt = _load_mlp_model()
        self.get_logger().info(
            f"MLP cargado | window_size={self.window_size} | dt_model={self.model_dt:.4f}s"
        )

        # ── Webots ──────────────────────────────────────────────────
        self.robot      = Supervisor()
        self.timestep   = int(self.robot.getBasicTimeStep())
        self.dt         = self.timestep / 1000.0

        self.drone_node  = self.robot.getFromDef("DRONE_BODY")
        self.gimbal_node = self.robot.getFromDef("GIMBAL")
        self.camera_node = self.robot.getFromDef("DRONE_CAMERA")

        self.prop_nodes = [
            self.robot.getFromDef(f"PROP_{n}") for n in ["FR", "FL", "RR", "RL"]
        ]
        self.prop_angle = 0.0

        # ── Cámara ──────────────────────────────────────────────────
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.timestep)
            w, h = self.camera.getWidth(), self.camera.getHeight()
            f_px = w / (2.0 * math.tan(self.camera.getFov() / 2.0))
            self._cam_info = CameraInfo()
            self._cam_info.header.frame_id  = "camera_gimbal"
            self._cam_info.width            = w
            self._cam_info.height           = h
            self._cam_info.distortion_model = "plumb_bob"
            self._cam_info.d = [0.0] * 5
            self._cam_info.k = [f_px, 0.0, w/2.0, 0.0, f_px, h/2.0, 0.0, 0.0, 1.0]
            self._cam_info.p = [f_px, 0.0, w/2.0, 0.0, 0.0, f_px, h/2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self._cam_w, self._cam_h = w, h

        self._cam_lock   = threading.Lock()
        self._cam_frame  = None
        self._cam_thread = threading.Thread(target=self._cam_publish_loop, daemon=True)
        self._cam_thread.start()

        # ── TF broadcaster ──────────────────────────────────────────
        self.tf_br = TransformBroadcaster(self)

        # ── Estado del dron ─────────────────────────────────────────
        self.tick     = 0
        self.z_ground = 0.05
        self.max_tilt = math.radians(5.0)

        # Estado inercial: [x, y, z, yaw]
        self.x    = np.array([0.0, 0.0, self.z_ground, 0.0])
        # Velocidades en body-frame (crudas, el simulador las mantiene aquí)
        self.v_body_raw = np.zeros(4)   # [vx_b, vy_b, vz_b, vpsi]
        # Velocidades inerciales (para odometría)
        self.xdot = np.zeros(4)

        # ── Filtro SG online ────────────────────────────────────────
        # Se actualiza en CADA paso, también durante TAKING_OFF,
        # para que al entrar en FLYING el buffer ya contenga historia útil.
        self.sg_filter = OnlineSavgolFilter(
            window_length=SG_WINDOW,
            polyorder=SG_POLYORD,
            n_signals=4
        )

        # ── Ventana deslizante de velocidades FILTRADAS ─────────────
        # Tamaño = window_size (del modelo).
        # Contiene las últimas window_size v_smooth (salida del SG).
        self.v_history = deque(
            [np.zeros(4)] * self.window_size, maxlen=self.window_size
        )

        # ── Ventana deslizante de comandos u ────────────────────────
        self.u_history = deque(
            [np.zeros(4)] * self.window_size, maxlen=self.window_size
        )

        # Comando actual
        self.u_cmd = np.zeros(4)    # [ux, uy, uz, uyaw]

        # ── Ángulos visuales suavizados (solo cosmética) ─────────────
        # Filtro exponencial de 1er orden: evita la vibración que
        # producen los saltos bruscos de u_cmd en el modelo 3D.
        self.vis_roll  = 0.0
        self.vis_pitch = 0.0

        # ── Contador de warm-up ─────────────────────────────────────
        # El MLP se activa solo después de window_size pasos en FLYING.
        # Mientras tanto se integra manualmente (velocidad constante o
        # con aceleración lineal simple para el despegue).
        self.warmup_steps     = 0
        self.warmup_done      = False

        # ── Control de comandos ──────────────────────────────────────
        self.last_cmd      = None
        self.last_cmd_time = time.time()

        # ── Modos de vuelo ───────────────────────────────────────────
        self.mode             = "IDLE"
        self.last_logged_mode = ""

        # ── Gimbal ───────────────────────────────────────────────────
        self.gimbal_pitch_current_deg = -10.0
        self.gimbal_pitch_target_deg  = -10.0
        self.gimbal_waiting           = False
        self.gimbal_moving_pitch      = False
        self.gimbal_wait_start        = time.time()

        # ── Publishers ───────────────────────────────────────────────
        self.pub_xy       = self.create_publisher(PointStamped, "/bebop/position",          10)
        self.pub_z        = self.create_publisher(Float64,      "/bebop/altitude",           10)
        self.pub_imu      = self.create_publisher(Imu,          "/bebop/imu",                10)
        self.pub_odom     = self.create_publisher(Odometry,     "/bebop/odom",               10)
        self.pub_cam      = self.create_publisher(Image,        "/bebop/camera/image_raw",   10)
        self.pub_cam_info = self.create_publisher(CameraInfo,   "/bebop/camera/camera_info", 10)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(Twist,   "/bebop/cmd_vel",     self.cmd_cb,         10)
        self.create_subscription(Empty,   "/bebop/takeoff",     self.takeoff_cb,     10)
        self.create_subscription(Empty,   "/bebop/land",        self.land_cb,        10)
        self.create_subscription(Empty,   "/bebop/emergency",   self.emergency_cb,   10)
        self.create_subscription(Vector3, "/bebop/move_camera", self.move_camera_cb, 10)

        self.create_timer(self.dt, self.timer_cb)

    # ────────────────────────────────────────────
    # Callbacks de subscripción
    # ────────────────────────────────────────────
    def cmd_cb(self, msg: Twist):
        self.last_cmd      = msg
        self.last_cmd_time = time.time()

    def takeoff_cb(self, _):
        if self.mode in ["IDLE", "LANDING"]:
            self.mode         = "TAKING_OFF"
            self.u_cmd[:]     = 0.0
            self.warmup_steps = 0
            self.warmup_done  = False
            self._reset_history()
            self.last_cmd_time = time.time()
            self.get_logger().info("Takeoff iniciado")

    def land_cb(self, _):
        if self.mode in ["FLYING", "TAKING_OFF"]:
            self.mode     = "LANDING"
            self.u_cmd[:] = 0.0
            self.get_logger().info("Landing iniciado")

    def emergency_cb(self, _):
        self.mode     = "EMERGENCY"
        self.u_cmd[:] = 0.0
        self.get_logger().warn("¡EMERGENCIA! Motores cortados")

    def move_camera_cb(self, msg: Vector3):
        target = float(np.clip(msg.x, GIMBAL_MIN_DEG, GIMBAL_MAX_DEG))
        if abs(target - self.gimbal_pitch_target_deg) < 0.5:
            return
        self.gimbal_pitch_target_deg = target
        self.gimbal_moving_pitch     = True
        self.gimbal_waiting          = True
        self.gimbal_wait_start       = time.time()

    # ────────────────────────────────────────────
    # Historial / reset
    # ────────────────────────────────────────────
    def _reset_history(self):
        """
        Reinicia buffers al tomar o después de emergency.
        El filtro SG se reinicia con el estado de velocidad actual
        para no inyectar un escalón brusco al reanudarse.
        """
        self.sg_filter.reset(self.v_body_raw)
        for _ in range(self.window_size):
            self.v_history.append(np.zeros(4))
            self.u_history.append(np.zeros(4))
        self.v_body_raw[:] = 0.0
        self.xdot[:]       = 0.0

    # ────────────────────────────────────────────
    # Filtrar y actualizar historial de velocidades
    # ────────────────────────────────────────────
    def _update_velocity_buffer(self, v_raw: np.ndarray) -> np.ndarray:
        """
        1. Ingresa v_raw al filtro SG online → v_smooth_now
        2. Empuja v_smooth_now al deque v_history
        3. Devuelve v_smooth_now

        Llamar UNA VEZ por paso de simulación, ANTES de calcular
        la aceleración con el MLP.
        """
        v_smooth_now = self.sg_filter.update(v_raw)
        self.v_history.append(v_smooth_now.copy())
        return v_smooth_now

    # ────────────────────────────────────────────
    # Ventana de entrada para el MLP
    # ────────────────────────────────────────────
    def _build_mlp_input(self, v_smooth_current: np.ndarray,
                         u_actual: np.ndarray) -> np.ndarray:
        """
        Construye el vector de entrada exactamente como en el script
        de entrenamiento:

            x_input = [ v_history (window_size × 4)  |
                        u_history (window_size × 4)  |
                        u_actual  (4)                ]

        v_history ya contiene v_smooth_current en su última posición
        (fue empujado por _update_velocity_buffer justo antes).
        """
        v_win = np.array(self.v_history)        # (window_size, 4)
        u_win = np.array(self.u_history)        # (window_size, 4)

        x_input = np.hstack(
            (v_win.flatten(), u_win.flatten(), u_actual.flatten())
        ).reshape(1, -1)
        return x_input

    # ────────────────────────────────────────────
    # Derivada MLP  →  dvdt en body-frame
    # ────────────────────────────────────────────
    def _mlp_derivative(self, v_smooth_current: np.ndarray,
                        u_actual: np.ndarray) -> np.ndarray:
        """
        Evalúa el MLP con la ventana actual.
        v_smooth_current ya está en v_history (último elemento).
        """
        x_input = self._build_mlp_input(v_smooth_current, u_actual)
        dvdt    = self.f_mlp(x_input).full().flatten()   # (4,)
        return dvdt

    # ────────────────────────────────────────────
    # Paso RK4 del MLP
    # ────────────────────────────────────────────
    def _rk4_step(self, v_smooth: np.ndarray,
                  u_actual: np.ndarray,
                  dt: float) -> np.ndarray:
        """
        Integra un paso RK4 usando el MLP como función de derivada.

        NOTA: para k2, k3, k4 el MLP se evalúa con el mismo historial
        congelado (la ventana no se modifica dentro del paso RK4),
        igual que en el script de validación donde el historial se
        actualiza solo una vez por instante t.
        """
        k1 = self._mlp_derivative(v_smooth,                  u_actual)
        k2 = self._mlp_derivative(v_smooth + 0.5 * dt * k1, u_actual)
        k3 = self._mlp_derivative(v_smooth + 0.5 * dt * k2, u_actual)
        k4 = self._mlp_derivative(v_smooth + dt * k3,        u_actual)
        return v_smooth + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    # ────────────────────────────────────────────
    # Integración cinemática body → inercial
    # ────────────────────────────────────────────
    def _integrate_inertial(self, v_body: np.ndarray, dt: float):
        vx_b, vy_b, vz_b, vpsi = v_body
        psi = self.x[3]

        x_dot   =  vx_b * math.cos(psi) - vy_b * math.sin(psi)
        y_dot   =  vx_b * math.sin(psi) + vy_b * math.cos(psi)
        z_dot   =  vz_b
        psi_dot =  vpsi

        self.x[0] += x_dot   * dt
        self.x[1] += y_dot   * dt
        self.x[2] += z_dot   * dt
        self.x[3] += psi_dot * dt
        self.x[3]  = math.remainder(self.x[3], 2.0 * math.pi)

        self.xdot[0] = x_dot
        self.xdot[1] = y_dot
        self.xdot[2] = z_dot
        self.xdot[3] = psi_dot

    # ────────────────────────────────────────────
    # Gimbal
    # ────────────────────────────────────────────
    def _step_gimbal(self):
        now = time.time()
        if self.gimbal_waiting:
            if (now - self.gimbal_wait_start) >= GIMBAL_DELAY_S:
                self.gimbal_waiting = False
        if self.gimbal_moving_pitch and not self.gimbal_waiting:
            speed = GIMBAL_SPEED_DEG_S * self.dt
            error = self.gimbal_pitch_target_deg - self.gimbal_pitch_current_deg
            if abs(error) <= speed:
                self.gimbal_pitch_current_deg = self.gimbal_pitch_target_deg
                self.gimbal_moving_pitch      = False
            else:
                self.gimbal_pitch_current_deg += math.copysign(speed, error)

    # ────────────────────────────────────────────
    # Timer principal
    # ────────────────────────────────────────────
    def timer_cb(self):
        self.tick += 1
        dt = self.dt   # 0.033 s exacto (basicTimeStep 33ms = 30 Hz = model_dt)

        # ── Leer comando (aplica en FLYING) ─────────────────────────
        timeout = (time.time() - self.last_cmd_time) > CMD_TIMEOUT_S
        if self.last_cmd is not None and not timeout:
            self.u_cmd[0] = float(np.clip(self.last_cmd.linear.x,  -1.0, 1.0))
            self.u_cmd[1] = float(np.clip(self.last_cmd.linear.y,  -1.0, 1.0))
            self.u_cmd[2] = float(np.clip(self.last_cmd.linear.z,  -1.0, 1.0))
            self.u_cmd[3] = float(np.clip(self.last_cmd.angular.z, -1.0, 1.0))
        elif self.mode == "FLYING":
            self.u_cmd[:] = 0.0

        # ── Máquina de estados ──────────────────────────────────────

        if self.mode == "TAKING_OFF":
            # Subida manual hasta 1.2 m
            self.v_body_raw = np.array([0.0, 0.0, 0.5, 0.0])
            self.x[2] += 0.5 * dt

            # Alimentar filtro e historial cada tick (ya son 30 Hz)
            self._update_velocity_buffer(self.v_body_raw)
            self.u_history.append(np.zeros(4))

            if self.x[2] >= 1.2:
                self.mode = "FLYING"
                self.get_logger().info("En vuelo — iniciando warm-up MLP")

        elif self.mode in ["LANDING", "EMERGENCY"]:
            speed = 0.5 if self.mode == "LANDING" else 1.0
            self.v_body_raw = np.array([0.0, 0.0, -speed, 0.0])
            self.x[2] -= speed * dt
            self.xdot[:] = 0.0

            self._update_velocity_buffer(self.v_body_raw)
            self.u_history.append(self.u_cmd.copy())

            if self.x[2] <= self.z_ground:
                self.x[2] = self.z_ground
                self.mode = "IDLE"
                self._reset_history()
                self.get_logger().info("En tierra")

        elif self.mode == "FLYING":

            # ── Warm-up: primeros window_size ticks en FLYING ────────
            if not self.warmup_done:
                self._update_velocity_buffer(self.v_body_raw)
                self.u_history.append(self.u_cmd.copy())
                self._integrate_inertial(self.v_body_raw, dt)

                self.warmup_steps += 1
                if self.warmup_steps >= self.window_size:
                    self.warmup_done = True
                    self.get_logger().info(
                        f"Warm-up completado ({self.window_size} pasos) — "
                        "MLP activo"
                    )

            # ── MLP activo ───────────────────────────────────────────
            else:
                # 1. Filtrar velocidad y empujar a v_history
                v_smooth = self._update_velocity_buffer(self.v_body_raw)

                # 2. Registrar comando en u_history
                self.u_history.append(self.u_cmd.copy())

                # 3. RK4 con dt exacto del modelo (33ms)
                v_new = self._rk4_step(v_smooth, self.u_cmd.copy(), dt)

                # 4. Actualizar velocidad body-frame
                self.v_body_raw = v_new.copy()

                # 5. Integración cinemática → posición inercial
                self._integrate_inertial(self.v_body_raw, dt)

                # 6. Límite de suelo
                if self.x[2] < self.z_ground:
                    self.x[2]          = self.z_ground
                    self.v_body_raw[2] = 0.0

        # ── Sanity check numérico ────────────────────────────────────
        if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.v_body_raw)):
            self.get_logger().error("Estado NaN/Inf detectado — reset")
            self.v_body_raw[:] = 0.0
            self.x[:3]         = [0.0, 0.0, 1.2]
            self._reset_history()
            self.warmup_steps  = 0
            self.warmup_done   = False

        # Límite de suelo global
        if self.x[2] < self.z_ground:
            self.x[2]          = self.z_ground
            self.v_body_raw[2] = 0.0

        # ── Orientación visual (filtro paso bajo exponencial) ────────
        # El target de inclinación viene del comando crudo, que puede
        # saltar abruptamente. El filtro suaviza la transición visual
        # sin afectar en absoluto la dinámica simulada por el MLP.
        roll_target  = float(self.u_cmd[1]) * self.max_tilt
        pitch_target = float(self.u_cmd[0]) * self.max_tilt
        self.vis_roll  += TILT_ALPHA * (roll_target  - self.vis_roll)
        self.vis_pitch += TILT_ALPHA * (pitch_target - self.vis_pitch)
        roll  = self.vis_roll
        pitch = self.vis_pitch
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, float(self.x[3]))

        # ── Mover nodo Webots ────────────────────────────────────────
        if self.drone_node:
            self.drone_node.getField("translation").setSFVec3f(
                [float(self.x[0]), float(self.x[1]), float(self.x[2])]
            )
            angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
            den   = math.sqrt(max(0.0, 1.0 - qw * qw))
            axis  = [qx/den, qy/den, qz/den] if den > 1e-6 else [0, 0, 1]
            self.drone_node.getField("rotation").setSFRotation(axis + [angle])

            if self.mode != "IDLE":
                self.prop_angle += 0.6
                for i, prop in enumerate(self.prop_nodes):
                    if prop:
                        prop.getField("rotation").setSFRotation(
                            [0, 0, 1, self.prop_angle * (1 if i % 2 == 0 else -1)]
                        )

        # ── Gimbal ───────────────────────────────────────────────────
        self._step_gimbal()

        if self.gimbal_node:
            gimbal_pitch_rad = math.radians(-self.gimbal_pitch_current_deg)
            # Reutiliza los ángulos ya suavizados (vis_roll/vis_pitch)
            # para que el gimbal no vibre tampoco.
            g_roll  = -self.vis_roll
            g_pitch = -self.vis_pitch + gimbal_pitch_rad
            gqx, gqy, gqz, gqw = quaternion_from_euler(g_roll, g_pitch, 0.0)
            g_angle = 2 * math.acos(max(-1.0, min(1.0, gqw)))
            g_den   = math.sqrt(max(0.0, 1.0 - gqw * gqw))
            g_axis  = ([gqx/g_den, gqy/g_den, gqz/g_den]
                       if g_den > 1e-6 else [0, 1, 0])
            self.gimbal_node.getField("rotation").setSFRotation(g_axis + [g_angle])

        # ── Publicaciones ────────────────────────────────────────────
        # basicTimeStep = 33ms → 1 tick = 33ms
        #   tick % 1 →  30 Hz  — IMU, TF, cámara
        #   tick % 2 →  15 Hz  — position (xy)
        #   tick % 3 →  ~10 Hz — altitude (z)
        #   tick % 6 →  ~5 Hz  — odometry

        # 30 Hz — IMU + TF + cámara
        if self.camera:
            raw = self.camera.getImage()
            if raw:
                with self._cam_lock:
                    self._cam_frame = (raw, self.get_clock().now().to_msg())
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

    # ────────────────────────────────────────────
    # Publishers ROS 2
    # ────────────────────────────────────────────
    def publish_tfs(self, qx, qy, qz, qw):
        now = self.get_clock().now().to_msg()

        t_base = TransformStamped()
        t_base.header.stamp    = now
        t_base.header.frame_id = "odom"
        t_base.child_frame_id  = "base_link"
        t_base.transform.translation.x = float(self.x[0])
        t_base.transform.translation.y = float(self.x[1])
        t_base.transform.translation.z = float(self.x[2])
        t_base.transform.rotation.x = qx
        t_base.transform.rotation.y = qy
        t_base.transform.rotation.z = qz
        t_base.transform.rotation.w = qw

        gimbal_pitch_rad = math.radians(-self.gimbal_pitch_current_deg)
        # Usa ángulos ya suavizados para que el TF de la cámara no vibre.
        g_roll  = -self.vis_roll
        g_pitch = -self.vis_pitch + gimbal_pitch_rad
        cgx, cgy, cgz, cgw = quaternion_from_euler(g_roll, g_pitch, 0.0)

        t_cam = TransformStamped()
        t_cam.header.stamp    = now
        t_cam.header.frame_id = "base_link"
        t_cam.child_frame_id  = "camera_gimbal"
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
        msg.header.frame_id = "base_link"
        msg.orientation.x   = qx
        msg.orientation.y   = qy
        msg.orientation.z   = qz
        msg.orientation.w   = qw
        self.pub_imu.publish(msg)

    def publish_xy(self):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x = float(self.x[0])
        msg.point.y = float(self.x[1])
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

    # ────────────────────────────────────────────
    # Hilo de publicación de imagen
    # ────────────────────────────────────────────
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
                msg.height   = h
                msg.width    = w
                msg.encoding = "bgr8"
                msg.step     = w * 3
                msg.data     = img_bgr.tobytes()
                self.pub_cam.publish(msg)
            else:
                time.sleep(0.002)


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────
def main():
    rclpy.init()
    rclpy.spin(BebopWebotsMLPSim())
    rclpy.shutdown()


if __name__ == "__main__":
    main()