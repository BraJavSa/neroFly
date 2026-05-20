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


_F1 = np.diag([0.988324, 0.986558, 0.802580, 0.853101])
_F2 = np.diag([0.018878, 0.025773, 0.122009, 0.122507])

CMD_TIMEOUT_S = 1.0 / 5.0

GIMBAL_MIN_DEG     = -90.0
GIMBAL_MAX_DEG     =  15.0
GIMBAL_SPEED_DEG_S =  22.0
GIMBAL_DELAY_S     =   0.5

CAM_TIMESTEP_MS = 32
DYN_TIMESTEP_MS = 64


class BebopWebotsFullSim(Node):
    def __init__(self):
        super().__init__("neroFlyulator")

        self.declare_parameter('enable_brake', True)
        self._brake_enabled = (
            self.get_parameter('enable_brake').get_parameter_value().bool_value
        )
        self.get_logger().info(
            f"Brake {'enabled' if self._brake_enabled else 'disabled'}"
        )

        self.robot       = Supervisor()
        self.drone_node  = self.robot.getFromDef("DRONE_BODY")
        self.gimbal_node = self.robot.getFromDef("GIMBAL")
        self.camera_node = self.robot.getFromDef("DRONE_CAMERA")

        self.prop_nodes = [
            self.robot.getFromDef(f"PROP_{n}") for n in ["FR", "FL", "RR", "RL"]
        ]
        self.prop_angle = 0.0

        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(CAM_TIMESTEP_MS)
            w, h = self.camera.getWidth(), self.camera.getHeight()
            f_px = w / (2.0 * math.tan(self.camera.getFov() / 2.0))
            self._cam_info = CameraInfo()
            self._cam_info.header.frame_id  = "camera_gimbal"
            self._cam_info.width            = w
            self._cam_info.height           = h
            self._cam_info.distortion_model = "plumb_bob"
            self._cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self._cam_info.k = [
                f_px, 0.0,  w / 2.0,
                0.0,  f_px, h / 2.0,
                0.0,  0.0,  1.0,
            ]
            self._cam_info.p = [
                f_px, 0.0,  w / 2.0, 0.0,
                0.0,  f_px, h / 2.0, 0.0,
                0.0,  0.0,  1.0,     0.0,
            ]
            self._cam_w, self._cam_h = w, h

        self._cam_lock   = threading.Lock()
        self._cam_frame  = None
        self._cam_thread = threading.Thread(
            target=self._cam_publish_loop, daemon=True
        )
        self._cam_thread.start()

        self.tf_br = TransformBroadcaster(self)

        self.dt       = DYN_TIMESTEP_MS / 1000.0
        self.z_ground = 0.05

        self.eta = np.array([0.0, 0.0, self.z_ground, 0.0])
        self.nu  = np.zeros(4)
        self.u   = np.zeros(4)

        self.gimbal_pitch_current_deg = -10.0
        self.gimbal_pitch_target_deg  = -10.0
        self.gimbal_waiting           = False
        self.gimbal_moving_pitch      = False
        self.gimbal_wait_start        = time.time()

        self.mode     = "IDLE"
        self.max_tilt = math.radians(5.0)

        self.last_cmd      = None
        self.last_cmd_time = time.time()

        self.brake_x = False
        self.brake_y = False

        self.pub_xy       = self.create_publisher(PointStamped, "/bebop/position",          10)
        self.pub_z        = self.create_publisher(Float64,      "/bebop/altitude",           10)
        self.pub_imu      = self.create_publisher(Imu,          "/bebop/imu",                10)
        self.pub_odom     = self.create_publisher(Odometry,     "/bebop/odom",               10)
        self.pub_cam      = self.create_publisher(Image,        "/bebop/camera/image_raw",   10)
        self.pub_cam_info = self.create_publisher(CameraInfo,   "/bebop/camera/camera_info", 10)

        self.create_subscription(Twist,   "/bebop/cmd_vel",     self.cmd_cb,         10)
        self.create_subscription(Empty,   "/bebop/takeoff",     self.takeoff_cb,     10)
        self.create_subscription(Empty,   "/bebop/land",        self.land_cb,        10)
        self.create_subscription(Empty,   "/bebop/emergency",   self.emergency_cb,   10)
        self.create_subscription(Vector3, "/bebop/move_camera", self.move_camera_cb, 10)

        self.create_timer(self.dt, self.timer_cb)

    def cmd_cb(self, msg: Twist):
        self.last_cmd      = msg
        self.last_cmd_time = time.time()

    def takeoff_cb(self, _):
        if self.mode in ["IDLE", "LANDING"]:
            self.mode  = "TAKING_OFF"
            self.u[:]  = 0.0
            self.nu[:] = 0.0
            self.last_cmd_time = time.time()

    def land_cb(self, _):
        if self.mode in ["FLYING", "TAKING_OFF"]:
            self.mode = "LANDING"
            self.u[:] = 0.0

    def emergency_cb(self, _):
        self.mode  = "EMERGENCY"
        self.u[:]  = 0.0
        self.nu[:] = 0.0

    def move_camera_cb(self, msg: Vector3):
        target = float(np.clip(msg.x, GIMBAL_MIN_DEG, GIMBAL_MAX_DEG))
        if abs(target - self.gimbal_pitch_target_deg) < 0.5:
            return
        self.gimbal_pitch_target_deg = target
        self.gimbal_moving_pitch     = True
        self.gimbal_waiting          = True
        self.gimbal_wait_start       = time.time()

    _BRAKE_DEAD_STOP = 0.01

    def _compute_brake_u(
        self, raw_x: float, raw_y: float
    ) -> tuple[bool, bool, float, float]:
        if not self._brake_enabled:
            return False, False, raw_x, raw_y

        timeout = (time.time() - self.last_cmd_time) > CMD_TIMEOUT_S
        brake_x = timeout or (abs(raw_x) < 1e-4)
        brake_y = timeout or (abs(raw_y) < 1e-4)

        if not brake_x and not brake_y:
            return False, False, raw_x, raw_y

        vx_b = self.nu[0] if brake_x else 0.0
        vy_b = self.nu[1] if brake_y else 0.0

        if brake_x and abs(vx_b) < self._BRAKE_DEAD_STOP:
            self.nu[0] = 0.0
            vx_b = 0.0
        if brake_y and abs(vy_b) < self._BRAKE_DEAD_STOP:
            self.nu[1] = 0.0
            vy_b = 0.0

        Kp  = 1.5
        u_x = float(np.clip(-Kp * vx_b, -1.0, 1.0))
        u_y = float(np.clip(-Kp * vy_b, -1.0, 1.0))

        if not brake_x:
            u_x = raw_x
        if not brake_y:
            u_y = raw_y

        return brake_x, brake_y, u_x, u_y

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

    @staticmethod
    def _J(psi: float) -> np.ndarray:
        c, s = math.cos(psi), math.sin(psi)
        return np.array([
            [ c, -s, 0.0, 0.0],
            [ s,  c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

    def timer_cb(self):

        if self.mode == "TAKING_OFF":
            self.nu[0] = self.nu[1] = self.nu[3] = 0.0
            self.nu[2] = 0.5
            eta_dot    = self._J(self.eta[3]) @ self.nu
            self.eta  += eta_dot * self.dt
            if self.eta[2] >= 1.2:
                self.nu[2] = 0.0
                self.mode  = "FLYING"

        elif self.mode in ["LANDING", "EMERGENCY"]:
            speed      = 0.5 if self.mode == "LANDING" else 1.0
            self.nu[:] = 0.0
            self.nu[2] = -speed
            eta_dot    = self._J(self.eta[3]) @ self.nu
            self.eta  += eta_dot * self.dt
            if self.eta[2] <= self.z_ground:
                self.eta[2] = self.z_ground
                self.nu[:]  = 0.0
                self.mode   = "IDLE"

        if self.mode == "FLYING":
            timeout = (time.time() - self.last_cmd_time) > CMD_TIMEOUT_S
            if self.last_cmd is not None and not timeout:
                raw_x   = float(self.last_cmd.linear.x)
                raw_y   = float(self.last_cmd.linear.y)
                raw_z   = float(self.last_cmd.linear.z)
                raw_yaw = float(self.last_cmd.angular.z)
            else:
                raw_x = raw_y = raw_z = raw_yaw = 0.0

            self.brake_x, self.brake_y, self.u[0], self.u[1] = (
                self._compute_brake_u(raw_x, raw_y)
            )
            self.u[2] = raw_z
            self.u[3] = raw_yaw

            nu_dot  = _F1 @ self.u - _F2 @ self.nu
            self.nu = self.nu + self.dt * nu_dot

            self.nu[:2] = np.clip(self.nu[:2], -5.0,  5.0)
            self.nu[2]  = np.clip(self.nu[2],  -5.0,  5.0)
            self.nu[3]  = np.clip(self.nu[3],  -6.0,  6.0)

            eta_dot  = self._J(self.eta[3]) @ self.nu
            self.eta = self.eta + self.dt * eta_dot
            self.eta[3] = math.remainder(self.eta[3], 2.0 * math.pi)

        if not np.all(np.isfinite(self.nu)) or not np.all(np.isfinite(self.eta)):
            self.nu[:]   = 0.0
            self.eta[:3] = [0.0, 0.0, 1.2]

        if self.eta[2] < self.z_ground:
            self.eta[2] = self.z_ground
            self.nu[2]  = 0.0

        roll  = float(self.u[1]) * self.max_tilt
        pitch = float(self.u[0]) * self.max_tilt
        psi   = float(self.eta[3])
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, psi)

        if self.drone_node:
            self.drone_node.getField("translation").setSFVec3f(
                [float(self.eta[0]), float(self.eta[1]), float(self.eta[2])]
            )
            angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
            den   = math.sqrt(max(0.0, 1.0 - qw * qw))
            axis  = [qx / den, qy / den, qz / den] if den > 1e-6 else [0, 0, 1]
            self.drone_node.getField("rotation").setSFRotation(axis + [angle])

           

        self._step_gimbal()

        if self.gimbal_node:
            gimbal_pitch_rad = math.radians(-self.gimbal_pitch_current_deg)
            g_roll  = -roll
            g_pitch = -pitch + gimbal_pitch_rad
            gqx, gqy, gqz, gqw = quaternion_from_euler(g_roll, g_pitch, 0.0)
            g_angle = 2.0 * math.acos(max(-1.0, min(1.0, gqw)))
            g_den   = math.sqrt(max(0.0, 1.0 - gqw * gqw))
            g_axis  = (
                [gqx / g_den, gqy / g_den, gqz / g_den]
                if g_den > 1e-6 else [0, 1, 0]
            )
            self.gimbal_node.getField("rotation").setSFRotation(g_axis + [g_angle])

        self.publish_imu(qx, qy, qz, qw)
        self.publish_tfs(qx, qy, qz, qw)
        self.publish_odom(qx, qy, qz, qw)
        self.publish_xy()
        self.publish_z()

        for _ in range(2):
            if self.robot.step(CAM_TIMESTEP_MS) == -1:
                rclpy.shutdown()
                return
            if self.camera:
                raw = self.camera.getImage()
                if raw:
                    with self._cam_lock:
                        self._cam_frame = (raw, self.get_clock().now().to_msg())
            if self.mode != "IDLE":
                self.prop_angle += 3.0
                for i, prop in enumerate(self.prop_nodes):
                    if prop:
                        prop.getField("rotation").setSFRotation(
                            [0, 0, 1, self.prop_angle * (1 if i % 2 == 0 else -1)]
                        )

    def publish_tfs(self, qx, qy, qz, qw):
        now = self.get_clock().now().to_msg()
        t   = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = "odom"
        t.child_frame_id  = "bebop_link"
        t.transform.translation.x = float(self.eta[0])
        t.transform.translation.y = float(self.eta[1])
        t.transform.translation.z = float(self.eta[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)

    def publish_imu(self, qx, qy, qz, qw):
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "bebop_link"
        msg.orientation.x   = qx
        msg.orientation.y   = qy
        msg.orientation.z   = qz
        msg.orientation.w   = qw
        self.pub_imu.publish(msg)

    def publish_xy(self):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x = float(self.eta[0])
        msg.point.y = float(self.eta[1])
        self.pub_xy.publish(msg)

    def publish_z(self):
        msg      = Float64()
        msg.data = float(self.eta[2])
        self.pub_z.publish(msg)

    def publish_odom(self, qx, qy, qz, qw):
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id  = "bebop_link"

        msg.pose.pose.position.x    = float(self.eta[0])
        msg.pose.pose.position.y    = float(self.eta[1])
        msg.pose.pose.position.z    = float(self.eta[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x  = float(self.nu[0])
        msg.twist.twist.linear.y  = float(self.nu[1])
        msg.twist.twist.linear.z  = float(self.nu[2])
        msg.twist.twist.angular.z = float(self.nu[3])

        self.pub_odom.publish(msg)

    def _cam_publish_loop(self):
        while True:
            with self._cam_lock:
                frame           = self._cam_frame
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
                msg.height          = h
                msg.width           = w
                msg.encoding        = "bgr8"
                msg.step            = w * 3
                msg.data            = img_bgr.tobytes()
                self.pub_cam.publish(msg)
            else:
                time.sleep(0.002)


def main():
    rclpy.init()
    rclpy.spin(BebopWebotsFullSim())
    rclpy.shutdown()


if __name__ == "__main__":
    main()