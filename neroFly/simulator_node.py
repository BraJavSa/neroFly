#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, TransformStamped, PointStamped
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import Imu, Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_euler
from controller import Supervisor

class BebopWebotsFullSim(Node):
    def __init__(self):
        super().__init__("neroFlyulator")

        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.drone_node = self.robot.getFromDef("DRONE_BODY")
        self.camera_node = self.robot.getFromDef("DRONE_CAMERA")
        
        self.prop_nodes = [self.robot.getFromDef(f"PROP_{n}") for n in ["FR", "FL", "RR", "RL"]]
        self.prop_angle = 0.0

        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.timestep)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_br = TransformBroadcaster(self)

        self.dt = self.timestep / 1000.0
        self.tick = 0
        self.z_ground = 0.05
        self.x = np.array([0.0, 0.0, self.z_ground, 0.0])
        self.xdot = np.zeros(4)
        self.u = np.zeros(4)
        
        self.last_cmd = None
        self.last_cmd_time = time.time()
        self.drift_timeout = 1.0 
        self.Ku = np.diag([0.8417, 0.8354, 3.966, 9.8524])
        self.Kv = np.diag([0.18227, 0.17095, 4.001, 4.7295])

        self.mode = "IDLE"
        self.last_logged_mode = ""
        self.max_tilt = math.radians(5.0)

        self.pub_xy = self.create_publisher(PointStamped, "/bebop/position", 10)
        self.pub_z = self.create_publisher(Float64, "/bebop/altitude", 10)
        self.pub_imu = self.create_publisher(Imu, "/bebop/imu", 10)
        self.pub_odom = self.create_publisher(Odometry, "/bebop/odom", 10)
        self.pub_cam = self.create_publisher(Image, "/bebop/camera", 10)

        self.create_subscription(Twist, "/bebop/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Empty, "/bebop/takeoff", self.takeoff_cb, 10)
        self.create_subscription(Empty, "/bebop/land", self.land_cb, 10)
        self.create_subscription(Empty, "/bebop/emergency", self.emergency_cb, 10)

        self.pub_cam = self.create_publisher(Image, "/bebop/camera/image_raw", 10)
        self.pub_cam_info = self.create_publisher(CameraInfo, "/bebop/camera/camera_info", 10)
        self.create_timer(self.dt, self.timer_cb)

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def takeoff_cb(self, _):
        if self.mode in ["IDLE", "LANDING"]:
            self.mode = "TAKING_OFF"
            self.u[:] = 0.0
            self.last_cmd_time = time.time()

    def land_cb(self, _):
        if self.mode in ["FLYING", "TAKING_OFF"]:
            self.mode = "LANDING"
            self.u[:] = 0.0

    def emergency_cb(self, _):
        self.mode = "EMERGENCY"
        self.u[:] = 0.0

    def timer_cb(self):
        self.tick += 1
        curr_time = time.time()

        if self.mode != self.last_logged_mode:
            self.get_logger().info(f"Drone Mode: {self.mode}")
            self.last_logged_mode = self.mode

        if self.mode == "TAKING_OFF":
            self.x[2] += 0.5 * self.dt
            if self.x[2] >= 1.2: self.mode = "FLYING"
        elif self.mode in ["LANDING", "EMERGENCY"]:
            speed = 0.5 if self.mode == "LANDING" else 1.0
            self.x[2] -= speed * self.dt
            if self.x[2] <= self.z_ground:
                self.x[2] = self.z_ground
                self.mode = "IDLE"

        if self.mode == "FLYING":
            if self.last_cmd and (curr_time - self.last_cmd_time) < self.drift_timeout:
                self.u = np.array([self.last_cmd.linear.x, self.last_cmd.linear.y, self.last_cmd.linear.z, self.last_cmd.angular.z])
            else:
                t = self.tick * self.dt
                self.u = np.array([0.05 * math.sin(0.8 * t), 0.05 * math.cos(0.7 * t), 0.005 * math.sin(0.4 * t), 0.0])
        else:
            self.u[:] = 0.0

        if self.mode == "FLYING":
            yaw = self.x[3]
            c, s = math.cos(yaw), math.sin(yaw)
            F = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            xddot = F @ (self.Ku @ self.u) - self.Kv @ self.xdot
            self.xdot += xddot * self.dt
            self.x += self.xdot * self.dt
        
        if self.x[2] < self.z_ground: 
            self.x[2] = self.z_ground
            self.xdot[2] = 0.0

        roll, pitch = float(self.u[1])*self.max_tilt, float(self.u[0])*self.max_tilt
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, float(self.x[3]))

        if self.drone_node:
            self.drone_node.getField("translation").setSFVec3f([float(self.x[0]), float(self.x[1]), float(self.x[2])])
            angle = 2 * math.acos(max(-1.0, min(1.0, qw)))
            den = math.sqrt(max(0.0, 1.0 - qw*qw))
            axis = [qx/den, qy/den, qz/den] if den > 1e-6 else [0, 0, 1]
            self.drone_node.getField("rotation").setSFRotation(axis + [angle])
            
            if self.mode != "IDLE":
                self.prop_angle += 0.6
                for i, prop in enumerate(self.prop_nodes):
                    if prop: prop.getField("rotation").setSFRotation([0, 0, 1, self.prop_angle * (1 if i % 2 == 0 else -1)])

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'camera_gimbal', now)
            rot = trans.transform.rotation
            ca_angle = 2 * math.acos(max(-1.0, min(1.0, rot.w)))
            ca_den = math.sqrt(max(0.0, 1.0 - rot.w*rot.w))
            ca_axis = [rot.x/ca_den, rot.y/ca_den, rot.z/ca_den] if ca_den > 1e-6 else [0, 0, 1]
            if self.camera_node:
                self.camera_node.getField("rotation").setSFRotation(ca_axis + [ca_angle])
        except Exception:
            pass

        if self.tick % 2 == 0:
            self.publish_camera()
            self.publish_imu(qx, qy, qz, qw)
            self.publish_tfs(qx, qy, qz, qw)
            self.publish_odom(qx, qy, qz, qw)

        if self.tick % 4 == 0: self.publish_xy()
        if self.tick % 6 == 0: self.publish_z()
        if self.robot.step(self.timestep) == -1: rclpy.shutdown()

    def publish_tfs(self, qx, qy, qz, qw):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(self.x[0]), float(self.x[1]), float(self.x[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf_br.sendTransform(t)

    def publish_imu(self, qx, qy, qz, qw):
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
        self.pub_imu.publish(msg)

    def publish_xy(self):
        msg = PointStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), "odom"
        msg.point.x, msg.point.y = float(self.x[0]), float(self.x[1])
        self.pub_xy.publish(msg)

    def publish_z(self):
        msg = Float64()
        msg.data = float(self.x[2])
        self.pub_z.publish(msg)

    def publish_odom(self, qx, qy, qz, qw):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = self.get_clock().now().to_msg(), "odom", "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = float(self.x[0]), float(self.x[1]), float(self.x[2])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qx, qy, qz, qw
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = float(self.xdot[0]), float(self.xdot[1]), float(self.xdot[2])
        msg.twist.twist.angular.z = float(self.xdot[3])
        self.pub_odom.publish(msg)

    
    def publish_camera(self):
        if self.camera:
            img_data = self.camera.getImage()
            if img_data:
                now = self.get_clock().now().to_msg()
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                fov = self.camera.getFov()

                # 1. Publicar CameraInfo (Parámetros Intrínsecos)
                info_msg = CameraInfo()
                info_msg.header.stamp = now
                info_msg.header.frame_id = "camera_gimbal"
                info_msg.width = width
                info_msg.height = height
                info_msg.distortion_model = "plumb_bob"
                info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0] # Sin distorsión en simulación básica

                # Cálculo de f basado en el FOV horizontal
                f = width / (2.0 * math.tan(fov / 2.0))
                
                # Matriz K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                info_msg.k = [f, 0.0, width/2.0, 
                              0.0, f, height/2.0, 
                              0.0, 0.0, 1.0]
                
                # Matriz P: [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
                info_msg.p = [f, 0.0, width/2.0, 0.0,
                              0.0, f, height/2.0, 0.0,
                              0.0, 0.0, 1.0, 0.0]

                self.pub_cam_info.publish(info_msg)

                # 2. Publicar Imagen (Conversión de BGRA a BGR)
                # Webots entrega BGRA, BebopTagNode espera bgr8 (3 canales)
                import numpy as np
                img_array = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
                img_bgr = img_array[:, :, :3] # Eliminamos el canal Alpha

                msg = Image()
                msg.header.stamp = now
                msg.header.frame_id = "camera_gimbal"
                msg.height = height
                msg.width = width
                msg.encoding = "bgr8"
                msg.step = width * 3
                msg.data = img_bgr.tobytes()
                
                self.pub_cam.publish(msg)
    
    
def main():
    rclpy.init()
    rclpy.spin(BebopWebotsFullSim())
    rclpy.shutdown()

if __name__ == "__main__": main()