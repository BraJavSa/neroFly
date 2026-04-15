#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from controller import Supervisor

class TargetVisualizer(Node):
    def __init__(self):
        super().__init__('target_visualizer')
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Acceso directo al Robot para moverlo
        self.target_node = self.robot.getSelf()
        self.trans_field = self.target_node.getField("translation")
        self.rot_field = self.target_node.getField("rotation")

        self.create_subscription(
            Float64MultiArray, 
            '/bebop/ref_vec', 
            self.ref_callback, 
            10)
        
        self.create_timer(self.timestep / 1000.0, self.timer_callback)
        self.get_logger().info("Visualizador activo. Flecha oculta bajo el origen.")

    def ref_callback(self, msg):
        if len(msg.data) >= 4:
            # Al recibir datos, se posiciona en las coordenadas reales (z >= 0)
            self.trans_field.setSFVec3f([float(msg.data[0]), float(msg.data[1]), float(msg.data[2])])
            self.rot_field.setSFRotation([0.0, 0.0, 1.0, float(msg.data[3])])

    def timer_callback(self):
        if self.robot.step(self.timestep) == -1:
            rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(TargetVisualizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()