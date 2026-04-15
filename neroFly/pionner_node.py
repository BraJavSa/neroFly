import rclpy
from controller import Robot
from geometry_msgs.msg import Twist

class PioneerRosController:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Nombres exactos de los motores del Pioneer 3AT en Webots
        self.wheels = []
        wheel_names = ['front left wheel', 'front right wheel', 'back left wheel', 'back right wheel']
        for name in wheel_names:
            motor = self.robot.getDevice(name)
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.wheels.append(motor)

        self.node = rclpy.create_node('pioneer_controller')
        self.subscription = self.node.create_subscription(
            Twist,
            '/pioneer/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def run(self):
        while self.robot.step(self.time_step) != -1:
            left_speed = self.linear_vel - self.angular_vel
            right_speed = self.linear_vel + self.angular_vel
            
            if len(self.wheels) == 4:
                self.wheels[0].setVelocity(left_speed)
                self.wheels[1].setVelocity(right_speed)
                self.wheels[2].setVelocity(left_speed)
                self.wheels[3].setVelocity(right_speed)
            
            rclpy.spin_once(self.node, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    pioneer_robot = Robot()
    controller = PioneerRosController(pioneer_robot)
    controller.run()
    # No olvides cerrar rclpy al terminar
    rclpy.shutdown()

if __name__ == '__main__':
    main()