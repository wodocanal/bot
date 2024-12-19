import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from time import time
from math import pi

class Regulator:
    def __init__(self, p=0, i=0, d=0):
        self.k_p = p
        self.k_i = i
        self.k_d = d

        self.target_speed = 0
        self.current_speed = 0

        self.error = self.target_speed - self.current_speed

        self.result = 0

        self.output_pwm = 0


    def update_target_speed(self, val):
        self.target_speed = val.data

    def update_current_speed(self, val):
        self.current_speed = val.data


    def calc(self):
        self.result = self.target_speed + (self.target_speed - self.current_speed) * self.k_p
        return self.result
    

class Regulator_Node(Node):
    def __init__(self):
        super().__init__('regulators')

        self.left_regulator = Regulator(p=0.3)
        self.right_regulator = Regulator(p=0.3)

        self.create_subscription(Float32, '/wheels/left/target_speed', self.left_regulator.update_target_speed, 10)
        self.create_subscription(Float32, '/wheels/left/current_speed', self.left_regulator.update_current_speed, 10)
        self.create_subscription(Float32, '/wheels/right/target_speed', self.right_regulator.update_target_speed, 10)
        self.create_subscription(Float32, '/wheels/right/current_speed', self.right_regulator.update_current_speed, 10)

        
        self.left_pwm_pub = self.create_publisher(Float32, '/wheels/left/pwm', 10)
        self.right_pwm_pub = self.create_publisher(Float32, '/wheels/right/pwm', 10)

        self.timer_update_val = self.create_timer(0.01, self.update_value)
        self.timer_publish = self.create_timer(0.01, self.publish)


    def update_value(self):
        self.left_regulator.calc()
        self.right_regulator.calc()

    def publish(self):
        left_pwm = Float32()
        right_pwm = Float32()

        left_pwm.data = self.left_regulator.result * 5
        right_pwm.data = self.right_regulator.result * 5

        self.left_pwm_pub.publish(left_pwm)
        self.right_pwm_pub.publish(right_pwm)


def main(args=None):
    rclpy.init(args=args)
    node = Regulator_Node()

    rclpy.spin(node)

    node.cleanup()
    rclpy.shutdown()

