import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from time import time
from math import pi

GPIO.setwarnings(False)

class Encoder:
    def __init__(self, pin, inverted=False):
        self.pin = pin
        self.inverted = inverted
        self.ticks_per_round = 80
        self.update_rate = 0.1
        
        self.direction = 1

        self.ticks = 0
        self.prev_ticks = 0
        self.prev_time = time()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._current_encoder = GPIO.input(self.pin)
        self._previous_encoder = self._current_encoder

        self.angle = 0.0
        self.prev_angle = 0.0
        self.delta = self.angle - self.prev_angle

        self.speed = 0.0

        

    def update_dir(self, speed): # обновление направления на основе целевого направления вращения колеса (не лучшая идея но пока что так)
        if speed > 0:
            self.direction = 1
        elif speed == 0:
            self.direction = 0 
        else:
            self.direction = -1
        
        if self.inverted:
            self.direction = self.direction * -1

    def update_tick(self): # обновление тиков
        self._current_encoder = GPIO.input(self.pin)

        if self._current_encoder != self._previous_encoder:
            self.ticks += self.direction
            
        self._previous_encoder = self._current_encoder

    def calc_angle(self): # вывод угла на основе текущего значения тиков
        angle = self.ticks / self.ticks_per_round  # calculating ticks
        angle = angle * 2 * pi  # to radians

        self.delta = angle - self.prev_angle
        self.prev_angle = self.angle
        self.angle = angle

    def calc_speed(self):       # TODO calculating current_speed
        current_time = time()
        time_delta = current_time - self.prev_time
        self.prev_time = time()

        ticks_delta = self.ticks - self.prev_ticks
        self.prev_ticks = self.ticks
        
        if ticks_delta > 0:
            speed = ticks_delta / time_delta / 100 #TODO исправь вычисления
        else:
            speed = 0.0

        self.speed = speed


    def cleanup(self):  #выполняется при выключении
        GPIO.cleanup(self.pin)


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoders')

        self.left_encoder = Encoder(pin=24, inverted = False)
        self.right_encoder = Encoder(pin=4, inverted = False)
        
        self.left_angle = self.create_publisher(Float32, '/wheels/left/angle', 10)
        self.right_angle = self.create_publisher(Float32, '/wheels/right/angle', 10)
        self.left_delta = self.create_publisher(Float32, '/wheels/left/delta', 10)
        self.right_delta = self.create_publisher(Float32, '/wheels/right/delta', 10)
        self.left_speed = self.create_publisher(Float32, '/wheels/left/current_speed', 10)
        self.right_speed = self.create_publisher(Float32, '/wheels/right/current_speed', 10)
        self.create_subscription(Float32, '/wheels/left/pwm', self.left_speed_callback, 10)
        self.create_subscription(Float32, '/wheels/right/pwm', self.right_speed_callback, 10)
        
        self.timer_ticks = self.create_timer(0.001, self.update_ticks)
        self.timer_update_val = self.create_timer(0.1, self.update_value)
        self.timer_publish = self.create_timer(0.01, self.publish_encoders)

    def left_speed_callback(self, msg):
        speed = msg.data
        self.left_encoder.update_dir(speed)

    def right_speed_callback(self, msg):
        speed = msg.data
        self.right_encoder.update_dir(speed)

    def update_ticks(self):
        self.left_encoder.update_tick()
        self.right_encoder.update_tick()

    def update_value(self):
        self.left_encoder.calc_angle()
        self.left_encoder.calc_speed()
        self.right_encoder.calc_angle()
        self.right_encoder.calc_speed()

    def publish_encoders(self):
        left_angle_msg = Float32()
        right_angle_msg = Float32()
        left_speed_msg = Float32()
        right_speed_msg = Float32()
        left_delta_msg = Float32()
        right_delta_msg = Float32()

        left_angle_msg.data = self.left_encoder.angle
        right_angle_msg.data = self.right_encoder.angle
        right_speed_msg.data = self.right_encoder.speed
        left_speed_msg.data = self.left_encoder.speed
        left_delta_msg.data = self.left_encoder.delta
        right_delta_msg.data = self.right_encoder.delta

        self.left_angle.publish(left_angle_msg)
        self.right_angle.publish(right_angle_msg)
        self.left_speed.publish(left_speed_msg)
        self.right_speed.publish(right_speed_msg)
        self.left_delta.publish(left_delta_msg)
        self.right_delta.publish(right_delta_msg)
        
    def cleanup(self):
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

    rclpy.spin(node)

    node.cleanup()
    rclpy.shutdown()

