import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from time import time
from math import pi

class Encoder:
    def __init__(self, pin, inverted=False):
        self.pin = pin
        self.inverted = inverted
        self.ticks_per_round = 80
        self.update_rate = 0.1
        
        self.direction = 0

        self.ticks = 0
        self.prev_ticks = 0
        self.prev_time = time()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin, GPIO.BOTH, callback=self.tick)

        self._current_encoder = GPIO.input(self.pin)
        self._previous_encoder = self.current_encoder

    def update_dir(self, speed): # обновление направления на основе целевого направления вращения колеса (не лучшая идея но пока что так)
        data = speed.data
        if data > 0:
            self.direstion = 1
        elif data == 0:
            self.direction = 1 #TODO replace to 0 for future 
        else:
            self.direction = -1
        
        if self.inverted:
            self.direction = self.direction * -1

    def tick(self): # обновление тиков
        self._current_encoder = GPIO.input(self.pin)

        if self._current_encoder != self._previous_encoder:
            self.ticks += self.direction
            
        self._previous_encoder = self._current_encoder

    def calc_angle(self): # вывод угла на основе текущего значения тиков
        angle = self.ticks / self.ticks_per_round  #calculating ticks
        angle = angle * 360 # to degrees
        angle = angle * pi / 180 # to radians
        return angle

    def calc_speed(self):       # TODO calculating current_speed
        current_time = time()
        time_delta = current_time - self.prev_time

        ticks_delta = self.ticks - self.prev_ticks
        
        if time_delta > 0:
            speed = ticks_delta / time_delta
        else:
            speed = 0.0

        return speed

    def cleanup(self):  #выполняется при выключении
        GPIO.remove_event_detect(self.pin)
        GPIO.cleanup(self.pin)


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoders')

        self.left_encoder = Encoder(pin=5, inverted = False)
        self.right_encoder = Encoder(pin=6, inverted = False)
        
        self.left_angle = self.create_publisher(Float32, '/wheels/left_wheel/angle', 10)
        self.left_speed = self.create_publisher(Float32, '/wheels/left_wheel/current_speed', 10)
        self.create_subscription(Float32, '/wheels/left_wheel/target_speed', self.left_speed_callback, 10)
        
        self.right_angle = self.create_publisher(Float32, '/wheels/right_wheel/angle', 10)
        self.right_speed = self.create_publisher(Float32, '/wheels/right_wheel/current_speed', 10)
        self.create_subscription(Float32, '/wheels/right_wheel/target_speed', self.right_speed_callback, 10)
        
        self.timer = self.create_timer(1/10, self.publish_encoders)

    def left_speed_callback(self, msg):
        self.left_encoder.update_direction(msg.data)

    def right_speed_callback(self, msg):
        self.right_encoder.update_direction(msg.data)

    def publish_encoders(self):
        left_angle_msg = Float32()
        left_angle_msg.data = self.left_encoder.calc_anlge()
        self.left_angle.publish(left_angle_msg)
        
        right_angle_msg = Float32()
        right_angle_msg.data = self.right_encoder.calc_anlge()
        self.right_angle.publish(right_angle_msg)
        
        left_speed_msg = Float32()
        left_speed_msg.data = self.left_encoder.calc_speed()
        self.left_speed.publish(left_speed_msg)
        
        right_speed_msg = Float32()
        right_speed_msg.data = self.right_encoder.calc_speed()
        self.right_speed.publish(right_speed_msg)
        
    def cleanup(self):
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Очистка перед завершением
    node.cleanup()
    rclpy.shutdown()

