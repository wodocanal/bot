import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

import time
from math import pi

GPIO.setwarnings(False)

OUTPUT = GPIO.OUT
LOW = GPIO.LOW
HIGH = GPIO.HIGH

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class Motor:
    def __init__(self, direction_pin, speed_pin, stop_pin, reverse=False):
        self.direction_pin = direction_pin
        self.speed_pin = speed_pin
        self.stop_pin = stop_pin
        self.reverse = reverse

        self.min_speed_pwm = 5

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, OUTPUT)
        GPIO.setup(self.speed_pin, OUTPUT)
        GPIO.setup(self.stop_pin, OUTPUT)

        self.pwm = GPIO.PWM(self.speed_pin, 1000)
        self.pwm.start(0)

        GPIO.output(self.stop_pin, HIGH)

    def set_direction(self, direction):
        if self.reverse:
            direction = not direction
        GPIO.output(self.direction_pin, HIGH if direction else LOW)

    def set_speed(self, speed):
        speed = constrain(speed, 0, 100)
        speed = speed/2
        self.pwm.ChangeDutyCycle(speed)

    def stop(self, state):
        if state:
            GPIO.output(self.stop_pin, HIGH)
            self.pwm.ChangeDutyCycle(0)
        else:
            GPIO.output(self.stop_pin, LOW)

    def set(self, speed):
        dir = bool(speed > 0)
        speed = abs(speed)

        if speed <= self.min_speed_pwm:
            self.stop(1)
        else:
            self.stop(0)
            self.set_direction(dir)
            self.set_speed(abs(constrain(speed, 0, 100)))

    def cleanup(self):
        self.stop(1);
        time.sleep(0.01);
        self.stop(0);

        self.pwm.stop()
        GPIO.cleanup()





class diff_drive_controller_node(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.wheel_separation = 0.34
        self.wheel_radius = 0.08
        self.max_speed = 100

        self.right_motor = Motor(direction_pin=17, speed_pin=18, stop_pin=27, reverse=False)
        self.left_motor = Motor(direction_pin=22, speed_pin=13, stop_pin=23, reverse=True)

        self.create_subscription(Float32, '/wheels/left/pwm', self.left_motor_callback, 10)
        self.create_subscription(Float32, '/wheels/right/pwm', self.right_motor_callback, 10)

    def left_motor_callback(self, msg):
        speed = msg.data
        self.left_motor.set(self.convert_speed_to_pwm(speed))

    def right_motor_callback(self, msg):
        speed = msg.data
        self.right_motor.set(self.convert_speed_to_pwm(speed))

    def convert_speed_to_pwm(self, speed):
        pwm_value = speed * 100
        return constrain(pwm_value, -self.max_speed, self.max_speed)

    def cleanup(self):
        self.left_motor.cleanup()
        self.right_motor.cleanup()


def main(args=None):
    rclpy.init(args=args)
    motors = diff_drive_controller_node()

    try:
        rclpy.spin(motors)
    except KeyboardInterrupt:
        motors.get_logger().info("stopped by interrupt")
    finally:
        motors.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

