import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

from time import time
from math import pi

MAX_PWM = 100
MAX_DELTA = 25


pins = {
    'right' : {
        'speed': 18,
        'dir': 17,
        'stop': 27,
        'encoder': 24}, 
    'left' : {
        'speed': 13,
        'dir': 22,
        'stop': 23,
        'encoder': 4},
}

def constrain(val, min_val, max_val):return min(max_val, max(min_val, val))

def tick2rad(data, ticks_per_round): return data / ticks_per_round * 2 * pi
def rad2tick(data, ticks_per_round): return data / 2 / pi * ticks_per_round


class PID:
    def __init__(self, p, i, d, max_val):
        self.p = p
        self.i = i
        self.d = d
        self.max_val = max_val

        self.old_error = 0
        self.integral = 0
        self.last_time = time()
    
    def calc(self, error):
        dt = time() - self.last_time
        self.last_time = time()

        self.integral += error * dt * self.i
        self.integral = constrain(self.integral, -self.max_val, self.max_val)
        diff = (error - self.old_error) * self.d / dt
        prop = error * self.p
        self.old_error = error
        return self.integral + diff + prop

class Motor_Encoder:
    def __init__(self, pins, ticks_per_round, reverse=False, enc_rev = False, min_pwm = 10, name=''):
        self.name = name
        
        self.pin_dir = pins['dir']
        self.pin_speed = pins['speed']
        self.pin_stop = pins['stop']
        self.pin_enc = pins['encoder']

        GPIO.setup(self.pin_dir, GPIO.OUT)
        GPIO.setup(self.pin_stop, GPIO.OUT)

        GPIO.setup(self.pin_speed, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin_speed, 1000)
        self.pwm.start(0)

        GPIO.setup(self.pin_enc, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin_enc, GPIO.BOTH, self._update_encoder)

        self.min_pwm = min_pwm
        self.reverse = reverse
        self.enc_rev = enc_rev
        
        self.ticks = 0
        self.ticks_per_round = ticks_per_round
        self.prev_ticks = 0

        self.current_angle = 0
        self.prev_angle = 0
        self.delta = 0

        self.target_speed = 0
        self.current_speed = 0
        self.direction = 1

        self.prev_time = time()

        self.pid = PID(0.4, 0, 0, max_val=100)

    def _set_dir_pwm(self, speed):
        if abs(speed) < self.min_pwm:
            GPIO.output(self.pin_stop, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(0)
        else:
            GPIO.output(self.pin_stop, GPIO.LOW)

            dir = int(speed >= 0)
            dir = not dir if self.reverse else dir
            self.direction = 1 if dir else -1

            GPIO.output(self.pin_dir, GPIO.HIGH if dir else GPIO.LOW)

            speed = constrain(abs(speed), self.min_pwm, MAX_PWM + 200)
            speed = speed / 4
            speed = constrain(speed, 20, 100)
            self.pwm.ChangeDutyCycle(speed)

    def set(self, speed):
        current = self.current_speed
        target = speed

        error = target - current
        error = int(self.pid.calc(error))

        result = int(target + (target - current) * 0.7)


        self._set_dir_pwm(speed)


    def _update_encoder(self, val):
        step = self.direction
        if self.enc_rev:
            step *= -1

        self.ticks+=step

    def update_data(self):
        time_delta = time() - self.prev_time
        self.prev_time = time()

        self.angle = self.ticks
        self.delta = self.angle - self.prev_angle
        self.prev_angle = self.angle

        self.current_speed = constrain(self.delta / time_delta / 3 * 2, -100, 100)
    

class diff_drive_controller_node(Node):
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        super().__init__('motors')

        self.wheel_separation = 0.34
        self.wheel_radius = 0.08

        self.left_pid = PID(0.4, 0.05, 0.4, max_val=100)
        self.right_motor = Motor_Encoder(pins=pins['right'], ticks_per_round=100, reverse=False, enc_rev = False, min_pwm = 30, name='right')

        self.right_pid = PID(0.4, 0.05, 0.4, max_val=100)
        self.left_motor = Motor_Encoder(pins=pins['left'], ticks_per_round=100, reverse=True, enc_rev = False, min_pwm = 30, name='left')

        self.get_logger().info("motors initialized")
        
        self.pub_left_angle = self.create_publisher(Float32, '/wheels/left/angle', 10)
        self.pub_left_delta = self.create_publisher(Float32, '/wheels/left/delta', 10)
        self.pub_left_speed = self.create_publisher(Float32, '/wheels/left/current_speed', 10)

        self.pub_right_angle = self.create_publisher(Float32, '/wheels/right/angle', 10)
        self.pub_right_delta = self.create_publisher(Float32, '/wheels/right/delta', 10)
        self.pub_right_speed = self.create_publisher(Float32, '/wheels/right/current_speed', 10)

        self.cmd_vel_callback_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer_publish = self.create_timer(0.1, self.publish_data)

    def cleanup(self):
        GPIO.output(self.left_motor.pin_stop, GPIO.LOW)
        GPIO.output(self.left_motor.pin_dir, GPIO.LOW)
        GPIO.output(self.right_motor.pin_stop, GPIO.LOW)
        GPIO.output(self.right_motor.pin_dir, GPIO.LOW)

        self.left_motor.pwm.ChangeDutyCycle(0)
        self.right_motor.pwm.ChangeDutyCycle(0)
        self.left_motor.pwm.stop()
        self.right_motor.pwm.stop()

        GPIO.cleanup()

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z / 3
        

        left_speed = (linear - angular) * 100
        right_speed = (linear + angular) * 100

        self.left_motor.set(left_speed)
        self.right_motor.set(right_speed)



    def publish_data(self):
        self.left_motor.update_data()
        self.right_motor.update_data()

        left_motor = self.left_motor
        right_motor = self.right_motor

        left_angle_msg = Float32()
        left_delta_msg = Float32()
        left_speed_msg = Float32()
        right_angle_msg = Float32()
        right_delta_msg = Float32()
        right_speed_msg = Float32()

        left_angle_msg.data = float(left_motor.angle)
        left_delta_msg.data = float(left_motor.delta)
        left_speed_msg.data = float(left_motor.current_speed)
        right_angle_msg.data = float(right_motor.angle)
        right_delta_msg.data = float(right_motor.delta)
        right_speed_msg.data = float(right_motor.current_speed)

        self.pub_left_angle.publish(left_angle_msg)
        self.pub_left_delta.publish(left_delta_msg)
        self.pub_left_speed.publish(left_speed_msg)
        self.pub_right_angle.publish(right_angle_msg)
        self.pub_right_delta.publish(right_delta_msg)
        self.pub_right_speed.publish(right_speed_msg)



def main(args=None):
    rclpy.init(args=args)
    motors = diff_drive_controller_node()

    try:
        motors.get_logger().info("motor controller started")
        rclpy.spin(motors)
    except KeyboardInterrupt:
        motors.get_logger().info("stopped by interrupt")
    finally:
        motors.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    