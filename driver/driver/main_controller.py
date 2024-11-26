import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.left_target_speed = self.create_publisher(Float32, '/wheels/left/pwm', 10) 
        self.right_target_speed = self.create_publisher(Float32, '/wheels/right/pwm', 10)
        
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z / 3

        left_speed = linear - angular
        right_speed = linear + angular

        left_pub = Float32()
        left_pub.data = float(constrain(left_speed, -1, 1))

        right_pub = Float32()
        right_pub.data = float(constrain(right_speed, -1, 1))
    
        self.left_target_speed.publish(left_pub)
        self.right_target_speed.publish(right_pub)
        

def main(args=None):
    rclpy.init(args=args)

    diff_drive_controller_node = DifferentialDriveController()
    rclpy.spin(diff_drive_controller_node)
    
    diff_drive_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
