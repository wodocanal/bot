import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Quaternion, TransformStamped
import math
from math import pi
from tf2_ros import TransformBroadcaster

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Подписки на дельты энкодеров
        self.left_encoder_sub = self.create_subscription(Float32, '/wheels/left/delta', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Float32, '/wheels/right/delta', self.right_encoder_callback, 10)

        # Инициализация TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Переменные для хранения положения и ориентации робота
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # ориентация (угол поворота вокруг оси Z)

        # Параметры робота
        self.wheel_base = 0.34  # расстояние между колесами (м)
        self.wheel_radius = 0.08  # радиус колеса (м)
        self.encoder_ticks_per_rev = 100 # количество тиков энкодера на один оборот

        # Дельты энкодеров
        self.left_encoder_angle = 0
        self.right_encoder_angle = 0

    def left_encoder_callback(self, msg):
        self.left_encoder_angle = msg.data
        self.update_odometry()

    def right_encoder_callback(self, msg):
        self.right_encoder_angle = msg.data
        self.update_odometry()

    def update_odometry(self):
        # Преобразуем углы энкодеров в перемещение колес
        left_distance = self.encoder_angle_to_distance(self.left_encoder_angle)
        right_distance = self.encoder_angle_to_distance(self.right_encoder_angle)

        # Рассчитываем перемещение робота
        delta_s = (right_distance + left_distance) / 2
        delta_theta = (right_distance - left_distance) / self.wheel_base / pi * 1.95

        # Обновляем положение и ориентацию робота
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)
        self.theta += delta_theta

        self.publish_odometry()

    def encoder_angle_to_distance(self, angle):
        return angle * self.wheel_radius / self.encoder_ticks_per_rev * 2 * pi

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def publish_odometry(self):
        # Получаем текущее время
        current_time = self.get_clock().now().to_msg()

        # Формируем сообщение об одометрии
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'

        # Позиция
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Ориентация (преобразуем угол в кватернион)
        odom_quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=odom_quat[0], y=odom_quat[1], z=odom_quat[2], w=odom_quat[3])

        # Линейная и угловая скорость
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Публикация одометрии
        self.odom_pub.publish(odom_msg)

        # Публикация TF трансформации для odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()