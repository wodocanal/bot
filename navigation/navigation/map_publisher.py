import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import yaml
import cv2
import numpy as np
from rclpy.clock import Clock
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

qos = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE
)


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Параметры узла
        self.declare_parameter('yaml_filename', '/home/wodo/workspace/maps/map_candidat.yaml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_frequency', 1.0)

        # Получение параметров
        yaml_filename = self.get_parameter('yaml_filename').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Загрузка карты
        self.map_data = self.load_map(yaml_filename)
        if self.map_data is None:
            self.get_logger().error('Failed to load map. Check your YAML file and paths.')
            return

        # Публикаторы
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', qos)


        # Таймер для публикации карты
        self.create_timer(1 / publish_frequency, self.publish_map)
        self.create_timer(1 / publish_frequency, self.publish_static_tf)

        # Публикация фрейма `map`
        # self.publish_static_tf()

    def load_map(self, yaml_filename):
        try:
            with open(yaml_filename, 'r') as file:
                yaml_data = yaml.safe_load(file)
            
            # Загрузка изображения карты
            image_path = yaml_data['image']
            resolution = yaml_data['resolution']
            origin = yaml_data['origin']
            negate = yaml_data['negate']
            occupied_thresh = yaml_data['occupied_thresh']
            free_thresh = yaml_data['free_thresh']

            # Чтение изображения карты
            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            image = cv2.flip(image, 0)  # Отзеркаливание по оси X
            if image is None:
                self.get_logger().error(f"Unable to read map image: {image_path}")
                return None

            # Учитываем параметр negate для инверсии
            if negate == 1:
                image = cv2.bitwise_not(image)

            # Преобразование изображения в OccupancyGrid

            map_data = np.full(image.shape, -1, dtype=np.int8)

            # Помечаем препятствия (значения 100)
            map_data[image <= 255 * occupied_thresh] = 100

            # Помечаем свободные ячейки (значения 0)
            map_data[(image > 255 * free_thresh) & (image <= 255)] = 0


            # map_data = (image <= 255 * occupied_thresh).astype(np.int8) * 100  # Препятствия
            # map_data += ((image > 255 * free_thresh) & (image > 0)).astype(np.int8) * -1  # Свободное пространство

            return {
                'data': map_data.flatten().tolist(),
                'resolution': resolution,
                'origin': origin,
                'width': image.shape[1],
                'height': image.shape[0]
            }
        except Exception as e:
            self.get_logger().error(f"Error loading map: {str(e)}")
            return None

    def publish_map(self):
        # Формирование сообщения OccupancyGrid
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = Clock().now().to_msg()
        map_msg.header.frame_id = self.frame_id

        map_msg.info.resolution = self.map_data['resolution']
        map_msg.info.width = self.map_data['width']
        map_msg.info.height = self.map_data['height']
        map_msg.info.origin.position.x = self.map_data['origin'][0]
        map_msg.info.origin.position.y = self.map_data['origin'][1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = math.pi

        map_msg.data = self.map_data['data']
        self.map_pub.publish(map_msg)

    def publish_static_tf(self):
        # Публикация статической трансформации для фрейма `map`
        tf_msg = TFMessage()
        transform = TransformStamped()

        transform.header.stamp = Clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = 'odom'  # Статический фрейм карты
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0

        tf_msg.transforms.append(transform)
        self.tf_pub.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
