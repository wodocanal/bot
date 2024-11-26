import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import os
from datetime import datetime

class ManualMapping(Node):
    def __init__(self):
        super().__init__('manual_mapping')

        # Создание папки для карт, если её нет
        self.maps_dir = os.path.join(os.getcwd(), "maps")
        os.makedirs(self.maps_dir, exist_ok=True)
        self.get_logger().info(f"Maps will be saved in: {self.maps_dir}")

        # Параметр сервиса сохранения карты
        self.declare_parameter('save_map_service', '/slam_toolbox/save_map')
        self.save_map_service = self.get_parameter('save_map_service').get_parameter_value().string_value

    def save_map(self):
        # Генерация уникального имени файла
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_file_path = os.path.join(self.maps_dir, f"map_{timestamp}")

        # Вызов сервиса сохранения карты
        self.get_logger().info(f"Saving the map to: {map_file_path}.")
        try:
            save_map_client = self.create_client(Empty, self.save_map_service)
            if not save_map_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {self.save_map_service} is not available.")
                return False

            # Передача пути к карте через переменную среды (slam_toolbox её поддерживает)
            os.environ['SLAM_TOOLBOX_MAP_FILE_NAME'] = map_file_path
            request = Empty.Request()
            future = save_map_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                self.get_logger().info(f"Map saved successfully to: {map_file_path}.yaml")
                return True
            else:
                self.get_logger().error("Failed to save the map.")
                return False
        except Exception as e:
            self.get_logger().error(f"Error while saving the map: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ManualMapping()

    try:
        input("Press Enter to save the map...")  # Ожидаем команды пользователя
        if node.save_map():
            node.get_logger().info("Map saved successfully!")
        else:
            node.get_logger().error("Failed to save the map.")
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
