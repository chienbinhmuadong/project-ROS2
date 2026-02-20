import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

class SensorDataHandler(Node):

    def __init__(self):
        super().__init__('camera_data_processing')
        sub_topic_name = "/camera/image_raw"
        self.camera_subscriber = self.create_subscription(
            Image,
            sub_topic_name,
            self.camera_cb,
            10
        )
        self.bridge = CvBridge()
        self.current_image = None
        self.get_logger().info(f"Subscribed: {sub_topic_name}")

    def camera_cb(self, msg):
        # Конвертация ROS2 Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Преобразуем в grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Выделение границ (Canny)
        edges = cv2.Canny(gray, 100, 200)

        # Показываем оригинал и границы
        cv2.imshow("camera", frame)
        cv2.imshow("edges", edges)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
