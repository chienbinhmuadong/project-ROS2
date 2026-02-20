import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarHandler(Node):
    def __init__(self):
        super().__init__('lidar_min_distance_node')
        self.subscription = self.create_subscription(LaserScan, '/diff_drive/scan', self.lidar_callback, 10)
        self.get_logger().info("initialized. Subscribed /scan")

        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        for i in range(len(ranges)):
            if not np.isfinite(ranges[i]) or ranges[i] < 0.02:
                ranges[i] = np.nan 
        
        def sector(angle_start, angle_end):
            a_start = math.radians(angle_start)
            a_end = math.radians(angle_end)

            i_start = int((a_start - msg.angle_min) / msg.angle_increment) 
            i_end = int((a_end - msg.angle_min) / msg.angle_increment)
            i_start = max(0, min(len(ranges) - 1, i_start))
            i_end = max(0, min(len(ranges) - 1, i_end))

            if i_start > i_end:
                i_start, i_end = i_end, i_start 

            sector = ranges[i_start:i_end+1]

            if np.all(np.isnan(sector)):
                return float('inf')
            
            return float(np.nanmin(sector))
        
        self.front_min = sector(-15, 15)
        self.left_min = sector(45, 135)
        self.right_min = sector(-135, -45)


def main(args=None):
    rclpy.init(args=args)
    node = LidarHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
