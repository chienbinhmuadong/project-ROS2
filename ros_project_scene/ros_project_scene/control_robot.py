import rclpy, math 
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, LaserScan 
import cv2
from cv_bridge import CvBridge
import numpy as np 



class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.lidar = self.create_subscription(LaserScan, '/diff_drive/scan', self.lidar_cb, 10)
        self.camera = self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.timer = self.create_timer(0.2, self.control_loop)
        self.time_start = None 
        self.msg_vel = Float64MultiArray()
        self.bridge = CvBridge()

        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        self.state = "GO"        # GO hoặc AVOID
        self.avoid_dir = None    # "L" hoặc "R"

        self.stop_dist  = 1.0  
        self.clear_dist = 1.4    # thoát tránh vật cản (lớn hơn stop_dist)

        # nhận diện cho camera
        self.white_hole = False
        self.hole_position = "CENTER"
        self.hole_size = 0
        self.white_threshold = 200 # ngưỡng màu trắng [0 - 225]
        self.min_hole = 300 # diện tích min cần tránh
        self.roi_height_ratio = 0.3 #lấy nửa dưới ảnh để xủ lý do khu vực đường đi chỉ thấy ở nửa dưới
        

    def lidar_cb(self, msg : LaserScan):
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
        
        # self.front_min = sector(-15, 15)        
        self.front_min = sector(-20, 20)        
        self.left_min = sector(45, 90)
        self.right_min = sector(-90, -45)
        self.get_logger().info(f"front_min: {self.front_min:.3f}, left_min: {self.left_min:.3f}, right_min: {self.right_min:.3f}")


    def camera_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        height, width = cv_image.shape[:2]   #kích thước ảnh gốc
        roi_y_start = int(height * (1 - self.roi_height_ratio))
        roi = cv_image[roi_y_start:height, 0:width]   # lấy phần nửa dưới ảnh để xử lý, độ dài cắt lấy một nửa, độ rộng giữ nguyên

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        adaptive_mask = cv2.adaptiveThreshold(
        blurred, 
        255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV,
        11,  # block size
        2    # C value
        )

        white_pixels = np.sum(adaptive_mask == 255)
        black_pixels = np.sum(adaptive_mask == 0)
        self.get_logger().info(f"Mask: White={white_pixels}, Black={black_pixels}")

        contours, _ = cv2.findContours(adaptive_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # timf contours cho vungf tranwgs

        largest_hole = None
        largest_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > max(self.min_hole, largest_area):
                largest_area = area
                largest_hole = contour

        if largest_hole is not None:
            self.white_hole = True
            self.hole_size = largest_area

            M = cv2.moments(largest_hole) # vi tri o trang
            if M['m00'] > 0:
                hole_cx = int(M['m10'] / M['m00'])

                if hole_cx < width/3: # vung ben trai
                    self.hole_position = "LEFT"
                elif hole_cx > 2/3*width:
                    self.hole_position = "RIGHT"
                else:
                    self.hole_position = "CENTER"

            self.get_logger().info(f"WHITE AREA: {self.hole_position}")
                # self.visualize_detection(cv_image, roi_y_start, largest_hole, hole_cx)
        else:
            self.white_hole = False
            self.hole_position = "CENTER"
            self.hole_size = 0
            self.get_logger().info(f"WHITE AREA NOT EXIST")


    def control_loop(self):
        if not np.isfinite(self.front_min):
            return

        if self.state == "GO":
            if self.front_min < self.stop_dist:  # Có vật cản
                self.state = "AVOID"
                if self.left_min > 1.2:
                    self.avoid_dir = "L"
                else:
                    self.avoid_dir = "R" 
        
        elif self.state == "AVOID":
            if self.front_min > self.clear_dist:  # Đã thoát
                self.state = "GO"
                self.avoid_dir = None


        if self.state == "GO":
            if self.front_min > self.clear_dist and self.left_min > self.clear_dist:
                self.msg_vel.data = [-5.0, 5.0]  # Quay trái
            # elif self.front_min > self.clear_dist and self.left_min < 0.5:
            #     self.msg_vel.data = [5.0, -5.0]
            else:
                self.msg_vel.data = [5.0, 5.0] 
        
            if self.white_hole:
                if self.hole_position == "LEFT" or self.hole_position == "CENTER":  #uu tien re phai kkhi gap ho o giua
                    self.msg_vel.data = [3.0, -3.0]
                elif self.hole_position == "RIGHT":
                    self.msg_vel.data = [-3.0, 3.0]
        
        elif self.state == "AVOID":
            if self.avoid_dir == "L":
                self.msg_vel.data = [-5.0, 5.0]  
            else:
                self.msg_vel.data = [5.0, -5.0]  
        
        self.pub_cmd.publish(self.msg_vel)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()