import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class MoveStraight(Node):
    def __init__(self):
        super().__init__('control_move_straight')
        self.pub_velocity = self.create_publisher(Float64MultiArray, 'velocity_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.time_start = None

    def timer_cb(self):
        now = self.get_clock().now()
        if self.time_start is None:
            self.time_start = now

        msg = Float64MultiArray()
        if (now - self.time_start).nanoseconds * 1e-9 >= 5:
            msg.data = [0.0, 0.0]
            self.pub_velocity.publish(msg)
            return

        w_left, w_right = 5.0, 5.0
        msg.data = [w_left, w_right]
        self.pub_velocity.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveStraight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()