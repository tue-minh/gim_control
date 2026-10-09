import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class TestPub(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(Float64MultiArray, '/impedance_gains', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [20.0, 20.0, 20.0, 2.0, 2.0, 2.0]
        self.pub.publish(msg)
        self.get_logger().info('Published')

def main():
    rclpy.init()
    node = TestPub()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
