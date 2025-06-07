import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyRelayNode(Node):
    def __init__(self):
        super().__init__('joy_relay_node')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Joy, '/new_joy', 10)

    def joy_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info(f'Relayed Joy message: {msg}')  # Added logging

def main(args=None):
    rclpy.init(args=args)
    node = JoyRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
