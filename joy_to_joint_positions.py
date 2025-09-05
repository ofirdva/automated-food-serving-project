# Created by Ofir Dvantman
# This script connects to abb_ros2_sim interfaces to control ABB IRB 1200
# Function: Converts joystick input to joint positions 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

class JoyToJointPositionsNode(Node):
    def __init__(self):
        super().__init__('joy_to_joint_positions_node')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joy_subscriber = self.create_subscription(Joy, '/new_joy', self.joy_callback, 10)

        self.joint_limits = [
            (-2.967, 2.967),
            (-1.745, 2.269),
            (-3.491, 1.222),
            (-4.712, 4.712),
            (-2.269, 2.269),
            (-6.283, 6.283)
        ]
        self.current_joint_positions = [0.0] * 6

        self.timer = self.create_timer(0.1, self.publish_initial_positions)

    def map_axes_to_joint_positions(self, axes):
        joint_positions = []
        for i, axis in enumerate(axes[:6]):
            min_limit, max_limit = self.joint_limits[i]
            joint_position = (axis + 1) / 2 * (max_limit - min_limit) + min_limit
            joint_positions.append(joint_position)
        return joint_positions

    def joy_callback(self, msg):
        if len(msg.axes) < 6:
            self.get_logger().warn('Not enough axes data received')
            return

        self.current_joint_positions = self.map_axes_to_joint_positions(msg.axes)
        self.get_logger().info(f'Joint positions: {self.current_joint_positions}')

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i+1}' for i in range(6)]
        joint_state_msg.position = self.current_joint_positions

        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {joint_state_msg.position}')

    def publish_initial_positions(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i+1}' for i in range(6)]
        joint_state_msg.position = [0.0] * 6

        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f'Publishing initial joint states: {joint_state_msg.position}')

        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointPositionsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

