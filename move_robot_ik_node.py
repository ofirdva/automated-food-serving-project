# Created by Ofir Dvantman
# This script connects to abb_ros2_sim interfaces to control ABB IRB 1200
# Function: Using DH parameters for moving the simulated robot into the desired target position

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np

class MoveRobotIKNode(Node):
    def __init__(self):
        super().__init__('move_robot_ik_node')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.target_position_subscriber = self.create_subscription(Point, '/target_position', self.target_position_callback, 10)

        self.robot = DHRobot(
    [
        RevoluteDH(d=0.399, a=0.0, alpha=np.pi/2),         # Joint 1
        RevoluteDH(d=0.0, a=0.448, alpha=0, offset=-np.pi/2),  # Joint 2
        RevoluteDH(d=0.0, a=0.042, alpha=-np.pi/2),        # Joint 3
        RevoluteDH(d=0.451, a=0, alpha=np.pi/2),           # Joint 4
        RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2),          # Joint 5
        RevoluteDH(d=0.082, a=0.0, alpha=0, offset=-np.pi)   # Joint 6 (test increased d6)
    ],
    name='abb_irb1200'
)

        self.joint_limits = [
            (-2.967, 2.967),
            (-1.745, 2.269),
            (-3.491, 1.222),
            (-4.712, 4.712),
            (-2.269, 2.269),
            (-6.283, 6.283)
        ]

        self.current_joint_positions = np.zeros(6)
        self.target_joint_positions = None
        self.reached_target = False
        self.previous_position = np.zeros(6)
        self.stagnation_counter = 0
        self.stagnation_threshold = 5  # Number of cycles with minimal movement to consider stagnation

        self.timer = self.create_timer(0.1, self.update_joint_states)

    def target_position_callback(self, msg):
        self.get_logger().info(f'Received target position: ({msg.x}, {msg.y}, {msg.z})')

        # Construct SE3 transformation for the end-effector position
        target_position = SE3(msg.x, msg.y, msg.z)  # Target for the end-effector (joint 6)

        # Calculate inverse kinematics from the current joint positions
        ik_solution = self.robot.ikine_LM(target_position, q0=self.current_joint_positions)

        if ik_solution.success:
            self.target_joint_positions = np.clip(
                np.array(ik_solution.q),
                [limit[0] for limit in self.joint_limits],
                [limit[1] for limit in self.joint_limits]
            )
            self.reached_target = False  # Reset the flag so the robot will move to the new target
            self.stagnation_counter = 0  # Reset stagnation counter
            self.get_logger().info(f'IK solution: {self.target_joint_positions}')

            # Perform FK validation
            fk_result = self.robot.fkine(self.target_joint_positions)
            fk_position = fk_result.t
            self.get_logger().info(f'FK Full Result: {fk_result}')
            self.get_logger().info(f'FK Position: {fk_position}, Target: {target_position.t}')

            # Calculate error between FK and target
            error = np.linalg.norm(fk_position - target_position.t)
            self.get_logger().info(f'Position error (mm): {error * 1000:.2f}')
        else:
            self.get_logger().warn('IK solution not found')

    def update_joint_states(self):
        if self.target_joint_positions is None or self.reached_target:
            return  # Do nothing if there's no target or the target has been reached

        # Calculate the difference between current and target positions
        delta = self.target_joint_positions - self.current_joint_positions

        # Check if the target is reached within a small threshold
        if np.linalg.norm(delta) < 1e-2:  # Adjust threshold as necessary
            self.current_joint_positions = self.target_joint_positions
            self.reached_target = True  # Target has been reached
            self.get_logger().info('Target position reached. Stopping movement.')

            # Stop publishing joint states to avoid vibrations
            self.timer.cancel()

        else:
            # Calculate the step size for smooth interpolation
            step_size = 0.02  # Adjust this for smoother/faster movement
            max_delta = np.max(np.abs(delta))

            self.current_joint_positions += delta * (step_size / max_delta)

            # Check for stagnation by comparing the current position with the previous position
            if np.linalg.norm(self.current_joint_positions - self.previous_position) < 1e-6:
                self.stagnation_counter += 1
            else:
                self.stagnation_counter = 0  # Reset the counter if movement is detected

            if self.stagnation_counter >= self.stagnation_threshold:
                self.get_logger().error('Stagnation detected! Stopping the robot to avoid potential issues.')
                self.timer.cancel()  # Stop the timer to cease further movement

            self.previous_position = self.current_joint_positions.copy()

        # Publish the current joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i+1}' for i in range(6)]
        joint_state_msg.position = [float(pos) for pos in self.current_joint_positions]

        # Publish joint states
        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {joint_state_msg.position}')

    def start_movement(self):
        # Restart the timer if a new target is provided
        if not self.timer.is_canceled():
            self.timer.reset()
        else:
            self.timer = self.create_timer(0.1, self.update_joint_states)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

