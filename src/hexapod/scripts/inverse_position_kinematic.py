#!/usr/bin/env python3
"""
Inverse Kinematics Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joints for seed)
INPUT: /hexapod/leg_X/end_effector_target (PointStamped - desired foot position)
OUTPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target joint angles)
Frequency: Callback-based (triggered by target)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('tolerance', 0.001)
        self.declare_parameter('use_analytical_ik', True)
        
        self.target_sub = self.create_subscription(
            PointStamped, 'end_effector_target', self.target_callback, 10)
        
        # OUTPUT: Publisher
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, 'joint_position_target', 10)
        
        self.get_logger().info('Inverse Kinematics initialized')
    
    
    def target_callback(self, msg):
        """INPUT: Receive target end effector position"""
        # TODO: Compute IK
        # TODO: Publish joint position target
        pass


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()