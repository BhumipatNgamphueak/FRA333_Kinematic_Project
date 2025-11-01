#!/usr/bin/env python3
"""
Forward Kinematics Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - joint positions)
OUTPUT: /hexapod/leg_X/end_effector_position (PointStamped - computed foot position)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped


class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)
        
        update_rate = self.get_parameter('update_rate').value
        
        # INPUT: Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # OUTPUT: Publisher
        self.ee_position_pub = self.create_publisher(
            PointStamped, 'end_effector_position', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.compute_fk)
        
        self.get_logger().info('Forward Kinematics initialized')
    
    def joint_state_callback(self, msg):
        """INPUT: Receive joint positions"""
        # TODO: Store joint positions
        pass
    
    def compute_fk(self):
        """OUTPUT: Compute forward kinematics - 100 Hz"""
        # TODO: Compute end effector position from joint angles
        # TODO: Publish end effector position
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()