#!/usr/bin/env python3
"""
Joint State Splitter Node - SKELETON
Input: /joint_states (global joint states from Gazebo)
Output: /hexapod/leg_{1-6}/joint_states (filtered per leg)
Frequency: Callback-based (no timer)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('joint_state_splitter')
        
        # Parameters
        for i in range(1, 7):
            self.declare_parameter(f'leg_{i}_joints', 
                [f'hip_joint_{i}', f'knee_joint_{i}', f'ankle_joint_{i}'])
        
        # INPUT: Subscribe to global joint states
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # OUTPUT: Publishers (one per leg)
        self.leg_pubs = {}
        for i in range(1, 7):
            self.leg_pubs[i] = self.create_publisher(
                JointState, f'leg_{i}/joint_states', 10)
        
        self.get_logger().info('Joint State Splitter initialized')
    
    def joint_state_callback(self, msg):
        """
        INPUT: Full joint states from Gazebo
        OUTPUT: Split joint states to each leg
        TODO: Filter joints by name for each leg and publish
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()