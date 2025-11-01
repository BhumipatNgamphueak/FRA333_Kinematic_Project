#!/usr/bin/env python3
"""
Inverse Velocity Kinematics Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joints for Jacobian)
INPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - desired foot velocity)
OUTPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - joint velocities)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray


class InverseVelocityKinematics(Node):
    def __init__(self):
        super().__init__('inverse_velocity_kinematics')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)
        
        update_rate = self.get_parameter('update_rate').value
        
        self.ee_vel_sub = self.create_subscription(
            Vector3Stamped, 'end_effector_velocity', self.ee_velocity_callback, 10)
        
        # OUTPUT: Publisher
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_feedforward', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.compute_velocity)
        
        self.get_logger().info('Inverse Velocity Kinematics initialized')
    
    def ee_velocity_callback(self, msg):
        """INPUT: Receive desired end effector velocity"""
        # TODO: Store EE velocity
        pass
    
    def compute_velocity(self):
        """OUTPUT: Compute joint velocities - 100 Hz"""
        # TODO: Compute Jacobian
        # TODO: Solve for joint velocities
        # TODO: Publish joint velocity feedforward
        pass


def main(args=None):
    rclpy.init(args=args)
    node = InverseVelocityKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()