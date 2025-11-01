#!/usr/bin/env python3
"""
Position PID Controller Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joint positions)
INPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target positions)
OUTPUT: /hexapod/leg_X/joint_velocity_target (Float64MultiArray - velocity commands)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PositionPIDController(Node):
    def __init__(self):
        super().__init__('position_pid_controller')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('position_kp', [10.0, 10.0, 10.0])
        self.declare_parameter('position_ki', [0.1, 0.1, 0.1])
        self.declare_parameter('position_kd', [1.0, 1.0, 1.0])
        self.declare_parameter('velocity_limit', 5.0)
        self.declare_parameter('control_rate', 100.0)
        
        control_rate = self.get_parameter('control_rate').value
        
        # INPUT: Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_position_target', self.target_callback, 10)
        
        # OUTPUT: Publisher
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_target', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info('Position PID Controller initialized')
    
    def joint_state_callback(self, msg):
        """INPUT: Receive current joint states"""
        # TODO: Store current position
        pass
    
    def target_callback(self, msg):
        """INPUT: Receive target joint positions"""
        # TODO: Store target position
        pass
    
    def control_loop(self):
        """OUTPUT: Main control loop - 100 Hz"""
        # TODO: Implement PID control
        # TODO: Publish velocity command
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PositionPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()