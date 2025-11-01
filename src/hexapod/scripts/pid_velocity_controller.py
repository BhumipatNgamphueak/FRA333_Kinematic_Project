#!/usr/bin/env python3
"""
Velocity PID Controller Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joint velocities)
INPUT: /hexapod/leg_X/joint_velocity_target (Float64MultiArray - target velocities from position controller)
INPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - feedforward from IVK)
OUTPUT: /effort_controller_leg_X/commands (Float64MultiArray - effort/torque commands to Gazebo)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class VelocityPIDController(Node):
    def __init__(self):
        super().__init__('velocity_pid_controller')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('velocity_kp', [5.0, 5.0, 5.0])
        self.declare_parameter('velocity_ki', [0.5, 0.5, 0.5])
        self.declare_parameter('velocity_kd', [0.1, 0.1, 0.1])
        self.declare_parameter('effort_limit', 10.0)
        self.declare_parameter('control_rate', 100.0)
        
        control_rate = self.get_parameter('control_rate').value
        
        # INPUT: Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_target', self.target_callback, 10)
        self.feedforward_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_feedforward', self.feedforward_callback, 10)
        
        # OUTPUT: Publisher - to Gazebo
        self.effort_pub = self.create_publisher(Float64MultiArray, 'commands', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info('Velocity PID Controller initialized')
    
    def joint_state_callback(self, msg):
        """INPUT: Receive current joint velocities"""
        # TODO: Store current velocity
        pass

    def feedforward_callback(self, msg):
        """INPUT: Receive feedforward joint velocities"""
        # TODO: Store feedforward velocity
        pass
    
    def target_callback(self, msg):
        """INPUT: Receive target joint velocities"""
        # TODO: Store target velocity
        pass
    
    def control_loop(self):
        """OUTPUT: Main control loop - 100 Hz"""
        # TODO: Implement velocity PID
        # TODO: Add feedforward term
        # TODO: Publish effort command
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()