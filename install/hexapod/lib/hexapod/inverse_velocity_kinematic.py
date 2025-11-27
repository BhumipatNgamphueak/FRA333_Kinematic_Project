#!/usr/bin/env python3
"""
Inverse Velocity Kinematics Node - FIXED WITH URDF RPY OFFSETS (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joints for Jacobian)
INPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - desired foot velocity in BASE_LINK frame)
OUTPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - joint velocities)
Frequency: 100 Hz

CRITICAL FIX: Jacobian now matches FK node's URDF-aware transformations
- Accounts for hip joint RPY offset (0, 0, -1.5708)
- Accounts for knee joint RPY offset (1.5708, 1.5708, 3.142)
- All computations in base_link frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class InverseVelocityKinematicsFixed(Node):
    def __init__(self):
        super().__init__('inverse_velocity_kinematics')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('damping_factor', 0.01)

        # Link lengths from xacro (in meters)
        self.declare_parameter('knee_joint_x', 0.078424)
        self.declare_parameter('knee_joint_y', -0.0031746)
        self.declare_parameter('knee_joint_z', 0.0010006)

        self.declare_parameter('ankle_joint_x', -0.087752)
        self.declare_parameter('ankle_joint_y', -0.081834)
        self.declare_parameter('ankle_joint_z', 0.0)

        self.declare_parameter('foot_pointer_joint_x', 0.18098)
        self.declare_parameter('foot_pointer_joint_y', -0.022156)
        self.declare_parameter('end_effector_joint_x', 0.0075528)
        self.declare_parameter('end_effector_joint_y', -0.00094278)

        # URDF offsets
        self.declare_parameter('hip_xyz_x', -1.2616e-05)
        self.declare_parameter('hip_xyz_y', -0.095255)
        self.declare_parameter('hip_xyz_z', 0.0)

        update_rate = self.get_parameter('update_rate').value
        leg_id = self.get_parameter('leg_id').value
        self.damping = self.get_parameter('damping_factor').value

        # Get link offsets
        self.knee_offset = np.array([
            self.get_parameter('knee_joint_x').value,
            self.get_parameter('knee_joint_y').value,
            self.get_parameter('knee_joint_z').value
        ])

        self.ankle_offset = np.array([
            self.get_parameter('ankle_joint_x').value,
            self.get_parameter('ankle_joint_y').value,
            self.get_parameter('ankle_joint_z').value
        ])

        self.foot_offset = np.array([
            self.get_parameter('foot_pointer_joint_x').value + self.get_parameter('end_effector_joint_x').value,
            self.get_parameter('foot_pointer_joint_y').value + self.get_parameter('end_effector_joint_y').value,
            0.0
        ])

        # URDF RPY offsets (CRITICAL: must match FK node)
        self.hip_xyz = np.array([
            self.get_parameter('hip_xyz_x').value,
            self.get_parameter('hip_xyz_y').value,
            self.get_parameter('hip_xyz_z').value
        ])
        self.hip_rpy = np.array([0.0, 0.0, -1.5708])  # From URDF
        self.knee_rpy = np.array([1.5708, 1.5708, 3.142])  # From URDF

        # State variables
        self.joint_angles = np.zeros(3)
        self.ee_velocity = np.zeros(3)
        self.joint_data_received = False
        self.ee_vel_received = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        self.ee_vel_sub = self.create_subscription(
            Vector3Stamped, 'end_effector_velocity', self.ee_velocity_callback, 10)

        # OUTPUT: Publisher
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_feedforward', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.compute_velocity)

        self.get_logger().info(f'Inverse Velocity Kinematics (URDF-aware) initialized for leg {leg_id}')
        self.get_logger().info('CRITICAL: Jacobian now matches FK node RPY transformations')

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint positions"""
        if len(msg.position) >= 3:
            self.joint_angles = np.array(msg.position[:3])
            self.joint_data_received = True

    def ee_velocity_callback(self, msg):
        """INPUT: Receive desired end effector velocity (in base_link frame)"""
        self.ee_velocity = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self.ee_vel_received = True

    def compute_velocity(self):
        """OUTPUT: Compute joint velocities using URDF-aware Jacobian - 100 Hz"""
        if not self.joint_data_received or not self.ee_vel_received:
            return

        theta1, theta2, theta3 = self.joint_angles

        # Compute URDF-aware Jacobian matrix (3x3)
        J = self.compute_jacobian_urdf_aware(theta1, theta2, theta3)

        # Solve for joint velocities using damped least squares
        # θ̇ = J^T(JJ^T + λ²I)^(-1)v
        JJT = J @ J.T
        damping_matrix = self.damping**2 * np.eye(3)

        try:
            joint_velocities = J.T @ np.linalg.inv(JJT + damping_matrix) @ self.ee_velocity
        except np.linalg.LinAlgError:
            self.get_logger().warn('Jacobian inversion failed, using pseudo-inverse')
            joint_velocities = np.linalg.pinv(J) @ self.ee_velocity

        # Publish joint velocities
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_velocities]
        self.joint_vel_pub.publish(msg)

    def compute_jacobian_urdf_aware(self, theta1, theta2, theta3):
        """
        Compute Jacobian matrix with URDF RPY offsets (MATCHES FK NODE)
        
        This is CRITICAL: The Jacobian must use the same frame transformations
        as the FK node to ensure consistency.
        
        J[i,j] = ∂position_i/∂theta_j in base_link frame
        """
        # URDF-aware rotation matrices (SAME AS FK NODE)
        R_hip_static = self.rpy_to_matrix(*self.hip_rpy)
        R_knee_static = self.rpy_to_matrix(*self.knee_rpy)
        
        # Frame 0 -> Frame 1 (Hip joint with RPY offset)
        R1 = R_hip_static @ self.rotation_z(theta1)
        p1 = self.hip_xyz + R1 @ self.knee_offset
        
        # Frame 1 -> Frame 2 (Knee joint with RPY offset)
        R2 = R1 @ R_knee_static @ self.rotation_z(theta2)
        p2 = p1 + R2 @ self.ankle_offset
        
        # Frame 2 -> Frame 3 (Ankle joint, no RPY offset)
        R3 = R2 @ self.rotation_z(theta3)
        p3 = p2 + R3 @ self.foot_offset  # End effector position in base_link
        
        # Joint axes in base_link frame
        # CRITICAL: Joint axes are Z-axis of joint frame AFTER RPY transform
        z0 = R_hip_static @ np.array([0, 0, 1])  # Hip axis after RPY
        z1 = R1 @ R_knee_static @ np.array([0, 0, 1])  # Knee axis after RPY
        z2 = R2 @ np.array([0, 0, 1])  # Ankle axis (no static RPY)
        
        # Jacobian columns: J_i = z_i × (p_e - p_i)
        # For revolute joints: v = ω × r
        J = np.zeros((3, 3))
        
        # Column 1: Effect of θ₁ (hip) on end effector
        # Position of hip joint in base frame is hip_xyz
        J[:, 0] = np.cross(z0, p3 - self.hip_xyz)
        
        # Column 2: Effect of θ₂ (knee) on end effector
        J[:, 1] = np.cross(z1, p3 - p1)
        
        # Column 3: Effect of θ₃ (ankle) on end effector
        J[:, 2] = np.cross(z2, p3 - p2)
        
        return J

    @staticmethod
    def rotation_x(theta):
        """Rotation matrix around X axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [1,  0,  0],
            [0,  c, -s],
            [0,  s,  c]
        ])

    @staticmethod
    def rotation_y(theta):
        """Rotation matrix around Y axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [ c, 0, s],
            [ 0, 1, 0],
            [-s, 0, c]
        ])

    @staticmethod
    def rotation_z(theta):
        """Rotation matrix around Z axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])

    def rpy_to_matrix(self, roll, pitch, yaw):
        """
        Convert RPY (roll-pitch-yaw) to rotation matrix
        Uses ZYX convention (yaw-pitch-roll order)
        """
        Rx = self.rotation_x(roll)
        Ry = self.rotation_y(pitch)
        Rz = self.rotation_z(yaw)
        return Rz @ Ry @ Rx


def main(args=None):
    rclpy.init(args=args)
    node = InverseVelocityKinematicsFixed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()