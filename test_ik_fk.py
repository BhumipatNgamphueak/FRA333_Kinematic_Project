#!/usr/bin/env python3
"""Test script to verify FK/IK consistency"""

import numpy as np

# URDF Parameters
knee_offset = np.array([0.078424, -0.0031746, 0.0010006])
ankle_offset = np.array([-0.087752, -0.081834, 0.0])
foot_offset = np.array([0.18098 + 0.0075528, -0.022156 + -0.00094278, 0.0])
hip_xyz = np.array([-1.2616e-05, -0.095255, 0.0])
hip_rpy = np.array([0.0, 0.0, -1.5708])

def rotation_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rotation_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotation_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def rpy_to_matrix(roll, pitch, yaw):
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)

def forward_kinematics(theta1, theta2, theta3):
    """Forward kinematics (URDF-aware)"""
    # Hip joint
    R_hip_static = rpy_to_matrix(*hip_rpy)
    R1 = R_hip_static @ rotation_z(theta1)
    T1 = hip_xyz + R1 @ knee_offset
    
    # Knee joint
    R_knee_static = rpy_to_matrix(1.5708, 1.5708, 3.142)
    R2 = R1 @ R_knee_static @ rotation_z(theta2)
    T2 = T1 + R2 @ ankle_offset
    
    # Ankle joint
    R3 = R2 @ rotation_z(theta3)
    T3 = T2 + R3 @ foot_offset
    
    return T3

# Test with target position
target = np.array([0.0, -0.275255, -0.05])  # attachment + offset
print(f"Target position: {target}")
print(f"  X: {target[0]:.6f}")
print(f"  Y: {target[1]:.6f}")
print(f"  Z: {target[2]:.6f}")
print()

# Test with zero joint angles
theta_zero = np.array([0.0, 0.0, 0.0])
pos_zero = forward_kinematics(*theta_zero)
print(f"FK at [0, 0, 0]: {pos_zero}")
print(f"  X: {pos_zero[0]:.6f}")
print(f"  Y: {pos_zero[1]:.6f}")
print(f"  Z: {pos_zero[2]:.6f}")
print()

# Test with typical stance angles
theta_stance = np.array([0.0, 1.8, 1.2])
pos_stance = forward_kinematics(*theta_stance)
print(f"FK at [0, 1.8, 1.2]: {pos_stance}")
print(f"  X: {pos_stance[0]:.6f}")
print(f"  Y: {pos_stance[1]:.6f}")
print(f"  Z: {pos_stance[2]:.6f}")
print()

# Calculate link lengths
L1 = np.linalg.norm(knee_offset)
L2 = np.linalg.norm(ankle_offset)
L3 = np.linalg.norm(foot_offset)
print(f"Link lengths:")
print(f"  L1 (hip-knee): {L1:.6f} m = {L1*1000:.2f} mm")
print(f"  L2 (knee-ankle): {L2:.6f} m = {L2*1000:.2f} mm")
print(f"  L3 (ankle-foot): {L3:.6f} m = {L3*1000:.2f} mm")
print(f"  Total reach: {(L1+L2+L3):.6f} m = {(L1+L2+L3)*1000:.2f} mm")
