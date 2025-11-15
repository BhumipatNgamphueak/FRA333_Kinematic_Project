# Hexapod Control System - Node Architecture Summary

## System Overview
This is a hierarchical control system for a 6-legged hexapod robot with **10 nodes total**:
- **3 Global Nodes** - coordinate all legs
- **7 Per-Leg Nodes** × 6 legs = **42 nodes** (only 1 per leg for now)

---

## Data Flow Diagram

```
/cmd_vel (User Input)
    ↓
[Gait Planner] (10 Hz)
    ↓ gait_parameters, body_velocity
    ↓
[State Machine] (50 Hz)
    ↓ phase_info (6 topics, one per leg)
    ↓
    ├─→ [Set Point Gen - Leg 1] (50 Hz) ──→ setpoint ──→ [Trajectory - Leg 1] (100 Hz)
    ├─→ [Set Point Gen - Leg 2] (50 Hz) ──→ setpoint ──→ [Trajectory - Leg 2] (100 Hz)
    ├─→ [Set Point Gen - Leg 3] (50 Hz) ──→ setpoint ──→ [Trajectory - Leg 3] (100 Hz)
    ├─→ [Set Point Gen - Leg 4] (50 Hz) ──→ setpoint ──→ [Trajectory - Leg 4] (100 Hz)
    ├─→ [Set Point Gen - Leg 5] (50 Hz) ──→ setpoint ──→ [Trajectory - Leg 5] (100 Hz)
    └─→ [Set Point Gen - Leg 6] (50 Hz) ──→ setpoint ──→ [Trajectory - Leg 6] (100 Hz)

For each leg:
    target_position → [IK] → joint_target → [Position PID] → velocity_target
                                                                      ↓
    target_velocity → [IVK] → velocity_feedforward ─────────→ [Velocity PID] → effort
                                                                      ↑
    joint_states ────────────────────────────────────────────────────┘
```

---

## Global Nodes (3 nodes)

### 1. Gait Planner
**File**: `gait_planning.py`  
**Frequency**: 10 Hz  
**Input**: 
- `/cmd_vel` (Twist) - velocity commands from teleop/navigation
**Output**:
- `/hexapod/gait_parameters` (Float64MultiArray) - [gait_type, step_height, step_length, cycle_time, duty_factor]
- `/hexapod/body_velocity` (Twist) - filtered/clipped velocity
**Function**: Selects gait pattern based on velocity and calculates step parameters

### 2. State Machine
**File**: `state_controller.py`  
**Frequency**: 50 Hz  
**Input**:
- `/hexapod/gait_parameters` (Float64MultiArray)
- `/hexapod/body_velocity` (Twist)
**Output**:
- `/hexapod/leg_{1-6}/phase_info` (Float64MultiArray) - [phase_type, progress, leg_phase]
**Function**: Tracks gait cycle time and generates phase information for each leg (stance/swing)

### 3. Joint State Splitter
**File**: `joint_state_splitter.py`  
**Frequency**: Callback-based  
**Input**:
- `/joint_states` (JointState) - all joints from Gazebo
**Output**:
- `/hexapod/leg_{1-6}/joint_states` (JointState) - filtered per leg
**Function**: Distributes global joint states to individual legs

---

## Per-Leg Nodes (7 nodes × 6 legs = 42 nodes)

Each leg has its own instance of these 7 nodes:

### 4. Set Point Generator
**File**: `set_point.py`  
**Frequency**: 50 Hz  
**Input**:
- `/hexapod/leg_X/phase_info` (Float64MultiArray) - swing/stance phase
- `/hexapod/body_velocity` (Twist) - for step length calculation
**Output**:
- `/hexapod/leg_X/end_effector_setpoint` (PointStamped) - discrete foot position
**Function**: Generates discrete waypoints for swing and stance trajectories

### 5. Trajectory Generator
**File**: `trajectory_planning.py`  
**Frequency**: 100 Hz  
**Input**:
- `/hexapod/leg_X/end_effector_setpoint` (PointStamped) - discrete waypoints
- `/hexapod/leg_X/phase_info` (Float64MultiArray) - for interpolation timing
**Output**:
- `/hexapod/leg_X/end_effector_target` (PointStamped) - smooth interpolated position
- `/hexapod/leg_X/end_effector_velocity` (Vector3Stamped) - velocity for feedforward
**Function**: Smoothly interpolates between setpoints using cubic/quintic splines

### 6. Inverse Position Kinematics
**File**: `inverse_position_kinematic.py`  
**Frequency**: Callback-based (triggered by target)  
**Input**:
- `/hexapod/leg_X/end_effector_target` (PointStamped) - desired foot position
- `/hexapod/leg_X/joint_states` (JointState) - for seed values
**Output**:
- `/hexapod/leg_X/joint_position_target` (Float64MultiArray) - target joint angles
**Function**: Converts Cartesian foot position to joint angles (analytical solution)

### 7. Inverse Velocity Kinematics
**File**: `inverse_velocity_kinematic.py`  
**Frequency**: 100 Hz  
**Input**:
- `/hexapod/leg_X/end_effector_velocity` (Vector3Stamped) - desired foot velocity
- `/hexapod/leg_X/joint_states` (JointState) - current joints for Jacobian
**Output**:
- `/hexapod/leg_X/joint_velocity_feedforward` (Float64MultiArray) - joint velocities
**Function**: Maps Cartesian velocity to joint velocities using Jacobian (damped least squares)

### 8. Position PID Controller
**File**: `pid_position_controller.py`  
**Frequency**: 100 Hz  
**Input**:
- `/hexapod/leg_X/joint_states` (JointState) - current joint positions
- `/hexapod/leg_X/joint_position_target` (Float64MultiArray) - target positions from IK
**Output**:
- `/hexapod/leg_X/joint_velocity_target` (Float64MultiArray) - velocity commands
**Function**: Position control loop with PID (outputs velocity)

### 9. Velocity PID Controller
**File**: `pid_velocity_controller.py`  
**Frequency**: 100 Hz  
**Input**:
- `/hexapod/leg_X/joint_states` (JointState) - current joint velocities
- `/hexapod/leg_X/joint_velocity_target` (Float64MultiArray) - from position controller
- `/hexapod/leg_X/joint_velocity_feedforward` (Float64MultiArray) - from IVK
**Output**:
- `/effort_controller_leg_X/commands` (Float64MultiArray) - torque to Gazebo
**Function**: Velocity control loop with PID + feedforward (outputs torque)

### 10. Forward Kinematics (Optional for monitoring)
**File**: `forward_position_kinematic.py`  
**Frequency**: 100 Hz  
**Input**:
- `/hexapod/leg_X/joint_states` (JointState) - joint positions
**Output**:
- `/hexapod/leg_X/end_effector_position` (PointStamped) - computed foot position
**Function**: Computes actual foot position for verification/visualization

---

## Control Hierarchy

```
Level 1: High-Level Planning (10 Hz)
    └─ Gait Planner: Select gait pattern

Level 2: Coordination (50 Hz)
    └─ State Machine: Coordinate leg phases
    └─ Set Point Generator: Generate discrete waypoints

Level 3: Trajectory Generation (100 Hz)
    └─ Trajectory Generator: Smooth interpolation

Level 4: Inverse Kinematics (100 Hz or callback)
    ├─ IK: Position mapping (Cartesian → Joint space)
    └─ IVK: Velocity mapping (feedforward)

Level 5: Control (100 Hz)
    ├─ Position PID: Position tracking
    └─ Velocity PID: Torque generation
```

---

## Key Design Features

1. **Hierarchical Control**: Clear separation of planning, coordination, and low-level control

2. **Topic-Based Communication**: ROS2 topics allow flexible node interconnection

3. **Per-Leg Independence**: Each leg has its own control stack (7 nodes)

4. **Feedforward + Feedback**: Combines velocity feedforward (IVK) with PID feedback

5. **Smooth Trajectories**: Cubic/quintic spline interpolation for smooth motion

6. **Frequency Separation**: 
   - Planning: 10 Hz (slow)
   - Coordination: 50 Hz (medium)
   - Control: 100 Hz (fast)

---

## Fixed Data Flow Issue

**Original Problem**: Set point generator was publishing to all 6 legs in the skeleton

**Solution**: Each leg now has its own set_point generator instance that:
- Subscribes to its own `/hexapod/leg_X/phase_info`
- Publishes to its own `/hexapod/leg_X/end_effector_setpoint`

This ensures proper one-to-one mapping:
```
state_machine → phase_info → set_point → setpoint → trajectory
```

---

## Launch File
The `simple_launch.py` file launches all nodes with proper remapping (currently configured for leg 1 only, expand the range to 1-6 for all legs).

---

## Next Steps for Implementation

1. Implement forward kinematics calculation
2. Implement inverse kinematics (analytical solution)
3. Implement Jacobian calculation for IVK
4. Implement PID control logic
5. Implement trajectory interpolation
6. Implement set point generation (swing/stance)
7. Test with Gazebo simulation
