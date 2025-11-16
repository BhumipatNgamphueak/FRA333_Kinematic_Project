# Hexapod Gait Framework Architecture

## Overview
The gait framework uses a 4-node architecture with proper separation of concerns:

```
/cmd_vel (Twist)
    ↓
┌─────────────────────────────────────────────────────────────────┐
│ 1. GAIT PLANNER (gait_planning.py) - 10 Hz                     │
│    - Receives velocity commands                                 │
│    - Selects gait type (tripod/wave/ripple)                    │
│    - Calculates step parameters                                │
├─────────────────────────────────────────────────────────────────┤
│ OUTPUT:                                                         │
│   /hexapod/gait_parameters (Float64MultiArray)                 │
│     [gait_type, step_height, step_length, cycle_time, duty]    │
│   /hexapod/body_velocity (Twist)                               │
└─────────────────────────────────────────────────────────────────┘
              ↓                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 2. STATE CONTROLLER (state_controller.py) - 50 Hz              │
│    - Manages global gait phase                                  │
│    - Calculates individual leg phases with offsets              │
│    - Determines stance/swing state for each leg                 │
├─────────────────────────────────────────────────────────────────┤
│ OUTPUT (for each leg 1-6):                                      │
│   /hexapod/leg_X/phase_info (Float64MultiArray)                │
│     [phase_type, progress, leg_phase]                           │
│     - phase_type: 0=stance, 1=swing                            │
│     - progress: 0.0-1.0 within current phase                   │
│     - leg_phase: 0.0-1.0 within gait cycle                     │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 3. SET POINT GENERATOR (set_point.py) - 50 Hz (ONE PER LEG)   │
│    - Subscribes to phase_info for THIS leg                     │
│    - Generates discrete foot positions                          │
│    - Stance: linear motion backward (foot on ground)            │
│    - Swing: parabolic arc forward (foot in air)                │
├─────────────────────────────────────────────────────────────────┤
│ OUTPUT:                                                         │
│   /hexapod/leg_X/end_effector_setpoint (PointStamped)          │
│     Discrete 3D position [x, y, z]                             │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 4. TRAJECTORY PLANNER (trajectory_planning.py) - 100 Hz        │
│    (ONE PER LEG)                                                │
│    - Receives discrete setpoints                                │
│    - Generates smooth cubic Hermite interpolation               │
│    - Produces continuous position & velocity                    │
├─────────────────────────────────────────────────────────────────┤
│ OUTPUT:                                                         │
│   /hexapod/leg_X/end_effector_target (PointStamped)            │
│     Smooth interpolated position                                │
│   /hexapod/leg_X/end_effector_velocity (Vector3Stamped)        │
│     Velocity vector                                             │
└─────────────────────────────────────────────────────────────────┘
```

## Node Responsibilities

### 1. Gait Planner (gait_planning.py) - 10 Hz
**Responsibility**: High-level gait selection and parameter calculation

**Inputs**:
- `/cmd_vel` (Twist) - Velocity commands from user/controller

**Outputs**:
- `/hexapod/gait_parameters` (Float64MultiArray)
  - `[0]` gait_type: 0=tripod, 1=wave, 2=ripple
  - `[1]` step_height: vertical clearance (m)
  - `[2]` step_length: horizontal stride (m)
  - `[3]` cycle_time: time for complete cycle (s)
  - `[4]` duty_factor: fraction of cycle in stance (0-1)
  
- `/hexapod/body_velocity` (Twist) - Filtered velocity commands

**Key Functions**:
- Velocity filtering and clipping
- Gait type selection based on speed
- Step parameter calculation
- Cycle time adaptation

---

### 2. State Controller (state_controller.py) - 50 Hz
**Responsibility**: Gait phase management and leg coordination

**Inputs**:
- `/hexapod/gait_parameters` (Float64MultiArray)
- `/hexapod/body_velocity` (Twist)

**Outputs** (6 publishers, one per leg):
- `/hexapod/leg_1/phase_info` through `/hexapod/leg_6/phase_info`
  - `[0]` phase_type: 0=stance, 1=swing
  - `[1]` progress: 0.0-1.0 within current phase
  - `[2]` leg_phase: 0.0-1.0 absolute phase in gait cycle

**Key Functions**:
- Global gait phase tracking
- Per-leg phase offset calculation
- Stance/swing determination
- Phase progress calculation

**Gait Phase Offsets**:
- **Tripod**: Legs [1,4,5] at phase 0.0, legs [2,3,6] at phase 0.5
- **Wave**: Sequential [0.0, 0.5, 1/6, 4/6, 2/6, 5/6]
- **Ripple**: Three pairs [0.0, 2/3, 1/3, 0.0, 2/3, 1/3]

---

### 3. Set Point Generator (set_point.py) - 50 Hz
**Responsibility**: Generate discrete foot positions based on phase

**Instances**: 6 nodes (one per leg with leg_id parameter)

**Inputs**:
- `/hexapod/leg_X/phase_info` (Float64MultiArray)
- `/hexapod/body_velocity` (Twist)
- `/hexapod/gait_parameters` (Float64MultiArray)

**Outputs**:
- `/hexapod/leg_X/end_effector_setpoint` (PointStamped)

**Key Functions**:
- **Stance phase**: Linear motion from front to back
  - `x = stance_x + step_length/2 - progress * step_length`
  - `z = stance_z` (on ground)
  
- **Swing phase**: Parabolic arc from back to front
  - `x = stance_x - step_length/2 + progress * step_length`
  - `z = stance_z + step_height * 4 * progress * (1 - progress)`

**Parameters per leg**:
- `leg_id`: 1-6
- `stance_x`, `stance_y`, `stance_z`: neutral foot position
- `update_rate`: 50.0 Hz

---

### 4. Trajectory Planner (trajectory_planning.py) - 100 Hz
**Responsibility**: Smooth interpolation between discrete setpoints

**Instances**: 6 nodes (one per leg)

**Inputs**:
- `/hexapod/leg_X/end_effector_setpoint` (PointStamped)

**Outputs**:
- `/hexapod/leg_X/end_effector_target` (PointStamped)
- `/hexapod/leg_X/end_effector_velocity` (Vector3Stamped)

**Key Functions**:
- Cubic Hermite interpolation
- Trapezoidal velocity profiling
- Trajectory buffer management
- Smooth position and velocity output

**Parameters**:
- `leg_id`: 1-6
- `vmax`: maximum velocity
- `amax`: maximum acceleration
- `trajectory_rate`: 100.0 Hz

---

## Data Flow Example

### Example: Walking Forward at 0.1 m/s

1. **Gait Planner** receives `/cmd_vel` with linear.x = 0.1
   - Selects tripod gait (type 0)
   - Calculates step_length = 0.05m, cycle_time = 1.0s, duty_factor = 0.5
   - Publishes to `/hexapod/gait_parameters` and `/hexapod/body_velocity`

2. **State Controller** receives gait parameters
   - Updates at 50 Hz, tracking global phase 0.0 → 1.0
   - For Leg 1 (phase offset 0.0):
     - At phase 0.0-0.5: stance (phase_type=0, progress 0→1)
     - At phase 0.5-1.0: swing (phase_type=1, progress 0→1)
   - For Leg 2 (phase offset 0.5):
     - At phase 0.0-0.5: swing
     - At phase 0.5-1.0: stance
   - Publishes phase_info for each leg at 50 Hz

3. **Set Point Generator** (Leg 1) receives phase_info
   - During stance (0.0-0.5s):
     - Generates positions moving from x=+0.025 to x=-0.025
     - Keeps z on ground
   - During swing (0.5-1.0s):
     - Generates positions moving from x=-0.025 to x=+0.025
     - Creates parabolic arc with peak at 0.03m height
   - Publishes discrete setpoint at 50 Hz

4. **Trajectory Planner** (Leg 1) receives setpoint
   - Interpolates smoothly between consecutive setpoints
   - Uses cubic Hermite with velocity continuity
   - Publishes smooth target position and velocity at 100 Hz

---

## Topic Summary

### Global Topics
- `/cmd_vel` → gait_planner
- `/hexapod/gait_parameters` → state_controller, set_point (×6)
- `/hexapod/body_velocity` → state_controller, set_point (×6)

### Per-Leg Topics (×6)
- `/hexapod/leg_X/phase_info` → set_point_X
- `/hexapod/leg_X/end_effector_setpoint` → trajectory_X
- `/hexapod/leg_X/end_effector_target` → [inverse kinematics]
- `/hexapod/leg_X/end_effector_velocity` → [inverse kinematics]

---

## Running the Framework

### Launch Order
```bash
# 1. Gait planner (1 node)
ros2 run hexapod gait_planning.py

# 2. State controller (1 node)
ros2 run hexapod state_controller.py

# 3. Set point generators (6 nodes, one per leg)
ros2 run hexapod set_point.py --ros-args -p leg_id:=1
ros2 run hexapod set_point.py --ros-args -p leg_id:=2
# ... repeat for legs 3-6

# 4. Trajectory planners (6 nodes, one per leg)
ros2 run hexapod trajectory_planning.py --ros-args -p leg_id:=1
ros2 run hexapod trajectory_planning.py --ros-args -p leg_id:=2
# ... repeat for legs 3-6
```

### Testing
```bash
# Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}}"

# Monitor gait parameters
ros2 topic echo /hexapod/gait_parameters

# Monitor leg 1 phase
ros2 topic echo /hexapod/leg_1/phase_info

# Monitor leg 1 setpoint
ros2 topic echo /hexapod/leg_1/end_effector_setpoint

# Monitor leg 1 smooth target
ros2 topic echo /hexapod/leg_1/end_effector_target
```

---

## Key Design Decisions

1. **Separation of Concerns**: Each node has a single, clear responsibility
2. **Hierarchical Rates**: 10 Hz (planning) → 50 Hz (coordination) → 100 Hz (execution)
3. **Scalability**: Easy to add new gaits or modify leg behavior
4. **Modularity**: Can test/debug each level independently
5. **Standardized Topics**: Uses ROS2 standard message types
6. **Per-Leg Instances**: Set point and trajectory nodes run independently per leg

---

## Future Enhancements

- Add terrain adaptation in set_point generator
- Implement body pose control (pitch/roll)
- Add dynamic gait switching during motion
- Integrate force feedback for stance phase
- Add obstacle avoidance in swing trajectories
