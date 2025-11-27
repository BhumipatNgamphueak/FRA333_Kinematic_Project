#!/usr/bin/env python3
"""
Velocity PID Controller Node with GRAVITY COMPENSATION - INNER LOOP (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joint velocities & positions)
INPUT: /hexapod/leg_X/joint_velocity_target (Float64MultiArray - target from position PID)
INPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - from IVK)
OUTPUT: /effort_controller_leg_X/commands (Float64MultiArray - torques to Gazebo)
Frequency: 100 Hz

CASCADE CONTROL WITH FEEDFORWARD + GRAVITY COMPENSATION:
  Position PID → Velocity Target ──┬──→ [Velocity PID] → Effort → Robot
                                    │      (THIS NODE)
  IVK Jacobian → Feedforward ───────┘             ↓
  Pinocchio → Gravity Compensation ───────────────┘

  Desired Velocity = Target + Feedforward
  Error = Desired - Current
  Torque = PID(Error) + Gravity_Compensation(q)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

# Pinocchio for gravity compensation
try:
    import pinocchio as pin
    from pinocchio.utils import zero
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("WARNING: Pinocchio not available. Gravity compensation disabled.")


class VelocityPIDControllerWithGravity(Node):
    def __init__(self):
        super().__init__('velocity_pid_controller')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('velocity_kp', [10.0, 10.0, 10.0])
        self.declare_parameter('velocity_ki', [0.5, 0.5, 0.5])
        self.declare_parameter('velocity_kd', [0.1, 0.1, 0.1])
        self.declare_parameter('effort_limit', 10.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('integral_limit', 1.0)
        self.declare_parameter('use_feedforward', True)
        self.declare_parameter('use_gravity_compensation', True)  # NEW PARAMETER
        self.declare_parameter('urdf_path', '')  # Path to URDF for Pinocchio

        leg_id = self.get_parameter('leg_id').value
        control_rate = self.get_parameter('control_rate').value

        # Get PID gains
        self.kp = np.array(self.get_parameter('velocity_kp').value)
        self.ki = np.array(self.get_parameter('velocity_ki').value)
        self.kd = np.array(self.get_parameter('velocity_kd').value)
        self.effort_limit = self.get_parameter('effort_limit').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.use_feedforward = self.get_parameter('use_feedforward').value
        self.use_gravity_comp = self.get_parameter('use_gravity_compensation').value

        # Control loop timing
        self.dt = 1.0 / control_rate

        # State variables (3 joints)
        self.current_position = np.zeros(3)  # NEW: Need position for gravity comp
        self.current_velocity = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.feedforward_velocity = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.integral_error = np.zeros(3)

        # Flags
        self.position_received = False
        self.velocity_received = False
        self.target_received = False
        self.feedforward_received = False

        # Pinocchio model setup
        self.pinocchio_model = None
        self.pinocchio_data = None
        self.leg_id = leg_id
        
        if PINOCCHIO_AVAILABLE and self.use_gravity_comp:
            self._setup_pinocchio_model()

        # INPUT: Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_target', self.target_callback, 10)
        self.feedforward_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_feedforward', self.feedforward_callback, 10)

        # OUTPUT: Publisher to Gazebo
        self.effort_pub = self.create_publisher(Float64MultiArray, 'commands', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f'Velocity PID Controller (with Gravity Comp) initialized for leg {leg_id}')
        self.get_logger().info(f'Control rate: {control_rate} Hz (dt={self.dt*1000:.2f}ms)')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Effort limit: ±{self.effort_limit} Nm')
        self.get_logger().info(f'Feedforward: {"ENABLED" if self.use_feedforward else "DISABLED"}')
        self.get_logger().info(f'Gravity Compensation: {"ENABLED" if self.use_gravity_comp and PINOCCHIO_AVAILABLE else "DISABLED"}')

    def _setup_pinocchio_model(self):
        """Initialize Pinocchio model for gravity compensation"""
        try:
            urdf_path = self.get_parameter('urdf_path').value
            
            if not urdf_path:
                # Try to load from default ROS package path
                urdf_path = '/mnt/project/hexapod.urdf'
            
            # Build model from URDF
            self.pinocchio_model = pin.buildModelFromUrdf(urdf_path)
            self.pinocchio_data = self.pinocchio_model.createData()
            
            # Get joint IDs for this leg
            self.joint_names = [
                f'hip_joint_{self.leg_id}',
                f'knee_joint_{self.leg_id}',
                f'ankle_joint_{self.leg_id}'
            ]
            
            # Get joint indices in the Pinocchio model
            self.joint_ids = []
            for joint_name in self.joint_names:
                if self.pinocchio_model.existJointName(joint_name):
                    joint_id = self.pinocchio_model.getJointId(joint_name)
                    self.joint_ids.append(joint_id)
                else:
                    self.get_logger().warn(f'Joint {joint_name} not found in URDF')
            
            if len(self.joint_ids) != 3:
                self.get_logger().error(f'Expected 3 joints, found {len(self.joint_ids)}')
                self.pinocchio_model = None
                return
            
            self.get_logger().info('Pinocchio model loaded successfully')
            self.get_logger().info(f'Joint IDs: {self.joint_ids}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load Pinocchio model: {str(e)}')
            self.pinocchio_model = None

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint positions and velocities"""
        if len(msg.position) >= 3 and len(msg.velocity) >= 3:
            self.current_position = np.array(msg.position[:3])  # [hip, knee, ankle]
            self.current_velocity = np.array(msg.velocity[:3])
            self.position_received = True
            self.velocity_received = True

    def target_callback(self, msg):
        """INPUT: Receive target velocities from Position PID"""
        if len(msg.data) >= 3:
            self.target_velocity = np.array(msg.data[:3])
            self.target_received = True

    def feedforward_callback(self, msg):
        """INPUT: Receive feedforward velocities from IVK"""
        if len(msg.data) >= 3:
            self.feedforward_velocity = np.array(msg.data[:3])
            self.feedforward_received = True

    def compute_gravity_compensation(self, q):
        """
        Compute gravity compensation torques using Pinocchio
        
        Args:
            q: Joint positions [hip, knee, ankle] for this leg
            
        Returns:
            tau_gravity: Gravity compensation torques (3,)
        """
        if not PINOCCHIO_AVAILABLE or self.pinocchio_model is None:
            return np.zeros(3)
        
        try:
            # Build full configuration vector for the robot
            # We need to set positions for ALL joints in the model
            q_full = zero(self.pinocchio_model.nq)
            
            # Set this leg's joint positions
            for i, joint_id in enumerate(self.joint_ids):
                # Pinocchio uses idx_q for position indices
                q_idx = self.pinocchio_model.joints[joint_id].idx_q
                if q_idx < len(q_full):
                    q_full[q_idx] = q[i]
            
            # Compute gravity vector using RNEA with zero velocity and acceleration
            v_full = zero(self.pinocchio_model.nv)
            a_full = zero(self.pinocchio_model.nv)
            
            # RNEA: Recursive Newton-Euler Algorithm
            # tau = RNEA(q, v, a) with v=0, a=0 gives us gravity torques
            tau_full = pin.rnea(self.pinocchio_model, self.pinocchio_data, q_full, v_full, a_full)
            
            # Extract torques for this leg's joints
            tau_gravity = np.zeros(3)
            for i, joint_id in enumerate(self.joint_ids):
                v_idx = self.pinocchio_model.joints[joint_id].idx_v
                if v_idx < len(tau_full):
                    tau_gravity[i] = tau_full[v_idx]
            
            return tau_gravity
            
        except Exception as e:
            self.get_logger().error(f'Gravity compensation failed: {str(e)}')
            return np.zeros(3)

    def control_loop(self):
        """
        OUTPUT: Main PID control loop with feedforward and gravity compensation - 100 Hz

        Control Law:
          desired_vel = target_vel + feedforward_vel
          error = desired_vel - current_vel
          tau_pid = Kp*error + Ki*∫error + Kd*d(error)/dt
          tau_gravity = g(q)  [from Pinocchio]
          tau_total = tau_pid + tau_gravity

        Benefits:
          - Feedforward: Faster response, reduced tracking error
          - Gravity Comp: Compensates for gravitational forces, better tracking
        """
        if not self.velocity_received or not self.position_received or not self.target_received:
            return

        # Combine target velocity with feedforward (if enabled)
        if self.use_feedforward and self.feedforward_received:
            desired_velocity = self.target_velocity + self.feedforward_velocity
        else:
            desired_velocity = self.target_velocity

        # Compute velocity error
        error = desired_velocity - self.current_velocity

        # Integral term (with anti-windup)
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error,
                                      -self.integral_limit,
                                      self.integral_limit)

        # Derivative term
        derivative = (error - self.previous_error) / self.dt

        # PID output (effort/torque command)
        tau_pid = (self.kp * error +
                   self.ki * self.integral_error +
                   self.kd * derivative)

        # Compute gravity compensation
        tau_gravity = np.zeros(3)
        if self.use_gravity_comp:
            tau_gravity = self.compute_gravity_compensation(self.current_position)

        # Total effort: PID + Gravity Compensation
        effort_command = tau_pid + tau_gravity

        # Apply effort limits (safety)
        effort_command = np.clip(effort_command,
                                -self.effort_limit,
                                self.effort_limit)

        # Update previous error
        self.previous_error = error.copy()

        # Publish effort to Gazebo
        msg = Float64MultiArray()
        msg.data = [float(e) for e in effort_command]
        self.effort_pub.publish(msg)

        # Debug logging (reduced frequency)
        if self.get_clock().now().nanoseconds % 1000000000 < 10000000:  # ~10 Hz logging
            self.get_logger().debug(
                f'τ_PID=[{tau_pid[0]:.2f}, {tau_pid[1]:.2f}, {tau_pid[2]:.2f}], '
                f'τ_grav=[{tau_gravity[0]:.2f}, {tau_gravity[1]:.2f}, {tau_gravity[2]:.2f}], '
                f'τ_total=[{effort_command[0]:.2f}, {effort_command[1]:.2f}, {effort_command[2]:.2f}]'
            )

    def reset_controller(self):
        """Reset PID state"""
        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPIDControllerWithGravity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()