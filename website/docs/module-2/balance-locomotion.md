---
sidebar_position: 1
---

# Balance and Locomotion Control

## Introduction to Bipedal Locomotion

Bipedal locomotion is one of the most challenging problems in humanoid robotics. Unlike wheeled or tracked robots, humanoid robots must maintain balance while walking, running, or performing other dynamic movements. This requires sophisticated control strategies that can handle the inherent instability of bipedal gait while adapting to various terrains and disturbances.

### The Challenge of Bipedal Walking

Human walking is a complex dynamic process that involves:
- Continuous balance maintenance
- Coordinated multi-joint movements
- Adaptation to terrain variations
- Energy-efficient motion patterns
- Disturbance rejection

### Key Concepts in Locomotion

#### Zero Moment Point (ZMP)
The ZMP is a critical concept in bipedal robotics, representing the point where the net moment of the ground reaction force is zero. For stable walking, the ZMP must remain within the support polygon defined by the feet.

#### Capture Point
The capture point indicates where the center of mass (CoM) will come to rest given the current momentum. It's crucial for balance recovery strategies.

#### Support Phases
- **Double Support**: Both feet on the ground
- **Single Support**: One foot on the ground
- **Flight Phase**: Neither foot on the ground (running)

## Mathematical Foundations

### ZMP Calculation

The ZMP coordinates are calculated as:

```
ZMP_x = (Σ(m_i * (ẍ_i * h - z_i * ẍ_i)) / Σ(m_i * g + m_i * z̈_i))
ZMP_y = (Σ(m_i * (ÿ_i * h - z_i * ÿ_i)) / Σ(m_i * g + m_i * z̈_i))
```

Where:
- h: Height of CoM above ground
- g: Gravitational acceleration
- m_i: Mass of link i
- ẍ_i, ÿ_i: Accelerations of link i
- z_i: Height of link i above ground

### Linear Inverted Pendulum Model (LIPM)

The LIPM simplifies bipedal dynamics by modeling the robot as a point mass supported by a massless leg:

```
ẍ = g/h * (x - x_zmp)
```

Where:
- x: CoM position
- x_zmp: ZMP position
- h: CoM height
- g: Gravitational acceleration

## Control Strategies

### 1. ZMP-Based Control

ZMP-based control maintains stability by tracking a desired ZMP trajectory:

```python
class ZMPController:
    def __init__(self):
        self.kp = 100.0  # Proportional gain
        self.kd = 20.0   # Derivative gain
        self.com_height = 0.8  # m

    def compute_control(self, current_zmp, desired_zmp, current_com, current_com_vel):
        # ZMP error
        zmp_error = desired_zmp - current_zmp

        # CoM acceleration command (inverted pendulum model)
        com_acc_desired = (self.gravity / self.com_height) * (current_com - desired_zmp)

        # Feedback control
        com_acc_feedback = self.kp * zmp_error + self.kd * current_com_vel

        return com_acc_desired + com_acc_feedback
```

### 2. Capture Point Control

Capture point control focuses on bringing the CoM to a safe position:

```python
class CapturePointController:
    def __init__(self):
        self.com_height = 0.8
        self.gravity = 9.81

    def compute_capture_point(self, com_pos, com_vel):
        """Calculate where CoM will come to rest"""
        omega = np.sqrt(self.gravity / self.com_height)
        return com_pos + com_vel / omega

    def is_capturable(self, capture_point, support_polygon):
        """Check if capture point is within support polygon"""
        cp_x, cp_y = capture_point
        min_x, max_x, min_y, max_y = support_polygon

        return (min_x <= cp_x <= max_x) and (min_y <= cp_y <= max_y)

    def compute_safe_footstep(self, current_com, current_com_vel, support_foot):
        """Compute safe footstep location"""
        capture_point = self.compute_capture_point(current_com, current_com_vel)

        # Ensure capture point is within safe region
        safe_x = np.clip(capture_point[0], support_foot[0] - 0.1, support_foot[0] + 0.3)
        safe_y = np.clip(capture_point[1], support_foot[1] - 0.2, support_foot[1] + 0.2)

        return [safe_x, safe_y]
```

### 3. Whole-Body Control

Whole-body control coordinates multiple tasks simultaneously:

```python
class WholeBodyController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.tasks = []

    def add_balance_task(self, desired_com_pos, weight=1.0):
        """Add center of mass balance task"""
        task = {
            'type': 'com',
            'desired': desired_com_pos,
            'weight': weight
        }
        self.tasks.append(task)

    def add_foot_task(self, foot_name, desired_pos, weight=1.0):
        """Add foot placement task"""
        task = {
            'type': 'foot',
            'name': foot_name,
            'desired': desired_pos,
            'weight': weight
        }
        self.tasks.append(task)

    def compute_joint_commands(self, current_state):
        """Compute joint commands using quadratic programming"""
        # Formulate QP problem
        # minimize: ||Ax - b||^2
        # subject to: Cx <= d

        # This would typically use a QP solver like CVXOPT
        pass
```

## Walking Pattern Generation

### 1. Preview Control

Preview control uses future reference trajectories to improve tracking:

```python
class PreviewController:
    def __init__(self, preview_steps=20, dt=0.01):
        self.preview_steps = preview_steps
        self.dt = dt
        self.gravity = 9.81
        self.com_height = 0.8

        # Precompute preview gains
        self.compute_preview_gains()

    def compute_preview_gains(self):
        """Compute gains for preview control"""
        omega = np.sqrt(self.gravity / self.com_height)

        # State-space representation
        A = np.array([[0, 1], [omega**2, 0]])
        B = np.array([0, -omega**2])

        # Discrete-time system
        dt = self.dt
        self.Ad = np.array([[1, np.sinh(omega*dt)/omega],
                           [omega*np.sinh(omega*dt), np.cosh(omega*dt)]])
        self.Bd = np.array([dt + np.sinh(omega*dt)/omega - np.cosh(omega*dt)/omega,
                           omega*(np.cosh(omega*dt) - 1)])

    def compute_control(self, current_com, future_reference):
        """Compute control with preview of future reference"""
        # Implementation of preview control algorithm
        pass
```

### 2. Footstep Planning

Footstep planning determines where and when to place feet:

```python
class FootstepPlanner:
    def __init__(self):
        self.step_length = 0.3  # m
        self.step_width = 0.2   # m
        self.step_height = 0.1  # m (for swing phase)

    def plan_footsteps(self, start_pos, goal_pos, terrain_map=None):
        """Plan sequence of footsteps to reach goal"""
        footsteps = []

        # Calculate number of steps needed
        distance = np.linalg.norm(np.array(goal_pos) - np.array(start_pos))
        num_steps = int(distance / self.step_length) + 1

        # Generate step positions
        for i in range(num_steps):
            # Alternating feet
            foot_offset = (-1)**i * self.step_width / 2

            # Interpolate position
            t = (i + 1) / (num_steps + 1)
            step_x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
            step_y = start_pos[1] + foot_offset

            footsteps.append([step_x, step_y, 0.0])  # x, y, z

        return footsteps

    def generate_swing_trajectory(self, start_pos, end_pos, height=0.1):
        """Generate smooth swing trajectory for foot"""
        # 5th order polynomial trajectory
        duration = 0.8  # seconds
        t = np.linspace(0, duration, int(duration/0.01))

        # Polynomial coefficients for smooth trajectory
        c0 = start_pos
        c1 = 0  # zero velocity at start
        c2 = 0  # zero acceleration at start
        c3 = 0  # zero velocity at end
        c4 = 0  # zero acceleration at end
        c5 = 0

        # Calculate coefficients for 5th order polynomial
        tf = duration
        c5 = 6 * (end_pos - start_pos) / tf**5
        c4 = -15 * (end_pos - start_pos) / tf**4
        c3 = 10 * (end_pos - start_pos) / tf**3

        trajectory = []
        for ti in t:
            pos = c0 + c1*ti + c2*ti**2 + c3*ti**3 + c4*ti**4 + c5*ti**5
            trajectory.append(pos)

        return trajectory
```

## Advanced Control Techniques

### 1. Model Predictive Control (MPC)

MPC optimizes future behavior based on a model of the system:

```python
class ModelPredictiveController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self.gravity = 9.81
        self.com_height = 0.8

    def solve_mpc(self, current_state, reference_trajectory):
        """Solve MPC optimization problem"""
        # Define optimization variables
        # Minimize: sum of (state_error^2 + control_effort^2)
        # Subject to: system dynamics, constraints

        # This would typically use an optimization solver
        # like CasADi or ACADO

        # Simplified approach using scipy.optimize
        from scipy.optimize import minimize

        def cost_function(control_sequence):
            total_cost = 0
            state = current_state.copy()

            for i in range(self.horizon):
                # Apply control and simulate forward
                state = self.predict_next_state(state, control_sequence[i])

                # Add cost for state deviation
                state_error = state - reference_trajectory[i]
                total_cost += np.sum(state_error**2)

                # Add control effort cost
                total_cost += 0.1 * np.sum(control_sequence[i]**2)

            return total_cost

        # Initial guess for control sequence
        initial_control = np.zeros((self.horizon, 2))  # 2D control

        # Solve optimization
        result = minimize(cost_function, initial_control.flatten())

        # Return first control action
        optimal_control = result.x[:2]
        return optimal_control

    def predict_next_state(self, state, control_input):
        """Predict next state using inverted pendulum model"""
        com_x, com_y, com_x_dot, com_y_dot = state
        zmp_x, zmp_y = control_input

        # Inverted pendulum dynamics
        com_x_ddot = self.gravity / self.com_height * (com_x - zmp_x)
        com_y_ddot = self.gravity / self.com_height * (com_y - zmp_y)

        # Integrate dynamics
        dt = self.dt
        new_com_x_dot = com_x_dot + com_x_ddot * dt
        new_com_y_dot = com_y_dot + com_y_ddot * dt
        new_com_x = com_x + new_com_x_dot * dt
        new_com_y = com_y + new_com_y_dot * dt

        return np.array([new_com_x, new_com_y, new_com_x_dot, new_com_y_dot])
```

### 2. Reinforcement Learning for Locomotion

RL can learn complex locomotion patterns:

```python
class RLLocomotionController:
    def __init__(self):
        # Neural network policy
        self.policy_network = self.build_policy_network()
        self.value_network = self.build_value_network()

    def build_policy_network(self):
        """Build neural network for policy"""
        import tensorflow as tf

        model = tf.keras.Sequential([
            tf.keras.layers.Dense(256, activation='tanh', input_shape=(20,)),  # state input
            tf.keras.layers.Dense(256, activation='tanh'),
            tf.keras.layers.Dense(128, activation='tanh'),
            tf.keras.layers.Dense(12, activation='tanh')  # joint commands
        ])

        return model

    def compute_action(self, state):
        """Get action from neural network policy"""
        state_tensor = tf.convert_to_tensor(state.reshape(1, -1), dtype=tf.float32)
        action = self.policy_network(state_tensor)
        return action.numpy().flatten()

    def compute_reward(self, robot_state, action):
        """Compute reward for current state and action"""
        # Forward velocity reward
        forward_vel = robot_state[6]  # assume 7th element is forward velocity
        reward = forward_vel * 2.0

        # Balance reward
        torso_angle = robot_state[3]  # assume 4th element is torso angle
        balance_reward = np.exp(-abs(torso_angle) * 10)
        reward += balance_reward

        # Penalty for excessive joint torques
        torque_penalty = -np.sum(np.abs(action)) * 0.01
        reward += torque_penalty

        # Penalty for falling
        if abs(torso_angle) > 0.5:  # rad
            reward -= 100

        return reward
```

## Stability Analysis

### Lyapunov Stability

For a system to be stable, we need a Lyapunov function V(x) such that:
- V(x) > 0 for all x ≠ 0
- V̇(x) ≤ 0 for all x

```python
def lyapunov_function(com_state, desired_state):
    """Define Lyapunov function for balance control"""
    com_pos, com_vel = com_state
    desired_pos, desired_vel = desired_state

    # Position error
    pos_error = com_pos - desired_pos

    # Velocity error
    vel_error = com_vel - desired_vel

    # Lyapunov function: V = 0.5 * (pos_error^2 + vel_error^2)
    V = 0.5 * (pos_error**2 + vel_error**2)

    return V

def lyapunov_derivative(com_state, com_dynamics, desired_state, desired_dynamics):
    """Compute Lyapunov derivative"""
    com_pos, com_vel = com_state
    desired_pos, desired_vel = desired_state

    # Error dynamics
    pos_error = com_pos - desired_pos
    vel_error = com_vel - desired_vel

    pos_error_dot = com_vel - desired_vel
    vel_error_dot = com_dynamics - desired_dynamics

    # V̇ = pos_error * pos_error_dot + vel_error * vel_error_dot
    V_dot = pos_error * pos_error_dot + vel_error * vel_error_dot

    return V_dot
```

### Stability Margins

```python
class StabilityAnalyzer:
    def __init__(self):
        self.stability_thresholds = {
            'com_deviation': 0.1,      # max CoM deviation (m)
            'torso_angle': 0.3,        # max torso angle (rad)
            'angular_velocity': 0.5,   # max angular velocity (rad/s)
            'zmp_margin': 0.05         # min ZMP margin (m)
        }

    def evaluate_stability(self, robot_state):
        """Evaluate current stability margins"""
        metrics = {}

        # Center of mass deviation
        com_deviation = abs(robot_state['com_x'] - robot_state['zmp_x'])
        metrics['com_stability'] = com_deviation < self.stability_thresholds['com_deviation']
        metrics['com_margin'] = self.stability_thresholds['com_deviation'] - com_deviation

        # Torso angle
        torso_angle = abs(robot_state['torso_angle'])
        metrics['angle_stability'] = torso_angle < self.stability_thresholds['torso_angle']
        metrics['angle_margin'] = self.stability_thresholds['torso_angle'] - torso_angle

        # ZMP margin
        support_polygon = robot_state['support_polygon']
        zmp_pos = (robot_state['zmp_x'], robot_state['zmp_y'])
        zmp_margin = self.calculate_zmp_margin(zmp_pos, support_polygon)
        metrics['zmp_stability'] = zmp_margin > self.stability_thresholds['zmp_margin']
        metrics['zmp_margin'] = zmp_margin

        return metrics

    def calculate_zmp_margin(self, zmp_pos, support_polygon):
        """Calculate minimum distance from ZMP to polygon edge"""
        zmp_x, zmp_y = zmp_pos
        min_distance = float('inf')

        # Calculate distance to each edge of support polygon
        for i in range(len(support_polygon)):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % len(support_polygon)]

            # Distance from point to line segment
            distance = self.point_to_line_distance(zmp_pos, p1, p2)
            min_distance = min(min_distance, distance)

        return min_distance
```

## Practical Implementation Considerations

### Sensor Fusion for Balance

```python
class BalanceSensorFusion:
    def __init__(self):
        # Kalman filter parameters
        self.process_noise = 0.1
        self.measurement_noise = 0.01
        self.estimate_error = 1.0
        self.estimate = 0.0

    def kalman_update(self, measurement):
        """Update state estimate using Kalman filter"""
        # Prediction step
        prediction = self.estimate
        pred_error = self.estimate_error + self.process_noise

        # Update step
        kalman_gain = pred_error / (pred_error + self.measurement_noise)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * pred_error

        return self.estimate

    def estimate_com_position(self, imu_data, force_sensors, vision_data):
        """Estimate CoM position using multiple sensors"""
        # IMU provides orientation and angular velocity
        orientation = self.integrate_gyro(imu_data['gyro'])

        # Force sensors provide ZMP information
        zmp_from_forces = self.calculate_zmp_from_forces(force_sensors)

        # Vision provides absolute position reference
        absolute_pos = vision_data['position'] if vision_data else None

        # Fuse all estimates
        fused_estimate = self.fuse_sensor_data(orientation, zmp_from_forces, absolute_pos)

        return fused_estimate
```

### Real-Time Control Implementation

```python
import threading
import time

class RealTimeBalanceController:
    def __init__(self):
        self.control_frequency = 1000  # Hz
        self.dt = 1.0 / self.control_frequency
        self.running = False
        self.control_thread = None

        # Initialize controllers
        self.zmp_controller = ZMPController()
        self.footstep_planner = FootstepPlanner()

    def start_control_loop(self):
        """Start real-time control loop in separate thread"""
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def control_loop(self):
        """Real-time control loop"""
        last_time = time.time()

        while self.running:
            current_time = time.time()
            elapsed = current_time - last_time

            if elapsed >= self.dt:
                # Read sensor data
                sensor_data = self.read_sensors()

                # Update state estimate
                robot_state = self.estimate_state(sensor_data)

                # Compute control commands
                control_commands = self.compute_control(robot_state)

                # Send commands to actuators
                self.send_commands(control_commands)

                last_time = current_time
            else:
                # Sleep to maintain timing
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def stop_control_loop(self):
        """Stop real-time control"""
        self.running = False
        if self.control_thread:
            self.control_thread.join()
```

## Advanced Locomotion Patterns

### Walking Gaits

```python
class WalkingGaitGenerator:
    def __init__(self):
        self.gait_params = {
            'walk': {'step_length': 0.3, 'step_height': 0.05, 'period': 0.8},
            'trot': {'step_length': 0.4, 'step_height': 0.08, 'period': 0.6},
            'pace': {'step_length': 0.2, 'step_height': 0.03, 'period': 1.0}
        }

    def generate_walk_pattern(self, gait_type='walk', duration=10.0):
        """Generate walking pattern for specified gait"""
        params = self.gait_params[gait_type]

        # Generate step timing
        step_timing = np.arange(0, duration, params['period'])

        # Generate foot trajectories
        trajectories = []
        for t in step_timing:
            # Calculate foot position based on gait phase
            phase = (t % params['period']) / params['period']

            # Generate swing trajectory
            if phase < 0.5:  # Swing phase
                swing_progress = phase * 2
                foot_x = params['step_length'] * swing_progress
                foot_z = params['step_height'] * np.sin(np.pi * swing_progress)
            else:  # Stance phase
                stance_progress = (phase - 0.5) * 2
                foot_x = params['step_length'] * (0.5 + 0.5 * stance_progress)
                foot_z = 0.0

            trajectories.append({
                'time': t,
                'position': [foot_x, 0, foot_z],
                'phase': 'swing' if phase < 0.5 else 'stance'
            })

        return trajectories
```

## Conclusion

Balance and locomotion control represents one of the most challenging aspects of humanoid robotics. Success requires careful integration of dynamics modeling, control theory, and real-time implementation. The approaches covered in this section—from classical ZMP control to modern reinforcement learning—provide a foundation for developing stable, efficient walking robots.

The next section will explore motor control and actuation systems that enable these sophisticated control strategies.

[Next: Motor Control →](./motor-control)