---
sidebar_position: 3
---

# Trajectory Planning and Execution

## Introduction to Trajectory Planning

Trajectory planning is a critical component of humanoid robotics that determines how the robot moves from one configuration to another. Unlike simple point-to-point movements, humanoid robots must plan complex, multi-degree-of-freedom trajectories that consider balance, obstacle avoidance, energy efficiency, and task requirements. The trajectory planning process bridges high-level goals with low-level motor control commands.

### Key Challenges in Humanoid Trajectory Planning

1. **High Dimensionality**: Humanoid robots have 30+ degrees of freedom that must be coordinated simultaneously.

2. **Dynamic Constraints**: Balance and stability must be maintained throughout the motion.

3. **Environmental Interaction**: Planning must account for contact with the environment.

4. **Real-time Requirements**: Many motions need to be replanned in response to environmental changes.

5. **Human-like Motion**: Trajectories should appear natural and human-like.

## Mathematical Foundations

### Configuration Space (C-Space)

The configuration space represents all possible joint configurations of the robot. For a humanoid robot with n joints, C-space is an n-dimensional space where each point represents a unique configuration.

```python
import numpy as np
from scipy.interpolate import CubicSpline

class ConfigurationSpace:
    def __init__(self, joint_limits):
        """
        Initialize configuration space with joint limits
        joint_limits: list of (min, max) tuples for each joint
        """
        self.joint_limits = np.array(joint_limits)
        self.n_dof = len(joint_limits)

    def is_valid_configuration(self, q):
        """Check if configuration q is within joint limits"""
        return np.all(q >= self.joint_limits[:, 0]) and np.all(q <= self.joint_limits[:, 1])

    def distance(self, q1, q2):
        """Calculate distance between two configurations"""
        return np.linalg.norm(q2 - q1)

    def interpolate(self, q_start, q_end, t):
        """Linear interpolation between configurations"""
        return q_start + t * (q_end - q_start)
```

### Path vs. Trajectory

- **Path**: Geometric route through configuration space (position only)
- **Trajectory**: Path with timing information (position, velocity, acceleration)

## Trajectory Representation

### Polynomial Trajectories

Polynomial trajectories are widely used due to their smoothness and ease of computation:

```python
class PolynomialTrajectory:
    def __init__(self, degree=5):
        self.degree = degree  # 5th order polynomial is common for smooth motion

    def generate_5th_order(self, q_start, q_end, t_start, t_end,
                          qd_start=0, qd_end=0, qdd_start=0, qdd_end=0):
        """
        Generate 5th order polynomial trajectory
        q: position, qd: velocity, qdd: acceleration
        """
        dt = t_end - t_start
        a0 = q_start
        a1 = qd_start
        a2 = qdd_start / 2

        # Solve for higher order coefficients
        # Using boundary conditions at start and end
        A = np.array([
            [dt**3, dt**4, dt**5],
            [3*dt**2, 4*dt**3, 5*dt**4],
            [6*dt, 12*dt**2, 20*dt**3]
        ])

        b = np.array([
            q_end - a0 - a1*dt - a2*dt**2,
            qd_end - a1 - 2*a2*dt,
            qdd_end - 2*a2
        ])

        a3, a4, a5 = np.linalg.solve(A, b)

        coefficients = [a0, a1, a2, a3, a4, a5]
        return coefficients

    def evaluate(self, coefficients, t, t_start):
        """Evaluate position, velocity, and acceleration at time t"""
        dt = t - t_start
        pos = sum(c * dt**i for i, c in enumerate(coefficients))

        # Velocity (first derivative)
        vel_coeffs = [i * c for i, c in enumerate(coefficients)][1:]
        vel = sum(c * dt**i for i, c in enumerate(vel_coeffs))

        # Acceleration (second derivative)
        acc_coeffs = [i * c for i, c in enumerate(vel_coeffs)][1:]
        acc = sum(c * dt**i for i, c in enumerate(acc_coeffs))

        return pos, vel, acc
```

### Spline-Based Trajectories

Splines provide smooth interpolation through multiple waypoints:

```python
class SplineTrajectory:
    def __init__(self):
        self.splines = {}  # One spline per joint

    def fit_spline(self, waypoints, times, smoothness=0.1):
        """
        Fit cubic splines through waypoints
        waypoints: array of [q1, q2, ..., qn] at each time
        times: array of time points
        """
        n_joints = len(waypoints[0])

        for j in range(n_joints):
            joint_positions = [waypoint[j] for waypoint in waypoints]
            # Create cubic spline for each joint
            self.splines[j] = CubicSpline(times, joint_positions,
                                        bc_type='natural',
                                        extrapolate=True)

    def evaluate(self, t):
        """Evaluate trajectory at time t"""
        positions = np.array([self.splines[j](t) for j in self.splines])

        # Compute derivatives for velocity and acceleration
        velocities = np.array([self.splines[j].derivative()(t) for j in self.splines])
        accelerations = np.array([self.splines[j].derivative(2)(t) for j in self.splines])

        return positions, velocities, accelerations
```

## Motion Planning Algorithms

### 1. Rapidly-exploring Random Trees (RRT)

RRT is effective for high-dimensional spaces and can handle complex constraints:

```python
class RRTPlanner:
    def __init__(self, config_space, step_size=0.1):
        self.config_space = config_space
        self.step_size = step_size
        self.nodes = []  # List of configurations
        self.edges = {}  # Parent-child relationships

    def plan(self, start_config, goal_config, max_iterations=10000):
        """Plan path from start to goal using RRT"""
        self.nodes = [start_config]
        self.edges[0] = None  # Root node has no parent

        for i in range(max_iterations):
            # Sample random configuration
            q_rand = self.sample_configuration()

            # Find nearest node in tree
            nearest_idx = self.nearest_node(q_rand)

            # Extend towards random configuration
            q_new = self.extend_towards(nearest_idx, q_rand)

            if q_new is not None:
                new_idx = len(self.nodes)
                self.nodes.append(q_new)
                self.edges[new_idx] = nearest_idx

                # Check if goal is reached
                if self.config_space.distance(q_new, goal_config) < self.step_size:
                    return self.extract_path(new_idx)

        return None  # Failed to find path

    def sample_configuration(self):
        """Sample random configuration within joint limits"""
        q = np.random.uniform(
            low=self.config_space.joint_limits[:, 0],
            high=self.config_space.joint_limits[:, 1]
        )
        return q

    def nearest_node(self, q):
        """Find index of nearest node in tree"""
        distances = [self.config_space.distance(q, node) for node in self.nodes]
        return np.argmin(distances)

    def extend_towards(self, nearest_idx, q_target):
        """Extend tree towards target configuration"""
        q_near = self.nodes[nearest_idx]

        # Direction vector
        direction = q_target - q_near
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            q_new = q_target
        else:
            # Move step_size in direction of target
            direction = direction / distance
            q_new = q_near + self.step_size * direction

        # Check if new configuration is valid
        if self.config_space.is_valid_configuration(q_new):
            return q_new

        return None

    def extract_path(self, goal_idx):
        """Extract path from goal to start by following parent pointers"""
        path = []
        current_idx = goal_idx

        while current_idx is not None:
            path.append(self.nodes[current_idx])
            current_idx = self.edges[current_idx]

        return path[::-1]  # Reverse to get start-to-goal path
```

### 2. Probabilistic Roadmaps (PRM)

PRM pre-computes a roadmap of the configuration space:

```python
class PRMPlanner:
    def __init__(self, config_space, n_samples=1000, connection_radius=0.5):
        self.config_space = config_space
        self.n_samples = n_samples
        self.connection_radius = connection_radius
        self.roadmap = {}  # Graph representation

    def build_roadmap(self):
        """Build roadmap by sampling configurations and connecting nearby ones"""
        # Sample random configurations
        samples = []
        for _ in range(self.n_samples):
            q = self.sample_valid_configuration()
            if q is not None:
                samples.append(q)

        # Connect nearby configurations
        for i, q1 in enumerate(samples):
            for j, q2 in enumerate(samples[i+1:], i+1):
                if (self.config_space.distance(q1, q2) < self.connection_radius and
                    self.is_collision_free_path(q1, q2)):
                    # Add edge to roadmap
                    if i not in self.roadmap:
                        self.roadmap[i] = []
                    if j not in self.roadmap:
                        self.roadmap[j] = []

                    self.roadmap[i].append(j)
                    self.roadmap[j].append(i)

    def sample_valid_configuration(self):
        """Sample configuration until valid one is found"""
        max_attempts = 100
        for _ in range(max_attempts):
            q = np.random.uniform(
                low=self.config_space.joint_limits[:, 0],
                high=self.config_space.joint_limits[:, 1]
            )
            if self.config_space.is_valid_configuration(q):
                return q
        return None

    def is_collision_free_path(self, q1, q2):
        """Check if path between q1 and q2 is collision-free"""
        # For simplicity, assume no obstacles
        # In practice, this would check against environment model
        return True
```

## Humanoid-Specific Trajectory Planning

### 1. Whole-Body Trajectory Planning

Humanoid robots must coordinate multiple tasks simultaneously:

```python
class WholeBodyTrajectoryPlanner:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.tasks = []  # List of prioritized tasks

    def add_balance_task(self, com_trajectory, priority=1):
        """Add center of mass balance task"""
        task = {
            'type': 'balance',
            'trajectory': com_trajectory,
            'priority': priority,
            'weight': 1.0
        }
        self.tasks.append(task)

    def add_end_effector_task(self, end_effector, trajectory, priority=2):
        """Add end-effector position task"""
        task = {
            'type': 'end_effector',
            'end_effector': end_effector,
            'trajectory': trajectory,
            'priority': priority,
            'weight': 1.0
        }
        self.tasks.append(task)

    def add_posture_task(self, desired_posture, priority=3):
        """Add joint posture task"""
        task = {
            'type': 'posture',
            'posture': desired_posture,
            'priority': priority,
            'weight': 0.1  # Lower weight for posture
        }
        self.tasks.append(task)

    def solve_trajectory(self, time_points):
        """Solve whole-body trajectory using prioritized task control"""
        n_timesteps = len(time_points)
        n_joints = self.robot_model.n_joints

        # Initialize joint trajectory
        joint_trajectory = np.zeros((n_timesteps, n_joints))

        # Sort tasks by priority
        sorted_tasks = sorted(self.tasks, key=lambda x: x['priority'])

        for t_idx, t in enumerate(time_points):
            current_q = joint_trajectory[t_idx-1] if t_idx > 0 else self.robot_model.q_home

            # Solve for each task
            delta_q = np.zeros(n_joints)

            for task in sorted_tasks:
                task_delta = self.compute_task_delta(task, current_q, t)
                delta_q += task_delta

            joint_trajectory[t_idx] = current_q + delta_q

        return joint_trajectory

    def compute_task_delta(self, task, current_q, time):
        """Compute joint space delta for a specific task"""
        if task['type'] == 'balance':
            # Compute CoM Jacobian and desired CoM velocity
            com_jacobian = self.robot_model.get_com_jacobian(current_q)
            desired_com_vel = self.compute_desired_com_velocity(task['trajectory'], time)

            # Pseudo-inverse to map CoM velocity to joint velocity
            joint_vel = np.linalg.pinv(com_jacobian) @ desired_com_vel
            return joint_vel * 0.01  # Assuming 10ms timestep

        elif task['type'] == 'end_effector':
            # Similar approach for end-effector task
            ee_jacobian = self.robot_model.get_jacobian(task['end_effector'], current_q)
            # ... compute and return delta
            pass

        elif task['type'] == 'posture':
            # Posture task - move toward desired joint configuration
            posture_error = task['posture'] - current_q
            return task['weight'] * posture_error * 0.01

        return np.zeros(self.robot_model.n_joints)
```

### 2. Walking Pattern Generation

Walking trajectories require special consideration for balance:

```python
class WalkingPatternGenerator:
    def __init__(self, step_length=0.3, step_width=0.2, step_height=0.05, step_period=0.8):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        self.step_period = step_period

    def generate_walk_trajectory(self, n_steps, start_pos=[0, 0]):
        """Generate walking trajectory for n steps"""
        trajectory = []

        current_x, current_y = start_pos

        for step in range(n_steps):
            # Determine which foot is swing foot (alternating)
            support_foot = 'left' if step % 2 == 0 else 'right'
            swing_foot = 'right' if step % 2 == 0 else 'left'

            # Calculate swing foot target position
            target_x = current_x + self.step_length
            target_y = current_y + ((-1)**step) * self.step_width / 2

            # Generate swing trajectory
            swing_trajectory = self.generate_swing_trajectory(
                start_pos=[current_x, current_y + ((-1)**(step+1)) * self.step_width / 2, 0],
                end_pos=[target_x, target_y, 0],
                height=self.step_height
            )

            # Add to overall trajectory
            for point in swing_trajectory:
                trajectory.append({
                    'time': step * self.step_period + point['time'],
                    'support_foot': support_foot,
                    'swing_foot': swing_foot,
                    'swing_foot_pos': point['position'],
                    'com_x': current_x + point['time'] / self.step_period * self.step_length
                })

            current_x = target_x
            current_y = target_y

        return trajectory

    def generate_swing_trajectory(self, start_pos, end_pos, height, steps=20):
        """Generate swing phase trajectory for foot"""
        trajectory = []
        dt = self.step_period / steps

        for i in range(steps):
            t = i * dt
            progress = t / self.step_period

            # Horizontal interpolation
            x = start_pos[0] + progress * (end_pos[0] - start_pos[0])
            y = start_pos[1] + progress * (end_pos[1] - start_pos[1])

            # Vertical trajectory with parabolic lift
            if progress < 0.5:
                # Lift phase
                z_progress = progress * 2
                z = start_pos[2] + height * np.sin(np.pi * z_progress / 2)**2
            else:
                # Lower phase
                z_progress = (progress - 0.5) * 2
                z = start_pos[2] + height * np.sin(np.pi * (1 - z_progress) / 2)**2

            trajectory.append({
                'time': t,
                'position': [x, y, z],
                'phase': 'swing'
            })

        return trajectory
```

## Optimization-Based Trajectory Planning

### Model Predictive Control (MPC) for Trajectories

MPC optimizes trajectories over a finite horizon:

```python
class ModelPredictiveTrajectoryPlanner:
    def __init__(self, horizon=20, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self.robot_model = None  # Should be set externally

    def solve_mpc_trajectory(self, current_state, reference_trajectory):
        """Solve MPC problem to find optimal trajectory"""
        import cvxpy as cp

        # Define optimization variables
        # State variables: position, velocity for each time step
        X = cp.Variable((self.horizon + 1, 2))  # [x, y] position
        U = cp.Variable((self.horizon, 2))     # [vx, vy] velocity commands

        # Cost function
        cost = 0

        # Tracking cost
        for k in range(self.horizon):
            cost += cp.sum_squares(X[k, :] - reference_trajectory[k])

        # Control effort cost
        for k in range(self.horizon):
            cost += 0.1 * cp.sum_squares(U[k, :])

        # Dynamics constraints (simplified)
        constraints = []
        constraints.append(X[0, :] == current_state[:2])  # Initial state

        for k in range(self.horizon):
            # Simple dynamics: x[k+1] = x[k] + dt * u[k]
            constraints.append(X[k+1, :] == X[k, :] + self.dt * U[k, :])

        # Velocity limits
        for k in range(self.horizon):
            constraints.append(cp.norm(U[k, :], 2) <= 2.0)  # Max velocity 2 m/s

        # State constraints (workspace limits)
        for k in range(self.horizon + 1):
            constraints.append(X[k, 0] >= -10)  # x_min
            constraints.append(X[k, 0] <= 10)   # x_max
            constraints.append(X[k, 1] >= -5)   # y_min
            constraints.append(X[k, 1] <= 5)    # y_max

        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if problem.status not in ["infeasible", "unbounded"]:
            return X.value, U.value
        else:
            return None, None
```

### Time-Optimal Trajectory Planning

Minimizing execution time while respecting constraints:

```python
class TimeOptimalTrajectoryPlanner:
    def __init__(self):
        self.max_velocity = 1.0
        self.max_acceleration = 2.0

    def generate_time_optimal_trajectory(self, start_pos, end_pos, start_vel=0, end_vel=0):
        """Generate time-optimal trajectory between start and end points"""
        distance = end_pos - start_pos

        # Calculate time-optimal profile
        # Phase 1: Acceleration
        t_acc = min(self.max_velocity / self.max_acceleration,
                   abs(distance) / self.max_velocity)
        d_acc = 0.5 * self.max_acceleration * t_acc**2

        # Phase 2: Constant velocity (if needed)
        remaining_distance = abs(distance) - 2 * d_acc  # Assume deceleration phase too

        if remaining_distance > 0:
            # Trapezoidal profile
            t_const = remaining_distance / self.max_velocity
            t_dec = t_acc  # Symmetric deceleration
        else:
            # Triangular profile - no constant velocity phase
            new_peak_vel = np.sqrt(abs(distance) * self.max_acceleration)
            t_acc = new_peak_vel / self.max_acceleration
            t_const = 0
            t_dec = t_acc

        total_time = t_acc + t_const + t_dec

        # Generate trajectory points
        trajectory = []
        dt = 0.01  # 100 Hz
        current_time = 0
        current_pos = start_pos
        current_vel = start_vel

        while current_time <= total_time:
            if current_time <= t_acc:
                # Acceleration phase
                current_vel = start_vel + self.max_acceleration * current_time
                current_pos = start_pos + 0.5 * self.max_acceleration * current_time**2
            elif current_time <= t_acc + t_const:
                # Constant velocity phase
                current_vel = self.max_velocity
                elapsed = current_time - t_acc
                current_pos = start_pos + d_acc + self.max_velocity * elapsed
            else:
                # Deceleration phase
                elapsed = current_time - t_acc - t_const
                current_vel = self.max_velocity - self.max_acceleration * elapsed
                current_pos = (start_pos + d_acc + self.max_velocity * t_const +
                              self.max_velocity * elapsed -
                              0.5 * self.max_acceleration * elapsed**2)

            trajectory.append({
                'time': current_time,
                'position': current_pos,
                'velocity': current_vel,
                'acceleration': self.get_acceleration(current_time, t_acc, t_const, total_time)
            })

            current_time += dt

        return trajectory

    def get_acceleration(self, t, t_acc, t_const, total_time):
        """Get acceleration at time t"""
        if t <= t_acc:
            return self.max_acceleration
        elif t <= t_acc + t_const:
            return 0
        elif t <= total_time:
            return -self.max_acceleration
        else:
            return 0
```

## Real-Time Trajectory Execution

### Trajectory Following Control

```python
class TrajectoryFollower:
    def __init__(self, kp=100, kd=20):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.trajectory = None
        self.current_time = 0
        self.time_step = 0.01  # 100 Hz

    def set_trajectory(self, trajectory):
        """Set trajectory to follow"""
        self.trajectory = trajectory
        self.current_time = 0

    def compute_control(self, current_state):
        """Compute control command to follow trajectory"""
        if self.trajectory is None:
            return np.zeros_like(current_state['position'])

        # Find current reference state
        ref_state = self.get_reference_state(self.current_time)

        # Compute position and velocity errors
        pos_error = ref_state['position'] - current_state['position']
        vel_error = ref_state['velocity'] - current_state['velocity']

        # PD control
        control = self.kp * pos_error + self.kd * vel_error

        # Update time
        self.current_time += self.time_step

        return control

    def get_reference_state(self, time):
        """Get reference state at given time using interpolation"""
        if len(self.trajectory) < 2:
            return self.trajectory[0] if self.trajectory else {'position': 0, 'velocity': 0}

        # Find surrounding trajectory points
        for i in range(len(self.trajectory) - 1):
            if self.trajectory[i]['time'] <= time < self.trajectory[i+1]['time']:
                # Linear interpolation between points
                t1, t2 = self.trajectory[i]['time'], self.trajectory[i+1]['time']
                w = (time - t1) / (t2 - t1)

                ref_pos = (1 - w) * self.trajectory[i]['position'] + w * self.trajectory[i+1]['position']
                ref_vel = (1 - w) * self.trajectory[i]['velocity'] + w * self.trajectory[i+1]['velocity']

                return {'position': ref_pos, 'velocity': ref_vel}

        # If past the end, return the last point
        return self.trajectory[-1]
```

### Online Trajectory Modification

```python
class OnlineTrajectoryModifier:
    def __init__(self):
        self.original_trajectory = None
        self.modified_trajectory = None
        self.modification_window = 5.0  # seconds to replan

    def update_trajectory(self, current_state, obstacle_detected=None):
        """Update trajectory based on current state and obstacles"""
        if self.original_trajectory is None:
            return None

        # Find current position in trajectory
        current_idx = self.find_current_trajectory_index(current_state['time'])

        if obstacle_detected:
            # Plan new path around obstacle
            new_path = self.plan_around_obstacle(
                current_state['position'],
                self.get_future_waypoints(current_idx)
            )

            # Blend new path with remaining trajectory
            modified = self.blend_trajectories(
                current_state['time'],
                new_path,
                self.original_trajectory[current_idx:]
            )

            return modified

        return self.original_trajectory[current_idx:]

    def plan_around_obstacle(self, current_pos, future_waypoints):
        """Plan path around detected obstacle"""
        # Simplified obstacle avoidance
        # In practice, this would use local planning algorithms
        avoidance_path = []

        # Move laterally to avoid obstacle
        lateral_offset = 0.5  # meters
        avoidance_point = [
            current_pos[0] + 0.5,  # Move forward slightly
            current_pos[1] + lateral_offset  # Move sideways
        ]

        avoidance_path.append(avoidance_point)

        # Then return to original path
        if future_waypoints:
            return_to_path = future_waypoints[0]
            avoidance_path.append(return_to_path)

        return avoidance_path
```

## Advanced Topics in Trajectory Planning

### Learning-Based Trajectory Generation

```python
class LearningBasedTrajectoryPlanner:
    def __init__(self):
        self.trajectory_database = []  # Store successful trajectories
        self.similarity_threshold = 0.1  # For trajectory retrieval

    def learn_from_demonstration(self, demonstration_trajectory):
        """Add successful trajectory to database"""
        self.trajectory_database.append(demonstration_trajectory)

    def retrieve_similar_trajectory(self, start_state, goal_state):
        """Retrieve similar trajectory from database"""
        best_similarity = float('inf')
        best_trajectory = None

        for traj in self.trajectory_database:
            similarity = self.calculate_similarity(
                start_state, goal_state,
                traj['start'], traj['goal']
            )

            if similarity < best_similarity:
                best_similarity = similarity
                best_trajectory = traj

        return best_trajectory if best_similarity < self.similarity_threshold else None

    def calculate_similarity(self, start1, goal1, start2, goal2):
        """Calculate similarity between two state pairs"""
        start_dist = np.linalg.norm(np.array(start1) - np.array(start2))
        goal_dist = np.linalg.norm(np.array(goal1) - np.array(goal2))
        return start_dist + goal_dist
```

### Multi-Modal Trajectory Planning

Planning for different types of motion (walking, crawling, climbing):

```python
class MultiModalTrajectoryPlanner:
    def __init__(self):
        self.motion_modes = {
            'walking': self.plan_walking_trajectory,
            'crawling': self.plan_crawling_trajectory,
            'climbing': self.plan_climbing_trajectory
        }
        self.mode_selector = MotionModeClassifier()

    def plan_for_environment(self, environment_type, start_config, goal_config):
        """Select appropriate motion mode and plan trajectory"""
        # Classify required motion mode
        mode = self.mode_selector.classify_mode(environment_type, start_config, goal_config)

        # Plan trajectory for selected mode
        if mode in self.motion_modes:
            return self.motion_modes[mode](start_config, goal_config)
        else:
            # Default to walking
            return self.motion_modes['walking'](start_config, goal_config)

    def plan_walking_trajectory(self, start_config, goal_config):
        """Plan walking trajectory between configurations"""
        # Use walking pattern generator
        pattern_gen = WalkingPatternGenerator()
        return pattern_gen.generate_walk_trajectory(10)  # 10 steps

    def plan_crawling_trajectory(self, start_config, goal_config):
        """Plan crawling trajectory (for low spaces)"""
        # Simplified implementation
        # In practice, this would generate different joint patterns
        pass

    def plan_climbing_trajectory(self, start_config, goal_config):
        """Plan climbing trajectory (for stairs/hills)"""
        # Generate trajectory with appropriate footstep patterns
        pass
```

## Performance Evaluation

### Trajectory Quality Metrics

```python
class TrajectoryEvaluator:
    def __init__(self):
        self.metrics = {}

    def evaluate_trajectory(self, trajectory, robot_model):
        """Evaluate trajectory quality using multiple metrics"""
        self.metrics = {}

        # Smoothness
        self.metrics['smoothness'] = self.calculate_smoothness(trajectory)

        # Energy efficiency
        self.metrics['energy'] = self.calculate_energy(trajectory, robot_model)

        # Balance maintenance
        self.metrics['balance'] = self.calculate_balance_score(trajectory, robot_model)

        # Task completion
        self.metrics['task_completion'] = self.calculate_task_completion(trajectory)

        # Safety
        self.metrics['safety'] = self.calculate_safety_score(trajectory, robot_model)

        return self.metrics

    def calculate_smoothness(self, trajectory):
        """Calculate trajectory smoothness (lower jerk)"""
        total_jerk = 0
        dt = trajectory[1]['time'] - trajectory[0]['time'] if len(trajectory) > 1 else 1.0

        for i in range(2, len(trajectory)):
            acc1 = (trajectory[i-1]['velocity'] - trajectory[i-2]['velocity']) / dt
            acc2 = (trajectory[i]['velocity'] - trajectory[i-1]['velocity']) / dt
            jerk = (acc2 - acc1) / dt
            total_jerk += abs(jerk)

        return 1.0 / (1.0 + total_jerk)  # Higher is better

    def calculate_energy(self, trajectory, robot_model):
        """Estimate energy consumption along trajectory"""
        total_energy = 0
        for i in range(1, len(trajectory)):
            # Approximate energy as sum of squared torques
            torques = robot_model.compute_inverse_dynamics(
                trajectory[i]['position'],
                trajectory[i]['velocity'],
                trajectory[i]['acceleration']
            )
            energy = np.sum(torques**2) * (trajectory[i]['time'] - trajectory[i-1]['time'])
            total_energy += energy

        return 1.0 / (1.0 + total_energy)  # Higher is better (less energy used)

    def calculate_balance_score(self, trajectory, robot_model):
        """Calculate how well trajectory maintains balance"""
        balance_score = 0
        n_points = len(trajectory)

        for point in trajectory:
            # Check if ZMP is within support polygon
            zmp = robot_model.compute_zmp(point['position'], point['velocity'], point['acceleration'])
            support_polygon = robot_model.get_support_polygon()

            if self.is_zmp_stable(zmp, support_polygon):
                balance_score += 1

        return balance_score / n_points if n_points > 0 else 0

    def is_zmp_stable(self, zmp, support_polygon):
        """Check if ZMP is within support polygon"""
        # Simplified point-in-polygon test
        # In practice, this would be more sophisticated
        zmp_x, zmp_y = zmp
        min_x = min(p[0] for p in support_polygon)
        max_x = max(p[0] for p in support_polygon)
        min_y = min(p[1] for p in support_polygon)
        max_y = max(p[1] for p in support_polygon)

        return min_x <= zmp_x <= max_x and min_y <= zmp_y <= max_y
```

## Conclusion

Trajectory planning is a sophisticated field that combines mathematical optimization, control theory, and robotics to enable humanoid robots to move effectively in complex environments. Modern approaches integrate multiple planning layers, from high-level path planning to low-level control, while considering the unique constraints of bipedal locomotion and human-like motion patterns.

The next chapter will explore adaptive control strategies that allow humanoid robots to adjust their movements in real-time based on environmental feedback and changing conditions.

[Next: Adaptive Control →](./adaptive-control)