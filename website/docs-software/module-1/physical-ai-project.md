---
sidebar_position: 5
---

# Physical AI Project: Simple Humanoid Balance Controller

## Project Overview

In this project, we'll implement a simple humanoid balance controller using the principles of Physical AI we've learned. We'll create a 2D planar biped model that can maintain balance when subjected to external disturbances. This project demonstrates the integration of dynamics, control theory, and simulation in a Physical AI system.

## Learning Objectives

By completing this project, you will:
- Implement a simple humanoid dynamics model
- Design a balance controller using feedback control
- Simulate the system in a physics environment
- Analyze the stability and performance of your controller
- Understand the challenges of physical AI in practice

## Prerequisites

- Basic understanding of Python and NumPy
- Familiarity with physics simulation concepts
- Understanding of control theory basics
- Access to a Python environment with required libraries

## Required Libraries

```bash
pip install numpy scipy matplotlib pygame
```

## System Design

### Robot Model

We'll create a simplified 2D planar biped with:
- 6 degrees of freedom (3 per leg)
- Point mass for torso
- Rigid links for legs
- No arms (for simplicity)

### Control Architecture

Our control system will include:
- State estimation (CoM position, velocity, orientation)
- Balance controller (ZMP-based or inverted pendulum)
- Joint-level control (PD control)

## Implementation

### 1. Robot Dynamics Model

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class SimpleBiped:
    def __init__(self):
        # Robot parameters
        self.torso_mass = 50.0  # kg
        self.leg_mass = 10.0   # kg per leg
        self.torso_height = 0.8  # m
        self.leg_length = 0.9    # m
        self.gravity = 9.81      # m/s^2

        # Initial state [x, z, theta, x_dot, z_dot, theta_dot]
        self.state = np.array([0.0, self.torso_height, 0.0, 0.0, 0.0, 0.0])

        # Support polygon (simplified)
        self.foot_separation = 0.2  # m

    def dynamics(self, state, t, control_input):
        """
        Simplified inverted pendulum dynamics with control input
        state = [x, z, theta, x_dot, z_dot, theta_dot]
        control_input = [desired CoM x, desired CoM z]
        """
        x, z, theta, x_dot, z_dot, theta_dot = state

        # Simplified equations of motion
        # For a 2D inverted pendulum: x_ddot = g * theta
        x_ddot = self.gravity * np.tan(theta) + control_input[0]
        z_ddot = control_input[1]  # Vertical force control
        theta_ddot = control_input[2]  # Torque control

        return [x_dot, z_dot, theta_dot, x_ddot, z_ddot, theta_ddot]

    def update(self, dt, control_input):
        """Update robot state using numerical integration"""
        # Integrate dynamics
        k1 = np.array(self.dynamics(self.state, 0, control_input))
        self.state = self.state + k1 * dt

        # Keep CoM within reasonable bounds
        if abs(self.state[0]) > 1.0:
            self.state[0] = np.sign(self.state[0]) * 1.0

    def get_com_position(self):
        """Get center of mass position"""
        return self.state[0], self.state[1]

    def get_orientation(self):
        """Get torso orientation"""
        return self.state[2]

    def is_stable(self):
        """Check if robot is stable (simplified)"""
        com_x, com_z = self.get_com_position()

        # Check if CoM is within support polygon
        left_foot = -self.foot_separation / 2
        right_foot = self.foot_separation / 2

        is_balanced = left_foot <= com_x <= right_foot
        is_upright = abs(self.get_orientation()) < 0.5  # radians

        return is_balanced and is_upright
```

### 2. Balance Controller

```python
class BalanceController:
    def __init__(self):
        # Control gains
        self.kp_com = 100.0  # Proportional gain for CoM position
        self.kd_com = 20.0   # Derivative gain for CoM velocity
        self.kp_theta = 50.0 # Proportional gain for orientation
        self.kd_theta = 10.0 # Derivative gain for angular velocity

        # Desired states
        self.desired_com_x = 0.0
        self.desired_com_z = 0.8
        self.desired_theta = 0.0

    def compute_control(self, robot_state, dt):
        """
        Compute control commands to maintain balance
        Returns [desired CoM x, desired CoM z, desired torque]
        """
        # Extract current state
        com_x, com_z = robot_state[0], robot_state[1]
        com_x_dot, com_z_dot = robot_state[3], robot_state[4]
        theta = robot_state[2]
        theta_dot = robot_state[5]

        # Compute errors
        com_x_error = self.desired_com_x - com_x
        com_z_error = self.desired_com_z - com_z
        theta_error = self.desired_theta - theta

        # Compute control outputs
        com_x_control = self.kp_com * com_x_error - self.kd_com * com_x_dot
        com_z_control = self.kp_com * com_z_error - self.kd_com * com_z_dot
        theta_control = self.kp_theta * theta_error - self.kd_theta * theta_dot

        return [com_x_control, com_z_control, theta_control]

    def set_desired_state(self, com_x, com_z, theta):
        """Set desired CoM position and orientation"""
        self.desired_com_x = com_x
        self.desired_com_z = com_z
        self.desired_theta = theta
```

### 3. Simulation Environment

```python
class SimulationEnvironment:
    def __init__(self):
        self.robot = SimpleBiped()
        self.controller = BalanceController()
        self.time = 0.0
        self.dt = 0.01  # 100 Hz control rate
        self.history = {'time': [], 'com_x': [], 'com_z': [], 'theta': [], 'stable': []}

    def apply_disturbance(self, time, magnitude):
        """Apply external disturbance at specific time"""
        if abs(self.time - time) < 0.1:  # Apply for 0.1 seconds
            return magnitude
        return 0.0

    def step(self):
        """Execute one simulation step"""
        # Compute control input
        control_input = self.controller.compute_control(self.robot.state, self.dt)

        # Add external disturbance
        disturbance = self.apply_disturbance(self.time, 20.0)  # N
        control_input[0] += disturbance

        # Update robot
        self.robot.update(self.dt, control_input)

        # Record state
        self.history['time'].append(self.time)
        self.history['com_x'].append(self.robot.get_com_position()[0])
        self.history['com_z'].append(self.robot.get_com_position()[1])
        self.history['theta'].append(self.robot.get_orientation())
        self.history['stable'].append(self.robot.is_stable())

        # Update time
        self.time += self.dt

    def run_simulation(self, duration):
        """Run simulation for specified duration"""
        steps = int(duration / self.dt)

        for _ in range(steps):
            self.step()

            # Check for failure
            if not self.robot.is_stable() and self.time > 0.5:
                print(f"Robot fell at time {self.time:.2f}s")
                break

    def plot_results(self):
        """Plot simulation results"""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

        # CoM X position
        ax1.plot(self.history['time'], self.history['com_x'])
        ax1.set_ylabel('CoM X Position (m)')
        ax1.grid(True)
        ax1.set_title('Center of Mass X Position Over Time')

        # CoM Z position
        ax2.plot(self.history['time'], self.history['com_z'])
        ax2.set_ylabel('CoM Z Position (m)')
        ax2.grid(True)
        ax2.set_title('Center of Mass Z Position Over Time')

        # Orientation
        ax3.plot(self.history['time'], np.degrees(self.history['theta']))
        ax3.set_ylabel('Torso Orientation (degrees)')
        ax3.set_xlabel('Time (s)')
        ax3.grid(True)
        ax3.set_title('Torso Orientation Over Time')

        plt.tight_layout()
        plt.show()

    def print_performance_metrics(self):
        """Print performance metrics"""
        stable_percentage = sum(self.history['stable']) / len(self.history['stable']) * 100
        final_com_x = self.history['com_x'][-1] if self.history['com_x'] else 0
        max_lean = np.max(np.abs(np.degrees(self.history['theta']))) if self.history['theta'] else 0

        print("\n=== Performance Metrics ===")
        print(f"Balance maintenance: {stable_percentage:.1f}%")
        print(f"Final CoM offset: {final_com_x:.3f}m")
        print(f"Maximum lean angle: {max_lean:.2f}°")
        print(f"Final stable: {self.history['stable'][-1] if self.history['stable'] else False}")
```

### 4. Main Simulation Loop

```python
def main():
    print("Starting Physical AI Balance Controller Simulation")
    print("=" * 50)

    # Create simulation environment
    sim = SimulationEnvironment()

    # Run simulation
    print("Running simulation for 5 seconds...")
    sim.run_simulation(5.0)

    # Analyze results
    print("\nSimulation complete!")
    sim.print_performance_metrics()

    # Plot results
    sim.plot_results()

    # Test with different disturbances
    print("\n" + "="*50)
    print("Testing with stronger disturbance...")

    sim2 = SimulationEnvironment()
    sim2.run_simulation(3.0)  # Shorter duration for stronger disturbance

    print("\nStrong disturbance test complete!")
    sim2.print_performance_metrics()
    sim2.plot_results()

if __name__ == "__main__":
    main()
```

## Controller Analysis

### Stability Analysis

The stability of our balance controller depends on several factors:

1. **Control Gains**: Properly tuned PD gains are crucial
2. **Sampling Rate**: Higher control rates generally improve stability
3. **Model Accuracy**: How well our simplified model represents reality
4. **Disturbance Magnitude**: Larger disturbances are harder to reject

### Performance Metrics

We evaluate our controller using:

- **Balance Maintenance**: Percentage of time robot remains stable
- **CoM Deviation**: How far CoM moves from desired position
- **Orientation Error**: Deviation from upright position
- **Settling Time**: Time to recover from disturbances

## Extending the Project

### 1. Add Walking Capability

```python
class WalkingController(BalanceController):
    def __init__(self):
        super().__init__()
        self.step_phase = 0.0
        self.step_frequency = 1.0  # Hz
        self.step_length = 0.3     # m

    def compute_walking_control(self, robot_state, dt):
        """Add walking pattern to balance control"""
        # Generate walking pattern
        self.step_phase += 2 * np.pi * self.step_frequency * dt
        foot_position = self.step_length * np.sin(self.step_phase)

        # Modify desired CoM to follow walking pattern
        self.set_desired_state(foot_position, 0.8, 0.0)

        return self.compute_control(robot_state, dt)
```

### 2. Adaptive Control

```python
class AdaptiveController(BalanceController):
    def __init__(self):
        super().__init__()
        self.adaptation_rate = 0.01
        self.model_error = 0.0

    def adapt_gains(self, state_error, control_effort):
        """Adapt control gains based on performance"""
        # Simple gain adaptation algorithm
        self.kp_com += self.adaptation_rate * state_error * control_effort
        self.kp_com = np.clip(self.kp_com, 10, 200)  # Limit gains
```

### 3. Machine Learning Integration

```python
class RLBalanceController:
    def __init__(self):
        self.weights = np.random.randn(6)  # Simple linear controller
        self.learning_rate = 0.001

    def compute_action(self, state):
        """Compute control action using learned policy"""
        # Simple linear policy: action = weights * state
        return np.dot(self.weights, state)

    def update_weights(self, state, reward, next_state):
        """Update weights using policy gradient"""
        # Simplified policy gradient update
        grad = state * reward
        self.weights += self.learning_rate * grad
```

## Real-World Considerations

### Sensor Noise and Delay

Real robots have imperfect sensors:

```python
def add_sensor_noise(measurement, noise_std):
    """Add realistic sensor noise"""
    return measurement + np.random.normal(0, noise_std)

def simulate_sensor_delay(measurement, delay_steps, buffer):
    """Simulate sensor delay"""
    buffer.append(measurement)
    if len(buffer) > delay_steps:
        return buffer.pop(0)
    return measurement
```

### Actuator Limitations

Real actuators have constraints:

```python
def apply_actuator_limits(torques, max_torque):
    """Apply actuator limits"""
    return np.clip(torques, -max_torque, max_torque)

def simulate_motor_dynamics(desired_torque, current_torque, time_constant):
    """Simulate first-order motor dynamics"""
    tau = time_constant
    dt = 0.01
    return current_torque + (desired_torque - current_torque) * (dt / tau)
```

## Troubleshooting Common Issues

### 1. Instability

**Symptoms**: Robot oscillates and falls over
**Solutions**:
- Reduce control gains
- Increase control frequency
- Add damping terms
- Check model parameters

### 2. Slow Response

**Symptoms**: Robot takes too long to recover from disturbances
**Solutions**:
- Increase proportional gains
- Add feedforward terms
- Improve sensor fusion
- Check computational delays

### 3. Overshooting

**Symptoms**: Robot overcorrects and oscillates
**Solutions**:
- Increase derivative gains
- Add anti-windup mechanisms
- Implement gain scheduling
- Use more sophisticated control methods

## Advanced Topics

### Model Predictive Control (MPC)

```python
def mpc_balance_controller(robot_state, prediction_horizon=10):
    """Model Predictive Control for balance"""
    # Predict future states
    future_states = predict_trajectory(robot_state, prediction_horizon)

    # Optimize control sequence
    optimal_control = optimize_control_sequence(future_states)

    # Return first control action
    return optimal_control[0]
```

### Robust Control

```python
def robust_balance_controller(robot_state, uncertainty_bounds):
    """Balance controller robust to model uncertainty"""
    # Design controller for worst-case scenario
    robust_gains = design_robust_gains(uncertainty_bounds)

    # Apply robust control law
    return apply_robust_control(robot_state, robust_gains)
```

## Conclusion

This project demonstrates the core principles of Physical AI by implementing a balance controller for a simplified humanoid robot. We've seen how dynamics, control theory, and simulation work together to create an intelligent physical system.

Key takeaways:
- Physical AI requires integration of multiple disciplines
- Simulation is essential for safe development
- Control design must account for physical constraints
- Real-world implementation presents additional challenges

The next modules will build on these foundations to explore more complex aspects of humanoid robotics including locomotion control, perception systems, and human-robot interaction.

[Next: Module 2: Control Systems for Humanoids →](../module-2/balance-locomotion)