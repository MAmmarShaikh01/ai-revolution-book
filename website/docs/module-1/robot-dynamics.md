---
sidebar_position: 3
---

# Robot Dynamics

## Understanding Robot Dynamics

Robot dynamics is the study of forces and torques that cause motion in robotic systems. For humanoid robots, understanding dynamics is crucial for achieving stable locomotion, precise manipulation, and safe interaction with the environment. Unlike simple point masses, humanoid robots are complex multibody systems with many degrees of freedom that must be controlled simultaneously.

### Why Dynamics Matter

Robot dynamics govern:
- How forces affect robot motion
- Energy consumption during movement
- Stability and balance maintenance
- Interaction with external objects
- Safety during operation

### Key Concepts

#### Degrees of Freedom (DOF)
The number of independent parameters that define the configuration of a mechanical system. Humanoid robots typically have 30+ DOF to achieve human-like mobility and dexterity.

#### Configuration Space
The space of all possible configurations of the robot, defined by joint angles and positions.

#### State Space
The space of all possible states of the robot, including positions, velocities, and potentially accelerations.

## Mathematical Foundations

### Newton-Euler Formulation

The Newton-Euler equations describe the motion of rigid bodies:

**Translation:**
```
F = ma
```

**Rotation:**
```
τ = Iα + ω × (Iω)
```

Where:
- F: Force vector
- τ: Torque vector
- m: Mass
- a: Linear acceleration
- I: Inertia tensor
- α: Angular acceleration
- ω: Angular velocity

### Lagrangian Formulation

The Lagrangian approach is often preferred for complex multibody systems:

```
L = T - V
```

Where:
- L: Lagrangian
- T: Kinetic energy
- V: Potential energy

The equations of motion are given by:

```
d/dt(∂L/∂q̇) - ∂L/∂q = Q
```

Where:
- q: Generalized coordinates
- Q: Generalized forces

## Forward and Inverse Dynamics

### Forward Dynamics

Given joint torques and current state, compute joint accelerations:

```
τ = H(q)q̈ + C(q,q̇)q̇ + g(q) + J^T F_ext
```

Where:
- H(q): Mass matrix
- C(q,q̇): Coriolis and centrifugal forces
- g(q): Gravitational forces
- J: Jacobian matrix
- F_ext: External forces

Solving for accelerations:
```
q̈ = H⁻¹(τ - Cq̇ - g - J^T F_ext)
```

### Inverse Dynamics

Given desired motion, compute required joint torques:

```
τ = H(q)q̈_des + C(q,q̇)q̇ + g(q) + J^T F_ext
```

## Multibody Dynamics for Humanoid Robots

### Kinematic Chains

Humanoid robots consist of multiple kinematic chains:
- **Leg chains**: For locomotion and balance
- **Arm chains**: For manipulation and interaction
- **Trunk**: Connecting and stabilizing element
- **Head**: For perception and communication

### Center of Mass (CoM)

The center of mass is crucial for balance and stability:

```
CoM = Σ(m_i * r_i) / Σ(m_i)
```

Where:
- m_i: Mass of link i
- r_i: Position of link i's center of mass

### Zero Moment Point (ZMP)

ZMP is a critical concept for bipedal stability:

```
ZMP_x = (Σ(m_i * (ẍ_i * h - z_i * ẍ_i)) / Σ(m_i * g + m_i * z̈_i))
ZMP_y = (Σ(m_i * (ÿ_i * h - z_i * ÿ_i)) / Σ(m_i * g + m_i * z̈_i))
```

Where:
- h: Height of CoM above ground
- g: Gravitational acceleration

## Control Strategies

### Computed Torque Control

Compensates for robot dynamics to achieve desired behavior:

```python
def computed_torque_control(q, qdot, q_des, qdot_des, qddot_des):
    # Compute desired torques
    q_error = q_des - q
    qdot_error = qdot_des - qdot

    # Feedforward + feedback control
    tau_ff = H(q) @ qddot_des + C(q, qdot) @ qdot_des + g(q)
    tau_fb = Kp @ q_error + Kd @ qdot_error

    return tau_ff + tau_fb
```

### Operational Space Control

Controls task-space variables while considering joint-space constraints:

```python
def operational_space_control(x_des, xdot_des, xddot_des, J, H, C, g):
    # Task-space mass matrix
    Lambda = inv(J @ inv(H) @ J.T)

    # Task-space dynamics
    x_error = x_des - x
    xdot_error = xdot_des - xdot

    # Desired task-space force
    f_des = Lambda @ (xddot_des + Kp @ x_error + Kd @ xdot_error)

    # Joint torques
    tau = J.T @ f_des + gravity_compensation(H, C, g)

    return tau
```

### Impedance Control

Creates desired dynamic behavior at contact points:

```python
def impedance_control(x_des, xdot_des, xddot_des, x, xdot,
                     M_d, D_d, K_d):
    # Impedance error
    x_error = x_des - x
    xdot_error = xdot_des - xdot

    # Desired impedance dynamics
    f_imp = M_d @ (xddot_des - xddot) + \
            D_d @ xdot_error + K_d @ x_error

    return f_imp
```

## Simulation and Modeling

### Multi-Body Simulation

Simulation environments like Gazebo, MuJoCo, and Isaac Sim provide:
- Accurate physics simulation
- Sensor simulation
- Environment modeling
- Control interface

### URDF (Unified Robot Description Format)

```xml
<robot name="humanoid_robot">
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0"
               iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>

  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh_link"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3"/>
  </joint>
</robot>
```

## Balance and Stability

### Static vs. Dynamic Balance

**Static Balance**: CoM remains within support polygon at all times
**Dynamic Balance**: System maintains balance through active control and momentum management

### Capture Point

The capture point indicates where the CoM will come to rest given current momentum:

```
Capture Point = CoM + (CoM_velocity / sqrt(g/height))
```

### Balance Control Strategies

1. **Ankle Strategy**: Small perturbations corrected by ankle torques
2. **Hip Strategy**: Larger perturbations addressed through hip movement
3. **Stepping Strategy**: Recovery through foot placement
4. **Momentum Control**: Using angular momentum for balance

## Practical Considerations

### Actuator Limitations

Real actuators have:
- Torque limits
- Velocity limits
- Power constraints
- Bandwidth limitations

### Sensor Noise and Delay

Real sensors introduce:
- Measurement noise
- Signal delay
- Calibration errors
- Environmental sensitivity

### Friction and Nonlinearities

Real systems exhibit:
- Joint friction
- Gear backlash
- Motor nonlinearities
- Flexible joints

## Advanced Topics

### Whole-Body Control

Coordinating multiple tasks simultaneously:
- Balance maintenance
- Manipulation tasks
- Locomotion
- Obstacle avoidance

### Hybrid Force-Motion Control

Controlling both position and force simultaneously:
- Natural compliance in contact tasks
- Stable interaction with environment
- Safe human-robot interaction

### Optimal Control

Using optimization to determine control strategies:
- Minimum energy consumption
- Smooth motion trajectories
- Constraint satisfaction

## Simulation Example

Here's a simple example of dynamics simulation:

```python
import numpy as np
from scipy.integrate import odeint

class SimpleHumanoid:
    def __init__(self):
        # Simplified model: inverted pendulum
        self.mass = 75.0  # kg
        self.height = 1.0  # m
        self.gravity = 9.81  # m/s^2

    def dynamics(self, state, t, control_torque):
        # State: [angle, angular_velocity]
        theta, theta_dot = state

        # Equation of motion for inverted pendulum
        theta_ddot = (self.gravity / self.height) * np.sin(theta) + \
                     control_torque / (self.mass * self.height**2)

        return [theta_dot, theta_ddot]

    def simulate_step(self, state, dt, control_torque):
        # Simple Euler integration
        theta, theta_dot = state
        theta_ddot = (self.gravity / self.height) * np.sin(theta) + \
                     control_torque / (self.mass * self.height**2)

        new_theta_dot = theta_dot + theta_ddot * dt
        new_theta = theta + new_theta_dot * dt

        return [new_theta, new_theta_dot]

# Example usage
robot = SimpleHumanoid()
state = [0.1, 0.0]  # Initial angle and velocity
dt = 0.01  # 100 Hz control

for i in range(1000):
    # Simple balance control
    balance_torque = -10 * state[0] - 5 * state[1]  # PD control
    state = robot.simulate_step(state, dt, balance_torque)

    if i % 100 == 0:
        print(f"Time: {i*dt:.2f}s, Angle: {state[0]:.3f} rad")
```

## Conclusion

Robot dynamics form the foundation for controlling complex humanoid systems. Understanding the mathematical principles and practical implementation challenges is essential for developing stable, efficient, and safe humanoid robots. The next section will explore simulation environments that allow us to test and validate these dynamic models.

[Next: Simulation Environments →](./simulation-environments)