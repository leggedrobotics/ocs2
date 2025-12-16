# OCS2 Toolbox

OCS2 (**O**ptimal **C**ontrol for **S**witched **S**ystems) is a C++ toolbox for formulating and solving nonlinear optimal control problems, with an emphasis on real-time **Model Predictive Control (MPC)** for robotics.

![legged-robot](https://leggedrobotics.github.io/ocs2/_static/gif/legged_robot.gif)

> **ROS 2 Jazzy port available:** use branch `ros2` (Ubuntu 24.04, colcon/ament). See **ROS 1 vs ROS 2** below.

## Highlights

- **Multiple solvers**
  - **SLQ**: continuous-time constrained DDP (SLQ/DDP family)
  - **iLQR**: discrete-time constrained DDP
  - **SQP**: multiple-shooting SQP (QP subproblems via HPIPM/BLASFEO)
  - **SLP**: sequential linear programming (PIPG)
  - **IPM**: multiple-shooting nonlinear interior-point method
- **Switched-system OCP support**: mode schedules and jump maps (single- and multi-domain problems).
- **Constraints**: hard/soft constraints with Augmented Lagrangian and relaxed barrier methods.
- **Derivatives**: analytic derivative interfaces + automatic differentiation (CppAD) and optional code generation (CppADCodeGen).
- **Robotics tooling**: URDF→model helpers (Pinocchio), kinematics, centroidal models, self-collision constraints (HPP-FCL).
- **ROS integration**: messages, nodes, and visualization/plotting tools for deploying MPC on robots.
- **Robotic examples**: end-to-end MPC examples (and ROS wrappers) for double integrator, cartpole, ballbot, quadrotor, mobile manipulator, legged robot, and more.

## Repository structure (high level)

- `ocs2_core`: core data types, math utilities, rollout interfaces, AD utilities.
- `ocs2_oc`: optimal control problem building blocks (costs, constraints, dynamics, pre-computation, reference management).
- `ocs2_ddp`, `ocs2_mpc`: DDP-family solvers + MPC infrastructure and runtime interfaces.
- `ocs2_sqp`, `ocs2_slp`, `ocs2_ipm`: alternative solvers (SQP / SLP / IPM).
- `ocs2_pinocchio/*`: URDF/Pinocchio tooling (centroidal model, kinematics, self-collision, visualization).
- `ocs2_ros_interfaces`, `ocs2_msgs`: ROS interfaces and message definitions.
- `ocs2_robotic_examples/*`: robotic examples and their ROS nodes.
- `ocs2_python_interface`: Python bindings for selected components.
- `ocs2_mpcnet`: MPC-Net tooling for learned policy deployment/training.
- `ocs2_doc`: documentation sources (Sphinx/Doxygen).

## Documentation

- Online documentation: https://leggedrobotics.github.io/ocs2/

## ROS 1 vs ROS 2

OCS2 historically targets **ROS 1 / catkin** (documented/tested primarily on **Ubuntu 20.04 + ROS Noetic**).

A **ROS 2 Jazzy** port (**Ubuntu 24.04**, **colcon/ament**) is available in this repository on branch `ros2`.

### ROS 2 Jazzy (branch `ros2`)

- Build instructions: https://github.com/leggedrobotics/ocs2/blob/ros2/installation.md
- Optional Jazzy Docker environment: https://github.com/leggedrobotics/ocs2/tree/ros2/docker
- Robotic assets: use the `ros2` branch of https://github.com/leggedrobotics/ocs2_robotic_assets

### ROS 1 Noetic (branch `main`)

Follow the installation instructions in the documentation:
- https://leggedrobotics.github.io/ocs2/ (Installation page)

## Try an example

After building and sourcing your workspace:

- **ROS 1 (Noetic, `main`)**
  ```bash
  roslaunch ocs2_cartpole_ros cartpole.launch
  ```

- **ROS 2 (Jazzy, `ros2`)**
  ```bash
  ros2 launch ocs2_cartpole_ros cartpole.launch.py
  ```

## Citing OCS2

```latex
@misc{OCS2,
  title  = {{OCS2}: An open source library for Optimal Control of Switched Systems},
  note   = {[Online]. Available: \url{https://github.com/leggedrobotics/ocs2}},
  author = {Farbod Farshidian and others}
}
```

## License

BSD 3-Clause, see `LICENCE.txt`.
