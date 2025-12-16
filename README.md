# OCS2 Toolbox

OCS2 (**O**ptimal **C**ontrol for **S**witched **S**ystems) is a C++ toolbox for formulating and solving nonlinear optimal control problems, with an emphasis on real-time **Model Predictive Control (MPC)** for robotics.

![legged-robot](https://leggedrobotics.github.io/ocs2/_static/gif/legged_robot.gif)

## ROS branches

| ROS stack | Branch |
| --- | --- |
| ROS 1 (catkin) | `main` |
| ROS 2 (colcon/ament) | `ros2` |

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

## Installation

- **ROS 1 (`main`, catkin)**: follow the installation instructions in the documentation (Installation page): https://leggedrobotics.github.io/ocs2/
- **ROS 2 (`ros2`, colcon/ament)**: follow the branch-specific instructions: https://github.com/leggedrobotics/ocs2/blob/ros2/installation.md

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
