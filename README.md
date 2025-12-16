# OCS2 Toolbox (ROS 2)

This branch (`ros2`) ports OCS2 from ROS 1/catkin to **ROS 2 (colcon/ament)** and currently targets **Ubuntu 24.04 + ROS 2 Jazzy**.
For the ROS 1 version, see branch `main`.

![legged-robot](https://leggedrobotics.github.io/ocs2/_static/gif/legged_robot.gif)

OCS2 (**O**ptimal **C**ontrol for **S**witched **S**ystems) is a C++ toolbox for formulating and solving nonlinear optimal control problems, with an emphasis on real-time **Model Predictive Control (MPC)** for robotics.

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. To facilitate deployment on robotic platforms, OCS2 provides tools for ROS interfaces.

## What’s included

- **Optimal control solvers**
  - **SLQ**: continuous-time constrained DDP
  - **iLQR**: discrete-time constrained DDP
  - **SQP**: multiple-shooting SQP (QP subproblems via HPIPM/BLASFEO)
  - **SLP**: sequential linear programming (PIPG)
  - **IPM**: multiple-shooting nonlinear interior-point method
- **Robotics tooling**: URDF/Pinocchio integration, kinematics, centroidal models, and self-collision constraints (HPP-FCL).
- **ROS integration**: messages, nodes, and visualization/plotting tools for deploying MPC on robots.
- **Robotic examples**: end-to-end MPC examples (and ROS wrappers) for double integrator, cartpole, ballbot, quadrotor, mobile manipulator, legged robot, and more.

## Port status (ROS 2 Jazzy)

- This branch replaces catkin with ament/colcon across the workspace.
- `blasfeo_catkin` / `hpipm_catkin` keep their names but are ROS 2 `ament_cmake` packages.
- `ocs2_mpcnet` and `ocs2_raisim` are currently not ported and are ignored via `COLCON_IGNORE`.
- `rqt_multiplot` is not released for Jazzy on Ubuntu 24.04; multiplot launch files are optional.

## Installation

Follow the branch-specific instructions in `installation.md` (dependencies, Pinocchio on Jazzy, and colcon build):
- [`installation.md`](installation.md)

Optional Docker environment:
- [`docker/README.md`](docker/README.md)

## Run an example

After building and sourcing your workspace:

```bash
ros2 launch ocs2_ballbot_ros ballbot_mpc_mrt.launch.py
```

## Documentation

Project documentation is hosted at:
- https://leggedrobotics.github.io/ocs2/

Note: the online docs still describe the ROS 1/catkin workflow; the solver concepts and APIs are largely identical, but build/launch instructions differ on this branch.

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
