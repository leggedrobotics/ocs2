.. index:: pair: page; Introduction
.. _doxid-introductionpage:

Introduction
============

**OCS2** is a C++ toolbox tailored for **O**\ ptimal **C**\ ontrol for **S**\ witched **S**\ ystems (OCS2). The toolbox provides an efficient implementation of the following algorithms:

* **SLQ**\: Continuous-time domin DDP
* **iLQR**\: Discrete-time domain DDP
* **SQP**\: Multiple-shooting algorithm based on `HPIPM <href="https://github.com/giaf/hpipm"/>`__
* **PISOC**\: Path integral stochatic optimal control

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides the user with additional tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. To facilitate its deployment on robotic platforms, the OCS2 provides tools for ROS interfaces. The toolbox’s efficient and numerically stable implementations in conjunction with its user-friendly interface have paved the way for employing it on numerous robotic applications with limited onboard computation power. 