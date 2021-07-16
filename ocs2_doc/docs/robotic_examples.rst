.. index:: pair: page; Robotic Examples

.. _doxid-ocs2_doc_robotic_examples:

Robotic Examples
================

OCS2 includes several robotic examples. We here briefly discuss the main
features of each example.

================== ========== ========== =========== ========
System             State Dim. Input Dim. Constrained Caching
================== ========== ========== =========== ========
Double Integrator  2          1          No          No
Cart Pole          4          1          Yes         No
Ballbot            10         3          No          No
Quadrotor          12         4          No          No
Mobile Manipulator 9          8          Yes         Yes / No
Legged Robot       24         24         Yes         No
================== ========== ========== =========== ========

.. _doxid-ocs2_doc_robotic_examples_double_integrator:

Double Integrator
^^^^^^^^^^^^^^^^^

The double integrator example is our simplest problem. It models a 1D
point mass that moves in the x-direction. The model is linear, and the
cost function is quadratic. The target point is set to the quadratic
cost through a reference manager module.

.. code-block:: bash

    # Build the example
    catkin build ocs2_double_integrator
    # Source workspace
    # Do not forget to change <...> parts
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # Launche the example
    roslaunch ocs2_double_integrator double_integrator.launch

.. _doxid-ocs2_doc_robotic_examples_cartpole:

Cart Pole
^^^^^^^^^

The cart-pole example is a classic control problem where a pole is
attached through an unactuated joint to a cart. The car moves along a
frictionless track. The goal is to swing up and balance the pendulum
starting from the downright position by accelerating a decelerating the
cart along the track. 

.. code-block:: bash

    # Build the example
    catkin build ocs2_cartpole
    # Source workspace
    # Do not forget to change <...> parts
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # Launche the example
    roslaunch ocs2_cartpole cartpole.launch

.. _doxid-ocs2_doc_robotic_examples_ballbot:

Ballbot
^^^^^^^

The Ballbot example is a 5DoF system. The platform is a
torque-controlled, omnidirectional robot that balances on a ball through
three omni-wheels. The system has nonlinear dynamics and exhibits
non-minimum-phase behavior. The system dynamics is based on the
Ballbot’s forward dynamics, and the linear approximation of the flow-map
is calculated through auto differentiation. The task objective is to
control the robot’s XY position and yaw based on user command. 

.. code-block:: bash

    # Build the example
    catkin build ocs2_ballbot
    # Source workspace
    # Do not forget to change <...> parts
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # Launche the example
    roslaunch ocs2_ballbot ballbot.launch

.. _doxid-ocs2_doc_robotic_examples_quadrotor:

Quadrotor
^^^^^^^^^

The quadrotor example is a 6DoF system. The platform is modeled as a
floating-base, rigid-body dynamics with a 3D moment and 1D force control
in the normal direction to the robot. The system dynamics and its
derivative are code generated. This example aims to track the user
command defined as the quadrotor’s 3D position and yaw. 

.. code-block:: bash

    # Build the example
    catkin build ocs2_quadrotor
    # Source workspace
    # Do not forget to change <...> parts
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # Launche the example
    roslaunch ocs2_quadrotor quadrotor.launch

.. _doxid-ocs2_doc_robotic_examples_mobile_manipulator:

Mobile Manipulator
^^^^^^^^^^^^^^^^^^

The mobile manipulator example is a fully kinematic problem. The model
consists of a 6DOF arm plus 2D position and heading of the mobile base.
The control inputs are the 6 joint velocities of the arm and the forward
and rotational velocities of the base. The objective of the task is to
track a 6DoF end-effector pose. The joint position and velocity limits
are included in the constraint of the optimal control problem.

Moreover, self-collision avoidance is achieved based on the collision
bodies of the URDF model and collision avoidance constraints (refer to
ocs2_self_collision). This example implements both the cache and the
non-cache variants of the MPC, which can be chosen through the
usePreComputation flag in the config file. 

.. code-block:: bash

    # Build the example
    catkin build ocs2_mobile_manipulator
    # Source workspace
    # Do not forget to change <...> parts
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # Launche the example
    roslaunch ocs2_mobile_manipulator mobile_manipulator.launch

.. _doxid-ocs2_doc_robotic_examples_legged_robot:

Legged Robot
^^^^^^^^^^^^

The legged robot example is a switched system problem. It implements an
MPC approach for motion control of a quadrupedal robot, Anymal. The
robot’s gait is defined by the user and can be modified during the
execution through a solver synchronized module (GaitReceiver). The mode
sequence and the target trajectories are defined through a reference
manager module (SwitchedModelReferenceManager). The cost function is a
quadratic penalty to track the commanded base position and yaw and
equally distribute the weight of the robot on the stance feet. The
problem has several mode-depended constraints, such as zero force for
the swing feet and zero velocity for the stance feet. The friction cone
is enforced on the contact forces, and to avoid foot scuffing, the swing
feet track a predefined motion in the z-direction.

The system dynamics are modeled in two ways which can be chosen from the
config file: (1) The single rigid body dynamics (SRBD): This model
assumes that the system has constant inertia regardless of its joint
position. It also includes the full kinematics of the system (2) The
full centroidal dynamics (FCD): This model uses the centroidal dynamics,
which incorporates the motion of the robot’s limbs. Similar to SRBD, it
considers the full kinematics of the robot.

.. code-block:: bash

    # Build the example
    catkin build ocs2_legged_robot
    # Source workspace
    # Do not forget to change <...> parts
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # Launche the example
    roslaunch ocs2_legged_robot legged_robot.launch
