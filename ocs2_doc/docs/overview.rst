.. index:: pair: page; Overview
.. _doxid-indexpage:

Overview
========

.. _doxid-index_1ocs2_doc_home:

OCS2
~~~~

This is a C++ library for an efficient continuous and discrete time optimal control implementation. It includes methods for solving optimal control for continuous time problem with exogenous signal for switching in between predefine modes. The toolbox is capable of solving constrained problems.

Our proposed method is based on a bi-level optimal control approach which synthesizes an optimal feedback control policy for continuous inputs in the bottom-level and optimizes the switching times in between two consecutive system modes in the top-level.

Our **O** ptimal **C** ontrol for **S** witched **S** ystems algorithm (OCS2 algorithm) consists of two main steps: a method which synthesizes the continuous input controller and a method which calculates the parametrized cost function derivatives with respect to the switching times. For synthesizing the continuous input controller, OCS2 uses the SLQ algorithm; a dynamic programming approach, which uses the Bellman equation of optimality to locally estimate the value function and consequently the optimal control law. In the second step, OCS2 uses a Riccati-based approach to compute the derivative of the total cost with respect to the switching times.

Moreover, the library provides tools for implementing the SLQ algorithm in MPC fashion. It also includes a ROS interface for receiving and sending the MPC policy. This library also uses CppADCodeGen an automatic-differentiation toolbox to calculate the derivatives of the system dynamics, constraint, and cost function.

.. _doxid-index_1ocs2_doc_link_section:

OCS2 Modules
~~~~~~~~~~~~

This library consists of the following main modules:



.. _doxid-index_1ocs2_doc_ocs2_core:

Core Module
-----------

The Core Module provides the followings features:

* Definition of the optimal control problem.

* A numerical integration with different ODE solvers such as explicit and implicit solvers.

* Interface classes for dynamics, constraints, and their derivatives.

* Interface classes to the auto-differentiated dynamics, constraints, and cost.

* An efficient linear interpolation class.

* Linear system representation in time and frequency domain.

* Jacobian and Hessian of general functions using Numerical Differentiation, or Automatic-Differentiation with code-generation or just-in-time (JIT) compilation.





.. _doxid-index_1ocs2_doc_ocs2_ddp:

DDP Module
----------

The DDP Module provides the followings features:

* A (un)constrained SLQ/ iLQR algorithm for (non-)switched systems farshidian17a.

* A continuous-time Riccati equations solver based on ODE solver.

* A continuous-time Riccati equations solver based on linearized equation and exponential method.

* A continuous-time Riccati equations solver using parallel computation instead of of the common sequential. approach farshidian17d.

* A general solver for linear, two-point Boundary Value Problem (BVP).





.. _doxid-index_1ocs2_doc_ocs2_ocs2:

OCS2 Module
-----------

The OCS2 Module provides the followings features:

* A continuous-time unconstrained algorithm for computing the derivative of the optimized value function w.r.t. switching times based on the underlying LQ problem farshidian17b.

* A continuous-time (un)constrained algorithm for computing the derivative of the optimized value function w.r.t. switching times based on a Riccati approach to solve the two-point Boundary Value Problem farshidian17b.

* A continuous-time (un)constrained algorithm for switched systems optimal control farshidian17b.





.. _doxid-index_1ocs2_doc_ocs2_mpc:

MPC Module
----------

The MPC Module provides the followings features:

* A general interface for Model Predictive Control (MPC).

* A real-time MPC algorithm for (non-)switched systems using SLQ as the optimal control solver farshidian17d.





.. _doxid-index_1ocs2_doc_ros_interfaces:

ROS Interfaces Module
---------------------

The ROS Interfaces Module provides the followings features:

* A ROS interface for MPC node with efficient message publishing scheme.

* A ROS interface for MRT (Model Reference Tracking) node which allows a user-friendly access method to the MPC policy.

* A command node for sending desired trajectory or the target goal to MPC node.

* A debugging class which imitates the full MPC-MRT process base on the MPC internal model.





.. _doxid-index_1ocs2_doc_ocs2_frank_wolfe:

Frank Wolfe Module
------------------

The Frank-Wolfe Module provides the followings features:

* An implementation of the Frank-Wolfe algorithm jaggi13 which is an iterative first-order gradient descent algorithm.

:ref:`Robotic Examples <doxid-ocs2_frank_wolfe_example>` provides the followings tools and examples:

* Robotic tools

* Ballbot

* Double integrator

* Quadrotor

* Cart Pole







.. _doxid-index_1ocs2_doc_source_code:

Source Code
~~~~~~~~~~~

The `source code`_ is publicly available.

.. _`source code`: https://bitbucket.org/leggedrobotics/ocs2/



.. _doxid-index_1cs2_doc_how_to_use:

How to use the OCS2 Toolbox
~~~~~~~~~~~~~~~~~~~~~~~~~~~

To get started with the control toolbox, please see :ref:`Getting Started <doxid-ocs2_doc_getting_started>`.





.. _doxid-index_1support:

Support
~~~~~~~

For any questions, issues or other troubleshooting please either

* create an issue: `https://bitbucket.org/leggedrobotics/ocs2_dev/issues <https://bitbucket.org/leggedrobotics/ocs2_dev/issues>`__

* contact: Farbod Farshidian, farbod (dot) farshidian (at) gmail (dot) com





.. _doxid-index_1ocs2_doc_ack:

Acknowledgement
~~~~~~~~~~~~~~~



.. _doxid-index_1ocs2_doc_lead:

Lead and Maintanance:
---------------------

* Farbod Farshidian





.. _doxid-index_1ocs2_doc_lead:

Lead and Maintanance:
---------------------

* Farbod Farshidian

* Jan Carius

* Ruben Grandia





.. _doxid-index_1ocs2_doc_contributors:

Contributors:
-------------

* Farbod Farshidian

* Jan Carius

* Ruben Grandia

* David Hoeller

* Asutosh Satapathy

* Markus Giftthaler







.. _doxid-index_1ocs2_doc_licence:

Licence Information
~~~~~~~~~~~~~~~~~~~

The OCS2 Toolbox is released under the BSD Licence, Version 3.0. Please note the licence and notice files in the source directory.





.. _doxid-index_1ocs2_doc_related:

Related Publications
~~~~~~~~~~~~~~~~~~~~

This toolbox has been used in the following publications:

.. bibliography::

   farshidian17d
   farshidian17a
   farshidian17b
   giftthaler17
