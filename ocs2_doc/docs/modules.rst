.. _doxid-index_1ocs2_doc_link_section:

ocs2 modules
~~~~~~~~~~~~

.. _doxid-index_1ocs2_doc_ocs2_core:

core module
-----------

the core module provides the followings features:

* definition of the optimal control problem.
* a numerical integration with different ode solvers such as explicit and implicit solvers.
* interface classes for dynamics, constraints, and their derivatives.
* interface classes to the auto-differentiated dynamics, constraints, and cost.
* an efficient linear interpolation class.
* linear system representation in time and frequency domain.
* jacobian and hessian of general functions using numerical differentiation, or automatic-differentiation with code-generation or just-in-time (jit) compilation.


.. _doxid-index_1ocs2_doc_ocs2_ddp:

ddp module
----------

the ddp module provides the followings features:

* a (un)constrained slq/ ilqr algorithm for (non-)switched systems farshidian17a.

* a continuous-time riccati equations solver based on ode solver.

* a continuous-time riccati equations solver based on linearized equation and exponential method.

* a continuous-time riccati equations solver using parallel computation instead of of the common sequential. approach farshidian17d.

* a general solver for linear, two-point boundary value problem (bvp).





.. _doxid-index_1ocs2_doc_ocs2_ocs2:

ocs2 module
-----------

the ocs2 module provides the followings features:

* a continuous-time unconstrained algorithm for computing the derivative of the optimized value function w.r.t. switching times based on the underlying lq problem farshidian17b.

* a continuous-time (un)constrained algorithm for computing the derivative of the optimized value function w.r.t. switching times based on a riccati approach to solve the two-point boundary value problem farshidian17b.

* a continuous-time (un)constrained algorithm for switched systems optimal control farshidian17b.





.. _doxid-index_1ocs2_doc_ocs2_mpc:

mpc module
----------

the mpc module provides the followings features:

* a general interface for model predictive control (mpc).

* a real-time mpc algorithm for (non-)switched systems using slq as the optimal control solver farshidian17d.





.. _doxid-index_1ocs2_doc_ros_interfaces:

ros interfaces module
---------------------

the ros interfaces module provides the followings features:

* a ros interface for mpc node with efficient message publishing scheme.

* a ros interface for mrt (model reference tracking) node which allows a user-friendly access method to the mpc policy.

* a command node for sending desired trajectory or the target goal to mpc node.

* a debugging class which imitates the full mpc-mrt process base on the mpc internal model.





.. _doxid-index_1ocs2_doc_ocs2_frank_wolfe:

frank wolfe module
------------------

the frank-wolfe module provides the followings features:

* an implementation of the frank-wolfe algorithm jaggi13 which is an iterative first-order gradient descent algorithm.

:ref:`robotic examples <doxid-ocs2_frank_wolfe_example>` provides the followings tools and examples:

* robotic tools

* ballbot

* double integrator

* quadrotor

* cart pole