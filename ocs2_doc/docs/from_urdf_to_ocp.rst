.. index:: pair: page; From URDF to OCP

.. _doxid-ocs2_doc_from_urdf_to_ocp:

From URDF to OCP
================

One of the main challenges that hinders the widespread application of
MPC for robotic tasks is the burden of setting up the optimal control
problem. OCS2 provides several helper classes for defining some of the
commonly-used models, cost, and constraints to mitigate this issue. To
this end, OCS2 interfaces to several third party packages such as
`RobCoGen`_, `CppADCodeGen`_, `Pinocchio`_, and `HPP-FCL`_. We here 
focus on our Pinocchio interfaces. All the discussed packages on this 
page can be found in the meta-package ocs2_pinocchio.

.. _`RobCoGen`: https://robcogenteam.bitbucket.io/
.. _`CppADCodeGen`: https://github.com/joaoleal/CppADCodeGen
.. _`Pinocchio`: https://github.com/stack-of-tasks/pinocchio
.. _`HPP-FCL`: https://github.com/humanoid-path-planner/hpp-fcl

Centroidal Model
----------------

A poly-articulated floating-base system, such as a legged robot, can be
modeled as an unactuated 3D rigid body to which is attached a set of
fully actuated limbs. Under the mild assumption that one has sufficient
control authority in the robot’s joints, it would be justifiable to
independently consider centroidal dynamics in the MPC formulation as a
simplified template model. This model’s state space comprises the
normalized centroidal momentum, the base coordinate, and the joint
positions. The input space is the concatenation of all contact wrenches
and the joint velocities. To capture the effect of the generalized
coordinates’ rate of change on the centroidal momentum, this model uses
the centroidal momentum matrix and introduces a correct mapping between
base twist and joint velocities. For more details on the implementation,
refer to the package ocs2_centroidal_model.

Kinematics
----------

OCS2 provides a Kinematics interface for any named frame in the URDF
model based on the Pinocchio library. This interface provides a
first-order model of position, orientation error, and velocity for a
list of named frames. OCS2 offers two interfaces:
PinocchioEndEffectorKinematics, which is based on analytical deviates,
and PinocchioEndEffectorKinematicsCppAd, which is based on auto
differentiation. The former is often used when one intends to rely on
the caching capability of OCS2; Otherwise, the CppAd variant should be
used.

Self Collision Avoidance
------------------------

The OCS2 library provides helper classes for defining self-collision
avoidance constraints. These constraints are conveniently defined
through the URDF model and a user-defined list of collision bodies. This
list should be a subset of the collision bodies of the URDF model. This list
is used to avoid collision checking in between all the bodies, and
to reduce the computation overhead. The collision constraints computation
requires the HPP-FCL and the Pinocchio libraries. For more details on
the implementation, refer to the package ocs2_self_collision.
