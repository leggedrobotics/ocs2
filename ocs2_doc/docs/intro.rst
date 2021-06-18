.. index:: pair: page; Intro
.. _doxid-intropage:

.. _doxid-index_1ocs2_doc_intro:

Introduction
============

This is a C++ library for an efficient continuous and discrete time optimal control implementation. It includes methods for solving optimal control for continuous time problem with exogenous signal for switching in between predefine modes. The toolbox is capable of solving constrained problems.

Our proposed method is based on a bi-level optimal control approach which synthesizes an optimal feedback control policy for continuous inputs in the bottom-level and optimizes the switching times in between two consecutive system modes in the top-level.

Our **O** ptimal **C** ontrol for **S** witched **S** ystems algorithm (OCS2 algorithm) consists of two main steps: a method which synthesizes the continuous input controller and a method which calculates the parametrized cost function derivatives with respect to the switching times. For synthesizing the continuous input controller, OCS2 uses the SLQ algorithm; a dynamic programming approach, which uses the Bellman equation of optimality to locally estimate the value function and consequently the optimal control law. In the second step, OCS2 uses a Riccati-based approach to compute the derivative of the total cost with respect to the switching times.

Moreover, the library provides tools for implementing the SLQ algorithm in MPC fashion. It also includes a ROS interface for receiving and sending the MPC policy. This library also uses CppADCodeGen an automatic-differentiation toolbox to calculate the derivatives of the system dynamics, constraint, and cost function.
