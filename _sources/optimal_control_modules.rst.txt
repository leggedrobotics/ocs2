.. index:: pair: page; Optimal Control Modules

.. _doxid-ocs2_doc_optimal_control_modules:

Optimal Control Modules
=======================

On this page, we provide you with some tips on how to define your MPC
problem. In OCS2, the OptimalControlProblem structure defines the main
components of the optimization problem, i.e., dynamics, costs, and
constraints. In addition to this, you may want to provide MPC with some
reference trajectories, predefined mode schedule (for switched systems),
and other external information such as an update for the model
parameters (e.g., in an adaptive control setting), a map of the
environment (e.g., SDF map for collision avoidance), and so on. All
these will be possible with the aid of ReferenceManager and
SolverSynchronizedModule.

We start with the components of OptimalControlProblem: cost functions,
soft constraints, (hard) constraints, dynamics, and pre-computation. In
general, the cost and (hard/soft) constraints are defined at three different
time instances: (1) during the intermediate time intervals, (2) at the
switching (prejump) times, and (3) at the final time of the optimization
horizon. The prejump components are only effective in the
switched-system problems, and they are evaluated based on state values
before switches (prejump values).

The OptimalControlProblem collects a set of cost and constraint terms
instead of a single cost or constraint. You may imagine these collectors
as containers in which you can add terms using a unique name and later
requesting them back by the same name. Each term decides on being active
or inactive by overriding the isActive method of the terms. This
facilitates the change of the MPC behavior. For example, you can use an
externally triggered signal to switch between base tracking or
end-effector tracking modes for a mobile manipulator. Moreover, these
collectors introduce a level of modularity in your code and provide a
convenient tool for extending and modifying your MPC throughout your
project. For example, if you decide to add a new constraint or cost to
your problem, you can add the new term to the optimal control problem
without modifying the other terms.

Costs
-----

As mentioned, cost functions are defined for three different time
instances: intermediate, prejump, and final. The intermediate cost terms
can be function of either time, state, and input
(OptimalControlProblem::costPtr) or time and state
(OptimalControlProblem::stateCostPtr). However, the prejump and the
final cost terms should only be a function of time and state. The cost
terms should be either inherited from StateCost or StateInputCost
classes. The derived class should define the cost value and its
quadratic approximation. For complex functions, you may use the
auto-differentiation version of these classes, namely StateCostCppAd or
StateInputCostCppAd where the user only requires to provide the cost
value. For implementation details of these classes refer to
"ocs2_core/cost" and for simple examples check QuadraticStateCost and
QuadraticStateInputCost.

Note: All our optimal control solvers assume that the sum of the Hession
of the intermediate cost terms w.r.t. input is positive definite at each
intermediate time. Moreover, except for PISOC, the solvers require that
the sum of the Hessian of all terms w.r.t. state is also positive
semi-definite at each time instance (intermediate, prejump, and final).
Note that although the use of hessian correction in the line-search
strategy and trust-region globalization strategy, to some extent, can
handle non-definiteness, the solvers often work more reliably if you
guarantee this positive definiteness. One way to avoid non-definiteness
is using a Gauss-Newton approximation technique for cost functions of
the form :math:`||f(x, u)||^2` where the linear approximation of
:math:`f(x,u)` will be used to form a positive definite Hessian. For
more details refer to StateInputCostGaussNewtonAd.

Constraints
-----------

Similar to costs, constraints are also defined for three different time
instances: intermediate, prejump, and final, with intermediate
constraints being a function of either time, state, input or time, state
and the prejump, and the final constraints being only the function of
time and state.

The constraint terms should be either inherited from StateConstraint or
StateInputConstraint classes. The derived class should define the
constraint value and its linear or quadratic approximation depending on
the order of the constraints (ConstraintOrder). For complex functions,
you may use the auto-differentiation version of these classes, namely
StateConstraintCppAd or StateInputConstraintCppAd where you only require
to provide the constraint value. For implementation details of the these
class refer to "ocs2_core/constraint" and for simple examples check
LinearStateConstraint and LinearStateInputConstraint.

For handling the constraints in OCS2, you can either use hard or soft
constraint approaches. The soft constraints are collected separately by
OptimalControlProblem. The soft constraints handling is based on a
penalty method where the constraints are wrapped with user-defined
penalty functions (for a list of these penalty functions, refer to
"ocs2_core/soft_constraint/penalties"). To create a soft constraint from
your constraint term, you can use StateSoftConstraint and
StateInputSoftConstraint classes. These classes take an instance of your
constraint term and your opted penalty function and create a cost term
that the soft constraint collectors can collect. The possibility of
setting the penalty functions alongside the constraints provides the
flexibility of using different penalty functions for each constraint
(different types and/or different hyper-parameters). This makes the
tuning of constraint violation easier.

Hard constraints (referred to as constraints) are handled with higher
precision through different techniques depending on their type. The
state-input equality constraints are handled through a projection
method. The state-only equality and inequalities are handled either
through a relaxed-barrier method or an augmented Lagrangian technique
(this feature is temporarily disabled and will be added in the next
release).

Note: Since the state-input equality constraints are handled through a
projection method, OCS2 assumes that the Jacobin of the constraints
w.r.t. input is full row rank. If this condition cannot be guaranteed,
you should use a soft constraint technique.

Dynamics
--------

The dynamics are defined by their flow-map, jump-map, and their
first-order approximations (refer to SystemDynamicsBase). For regular
systems, the jump-map is an identity map, but this map can be a
nonlinear function of the state for a switched system. For complex
functions, you may use the auto-differentiation version of dynamics
SystemDynamicsBaseAD.

Pre-computation
---------------

OCS2 is cache-friendly, which means that you can share computation
between cost, constraints, system dynamics, and their approximations.
To achieve this, OCS2 uses PreComputation. Before evaluating cost,
constraints, and system dynamics, OCS2 solvers call
PreComputation::request (requestPreJump at a switching time or
requestFinal at the final time) with a request message indicating which
operations will be conducted next. Therefore, you can implement request()
method based on its input argument indicating the request set.

Note: As a general rule, you should avoid using caching and only later
implement the caching version to gain performance. For an example
refer to "ocs2_robotic_examples/ocs2_mobile_manipulator".

Changing parameters of the Optimal Control Problem
--------------------------------------------------

Once you have defined and set your OptimalControlProblem to a solver,
the solver creates a copy (in fact, many copies) of it internally.
Therefore if you decide to change any parameter in the optimal control
problem, such as the reference trajectories or a model parameter, you
cannot achieve this by simply modifying the parameters of the optimal
control problem you have access to. Moreover, regardless of this
technical point, you should avoid changing these parameters arbitrarily
at any time. The reason is that your MPC solver might be in the middle
of an iteration when you alter a parameter, and this will cause
undefined behavior in the solver. To circumvent this issue, OCS2
introduces the concepts of Reference Manager and Solver Synchronized
Modules. In general, these are synchronization concepts and ensure that
the parameters update occurs at the correct times (before and/or after
each iteration of MPC). In other words, they synchronize parameter
updates with MPC iterations.

To access the updated parameters/information in your optimal control
components such as cost, constraint, dynamics, ..., you need to take the
following steps: (1) Create a shared pointer of your synchronization
module. (2) Share the address of this instance with your costs,
constraints, or dynamics. (3) Set it to the solver through
SolverBase::setReferenceManager, SolverBase::addSynchronizedModule, or
SolverBase::setSynchronizedModules.

Note: There should be only one instance of each synchronization module
in your whole MPC problem.

Reference Manager Interface
---------------------------

ReferenceManagerInterface creates a generic interface for defining the
target trajectories and mode schedule (used only in switched systems).
Each solver of OCS2 will call preSolverRun() of the reference manager
before starting a new iteration of MPC. For an implementation of this
interface, you can refer to ReferenceManager class. ReferenceManager has
two decorator classes: ReferenceManagerRos that adds ROS communication
to the ReferenceManager and LoopshapingReferenceManager which extend it
to the the loop-shaped OptimalControlProblem.

As the reference manager runs in sequence to the main loop of MPC, for
efficiency reasons, you should avoid complex operations in preSolverRun.
To achieve this, you should process these parameters in a different
thread and save the result in a buffer memory. Then in preSolverRun,
just update the active parameters through address swapping. OCS2
provides a helper class for this very reason called BufferedValue.

Solver Synchronized Modules
---------------------------

SolverSynchronizedModules is similar to ReferenceManagerInterface but
for general-purpose applications. It only has two pure virtual methods
preSolverRun and postSolverRun, which, as the names suggest, are called
before and after each MPC iteration. The preSolverRun method also has
access to a recently updated ReferenceManagerInterface. In contrast, the
postSolverRun method has access to the MPC solution.

Similar to ReferenceManagerInterface, SolverSynchronizedModules run in
sequence to the main loop of MPC. Therefore, for efficiency reasons, you
should avoid complex operations in preSolverRun and postSolverRun. To
achieve this, you should save/compute these parameters in a different
thread and save the result in a buffer. Then in preSolverRun or
postSolverRun, you can update the active parameters through address
swapping. You can use BufferedValue class for this purpose.
