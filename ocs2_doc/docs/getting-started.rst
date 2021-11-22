.. index:: pair: page; Getting Started

.. _doxid-ocs2_doc_getting_started:

Getting Started
===============
We here briefly explain the steps you need to take in order to apply
OCS2 for your specific problem. We already assume that you have
successfully installed the library based on the description in the
:ref:`Installation page <doxid-ocs2_doc_installation>`.


Where to start?
~~~~~~~~~~~~~~~

The first and maybe the most important step is to decide what model
complexity you want to use for the problem in hand. For example, does a
kinematic model suffice for tasks you plan to solve with MPC? Some of
the common models are:

-  **Kinematic model**: This model is often employed for fixed or mobile
   base manipulators. The objectives of this problem are: end-effector
   tracking, self-collision avoidance, and collision-free navigation.
   For an example refer to 
   :ref:`Mobile Manipulator example <doxid-ocs2_doc_robotic_examples_mobile_manipulator>`.

-  **Dynamic model**: This is the most complete rigid-body model which
   takes in to account both the kinematics and the dynamics of the
   system. For an example refer to 
   :ref:`Ballbot example <doxid-ocs2_doc_robotic_examples_ballbot>`.

-  **Centroidal model**: This is a model often used for legged robots
   since it has the best of two worlds. First it consider the full
   kinematics of the system. Second, instead of the full dynamics which
   is computationally expensive, it only considers part of the
   dynamics that is affected by external forces. For an example refer
   to :ref:`Legged Robot example <doxid-ocs2_doc_robotic_examples_legged_robot>`.

After choosing your model, instead of starting from scratch, it is often
more convenient to start from a similar example. The OCS2 library
already includes a diverse set of robotic examples which covers most of
the commonly-used models. For more details on these examples read this
page :ref:`Robotic Examples <doxid-ocs2_doc_robotic_examples>`.


.. _doxid-ocs2_doc_getting_started_the_optimal_control_formulation:

The optimal control formulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

OCS2 is tailored to solve optimal control problems for switched systems.
A switched system consists of a finite number of dynamical subsystems
subjected to discrete events which cause transition between these
subsystems. Switched system models are encountered in many practical
applications such as automobiles and locomotives with different gears,
DC-DC converters, manufacturing processes, biological systems, and
walking robotics. The optimal control for such a system can be
formulated as:

.. math::

   \label{eq:OCProblem}
       \begin{cases}
       \underset{\mathbf u(.)}{\min} \ \ \sum_i \phi_i(\mathbf x(t_{i+1})) + \displaystyle \int_{t_i}^{t_{i+1}} l_i(\mathbf x(t), \mathbf u(t), t) \, dt \\
       \text{s.t.} \ \ \mathbf x(t_0) = \mathbf x_0 \,\hspace{12em} \text{initial state} \\ 
       \ \ \ \ \ \dot{\mathbf x}(t) = \mathbf f_i(\mathbf x(t), \mathbf u(t), t) \hspace{8em} \text{system flow map} \\
       \ \ \ \ \ \mathbf x(t_{i+1}^+) = \mathbf j(\mathbf x(t_{i+1})) \hspace{9em} \text{system jump map} \\
       \ \ \ \ \ {\mathbf g_1}_i(\mathbf x(t), \mathbf u(t), t) = \mathbf{0} \hspace{8.5em} \text{state-input equality constraints} \\
       \ \ \ \ \ {\mathbf g_2}_i(\mathbf x(t), t) = \mathbf{0} \, \hspace{10.5em}  \text{state-only equality constraints} \\
       \ \ \ \ \ \mathbf h_i(\mathbf x(t), \mathbf u(t), t) \geq \mathbf{0} \hspace{9em}  \text{inequality constraints} \\
       \ \ \ \ \ \text{for  } t_i < t < t_{i+1} \text{  and  } i \in \{0, 1, \cdots, I-1 \}
       \end{cases}

where :math:`t_i`\ s is the switching times and :math:`t_I` is the final
time. For each mode, the nonlinear cost function consists of a pre-jump
cost and an intermediate cost. While at the first glance this formulation
seems a bit intimidating, it actually has a simple interpretation and
brings a maximum flexibility for defining an MPC problem. One can
imagine this problem as a sequence of optimal control sub-problems which
are connected to each other through a system jump map. Each sub-problem
can even have different state and input dimensions as long as the jump
map is correctly defined in between them. As mentioned, the switches in
this formulation are based on time-triggered events. This means that you
need to define both the sequence of the modes and the event times where
the switches take place.

For many robotic platforms such as quadrotor, ballbot, and cartpole,
the system has only a single mode (i.e., they are single domain systems).
In these cases, the above formulation simplifies as:

.. math::

   \label{eq:OCProblem_simple}
       \begin{cases}
       \underset{\mathbf u(.)}{\min} \ \ \phi(\mathbf x(t_I)) + \displaystyle \int_{t_0}^{t_I} l(\mathbf x(t), \mathbf u(t), t) \, dt \\
       \text{s.t.} \ \ \mathbf x(t_0) = \mathbf x_0 \,\hspace{11.5em} \text{initial state} \\ 
       \ \ \ \ \ \dot{\mathbf x}(t) = \mathbf f(\mathbf x(t), \mathbf u(t), t) \hspace{7.5em} \text{system flow map} \\
       \ \ \ \ \ \mathbf g_1(\mathbf x(t), \mathbf u(t), t) = \mathbf{0} \hspace{8.5em} \text{state-input equality constraints} \\
       \ \ \ \ \ \mathbf g_2(\mathbf x(t), t) = \mathbf{0}  \hspace{10.5em}  \text{state-only equality constraints}  \\
       \ \ \ \ \ \mathbf h(\mathbf x(t), \mathbf u(t), t) \geq \mathbf{0} \hspace{8.5em}  \text{inequality constraints}
       \end{cases}

which is in the form of a regular optimal control problem.


How to setup an optimal control problem?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to define your optimal control, you need to define at least the
system dynamics and a cost function with a positive definitive Hessian
matrix w.r.t. input and a positive semi-definite Hessian matrix w.r.t.
state. Luckily, OCS2 provides a closely matching interface for defining
the optimal control problem which is called OptimalControlProblem
(:ref:`refer to this page <doxid-ocs2_doc_optimal_control_modules>`).

.. code-block:: cpp

  /** Optimal Control Problem definition */
  struct OptimalControlProblem {
    /* Cost */
    /** Intermediate cost */
    std::unique_ptr<StateInputCostCollection> costPtr;
    /** Intermediate state-only cost */
    std::unique_ptr<StateCostCollection> stateCostPtr;
    /** Pre-jump cost */
    std::unique_ptr<StateCostCollection> preJumpCostPtr;
    /** Final cost */
    std::unique_ptr<StateCostCollection> finalCostPtr;
  
    /* Soft constraints */
    /** Intermediate soft constraint penalty */
    std::unique_ptr<StateInputCostCollection> softConstraintPtr;
    /** Intermediate state-only soft constraint penalty */
    std::unique_ptr<StateCostCollection> stateSoftConstraintPtr;
    /** Pre-jump soft constraint penalty */
    std::unique_ptr<StateCostCollection> preJumpSoftConstraintPtr;
    /** Final soft constraint penalty */
    std::unique_ptr<StateCostCollection> finalSoftConstraintPtr;
  
    /* Constraints */
    /** Intermediate equality constraints, full row rank w.r.t. inputs */
    std::unique_ptr<StateInputConstraintCollection> equalityConstraintPtr;
    /** Intermediate state-only equality constraints */
    std::unique_ptr<StateConstraintCollection> stateEqualityConstraintPtr;
    /** Intermediate inequality constraints */
    std::unique_ptr<StateInputConstraintCollection> inequalityConstraintPtr;
    /** pre-jump constraints */
    std::unique_ptr<StateConstraintCollection> preJumpEqualityConstraintPtr;
    /** final constraints */
    std::unique_ptr<StateConstraintCollection> finalEqualityConstraintPtr;
  
    /* Dynamics */
    /** System dynamics pointer */
    std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  
    /* Misc. */
    /** The pre-computation module */
    std::unique_ptr<PreComputation> preComputationPtr;
    
    ...
  }

How to setup an MPC loop?
~~~~~~~~~~~~~~~~~~~~~~~~~

So far you have created an optimal control problem. To setup an MPC, you
need to solve this problem repeatedly at each control tick with the
latest state measurement. While for simple systems, solving this problem
in realtime is possible, for many robotic platforms with limited onboard
compute power and high control frequency loop, this is not possible. To
this end, you require the followings: (1) To run the MPC as fast as
possible with the latest state measurement. (2) To use the latest MPC
output without worrying about any racing issue while reading its output.
To this end, you require some synchronization mechanisms to facilitate
these requirements. OCS2 provides such functionalities by introducing
the concept of **MPC** Interface and **MRT** (Model Reference Tracking)
Interface.

MPC Interface
-------------

MPC interface is responsible for safely updating the solver with the
latest measurement. Thus the user can safely set the latest state to the
solver and advance it. If the solver is not yet terminated from the
previous call, the state will be buffered until the solver is ready, the
buffer size is one so the solver always will get the latest state.

MRT Interface
-------------

MRT interface is responsible for the safe access to the outcome of
the solver. It provides two views to the output: the time-based and the
state-base view. In the time-based approach, MRT only outputs the
optimized state-input pair for the inquiry time based on a linear
interpolation of the optimized state-input trajectory. On the other
hand, the state-based technique evaluates the optimal input using the
feedback policy for the given time and state. Note that the feedback
policy option should be activated in the solver settings (given that the
solver supports the feedback policy).

ROS and non-ROS versions
------------------------

MPC and MRT interfaces are working in tandem and you need both to deploy
MPC on your robot. Depending on that you run both MPC and MRT on a
single machine or on different machines, you should use either
MpcMrtInterface or the pair of MpcRosInterface and MrtRosInterface. As
the naming suggests the latter uses ROS for communicating between MPC
and MRT nodes.

How to test your MPC output?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The final stage is tuning your cost and other algorithmic
hyper-parameters. In order to separate the problem of planning from
tracking controller which transform the MPC outputs to the robot command
input (such as torques, and desired joint angles and velocities), OCS2
is equipped with a so-called dummy-simulator. At its simplest form, the
MRT dummy simulator only interpolates your optimized state-input and
visualize them in rviz. However, if you set a Rollout instance to the
dummy simulator, it uses it to simulate the MPC policy. In this case, if
you employ the same dynamics as you used in your optimal control problem,
you will simulate the MPC output with the exact model used for planning.
More advance simulator such as RaiSim can be also used as the rollout instance.
