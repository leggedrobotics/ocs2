.. index:: pair: page; Example
.. _doxid-ocs2_doc_example:

Example
=======

Here, we will walk you through how to setup a simple switched system's optimal control problem. We would assume that the switching times are fixed and we are only interested in optimizing the continuous control inputs. This system consists of three subsystems. In the followings, we will shortly discuss how to setup each of the main components of the problem namely: logic rules, system dynamics, system derivatives, constraints, cost function, and operating trajectory. Finally we will discuss how to setup the optimal control problem using SLQ algorithm.



.. _doxid-ocs2_doc_example_1ocs2_doc_example_components:

Problem Definition
~~~~~~~~~~~~~~~~~~



.. _doxid-ocs2_doc_example_1ocs2_doc_example_logic:

Logic Rules
-----------

The logic rules let the users to implement mixed logic systems such as switched system. All the logic rules should be derived from ocs2::LogicRulesBase. The base class assume that there is a vector of time on which some user defined logics will change. Three pure methods are needed to be implemented. ocs2::LogicRulesBase::rewind method which is used in the MPC application. ocs2::LogicRulesBase::adjustController which user can use to adjust the controller at the moment that logic rules changes. Finally, ocs2::LogicRulesBase::update method which is a user-defined method to adjust internal variables of the logic class whenever a new event times are set.

.. code-block:: cpp

	class EXP1_LogicRules : public LogicRulesBase<2,1>
	{
	public:
	        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	        typedef LogicRulesBase<2,1> BASE;
	
	        EXP1_LogicRules() = default;
	
	        ~EXP1_LogicRules() = default;
	
	        EXP1_LogicRules(const scalar_array_t& switchingTimes)
	        : BASE(switchingTimes)
	        {}
	
	        void rewind(const scalar_t& lowerBoundTime,
	                        const scalar_t& upperBoundTime) override
	        {}
	
	        void adjustController(controller_t& controller) const override
	        {}
	
	        void update() override
	        {}
	
	private:
	
	};





.. _doxid-ocs2_doc_example_1ocs2_doc_example_dynamics:

System Dynamics
---------------

As mentioned before, this problem has tree subsystems. Here, we will show how to setup one of these subsystems as an example. Then we will show how we can assemble these three subsystems in one system using logic rules. Bellow is an example on how to implement a simple system dynamics (here is one of the subsystems)

.. code-block:: cpp

	class EXP1_Sys1 : public ControlledSystemBase<2,1>
	{
	public:
	        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	        EXP1_Sys1() = default;
	        ~EXP1_Sys1() = default;
	
	        void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
	        
	                dxdt(0) = x(0) + u(0)*sin(x(0));
	                dxdt(1) = -x(1) - u(0)*cos(x(1));
	        }
	
	        EXP1_Sys1* clone() const override {
	        
	                return new EXP1_Sys1(*this);
	        }
	};

In the same way, we can implement the other two subsystems. Now we show how to implement a system which has these systems as its subsystem. Note that if our system was a non-switching system there was no need for the following class.

.. code-block:: cpp

	class EXP1_System : public ControlledSystemBase<2,1>
	{
	public:
	        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	        typedef ControlledSystemBase<2,1> Base;
	
	        EXP1_System()
	        : activeSubsystem_(0),
	          subsystemDynamicsPtr_(3)
	        {
	                subsystemDynamicsPtr_[0].reset( new EXP1_Sys1 );
	                subsystemDynamicsPtr_[1].reset( new EXP1_Sys2 );
	                subsystemDynamicsPtr_[2].reset( new EXP1_Sys3 );
	        }
	
	        ~EXP1_System() = default;
	
	        EXP1_System(const EXP1_System& other)
	        : activeSubsystem_(other.activeSubsystem_),
	          subsystemDynamicsPtr_(3)
	        {
	                subsystemDynamicsPtr_[0].reset(other.subsystemDynamicsPtr_[0]->clone());
	                subsystemDynamicsPtr_[1].reset(other.subsystemDynamicsPtr_[1]->clone());
	                subsystemDynamicsPtr_[2].reset(other.subsystemDynamicsPtr_[2]->clone());
	        }
	
	        EXP1_System* clone() const override {
	                return new EXP1_System(*this);
	        }
	
	        void initializeModel(
	                        HybridLogicRulesMachine<2, 1, EXP1_LogicRules>& logicRulesMachine,
	                        const size_t& partitionIndex,
	                        const char* algorithmName=NULL) override {
	
	                Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
	
	                findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );
	        }
	
	        void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u,
	                        state_vector_t& dxdt) override {
	
	                activeSubsystem_ = findActiveSubsystemFnc_(t);
	
	                subsystemDynamicsPtr_[activeSubsystem_]->computeFlowMap(t, x, u, dxdt);
	        }
	
	private:
	        int activeSubsystem_;
	        std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
	        std::vector<Base::Ptr> subsystemDynamicsPtr_;
	};

We now explain each method of this class. The class overrides the clone method using the copy constructor which is also overridden in order to ensure deep copying. In order to detect the active subsystem, we use the initialization method. This method gets a reference to the logic machine which is a wrapper for the logic rule provided to the optimizer by use. The logic machine provides us with number of useful functionalities such as ocs2::HybridLogicRulesMachine::getHandleToFindActiveEventCounter. This method returns a functional which can be used to find the active subsystem. Later this functional is used in ocs2::ControlledSystemBase::computeFlowMap to find the active partition and thus calling the correct subsystem.





.. _doxid-ocs2_doc_example_1ocs2_doc_example_derivaties:

Dynamics Derivatives
--------------------

In this section we will show how to implement the system dynamics. As before, we start with an example of subsystems then we show how to implement the switched system's dynamics derivatives. For the subsystem's derivatives, we have

.. code-block:: cpp

	class EXP1_SysDerivative1 : public DerivativesBase<2,1>
	{
	public:
	        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	        EXP1_SysDerivative1() = default;
	        ~EXP1_SysDerivative1() = default;
	
	        void getFlowMapDerivativeState(state_matrix_t& A) override {
	        
	                A << u_(0)*cos(x_(0))+1, 0, 0, u_(0)*sin(x_(1))-1;
	        }
	        void getFlowMapDerivativeInput(state_input_matrix_t& B) override {
	        
	                B << sin(x_(0)), -cos(x_(1));
	        }
	
	        EXP1_SysDerivative1* clone() const override {
	        
	                return new EXP1_SysDerivative1(*this);
	        }
	};

Then, for the derivatives of the switched system, we will have

.. code-block:: cpp

	class EXP1_SystemDerivative : public DerivativesBase<2,1>
	{
	public:
	        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	        typedef DerivativesBase<2,1> Base;
	
	        EXP1_SystemDerivative()
	        : activeSubsystem_(0),
	          subsystemDerivativesPtr_(3)
	        {
	                subsystemDerivativesPtr_[0].reset( new EXP1_SysDerivative1 );
	                subsystemDerivativesPtr_[1].reset( new EXP1_SysDerivative2 );
	                subsystemDerivativesPtr_[2].reset( new EXP1_SysDerivative3 );
	        }
	
	        ~EXP1_SystemDerivative() {}
	
	        EXP1_SystemDerivative(const EXP1_SystemDerivative& other)
	        : activeSubsystem_(other.activeSubsystem_),
	          subsystemDerivativesPtr_(3)
	        {
	                subsystemDerivativesPtr_[0].reset(other.subsystemDerivativesPtr_[0]->clone());
	                subsystemDerivativesPtr_[1].reset(other.subsystemDerivativesPtr_[1]->clone());
	                subsystemDerivativesPtr_[2].reset(other.subsystemDerivativesPtr_[2]->clone());
	        }
	
	
	        void initializeModel(
	                        HybridLogicRulesMachine<2, 1, EXP1_LogicRules>& logicRulesMachine,
	                        const size_t& partitionIndex,
	                        const char* algorithmName=NULL) override {
	
	                Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
	
	                findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );
	        }
	
	        EXP1_SystemDerivative* clone() const override {
	                return new EXP1_SystemDerivative(*this);
	        }
	
	        void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
	
	                Base::setCurrentStateAndControl(t, x, u);
	                activeSubsystem_ = findActiveSubsystemFnc_(t);
	                subsystemDerivativesPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
	        }
	
	        void getFlowMapDerivativeState(state_matrix_t& A) override {
	                subsystemDerivativesPtr_[activeSubsystem_]->getFlowMapDerivativeState(A);
	        }
	
	        void getFlowMapDerivativeInput(state_input_matrix_t& B) override {
	                subsystemDerivativesPtr_[activeSubsystem_]->getFlowMapDerivativeInput(B);
	        }
	
	private:
	        int activeSubsystem_;
	        std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
	        std::vector<Base::Ptr> subsystemDerivativesPtr_;
	
	};

Here, we skip the description for the cloning and initialization methods since it is similar to :ref:`System Dynamics <doxid-ocs2_doc_example_1ocs2_doc_example_dynamics>`.





.. _doxid-ocs2_doc_example_1ocs2_doc_example_constraints:

Constraints
-----------

Since this example is an unconstrained system, we can use the default implementation directly.

.. code-block:: cpp

	using EXP1_SystemConstraint = ConstraintBase<2,1>;





.. _doxid-ocs2_doc_example_1ocs2_doc_example_cost:

Cost Function
-------------

The cost functions implementation for the subsystem and the switched system are straight forward and we don't repeat them here. For more detailed, you can check the implementation `EXP1.h <../../../ocs2_slq/test/include/ocs2_slq/test/EXP1.h>`__





.. _doxid-ocs2_doc_example_1ocs2_doc_example_operating:

Operating Trajectory
--------------------

For this example, we use operation point default implementation

.. code-block:: cpp

	using EXP1_SystemOperatingTrajectories = SystemOperatingPoint<2,1>;







.. _doxid-ocs2_doc_example_1ocs2_doc_example_solver:

Optimal Solvers
~~~~~~~~~~~~~~~



.. _doxid-ocs2_doc_example_1ocs2_doc_example_slq:

SLQ
---

An implemntation of the SLQ algorithm for optimizing this example can be found `exp1_slq_test.cpp <../../../ocs2_slq/test/exp1_slq_test.cpp>`__. To use the SLQ optimizer, you need to simply pass the problem components to the solver. SLQ has two implementations: the single-thread implementation and multi-thread implementation. The multi-thread implementation is more efficient, However debugging might be a bit difficult. As a rule of thumb it is better to design the problem for the single-thread version and then use the multi-thread for higher speed.

.. code-block:: cpp

	// SLQ - single core version
	SLQ_MP<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slq_mp(
	                &systemDynamics, &systemDerivative,
	                &systemConstraint, &systemCostFunction,
	                &operatingTrajectories, slqSettings, &logicRules);
	
	// run multi-core SLQ
	slq_mp.run(startTime, initState, finalTime, partitioningTimes);

Above you see a snippet of the code where the multi-thread SLQ is instantiated and then it is used to optimize the problem. The inputs to the run methods are self-explanatory. A short note on the partitioningTimes, this input is used to define blocks of time partitions for which its computation will be distributed among different threads.





.. _doxid-ocs2_doc_example_1ocs2_doc_example_gslq:

GSLQ
----

An implemntation of the GSLQ algorithm for optimizing this example can be found `exp1_gslq_test.cpp <../../../ocs2_ocs2/test/exp1_gslq_test.cpp>`__. To use ocs2::GSLQ for computing the gradient of the bi-level optimization problem w.r.t. event times, we need first to run SLQ algorithm. Then, we should use a class named as ocs2::SLQ_DataCollector to collect the required variables from SLQ and pass them to the GSLQ algorithm. Below is an example of this procedure for computing the gradient w.r.t. event times at "optimumEventTimes" point.

.. code-block:: cpp

	// SLQ - single core version
	SLQ<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slq(
	                &systemDynamics, &systemDerivative,
	                &systemConstraint, &systemCostFunction,
	                &operatingTrajectories, slqSettings, &logicRules);
	// SLQ data collector
	SLQ_DataCollector<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slqDataCollector;
	// GSLQ
	GSLQ_BASE<STATE_DIM, INPUT_DIM, EXP1_LogicRules> gslq(slqSettings);
	
	// run GSLQ 
	slq.run(startTime, initState, finalTime, partitioningTimes);
	slqDataCollector.collect(&slq);
	gslq.run(optimumEventTimes, &slqDataCollector);





.. _doxid-ocs2_doc_example_1ocs2_doc_example_ocs2:

OCS2
----

An implemntation of the OCS2 algorithm for optimizing this example can be found `exp1_ocs2_test.cpp <../../../ocs2_ocs2/test/exp1_ocs2_test.cpp>`__. This class optimizes both the event times (switching times) and the continuous inputs. The OCS2 algorithm uses the GDDP algorithm for optimizing the event times and the DDP algorithm for optimizing the continuous inputs. It is based on the Frank-Wolfe gradient descent algorithm to solve the bi-level optimization. Below you see how to instantiate and run the OCS2 class.

run ocs2 using LQ ocs2.run(startTime, initState, finalTime, partitioningTimes, initEventTimes);





.. _doxid-ocs2_doc_example_1ocs2_doc_example_mpc:

MPC
---

To see how to use the MPC method please refer to the `Robotic Examples <../../../ocs2_robotic_examples/doc/html/index.html>`__.

