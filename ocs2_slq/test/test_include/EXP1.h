/*
 * EXP1.h
 *
 *  Created on: Jan 11, 2016
 *      Author: farbod
 */

#ifndef EXP1_OCS2_OCS2_H_
#define EXP1_OCS2_OCS2_H_

#include <cmath>
#include <limits>

#include <ocs2_core/logic/LogicRulesBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

	void adjustController(controller_t& controller) const override
	{}

	void update() override
	{}

private:

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys1 : public ControlledSystemBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_Sys1() {}
	~EXP1_Sys1() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
		dxdt(0) = x(0) + u(0)*sin(x(0));
		dxdt(1) = -x(1) - u(0)*cos(x(1));
	}

	EXP1_Sys1* clone() const override {
		return new EXP1_Sys1(*this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys2 : public ControlledSystemBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_Sys2() {}
	~EXP1_Sys2() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
		dxdt(0) = x(1) + u(0)*sin(x(1));
		dxdt(1) = -x(0) - u(0)*cos(x(0));
	}

	EXP1_Sys2* clone() const override {
		return new EXP1_Sys2(*this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys3 : public ControlledSystemBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_Sys3() {}
	~EXP1_Sys3() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
		dxdt(0) = -x(0) - u(0)*sin(x(0));
		dxdt(1) = x(1) + u(0)*cos(x(1));
	}

	EXP1_Sys3* clone() const override {
		return new EXP1_Sys3(*this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_System : public ControlledSystemBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ControlledSystemBase<2,1,EXP1_LogicRules> Base;

	EXP1_System()
	: activeSubsystem_(0),
	  subsystemDynamicsPtr_(3)
	{
		subsystemDynamicsPtr_[0] = std::allocate_shared<EXP1_Sys1, Eigen::aligned_allocator<EXP1_Sys1>>( Eigen::aligned_allocator<EXP1_Sys1>() );
		subsystemDynamicsPtr_[1] = std::allocate_shared<EXP1_Sys2, Eigen::aligned_allocator<EXP1_Sys2>>( Eigen::aligned_allocator<EXP1_Sys2>() );
		subsystemDynamicsPtr_[2] = std::allocate_shared<EXP1_Sys3, Eigen::aligned_allocator<EXP1_Sys3>>( Eigen::aligned_allocator<EXP1_Sys3>() );
	}

	~EXP1_System() {}

	EXP1_System(const EXP1_System& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemDynamicsPtr_(3)
	{
		subsystemDynamicsPtr_[0] = Base::Ptr(other.subsystemDynamicsPtr_[0]->clone());
		subsystemDynamicsPtr_[1] = Base::Ptr(other.subsystemDynamicsPtr_[1]->clone());
		subsystemDynamicsPtr_[2] = Base::Ptr(other.subsystemDynamicsPtr_[2]->clone());
	}

	void initializeModel(const LogicRulesMachine<2, 1, EXP1_LogicRules>& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override {

		Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

		findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );
	}

	EXP1_System* clone() const override {
		return new EXP1_System(*this);
	}

	void computeDerivative(const scalar_t& t, const state_vector_t& x, const input_vector_t& u,
			state_vector_t& dxdt) override {

		activeSubsystem_ = findActiveSubsystemFnc_(t);

		subsystemDynamicsPtr_[activeSubsystem_]->computeDerivative(t, x, u, dxdt);
	}

public:
	int activeSubsystem_;
	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
	std::vector<Base::Ptr> subsystemDynamicsPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative1 : public DerivativesBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_SysDerivative1() {};
	~EXP1_SysDerivative1() {};

	void getDerivativeState(state_matrix_t& A)  { A << u_(0)*cos(x_(0))+1, 0, 0, u_(0)*sin(x_(1))-1; }
	void getDerivativesControl(control_gain_matrix_t& B) { B << sin(x_(0)), -cos(x_(1)); }

	EXP1_SysDerivative1* clone() const override {
		return new EXP1_SysDerivative1(*this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative2 : public DerivativesBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_SysDerivative2() {};
	~EXP1_SysDerivative2() {};

	void getDerivativeState(state_matrix_t& A)  { A << 0, u_(0)*cos(x_(1))+1, u_(0)*sin(x_(0))-1, 0; }
	void getDerivativesControl(control_gain_matrix_t& B) { B << sin(x_(1)), -cos(x_(0)); }

	EXP1_SysDerivative2* clone() const override {
		return new EXP1_SysDerivative2(*this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative3 : public DerivativesBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_SysDerivative3() {};
	~EXP1_SysDerivative3() {};

	void getDerivativeState(state_matrix_t& A)  { A << -u_(0)*cos(x_(0))-1, 0, 0, 1-u_(0)*sin(x_(1)); }
	void getDerivativesControl(control_gain_matrix_t& B) { B << -sin(x_(0)), cos(x_(1)); }

	EXP1_SysDerivative3* clone() const override {
		return new EXP1_SysDerivative3(*this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SystemDerivative : public DerivativesBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DerivativesBase<2,1,EXP1_LogicRules> Base;

	EXP1_SystemDerivative()
	: activeSubsystem_(0),
	  subsystemDerivativesPtr_(3)
	{
		subsystemDerivativesPtr_[0] = std::allocate_shared<EXP1_SysDerivative1, Eigen::aligned_allocator<EXP1_SysDerivative1>>(
				Eigen::aligned_allocator<EXP1_SysDerivative1>() );
		subsystemDerivativesPtr_[1] = std::allocate_shared<EXP1_SysDerivative2, Eigen::aligned_allocator<EXP1_SysDerivative2>>(
				Eigen::aligned_allocator<EXP1_SysDerivative2>() );
		subsystemDerivativesPtr_[2] = std::allocate_shared<EXP1_SysDerivative3, Eigen::aligned_allocator<EXP1_SysDerivative3>>(
				Eigen::aligned_allocator<EXP1_SysDerivative3>() );
	}

	~EXP1_SystemDerivative() {}

	EXP1_SystemDerivative(const EXP1_SystemDerivative& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemDerivativesPtr_(3)
	{
		subsystemDerivativesPtr_[0] = Base::Ptr(other.subsystemDerivativesPtr_[0]->clone());
		subsystemDerivativesPtr_[1] = Base::Ptr(other.subsystemDerivativesPtr_[1]->clone());
		subsystemDerivativesPtr_[2] = Base::Ptr(other.subsystemDerivativesPtr_[2]->clone());
	}


	void initializeModel(const LogicRulesMachine<2, 1, EXP1_LogicRules>& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override {

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

	void getDerivativeState(state_matrix_t& A)  {
		subsystemDerivativesPtr_[activeSubsystem_]->getDerivativeState(A);
	}

	void getDerivativesControl(control_gain_matrix_t& B) {
		subsystemDerivativesPtr_[activeSubsystem_]->getDerivativesControl(B);
	}

public:
	int activeSubsystem_;
	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
	std::vector<Base::Ptr> subsystemDerivativesPtr_;

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP1_SystemConstraint = ConstraintBase<2,1,EXP1_LogicRules>;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction1 : public CostFunctionBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_CostFunction1() {};
	~EXP1_CostFunction1() {};

	void evaluate(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void controlDerivative(input_vector_t& dLdu)  { dLdu << u_; }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0; }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

	EXP1_CostFunction1* clone() const override {
		return new EXP1_CostFunction1(*this);
	};
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction2 : public CostFunctionBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_CostFunction2() {};
	~EXP1_CostFunction2() {};

	void evaluate(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void controlDerivative(input_vector_t& dLdu)  { dLdu << u_; }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0; }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

	EXP1_CostFunction2* clone() const override {
		return new EXP1_CostFunction2(*this);
	};

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction3 : public CostFunctionBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_CostFunction3() {};
	~EXP1_CostFunction3() {};

	void evaluate(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void controlDerivative(input_vector_t& dLdu)  { dLdu << u_; }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2); }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx << (x_(0)-1.0), (x_(1)+1.0); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx << 1.0, 0.0, 0.0, 1.0; }

	EXP1_CostFunction3* clone() const override {
		return new EXP1_CostFunction3(*this);
	};

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction : public CostFunctionBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef CostFunctionBase<2,1,EXP1_LogicRules> Base;

	EXP1_CostFunction()
	: activeSubsystem_(0),
	  subsystemCostsPtr_(3)
	{
		subsystemCostsPtr_[0] = std::allocate_shared<EXP1_CostFunction1, Eigen::aligned_allocator<EXP1_CostFunction1>>(
				Eigen::aligned_allocator<EXP1_CostFunction1>() );
		subsystemCostsPtr_[1] = std::allocate_shared<EXP1_CostFunction2, Eigen::aligned_allocator<EXP1_CostFunction2>>(
				Eigen::aligned_allocator<EXP1_CostFunction2>() );
		subsystemCostsPtr_[2] = std::allocate_shared<EXP1_CostFunction3, Eigen::aligned_allocator<EXP1_CostFunction3>>(
				Eigen::aligned_allocator<EXP1_CostFunction3>() );
	}

	~EXP1_CostFunction() {}

	EXP1_CostFunction(const EXP1_CostFunction& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemCostsPtr_(3)
	{
		subsystemCostsPtr_[0] = Base::Ptr(other.subsystemCostsPtr_[0]->clone());
		subsystemCostsPtr_[1] = Base::Ptr(other.subsystemCostsPtr_[1]->clone());
		subsystemCostsPtr_[2] = Base::Ptr(other.subsystemCostsPtr_[2]->clone());
	}

	void initializeModel(const LogicRulesMachine<2, 1, EXP1_LogicRules>& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override {

		Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

		findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );
	}

	EXP1_CostFunction* clone() const override {
		return new EXP1_CostFunction(*this);
	}

	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {

		Base::setCurrentStateAndControl(t, x, u);
		activeSubsystem_ = findActiveSubsystemFnc_(t);
		subsystemCostsPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
	}

	void evaluate(scalar_t& L) {
		subsystemCostsPtr_[activeSubsystem_]->evaluate(L);
	}

	void stateDerivative(state_vector_t& dLdx) {
		subsystemCostsPtr_[activeSubsystem_]->stateDerivative(dLdx);
	}
	void stateSecondDerivative(state_matrix_t& dLdxx)  {
		subsystemCostsPtr_[activeSubsystem_]->stateSecondDerivative(dLdxx);
	}
	void controlDerivative(input_vector_t& dLdu)  {
		subsystemCostsPtr_[activeSubsystem_]->controlDerivative(dLdu);
	}
	void controlSecondDerivative(control_matrix_t& dLduu)  {
		subsystemCostsPtr_[activeSubsystem_]->controlSecondDerivative(dLduu);
	}

	void stateControlDerivative(control_feedback_t& dLdxu) {
		subsystemCostsPtr_[activeSubsystem_]->stateControlDerivative(dLdxu);
	}

	void terminalCost(scalar_t& Phi) {
		subsystemCostsPtr_[activeSubsystem_]->terminalCost(Phi);
	}
	void terminalCostStateDerivative(state_vector_t& dPhidx)  {
		subsystemCostsPtr_[activeSubsystem_]->terminalCostStateDerivative(dPhidx);
	}
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  {
		subsystemCostsPtr_[activeSubsystem_]->terminalCostStateSecondDerivative(dPhidxx);
	}

public:
	int activeSubsystem_;
	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
	std::vector<std::shared_ptr<CostFunctionBase<2,1,EXP1_LogicRules> > > subsystemCostsPtr_;

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP1_SystemOperatingTrajectories = SystemOperatingPoint<2,1,EXP1_LogicRules>;

} // namespace ocs2

#endif /* EXP1_OCS2_H_ */
