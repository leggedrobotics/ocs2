/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef EXP1_OCS2_H_
#define EXP1_OCS2_H_

#include <cmath>
#include <limits>

#include <ocs2_core/logic/rules/LogicRulesBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_LogicRules : public LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase BASE;

	EXP1_LogicRules() = default;

	~EXP1_LogicRules() = default;

	EXP1_LogicRules(const scalar_array_t& eventTimes)
	: BASE(eventTimes)
	{}

	void rewind(const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) override
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

	EXP1_Sys1() = default;
	~EXP1_Sys1() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
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

	EXP1_Sys2() = default;
	~EXP1_Sys2() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
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

	EXP1_Sys3() = default;
	~EXP1_Sys3() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
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
			LogicRulesMachine<EXP1_LogicRules>& logicRulesMachine,
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative1 : public DerivativesBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_SysDerivative1() {};
	~EXP1_SysDerivative1() {};

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


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative2 : public DerivativesBase<2,1,EXP1_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP1_SysDerivative2() {};
	~EXP1_SysDerivative2() {};

	void getFlowMapDerivativeState(state_matrix_t& A) override {
		A << 0, u_(0)*cos(x_(1))+1, u_(0)*sin(x_(0))-1, 0;
	}
	void getFlowMapDerivativeInput(state_input_matrix_t& B) override {
		B << sin(x_(1)), -cos(x_(0));
	}

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

	void getFlowMapDerivativeState(state_matrix_t& A) override {
		A << -u_(0)*cos(x_(0))-1, 0, 0, 1-u_(0)*sin(x_(1));
	}
	void getFlowMapDerivativeInput(state_input_matrix_t& B) override {
		B << -sin(x_(0)), cos(x_(1));
	}

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
			LogicRulesMachine<EXP1_LogicRules>& logicRulesMachine,
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

	void getIntermediateCost(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2); }

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu)  { dLdu << u_; }
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu)  { dLduu << 1.0; }

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) { dLdxu.setZero(); }

	void getTerminalCost(scalar_t& Phi) { Phi = 0; }
	void getTerminalCostDerivativeState(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

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

	void getIntermediateCost(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2); }

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu)  { dLdu << u_; }
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu)  { dLduu << 1.0; }

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) { dLdxu.setZero(); }

	void getTerminalCost(scalar_t& Phi) { Phi = 0; }
	void getTerminalCostDerivativeState(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

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

	void getIntermediateCost(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2); }

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu)  { dLdu << u_; }
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu)  { dLduu << 1.0; }

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) { dLdxu.setZero(); }

	void getTerminalCost(scalar_t& Phi) { Phi = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2); }
	void getTerminalCostDerivativeState(state_vector_t& dPhidx)  { dPhidx << (x_(0)-1.0), (x_(1)+1.0); }
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx)  { dPhidxx << 1.0, 0.0, 0.0, 1.0; }

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
		subsystemCostsPtr_[0].reset( new EXP1_CostFunction1 );
		subsystemCostsPtr_[1].reset( new EXP1_CostFunction2 );
		subsystemCostsPtr_[2].reset( new EXP1_CostFunction3 );
	}

	~EXP1_CostFunction() {}

	EXP1_CostFunction(const EXP1_CostFunction& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemCostsPtr_(3)
	{
		subsystemCostsPtr_[0].reset(other.subsystemCostsPtr_[0]->clone());
		subsystemCostsPtr_[1].reset(other.subsystemCostsPtr_[1]->clone());
		subsystemCostsPtr_[2].reset(other.subsystemCostsPtr_[2]->clone());
	}

	void initializeModel(
			LogicRulesMachine<EXP1_LogicRules>& logicRulesMachine,
			const size_t& partitionIndex,
			const char* algorithmName=NULL) override {

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

	void getIntermediateCost(scalar_t& L) {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCost(L);
	}

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostDerivativeState(dLdx);
	}
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx)  {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeState(dLdxx);
	}
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu)  {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostDerivativeInput(dLdu);
	}
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu)  {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeInput(dLduu);
	}

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostDerivativeInputState(dLdxu);
	}

	void getTerminalCost(scalar_t& Phi) {
		subsystemCostsPtr_[activeSubsystem_]->getTerminalCost(Phi);
	}
	void getTerminalCostDerivativeState(state_vector_t& dPhidx)  {
		subsystemCostsPtr_[activeSubsystem_]->getTerminalCostDerivativeState(dPhidx);
	}
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx)  {
		subsystemCostsPtr_[activeSubsystem_]->getTerminalCostSecondDerivativeState(dPhidxx);
	}

public:
	int activeSubsystem_;
	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
	std::vector<std::shared_ptr<Base>> subsystemCostsPtr_;

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP1_SystemOperatingTrajectories = SystemOperatingPoint<2,1,EXP1_LogicRules>;

} // namespace ocs2

#endif /* EXP1_OCS2_H_ */
