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

#ifndef EXP0_OCS2_H_
#define EXP0_OCS2_H_

#include <cmath>
#include <limits>

#include <ocs2_core/logic/rules/LogicRulesBase.h>
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
class EXP0_LogicRules : public LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase BASE;

	EXP0_LogicRules() = default;

	~EXP0_LogicRules() = default;

	EXP0_LogicRules(const scalar_array_t& switchingTimes)
	: BASE(switchingTimes)
	{}

	void rewind(const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) final
	{}

	void update() final
	{}

private:

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys1 : public ControlledSystemBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP0_Sys1() = default;
	~EXP0_Sys1() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u,
			Eigen::Vector2d& dxdt) final {

		Eigen::Matrix2d A;
		A << 0.6, 1.2, -0.8, 3.4;
		Eigen::Vector2d B;
		B << 1, 1;

		dxdt = A*x + B*u;
	}

	EXP0_Sys1* clone() const final {
		return new EXP0_Sys1(*this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys2 : public ControlledSystemBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP0_Sys2() = default;
	~EXP0_Sys2() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u,
			Eigen::Vector2d& dxdt) final {

		Eigen::Matrix2d A;
		A << 4, 3, -1, 0;
		Eigen::Vector2d B;
		B << 2, -1;

		dxdt = A*x + B*u;
	}

	EXP0_Sys2* clone() const final {
		return new EXP0_Sys2(*this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_System : public ControlledSystemBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ControlledSystemBase<2,1,EXP0_LogicRules> Base;

	explicit EXP0_System(std::shared_ptr<const EXP0_LogicRules> logicRulesPtr) :
	logicRulesPtr_(std::move(logicRulesPtr)),
	activeSubsystem_(0),
	subsystemDynamicsPtr_(2)
	{
		subsystemDynamicsPtr_[0] = std::allocate_shared<EXP0_Sys1, Eigen::aligned_allocator<EXP0_Sys1>>( Eigen::aligned_allocator<EXP0_Sys1>() );
		subsystemDynamicsPtr_[1] = std::allocate_shared<EXP0_Sys2, Eigen::aligned_allocator<EXP0_Sys2>>( Eigen::aligned_allocator<EXP0_Sys2>() );
	}

	~EXP0_System() override = default;

	EXP0_System(const EXP0_System& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemDynamicsPtr_(2)
	{
		subsystemDynamicsPtr_[0] = Base::Ptr(other.subsystemDynamicsPtr_[0]->clone());
		subsystemDynamicsPtr_[1] = Base::Ptr(other.subsystemDynamicsPtr_[1]->clone());
		logicRulesPtr_ = other.logicRulesPtr_;
	}

	EXP0_System* clone() const final {
		return new EXP0_System(*this);
	}

	void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u,
			state_vector_t& dxdt) final {

		activeSubsystem_ = logicRulesPtr_->getEventTimeCount(t);
		subsystemDynamicsPtr_[activeSubsystem_]->computeFlowMap(t, x, u, dxdt);
	}

public:
	int activeSubsystem_;
	std::vector<Base::Ptr> subsystemDynamicsPtr_;
	std::shared_ptr<const EXP0_LogicRules> logicRulesPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_SysDerivative1 : public DerivativesBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP0_SysDerivative1() = default;
	~EXP0_SysDerivative1() = default;

	void getFlowMapDerivativeState(state_matrix_t& A) final { A << 0.6, 1.2, -0.8, 3.4; }
	void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << 1, 1; }

	EXP0_SysDerivative1* clone() const final {
		return new EXP0_SysDerivative1(*this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_SysDerivative2 : public DerivativesBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP0_SysDerivative2() = default;
	~EXP0_SysDerivative2() = default;

	void getFlowMapDerivativeState(state_matrix_t& A) final { A << 4, 3, -1, 0; }
	void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << 2, -1; }

	EXP0_SysDerivative2* clone() const final {
		return new EXP0_SysDerivative2(*this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_SystemDerivative : public DerivativesBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DerivativesBase<2,1,EXP0_LogicRules> Base;

	explicit EXP0_SystemDerivative(std::shared_ptr<const EXP0_LogicRules> logicRulesPtr) :
			logicRulesPtr_(std::move(logicRulesPtr)),
			activeSubsystem_(0),
			subsystemDerivativesPtr_(2)
	{
		subsystemDerivativesPtr_[0] = std::allocate_shared<EXP0_SysDerivative1, Eigen::aligned_allocator<EXP0_SysDerivative1>>(
				Eigen::aligned_allocator<EXP0_SysDerivative1>() );
		subsystemDerivativesPtr_[1] = std::allocate_shared<EXP0_SysDerivative2, Eigen::aligned_allocator<EXP0_SysDerivative2>>(
				Eigen::aligned_allocator<EXP0_SysDerivative2>() );
	}

	~EXP0_SystemDerivative() {}

	EXP0_SystemDerivative(const EXP0_SystemDerivative& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemDerivativesPtr_(2)
	{
		subsystemDerivativesPtr_[0] = Base::Ptr(other.subsystemDerivativesPtr_[0]->clone());
		subsystemDerivativesPtr_[1] = Base::Ptr(other.subsystemDerivativesPtr_[1]->clone());
		logicRulesPtr_ = other.logicRulesPtr_;
	}


	EXP0_SystemDerivative* clone() const final {
		return new EXP0_SystemDerivative(*this);
	}

	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {

		Base::setCurrentStateAndControl(t, x, u);
		activeSubsystem_ = logicRulesPtr_->getEventTimeCount(t);
		subsystemDerivativesPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
	}

	void getFlowMapDerivativeState(state_matrix_t& A) final {
		subsystemDerivativesPtr_[activeSubsystem_]->getFlowMapDerivativeState(A);
	}

	void getFlowMapDerivativeInput(state_input_matrix_t& B) final {
		subsystemDerivativesPtr_[activeSubsystem_]->getFlowMapDerivativeInput(B);
	}

public:
	int activeSubsystem_;
  	std::shared_ptr<const EXP0_LogicRules> logicRulesPtr_;
	std::vector<Base::Ptr> subsystemDerivativesPtr_;

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP0_SystemConstraint = ConstraintBase<2,1,EXP0_LogicRules>;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction1 : public CostFunctionBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP0_CostFunction1() = default;
	~EXP0_CostFunction1() = default;

	void getIntermediateCost(scalar_t& L) final { L = 0.5*(x_(1)-2.0)*(x_(1)-2.0) + 0.5*u_(0)*u_(0); }

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << 0.0, (x_(1)-2.0); }
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx)  final { dLdxx << 0.0, 0.0, 0.0, 1.0; }
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << u_; }
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 1.0; }

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

	void getTerminalCost(scalar_t& Phi) final { Phi = 0; }
	void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx.setZero(); }
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx.setZero(); }

	EXP0_CostFunction1* clone() const final {
		return new EXP0_CostFunction1(*this);
	};
};



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction2 : public CostFunctionBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP0_CostFunction2() = default;
	~EXP0_CostFunction2() = default;

	void getIntermediateCost(scalar_t& L) final { L = 0.5*(x_(1)-2.0)*(x_(1)-2.0) + 0.5*u_(0)*u_(0); }

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << 0.0, (x_(1)-2.0); }
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final { dLdxx << 0.0, 0.0, 0.0, 1.0; }
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << u_; }
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 1.0; }

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

	void getTerminalCost(scalar_t& Phi) final { Phi = 0.5*(x_(0)-4.0)*(x_(0)-4.0) + 0.5*(x_(1)-2.0)*(x_(1)-2.0); }
	void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx << (x_(0)-4.0), (x_(1)-2.0); }
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx.setIdentity(); }

	EXP0_CostFunction2* clone() const final {
		return new EXP0_CostFunction2(*this);
	};

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction : public CostFunctionBase<2,1,EXP0_LogicRules>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef CostFunctionBase<2,1,EXP0_LogicRules> Base;

	explicit EXP0_CostFunction(std::shared_ptr<const EXP0_LogicRules> logicRulesPtr) :
        logicRulesPtr_(std::move(logicRulesPtr)),
	    activeSubsystem_(0),
	    subsystemCostsPtr_(2)
	{
		subsystemCostsPtr_[0] = std::allocate_shared<EXP0_CostFunction1, Eigen::aligned_allocator<EXP0_CostFunction1>>(
				Eigen::aligned_allocator<EXP0_CostFunction1>() );
		subsystemCostsPtr_[1] = std::allocate_shared<EXP0_CostFunction2, Eigen::aligned_allocator<EXP0_CostFunction2>>(
				Eigen::aligned_allocator<EXP0_CostFunction2>() );
	}

	~EXP0_CostFunction() {}

	EXP0_CostFunction(const EXP0_CostFunction& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemCostsPtr_(2)
	{
		subsystemCostsPtr_[0] = Base::Ptr(other.subsystemCostsPtr_[0]->clone());
		subsystemCostsPtr_[1] = Base::Ptr(other.subsystemCostsPtr_[1]->clone());
        logicRulesPtr_ = other.logicRulesPtr_;
	}

	EXP0_CostFunction* clone() const final {
		return new EXP0_CostFunction(*this);
	}

	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {

		Base::setCurrentStateAndControl(t, x, u);
		activeSubsystem_ = logicRulesPtr_->getEventTimeCount(t);
		subsystemCostsPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
	}

	void getIntermediateCost(scalar_t& L) final {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCost(L);
	}

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) final {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostDerivativeState(dLdx);
	}
	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx)  final {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeState(dLdxx);
	}
	void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostDerivativeInput(dLdu);
	}
	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeInput(dLduu);
	}

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final {
		subsystemCostsPtr_[activeSubsystem_]->getIntermediateCostDerivativeInputState(dLdxu);
	}

	void getTerminalCost(scalar_t& Phi) {
		subsystemCostsPtr_[activeSubsystem_]->getTerminalCost(Phi);
	}
	void getTerminalCostDerivativeState(state_vector_t& dPhidx) final {
		subsystemCostsPtr_[activeSubsystem_]->getTerminalCostDerivativeState(dPhidx);
	}
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final {
		subsystemCostsPtr_[activeSubsystem_]->getTerminalCostSecondDerivativeState(dPhidxx);
	}

public:
	int activeSubsystem_;
    std::shared_ptr<const EXP0_LogicRules> logicRulesPtr_;
	std::vector<std::shared_ptr<CostFunctionBase<2,1,EXP0_LogicRules> > > subsystemCostsPtr_;

};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP0_SystemOperatingTrajectories = SystemOperatingPoint<2,1,EXP0_LogicRules>;

} // namespace ocs2

#endif /* EXP0_OCS2_H_ */
