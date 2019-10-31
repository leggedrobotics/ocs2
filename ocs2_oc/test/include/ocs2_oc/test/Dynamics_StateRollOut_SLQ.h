#pragma once

#include <ocs2_core/logic/rules/HybridLogicRules.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>


enum { STATE_DIM = 2, INPUT_DIM = 1 };

namespace ocs2{

class system_logic : public HybridLogicRules{
	using logic_rules_t = system_logic;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = HybridLogicRules;

	system_logic()=default;

	~system_logic() = default;

	system_logic(scalar_array_t switchingTimes, size_array_t subsystemsSequence)
	: BASE(std::move(switchingTimes), std::move(subsystemsSequence))
	{}

	void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) final
			{}

	void update() final
			{}

protected:

	void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate,
			const scalar_t& startTime,
			const scalar_t& finalTime) override {};


};



class system_dyn1 : public ControlledSystemBase<STATE_DIM,INPUT_DIM>{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	system_dyn1() = default;
	~system_dyn1() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt){

		Eigen::Matrix<double,STATE_DIM,STATE_DIM> A;
		A << 0,1,-1,0;
		Eigen::Matrix<double,STATE_DIM,INPUT_DIM> B;
		B << 0,0,0;
		Eigen::Matrix<double,STATE_DIM,1> F;
		F << 0,0,0;

		dxdt = A*x + B*u + F;
	}

	void computeJumpMap(	const scalar_t& time,
			const state_vector_t& state,
			state_vector_t& mappedState	)
	{
		mappedState[0] = state[0];
		mappedState[1] = state[1];
	}

	void computeGuardSurfaces(	const scalar_t& time,
			const state_vector_t& state,
			dynamic_vector_t& guardSurfacesValue){

		guardSurfacesValue = Eigen::Matrix<double,2,1>();
		guardSurfacesValue[0] = 1;
		guardSurfacesValue[1] = -state[0]*state[1];
	}

	system_dyn1* clone() const final{
		return new system_dyn1(*this);
	}
};

class system_dyn2 : public ControlledSystemBase<STATE_DIM,INPUT_DIM>{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	system_dyn2()  = default;
	~system_dyn2() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt){

		Eigen::Matrix<double,STATE_DIM,STATE_DIM> A;
		A << -0,3,-3,0;
		Eigen::Matrix<double,STATE_DIM,INPUT_DIM> B;
		B << 0,1,0;
		Eigen::Matrix<double,STATE_DIM,1> F;
		F << 0,0,0;

		dxdt = A*x + B*u + F;
	}

	void computeJumpMap(	const scalar_t& time,
			const state_vector_t& state,
			state_vector_t& mappedState	)
	{
		mappedState[0] = state[0];
		mappedState[1] = state[1];
	}

	void computeGuardSurfaces(	const scalar_t& time,
			const state_vector_t& state,
			dynamic_vector_t& guardSurfacesValue){

		guardSurfacesValue = Eigen::Matrix<double,2,1>();

		guardSurfacesValue[0] = state[0]*state[1];
		guardSurfacesValue[1] = 1;
	}

	system_dyn2* clone() const final{
		return new system_dyn2(*this);
	}
};

class system_dyn : public ControlledSystemBase<STATE_DIM,INPUT_DIM>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	using Base = ControlledSystemBase<2,1>;

	system_dyn(std::shared_ptr<const system_logic> logicRulesPtr) :
		logicRulesPtr_(std::move(logicRulesPtr)),
		activeSubsystem_(1),
		subsystemDynamicsPtr_(2)
	{
		subsystemDynamicsPtr_[0].reset( new system_dyn1 );
		subsystemDynamicsPtr_[1].reset( new system_dyn2 );
	}

	~system_dyn() = default;

	system_dyn* clone() const final{
		return new system_dyn(*this);
	}

	system_dyn(const system_dyn& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemDynamicsPtr_(2)
	{
		subsystemDynamicsPtr_[0].reset(other.subsystemDynamicsPtr_[0]->clone());
		subsystemDynamicsPtr_[1].reset(other.subsystemDynamicsPtr_[1]->clone());
		logicRulesPtr_ = other.logicRulesPtr_;
	}

	void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u,
			state_vector_t& dxdt) final {

		activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t);
		subsystemDynamicsPtr_[activeSubsystem_]->computeFlowMap(t, x, u, dxdt);
	}

	void ComputeJumpMaps(const scalar_t& time, const state_vector_t state, state_vector_t mappedstate)
	{
		activeSubsystem_ = logicRulesPtr_->getSubSystemTime(time);
		subsystemDynamicsPtr_[activeSubsystem_]->computeJumpMap(time, state, mappedstate);
	}

	void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue)
	{
		activeSubsystem_ = logicRulesPtr_->getSubSystemTime(time);
		subsystemDynamicsPtr_[activeSubsystem_]->computeGuardSurfaces(time,state,guardSurfacesValue);

	}

private:
	int activeSubsystem_;
	std::shared_ptr<const system_logic> logicRulesPtr_;
	std::vector<Base::Ptr> subsystemDynamicsPtr_;

};

class system_der_1 : public DerivativesBase<STATE_DIM,INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	system_der_1() = default;
	~system_der_1() = default;

	void getFlowMapDerivativeState(state_matrix_t &A) override{
		A << 0,1,-1,0;
	}

	void getFlowMapDerivativeInput(state_input_matrix_t &B) override
	{
		B << 0,0,0;
	}

	system_der_1* clone() const override{
		return new system_der_1(*this);
	}
};

class system_der_2 : public DerivativesBase<STATE_DIM,INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	system_der_2() = default;
	~system_der_2() = default;

	void getFlowMapDerivativeState(state_matrix_t &A) override{
		A << -0,3,-3,0;
	}

	void getFlowMapDerivativeInput(state_input_matrix_t &B) override
	{
		B << 0,1,0;
	}

	system_der_2* clone() const override{
		return new system_der_2 (*this);
	}
};

class system_der : public DerivativesBase<STATE_DIM,INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	using Base = DerivativesBase<STATE_DIM,INPUT_DIM>;

	system_der(std::shared_ptr<const system_logic> logicRulesPtr) :
		logicRulesPtr_(std::move(logicRulesPtr)),
		activeSubsystem_(1),
		subsystemDerPtr_(2)
	{
		subsystemDerPtr_[0].reset( new system_der_1 );
		subsystemDerPtr_[1].reset( new system_der_2 );
	}

	~system_der() = default;

	system_der* clone() const final {
		return new system_der(*this);
	}

	system_der(const system_der& other)
	: activeSubsystem_(other.activeSubsystem_),
	  subsystemDerPtr_(2)
	{
		subsystemDerPtr_[0].reset(other.subsystemDerPtr_[0]->clone());
		subsystemDerPtr_[1].reset(other.subsystemDerPtr_[1]->clone());
		logicRulesPtr_ = other.logicRulesPtr_;
	}

	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {

		Base::setCurrentStateAndControl(t, x, u);
		activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t);
		subsystemDerPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
	}

	void getFlowMapDerivativeState(state_matrix_t& A) final {
		subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeState(A);
	}

	void getFlowMapDerivativeInput(state_input_matrix_t& B) final {
		subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeInput(B);
	}


private:
	int activeSubsystem_;
	std::shared_ptr<const system_logic> logicRulesPtr_;
	std::vector<Base::Ptr> subsystemDerPtr_;



};



class system_cost : public CostFunctionBase<STATE_DIM,INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Base = CostFunctionBase<3,1>;

	system_cost() = default;
	~system_cost() = default;

	system_cost* clone() const final{
		return new system_cost(*this);
	}

	/*
	Intermediate Cost Functions
	 */
	void getIntermediateCost(scalar_t &L) final{
		L = 0.5*pow(x_[0],2) + 0.5*pow(x_[1],2) + 0.5*pow(u_[0],2);
	}

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) final {
		dLdx << x_[0] , x_[1] , 0;
	}

	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final {
		dLdxx << 1.0 , 0.0, 1.0, 0.0;
	}

	void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final {
		dLdu << 1*u_[0];
	}

	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final {
		dLduu << 1;
	}

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final {
		dLdxu.setZero();
	}
	/*
	Terminal Cost Functions
	 */
	void getTerminalCost(scalar_t& Phi) final {Phi = 0.5*pow(x_[0],2) *pow(x_[1],2);}
	void getTerminalCostDerivativeState(state_vector_t &dPhidx) final{dPhidx<< x_[0], x_[1], 0;}
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final
			{dPhidxx<<1.0 , 0.0, 1.0, 0.0;}

};

using system_constr = ConstraintBase<STATE_DIM,INPUT_DIM>;
using system_op = SystemOperatingPoint<STATE_DIM,INPUT_DIM>;

}
