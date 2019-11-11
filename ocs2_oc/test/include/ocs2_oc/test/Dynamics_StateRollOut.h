#pragma once

#include <ocs2_core/logic/rules/HybridLogicRules.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>

namespace ocs2{

class ball_tester_logic : public HybridLogicRules{


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = HybridLogicRules;

	ball_tester_logic()=default;

	~ball_tester_logic() = default;

	ball_tester_logic(scalar_array_t switchingTimes, size_array_t subsystemsSequence)
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


class ball_tester_dyn : public ControlledSystemBase<2,1>{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ball_tester_dyn() = default;
	~ball_tester_dyn() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt){

	  Eigen::Matrix<double,2,2> A;
	    A << 0,1,0,0 ;
	    Eigen::Matrix<double,2,1> B;
	    B << 0,1;
	    Eigen::Matrix<double,2,1> F;
	    F << 0,-9.81;

	    dxdt = A*x + B*u + F;
}

	void computeJumpMap(	const scalar_t& time, 
				const state_vector_t& state,
				state_vector_t& mappedState	)
	{
			mappedState[0] = state[0];
			mappedState[1] = -state[1];
	}
				

	void computeGuardSurfaces(	const scalar_t& time, 
					const state_vector_t& state, 
					dynamic_vector_t& guardSurfacesValue){

	guardSurfacesValue = Eigen::Matrix<double,2,1>();
	guardSurfacesValue[0] = state[0];
	guardSurfacesValue[1] = -state[0] + 0.1 + time/50;
	}

	ball_tester_dyn* clone() const final{
	    return new ball_tester_dyn(*this);
	}

};

class ball_tester_der : public DerivativesBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ball_tester_der() = default;
	~ball_tester_der() = default;

	void getFlowMapDerivativeState(state_matrix_t &A) override{
		A << 0,1,0,0;
	}
	
	void getFlowMapDerivativeInput(state_input_matrix_t &B) override
	{
		B << 0,1;
	}
	
	ball_tester_der* clone() const override{
		return new ball_tester_der(*this);
	}
};

class ball_tester_cost : public CostFunctionBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Base = CostFunctionBase<2,1>;

	ball_tester_cost() = default;
	~ball_tester_cost() = default;

	ball_tester_cost* clone() const final{
		return new ball_tester_cost(*this);
	}

	/*
	Intermediate Cost Functions
	*/
	void getIntermediateCost(scalar_t &L) final{
		L = 0.5*pow(x_[0],2) + 0.5*pow(x_[1],2) + 0.05*pow(u_[0],2);
	}

	void getIntermediateCostDerivativeState(state_vector_t& dLdx) final {
		dLdx << x_[0] , x_[1];
	}

	void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final {
		dLdxx << 1.0 , 0.0 , 0.0, 1.0;
	}

	void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final {
		dLdu << 0.1*u_[0];
	}

	void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final {
		dLduu << 0.1;
	}

	void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final {
		dLdxu.setZero();
	}
	/*
	Terminal Cost Functions
	*/
	void getTerminalCost(scalar_t& Phi) final {0.5*pow(x_[0],2) + 0.5*pow(x_[1],2);}
	void getTerminalCostDerivativeState(state_vector_t &dPhidx) final{dPhidx<< x_[0], x_[1];}
	void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final 
	{dPhidxx<<1.0, 0.0, 0.0, 1.0;}

};

using ball_tester_constr = ConstraintBase<2,1>;
using ball_tester_op = SystemOperatingPoint<2,1>;

}
