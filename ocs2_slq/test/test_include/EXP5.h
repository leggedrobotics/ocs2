/*
 * EXP5.h
 *
 *  Created on: sept 15, 2016
 *      Author: markus
 */

#ifndef EXP5_OCS2_H_
#define EXP5_OCS2_H_


#include <cmath>

#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP5_Sys1 : public ControlledSystemBase<4,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double, 4, 1> state_vector_t;
	typedef Eigen::Matrix<double, 2, 1> input_vector_t;


	EXP5_Sys1() {
	}
	~EXP5_Sys1() {}

	void computeFlowMap(const double& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt)  {
		dxdt(0) = x(1);
		dxdt(1) = u(0);
		dxdt(2) = x(3);
		dxdt(3) = u(1);
	}

//	void computeConstriant2(const double& t, const state_vector_t& x, size_t& numConstraint1, input_vector_t& g1)  override {
//		numConstraint1 = 1;
//		g1(0) = x(2) - x(0) - d0;
//	}

	void computeFinalConstriant2(const double& t, const state_vector_t& x, size_t& numFinalConstraint2, input_vector_t& g2Final)  {
		numFinalConstraint2 = 1;
		g2Final(0) = x(2) - x(0) - d0;
	}

	std::shared_ptr<ControlledSystemBase<4, 2> > clone() const {
		return std::allocate_shared<EXP5_Sys1, Eigen::aligned_allocator<EXP5_Sys1>>( Eigen::aligned_allocator<EXP5_Sys1>(), *this);
	}

public:
	double d0 = 0.1;
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP5_SysDerivative1 : public DerivativesBase<4,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP5_SysDerivative1() {};
	~EXP5_SysDerivative1() {};

	void getDerivativeState(state_matrix_t& A)  {
		A.setZero();
		A(0,1) = 1.0;
		A(2,3) = 1.0;
	}
	void getDerivativesControl(state_input_matrix_t& B) {
		B.setZero();
		B(1,0) = 1.0;
		B(3,1) = 1.0;
	}
//	void getConstraint2DerivativesState(constraint2_state_matrix_t& C) {
//		C.topRows<1>() << -1.0, 0.0, 1.0, 0.0;
//	}

	void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F) {
		F.topRows<1>() << -1.0, 0.0, 1.0, 0.0;
	}

	std::shared_ptr<DerivativesBase<4,2> > clone() const {
		return std::allocate_shared<EXP5_SysDerivative1, Eigen::aligned_allocator<EXP5_SysDerivative1>>( Eigen::aligned_allocator<EXP5_SysDerivative1>(), *this);
	}
};



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP5_CostFunction1 : public CostFunctionBase<4,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP5_CostFunction1() {
		R.setIdentity();
		Q_final.setZero();
		Q_final(2,2) = 100.0;
		Q_final(3,3) = 10.0;

		x_ref.setZero();
		x_ref(2) = 1.0;
	};
	~EXP5_CostFunction1() {};

	void evaluate(scalar_t& L) {
		L = 0.5* u_.transpose()*R*u_;
	}

	void stateDerivative(state_vector_t& dLdx) {
		dLdx.setZero();
	}

	void stateSecondDerivative(state_matrix_t& dLdxx)  {
		dLdxx.setZero();
	}

	void controlDerivative(input_vector_t& dLdu)  {
		dLdu = R*u_;
	}

	void controlSecondDerivative(input_matrix_t& dLduu)  { dLduu = R; }

	void stateControlDerivative(input_state_matrix_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) {
		Phi = 0.5* (x_-x_ref).transpose()*Q_final*(x_-x_ref);
	}

	void terminalCostStateDerivative(state_vector_t& dPhidx)  {
		dPhidx = Q_final*(x_-x_ref);
	}

	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx){
		dPhidxx = Q_final;
	}

	std::shared_ptr<CostFunctionBase<4,2> > clone() const {
		return std::allocate_shared<EXP5_CostFunction1, Eigen::aligned_allocator<EXP5_CostFunction1>>( Eigen::aligned_allocator<EXP5_CostFunction1>(), *this);
	};

private:
	input_matrix_t R;
	state_matrix_t Q_final;
	state_vector_t x_ref;
};


} // namespace ocs2


#endif /* EXP5_H_ */
