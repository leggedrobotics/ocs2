/*
 * EXP4.h
 *
 *  Created on: Apr 25, 2016
 *      Author: farbod
 */

#ifndef EXP4_H_
#define EXP4_H_

#include <cmath>

#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP4_Sys : public ControlledSystemBase<6,4>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP4_Sys() {}
	~EXP4_Sys() {}

	void computeFlowMap(const double& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt)  {
		Dimensions<6,4>::state_matrix_t A;
		A.setZero(); A(0,2)=1.0; A(1,3)=1.0;

		Dimensions<6,4>::state_input_matrix_t B;
		B << Eigen::MatrixXd::Zero(2,4), Eigen::MatrixXd::Identity(4,4);

		dxdt = A*x + B*u;
	}

	void computeConstriant1(const double& t, const state_vector_t& x, const input_vector_t& u, size_t& numConstraint1, input_vector_t& g1)  override {
		numConstraint1 = 1;
		g1(0) = (x(4)-x(0))*(u(2)-x(2)) + (x(5)-x(1))*(u(3)-x(3));
	}

	std::shared_ptr<ControlledSystemBase<6,4> > clone() const {
		return std::allocate_shared<EXP4_Sys, Eigen::aligned_allocator<EXP4_Sys>>( Eigen::aligned_allocator<EXP4_Sys>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP4_SysDerivative : public DerivativesBase<6,4>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP4_SysDerivative() {};
	~EXP4_SysDerivative() {};

	void getDerivativeState(state_matrix_t& A)  {
		A.setZero(); A(0,2)=1.0; A(1,3)=1.0;
	}
	void getDerivativesControl(state_input_matrix_t& B) {
		B << Eigen::MatrixXd::Zero(2,4), Eigen::MatrixXd::Identity(4,4);
	}
	void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {
		C.topRows<1>() << -(u_(2)-x_(2)), -(u_(3)-x_(3)), -(x_(4)-x_(0)), -(x_(5)-x_(1)), (u_(2)-x_(2)), (u_(3)-x_(3));
	}
	void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) {
		D.topRows<1>() << 0.0, 0.0, (x_(4)-x_(0)), (x_(5)-x_(1));
	}

	std::shared_ptr<DerivativesBase<6,4> > clone() const {
		return std::allocate_shared<EXP4_SysDerivative, Eigen::aligned_allocator<EXP4_SysDerivative>>( Eigen::aligned_allocator<EXP4_SysDerivative>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP4_CostFunction : public CostFunctionBase<6,4>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP4_CostFunction() {
		xFinal_ << 4.5, 0.0, 0.0, 0.0, 2.0, sqrt(5.0);

		QFinal_.setZero();
		QFinal_(0,0)=100;  QFinal_(1,1)=100;  QFinal_(2,2)=10;  QFinal_(3,3)=10;
		QFinal_(4,4)=100;  QFinal_(5,5)=100;

		R_.setZero();
		R_(0,0)=1.0;  R_(1,1)=1.0;
		R_(2,2)=0.1;  R_(3,3)=0.1;
	};
	~EXP4_CostFunction() {};

	void evaluate(scalar_t& L) { L = 0.5*u_.transpose()*R_*u_; }

	void stateDerivative(state_vector_t& dLdx) { dLdx.setZero(); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx.setZero(); }
	void controlDerivative(input_vector_t& dLdu)  { dLdu = R_*u_; }
	void controlSecondDerivative(input_matrix_t& dLduu)  { dLduu = R_; }

	void stateControlDerivative(input_state_matrix_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0.5*(x_-xFinal_).transpose()*QFinal_*(x_-xFinal_); }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx = QFinal_*(x_-xFinal_); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx = QFinal_; }

	std::shared_ptr<CostFunctionBase<6,4> > clone() const {
		return std::allocate_shared<EXP4_CostFunction, Eigen::aligned_allocator<EXP4_CostFunction>>( Eigen::aligned_allocator<EXP4_CostFunction>(), *this);
	};

private:
	state_vector_t xFinal_;
	state_matrix_t QFinal_;
	input_matrix_t R_;
};

#endif /* EXP4_H_ */

} // namespace ocs2
