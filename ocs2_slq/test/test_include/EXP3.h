/*
 * EXP3.h
 *
 *  Created on: Apr 15, 2016
 *      Author: farbod
 */

#ifndef EXP3_OCS2_H_
#define EXP3_OCS2_H_


#include <cmath>

#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_Sys1 : public ControlledSystemBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_Sys1() {}
	~EXP3_Sys1() {}

	void computeDerivative(const double& t, const Eigen::Vector2d& x, const Eigen::Vector2d& u, Eigen::Vector2d& dxdt)  {
//		dxdt(0) = x(0) + u(0)*sin(x(0)) - u(1)*cos(x(1));
//		dxdt(1) = -x(1) - u(0)*cos(x(1)) + u(1)*sin(x(0));
		dxdt(0) = x(0) + u(0)*sin(x(0));
		dxdt(1) = -x(1) - u(0)*cos(x(1));

	}

	void computeConstriant1(const double& t, const Eigen::Vector2d& x, const Eigen::Vector2d& u, size_t& numConstraint1, Eigen::Vector2d& g1)  override {

		numConstraint1 = 1;
		g1(0) = u(1)*sin(x(0)) - u(1)*cos(x(1)) + 0.1*u(1) - 1;
	}

	std::shared_ptr<ControlledSystemBase<2, 2> > clone() const {
		return std::allocate_shared<EXP3_Sys1, Eigen::aligned_allocator<EXP3_Sys1>>( Eigen::aligned_allocator<EXP3_Sys1>(), *this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_Sys2 : public ControlledSystemBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_Sys2() {}
	~EXP3_Sys2() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Vector2d& u, Eigen::Vector2d& dxdt)  {
//		dxdt(0) = x(1) + u(0)*sin(x(1)) - u(1)*cos(x(0));
//		dxdt(1) = -x(0) - u(0)*cos(x(0)) + u(1)*sin(x(1));
		dxdt(0) = x(1) + u(0)*sin(x(1));
		dxdt(1) = -x(0) - u(0)*cos(x(0));

	}

	void computeConstriant1(const double& t, const Eigen::Vector2d& x, const Eigen::Vector2d& u, size_t& numConstraint1, Eigen::Vector2d& g1)  override {

		numConstraint1 = 1;
		g1(0) = u(1)*sin(x(1)) - u(1)*cos(x(0)) + 0.1*u(1) - 1;
	}

	std::shared_ptr<ControlledSystemBase<2, 2> > clone() const {
		return std::allocate_shared<EXP3_Sys2, Eigen::aligned_allocator<EXP3_Sys2>>( Eigen::aligned_allocator<EXP3_Sys2>(), *this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_Sys3 : public ControlledSystemBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_Sys3() {}
	~EXP3_Sys3() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Vector2d& u, Eigen::Vector2d& dxdt)  {
//		dxdt(0) = -x(0) - u(0)*sin(x(0)) + u(1)*cos(x(1));
//		dxdt(1) = x(1) + u(0)*cos(x(1)) - u(1)*sin(x(0));
		dxdt(0) = -x(0) - u(0)*sin(x(0));
		dxdt(1) = x(1) + u(0)*cos(x(1));
	}

	void computeConstriant1(const double& t, const Eigen::Vector2d& x, const Eigen::Vector2d& u, size_t& numConstraint1, Eigen::Vector2d& g1)  override {

		numConstraint1 = 1;
		g1(0) = - u(1)*sin(x(0)) + u(1)*cos(x(1)) + 0.1*u(1) - 1;
	}

	std::shared_ptr<ControlledSystemBase<2, 2> > clone() const {
		return std::allocate_shared<EXP3_Sys3, Eigen::aligned_allocator<EXP3_Sys3>>( Eigen::aligned_allocator<EXP3_Sys3>(), *this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_SysDerivative1 : public DerivativesBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_SysDerivative1() {};
	~EXP3_SysDerivative1() {};

	void getDerivativeState(state_matrix_t& A)  {
//		A << 1+u_(0)*cos(x_(0)), u_(1)*sin(x_(1)), u_(1)*cos(x_(0)), -1+u_(0)*sin(x_(1));
		A << 1+u_(0)*cos(x_(0)), 0.0, 0.0, -1+u_(0)*sin(x_(1));
	}
	void getDerivativesControl(control_gain_matrix_t& B) {
//		B << sin(x_(0)), -cos(x_(1)), -cos(x_(1)), sin(x_(0));
		B << sin(x_(0)), 0.0, -cos(x_(1)), 0.0;
	}
	void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {
		C.topRows<1>() << u_(1)*cos(x_(0)), u_(1)*sin(x_(1));
	}
	void getConstraint1DerivativesControl(constraint1_control_matrix_t& D) {
		D.topRows<1>() << 0.0, sin(x_(0))-cos(x_(1))+0.1;
	}

	std::shared_ptr<DerivativesBase<2,2> > clone() const {
		return std::allocate_shared<EXP3_SysDerivative1, Eigen::aligned_allocator<EXP3_SysDerivative1>>( Eigen::aligned_allocator<EXP3_SysDerivative1>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_SysDerivative2 : public DerivativesBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_SysDerivative2() {};
	~EXP3_SysDerivative2() {};

	void getDerivativeState(state_matrix_t& A)  {
//		A << u_(1)*sin(x_(1)), 1+u_(0)*cos(x_(1)), -1+u_(0)*sin(x_(0)), u_(1)*cos(x_(1));
		A << 0.0, 1+u_(0)*cos(x_(1)), -1+u_(0)*sin(x_(0)), 0.0;
	}
	void getDerivativesControl(control_gain_matrix_t& B) {
//		B << sin(x_(1)), -cos(x_(0)), -cos(x_(0)), sin(x_(1));
		B << sin(x_(1)), 0.0, -cos(x_(0)), 0.0;
	}
	void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {
		C.topRows<1>() << u_(1)*sin(x_(1)), -u_(1)*cos(x_(1));
	}
	void getConstraint1DerivativesControl(constraint1_control_matrix_t& D) {
		D.topRows<1>() << 0.0, sin(x_(1))-cos(x_(0))+0.1;
	}

	std::shared_ptr<DerivativesBase<2,2> > clone() const {
		return std::allocate_shared<EXP3_SysDerivative2, Eigen::aligned_allocator<EXP3_SysDerivative2>>( Eigen::aligned_allocator<EXP3_SysDerivative2>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_SysDerivative3 : public DerivativesBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_SysDerivative3() {};
	~EXP3_SysDerivative3() {};

	void getDerivativeState(state_matrix_t& A)  {
//		A << -1-u_(0)*cos(x_(0)), -u_(1)*sin(x_(1)), -u_(1)*cos(x_(0)), 1-u_(0)*sin(x_(1));
		A << -1-u_(0)*cos(x_(0)), 0.0, 0.0, 1-u_(0)*sin(x_(1));
	}
	void getDerivativesControl(control_gain_matrix_t& B) {
//		B << -sin(x_(0)), cos(x_(1)), cos(x_(1)), -sin(x_(0));
		B << -sin(x_(0)), 0.0, cos(x_(1)), 0.0;
	}
	void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {
		C.topRows<1>() << -u_(1)*cos(x_(0)), -u_(1)*sin(x_(1));
	}
	void getConstraint1DerivativesControl(constraint1_control_matrix_t& D) {
		D.topRows<1>() << 0.0, -sin(x_(0))+cos(x_(1))+0.1;
	}

	std::shared_ptr<DerivativesBase<2,2> > clone() const {
		return std::allocate_shared<EXP3_SysDerivative3, Eigen::aligned_allocator<EXP3_SysDerivative3>>( Eigen::aligned_allocator<EXP3_SysDerivative3>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_CostFunction1 : public CostFunctionBase<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_CostFunction1() {};
	~EXP3_CostFunction1() {};

	void evaluate(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2) + + 0.5*alpha_*pow(u_(1), 2); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void controlDerivative(control_vector_t& dLdu)  { dLdu << u_(0), alpha_*u_(1); }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0, 0.0, 0.0, alpha_; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0; }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

	std::shared_ptr<CostFunctionBase<2,2> > clone() const {
		return std::allocate_shared<EXP3_CostFunction1, Eigen::aligned_allocator<EXP3_CostFunction1>>( Eigen::aligned_allocator<EXP3_CostFunction1>(), *this);
	};

private:
	double alpha_ = 0.01;
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP3_CostFunction2 : public CostFunctionBaseOCS2<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_CostFunction2() {};
	~EXP3_CostFunction2() {};

	void evaluate(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2) + 0.5*alpha_*pow(u_(1), 2); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void controlDerivative(control_vector_t& dLdu)  { dLdu << u_(0), alpha_*u_(1); }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0, 0.0, 0.0, alpha_; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0; }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

	std::shared_ptr<CostFunctionBaseOCS2<2,2> > clone() const {
		return std::allocate_shared<EXP3_CostFunction2, Eigen::aligned_allocator<EXP3_CostFunction2>>( Eigen::aligned_allocator<EXP3_CostFunction2>(), *this);
	};

private:
	double alpha_ = 0.01;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

class EXP3_CostFunction3 : public CostFunctionBaseOCS2<2,2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP3_CostFunction3() {};
	~EXP3_CostFunction3() {};

	void evaluate(scalar_t& L) { L = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2) + 0.5*pow(u_(0), 2) + 0.5*alpha_*pow(u_(1), 2); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << (x_(0)-1.0), (x_(1)+1.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 1.0, 0.0, 0.0, 1.0; }
	void controlDerivative(control_vector_t& dLdu)  { dLdu << u_(0), alpha_*u_(1); }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0, 0.0, 0.0, alpha_; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0.5*pow(x_(0)-1.0, 2) + 0.5*pow(x_(1)+1.0, 2); }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx << (x_(0)-1.0), (x_(1)+1.0); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx << 1.0, 0.0, 0.0, 1.0; }

	std::shared_ptr<CostFunctionBaseOCS2<2,2> > clone() const {
		return std::allocate_shared<EXP3_CostFunction3, Eigen::aligned_allocator<EXP3_CostFunction3>>( Eigen::aligned_allocator<EXP3_CostFunction3>(), *this);
	};

private:
	double alpha_ = 0.01;

};


} // namespace ocs2


#endif /* EXP3_H_ */
