/*
 * EXP2.h
 *
 *  Created on: Jan 6, 2016
 *      Author: farbod
 */

#ifndef EXP2_OCS2_H_
#define EXP2_OCS2_H_

#include "GSLQ/GLQP.h"


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP2_Sys1 : public ControlledSystemBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP2_Sys1() {}
	~EXP2_Sys1() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
		Eigen::Matrix2d A;
		A << 0.6, 1.2, -0.8, 3.4;
		Eigen::Vector2d B;
		B << 1, 1;

		dxdt = A*x + B*u;
	}

	std::shared_ptr<ControlledSystemBase<2, 1> > clone() const {
		return std::allocate_shared<EXP2_Sys1, Eigen::aligned_allocator<EXP2_Sys1>>( Eigen::aligned_allocator<EXP2_Sys1>(), *this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP2_Sys2 : public ControlledSystemBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP2_Sys2() {}
	~EXP2_Sys2() {}

	void computeDerivative( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt)  {
		Eigen::Matrix2d A;
		A << 4, 3, -1, 0;
		Eigen::Vector2d B;
		B << 2, -1;

		dxdt = A*x + B*u;
	}

	std::shared_ptr<ControlledSystemBase<2, 1> > clone() const {
		return std::allocate_shared<EXP2_Sys2, Eigen::aligned_allocator<EXP2_Sys2>>( Eigen::aligned_allocator<EXP2_Sys2>(), *this);
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP2_SysDerivative1 : public DerivativesBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP2_SysDerivative1() {};
	~EXP2_SysDerivative1() {};

	void getDerivativeState(state_matrix_t& A)  { A << 0.6, 1.2, -0.8, 3.4; }
	void getDerivativesControl(control_gain_matrix_t& B) { B << 1, 1; }

	std::shared_ptr<DerivativesBase<2,1> > clone() const {
		return std::allocate_shared<EXP2_SysDerivative1, Eigen::aligned_allocator<EXP2_SysDerivative1>>( Eigen::aligned_allocator<EXP2_SysDerivative1>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP2_SysDerivative2 : public DerivativesBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP2_SysDerivative2() {};
	~EXP2_SysDerivative2() {};

	void getDerivativeState(state_matrix_t& A)  { A << 4, 3, -1, 0; }
	void getDerivativesControl(control_gain_matrix_t& B) { B << 2, -1; }

	std::shared_ptr<DerivativesBase<2,1> > clone() const {
		return std::allocate_shared<EXP2_SysDerivative2, Eigen::aligned_allocator<EXP2_SysDerivative2>>( Eigen::aligned_allocator<EXP2_SysDerivative2>(), *this);
	}
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP2_CostFunction1 : public CostFunctionBaseOCS2<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP2_CostFunction1() {};
	~EXP2_CostFunction1() {};

	void evaluate(scalar_t& L) { L = 0.5*(x_(1)-2.0)*(x_(1)-2.0) + 0.5*u_(0)*u_(0); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << 0.0, (x_(1)-2.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 0.0, 0.0, 0.0, 1.0; }
	void controlDerivative(control_vector_t& dLdu)  { dLdu << u_; }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0; }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx.setZero(); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx.setZero(); }

	std::shared_ptr<CostFunctionBaseOCS2<2,1> > clone() const {
		return std::allocate_shared<EXP2_CostFunction1, Eigen::aligned_allocator<EXP2_CostFunction1>>( Eigen::aligned_allocator<EXP2_CostFunction1>(), *this);
	};
};



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP2_CostFunction2 : public CostFunctionBaseOCS2<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EXP2_CostFunction2() {};
	~EXP2_CostFunction2() {};

	void evaluate(scalar_t& L) { L = 0.5*(x_(1)-2.0)*(x_(1)-2.0) + 0.5*u_(0)*u_(0); }

	void stateDerivative(state_vector_t& dLdx) { dLdx << 0.0, (x_(1)-2.0); }
	void stateSecondDerivative(state_matrix_t& dLdxx)  { dLdxx << 0.0, 0.0, 0.0, 1.0; }
	void controlDerivative(control_vector_t& dLdu)  { dLdu << u_; }
	void controlSecondDerivative(control_matrix_t& dLduu)  { dLduu << 1.0; }

	void stateControlDerivative(control_feedback_t& dLdxu) { dLdxu.setZero(); }

	void terminalCost(scalar_t& Phi) { Phi = 0.5*(x_(0)-4.0)*(x_(0)-4.0) + 0.5*(x_(1)-2.0)*(x_(1)-2.0); }
	void terminalCostStateDerivative(state_vector_t& dPhidx)  { dPhidx << (x_(0)-4.0), (x_(1)-2.0); }
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  { dPhidxx.setIdentity(); }

	std::shared_ptr<CostFunctionBaseOCS2<2,1> > clone() const {
		return std::allocate_shared<EXP2_CostFunction2, Eigen::aligned_allocator<EXP2_CostFunction2>>( Eigen::aligned_allocator<EXP2_CostFunction2>(), *this);
	};

};

} // namespace ocs2

#endif /* EXP2_H_ */
