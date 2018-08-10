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
