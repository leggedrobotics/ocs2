/*
 * LTI_Equations.h
 *
 *  Created on: Jun 27, 2017
 *      Author: farbod
 */

#ifndef OCS2_LTI_EQUATIONS_H_
#define OCS2_LTI_EQUATIONS_H_

#include "ocs2_core/dynamics/SystemBase.h"

namespace ocs2{

/**
 * LTI system class for linear defined as \f$ dx/dt = G_m*x(t) + G_v \f$.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class LTI_Equations : public SystemBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double, STATE_DIM, 1> 		state_vector_t;
	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;

	LTI_Equations() {}
	~LTI_Equations() {}

	/**
	 * Set the LTI system coefficients, \f$ dx/dt = G_m*x(t) + G_v \f$.
	 * @param [in] Gm: G_m matrix.
	 * @param [in] Gv: G_v vector.
	 */
	void setData(const state_matrix_t& Gm, const state_vector_t& Gv) {
		Gm_ = Gm;
		Gv_ = Gv;
	}

	/**
	 * Computes the LTI system dynamics.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [out] dxdt: Current state time derivative
	 */
	void computeDerivative(const double& t, const state_vector_t& x, state_vector_t& dxdt) override {
		dxdt = Gm_*x + Gv_;
	}


private:
	// members required in computeDerivative
	state_matrix_t Gm_;
	state_vector_t Gv_;
};

} // namespace ocs2


#endif /* OCS2_LTI_EQUATIONS_H_ */
