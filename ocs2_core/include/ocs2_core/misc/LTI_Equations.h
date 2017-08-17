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

template <int STATE_DIM>
class LTI_Equations : public SystemBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double, STATE_DIM, 1> 		state_vector_t;
	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;

	LTI_Equations() {}
	~LTI_Equations() {}

	void setData(const state_matrix_t& Gm, const state_vector_t& Gv) {
		Gm_ = Gm;
		Gv_ = Gv;
	}

	void computeDerivative(const double& z, const state_vector_t& x, state_vector_t& dx) {
		// dx = Gm x + Gv
		dx = Gm_*x + Gv_;
	}


private:
	// members required in computeDerivative
	state_matrix_t Gm_;
	state_vector_t Gv_;
};

} // namespace ocs2


#endif /* OCS2_LTI_EQUATIONS_H_ */
