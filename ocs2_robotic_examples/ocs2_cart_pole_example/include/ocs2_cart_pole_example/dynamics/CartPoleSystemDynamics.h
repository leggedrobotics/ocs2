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

#ifndef CART_POLE_SYSTEM_DYNAMICS_OCS2_H_
#define CART_POLE_SYSTEM_DYNAMICS_OCS2_H_

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>

#include "ocs2_cart_pole_example/definitions.h"
#include "ocs2_cart_pole_example/CartPoleParameters.h"

namespace ocs2 {
namespace cartpole {

class CartPoleSytemDynamics
		: public SystemDynamicsBaseAD<CartPoleSytemDynamics, cartpole::STATE_DIM_, cartpole::INPUT_DIM_, NullLogicRules, 1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<CartPoleSytemDynamics> Ptr;
	typedef std::shared_ptr<const CartPoleSytemDynamics> ConstPtr;

	typedef SystemDynamicsBaseAD<CartPoleSytemDynamics, cartpole::STATE_DIM_, cartpole::INPUT_DIM_, NullLogicRules, 1> BASE;
	typedef typename BASE::scalar_t scalar_t;
	typedef typename BASE::state_vector_t state_vector_t;
	typedef typename BASE::state_matrix_t state_matrix_t;
	typedef typename BASE::input_vector_t input_vector_t;
	typedef typename BASE::state_input_matrix_t state_input_matrix_t;
	typedef CartPoleParameters<scalar_t> cart_pole_parameters_t;

	CartPoleSytemDynamics(const cart_pole_parameters_t& cartPoleParameters)
	: param_(cartPoleParameters)
	{}

	~CartPoleSytemDynamics() = default;

	/**
	 * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	template <typename SCALAR_T>
	void systemFlowMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, BASE::state_dim_, 1>& state,
			const Eigen::Matrix<SCALAR_T, BASE::input_dim_, 1>& input,
			Eigen::Matrix<SCALAR_T, BASE::state_dim_, 1>& stateDerivative) {

		const SCALAR_T cosTheta = cos(state(0));
		const SCALAR_T sinTheta = sin(state(0));

		// Inertia tensor
		Eigen::Matrix<SCALAR_T, 2, 2> I;
		I << 	static_cast<SCALAR_T>(param_.poleSteinerMoi_),
				static_cast<SCALAR_T>(-param_.poleMass_*param_.poleHalfLength_*cosTheta),
				static_cast<SCALAR_T>(-param_.poleMass_*param_.poleHalfLength_*cosTheta),
				static_cast<SCALAR_T>(param_.cartMass_ + param_.poleMass_);

		// RHS
		Eigen::Matrix<SCALAR_T, 2, 1> rhs(
				param_.poleMass_*param_.poleHalfLength_*param_.gravity_*sinTheta,
				input(0) - param_.poleMass_*param_.poleHalfLength_*pow(state(2),2)*sinTheta);

		// dxdt
		stateDerivative(0) = state(2);
		stateDerivative(1) = state(3);
		stateDerivative.template tail<2>() = I.inverse()*rhs;
	}

private:
	cart_pole_parameters_t param_;
};

} // namespace cartpole
} // namespace ocs2

#endif //CART_POLE_SYSTEM_DYNAMICS_OCS2_H_
