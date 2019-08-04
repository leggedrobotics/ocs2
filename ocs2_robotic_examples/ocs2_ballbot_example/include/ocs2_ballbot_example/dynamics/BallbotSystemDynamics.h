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

#ifndef BALLBOTSYSTEMDYNAMICS_OCS2_BALLBOT_OCS2_H_
#define BALLBOTSYSTEMDYNAMICS_OCS2_BALLBOT_OCS2_H_

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

// robcogen
#include <ocs2_robotic_tools/rbd_libraries/robcogen/iit/rbd/rbd.h>
#include <ocs2_robotic_tools/rbd_libraries/robcogen/iit/rbd/traits/TraitSelector.h>

// ballbot
#include "ocs2_ballbot_example/definitions.h"
#include "ocs2_ballbot_example/generated/forward_dynamics.h"
#include "ocs2_ballbot_example/generated/inverse_dynamics.h"
#include "ocs2_ballbot_example/generated/inertia_properties.h"
#include "ocs2_ballbot_example/generated/transforms.h"
#include "ocs2_ballbot_example/generated/jsim.h"
#include <ocs2_ballbot_example/BallbotParameters.h>

namespace ocs2 {
namespace ballbot {

/**
 * BallbotSystemDynamics class
 * This class implements the dynamics for the ballbot.
 * The equations of motion are generated through robcogen, using the following set of generalized coordinates:
 * (ballPosition_x, ballPosition_y, eulerAnglesZyx theta_z, eulerAnglesZyx theta_y, eulerAnglesZyx theta_x)
 * The control input are u = (torque_wheel1, torque_wheel2, torque_wheel3)
 */
class BallbotSystemDynamics : public SystemDynamicsBaseAD<BallbotSystemDynamics, ballbot::STATE_DIM_, ballbot::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Ptr = std::shared_ptr<BallbotSystemDynamics>;
	using ConstPtr = std::shared_ptr<const BallbotSystemDynamics>;

	using BASE = ocs2::SystemDynamicsBaseAD<BallbotSystemDynamics, ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;
	using scalar_t = typename BASE::scalar_t;
	using state_vector_t = typename BASE::state_vector_t;
	using state_matrix_t = typename BASE::state_matrix_t;
	using input_vector_t = typename BASE::input_vector_t;

    using ballbot_parameters_t = BallbotParameters<scalar_t>;

	/**
	 * Constructor.
	 *
	 * @param [in] dynamicLibraryIsCompiled: Whether a library is already complied.
	 */
	BallbotSystemDynamics(const bool& dynamicLibraryIsCompiled = false)
	: BASE(dynamicLibraryIsCompiled)
	{
	    wheelRadius_ = param_.wheelRadius_;
	    ballRadius_ = param_.ballRadius_;
	    squaredRadiusRatio_ = (ballRadius_ - wheelRadius_)/(pow(2, 0.5)*wheelRadius_);
	}

	/**
	 * Destructor
	 */
	~BallbotSystemDynamics() override = default;

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

		// compute actuationMatrix S_transposed which appears in the equations: M(q)\dot v + h = S^(transpose)\tau
		Eigen::Matrix<SCALAR_T, 5, 3> S_transposed = Eigen::Matrix<SCALAR_T, 5, 3>::Zero();

		const SCALAR_T& cyaw = cos(state(2));
		const SCALAR_T& cpitch = cos(state(3));
		const SCALAR_T& croll = cos(state(4));

		const SCALAR_T& syaw = sin(state(2));
		const SCALAR_T& spitch = sin(state(3));
		const SCALAR_T& sroll = sin(state(4));

		S_transposed(0, 0) = -(sqrt(2)*(cyaw*sroll - croll*spitch*syaw))/(2*wheelRadius_) - (sqrt(2)*cpitch*syaw)/(2*wheelRadius_);
		S_transposed(0, 1) = (sqrt(2)*cpitch*syaw)/(4*wheelRadius_) - (sqrt(2)*(cyaw*sroll - croll*spitch*syaw))/(2*wheelRadius_) - (sqrt(2)*sqrt(3)*(croll*cyaw + spitch*sroll*syaw))/(4*wheelRadius_);
		S_transposed(0, 2) = (sqrt(2)*cpitch*syaw)/(4*wheelRadius_) - (sqrt(2)*(cyaw*sroll - croll*spitch*syaw))/(2*wheelRadius_) + (sqrt(2)*sqrt(3)*(croll*cyaw + spitch*sroll*syaw))/(4*wheelRadius_);

		S_transposed(1, 0) = (sqrt(2)*cpitch*cyaw)/(2*wheelRadius_) - (sqrt(2)*(sroll*syaw + croll*cyaw*spitch))/(2*wheelRadius_);
		S_transposed(1, 1) = - (sqrt(2)*(sroll*syaw + croll*cyaw*spitch))/(2*wheelRadius_) - (sqrt(2)*cpitch*cyaw)/(4*wheelRadius_) - (sqrt(2)*sqrt(3)*(croll*syaw - cyaw*spitch*sroll))/(4*wheelRadius_);
		S_transposed(1, 2) = (sqrt(2)*sqrt(3)*(croll*syaw - cyaw*spitch*sroll))/(4*wheelRadius_) - (sqrt(2)*cpitch*cyaw)/(4*wheelRadius_) - (sqrt(2)*(sroll*syaw + croll*cyaw*spitch))/(2*wheelRadius_);

		S_transposed(2, 0) = -(sqrt(2)*ballRadius_*(spitch + cpitch*croll))/(2*wheelRadius_);
		S_transposed(2, 1) = (sqrt(2)*ballRadius_*(spitch - 2*cpitch*croll + sqrt(3)*cpitch*sroll))/(4*wheelRadius_);
		S_transposed(2, 2) = -(sqrt(2)*ballRadius_*(2*cpitch*croll - spitch + sqrt(3)*cpitch*sroll))/(4*wheelRadius_);

		S_transposed(3, 0) = (sqrt(2)*ballRadius_*sroll)/(2*wheelRadius_);
		S_transposed(3, 1) = (sqrt(2)*ballRadius_*(2*sroll + sqrt(3)*croll))/(4*wheelRadius_);
		S_transposed(3, 2) = (sqrt(2)*ballRadius_*(2*sroll - sqrt(3)*croll))/(4*wheelRadius_);

		S_transposed(4, 0) = (sqrt(2)*ballRadius_)/(2*wheelRadius_);
		S_transposed(4, 1) = -(sqrt(2)*ballRadius_)/(4*wheelRadius_);
		S_transposed(4, 2) = -(sqrt(2)*ballRadius_)/(4*wheelRadius_);

		// test for the autogenerated code
		iit::Ballbot::tpl::JointState<SCALAR_T> qdd;

		using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
		iit::Ballbot::dyn::tpl::InertiaProperties<trait_t> inertias;
		iit::Ballbot::tpl::MotionTransforms<trait_t> transforms;
		iit::Ballbot::dyn::tpl::ForwardDynamics<trait_t> forward_dyn(inertias, transforms);
		forward_dyn.fd(qdd, state.template head<5>(), state.template tail<5>(), S_transposed*input);


		// dxdt
		stateDerivative.template head<5>() = state.template tail<5>();
		stateDerivative.template tail<5>() = qdd;
	}

private:

    ballbot_parameters_t param_;
	scalar_t wheelRadius_;
	scalar_t ballRadius_;
	scalar_t squaredRadiusRatio_;

};

} // namespace ballbot
} // namespace ocs2

#endif /* BALLBOTSYSTEMDYNAMICS_OCS2_BALLBOT_OCS2_H_ */
