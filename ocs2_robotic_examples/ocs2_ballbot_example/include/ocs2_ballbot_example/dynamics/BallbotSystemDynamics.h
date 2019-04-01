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

namespace ocs2 {
namespace ballbot {

/**
 * BallbotSystemDynamics class
 * This class implements the dynamics for the rezero robot.
 * The equations of motion are generated through robcogen, using the following set of generalized coordinates:
 * (ballPosition_x, ballPosition_y, eulerAnglesZyx theta_z, eulerAnglesZyx theta_y, eulerAnglesZyx theta_x, anypulator_joint_1 qa1, anypulator_joint_2 qa2, anypulator_joint_3 qa3)
 * The control input are u = (torque_wheel1, torque_wheel2, torque_wheel3, torque_anypulator_joint1, torque_anypulator_joint2, torque_anypulator_joint3)
 * The transformation from wheels torques to joint accelerations is computed through Mathematica computations and has been tested on the robot
 */
class BallbotSystemDynamics : public SystemDynamicsBaseAD<BallbotSystemDynamics, ballbot::STATE_DIM_, ballbot::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<BallbotSystemDynamics> Ptr;
	typedef std::shared_ptr<const BallbotSystemDynamics> ConstPtr;

	typedef ocs2::SystemDynamicsBaseAD<BallbotSystemDynamics, ballbot::STATE_DIM_, ballbot::INPUT_DIM_> BASE;
	typedef typename BASE::scalar_t scalar_t;
	typedef typename BASE::state_vector_t state_vector_t;
	typedef typename BASE::state_matrix_t state_matrix_t;
	typedef typename BASE::input_vector_t input_vector_t;

	/**
	 * Constructor.
	 *
	 * @param [in] dynamicLibraryIsCompiled: Whether a library is already complied.
	 */
	BallbotSystemDynamics(const bool& dynamicLibraryIsCompiled = false)
	: BASE(dynamicLibraryIsCompiled)
	{}

	/**
	 * Destructor
	 */
	~BallbotSystemDynamics() = default;

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

		const SCALAR_T& yaw   = state(2);
		const SCALAR_T& pitch = state(3);
		const SCALAR_T& roll  = state(4);

		S_transposed(0, 0) = -((cos(yaw)*sin(roll)+(cos(pitch)-cos(roll)*sin(pitch))*sin(yaw))/(pow(2,0.5)*wheelRadius_));
		S_transposed(0, 1) = (-cos(yaw)*(pow(3,0.5)*cos(roll)+2*sin(roll))+(cos(pitch)+(2*cos(roll)-pow(3,0.5)*sin(roll))*sin(pitch))*sin(yaw))/(2*pow(2,0.5)*wheelRadius_);
		S_transposed(0, 2) = (cos(yaw)*(pow(3,0.5)*cos(roll)-2*sin(roll))+(cos(pitch)+(2*cos(roll)+pow(3,0.5)*sin(roll))*sin(pitch))*sin(yaw))/(2*pow(2,0.5)*wheelRadius_);

		S_transposed(1, 0) = (cos(yaw)*(cos(pitch)-cos(roll)*sin(pitch))-sin(roll)*sin(yaw))/(pow(2,0.5)*wheelRadius_);
		S_transposed(1, 1) = -((cos(yaw)*(cos(pitch)+(2*cos(roll)-pow(3,0.5)*sin(roll))*sin(pitch))+(pow(3,0.5)*cos(roll)+2*sin(roll))*sin(yaw))/(2*pow(2,0.5)*wheelRadius_));
		S_transposed(1, 2) = (-cos(yaw)*(cos(pitch)+(2*cos(roll)+pow(3,0.5)*sin(roll))*sin(pitch))+(pow(3,0.5)*cos(roll)-2*sin(roll))*sin(yaw))/(2*pow(2,0.5)*wheelRadius_);

		S_transposed(2, 0) = -(((ballRadius_ + wheelRadius_)*(cos(roll)*cos(pitch)+sin(pitch)))/(pow(2, 0.5)*wheelRadius_));
		S_transposed(2, 1) = -(((ballRadius_ + wheelRadius_)*(2*cos(roll)*cos(pitch)-pow(3, 0.5)*cos(pitch)*sin(roll)-sin(pitch)))/(2*pow(2, 0.5)*wheelRadius_));
		S_transposed(2, 2) = -(((ballRadius_ + wheelRadius_)*(2*cos(roll)*cos(pitch)+pow(3, 0.5)*cos(pitch)*sin(roll)-sin(pitch)))/(2*pow(2, 0.5)*wheelRadius_));

		S_transposed(3, 0) = ((ballRadius_ + wheelRadius_)*sin(roll))/(pow(2, 0.5)*wheelRadius_);
		S_transposed(3, 1) = ((ballRadius_ + wheelRadius_)*(pow(3, 0.5)*cos(roll)+2*sin(roll)))/(2*pow(2, 0.5)*wheelRadius_);
		S_transposed(3, 2) = -(((ballRadius_ + wheelRadius_)*(pow(3, 0.5)*cos(roll)-2*sin(roll)))/(2*pow(2, 0.5)*wheelRadius_));

		S_transposed(4, 0) = (ballRadius_ + wheelRadius_)/(pow(2, 0.5)*wheelRadius_);
		S_transposed(4, 1) = -(wheelRadius_ + ballRadius_)/(2*pow(2, 0.5)*wheelRadius_);
		S_transposed(4, 2) = -(wheelRadius_ + ballRadius_)/(2*pow(2, 0.5)*wheelRadius_);


		// test for the autogenerated code
		iit::Ballbot::tpl::JointState<SCALAR_T> qdd;
		iit::Ballbot::tpl::JointState<SCALAR_T> new_input = S_transposed*input;

		typedef typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait trait_t;
		iit::Ballbot::dyn::tpl::InertiaProperties<trait_t> inertias;
		iit::Ballbot::tpl::MotionTransforms<trait_t> transforms;
		iit::Ballbot::dyn::tpl::ForwardDynamics<trait_t> forward_dyn(inertias, transforms);
		forward_dyn.fd(qdd, state.template head(5), state.template tail(5), new_input);


		// dxdt
		stateDerivative.template head<5>() = state.template tail<5>();
		stateDerivative.template tail<5>() = qdd;
	}

private:
	const scalar_t wheelRadius_ = 0.064;
	const scalar_t ballRadius_ = 0.125;

};

} // namespace ballbot
} // namespace ocs2

#endif /* BALLBOTSYSTEMDYNAMICS_OCS2_BALLBOT_OCS2_H_ */
