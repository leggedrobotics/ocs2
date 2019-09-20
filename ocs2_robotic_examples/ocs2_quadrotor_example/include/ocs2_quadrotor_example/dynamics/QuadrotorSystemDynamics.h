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

#pragma once

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include "ocs2_quadrotor_example/QuadrotorParameters.h"
#include "ocs2_quadrotor_example/definitions.h"

namespace ocs2 {
namespace quadrotor {

class QuadrotorSystemDynamics final : public ControlledSystemBase<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<QuadrotorSystemDynamics>;
  using ConstPtr = std::shared_ptr<const QuadrotorSystemDynamics>;

  using BASE = ControlledSystemBase<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_>;
  using scalar_t = typename BASE::scalar_t;
  using state_vector_t = typename BASE::state_vector_t;
  using input_vector_t = typename BASE::input_vector_t;
  using quadrotor_parameters_t = QuadrotorParameters<scalar_t>;

  /**
   * Constructor
   *
   * @param quadrotorParameters: Quadrotor parameters.
   */
  QuadrotorSystemDynamics(const quadrotor_parameters_t& quadrotorParameters) : param_(quadrotorParameters) {}

  /**
   * Destructor
   */
  ~QuadrotorSystemDynamics() override = default;

  QuadrotorSystemDynamics* clone() const override { return new QuadrotorSystemDynamics(*this); }

  void computeFlowMap(const scalar_t& time, const state_vector_t& state, const input_vector_t& input,
                      state_vector_t& stateDerivative) override {
    // angular velocities to Euler angle Derivatives transformation
    Eigen::Matrix<scalar_t, 3, 1> eulerAngle = state.segment<3>(3);
    Eigen::Matrix<scalar_t, 3, 3> T = AngularVelocitiesToEulerAngleDerivativesMatrix<scalar_t>(eulerAngle);

    // positions
    scalar_t qxQ = state(0);  // x
    scalar_t qyQ = state(1);  // y
    scalar_t qzQ = state(2);  // z

    // euler angles xyz
    scalar_t qph = state(3);
    scalar_t qth = state(4);
    scalar_t qps = state(5);

    // positions derivatives
    scalar_t dqxQ = state(6);  // x
    scalar_t dqyQ = state(7);  // y
    scalar_t dqzQ = state(8);  // z

    // angular velocity xyz
    scalar_t dqph = state(9);
    scalar_t dqth = state(10);
    scalar_t dqps = state(11);
    // Euler angle derivatives
    Eigen::Matrix<scalar_t, 3, 1> eulerAngleDerivatives = T * state.segment<3>(9);

    // Applied force and momentums
    scalar_t Fz = input(0);
    scalar_t Mx = input(1);
    scalar_t My = input(2);
    scalar_t Mz = input(3);

    scalar_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = std::cos(qth);
    t4 = std::sin(qth);
    t5 = 1.0 / param_.Thxxyy_;
    t6 = std::cos(qps);
    t7 = std::sin(qps);
    t8 = dqph * dqph;
    t9 = qth * 2.0;
    t10 = std::sin(t9);
    t11 = 1.0 / t3;
    t12 = param_.Thzz_ * param_.Thzz_;
    t13 = t3 * t3;

    stateDerivative(0) = dqxQ;
    stateDerivative(1) = dqyQ;
    stateDerivative(2) = dqzQ;
    stateDerivative(3) = eulerAngleDerivatives(0);
    stateDerivative(4) = eulerAngleDerivatives(1);
    stateDerivative(5) = eulerAngleDerivatives(2);
    stateDerivative(6) = Fz * t2 * t4;
    stateDerivative(7) = -Fz * t2 * t3 * std::sin(qph);

    stateDerivative(8) = t2 * (param_.quadrotorMass_ * param_.gravity_ - Fz * t3 * std::cos(qph)) * (-1.0);
    stateDerivative(9) =
        -t5 * t11 *
        (-Mx * t6 + My * t7 + param_.Thzz_ * dqps * dqth - param_.Thxxyy_ * dqph * dqth * t4 * (2.0) + param_.Thzz_ * dqph * dqth * t4);
    stateDerivative(10) = t5 * (Mx * t7 + My * t6 - param_.Thxxyy_ * t8 * t10 * (1.0 / 2.0) + param_.Thzz_ * t8 * t10 * (1.0 / 2.0) +
                                param_.Thzz_ * dqph * dqps * t3);
    stateDerivative(11) = (t5 * t11 *
                           (Mz * param_.Thxxyy_ * t3 + dqph * dqth * t12 - dqph * dqth * t12 * t13 + dqps * dqth * t4 * t12 -
                            param_.Thxxyy_ * param_.Thzz_ * dqph * dqth * (2.0) - Mx * param_.Thzz_ * t4 * t6 +
                            My * param_.Thzz_ * t4 * t7 + param_.Thxxyy_ * param_.Thzz_ * dqph * dqth * t13)) /
                          param_.Thzz_;
  }

 private:
  quadrotor_parameters_t param_;
};

}  // namespace quadrotor
}  // namespace ocs2
