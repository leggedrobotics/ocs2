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

#include <ocs2_core/dynamics/DerivativesBase.h>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include "ocs2_quadrotor_example/QuadrotorParameters.h"
#include "ocs2_quadrotor_example/definitions.h"

namespace ocs2 {
namespace quadrotor {

class QuadrotorDynamicsDerivatives final : public DerivativesBase<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<QuadrotorDynamicsDerivatives>;
  using ConstPtr = std::shared_ptr<const QuadrotorDynamicsDerivatives>;

  using BASE = DerivativesBase<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_>;
  using scalar_t = typename BASE::scalar_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using input_vector_t = typename BASE::input_vector_t;
  using state_input_matrix_t = typename BASE::state_input_matrix_t;
  using quadrotor_parameters_t = QuadrotorParameters<scalar_t>;

  QuadrotorDynamicsDerivatives(const quadrotor_parameters_t& quadrotorParameters) : param_(quadrotorParameters) {}

  ~QuadrotorDynamicsDerivatives() override = default;

  QuadrotorDynamicsDerivatives* clone() const override { return new QuadrotorDynamicsDerivatives(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    // BASE class method
    BASE::setCurrentStateAndControl(t, x, u);

    // Jacobian of angular velocity mapping
    Eigen::Matrix<scalar_t, 3, 1> eulerAngle = x.segment<3>(3);
    Eigen::Matrix<scalar_t, 3, 1> angularVelocity = x.segment<3>(9);
    jacobianOfAngularVelocityMapping_ = JacobianOfAngularVelocityMapping(eulerAngle, angularVelocity).transpose();
  }

  void getFlowMapDerivativeState(state_matrix_t& A) override {
    // positions
    scalar_t qxQ = BASE::x_(0);  // x
    scalar_t qyQ = BASE::x_(1);  // y
    scalar_t qzQ = BASE::x_(2);  // z

    // euler angles xyz
    scalar_t qph = BASE::x_(3);
    scalar_t qth = BASE::x_(4);
    scalar_t qps = BASE::x_(5);

    // positions derivatives
    scalar_t dqxQ = BASE::x_(6);  // x
    scalar_t dqyQ = BASE::x_(7);  // y
    scalar_t dqzQ = BASE::x_(8);  // z

    // euler angle derivatives xyz
    scalar_t dqph = BASE::x_(9);
    scalar_t dqth = BASE::x_(10);
    scalar_t dqps = BASE::x_(11);

    // Applied force and momentums
    scalar_t Fz = BASE::u_(0);
    scalar_t Mx = BASE::u_(1);
    scalar_t My = BASE::u_(2);
    scalar_t Mz = BASE::u_(3);

    scalar_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t26, t21, t22, t27, t23, t24, t25;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = std::cos(qth);
    t4 = std::sin(qph);
    t5 = std::cos(qph);
    t6 = std::sin(qth);
    t7 = 1.0 / param_.Thxxyy_;
    t8 = std::cos(qps);
    t9 = std::sin(qps);
    t10 = 1.0 / t3;
    t11 = param_.Thxxyy_ * 2.0;
    t12 = param_.Thzz_ - t11;
    t13 = qth * 2.0;
    t14 = std::cos(t13);
    t15 = My * t9;
    t16 = std::sin(t13);
    t17 = 1.0 / (t3 * t3);
    t18 = qth * 3.0;
    t19 = std::sin(t18);
    t20 = My * t8;
    t21 = Mx * t9;
    t22 = t20 + t21;
    t23 = t3 * t3;
    t24 = t6 * t6;
    t25 = param_.Thzz_ * dqps * t6;

    A.setZero();
    A.block<3, 3>(0, 6).setIdentity();
    A.block<3, 3>(3, 3) = jacobianOfAngularVelocityMapping_.block<3, 3>(0, 0);
    A.block<3, 3>(3, 9) = jacobianOfAngularVelocityMapping_.block<3, 3>(0, 3);

    A(6, 4) = Fz * t2 * t3;
    A(7, 3) = -Fz * t2 * t3 * t5;
    A(7, 4) = Fz * t2 * t4 * t6;
    A(8, 3) = -Fz * t2 * t3 * t4;
    A(8, 4) = -Fz * t2 * t5 * t6;
    A(9, 4) = -t6 * t7 * t17 *
                  (t15 - Mx * t8 + param_.Thzz_ * dqps * dqth - param_.Thxxyy_ * dqph * dqth * t6 * 2.0 + param_.Thzz_ * dqph * dqth * t6) -
              dqph * dqth * t7 * t12;
    A(9, 5) = -t7 * t10 * t22;
    A(9, 9) = -dqth * t6 * t7 * t10 * t12;
    A(9, 10) = -t7 * t10 * (param_.Thzz_ * dqps - param_.Thxxyy_ * dqph * t6 * 2.0 + param_.Thzz_ * dqph * t6);
    A(9, 11) = -param_.Thzz_ * dqth * t7 * t10;
    A(10, 4) = -dqph * t7 * (t25 + param_.Thxxyy_ * dqph * t14 - param_.Thzz_ * dqph * t14);
    A(10, 5) = -t7 * (t15 - Mx * t8);
    A(10, 9) = t7 * (-param_.Thxxyy_ * dqph * t16 + param_.Thzz_ * dqps * t3 + param_.Thzz_ * dqph * t16);
    A(10, 11) = param_.Thzz_ * dqph * t3 * t7;
    A(11, 4) = t7 * t17 *
               (Mx * t8 * -4.0 + My * t9 * 4.0 + param_.Thzz_ * dqps * dqth * 4.0 - param_.Thxxyy_ * dqph * dqth * t6 * 9.0 -
                param_.Thxxyy_ * dqph * dqth * t19 + param_.Thzz_ * dqph * dqth * t6 * 5.0 + param_.Thzz_ * dqph * dqth * t19) *
               (1.0 / 4.0);
    A(11, 5) = t6 * t7 * t10 * t22;
    A(11, 9) = dqth * t7 * t10 * (param_.Thzz_ - t11 + param_.Thxxyy_ * t23 - param_.Thzz_ * t23);
    A(11, 10) = t7 * t10 * (t25 - param_.Thxxyy_ * dqph - param_.Thxxyy_ * dqph * t24 + param_.Thzz_ * dqph * t24);
    A(11, 11) = param_.Thzz_ * dqth * t6 * t7 * t10;
  }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override {
    scalar_t qph = BASE::x_(3);
    scalar_t qth = BASE::x_(4);
    scalar_t qps = BASE::x_(5);

    scalar_t t2, t3, t4, t5, t6, t7, t8;  // t9, t10, t11;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = cos(qth);
    t4 = 1.0 / param_.Thxxyy_;
    t5 = 1.0 / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    B.setZero();
    B(6, 0) = t2 * t8;
    B(7, 0) = -t2 * t3 * sin(qph);
    B(8, 0) = t2 * t3 * cos(qph);
    B(9, 1) = t4 * t5 * t7;
    B(9, 2) = -t4 * t5 * t6;
    B(10, 1) = t4 * t6;
    B(10, 2) = t4 * t7;
    B(11, 1) = -t4 * t5 * t7 * t8;
    B(11, 2) = t4 * t5 * t6 * t8;
    B(11, 3) = 1.0 / param_.Thzz_;
  }

 private:
  quadrotor_parameters_t param_;

  Eigen::Matrix<scalar_t, 3, 6> jacobianOfAngularVelocityMapping_;
};

}  // namespace quadrotor
}  // namespace ocs2
