/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <cmath>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include "ocs2_quadrotor/dynamics/QuadrotorSystemDynamics.h"

namespace ocs2 {
namespace quadrotor {

vector_t QuadrotorSystemDynamics::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) {
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
  t3 = cos(qth);
  t4 = sin(qth);
  t5 = 1.0 / param_.Thxxyy_;
  t6 = cos(qps);
  t7 = sin(qps);
  t8 = dqph * dqph;
  t9 = qth * 2.0;
  t10 = sin(t9);
  t11 = 1.0 / t3;
  t12 = param_.Thzz_ * param_.Thzz_;
  t13 = t3 * t3;

  vector_t stateDerivative(STATE_DIM);
  stateDerivative(0) = dqxQ;
  stateDerivative(1) = dqyQ;
  stateDerivative(2) = dqzQ;
  stateDerivative(3) = eulerAngleDerivatives(0);
  stateDerivative(4) = eulerAngleDerivatives(1);
  stateDerivative(5) = eulerAngleDerivatives(2);
  stateDerivative(6) = Fz * t2 * t4;
  stateDerivative(7) = -Fz * t2 * t3 * sin(qph);

  stateDerivative(8) = t2 * (param_.quadrotorMass_ * param_.gravity_ - Fz * t3 * cos(qph)) * (-1.0);
  stateDerivative(9) =
      -t5 * t11 *
      (-Mx * t6 + My * t7 + param_.Thzz_ * dqps * dqth - param_.Thxxyy_ * dqph * dqth * t4 * (2.0) + param_.Thzz_ * dqph * dqth * t4);
  stateDerivative(10) = t5 * (Mx * t7 + My * t6 - param_.Thxxyy_ * t8 * t10 * (1.0 / 2.0) + param_.Thzz_ * t8 * t10 * (1.0 / 2.0) +
                              param_.Thzz_ * dqph * dqps * t3);
  stateDerivative(11) = (t5 * t11 *
                         (Mz * param_.Thxxyy_ * t3 + dqph * dqth * t12 - dqph * dqth * t12 * t13 + dqps * dqth * t4 * t12 -
                          param_.Thxxyy_ * param_.Thzz_ * dqph * dqth * (2.0) - Mx * param_.Thzz_ * t4 * t6 + My * param_.Thzz_ * t4 * t7 +
                          param_.Thxxyy_ * param_.Thzz_ * dqph * dqth * t13)) /
                        param_.Thzz_;
  return stateDerivative;
}

VectorFunctionLinearApproximation QuadrotorSystemDynamics::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                               const PreComputation& preComp) {
  VectorFunctionLinearApproximation dynamics;
  dynamics.f = computeFlowMap(t, x, u, preComp);

  // Jacobian of angular velocity mapping
  Eigen::Matrix<scalar_t, 3, 1> eulerAngle = x.segment<3>(3);
  Eigen::Matrix<scalar_t, 3, 1> angularVelocity = x.segment<3>(9);
  jacobianOfAngularVelocityMapping_ = JacobianOfAngularVelocityMapping(eulerAngle, angularVelocity).transpose();

  // positions
  scalar_t qxQ = x(0);  // x
  scalar_t qyQ = x(1);  // y
  scalar_t qzQ = x(2);  // z

  // euler angles xyz
  scalar_t qph = x(3);
  scalar_t qth = x(4);
  scalar_t qps = x(5);

  // positions derivatives
  scalar_t dqxQ = x(6);  // x
  scalar_t dqyQ = x(7);  // y
  scalar_t dqzQ = x(8);  // z

  // euler angle derivatives xyz
  scalar_t dqph = x(9);
  scalar_t dqth = x(10);
  scalar_t dqps = x(11);

  // Applied force and momentums
  scalar_t Fz = u(0);
  scalar_t Mx = u(1);
  scalar_t My = u(2);
  scalar_t Mz = u(3);

  {  // derivative state
    scalar_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t26, t21, t22, t27, t23, t24, t25;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = cos(qth);
    t4 = sin(qph);
    t5 = cos(qph);
    t6 = sin(qth);
    t7 = 1.0 / param_.Thxxyy_;
    t8 = cos(qps);
    t9 = sin(qps);
    t10 = 1.0 / t3;
    t11 = param_.Thxxyy_ * 2.0;
    t12 = param_.Thzz_ - t11;
    t13 = qth * 2.0;
    t14 = cos(t13);
    t15 = My * t9;
    t16 = sin(t13);
    t17 = 1.0 / (t3 * t3);
    t18 = qth * 3.0;
    t19 = sin(t18);
    t20 = My * t8;
    t21 = Mx * t9;
    t22 = t20 + t21;
    t23 = t3 * t3;
    t24 = t6 * t6;
    t25 = param_.Thzz_ * dqps * t6;

    matrix_t& A = dynamics.dfdx;
    A.setZero(STATE_DIM, STATE_DIM);
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

  {  // derivative input
    scalar_t t2, t3, t4, t5, t6, t7, t8;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = cos(qth);
    t4 = 1.0 / param_.Thxxyy_;
    t5 = 1.0 / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    matrix_t& B = dynamics.dfdu;
    B.setZero(STATE_DIM, INPUT_DIM);
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
  return dynamics;
}

}  // namespace quadrotor
}  // namespace ocs2
