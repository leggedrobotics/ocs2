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

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_cart_pole_example/CartPoleParameters.h"
#include "ocs2_cart_pole_example/definitions.h"

namespace ocs2 {
namespace cartpole {

/**
 * CartPole dynamics.
 * refer to: https://pdfs.semanticscholar.org/f95b/9d4cc0814034f2e601cb91fcd70b2e806420.pdf
 */
class CartPoleSytemDynamics : public SystemDynamicsBaseAD<cartpole::STATE_DIM_, cartpole::INPUT_DIM_, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = SystemDynamicsBaseAD<cartpole::STATE_DIM_, cartpole::INPUT_DIM_, 1>;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using cart_pole_parameters_t = CartPoleParameters<scalar_t>;

  explicit CartPoleSytemDynamics(const cart_pole_parameters_t& cartPoleParameters) : param_(cartPoleParameters) {}

  ~CartPoleSytemDynamics() override = default;

  CartPoleSytemDynamics(const CartPoleSytemDynamics& rhs) = default;

  CartPoleSytemDynamics* clone() const override { return new CartPoleSytemDynamics(*this); }

  void systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                     ad_dynamic_vector_t& stateDerivative) const override {
    const ad_scalar_t cosTheta = cos(state(0));
    const ad_scalar_t sinTheta = sin(state(0));

    // Inertia tensor
    Eigen::Matrix<ad_scalar_t, 2, 2> I;
    I << static_cast<ad_scalar_t>(param_.poleSteinerMoi_), static_cast<ad_scalar_t>(param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
        static_cast<ad_scalar_t>(param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
        static_cast<ad_scalar_t>(param_.cartMass_ + param_.poleMass_);

    // RHS
    Eigen::Matrix<ad_scalar_t, 2, 1> rhs(param_.poleMass_ * param_.poleHalfLength_ * param_.gravity_ * sinTheta,
                                         input(0) + param_.poleMass_ * param_.poleHalfLength_ * pow(state(2), 2) * sinTheta);

    // dxdt
    stateDerivative(0) = state(2);
    stateDerivative(1) = state(3);
    stateDerivative.template tail<2>() = I.inverse() * rhs;
  }

 private:
  cart_pole_parameters_t param_;
};

}  // namespace cartpole
}  // namespace ocs2
