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

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "ocs2_cart_pole_example/CartPoleParameters.h"
#include "ocs2_cart_pole_example/definitions.h"

namespace ocs2 {
namespace cartpole {

class CartPoleCost final : public QuadraticCostFunction<cartpole::STATE_DIM_, cartpole::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartPoleCost>;
  using ConstPtr = std::shared_ptr<const CartPoleCost>;

  using BASE = QuadraticCostFunction<cartpole::STATE_DIM_, cartpole::INPUT_DIM_>;
  using scalar_t = typename BASE::scalar_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using input_vector_t = typename BASE::input_vector_t;
  using input_matrix_t = typename BASE::input_matrix_t;

  /**
   * Constructor for the running and final cost function defined as the following:
   * - \f$ L = 0.5(x-x_{nominal})' Q (x-x_{nominal}) + 0.5(u-u_{nominal})' R (u-u_{nominal}) \f$
   * - \f$ \Phi = 0.5(x-x_{final})' Q_{final} (x-x_{final}) \f$.
   * @param [in] Q: \f$ Q \f$
   * @param [in] R: \f$ R \f$
   * @param [in] xNominal: \f$ x_{nominal}\f$
   * @param [in] uNominal: \f$ u_{nominal}\f$
   * @param [in] xFinal: \f$ x_{final}\f$
   * @param [in] QFinal: \f$ Q_{final}\f$
   */
  CartPoleCost(const state_matrix_t& Q, const input_matrix_t& R, const state_vector_t& x_nominal, const input_vector_t& u_nominal,
               const state_matrix_t& Q_final, const state_vector_t& x_final)
      : QuadraticCostFunction(Q, R, x_nominal, u_nominal, Q_final, x_final) {}

  /**
   * Destructor
   */
  ~CartPoleCost() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  CartPoleCost* clone() const override { return new CartPoleCost(*this); }

 private:
};

}  // namespace cartpole
}  // namespace ocs2
