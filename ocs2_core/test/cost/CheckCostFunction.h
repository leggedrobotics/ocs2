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

#include "ocs2_core/cost/QuadraticCostFunction.h"


template <size_t STATE_DIM, size_t INPUT_DIM>
inline void checkCostFunction(size_t numTests, ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>* cost1,
                       ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>* cost2, bool& success) {
  using quadratic_cost_t = ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename quadratic_cost_t::scalar_t;
  using state_vector_t = typename quadratic_cost_t::state_vector_t;
  using state_matrix_t = typename quadratic_cost_t::state_matrix_t;
  using input_vector_t = typename quadratic_cost_t::input_vector_t;
  using input_matrix_t = typename quadratic_cost_t::input_matrix_t;
  using input_state_matrix_t = typename quadratic_cost_t::input_state_matrix_t;

  success = true;

  const scalar_t precision = 1e-9;

  state_vector_t x;
  input_vector_t u;

  for (size_t it = 0; it < numTests; it++) {
    x.setRandom();
    u.setRandom();

    cost1->setCurrentStateAndControl(0.0, x, u);
    cost2->setCurrentStateAndControl(0.0, x, u);

    scalar_t L, ad_L;
    cost1->getIntermediateCost(L);
    cost2->getIntermediateCost(ad_L);
    if (std::abs(L - ad_L) > precision) {
      std::cout << "L:    " << L << std::endl;
      std::cout << "al_L: " << ad_L << std::endl;
      success = false;
    }

    state_vector_t dLdx, ad_dLdx;
    cost1->getIntermediateCostDerivativeState(dLdx);
    cost2->getIntermediateCostDerivativeState(ad_dLdx);
    if (!dLdx.isApprox(ad_dLdx, precision)) {
      std::cout << "dLdx:    " << dLdx.transpose() << std::endl;
      std::cout << "al_dLdx: " << ad_dLdx.transpose() << std::endl;
      success = false;
    }

    state_matrix_t dLdxx, ad_dLdxx;
    cost1->getIntermediateCostSecondDerivativeState(dLdxx);
    cost2->getIntermediateCostSecondDerivativeState(ad_dLdxx);
    if (!dLdxx.isApprox(ad_dLdxx, precision)) {
      std::cout << "dLdxx:    \n" << dLdxx << std::endl;
      std::cout << "al_dLdxx: \n" << ad_dLdxx << std::endl;
      success = false;
    }

    input_vector_t dLdu, ad_dLdu;
    cost1->getIntermediateCostDerivativeInput(dLdu);
    cost2->getIntermediateCostDerivativeInput(ad_dLdu);
    if (!dLdu.isApprox(ad_dLdu, precision)) {
      std::cout << "dLdu:    " << dLdu.transpose() << std::endl;
      std::cout << "al_dLdu: " << ad_dLdu.transpose() << std::endl;
      success = false;
    }

    input_matrix_t dLduu, ad_dLduu;
    cost1->getIntermediateCostSecondDerivativeInput(dLduu);
    cost2->getIntermediateCostSecondDerivativeInput(ad_dLduu);
    if (!dLduu.isApprox(ad_dLduu, precision)) {
      std::cout << "dLduu:    \n" << dLduu << std::endl;
      std::cout << "al_dLduu: \n" << ad_dLduu << std::endl;
      success = false;
    }

    input_state_matrix_t dLdxu, ad_dLdxu;
    cost1->getIntermediateCostDerivativeInputState(dLdxu);
    cost2->getIntermediateCostDerivativeInputState(ad_dLdxu);
    if (!dLdxu.isApprox(ad_dLdxu, precision)) {
      std::cout << "dLdxu:    \n" << dLdxu << std::endl;
      std::cout << "al_dLdxu: \n" << ad_dLdxu << std::endl;
      success = false;
    }

    scalar_t Phi, ad_Phi;
    cost1->getTerminalCost(Phi);
    cost2->getTerminalCost(ad_Phi);
    if (std::abs(Phi - ad_Phi) > precision) {
      std::cout << "Phi:    " << Phi << std::endl;
      std::cout << "al_Phi: " << ad_Phi << std::endl;
      success = false;
    }

    state_vector_t dPhidx, ad_dPhidx;
    cost1->getTerminalCostDerivativeState(dPhidx);
    cost2->getTerminalCostDerivativeState(ad_dPhidx);
    if (!dPhidx.isApprox(ad_dPhidx, precision)) {
      std::cout << "dPhidx:    " << dPhidx.transpose() << std::endl;
      std::cout << "al_dPhidx: " << ad_dPhidx.transpose() << std::endl;
      success = false;
    }

    state_matrix_t dPhidxx, ad_dPhidxx;
    cost1->getTerminalCostSecondDerivativeState(dPhidxx);
    cost2->getTerminalCostSecondDerivativeState(ad_dPhidxx);
    if (!dPhidxx.isApprox(ad_dPhidxx, precision)) {
      std::cout << "dPhidxx:    \n" << dPhidxx << std::endl;
      std::cout << "al_dPhidxx: \n" << ad_dPhidxx << std::endl;
      success = false;
    }
  }  // end of for loop
}
