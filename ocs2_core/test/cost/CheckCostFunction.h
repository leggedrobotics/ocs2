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

inline void checkCostFunction(size_t numTests, ocs2::CostFunctionBase* cost1, ocs2::CostFunctionBase* cost2, bool& success,
                              size_t state_dim, size_t input_dim) {
  using ocs2::matrix_t;
  using ocs2::scalar_t;
  using ocs2::vector_t;

  success = true;

  const scalar_t precision = 1e-9;

  vector_t x;
  vector_t u;

  for (size_t it = 0; it < numTests; it++) {
    x.setRandom(state_dim);
    u.setRandom(input_dim);

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

    vector_t dLdx(state_dim), ad_dLdx(state_dim);
    cost1->getIntermediateCostDerivativeState(dLdx);
    cost2->getIntermediateCostDerivativeState(ad_dLdx);
    if (!dLdx.isApprox(ad_dLdx, precision)) {
      std::cout << "dLdx:    " << dLdx.transpose() << std::endl;
      std::cout << "al_dLdx: " << ad_dLdx.transpose() << std::endl;
      success = false;
    }

    matrix_t dLdxx(state_dim, state_dim), ad_dLdxx(state_dim, state_dim);
    cost1->getIntermediateCostSecondDerivativeState(dLdxx);
    cost2->getIntermediateCostSecondDerivativeState(ad_dLdxx);
    if (!dLdxx.isApprox(ad_dLdxx, precision)) {
      std::cout << "dLdxx:    \n" << dLdxx << std::endl;
      std::cout << "al_dLdxx: \n" << ad_dLdxx << std::endl;
      success = false;
    }

    vector_t dLdu(input_dim), ad_dLdu(input_dim);
    cost1->getIntermediateCostDerivativeInput(dLdu);
    cost2->getIntermediateCostDerivativeInput(ad_dLdu);
    if (!dLdu.isApprox(ad_dLdu, precision)) {
      std::cout << "dLdu:    " << dLdu.transpose() << std::endl;
      std::cout << "al_dLdu: " << ad_dLdu.transpose() << std::endl;
      success = false;
    }

    matrix_t dLduu(input_dim, input_dim), ad_dLduu(input_dim, input_dim);
    cost1->getIntermediateCostSecondDerivativeInput(dLduu);
    cost2->getIntermediateCostSecondDerivativeInput(ad_dLduu);
    if (!dLduu.isApprox(ad_dLduu, precision)) {
      std::cout << "dLduu:    \n" << dLduu << std::endl;
      std::cout << "al_dLduu: \n" << ad_dLduu << std::endl;
      success = false;
    }

    matrix_t dLdux(input_dim, state_dim), ad_dLdux(input_dim, state_dim);
    cost1->getIntermediateCostDerivativeInputState(dLdux);
    cost2->getIntermediateCostDerivativeInputState(ad_dLdux);
    if (!dLdux.isApprox(ad_dLdux, precision)) {
      std::cout << "dLdux:    \n" << dLdux << std::endl;
      std::cout << "al_dLdux: \n" << ad_dLdux << std::endl;
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

    vector_t dPhidx(state_dim), ad_dPhidx(state_dim);
    cost1->getTerminalCostDerivativeState(dPhidx);
    cost2->getTerminalCostDerivativeState(ad_dPhidx);
    if (!dPhidx.isApprox(ad_dPhidx, precision)) {
      std::cout << "dPhidx:    " << dPhidx.transpose() << std::endl;
      std::cout << "al_dPhidx: " << ad_dPhidx.transpose() << std::endl;
      success = false;
    }

    matrix_t dPhidxx(state_dim, state_dim), ad_dPhidxx(state_dim, state_dim);
    cost1->getTerminalCostSecondDerivativeState(dPhidxx);
    cost2->getTerminalCostSecondDerivativeState(ad_dPhidxx);
    if (!dPhidxx.isApprox(ad_dPhidxx, precision)) {
      std::cout << "dPhidxx:    \n" << dPhidxx << std::endl;
      std::cout << "al_dPhidxx: \n" << ad_dPhidxx << std::endl;
      success = false;
    }
  }  // end of for loop
}
