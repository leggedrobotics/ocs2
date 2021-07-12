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

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {

/**
 * A class for linearizing system dynamics. The linearized system dynamics is defined as: \n
 *
 * - Linearized system:   \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 */
class SystemDynamicsLinearizer final : public SystemDynamicsBase {
 public:
  /** Constructor */
  explicit SystemDynamicsLinearizer(std::unique_ptr<ControlledSystemBase> nonlinearSystemPtr, bool doubleSidedDerivative = true,
                                    bool isSecondOrderSystem = false, scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon());

  /** Default destructor */
  ~SystemDynamicsLinearizer() override = default;

  /** Clone */
  SystemDynamicsLinearizer* clone() const override;

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override;

 private:
  /** Copy constructor with pre-computation */
  SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other);

  std::unique_ptr<ControlledSystemBase> controlledSystemPtr_;
  bool doubleSidedDerivative_;
  bool isSecondOrderSystem_;
  scalar_t eps_;
};

}  // namespace ocs2
