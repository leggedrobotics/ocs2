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
#include <ocs2_core/dynamics/DerivativesBase.h>

namespace ocs2 {

/**
 * A class for linearizing system dynamics. The linearized system dynamics is defined as: \n
 *
 * - Linearized system:   \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 */
class SystemDynamicsLinearizer : public DerivativesBase {
 public:
  /**
   * Constructor
   */
  SystemDynamicsLinearizer(std::shared_ptr<ControlledSystemBase> nonlinearSystemPtr, bool doubleSidedDerivative = true,
                           bool isSecondOrderSystem = false);

  /**
   * Copy constructor
   *
   * @param [in] other: Instance of the other class.
   */
  SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other);

  /**
   * Default destructor
   */
  ~SystemDynamicsLinearizer() override = default;

  /**
   * Copy assignment operator
   */
  SystemDynamicsLinearizer& operator=(const SystemDynamicsLinearizer& other);

  /**
   * The A matrix at a given operating point for the linearized system,
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] A: \f$ A(t) \f$ matrix.
   */
  void getFlowMapDerivativeState(matrix_t& A) override;

  /**
   * The B matrix at a given operating point for the linearized system,
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] B: \f$ B(t) \f$ matrix.
   */
  void getFlowMapDerivativeInput(matrix_t& B) override;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  SystemDynamicsLinearizer* clone() const override;

 private:
  std::shared_ptr<ControlledSystemBase> controlledSystemPtr_;
  bool doubleSidedDerivative_;
  bool isSecondOrderSystem_;
};

}  // namespace ocs2
