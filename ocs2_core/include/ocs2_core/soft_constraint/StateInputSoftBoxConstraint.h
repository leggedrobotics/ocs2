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
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/penalties/PenaltyBase.h>

namespace ocs2 {

/**
 *   Implements the cost penalty for state-input box constraints
 */
class StateInputSoftBoxConstraint final : public StateInputCost {
 public:
  struct BoxConstraint {
    //! Index of the constraint in the state or input vector
    size_t index = 0;

    //! Lower bound of the box constraint (default is low, but not numeric::lowest to prevent underflow)
    scalar_t lowerBound = -1e30;

    //! Upper bound of the box constraint (default is high, but not numeric::max to prevent overflow)
    scalar_t upperBound = 1e30;

    //! Penalty function
    std::unique_ptr<PenaltyBase> penaltyPtr;

    /* Constructors and assignment operators */
    BoxConstraint() = default;
    ~BoxConstraint() = default;
    BoxConstraint(const BoxConstraint& other);
    BoxConstraint& operator=(const BoxConstraint& other);
    BoxConstraint(BoxConstraint&& other) noexcept = default;
    BoxConstraint& operator=(BoxConstraint&& other) noexcept = default;
  };

  /**
   * Constructor.
   * @param stateBoxConstraints : box constraint specification for states
   * @param inputBoxConstraints : box constraint specification for inputs
   */
  StateInputSoftBoxConstraint(std::vector<BoxConstraint> stateBoxConstraints, std::vector<BoxConstraint> inputBoxConstraints);

  ~StateInputSoftBoxConstraint() override = default;

  StateInputSoftBoxConstraint* clone() const override;

  bool isActive(scalar_t time) const override;

  /**
   * Takes the cost value at the given time, state, input and adds it as constant offset to further evaluations.
   *
   * Taking a penalty on constraint bounds that are far away can create a large (negative) value inside the feasible set, for example when
   * using a relaxed barrier constraint. Adding a constant offset does not change the optimal solution, but gives peace of mind that the
   * cost values are in a reasonable absolute range.
   */
  void initializeOffset(scalar_t time, const vector_t& state, const vector_t& input);

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& /* targetTrajectories */,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& /* targetTrajectories */,
                                                                 const PreComputation& preComp) const override;

 private:
  StateInputSoftBoxConstraint(const StateInputSoftBoxConstraint& other) = default;

  void sortByIndex(std::vector<BoxConstraint>& boxConstraints) const;

  scalar_t getValue(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints) const;

  void fillQuadraticApproximation(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints, scalar_t& value,
                                  vector_t& firstDerivative, matrix_t& secondDerivative) const;

  std::vector<BoxConstraint> stateBoxConstraints_;
  std::vector<BoxConstraint> inputBoxConstraints_;
  scalar_t offset_;
};

}  // namespace ocs2
