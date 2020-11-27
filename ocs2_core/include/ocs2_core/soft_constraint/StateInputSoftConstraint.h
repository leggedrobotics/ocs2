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
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>

namespace ocs2 {

class StateInputSoftConstraint : public StateInputCost {
 public:
  /**
   * Constructor.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyFunctionPtrArray: An array of pointers to the penalty function on the constraint.
   * @param [in] constraintOrder: The order of constraint's approximation.
   */
  StateInputSoftConstraint(std::unique_ptr<StateInputConstraint> constraintPtr,
                           std::vector<std::unique_ptr<PenaltyFunctionBase>> penaltyFunctionPtrArray,
                           ConstraintOrder constraintOrder = ConstraintOrder::Quadratic);

  /**
   * Constructor.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyFunction: A pointer to the penalty function on the constraint.
   * @param [in] constraintOrder: The order of constraint's approximation.
   */
  StateInputSoftConstraint(std::unique_ptr<StateInputConstraint> constraintPtr, size_t numConstraints,
                           std::unique_ptr<PenaltyFunctionBase> penaltyFunction,
                           ConstraintOrder constraintOrder = ConstraintOrder::Quadratic);

  ~StateInputSoftConstraint() override = default;

  StateInputSoftConstraint* clone() const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const CostDesiredTrajectories& /* desiredTrajectory */) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const CostDesiredTrajectories& /* desiredTrajectory */) const override;

 private:
  StateInputSoftConstraint(const StateInputSoftConstraint& other);

  std::unique_ptr<StateInputConstraint> constraintPtr_;
  SoftConstraintPenalty penalty_;
  ConstraintOrder constraintOrder_;
};

}  // namespace ocs2
