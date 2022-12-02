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

#include <ocs2_core/augmented_lagrangian/StateAugmentedLagrangian.h>
#include <ocs2_core/augmented_lagrangian/StateInputAugmentedLagrangian.h>

/**
 * Helper factory functions
 */
namespace ocs2 {

/**
 * Factory function for state augmented Lagrangian.
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
 */
std::unique_ptr<StateAugmentedLagrangian> create(std::unique_ptr<StateConstraint> constraintPtr,
                                                 std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray);

/**
 * Factory function for state augmented Lagrangian.
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
 */
std::unique_ptr<StateAugmentedLagrangian> create(std::unique_ptr<StateConstraint> constraintPtr,
                                                 std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr);

/**
 * Factory function for state-input augmented Lagrangian.
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
 */
std::unique_ptr<StateInputAugmentedLagrangian> create(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                      std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray);

/**
 * Factory function for state-input augmented Lagrangian.
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
 */
std::unique_ptr<StateInputAugmentedLagrangian> create(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                      std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr);

}  // namespace ocs2
