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

#include <memory>
#include <string>
#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "ocs2_perceptive/distance_transform/DistanceTransformInterface.h"

namespace ocs2 {

/**
 * End-effector distance constraint Function.
 */
class EndEffectorDistanceConstraint final : public ocs2::StateConstraint {
 public:
  /** Constructor */
  EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight, std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr);

  /** Constructor */
  EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight, std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr,
                                scalar_array_t clearances);

  /** Default destructor */
  ~EndEffectorDistanceConstraint() override = default;

  void set(const DistanceTransformInterface& distanceTransform);
  void set(scalar_t clearance, const DistanceTransformInterface& distanceTransform);
  void set(const scalar_array_t& clearances, const DistanceTransformInterface& distanceTransform);

  EndEffectorDistanceConstraint* clone() const override { return new EndEffectorDistanceConstraint(*this); }
  size_t getNumConstraints(scalar_t time) const override { return kinematicsPtr_->getIds().size(); }
  const std::vector<std::string>& getIDs() const { return kinematicsPtr_->getIds(); }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

  size_t getStateDimensions() const { return stateDim_; }
  EndEffectorKinematics<scalar_t>& getKinematics() { return *kinematicsPtr_; }

 private:
  EndEffectorDistanceConstraint(const EndEffectorDistanceConstraint& other);

  const size_t stateDim_;
  const scalar_t weight_;

  std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr_;

  scalar_array_t clearances_;
  const DistanceTransformInterface* distanceTransformPtr_ = nullptr;
};

}  // namespace ocs2
