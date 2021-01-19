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

#include <ocs2_mobile_manipulator_example/definitions.h>
#include <ocs2_pinocchio_interface/EndEffectorKinematics.h>

#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>

namespace mobile_manipulator {

class EndEffectorCost final : public ocs2::CostFunctionBase {
 public:
  EndEffectorCost(const ocs2::EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ocs2::PenaltyFunctionBase& penalty);
  ~EndEffectorCost() override = default;

  EndEffectorCost* clone() const override { return new EndEffectorCost(*this); }

  scalar_t cost(scalar_t time, const vector_t& state, const vector_t& input) override;
  scalar_t finalCost(scalar_t time, const vector_t& state) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t time, const vector_t& state) override;

 private:
  EndEffectorCost(const EndEffectorCost& rhs);

  std::pair<vector_t, Eigen::Quaternion<scalar_t>> interpolateReference(scalar_t time) const;

  ocs2::SoftConstraintPenalty constraintPenalty_;
  std::unique_ptr<ocs2::EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  size_t endEffectorIndex_;
};

}  // namespace mobile_manipulator
