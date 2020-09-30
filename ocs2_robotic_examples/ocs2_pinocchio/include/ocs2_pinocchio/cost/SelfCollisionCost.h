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

#include <ocs2_pinocchio/PinocchioInterface.h>
#include <ocs2_pinocchio/PinocchioGeometryInterface.hpp>

#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/cost/CostFunctionBase.h>

namespace ocs2 {

class SelfCollisionCost final : public ocs2::CostFunctionBase {
 public:
  SelfCollisionCost(ocs2::PinocchioInterface<scalar_t> pinocchioInterface, ocs2::PinocchioGeometryInterface geometryInterfaceSelfCollision,
                    scalar_t minimumDistance, scalar_t mu, scalar_t delta);
  ~SelfCollisionCost() override = default;

  /* Copy constructor */
  SelfCollisionCost(const SelfCollisionCost& rhs);

  SelfCollisionCost* clone() const override { return new SelfCollisionCost(*this); }

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCost(scalar_t t, const vector_t& x) override;

  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;

 private:
  ocs2::PinocchioInterface<scalar_t> pinocchioInterface_;
  ocs2::PinocchioGeometryInterface pinocchioGeometrySelfCollisions_;

  scalar_t minimumDistance_;

  const ocs2::RelaxedBarrierPenalty relaxedBarrierPenalty_;
};

}  // namespace ocs2
