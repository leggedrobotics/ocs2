/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/CostCollection.h>
#include <ocs2_core/cost/CostFunctionBase.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_legged_robot_example/common/ModelSettings.h"
#include "ocs2_legged_robot_example/common/Types.h"
#include "ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h"

namespace ocs2 {
namespace legged_robot {

class LeggedRobotCost : public CostFunctionBase {
 public:
  LeggedRobotCost(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info,
                  const SwitchedModelModeScheduleManager& modeScheduleManager, const std::string& taskFile, ModelSettings modelSettings);
  ~LeggedRobotCost() override = default;
  LeggedRobotCost* clone() const override { return new LeggedRobotCost(*this); }

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCost(scalar_t t, const vector_t& x) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;

 private:
  LeggedRobotCost(const LeggedRobotCost& rhs);

  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info);
  void initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info, matrix_t& R);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelPinocchioMapping pinocchioMapping_;
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
  const ModelSettings modelSettings_;

  CostCollection<StateInputCost> stateInputCostCollection_;

  StateInputCost* baseTrackingCostPtr_ = nullptr;
};

}  // namespace legged_robot
}  // namespace ocs2
