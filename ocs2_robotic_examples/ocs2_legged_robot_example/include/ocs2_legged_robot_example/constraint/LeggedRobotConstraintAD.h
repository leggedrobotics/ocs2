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

#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/ConstraintCollection.h>
#include <ocs2_core/constraint/StateConstraintCollection.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>

#include <ocs2_legged_robot_example/common/ModelSettings.h>
#include <ocs2_legged_robot_example/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

namespace ocs2 {
namespace legged_robot {

class LeggedRobotConstraintAD : public ocs2::ConstraintBase {
 public:
  using Base = ocs2::ConstraintBase;

  LeggedRobotConstraintAD(const SwitchedModelModeScheduleManager& modeScheduleManager,
                          const SwingTrajectoryPlanner& swingTrajectoryPlannerPtr, const PinocchioInterface& pinocchioInterface,
                          const CentroidalModelPinocchioMappingCppAd& pinocchioMappingAd, const ModelSettings& options = ModelSettings());

  LeggedRobotConstraintAD(const LeggedRobotConstraintAD& rhs);

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms(const PinocchioInterface& pinocchioInterface,
                                 const CentroidalModelPinocchioMappingCppAd& pinocchioMappingAd);

  ~LeggedRobotConstraintAD() override = default;

  LeggedRobotConstraintAD* clone() const override;

  vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;

  vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;

  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                    const vector_t& u) override;

  VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                  const vector_t& u) override;

  vector_array_t stateInputEqualityConstraintDerivativesEventTimes(scalar_t t, const vector_t& x, const vector_t& u) override;

  /**
   * set the stance legs
   */
  void setStanceLegs(const contact_flag_t& stanceLegs);

  /**
   * get the model's stance leg
   */
  void getStanceLegs(contact_flag_t& stanceLegs);

 private:
  void timeUpdate(scalar_t t);

  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;

  ModelSettings options_;

  contact_flag_t stanceLegs_;
  size_t numEventTimes_;

  std::unique_ptr<ocs2::StateInputConstraintCollection> equalityStateInputConstraintCollectionPtr_;
  std::unique_ptr<ocs2::StateInputConstraintCollection> inequalityStateInputConstraintCollectionPtr_;
};

}  // namespace legged_robot
}  // end of namespace ocs2
