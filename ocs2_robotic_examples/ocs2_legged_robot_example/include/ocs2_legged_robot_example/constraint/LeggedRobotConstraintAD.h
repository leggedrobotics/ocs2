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

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "ocs2_legged_robot_example/gait/SwitchedModelModeScheduleManager.h"
#include "ocs2_legged_robot_example/reference/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_legged_robot_example/common/ModelSettings.h"
#include "ocs2_legged_robot_example/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_legged_robot_example/constraint/FrictionConeConstraint.h"
#include "ocs2_legged_robot_example/constraint/ZeroForceConstraint.h"

namespace ocs2 {
namespace legged_robot {

class LeggedRobotConstraintAD : public ConstraintBase {
 public:
  LeggedRobotConstraintAD(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                          const SwitchedModelModeScheduleManager& modeScheduleManager,
                          const SwingTrajectoryPlanner& swingTrajectoryPlannerPtr, ModelSettings modelSettings);

  ~LeggedRobotConstraintAD() override = default;
  LeggedRobotConstraintAD* clone() const override { return new LeggedRobotConstraintAD(*this); }

  vector_t stateInputEqualityConstraint(scalar_t time, const vector_t& state, const vector_t& input) override;
  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t time, const vector_t& state,
                                                                                    const vector_t& input) override;

  vector_t inequalityConstraint(scalar_t time, const vector_t& state, const vector_t& input) override;
  VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                  const vector_t& input) override;

 private:
  /** Copy constructor */
  LeggedRobotConstraintAD(const LeggedRobotConstraintAD& rhs);

  /** Initializes constraint terms */
  void initializeConstraintTerms(const PinocchioInterface& pinocchioInterface);

  /** Gets pointer from constraint collections */
  void collectConstraintPointers();

  /** Sets up the state-input equality constraints for a inquiry time */
  void updateStateInputEqualityConstraints(scalar_t time);

  /** Sets up the inequality constraints for a inquiry time */
  void updateInequalityConstraints(scalar_t time);

  const CentroidalModelInfo info_;
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  const ModelSettings modelSettings_;

  std::unique_ptr<ocs2::StateInputConstraintCollection> equalityStateInputConstraintCollectionPtr_;
  std::unique_ptr<ocs2::StateInputConstraintCollection> inequalityStateInputConstraintCollectionPtr_;

  // Individual constraint access (non-owning)
  std::vector<FrictionConeConstraint*> eeFrictionConeConstraints_;
  std::vector<ZeroForceConstraint*> eeZeroForceConstraints_;
  std::vector<EndEffectorLinearConstraint*> eeZeroVelocityConstraints_;
  std::vector<EndEffectorLinearConstraint*> eeNormalVelocityConstraints_;
};

}  // namespace legged_robot
}  // end of namespace ocs2
