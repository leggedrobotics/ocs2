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

  LeggedRobotConstraintAD(std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                          std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                          const PinocchioInterface& pinocchioInterface, CentroidalModelPinocchioMapping<ad_scalar_t>& pinocchioMappingAd,
                          const ModelSettings& options = ModelSettings());

  LeggedRobotConstraintAD(const LeggedRobotConstraintAD& rhs);

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms(const PinocchioInterface& pinocchioInterface,
                                 CentroidalModelPinocchioMapping<ad_scalar_t>& pinocchioMappingAd);

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

  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
  std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;

  ModelSettings options_;

  contact_flag_t stanceLegs_;
  size_t numEventTimes_;

  std::unique_ptr<ocs2::StateInputConstraintCollection> equalityStateInputConstraintCollectionPtr_;
  std::unique_ptr<ocs2::StateInputConstraintCollection> inequalityStateInputConstraintCollectionPtr_;
};

}  // namespace legged_robot
}  // end of namespace ocs2
