#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/ModelSettings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

// Constraints
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/FootNormalConstraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

class ComKinoConstraintBaseAd : public ocs2::ConstraintBase {
 public:
  using ad_com_model_t = ComModelBase<ocs2::CppAdInterface::ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>;

  ComKinoConstraintBaseAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                          const SwitchedModelModeScheduleManager& modeScheduleManager, const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                          ModelSettings options = ModelSettings());

  ~ComKinoConstraintBaseAd() override = default;

  ComKinoConstraintBaseAd* clone() const override;

  vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;
  vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;

  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                    const vector_t& u) override;
  VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                  const vector_t& u) override;

 protected:
  /** protected copy constructor to implement clone */
  ComKinoConstraintBaseAd(const ComKinoConstraintBaseAd& rhs);

 private:
  /** Initialize Constraint Terms */
  void initializeConstraintTerms();

  /** Get pointer from constraint collection */
  void collectConstraintPointers();

  /** Sets up the state-input constraints for a query at time t */
  void updateStateInputEqualityConstraints(scalar_t t);

  /** Sets up the inequality constraints for a query at time t */
  void updateInequalityConstraints(scalar_t t);

  std::unique_ptr<ocs2::StateInputConstraintCollection> equalityStateInputConstraintCollectionPtr_;
  std::unique_ptr<ocs2::StateInputConstraintCollection> inequalityConstraintsCollectionPtr_;

  // Individual constraint access (non-owning)
  feet_array_t<FootNormalConstraint*> eeNormalConstraints_;
  feet_array_t<EndEffectorVelocityConstraint*> eeVelConstraints_;
  feet_array_t<ZeroForceConstraint*> zeroForceConstraints_;
  feet_array_t<FrictionConeConstraint*> frictionConeConstraints_;

  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  ModelSettings options_;

  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
};

}  // end of namespace switched_model
