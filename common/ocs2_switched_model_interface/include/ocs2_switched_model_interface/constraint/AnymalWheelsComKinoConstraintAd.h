#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/ConstraintBase.h>

#include "ocs2_switched_model_interface/constraint/ConstraintCollection.h"
#include "ocs2_switched_model_interface/core/ModelSettings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class AnymalWheelsComKinoConstraintAd : public ocs2::ConstraintBase {
 public:
  using Base = ocs2::ConstraintBase;

  using ad_base_t = CppAD::cg::CG<scalar_t>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using ConstraintCollection_t = ocs2::ConstraintCollection<STATE_DIM, INPUT_DIM>;
  using LinearConstraintApproximationAsMatrices_t = ocs2::LinearConstraintApproximationAsMatrices<STATE_DIM, INPUT_DIM>;
  using QuadraticConstraintApproximation_t = ocs2::QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using ConstraintTerm_t = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;

  // Enumeration and naming
  enum class FeetEnum { LF, RF, LH, RH };

  AnymalWheelsComKinoConstraintAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                  std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                                  ModelSettings options = ModelSettings());

  AnymalWheelsComKinoConstraintAd(const AnymalWheelsComKinoConstraintAd& rhs);

  AnymalWheelsComKinoConstraintAd* clone() const override;

  /** General Anymal switched_model  */
  ~AnymalWheelsComKinoConstraintAd() override = default;

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms();

  vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;
  vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;

  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                    const vector_t& u) override;
  VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                  const vector_t& u) override;

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

 private:
  // state input equality constraints
  ConstraintCollection_t equalityStateInputConstraintCollection_;

  // inequality constraints
  ConstraintCollection_t inequalityConstraintCollection_;

  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  ModelSettings options_;

  size_t numEventTimes_;
  contact_flag_t stanceLegs_;

  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
  std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;
};

}  // end of namespace switched_model
