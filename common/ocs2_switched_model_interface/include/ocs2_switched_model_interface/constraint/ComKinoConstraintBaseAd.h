#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/ConstraintBase.h>

#include "ocs2_switched_model_interface/constraint/ConstraintCollection.h"
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/ModelSettings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class ComKinoConstraintBaseAd : public ocs2::ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using ConstraintCollection_t = ocs2::ConstraintCollection<STATE_DIM, INPUT_DIM>;
  using LinearConstraintApproximationAsMatrices_t = ocs2::LinearConstraintApproximationAsMatrices<STATE_DIM, INPUT_DIM>;
  using QuadraticConstraintApproximation_t = ocs2::QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using ConstraintTerm_t = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;

  // Enumeration and naming
  enum class FeetEnum { LF, RF, LH, RH };
  const std::array<std::string, 4> feetNames{"LF", "RF", "LH", "RH"};

  ComKinoConstraintBaseAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                          std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                          std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr, ModelSettings options = ModelSettings());

  ComKinoConstraintBaseAd(const ComKinoConstraintBaseAd& rhs);

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms();

  ~ComKinoConstraintBaseAd() override = default;

  ComKinoConstraintBaseAd* clone() const override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

  size_t numStateInputConstraint(const scalar_t& time) override;
  void getConstraint1(constraint1_vector_t& e) override;

  size_t numStateOnlyConstraint(const scalar_t& time) override;
  void getConstraint2(constraint2_vector_t& h) override;
  void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

  size_t numStateOnlyFinalConstraint(const scalar_t& time) override;
  void getFinalConstraint2(constraint2_vector_t& h_f) override;
  void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_final) override;

  void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override;
  void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) override;
  void getConstraint1DerivativesEventTimes(constraint1_vector_array_t& g1DevArray) override;

  size_t numInequalityConstraint(const scalar_t& time) override;
  void getInequalityConstraint(scalar_array_t& h) override;
  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override;
  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override;
  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override;
  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override;
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override;

  /**
   * set the stance legs
   */
  void setStanceLegs(const contact_flag_t& stanceLegs);

  /**
   * get the model's stance leg
   */
  void getStanceLegs(contact_flag_t& stanceLegs);

 private:
  // state input equality constraints
  ConstraintCollection_t equalityStateInputConstraintCollection_;
  bool stateInputConstraintsComputed_;
  LinearConstraintApproximationAsMatrices_t linearStateInputConstraintApproximation_;

  // inequality constraints
  ConstraintCollection_t inequalityConstraintCollection_;
  bool inequalityConstraintsComputed_;
  QuadraticConstraintApproximation_t quadraticInequalityConstraintApproximation_;

  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  ModelSettings options_;

  size_t numEventTimes_;
  contact_flag_t stanceLegs_;

  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
  std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;
};

}  // end of namespace switched_model
