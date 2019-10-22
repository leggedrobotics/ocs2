#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include "ocs2_switched_model_interface/constraint/ConstraintCollection.h"
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

class ComKinoConstraintBaseAd : public ocs2::ConstraintBase<24, 24> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t JOINT_COORD_SIZE = 12;
  static constexpr size_t STATE_DIM = 24;
  static constexpr size_t INPUT_DIM = 24;
  static constexpr size_t NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS;

  using logic_rules_t = SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double>;
  using foot_cpg_t = typename logic_rules_t::foot_cpg_t;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = ComModelBase<JOINT_COORD_SIZE, ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<JOINT_COORD_SIZE, ad_scalar_t>;

  using Base = ocs2::ConstraintBase<24, 24>;
  using typename Base::constraint1_input_matrix_t;
  using typename Base::constraint1_state_matrix_t;
  using typename Base::constraint1_vector_array_t;
  using typename Base::constraint1_vector_t;
  using typename Base::constraint2_state_matrix_t;
  using typename Base::constraint2_vector_array_t;
  using typename Base::constraint2_vector_t;
  using typename Base::input_matrix_array_t;
  using typename Base::input_matrix_t;
  using typename Base::input_state_matrix_array_t;
  using typename Base::input_state_matrix_t;
  using typename Base::input_vector_array_t;
  using typename Base::input_vector_t;
  using typename Base::scalar_array_t;
  using typename Base::scalar_t;
  using typename Base::state_input_matrix_t;
  using typename Base::state_matrix_array_t;
  using typename Base::state_matrix_t;
  using typename Base::state_vector_array_t;
  using typename Base::state_vector_t;

  using contact_flag_t = typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t;
  using base_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t;
  using joint_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t;

  using ConstraintCollection_t = ocs2::ConstraintCollection<STATE_DIM, INPUT_DIM>;
  using LinearConstraintApproximationAsMatrices_t = ocs2::LinearConstraintApproximationAsMatrices<STATE_DIM, INPUT_DIM>;
  using QuadraticConstraintApproximation_t = ocs2::QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using ConstraintTerm_t = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;

  // Enumeration and naming
  enum class FeetEnum { LF, RF, LH, RH };
  const std::array<std::string, 4> feetNames{"LF", "RF", "LH", "RH"};

  ComKinoConstraintBaseAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                          std::shared_ptr<const logic_rules_t> logicRulesPtr, const Model_Settings& options = Model_Settings())
      : Base(),
        adKinematicModelPtr_(adKinematicModel.clone()),
        adComModelPtr_(adComModel.clone()),
        logicRulesPtr_(std::move(logicRulesPtr)),
        options_(options),
        inequalityConstraintsComputed_(false),
        stateInputConstraintsComputed_(false) {
    if (!logicRulesPtr_) {
      throw std::runtime_error("[ComKinoConstraintBaseAD] logicRules cannot be a nullptr");
    }
    InitializeConstraintTerms();
  }

  ComKinoConstraintBaseAd(const ComKinoConstraintBaseAd& rhs)
      : Base(rhs),
        adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
        adComModelPtr_(rhs.adComModelPtr_->clone()),
        logicRulesPtr_(rhs.logicRulesPtr_),
        options_(rhs.options_),
        inequalityConstraintCollection_(rhs.inequalityConstraintCollection_),
        equalityStateInputConstraintCollection_(rhs.equalityStateInputConstraintCollection_),
        inequalityConstraintsComputed_(false),
        stateInputConstraintsComputed_(false) {}

  /**
   * Initialize Constraint Terms
   */
  void InitializeConstraintTerms();

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
  // State input equality constraints
  ConstraintCollection_t equalityStateInputConstraintCollection_;
  bool stateInputConstraintsComputed_;
  LinearConstraintApproximationAsMatrices_t linearStateInputConstraintApproximation_;

  // Inequality constraints
  ConstraintCollection_t inequalityConstraintCollection_;
  bool inequalityConstraintsComputed_;
  QuadraticConstraintApproximation_t quadraticInequalityConstraintApproximation_;

  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  Model_Settings options_;

  std::shared_ptr<const logic_rules_t> logicRulesPtr_;
  contact_flag_t stanceLegs_;
  size_t numEventTimes_;

  std::array<const foot_cpg_t*, NUM_CONTACT_POINTS_> zDirectionRefsPtr_;
};

}  // end of namespace switched_model

