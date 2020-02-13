#pragma once

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
// #include <ocs2_core/automatic_differentiation/CppAdInterface.h>
// #include <ocs2_core/constraint/ConstraintBase.h>
#include "ocs2_anymal_switched_model/core/AnymalKinematics.h"

namespace switched_model {

class AnymalComKinoConstraintAd final : public switched_model::ComKinoConstraintBaseAd  {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = switched_model::ComKinoConstraintBaseAd;
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
  using typename Base::ConstraintCollection_t;
  using typename Base::LinearConstraintApproximationAsMatrices_t;
  using typename Base::QuadraticConstraintApproximation_t;
  using typename Base::ConstraintTerm_t;
  using typename Base::ad_base_t;
  using typename Base::ad_scalar_t;
  using typename Base::ad_com_model_t;
  using typename Base::ad_kinematic_model_t;
  using typename Base::foot_cpg_t;
  using typename Base::logic_rules_t;
  using anymalKinematics_t = anymal::AnymalKinematics;
  // typedef std::shared_ptr<AnymalKinematics_t> anymalKinematicModelPtr_t ;

  // Enumeration and naming
  using Base::Feet;

  AnymalComKinoConstraintAd(const anymalKinematics_t& anymalKinematicModel, const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
      std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings& options = ModelSettings());

  AnymalComKinoConstraintAd(const AnymalComKinoConstraintAd& rhs) = default;
    // : Base(rhs), anymalKinematicModel_(rhs.anymalKinematicModel_) { }

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms() override;

  ~AnymalComKinoConstraintAd() override = default;

  AnymalComKinoConstraintAd* clone() const override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

 private:
  // anymalKinematicModelPtr_t anymalKinematicModelPtr_;
  const anymalKinematics_t& anymalKinematicModel_;
};

}  // end of namespace switched_model
