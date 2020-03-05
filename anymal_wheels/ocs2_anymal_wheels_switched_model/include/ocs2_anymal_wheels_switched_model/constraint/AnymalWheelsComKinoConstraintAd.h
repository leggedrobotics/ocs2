#pragma once

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace anymal {

  using switched_model::NUM_CONTACT_POINTS;

class AnymalWheelsComKinoConstraintAd : public switched_model::ComKinoConstraintBaseAd {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  using Base = switched_model::ComKinoConstraintBaseAd;
  using Base::logic_rules_t;
  using Base::foot_cpg_t;

  using Base::ad_base_t;
  using Base::ad_scalar_t;
  using Base::ad_com_model_t;
  using Base::ad_kinematic_model_t;

  using Base::ConstraintCollection_t;
  using Base::LinearConstraintApproximationAsMatrices_t;
  using Base::QuadraticConstraintApproximation_t;
  using Base::ConstraintTerm_t;
  using Base::ModelSettings_t;

  /* Constraint Terms */
  using FrictionConeConstraint_t = switched_model::constraints::FrictionConeConstraint;
  using ZeroForceConstraint_t = switched_model::constraints::ZeroForceConstraint;
  using EndEffectorVelocityConstraint_t = switched_model::constraints::EndEffectorVelocityConstraint;
  using EndEffectorVelocityInFootFrameConstraint_t = switched_model::constraints::EndEffectorVelocityInFootFrameConstraint;

  /* Settings */
  using EndEffectorVelocityConstraintSettings_t = switched_model::constraints::EndEffectorVelocityConstraintSettings;
  using EndEffectorVelocityInFootFrameConstraintSettings_t = switched_model::constraints::EndEffectorVelocityInFootFrameConstraintSettings;


  AnymalWheelsComKinoConstraintAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                          std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings_t& options = ModelSettings_t()
                          );

  AnymalWheelsComKinoConstraintAd* clone() const override;

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms() override;


  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

};

}  // end of namespace anymal
