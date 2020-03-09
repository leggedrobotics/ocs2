#pragma once

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

using switched_model::NUM_CONTACT_POINTS;

class AnymalWheelsComKinoConstraintAd : public ComKinoConstraintBaseAd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = switched_model::ComKinoConstraintBaseAd;
  using Base::foot_cpg_t;
  using Base::logic_rules_t;

  using Base::ad_base_t;
  using Base::ad_com_model_t;
  using Base::ad_kinematic_model_t;
  using Base::ad_scalar_t;

  using Base::ConstraintCollection_t;
  using Base::ConstraintTerm_t;
  using Base::LinearConstraintApproximationAsMatrices_t;
  using Base::QuadraticConstraintApproximation_t;

  /* Constraint Terms */
  using FrictionConeConstraint_t = switched_model::FrictionConeConstraint;
  using ZeroForceConstraint_t = switched_model::ZeroForceConstraint;
  using EndEffectorVelocityConstraint_t = switched_model::EndEffectorVelocityConstraint;
  using EndEffectorVelocityInFootFrameConstraint_t = switched_model::EndEffectorVelocityInFootFrameConstraint;

  /* Settings */
  using EndEffectorVelocityConstraintSettings_t = switched_model::EndEffectorVelocityConstraintSettings;
  using EndEffectorVelocityInFootFrameConstraintSettings_t = switched_model::EndEffectorVelocityInFootFrameConstraintSettings;

  AnymalWheelsComKinoConstraintAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                  std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings& options = ModelSettings());

  AnymalWheelsComKinoConstraintAd* clone() const override;

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms() override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;
};

}  // end of namespace switched_model
