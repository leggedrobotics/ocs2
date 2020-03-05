#pragma once

#include "ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h"
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

class ComKinoConstraintImplAd : public ComKinoConstraintBaseAd {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  using Base                                      = ComKinoConstraintBaseAd;
  using logic_rules_t                             = Base::logic_rules_t;
  using foot_cpg_t                                = Base::foot_cpg_t;

  using ad_base_t                                 = Base::ad_base_t;
  using ad_scalar_t                              = Base::ad_scalar_t;
  using ad_com_model_t                            = Base::ad_com_model_t;
  using ad_kinematic_model_t                      = Base::ad_kinematic_model_t;

  using ConstraintCollection_t                    = Base::ConstraintCollection_t;
  using LinearConstraintApproximationAsMatrices_t = Base::LinearConstraintApproximationAsMatrices_t;
  using QuadraticConstraintApproximation_t        = Base::QuadraticConstraintApproximation_t;
  using ConstraintTerm_t                          = Base::ConstraintTerm_t;

  /* Constraint Terms */
  using FrictionConeConstraint_t = constraints::FrictionConeConstraint;
  using ZeroForceConstraint_t = constraints::ZeroForceConstraint;
  using EndEffectorVelocityConstraint_t = constraints::EndEffectorVelocityConstraint;

  /* Constraint Settings */
  using EndEffectorVelocityConstraintSettings_t = constraints::EndEffectorVelocityConstraintSettings;


  ComKinoConstraintImplAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                          std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings& options = ModelSettings()
                          );

  ComKinoConstraintImplAd* clone() const override;
  ComKinoConstraintImplAd(const ComKinoConstraintImplAd& rhs) = default;

  /**
   * Initialize Constraint Terms
   */
  void initializeConstraintTerms() override;


  /**
   * Set the Constraint Terms from the state and controls
   */
  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

};

}  // end of namespace switched_model

