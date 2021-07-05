//
// Created by rgrandia on 29.04.20.
//

#pragma once

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/foot_planner/FootPhase.h>

namespace switched_model {

/**
 * Implements the swing motion equality constraint in the normal direction of the terrain.
 *
 * The constraint is a hybrid position-velocity constraint formulated in task space:
 * A_p * p + A_v * v + b = 0
 *
 * The derivative of the end-effector velocity and position w.r.t the joint space is generated with auto-differentation.
 */
class FootNormalConstraint : public ocs2::StateInputConstraintCppAd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;
  using settings_t = FootNormalConstraintMatrix;

  FootNormalConstraint(int legNumber, settings_t settings, const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                       bool generateModel);

  FootNormalConstraint* clone() const override;

  void configure(const settings_t& settings) { settings_ = settings; };

  vector_t getParameters(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }

 private:
  FootNormalConstraint(const FootNormalConstraint& rhs);

  ocs2::ad_vector_t constraintFunction(ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

  settings_t settings_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  std::unique_ptr<ad_kinematic_model_t> adKinematicsModelPtr_;
  int legNumber_;
};

}  // namespace switched_model
