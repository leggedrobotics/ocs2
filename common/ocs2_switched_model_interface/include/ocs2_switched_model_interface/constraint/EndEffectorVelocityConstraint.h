#pragma once

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>

namespace switched_model {

using EndEffectorVelocityConstraintSettings = EndEffectorConstraintSettings;

class EndEffectorVelocityConstraint final : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using settings_t = EndEffectorVelocityConstraintSettings;

  EndEffectorVelocityConstraint(int legNumber, settings_t settings, ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel,
                                bool generateModels = true, const std::string& constraintPrefix = "o_EEVelocityConstraint_");

  ~EndEffectorVelocityConstraint() override = default;

  EndEffectorVelocityConstraint* clone() const override;

 private:
  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs) = default;

  static void adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber,
                     const ocs2::ad_vector_t& tapedInput, ocs2::ad_vector_t& o_footVelocity);
};
}  // namespace switched_model
