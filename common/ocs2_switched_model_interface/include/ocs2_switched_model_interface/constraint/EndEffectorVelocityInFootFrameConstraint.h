#pragma once

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>

namespace switched_model {

using EndEffectorVelocityInFootFrameConstraintSettings = EndEffectorVelocityConstraintSettings;

class EndEffectorVelocityInFootFrameConstraint final : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using settings_t = EndEffectorVelocityInFootFrameConstraintSettings;

  EndEffectorVelocityInFootFrameConstraint(int legNumber, settings_t settings, ad_com_model_t& adComModel,
                                           ad_kinematic_model_t& adKinematicsModel, bool generateModels = true,
                                           const std::string& constraintPrefix = "EEVelocityInFootFrameConstraint_");

  ~EndEffectorVelocityInFootFrameConstraint() override = default;

  EndEffectorVelocityInFootFrameConstraint* clone() const override;

 private:
  EndEffectorVelocityInFootFrameConstraint(const EndEffectorVelocityInFootFrameConstraint& rhs) = default;

  static void adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber,
                     const ocs2::ad_vector_t& tapedInput, ocs2::ad_vector_t& f_footVelocityInFootFrame);
};
}  // namespace switched_model
