#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

namespace switched_model {

/**
 * Implements the constraint that the velocity of the wheel contact point must be zero in y-direction for a foot in contact:
 *
 * y-axis(foot) * footVelocity = 0
 */
class EndEffectorVelocityInFootFrameConstraint final : public ocs2::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  EndEffectorVelocityInFootFrameConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager,
                                           const ad_kinematic_model_t& adKinematicsModel, bool generateModels);

  ~EndEffectorVelocityInFootFrameConstraint() override = default;

  EndEffectorVelocityInFootFrameConstraint* clone() const override;

  bool isActive(scalar_t time) const override;

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const ocs2::PreComputation& preComp) const override;

 private:
  EndEffectorVelocityInFootFrameConstraint(const EndEffectorVelocityInFootFrameConstraint& rhs);

  static void adfunc(const ad_kinematic_model_t& adKinematicsModel, int legNumber, const ocs2::ad_vector_t& tapedInput,
                     ocs2::ad_vector_t& f_footVelocityInFootFrame);

  int legNumber_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
  std::unique_ptr<ocs2::CppAdInterface> adInterface_;
};
}  // namespace switched_model
