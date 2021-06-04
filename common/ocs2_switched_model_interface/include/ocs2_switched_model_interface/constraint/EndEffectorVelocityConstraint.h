#pragma once

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>

namespace switched_model {

using EndEffectorVelocityConstraintSettings = EndEffectorConstraintSettings;

class EndEffectorVelocityConstraint : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = EndEffectorConstraint;
  using typename BASE::ad_com_model_t;
  using typename BASE::ad_interface_t;
  using typename BASE::ad_kinematic_model_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::ad_vector_t;
  using typename BASE::constraint_timeStateInput_matrix_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;
  using typename BASE::timeStateInput_matrix_t;
  using settings_t = EndEffectorVelocityConstraintSettings;

  explicit EndEffectorVelocityConstraint(int legNumber, settings_t settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels = true,
                                         std::string constraintPrefix = "o_EEVelocityConstraint_")
      : BASE(ConstraintOrder::Linear, std::move(constraintPrefix), legNumber, std::move(settings), adComModel, adKinematicsModel,
             EndEffectorVelocityConstraint::adfunc, generateModels) {}

  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs) = default;

  EndEffectorVelocityConstraint* clone() const override { return new EndEffectorVelocityConstraint(*this); };

 private:
  static void adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber, const ad_vector_t& tapedInput,
                     ad_vector_t& o_footVelocity) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
    comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

    // Extract elements from state
    const base_coordinate_ad_t basePose = getComPose(x);
    const base_coordinate_ad_t com_baseTwist = getComLocalVelocities(x);
    const joint_coordinate_ad_t qJoints = getJointPositions(x);
    const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

    o_footVelocity = adKinematicsModel.footVelocityInOriginFrame(legNumber, basePose, com_baseTwist, qJoints, dqJoints);
  };
};
}  // namespace switched_model
