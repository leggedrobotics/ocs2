//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"

namespace switched_model {

EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(int legNumber, settings_t settings, ad_com_model_t& adComModel,
                                                             ad_kinematic_model_t& adKinematicsModel, bool generateModels,
                                                             const std::string& constraintPrefix)
    : EndEffectorConstraint(constraintPrefix, legNumber, std::move(settings), adComModel, adKinematicsModel,
                            EndEffectorVelocityConstraint::adfunc, generateModels) {}

EndEffectorVelocityConstraint* EndEffectorVelocityConstraint::clone() const {
  return new EndEffectorVelocityConstraint(*this);
};

void EndEffectorVelocityConstraint::adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber,
                                           const ocs2::ad_vector_t& tapedInput, ocs2::ad_vector_t& o_footVelocity) {
  // Extract elements from taped input
  comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
  comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

  // Extract elements from state
  const base_coordinate_ad_t basePose = getComPose(x);
  const base_coordinate_ad_t com_baseTwist = getComLocalVelocities(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);
  const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

  // Get base state from com state
  o_footVelocity = adKinematicsModel.footVelocityInOriginFrame(legNumber, basePose, com_baseTwist, qJoints, dqJoints);
};

}  // namespace switched_model