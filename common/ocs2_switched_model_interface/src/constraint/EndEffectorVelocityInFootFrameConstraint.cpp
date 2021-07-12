//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h"

namespace switched_model {

EndEffectorVelocityInFootFrameConstraint::EndEffectorVelocityInFootFrameConstraint(int legNumber, settings_t settings,
                                                                                   ad_com_model_t& adComModel,
                                                                                   ad_kinematic_model_t& adKinematicsModel,
                                                                                   bool generateModels, const std::string& constraintPrefix)
    : EndEffectorConstraint(constraintPrefix, legNumber, std::move(settings), adComModel, adKinematicsModel,
                            EndEffectorVelocityInFootFrameConstraint::adfunc, generateModels) {}

EndEffectorVelocityInFootFrameConstraint* EndEffectorVelocityInFootFrameConstraint::clone() const {
  return new EndEffectorVelocityInFootFrameConstraint(*this);
};

void EndEffectorVelocityInFootFrameConstraint::adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber,
                                                      const ocs2::ad_vector_t& tapedInput, ocs2::ad_vector_t& f_footVelocityInFootFrame) {
  // Extract elements from taped input
  comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
  comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

  // Extract elements from state
  const base_coordinate_ad_t comPose = getComPose(x);
  const base_coordinate_ad_t com_comTwist = getComLocalVelocities(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);
  const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

  // Get base state from com state
  const base_coordinate_ad_t com_baseTwist = adComModel.calculateBaseLocalVelocities(com_comTwist);
  f_footVelocityInFootFrame = adKinematicsModel.footVelocityInFootFrame(legNumber, com_baseTwist, qJoints, dqJoints);
};

}  // namespace switched_model