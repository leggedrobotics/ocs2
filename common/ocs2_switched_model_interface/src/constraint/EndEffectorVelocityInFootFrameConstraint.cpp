//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h"

namespace switched_model {

EndEffectorVelocityInFootFrameConstraint::EndEffectorVelocityInFootFrameConstraint(
    int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager, const ad_kinematic_model_t& adKinematicsModel,
    bool generateModels)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear), legNumber_(legNumber), modeScheduleManager_(&modeScheduleManager) {
  const std::string modelName{"EEVelocityInFootFrameConstraint_" + std::to_string(legNumber)};
  const std::string modelFolder{"/tmp/ocs2"};
  const size_t domainDim = STATE_DIM + INPUT_DIM;

  auto diffFunc = [&](const ocs2::ad_vector_t& x, ocs2::ad_vector_t& y) { adfunc(adKinematicsModel, legNumber, x, y); };
  adInterface_.reset(new ocs2::CppAdInterface(diffFunc, domainDim, modelName, modelFolder));

  const auto orderCppAd = ocs2::CppAdInterface::ApproximationOrder::First;
  if (generateModels) {
    adInterface_->createModels(orderCppAd, true);
  } else {
    adInterface_->loadModelsIfAvailable(orderCppAd, true);
  }
}

EndEffectorVelocityInFootFrameConstraint::EndEffectorVelocityInFootFrameConstraint(const EndEffectorVelocityInFootFrameConstraint& rhs)
    : ocs2::StateInputConstraint(rhs),
      legNumber_(rhs.legNumber_),
      modeScheduleManager_(rhs.modeScheduleManager_),
      adInterface_(new ocs2::CppAdInterface(*rhs.adInterface_)) {}

EndEffectorVelocityInFootFrameConstraint* EndEffectorVelocityInFootFrameConstraint::clone() const {
  return new EndEffectorVelocityInFootFrameConstraint(*this);
};

bool EndEffectorVelocityInFootFrameConstraint::isActive(scalar_t time) const {
  return modeScheduleManager_->getContactFlags(time)[legNumber_];
}

size_t EndEffectorVelocityInFootFrameConstraint::getNumConstraints(scalar_t time) const {
  return 1;
}

vector_t EndEffectorVelocityInFootFrameConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                            const ocs2::PreComputation& preComp) const {
  // Assemble input
  vector_t tapedInput(STATE_DIM + INPUT_DIM);
  tapedInput << state, input;

  return adInterface_->getFunctionValue(tapedInput);
}

VectorFunctionLinearApproximation EndEffectorVelocityInFootFrameConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const ocs2::PreComputation& preComp) const {
  // Assemble input
  vector_t tapedInput(STATE_DIM + INPUT_DIM);
  tapedInput << state, input;

  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = adInterface_->getFunctionValue(tapedInput);

  const auto stateInputJacobian = adInterface_->getJacobian(tapedInput);

  linearApproximation.dfdx = stateInputJacobian.leftCols<STATE_DIM>();
  linearApproximation.dfdu = stateInputJacobian.rightCols<INPUT_DIM>();

  return linearApproximation;
}

void EndEffectorVelocityInFootFrameConstraint::adfunc(const ad_kinematic_model_t& adKinematicsModel, int legNumber,
                                                      const ocs2::ad_vector_t& tapedInput, ocs2::ad_vector_t& y) {
  // Extract elements from taped input
  comkino_state_ad_t x = tapedInput.head<STATE_DIM>();
  comkino_input_ad_t u = tapedInput.tail<INPUT_DIM>();

  // Extract elements from state
  const base_coordinate_ad_t baseTwist = getBaseLocalVelocities(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);
  const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

  // Get base state from com state;
  const auto f_footVelocityInFootFrame = adKinematicsModel.footVelocityInFootFrame(legNumber, baseTwist, qJoints, dqJoints);
  y.resize(1);
  y << f_footVelocityInFootFrame.y();
};

}  // namespace switched_model