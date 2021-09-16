//
// Created by rgrandia on 23.10.19.
//

#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>

namespace switched_model {

SwitchedModelStateEstimator::SwitchedModelStateEstimator(const com_model_t& comModel) : comModelPtr_(comModel.clone()) {}

SwitchedModelStateEstimator::SwitchedModelStateEstimator(const SwitchedModelStateEstimator& rhs)
    : comModelPtr_(rhs.comModelPtr_->clone()) {}

comkino_state_t SwitchedModelStateEstimator::estimateComkinoModelState(const rbd_state_t& rbdState) const {
  comkino_state_t comkinoState;
  comkinoState << estimateComState(rbdState), getJointPositions(rbdState);
  return comkinoState;
}

com_state_t SwitchedModelStateEstimator::estimateComState(const rbd_state_t& rbdState) const {
  com_state_t comState;

  base_coordinate_t basePose = getBasePose(rbdState);
  base_coordinate_t baselocalVelocities = getBaseLocalVelocity(rbdState);

  comState << basePose, baselocalVelocities;
  return comState;
}

rbd_state_t SwitchedModelStateEstimator::estimateRbdModelState(const comkino_state_t& comkinoState,
                                                               const joint_coordinate_t& dqJoints) const {
  rbd_state_t rbdState;

  base_coordinate_t basePose = getBasePose(comkinoState);
  base_coordinate_t baseLocalVelocities = getBaseLocalVelocities(comkinoState);
  joint_coordinate_t qJoints = getJointPositions(comkinoState);

  rbdState << basePose, qJoints, baseLocalVelocities, dqJoints;
  return rbdState;
}

}  // namespace switched_model