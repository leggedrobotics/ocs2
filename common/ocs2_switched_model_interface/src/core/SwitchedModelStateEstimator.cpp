//
// Created by rgrandia on 23.10.19.
//

#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>

namespace switched_model {

comkino_state_t estimateComkinoModelState(const rbd_state_t& rbdState) {
  comkino_state_t comkinoState;
  comkinoState << getBasePose(rbdState), getBaseLocalVelocity(rbdState), getJointPositions(rbdState);
  return comkinoState;
}

rbd_state_t estimateRbdModelState(const comkino_state_t& comkinoState, const joint_coordinate_t& dqJoints) {
  rbd_state_t rbdState;
  rbdState << getBasePose(comkinoState), getJointPositions(comkinoState), getBaseLocalVelocities(comkinoState), dqJoints;
  return rbdState;
}

}  // namespace switched_model