/*
 * SwitchedModelStateEstimator.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#pragma once

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

#include <memory>

namespace switched_model {

/**
 * calculate comkino switched model state from the rbd model state.
 */
comkino_state_t estimateComkinoModelState(const rbd_state_t& rbdState);

/**
 * Calculates the RBD model state from the comkino switched model state and joint velocities.
 *
 * @param [in] comkinoState: comkino switched model state.
 * @param [in] dqJoints: joint velocities.
 * @param [out] rbdState: RBD model state
 */
rbd_state_t estimateRbdModelState(const comkino_state_t& comkinoState, const joint_coordinate_t& dqJoints);

}  // namespace switched_model
