/*
 * SwitchedModelStateEstimator.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/core/ComModelBase.h"

#include <memory>

namespace switched_model {

class SwitchedModelStateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using com_model_t = ComModelBase<double>;

  explicit SwitchedModelStateEstimator(const com_model_t& comModel);

  SwitchedModelStateEstimator(const SwitchedModelStateEstimator& rhs);

  /**
   * calculate comkino switched model state from the rbd model state.
   */
  comkino_state_t estimateComkinoModelState(const rbd_state_t& rbdState) const;

  /**
   * Calculates CoMDynamics state from the generalized coordinates and their velocities.
   */
  com_state_t estimateComState(const rbd_state_t& rbdState) const;

  /**
   * Calculates the RBD model state from the comkino switched model state and joint velocities.
   *
   * @param [in] comkinoState: comkino switched model state.
   * @param [in] dqJoints: joint velocities.
   * @param [out] rbdState: RBD model state
   */
  rbd_state_t estimateRbdModelState(const comkino_state_t& comkinoState, const joint_coordinate_t& dqJoints) const;

 private:
  std::unique_ptr<com_model_t> comModelPtr_;
};

}  // namespace switched_model
