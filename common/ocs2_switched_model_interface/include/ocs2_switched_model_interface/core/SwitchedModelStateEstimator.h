/*
 * SwitchedModelStateEstimator.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#pragma once

#include <Eigen/Dense>

#include "ocs2_switched_model_interface/core/ComModelBase.h"

namespace switched_model {

class SwitchedModelStateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t JOINT_COORD_SIZE = 12;

  using scalar_t = double;
  using com_model_state_t = Eigen::Matrix<scalar_t, 12, 1>;
  using comkino_model_state_t = Eigen::Matrix<scalar_t, 12 + JOINT_COORD_SIZE, 1>;
  using full_model_state_t = Eigen::Matrix<scalar_t, 12 + 2 * JOINT_COORD_SIZE, 1>;
  using rbd_model_state_t = Eigen::Matrix<scalar_t, 12 + 2 * JOINT_COORD_SIZE, 1>;
  using joint_coordinate_t = Eigen::Matrix<scalar_t, JOINT_COORD_SIZE, 1>;
  using base_coordinate_t = Eigen::Matrix<scalar_t, 6, 1>;

  SwitchedModelStateEstimator(const ComModelBase<scalar_t>& comModel) : comModelPtr_(comModel.clone()) {}

  SwitchedModelStateEstimator(const SwitchedModelStateEstimator& rhs) : comModelPtr_(rhs.comModelPtr_->clone()) {}

  static joint_coordinate_t getJointPositions(const rbd_model_state_t& rbdState) {
    return rbdState.template segment<JOINT_COORD_SIZE>(6);
  }

  static base_coordinate_t getBasePose(const rbd_model_state_t& rbdState) {
    return rbdState.template segment<6>(0);
  }

  static base_coordinate_t getBaseLocalVelocity(const rbd_model_state_t& rbdState) {
    return rbdState.template segment<6>(18);
  }

  static joint_coordinate_t getJointPositions(const comkino_model_state_t& comkinoState) {
    return comkinoState.template segment<JOINT_COORD_SIZE>(12);
  }

  static base_coordinate_t getComPose(const comkino_model_state_t& comkinoState) {
    return comkinoState.template segment<6>(0);
  }

  static base_coordinate_t getComLocalVelocities(const comkino_model_state_t& comkinoState) {
    return comkinoState.template segment<6>(6);
  }

  /**
   * calculate comkino switched model state from the rbd model state.
   */
  comkino_model_state_t estimateComkinoModelState(const rbd_model_state_t& rbdState) const {
    comkino_model_state_t comkinoState;
    comkinoState <<  estimateComState(rbdState), getJointPositions(rbdState);
    return comkinoState;
  }

  /**
   * Calculates CoMDynamics state from the generalized coordinates and their velocities.
   */
  com_model_state_t estimateComState(const rbd_model_state_t& rbdState) const {
    com_model_state_t comState;

    base_coordinate_t basePose = getBasePose(rbdState);
    base_coordinate_t baselocalVelocities = getBaseLocalVelocity(rbdState);

    base_coordinate_t comPose = comModelPtr_->calculateComPose(basePose);
    base_coordinate_t comLocalVelocities = comModelPtr_->calculateComLocalVelocities(baselocalVelocities);

    comState << comPose, comLocalVelocities;
    return comState;
  }

  /**
   * Calculates the RBD model state from the comkino switched model state and joint velocities.
   *
   * @param [in] comkinoState: comkino switched model state.
   * @param [in] dqJoints: joint velocities.
   * @param [out] rbdState: RBD model state
   */
  rbd_model_state_t estimateRbdModelState(const comkino_model_state_t& comkinoState, const joint_coordinate_t& dqJoints) const {
    rbd_model_state_t rbdState;

    base_coordinate_t comPose = getComPose(comkinoState);
    base_coordinate_t comLocalVelocities = getComLocalVelocities(comkinoState);
    joint_coordinate_t qJoints = getJointPositions(comkinoState);

    base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);;
    base_coordinate_t baseLocalVelocities = comModelPtr_->calculateBaseLocalVelocities(comLocalVelocities);;

    rbdState << basePose, qJoints, baseLocalVelocities, dqJoints;
    return rbdState;
  }

 private:
  std::unique_ptr<ComModelBase<scalar_t>> comModelPtr_;
};

}  // namespace switched_model
