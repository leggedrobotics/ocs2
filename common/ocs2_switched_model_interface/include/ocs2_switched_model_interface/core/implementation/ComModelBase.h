/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename ComModelBase<SCALAR_T>::base_coordinate_t ComModelBase<SCALAR_T>::calculateBasePose(
    const base_coordinate_t& comPose) const {
  base_coordinate_t basePose;

  // Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
  matrix3s_t o_R_b = RotationMatrixBasetoOrigin<SCALAR_T>(comPose.template head<3>());

  // base to CoM displacement in the CoM frame
  vector3s_t com_base2CoM = comPositionBaseFrame();

  // base coordinate
  basePose.template head<3>() = comPose.template segment<3>(0);
  basePose.template tail<3>() = comPose.template segment<3>(3) - o_R_b * com_base2CoM;
  return basePose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename ComModelBase<SCALAR_T>::base_coordinate_t ComModelBase<SCALAR_T>::calculateComPose(
    const base_coordinate_t& basePose) const {
  base_coordinate_t comPose;

  // Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
  matrix3s_t o_R_b = RotationMatrixBasetoOrigin<SCALAR_T>(basePose.template head<3>());

  // base to CoM displacement in the CoM frame
  vector3s_t com_base2CoM = comPositionBaseFrame();

  // base coordinate
  comPose.template head<3>() = basePose.template segment<3>(0);
  comPose.template tail<3>() = basePose.template segment<3>(3) + o_R_b * com_base2CoM;
  return comPose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename ComModelBase<SCALAR_T>::base_coordinate_t ComModelBase<SCALAR_T>::calculateBaseLocalVelocities(
    const base_coordinate_t& comLocalVelocities) const {
  base_coordinate_t baseLocalVelocities;

  // base to CoM displacement in the CoM frame
  vector3s_t com_base2CoM = comPositionBaseFrame();
  vector3s_t com_com2base = -com_base2CoM;

  // local velocities of Base (com_W_b)
  baseLocalVelocities.template head<3>() = comLocalVelocities.template head<3>();
  baseLocalVelocities.template tail<3>() =
      comLocalVelocities.template tail<3>() + comLocalVelocities.template head<3>().cross(com_com2base);

  return baseLocalVelocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename ComModelBase<SCALAR_T>::base_coordinate_t ComModelBase<SCALAR_T>::calculateComLocalVelocities(
    const base_coordinate_t& baseLocalVelocities) const {
  base_coordinate_t comLocalVelocities;

  // base to CoM displacement in the CoM frame
  vector3s_t com_base2CoM = comPositionBaseFrame();

  // local velocities of com (com_W_com)
  comLocalVelocities.template head<3>() = baseLocalVelocities.template head<3>();
  comLocalVelocities.template tail<3>() =
      baseLocalVelocities.template tail<3>() + baseLocalVelocities.template head<3>().cross(com_base2CoM);

  return comLocalVelocities;
}

}  // end of namespace switched_model
