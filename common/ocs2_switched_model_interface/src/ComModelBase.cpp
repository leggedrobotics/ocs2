/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/core/ComModelBase.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
matrix3_s_t<SCALAR_T> ComModelBase<SCALAR_T>::rotationalInertia() const {
  matrix6_s_t<SCALAR_T> inertia = comInertia();
  return inertia.template topLeftCorner<3, 3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
matrix6_s_t<SCALAR_T> ComModelBase<SCALAR_T>::comInertiaInverse() const {
  matrix6_s_t<SCALAR_T> MInverse;
  MInverse << rotationalInertia().inverse(), matrix3_s_t<SCALAR_T>::Zero(), matrix3_s_t<SCALAR_T>::Zero(),
      matrix3_s_t<SCALAR_T>::Identity() / totalMass();
  return MInverse;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> ComModelBase<SCALAR_T>::calculateBasePose(const base_coordinate_s_t<SCALAR_T>& comPose) const {
  // base to CoM displacement in the Origin frame
  vector3_s_t<SCALAR_T> o_base2CoM = rotateVectorBaseToOrigin<SCALAR_T>(comPositionBaseFrame(), getOrientation(comPose));

  base_coordinate_s_t<SCALAR_T> basePose;
  basePose.template head<3>() = getOrientation(comPose);
  basePose.template tail<3>() = getPositionInOrigin(comPose) - o_base2CoM;

  return basePose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> ComModelBase<SCALAR_T>::calculateComPose(const base_coordinate_s_t<SCALAR_T>& basePose) const {
  // base to CoM displacement in the Origin frame
  vector3_s_t<SCALAR_T> o_base2CoM = rotateVectorBaseToOrigin<SCALAR_T>(comPositionBaseFrame(), getOrientation(basePose));

  base_coordinate_s_t<SCALAR_T> comPose;
  comPose.template head<3>() = getOrientation(basePose);
  comPose.template tail<3>() = getPositionInOrigin(basePose) + o_base2CoM;

  return comPose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> ComModelBase<SCALAR_T>::calculateBaseLocalVelocities(
    const base_coordinate_s_t<SCALAR_T>& comLocalVelocities) const {
  // base to CoM displacement in the CoM frame
  vector3_s_t<SCALAR_T> com_base2CoM = comPositionBaseFrame();
  vector3_s_t<SCALAR_T> com_com2base = -com_base2CoM;

  // local velocities of Base (com_W_b)
  const vector3_s_t<SCALAR_T> angularVelocity = getAngularVelocity(comLocalVelocities);

  base_coordinate_s_t<SCALAR_T> baseLocalVelocities;
  baseLocalVelocities.template head<3>() = angularVelocity;
  baseLocalVelocities.template tail<3>() = getLinearVelocity(comLocalVelocities) + angularVelocity.cross(com_com2base);

  return baseLocalVelocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> ComModelBase<SCALAR_T>::calculateComLocalVelocities(
    const base_coordinate_s_t<SCALAR_T>& baseLocalVelocities) const {
  // base to CoM displacement in the CoM frame
  vector3_s_t<SCALAR_T> com_base2CoM = comPositionBaseFrame();

  // local velocities of com (com_W_com)
  const vector3_s_t<SCALAR_T> angularVelocity = getAngularVelocity(baseLocalVelocities);

  base_coordinate_s_t<SCALAR_T> comLocalVelocities;
  comLocalVelocities.template head<3>() = angularVelocity;
  comLocalVelocities.template tail<3>() = getLinearVelocity(baseLocalVelocities) + angularVelocity.cross(com_base2CoM);

  return comLocalVelocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> ComModelBase<SCALAR_T>::calculateBaseLocalAccelerations(
    const base_coordinate_s_t<SCALAR_T>& comLocalAccelerations, const base_coordinate_s_t<SCALAR_T>& comLocalVelocities) const {
  // get CoM to base displacement in the CoM frame
  vector3_s_t<SCALAR_T> com_base2CoM = comPositionBaseFrame();
  vector3_s_t<SCALAR_T> com_com2base = -com_base2CoM;

  // local velocities and accelerations of CoM (com_W_com)
  const vector3_s_t<SCALAR_T> angularVelocity = getAngularVelocity(comLocalVelocities);
  const vector3_s_t<SCALAR_T> angularAcceleration = getAngularAcceleration(comLocalAccelerations);

  base_coordinate_s_t<SCALAR_T> baseLocalAccelerations;
  baseLocalAccelerations.template head<3>() = angularAcceleration;
  baseLocalAccelerations.template tail<3>() = getLinearAcceleration(comLocalAccelerations) + angularAcceleration.cross(com_com2base) +
                                              angularVelocity.cross(angularVelocity.cross(com_com2base));

  return baseLocalAccelerations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> ComModelBase<SCALAR_T>::calculateComLocalAccelerations(
    const base_coordinate_s_t<SCALAR_T>& baseLocalAccelerations, const base_coordinate_s_t<SCALAR_T>& baseLocalVelocities) const {
  // get base to CoM displacement in the CoM frame
  vector3_s_t<SCALAR_T> com_base2CoM = comPositionBaseFrame();

  // local velocities and accelerations of Base (com_W_b)
  const vector3_s_t<SCALAR_T> angularVelocity = getAngularVelocity(baseLocalVelocities);
  const vector3_s_t<SCALAR_T> angularAcceleration = getAngularAcceleration(baseLocalAccelerations);

  base_coordinate_s_t<SCALAR_T> comLocalAccelerations;
  comLocalAccelerations.template head<3>() = angularAcceleration;
  comLocalAccelerations.template tail<3>() = getLinearAcceleration(baseLocalAccelerations) + angularAcceleration.cross(com_base2CoM) +
                                             angularVelocity.cross(angularVelocity.cross(com_base2CoM));

  return comLocalAccelerations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
comkino_input_t weightCompensatingInputs(const ComModelBase<scalar_t>& comModel, const contact_flag_t& contactFlags,
                                         const vector3_t& baseOrientation) {
  const int numStanceLegs = numberOfClosedContacts(contactFlags);

  comkino_input_t inputs = comkino_input_t::Zero();
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = comModel.totalMass() * 9.81;
    const vector3_t forceInWorld = vector3_t{0.0, 0.0, totalWeight / numStanceLegs};
    const vector3_t forceInBase = rotateVectorOriginToBase(forceInWorld, baseOrientation);

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }

  return inputs;
}

template class ComModelBase<scalar_t>;
template class ComModelBase<ocs2::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
