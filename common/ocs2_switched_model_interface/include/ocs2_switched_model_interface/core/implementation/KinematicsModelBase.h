/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 3, 2017
 *      Author: Farbod
 */

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void KinematicsModelBase<SCALAR_T>::update(const generalized_coordinate_t& generalizedCoordinate) {
  update(generalizedCoordinate.template head<6>(), generalizedCoordinate.template tail<JOINT_COORDINATE_SIZE>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void KinematicsModelBase<SCALAR_T>::update(const base_coordinate_t& qBase, const joint_coordinate_t& qJoint) {
  qBase_ = qBase;
  qJoint_ = qJoint;
  b_R_o_ = RotationMatrixOrigintoBase<SCALAR_T>(qBase_.template head<3>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
std::array<typename KinematicsModelBase<SCALAR_T>::vector3d_t, NUM_CONTACT_POINTS> KinematicsModelBase<SCALAR_T>::feetPositionsBaseFrame()
    const {
  std::array<vector3d_t, NUM_CONTACT_POINTS> feetPositions;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    feetPositions[i] = footPositionBaseFrame(i);
  }
  return feetPositions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename KinematicsModelBase<SCALAR_T>::vector3d_t KinematicsModelBase<SCALAR_T>::footPositionOriginFrame(size_t footIndex) const {
  // calculate foot position in Base frame
  vector3d_t o_footPosition;
  vector3d_t b_footPosition = footPositionBaseFrame(footIndex);
  // calculate foot position in Origin frame
  o_footPosition = b_R_o_.transpose() * b_footPosition + qBase_.template tail<3>();
  return o_footPosition;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
std::array<typename KinematicsModelBase<SCALAR_T>::vector3d_t, NUM_CONTACT_POINTS> KinematicsModelBase<SCALAR_T>::feetPositionsOriginFrame()
    const {
  std::array<vector3d_t, 4> feetPositions;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    feetPositions[i] = footPositionOriginFrame(i);
  }
  return feetPositions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename KinematicsModelBase<SCALAR_T>::geometric_jacobian_t KinematicsModelBase<SCALAR_T>::FromBaseJacobianToInertiaJacobian(
    const matrix3d_t& i_R_b, const vector3d_t& b_r_point, const Eigen::Matrix<SCALAR_T, 6, JOINT_COORDINATE_SIZE>& b_J_point) {
  geometric_jacobian_t i_J_point;
  // rotation
  i_J_point.template topRows<3>() << i_R_b, matrix3d_t::Zero(), i_R_b * b_J_point.template topRows<3>();
  // translation
  i_J_point.template bottomRows<3>() << -i_R_b * switched_model::CrossProductMatrix(b_r_point), i_R_b,
      i_R_b * b_J_point.template bottomRows<3>();
  return i_J_point;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename KinematicsModelBase<SCALAR_T>::vector3d_t KinematicsModelBase<SCALAR_T>::FromBaseVelocityToInertiaVelocity(
    const matrix3d_t& i_R_b, const base_coordinate_t& baseLocalVelocities, const vector3d_t& b_r_point, const vector3d_t& b_v_point) {
  vector3d_t i_v_point =
      i_R_b * (b_v_point + baseLocalVelocities.template tail<3>() + baseLocalVelocities.template head<3>().cross(b_r_point));
  return i_v_point;
}

}  // end of namespace switched_model
