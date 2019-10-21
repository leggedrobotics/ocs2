/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 3, 2017
 *      Author: Farbod
 */

#include <iostream>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::update(const generalized_coordinate_t& generalizedCoordinate) {
  update(generalizedCoordinate.template head<6>(), generalizedCoordinate.template tail<JOINT_COORD_SIZE>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
template <typename BASE_COORDINATE, typename JOINT_COORDINATE>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::update(const Eigen::DenseBase<BASE_COORDINATE>& qBase,
                                                             const Eigen::DenseBase<JOINT_COORDINATE>& qJoint) {
  qBase_ = qBase;
  qJoint_ = qJoint;
  b_R_o_ = RotationMatrixOrigintoBase<SCALAR_T>(qBase_.template head<3>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::feetPositionsBaseFrame(std::array<vector3d_t, 4>& feetPositions) {
  for (size_t i = 0; i < 4; i++) footPositionBaseFrame(i, feetPositions[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::footPositionOriginFrame(const size_t& footIndex, vector3d_t& footPosition) {
  // calculate foot position in Base frame
  vector3d_t b_footPosition;
  footPositionBaseFrame(footIndex, b_footPosition);
  // calculate foot position in Origin frame
  footPosition = b_R_o_.transpose() * b_footPosition + qBase_.template tail<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
                                                                              const size_t& footIndex, vector3d_t& footPosition) {
  // update the class
  update(generalizedCoordinate);
  // calculate foot position in Origin frame
  footPositionOriginFrame(footIndex, footPosition);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::feetPositionsOriginFrame(std::array<vector3d_t, 4>& feetPositions) {
  for (size_t i = 0; i < 4; i++) footPositionOriginFrame(i, feetPositions[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
                                                                               std::array<vector3d_t, 4>& feetPositions) {
  // update the class
  update(generalizedCoordinate);
  // calculate feet's positions in Origin frame
  feetPositionsOriginFrame(feetPositions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::FromBaseJacobianToInertiaJacobian(
    const matrix3d_t& i_R_b, const vector3d_t& b_r_point, const Eigen::Matrix<SCALAR_T, 6, JOINT_COORD_SIZE>& b_J_point,
    Eigen::Matrix<SCALAR_T, 6, JOINT_COORD_SIZE + 6>& i_J_point) {
  // rotation
  i_J_point.template topRows<3>() << i_R_b, matrix3d_t::Zero(), i_R_b * b_J_point.template topRows<3>();
  // translation
  i_J_point.template bottomRows<3>() << -i_R_b * switched_model::CrossProductMatrix(b_r_point), i_R_b,
      i_R_b * b_J_point.template bottomRows<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::FromBaseVelocityToInertiaVelocity(const matrix3d_t& i_R_b,
                                                                                        const base_coordinate_t& baseLocalVelocities,
                                                                                        const vector3d_t& b_r_point,
                                                                                        const vector3d_t& b_v_point,
                                                                                        vector3d_t& i_v_point) {
  // point velocities in the inertia frame
  i_v_point = i_R_b * (b_v_point + baseLocalVelocities.template tail<3>() + baseLocalVelocities.template head<3>().cross(b_r_point));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
typename KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::vector3d_t KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::ToEulerAngle(
    const Eigen::Quaternion<SCALAR_T>& q) {
  vector3d_t rollPitchYaw;

  // roll (x-axis rotation)
  SCALAR_T sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  SCALAR_T cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  rollPitchYaw(0) = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  SCALAR_T sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    rollPitchYaw(1) = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    rollPitchYaw(1) = asin(sinp);

  // yaw (z-axis rotation)
  SCALAR_T siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  SCALAR_T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  rollPitchYaw(2) = atan2(siny_cosp, cosy_cosp);

  return rollPitchYaw;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
const typename KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::matrix3d_t&
KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T>::rotationMatrixOrigintoBase() const {
  return b_R_o_;
}

}  // end of namespace switched_model
