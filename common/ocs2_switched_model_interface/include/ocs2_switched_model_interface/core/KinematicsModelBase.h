/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 2, 2017
 *      Author: Farbod
 */

#pragma once

#include <array>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * ModelKinematics Base Class
 * @tparam SCALAR_T
 */
template <typename SCALAR_T>
class KinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using geometric_jacobian_t = Eigen::Matrix<SCALAR_T, 6, GENERALIZED_COORDINATE_SIZE>;
  using joint_jacobian_t = Eigen::Matrix<SCALAR_T, 6, JOINT_COORDINATE_SIZE>;

  KinematicsModelBase() = default;

  virtual ~KinematicsModelBase() = default;

  virtual KinematicsModelBase<SCALAR_T>* clone() const = 0;

  virtual vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(size_t footIndex,
                                                              const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  std::array<vector3_s_t<SCALAR_T>, NUM_CONTACT_POINTS> positionBaseToFeetInBaseFrame(
      const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  vector3_s_t<SCALAR_T> footPositionInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T> basePoseInOriginFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  std::array<vector3_s_t<SCALAR_T>, NUM_CONTACT_POINTS> feetPositionsInOriginFrame(
      const base_coordinate_s_t<SCALAR_T> basePose, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual joint_jacobian_t baseToFootJacobianInBaseFrame(size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  vector3_s_t<SCALAR_T> footVelocityRelativeToBaseInBaseFrame(size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                              const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T> baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T> basePoseInOriginFrame,
                                                  const base_coordinate_s_t<SCALAR_T> baseTwistInBaseFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  std::array<vector3_s_t<SCALAR_T>, NUM_CONTACT_POINTS> feetVelocitiesInOriginFrame(
      const base_coordinate_s_t<SCALAR_T> basePoseInOriginFrame, const base_coordinate_s_t<SCALAR_T> baseTwistInBaseFrame,
      const joint_coordinate_s_t<SCALAR_T>& jointPositions, const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;
};

extern template class KinematicsModelBase<double>;
extern template class KinematicsModelBase<ocs2::CppAdInterface<double>::ad_scalar_t>;

}  // end of namespace switched_model
