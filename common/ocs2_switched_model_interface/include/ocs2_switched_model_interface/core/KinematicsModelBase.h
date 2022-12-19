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

  using joint_jacobian_t = Eigen::Matrix<SCALAR_T, 6, JOINT_COORDINATE_SIZE>;
  using joint_jacobian_block_t = Eigen::Matrix<SCALAR_T, 6, 3>;

  KinematicsModelBase() = default;

  virtual ~KinematicsModelBase() = default;

  virtual KinematicsModelBase<SCALAR_T>* clone() const = 0;

  virtual vector3_s_t<SCALAR_T> baseToLegRootInBaseFrame(size_t footIndex) const = 0;

  vector3_s_t<SCALAR_T> legRootInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose) const;

  matrix3_s_t<SCALAR_T> orientationLegRootToOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose) const;

  vector3_s_t<SCALAR_T> legRootVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const;

  vector3_s_t<SCALAR_T> legRootVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                     const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const;

  virtual vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(size_t footIndex,
                                                              const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  feet_array_t<vector3_s_t<SCALAR_T>> positionBaseToFeetInBaseFrame(const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  vector3_s_t<SCALAR_T> footPositionInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  feet_array_t<vector3_s_t<SCALAR_T>> feetPositionsInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePose,
                                                                 const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  joint_jacobian_t baseToFootJacobianInBaseFrame(size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual joint_jacobian_block_t baseToFootJacobianBlockInBaseFrame(size_t footIndex,
                                                                    const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  virtual matrix3_s_t<SCALAR_T> footOrientationInBaseFrame(size_t footIndex,
                                                           const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  matrix3_s_t<SCALAR_T> footOrientationInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose,
                                                     const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual vector3_s_t<SCALAR_T> footVelocityRelativeToBaseInBaseFrame(size_t footIndex,
                                                                      const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                      const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                  const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  vector3_s_t<SCALAR_T> footVelocityInFootFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  feet_array_t<vector3_s_t<SCALAR_T>> feetVelocitiesInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                                  const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  struct CollisionSphere {
    vector3_s_t<SCALAR_T> position;
    SCALAR_T radius;
  };
  std::vector<CollisionSphere> collisionSpheresInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                             const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  virtual std::vector<CollisionSphere> collisionSpheresInBaseFrame(const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;
};

extern template class KinematicsModelBase<scalar_t>;
extern template class KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
