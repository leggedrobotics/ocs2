#pragma once

#include <array>
#include <string>
#include <vector>

#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_anymal_models/QuadrupedPinocchioMapping.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class QuadrupedKinematics final : public switched_model::KinematicsModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef switched_model::KinematicsModelBase<SCALAR_T> BASE;
  using typename BASE::CollisionSphere;
  using typename BASE::joint_jacobian_block_t;
  using typename BASE::joint_jacobian_t;

  QuadrupedKinematics(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);
  ~QuadrupedKinematics() = default;

  QuadrupedKinematics<SCALAR_T>* clone() const override;

  switched_model::vector3_s_t<SCALAR_T> baseToLegRootInBaseFrame(size_t footIndex) const override;

  switched_model::vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  joint_jacobian_block_t baseToFootJacobianBlockInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  switched_model::matrix3_s_t<SCALAR_T> footOrientationInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  switched_model::vector3_s_t<SCALAR_T> footVelocityRelativeToBaseInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities) const override;

  std::vector<CollisionSphere> collisionSpheresInBaseFrame(
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

 private:
  QuadrupedKinematics(const QuadrupedKinematics& rhs);

  using PinocchioInterface_s_t = ocs2::PinocchioInterfaceTpl<SCALAR_T>;

  template <typename T = SCALAR_T, typename std::enable_if<std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface_s_t castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface.toCppAd();
  }

  template <typename T = SCALAR_T, typename std::enable_if<!std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface_s_t castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface;
  }

  switched_model::vector3_s_t<SCALAR_T> relativeTranslationInBaseFrame(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                       std::size_t frame) const;
  switched_model::matrix3_s_t<SCALAR_T> relativeOrientationInBaseFrame(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                       std::size_t frame) const;

  std::unique_ptr<PinocchioInterface_s_t> pinocchioInterfacePtr_;
  QuadrupedPinocchioMapping pinocchioMapping_;
  switched_model::feet_array_t<switched_model::vector3_s_t<SCALAR_T>> baseToLegRootInBaseFrame_;
};

}  // namespace tpl

using QuadrupedKinematics = tpl::QuadrupedKinematics<ocs2::scalar_t>;
using QuadrupedKinematicsAd = tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::QuadrupedKinematics<ocs2::scalar_t>;
extern template class anymal::tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
