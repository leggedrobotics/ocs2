//
// Created by rgrandia on 25.03.22.
//

#include <pinocchio/fwd.hpp>

#include "ocs2_anymal_motion_tracking/ModelParentSkeleton.h"

#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_switched_model_interface/core/Rotations.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace switched_model {

template <>
ocs2::PinocchioInterfaceTpl<scalar_t> getPinnochioInterface<>(const std::string& urdf) {
  return ocs2::getPinocchioInterfaceFromUrdfFile(urdf);
}

template <>
ocs2::PinocchioInterfaceTpl<ad_scalar_t> getPinnochioInterface<ad_scalar_t>(const std::string& urdf) {
  return getPinnochioInterface<scalar_t>(urdf).toCppAd();
}

template <typename SCALAR_T>
std::vector<std::string> getAllFrames(const ocs2::PinocchioInterfaceTpl<SCALAR_T>& pinnochioInterface) {
  const auto& frames = pinnochioInterface.getModel().frames;
  std::vector<std::string> names;
  names.reserve(frames.size());
  for (const auto& frame : frames) {
    names.push_back(frame.name);
  }
  return names;
}

template std::vector<std::string> getAllFrames<scalar_t>(const ocs2::PinocchioInterfaceTpl<scalar_t>& pinnochioInterface);
template std::vector<std::string> getAllFrames<ad_scalar_t>(const ocs2::PinocchioInterfaceTpl<ad_scalar_t>& pinnochioInterface);

template <typename SCALAR_T>
ModelParentSkeleton<SCALAR_T>::ModelParentSkeleton(ocs2::PinocchioInterfaceTpl<SCALAR_T> pinnochioInterface,
                                                   const std::vector<std::string>& frames)
    : pinocchioInterface_(std::move(pinnochioInterface)) {
  const auto& model = pinocchioInterface_.getModel();
  for (const auto& name : frames) {
    frameIds_.push_back(model.getBodyId(name));
  }
  motionTargets_.resize(frameIds_.size());
}

template <typename SCALAR_T>
const std::vector<MotionTarget<SCALAR_T>>& ModelParentSkeleton<SCALAR_T>::update(const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& state,
                                                                                 const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input) {
  const auto basePose = getBasePose(state);
  const auto baseOrientation = getOrientation(basePose);
  const auto basePosition = getPositionInOrigin(basePose);
  const auto baseLocalTwist = getBaseLocalVelocities(state);
  const auto baseLocalLinear = getLinearVelocity(baseLocalTwist);
  const auto baseLocalAngular = getAngularVelocity(baseLocalTwist);

  // Base Quaternion
  const Eigen::Quaternion<SCALAR_T> baseQuat(Eigen::AngleAxis<SCALAR_T>(baseOrientation(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
                                             Eigen::AngleAxis<SCALAR_T>(baseOrientation(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
                                             Eigen::AngleAxis<SCALAR_T>(baseOrientation(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()));

  const auto q = mapJointConfigurationOcs2ToPinocchio(state.tail(JOINT_COORDINATE_SIZE));
  const auto v = mapJointConfigurationOcs2ToPinocchio(input.tail(JOINT_COORDINATE_SIZE));

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q, v);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  using Model = typename ocs2::PinocchioInterfaceTpl<SCALAR_T>::Model;

  for (int i = 0; i < frameIds_.size(); ++i) {
    const auto frameId = frameIds_[i];

    // Compute frame info
    const auto& transform = pinocchio::updateFramePlacement(model, data, frameId);
    const auto frameVelocity = pinocchio::getFrameVelocity(model, data, frameId, rf);

    motionTargets_[i].poseInWorld.position = basePosition + baseQuat * transform.translation();
    motionTargets_[i].poseInWorld.orientation = baseQuat.toRotationMatrix() * transform.rotation();

    motionTargets_[i].twistInWorld.linear =
        baseQuat * (baseLocalLinear + baseLocalAngular.cross(transform.translation()) + frameVelocity.linear());
    motionTargets_[i].twistInWorld.angular = baseQuat * (baseLocalAngular + frameVelocity.angular());
  }
  return motionTargets_;
}

template <typename SCALAR_T>

switched_model::joint_coordinate_s_t<SCALAR_T> ModelParentSkeleton<SCALAR_T>::mapJointConfigurationOcs2ToPinocchio(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions;
  feet_array_t<size_t> mapFeetOrderOcs2ToPinocchio{0, 2, 1, 3};
  // OCS2 LF
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio[0]) = jointPositions.template segment<3>(0);
  // OCS2 RF
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio[1]) = jointPositions.template segment<3>(3);
  // OCS2 LH
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio[2]) = jointPositions.template segment<3>(6);
  // OCS2 RH
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio[3]) = jointPositions.template segment<3>(9);
  return pinocchioJointPositions;
}

/* Explicit template instantiation for scalar_t and ad_scalar_t */
template class ModelParentSkeleton<scalar_t>;
template class ModelParentSkeleton<ad_scalar_t>;

}  // namespace switched_model