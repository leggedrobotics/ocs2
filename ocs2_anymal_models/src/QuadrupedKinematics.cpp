#include "ocs2_anymal_models/QuadrupedKinematics.h"

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const ocs2::PinocchioInterface& pinocchioInterface,
                                                   const QuadrupedPinocchioMappingTpl<SCALAR_T>& pinnochioMapping)
    : pinocchioInterfacePtr_(new PinocchioInterface(castPinocchioInterface(pinocchioInterface))), pinocchioMapping_(pinnochioMapping) {
  // Frame index mapping
  auto checkAndSetIndex = [this](std::size_t footIndex, const FrameIndex frameIndex, const std::string& name) {
    const auto& model = pinocchioInterfacePtr_->getModel();
    if (model.existBodyName(name)) {
      mapFrameIndexToId_[footIndex].setId(frameIndex, model.getBodyId(name));
    } else {
      throw std::runtime_error("[QuadrupedKinematics] Body " + name + " does not exist.");
    }
  };
  for (std::size_t i = 0; i < switched_model::NUM_CONTACT_POINTS; ++i) {
    checkAndSetIndex(i, FrameIndex::HAA, switched_model::feetNames[i] + "_HAA");
    checkAndSetIndex(i, FrameIndex::FOOT, switched_model::feetNames[i] + "_FOOT");
    checkAndSetIndex(i, FrameIndex::KFE, switched_model::feetNames[i] + "_KFE");
  }
}

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const QuadrupedKinematics& rhs)
    : pinocchioInterfacePtr_(new PinocchioInterface(*rhs.pinocchioInterfacePtr_)),
      pinocchioMapping_(rhs.pinocchioMapping_),
      mapFrameIndexToId_(rhs.mapFrameIndexToId_){};

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>* QuadrupedKinematics<SCALAR_T>::clone() const {
  return new QuadrupedKinematics<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::baseToLegRootInBaseFrame(size_t footIndex) const {
  switched_model::joint_coordinate_s_t<SCALAR_T> zeroConfiguration;
  zeroConfiguration.setZero();
  pinocchio::FrameIndex frameId = mapFrameIndexToId_[footIndex].getId(FrameIndex::HAA);
  return relativeTranslationInBaseFrame(zeroConfiguration, frameId);
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  pinocchio::FrameIndex frameId = mapFrameIndexToId_[footIndex].getId(FrameIndex::FOOT);
  const auto pinocchioJointPositions = pinocchioMapping_.mapJointOcs2ToPinocchio(jointPositions);
  return relativeTranslationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    std::size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const -> joint_jacobian_block_t {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();

  const auto pinocchioJointPositions = pinocchioMapping_.mapJointOcs2ToPinocchio(jointPositions);
  pinocchio::FrameIndex frameId = mapFrameIndexToId_[footIndex].getId(FrameIndex::FOOT);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  joint_jacobian_t J;
  pinocchio::computeFrameJacobian(model, data, pinocchioJointPositions, frameId, rf, J);

  // In Pinocchio, the first three coordinates of J represent linear part and the last three coordinates represent angular part. But it
  // is the other way round in OCS2. Swap linear and angular parts to stick to the originally used OCS2's convention.
  joint_jacobian_block_t res;
  res.template block<3, 3>(0, 0) = J.template block<3, 3>(3, pinocchioMapping_.mapFootIdxOcs2ToPinocchio(footIndex) * 3u);
  res.template block<3, 3>(3, 0) = J.template block<3, 3>(0, pinocchioMapping_.mapFootIdxOcs2ToPinocchio(footIndex) * 3u);

  return res;
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  pinocchio::FrameIndex frameId = mapFrameIndexToId_[footIndex].getId(FrameIndex::FOOT);
  const auto pinocchioJointPositions = pinocchioMapping_.mapJointOcs2ToPinocchio(jointPositions);
  return relativeOrientationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::collisionSpheresInBaseFrame(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const
    -> std::vector<CollisionSphere> {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto pinocchioJointPositions = pinocchioMapping_.mapJointOcs2ToPinocchio(jointPositions);
  pinocchio::forwardKinematics(model, data, pinocchioJointPositions);

  const switched_model::vector3_s_t<SCALAR_T> kneeOffsetInKneeFrame{SCALAR_T(-0.055), SCALAR_T(0.0), SCALAR_T(0.0)};
  std::vector<CollisionSphere> collisionSpheres;
  const SCALAR_T kneeRadius(0.08);

  for (size_t i = 0; i < switched_model::NUM_CONTACT_POINTS; i++) {
    pinocchio::FrameIndex frameId = mapFrameIndexToId_[i].getId(FrameIndex::KFE);
    const auto& transformation = pinocchio::updateFramePlacement(model, data, frameId);
    collisionSpheres.push_back({transformation.act(kneeOffsetInKneeFrame), kneeRadius});
  }

  return collisionSpheres;
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::relativeTranslationInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, pinocchio::FrameIndex frame) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  pinocchio::forwardKinematics(model, data, jointPositions);

  return pinocchio::updateFramePlacement(model, data, frame).translation();
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::relativeOrientationInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, pinocchio::FrameIndex frame) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  pinocchio::forwardKinematics(model, data, jointPositions);

  return pinocchio::updateFramePlacement(model, data, frame).rotation();
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::QuadrupedKinematics<ocs2::scalar_t>;
template class anymal::tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
