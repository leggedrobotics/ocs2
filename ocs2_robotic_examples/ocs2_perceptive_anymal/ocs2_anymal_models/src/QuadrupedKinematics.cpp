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
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const FrameDeclaration& frameDeclaration,
                                                   const ocs2::PinocchioInterface& pinocchioInterface)
    : pinocchioInterfacePtr_(new PinocchioInterface_s_t(castPinocchioInterface(pinocchioInterface))),
      pinocchioMapping_(frameDeclaration, pinocchioInterface) {
  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    switched_model::joint_coordinate_s_t<SCALAR_T> zeroConfiguration;
    zeroConfiguration.setZero();
    baseToLegRootInBaseFrame_[footIndex] = relativeTranslationInBaseFrame(zeroConfiguration, pinocchioMapping_.getHipFrameId(footIndex));
  }
}

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const QuadrupedKinematics& rhs)
    : pinocchioInterfacePtr_(new PinocchioInterface_s_t(*rhs.pinocchioInterfacePtr_)),
      pinocchioMapping_(rhs.pinocchioMapping_),
      baseToLegRootInBaseFrame_(rhs.baseToLegRootInBaseFrame_){};

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>* QuadrupedKinematics<SCALAR_T>::clone() const {
  return new QuadrupedKinematics<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::baseToLegRootInBaseFrame(size_t footIndex) const {
  return baseToLegRootInBaseFrame_[footIndex];
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);
  const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
  return relativeTranslationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    std::size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const -> joint_jacobian_block_t {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();

  const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
  pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);

  joint_jacobian_t J;
  pinocchio::computeFrameJacobian(model, data, pinocchioJointPositions, frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

  // In Pinocchio, the first three coordinates of J represent linear part and the last three coordinates represent angular part. But it
  // is the other way round in OCS2. Swap linear and angular parts to stick to the originally used OCS2's convention.
  joint_jacobian_block_t res;
  res.template block<3, 3>(0, 0) = J.template block<3, 3>(3, pinocchioMapping_.getPinocchioFootIndex(footIndex) * 3u);
  res.template block<3, 3>(3, 0) = J.template block<3, 3>(0, pinocchioMapping_.getPinocchioFootIndex(footIndex) * 3u);

  return res;
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);
  const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
  return relativeOrientationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::footVelocityRelativeToBaseInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();

  const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
  const auto pinocchioJointVelocities = pinocchioMapping_.getPinocchioJointVector(jointVelocities);

  pinocchio::forwardKinematics(model, data, pinocchioJointPositions, pinocchioJointVelocities);

  pinocchio::FrameIndex frameId = pinocchioMapping_.getFootFrameId(footIndex);

  return getFrameVelocity(model, data, frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::collisionSpheresInBaseFrame(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const
    -> std::vector<CollisionSphere> {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto pinocchioJointPositions = pinocchioMapping_.getPinocchioJointVector(jointPositions);
  pinocchio::forwardKinematics(model, data, pinocchioJointPositions);

  const auto& linkFrames = pinocchioMapping_.getCollisionLinkFrameIds();
  const auto& decl = pinocchioMapping_.getCollisionDeclaration();

  std::vector<CollisionSphere> collisionSpheres;
  collisionSpheres.reserve(linkFrames.size());
  for (int i = 0; i < linkFrames.size(); ++i) {
    const auto& transformation = pinocchio::updateFramePlacement(model, data, linkFrames[i]);
    const switched_model::vector3_s_t<SCALAR_T> offset = decl[i].offset.cast<SCALAR_T>();
    collisionSpheres.push_back({transformation.act(offset), SCALAR_T(decl[i].radius)});
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
