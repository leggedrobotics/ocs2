#include "ocs2_anymal_models/QuadrupedKinematics.h"

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// Urdf
#include <urdf_parser/urdf_parser.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
QuadrupedKinematics<SCALAR_T>::QuadrupedKinematics(const std::string& urdfFile) : mapFeetOrderOcs2ToPinocchio_{0, 2, 1, 3} {
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[AnymalKinematicsInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[AnymalKinematicsInterface] URDF file not found: " + urdfFilePath.string());
  }
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile)));

  // Frame index mapping
  auto checkAndSetIndex = [this](std::size_t footIndex, const FrameIndex frameIndex, const std::string& name) {
    const auto& model = pinocchioInterfacePtr_->getModel();
    if (model.existFrame(name)) {
      mapFrameIndexToId_[footIndex].setId(frameIndex, model.getFrameId(name));
    } else {
      throw std::runtime_error("[QuadrupedKinematics] Frame " + name + " does not exist.");
    }
  };
  for (std::size_t i = 0; i < switched_model::NUM_CONTACT_POINTS; ++i) {
    checkAndSetIndex(i, FrameIndex::HAA, switched_model::feetNames[i] + "_HAA");
    checkAndSetIndex(i, FrameIndex::FOOT, switched_model::feetNames[i] + "_FOOT");
    checkAndSetIndex(i, FrameIndex::KFE, switched_model::feetNames[i] + "_KFE");
  }
}
// template <typename SCALAR_T>
// QuadrupedKinematics<SCALAR_T>* QuadrupedKinematics<SCALAR_T>::clone() const {
//   return new QuadrupedKinematics<SCALAR_T>(*this);
// }

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::createPinocchioInterface(const std::string& urdfFile) -> PinocchioInterface {
  // parse the URDF
  const auto urdfTreePtr = ::urdf::parseURDFFile(urdfFile);
  pinocchio::ModelTpl<SCALAR_T> model;
  pinocchio::urdf::buildModel(urdfTreePtr, model);
  return PinocchioInterface(model, urdfTreePtr);
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
  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions = mapJointConfigurationOcs2ToPinocchio(jointPositions);
  return relativeTranslationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    std::size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const -> joint_jacobian_block_t {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();

  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions = mapJointConfigurationOcs2ToPinocchio(jointPositions);
  pinocchio::FrameIndex frameId = mapFrameIndexToId_[footIndex].getId(FrameIndex::FOOT);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  joint_jacobian_t J;
  pinocchio::computeFrameJacobian(model, data, pinocchioJointPositions, frameId, rf, J);

  // In Pinocchio, the first three coordinates of J represent linear part and the last three coordinates represent angular part. But it
  // is the other way round in RobCoGen. Swap linear and angular parts to stick to the originally used RobCoGen's convention.
  J.topRows(3).swap(J.bottomRows(3));

  return J.template block<6, 3>(0, mapFeetOrderOcs2ToPinocchio_[footIndex] * 3u);
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  pinocchio::FrameIndex frameId = mapFrameIndexToId_[footIndex].getId(FrameIndex::FOOT);
  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions = mapJointConfigurationOcs2ToPinocchio(jointPositions);
  return relativeOrientationInBaseFrame(pinocchioJointPositions, frameId);
}

template <typename SCALAR_T>
auto QuadrupedKinematics<SCALAR_T>::collisionSpheresInBaseFrame(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const
    -> std::vector<CollisionSphere> {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions = mapJointConfigurationOcs2ToPinocchio(jointPositions);
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
switched_model::joint_coordinate_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::mapJointConfigurationOcs2ToPinocchio(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions;
  // OCS2 LF
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[0]) = jointPositions.template segment<3>(0);
  // OCS2 RF
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[1]) = jointPositions.template segment<3>(3);
  // OCS2 LH
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[2]) = jointPositions.template segment<3>(6);
  // OCS2 RH
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[3]) = jointPositions.template segment<3>(9);

  return pinocchioJointPositions;
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::relativeTranslationInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, const pinocchio::FrameIndex frame) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  pinocchio::forwardKinematics(model, data, jointPositions);

  return pinocchio::updateFramePlacement(model, data, frame).translation();
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> QuadrupedKinematics<SCALAR_T>::relativeOrientationInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, const pinocchio::FrameIndex frame) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  pinocchio::forwardKinematics(model, data, jointPositions);

  return pinocchio::updateFramePlacement(model, data, frame).rotation();
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::QuadrupedKinematics<ocs2::scalar_t>;
// template class anymal::tpl::QuadrupedKinematics<ocs2::ad_scalar_t>;
