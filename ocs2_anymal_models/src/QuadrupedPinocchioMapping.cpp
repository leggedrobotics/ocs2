#include "ocs2_anymal_models/QuadrupedPinocchioMapping.h"

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace anymal {

QuadrupedPinocchioMapping::QuadrupedPinocchioMapping(const FrameDeclaration& frameDeclaration,
                                                     const ocs2::PinocchioInterface& pinocchioInterface) {
  for (int i = 0; i < switched_model::NUM_CONTACT_POINTS; ++i) {
    hipFrameIds_[i] = getBodyId(frameDeclaration.legs[i].root, pinocchioInterface);
    footFrameIds_[i] = getBodyId(frameDeclaration.legs[i].tip, pinocchioInterface);
  }
  ocs2JointNames_ = getJointNames(frameDeclaration);
  extractPinocchioJointNames(pinocchioInterface);
  extractFeetOrdering(pinocchioInterface);

  for (const auto& collision : frameDeclaration.collisions) {
    collisionLinkFrameIds_.push_back(getBodyId(collision.link, pinocchioInterface));
  }
  collisionDeclaration_ = frameDeclaration.collisions;
}

namespace {
template <typename SCALAR_T>
switched_model::joint_coordinate_s_t<SCALAR_T> getPinocchioJointVectorImpl(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::feet_array_t<size_t>& mapFeetOrderOcs2ToPinocchio) {
  switched_model::joint_coordinate_s_t<SCALAR_T> pinocchioJointPositions;
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
}  // namespace

switched_model::joint_coordinate_t QuadrupedPinocchioMapping::getPinocchioJointVector(
    const switched_model::joint_coordinate_t& jointPositions) const {
  return getPinocchioJointVectorImpl(jointPositions, mapFeetOrderOcs2ToPinocchio_);
}

switched_model::joint_coordinate_ad_t QuadrupedPinocchioMapping::getPinocchioJointVector(
    const switched_model::joint_coordinate_ad_t& jointPositions) const {
  return getPinocchioJointVectorImpl(jointPositions, mapFeetOrderOcs2ToPinocchio_);
}

size_t QuadrupedPinocchioMapping::getBodyId(const std::string& bodyName, const ocs2::PinocchioInterface& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  // Try as body first, to prevent a conflict when a body and joint have the same name
  if (model.existBodyName(bodyName)) {
    return model.getBodyId(bodyName);
  } else {
    // Try to take the joint frame if body does not exist
    if (model.existJointName(bodyName)) {
      return model.getFrameId(bodyName, pinocchio::JOINT);
    } else {
      throw std::runtime_error("[QuadrupedPinocchioMapping] Body " + bodyName + " does not exist.");
    }
  }
}

void QuadrupedPinocchioMapping::extractPinocchioJointNames(const ocs2::PinocchioInterface& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  std::vector<std::pair<std::string, size_t>> pinocchioJointIds;
  for (const auto& jointName : ocs2JointNames_) {
    if (!model.existJointName(jointName)) {
      throw std::runtime_error("[QuadrupedPinocchioMapping] Joint " + jointName + " does not exist.");
    } else {
      pinocchioJointIds.push_back({jointName, model.getJointId(jointName)});
    }
  }
  std::sort(pinocchioJointIds.begin(), pinocchioJointIds.end(),
            [](const std::pair<std::string, size_t>& lhs, const std::pair<std::string, size_t>& rhs) { return lhs.second < rhs.second; });

  for (const auto& jointPair : pinocchioJointIds) {
    pinocchioJointNames_.push_back(jointPair.first);
  }
}

void QuadrupedPinocchioMapping::extractFeetOrdering(const ocs2::PinocchioInterface& pinocchioInterface) {
  for (int i = 0; i < switched_model::NUM_CONTACT_POINTS; ++i) {
    size_t jointMapping =
        std::find(pinocchioJointNames_.begin(), pinocchioJointNames_.end(), ocs2JointNames_[3 * i]) - pinocchioJointNames_.begin();
    size_t pinocchioIdx = jointMapping / 3;
    mapFeetOrderOcs2ToPinocchio_[i] = pinocchioIdx;
    mapFeetOrderPinocchioToOcs2_[pinocchioIdx] = i;
  }
}

}  // namespace anymal
