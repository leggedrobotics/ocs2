#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_anymal_models/FrameDeclaration.h>

namespace anymal {

/**
 * Used to map joint configuration space from OCS2 to Pinocchio. In OCS2, the feet order is {LF, RF, LH, RH}. But in Pinocchio, the feet
 * order depends on the URDF.
 */
class QuadrupedPinocchioMapping {
 public:
  QuadrupedPinocchioMapping(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);

  switched_model::joint_coordinate_t getPinocchioJointVector(const switched_model::joint_coordinate_t& jointPositions) const;

  switched_model::joint_coordinate_ad_t getPinocchioJointVector(const switched_model::joint_coordinate_ad_t& jointPositions) const;

  size_t getPinocchioFootIndex(size_t ocs2FootIdx) const { return mapFeetOrderOcs2ToPinocchio_[ocs2FootIdx]; }

  size_t getFootFrameId(size_t ocs2FootIdx) const { return footFrameIds_[ocs2FootIdx]; }

  size_t getHipFrameId(size_t ocs2FootIdx) const { return hipFrameIds_[ocs2FootIdx]; }

  const std::vector<size_t>& getCollisionLinkFrameIds() const { return collisionLinkFrameIds_; }
  const std::vector<CollisionDeclaration>& getCollisionDeclaration() const { return collisionDeclaration_; }

  const std::vector<std::string>& getOcs2JointNames() const { return ocs2JointNames_; }

  const std::vector<std::string>& getPinocchioJointNames() const { return pinocchioJointNames_; }

  static size_t getBodyId(const std::string& bodyName, const ocs2::PinocchioInterface& pinocchioInterface);

 private:
  void extractPinocchioJointNames(const ocs2::PinocchioInterface& pinocchioInterface);
  void extractFeetOrdering(const ocs2::PinocchioInterface& pinocchioInterface);

  // Frame Ids
  switched_model::feet_array_t<size_t> hipFrameIds_;
  switched_model::feet_array_t<size_t> footFrameIds_;

  // Collisions
  std::vector<size_t> collisionLinkFrameIds_;
  std::vector<CollisionDeclaration> collisionDeclaration_;

  // Feet ordering
  switched_model::feet_array_t<size_t> mapFeetOrderOcs2ToPinocchio_;
  switched_model::feet_array_t<size_t> mapFeetOrderPinocchioToOcs2_;

  // Frame names
  std::vector<std::string> ocs2JointNames_;
  std::vector<std::string> pinocchioJointNames_;
};

}  // namespace anymal
