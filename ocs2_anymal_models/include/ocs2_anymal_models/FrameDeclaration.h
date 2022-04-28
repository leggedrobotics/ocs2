//
// Created by rgrandia on 27.04.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace anymal {

struct CollisionDeclaration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string link;
  switched_model::scalar_t radius;
  switched_model::vector3_t offset;
};

struct LimbFrames {
  std::string root;
  std::string tip;
  std::vector<std::string> joints;
};

struct FrameDeclaration {
  std::string root;
  switched_model::feet_array_t<LimbFrames> legs;
  std::vector<CollisionDeclaration> collisions;
};

std::vector<std::string> getJointNames(const FrameDeclaration& frameDeclaration);

LimbFrames limbFramesFromFile(const std::string& file, const std::string& field);

FrameDeclaration frameDeclarationFromFile(const std::string& file);

}  // namespace anymal