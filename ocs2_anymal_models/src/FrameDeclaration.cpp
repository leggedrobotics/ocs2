//
// Created by rgrandia on 27.04.22.
//

#include "ocs2_anymal_models/FrameDeclaration.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

namespace anymal {

std::vector<std::string> getJointNames(const FrameDeclaration& frameDeclaration) {
  std::vector<std::string> jointNames;
  for (const auto& leg : frameDeclaration.legs) {
    jointNames.insert(jointNames.end(), leg.joints.begin(), leg.joints.end());
  }
  return jointNames;
}

LimbFrames limbFramesFromFile(const std::string& file, const std::string& field) {
  LimbFrames frames;
  ocs2::loadData::loadCppDataType(file, field + ".root", frames.root);
  ocs2::loadData::loadCppDataType(file, field + ".tip", frames.tip);
  ocs2::loadData::loadStdVector(file, field + ".joints", frames.joints, false);
  return frames;
}

FrameDeclaration frameDeclarationFromFile(const std::string& file) {
  FrameDeclaration decl;
  ocs2::loadData::loadCppDataType(file, "root", decl.root);
  decl.legs[0] = limbFramesFromFile(file, "left_front");
  decl.legs[1] = limbFramesFromFile(file, "right_front");
  decl.legs[2] = limbFramesFromFile(file, "left_hind");
  decl.legs[3] = limbFramesFromFile(file, "right_hind");

  std::vector<std::pair<std::string, ocs2::scalar_t>> collisionSpheres;
  ocs2::loadData::loadStdVectorOfPair(file, "collisions.collisionSpheres", collisionSpheres, false);

  ocs2::matrix_t offsets(collisionSpheres.size(), 3);
  ocs2::loadData::loadEigenMatrix(file, "collisions.collisionOffsets", offsets);

  decl.collisions.reserve(collisionSpheres.size());
  for (int i = 0; i < collisionSpheres.size(); ++i) {
    CollisionDeclaration collisionDeclaration;
    collisionDeclaration.link = collisionSpheres[i].first;
    collisionDeclaration.radius = collisionSpheres[i].second;
    collisionDeclaration.offset = offsets.row(i).transpose();
    decl.collisions.push_back(std::move(collisionDeclaration));
  }

  return decl;
}

}  // namespace anymal