//
// Created by rgrandia on 22.09.20.
//

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/QuadrupedInverseKinematics.h>
#include <ocs2_anymal_models/QuadrupedKinematics.h>
#include <ocs2_anymal_models/package_path.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <unordered_map>

namespace anymal {

std::string toString(AnymalModel model) {
  static const std::unordered_map<AnymalModel, std::string> map{
      {AnymalModel::Camel, "camel"}};
  return map.at(model);
}

AnymalModel stringToAnymalModel(const std::string& name) {
  static const std::unordered_map<std::string, AnymalModel> map{
      {"camel", AnymalModel::Camel}};
  return map.at(name);
}

std::string getUrdfPath(AnymalModel model) {
  switch (model) {
    case AnymalModel::Camel:
      return getPath() + "/urdf/anymal_camel_rsl.urdf";
    default:
      throw std::runtime_error("[AnymalModels] no default urdf available");
  }
}

std::string getUrdfString(const std::string& urdfPath) {
  std::ifstream stream(urdfPath.c_str());
  if (!stream) {
    throw std::runtime_error("File " + urdfPath + " does not exist");
  }

  std::string xml_str((std::istreambuf_iterator<char>(stream)),
                      std::istreambuf_iterator<char>());
  return xml_str;
}

std::string getUrdfString(AnymalModel model) {
  const auto path = getUrdfPath(model);
  return getUrdfString(path);
}

std::unique_ptr<switched_model::InverseKinematicsModelBase>
getAnymalInverseKinematics(const FrameDeclaration& frameDeclaration,
                           const std::string& urdf) {
  return std::unique_ptr<switched_model::InverseKinematicsModelBase>(
      new QuadrupedInverseKinematics(
          frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>
getAnymalKinematics(const FrameDeclaration& frameDeclaration,
                    const std::string& urdf) {
  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(
      new QuadrupedKinematics(frameDeclaration,
                              ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>
getAnymalKinematicsAd(const FrameDeclaration& frameDeclaration,
                      const std::string& urdf) {
  return std::unique_ptr<
      switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(
      new QuadrupedKinematicsAd(
          frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(
    const FrameDeclaration& frameDeclaration, const std::string& urdf) {
  return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(
      new QuadrupedCom(frameDeclaration,
                       createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>
getAnymalComModelAd(const FrameDeclaration& frameDeclaration,
                    const std::string& urdf) {
  return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(
      new QuadrupedComAd(
          frameDeclaration,
          createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

}  // namespace anymal
