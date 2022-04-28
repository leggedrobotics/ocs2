//
// Created by rgrandia on 22.09.20.
//

#include <ocs2_anymal_models/AnymalModels.h>

#include <unordered_map>

#include <ros/package.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/QuadrupedKinematics.h>
#include <ocs2_anymal_models/package_path.h>

namespace anymal {

std::string toString(AnymalModel model) {
  static const std::unordered_map<AnymalModel, std::string> map{
      {AnymalModel::Cerberus, "cerberus"}, {AnymalModel::Chimera, "chimera"}, {AnymalModel::Camel, "camel"}};
  return map.at(model);
}

AnymalModel stringToAnymalModel(const std::string& name) {
  static const std::unordered_map<std::string, AnymalModel> map{
      {"cerberus", AnymalModel::Cerberus}, {"chimera", AnymalModel::Chimera}, {"camel", AnymalModel::Camel}};
  return map.at(name);
}

std::string getUrdfPath(AnymalModel model) {
  switch (model) {
    case AnymalModel::Cerberus:
      return getPath() + "/urdf/anymal_cerberus_rsl.urdf";
    case AnymalModel::Chimera:
      return getPath() + "/urdf/anymal_chimera_rsl.urdf";
    case AnymalModel::Camel:
      return getPath() + "/urdf/anymal_camel_rsl.urdf";
    case AnymalModel::Urdf:
      throw std::runtime_error("[AnymalModels] no default urdf available");
    default:
      throw std::runtime_error("[AnymalModels] unkown model");
  }
}

std::string getUrdfString(AnymalModel model) {
  const auto path = getUrdfPath(model);
  std::ifstream stream(path.c_str());
  if (!stream) {
    throw std::runtime_error("File " + path + " does not exist");
  }

  std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  return xml_str;
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(const std::string& urdf) {
  //  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(
  //      new QuadrupedKinematics(ocs2::getPinocchioInterfaceFromUrdfString(urdf), QuadrupedMapping({0, 2, 1, 3})));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(const std::string& urdf) {
  //  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(
  //      new QuadrupedKinematicsAd(ocs2::getPinocchioInterfaceFromUrdfString(urdf), QuadrupedMappingAd({0, 2, 1, 3})));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(const std::string& urdf) {
  //  return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(
  //      new QuadrupedCom(createQuadrupedPinocchioInterfaceFromUrdfString(urdf), QuadrupedMapping({0, 2, 1, 3})));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(const std::string& urdf) {
  //  return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(
  //      new QuadrupedComAd(createQuadrupedPinocchioInterfaceFromUrdfString(urdf), QuadrupedMappingAd({0, 2, 1, 3})));
}

}  // namespace anymal
