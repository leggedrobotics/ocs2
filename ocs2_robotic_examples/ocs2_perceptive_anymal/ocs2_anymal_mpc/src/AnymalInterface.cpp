//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_mpc/AnymalInterface.h"

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  return getAnymalInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                            frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
}

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration) {
  std::unique_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};
  if (settings.modelSettings_.analyticalInverseKinematics_) {
    invKin = getAnymalInverseKinematics(frameDeclaration, urdf);
  }
  auto kin = getAnymalKinematics(frameDeclaration, urdf);
  auto kinAd = getAnymalKinematicsAd(frameDeclaration, urdf);
  auto com = getAnymalComModel(frameDeclaration, urdf);
  auto comAd = getAnymalComModelAd(frameDeclaration, urdf);
  auto jointNames = getJointNames(frameDeclaration);
  auto baseName = frameDeclaration.root;

  return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedPointfootInterface(
      *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
}

std::string getConfigFolder(const std::string& configName) {
  return ament_index_cpp::get_package_share_directory("ocs2_anymal_mpc") + "/config/" + configName;
}

std::string getTaskFilePath(const std::string& configName) {
  return getConfigFolder(configName) + "/task.info";
}

}  // namespace anymal
