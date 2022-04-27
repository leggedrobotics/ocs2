//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_mpc/AnymalInterface.h"

#include <ros/package.h>

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>
#include <ocs2_quadruped_interface/QuadrupedWheeledInterface.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder,
                                                                       bool wheels) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  return getAnymalInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"), wheels);
}

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings, bool wheels) {
  auto kin = getAnymalKinematics(urdf);
  auto kinAd = getAnymalKinematicsAd(urdf);
  auto com = getAnymalComModel(urdf);
  auto comAd = getAnymalComModelAd(urdf);

  if (wheels) {
    return std::unique_ptr<switched_model::QuadrupedInterface>(
        new switched_model::QuadrupedWheeledInterface(*kin, *kinAd, *com, *comAd, std::move(settings)));
  } else {
    return std::unique_ptr<switched_model::QuadrupedInterface>(
        new switched_model::QuadrupedPointfootInterface(*kin, *kinAd, *com, *comAd, std::move(settings)));
  }
}

std::string getConfigFolder(const std::string& configName) {
  return ros::package::getPath("ocs2_anymal_mpc") + "/config/" + configName;
}

std::string getTaskFilePath(const std::string& configName) {
  return getConfigFolder(configName) + "/task.info";
}

}  // namespace anymal
