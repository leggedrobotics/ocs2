//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_mpc/AnymalInterface.h"

#include <ros/package.h>

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>
#include <ocs2_quadruped_interface/QuadrupedWheeledInterface.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(AnymalModel model, const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = getAnymalKinematics(model);
  auto kinAd = getAnymalKinematicsAd(model);
  auto com = getAnymalComModel(model);
  auto comAd = getAnymalComModelAd(model);

  switch (model) {
    case AnymalModel::Bear:
    case AnymalModel::Cerberus:
    case AnymalModel::Chimera:
    case AnymalModel::Chip:
      return std::unique_ptr<switched_model::QuadrupedInterface>(
          new switched_model::QuadrupedPointfootInterface(*kin, *kinAd, *com, *comAd, taskFolder));
    case AnymalModel::Wheels:
    case AnymalModel::WheelsChimera:
      return std::unique_ptr<switched_model::QuadrupedInterface>(
          new switched_model::QuadrupedWheeledInterface(*kin, *kinAd, *com, *comAd, taskFolder));
    default:
      throw std::runtime_error("[getAnymalInterface] unkown model");
  }
}

std::string getConfigFolder(const std::string& configName) {
  return ros::package::getPath("ocs2_anymal_mpc") + "/config/" + configName;
}

std::string getTaskFilePath(const std::string& configName) {
  return getConfigFolder(configName) + "/task.info";
}

}  // namespace anymal
