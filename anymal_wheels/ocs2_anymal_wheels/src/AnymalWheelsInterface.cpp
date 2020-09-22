//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_models/AnymalModels.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedWheeledInterface> getAnymalWheelsInterface(const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = getAnymalKinematics(AnymalModel::Wheels);
  auto kinAd = getAnymalKinematicsAd(AnymalModel::Wheels);
  auto com = getAnymalComModel(AnymalModel::Wheels);
  auto comAd = getAnymalComModelAd(AnymalModel::Wheels);
  return std::unique_ptr<switched_model::QuadrupedWheeledInterface>(
      new switched_model::QuadrupedWheeledInterface(*kin, *kinAd, *com, *comAd, taskFolder));
}

std::string getTaskFileFolderWheels(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_wheels") + "/config/" + taskName;
}

std::string getTaskFilePathWheels(const std::string& taskName) {
  return getTaskFileFolderWheels(taskName) + "/task.info";
}

}  // namespace anymal
