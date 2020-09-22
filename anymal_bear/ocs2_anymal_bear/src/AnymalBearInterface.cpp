//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_bear/AnymalBearInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_models/AnymalModels.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedPointfootInterface> getAnymalBearInterface(const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = getAnymalKinematics(AnymalModel::Bear);
  auto kinAd = getAnymalKinematicsAd(AnymalModel::Bear);
  auto com = getAnymalComModel(AnymalModel::Bear);
  auto comAd = getAnymalComModelAd(AnymalModel::Bear);
  return std::unique_ptr<switched_model::QuadrupedPointfootInterface>(
      new switched_model::QuadrupedPointfootInterface(*kin, *kinAd, *com, *comAd, taskFolder));
}

std::string getTaskFileFolderBear(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_bear") + "/config/" + taskName;
}

std::string getTaskFilePathBear(const std::string& taskName) {
  return getTaskFileFolderBear(taskName) + "/task.info";
}

}  // namespace anymal
