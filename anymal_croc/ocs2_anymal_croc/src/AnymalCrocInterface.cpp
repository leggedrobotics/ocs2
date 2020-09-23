//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_croc/AnymalInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_models/AnymalModels.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedPointfootInterface> getAnymalCrocInterface(const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = getAnymalKinematics(AnymalModel::Croc);
  auto kinAd = getAnymalKinematicsAd(AnymalModel::Croc);
  auto com = getAnymalComModel(AnymalModel::Croc);
  auto comAd = getAnymalComModelAd(AnymalModel::Croc);
  return std::unique_ptr<switched_model::QuadrupedPointfootInterface>(
      new switched_model::QuadrupedPointfootInterface(*kin, *kinAd, *com, *comAd, taskFolder));
}

std::string getTaskFileFolderCroc(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_croc") + "/config/" + taskName;
}

std::string getTaskFilePathCroc(const std::string& taskName) {
  return getTaskFileFolderCroc(taskName) + "/task.info";
}

}  // namespace anymal
