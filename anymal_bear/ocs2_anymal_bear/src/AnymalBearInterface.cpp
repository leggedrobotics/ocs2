//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_bear/AnymalBearInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_bear_switched_model/core/AnymalBearCom.h>
#include <ocs2_anymal_bear_switched_model/core/AnymalBearKinematics.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalBearInterface(const std::string& taskName) {
  const auto taskFolder = getTaskFileFolderBear(taskName);
  auto kin = AnymalBearKinematics();
  auto kinAd = AnymalBearKinematicsAd();
  auto com = AnymalBearCom();
  auto comAd = AnymalBearComAd();
  return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedInterface(kin, kinAd, com, comAd, taskFolder));
}

std::string getTaskFileFolderBear(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_bear") + "/config/" + taskName;
}

std::string getTaskFilePathBear(const std::string& taskName) {
  return getTaskFileFolderBear(taskName) + "/task.info";
}

}  // namespace anymal
