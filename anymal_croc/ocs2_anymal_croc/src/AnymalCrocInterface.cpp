//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_croc/AnymalCrocInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_croc_switched_model/core/AnymalCrocCom.h>
#include <ocs2_anymal_croc_switched_model/core/AnymalCrocKinematics.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedPointfootInterface> getAnymalCrocInterface(const std::string& taskFolder) {
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalCrocKinematics();
  auto kinAd = AnymalCrocKinematicsAd();
  auto com = AnymalCrocCom();
  auto comAd = AnymalCrocComAd();
  return std::unique_ptr<switched_model::QuadrupedPointfootInterface>(
      new switched_model::QuadrupedPointfootInterface(kin, kinAd, com, comAd, taskFolder));
}

std::string getTaskFileFolderCroc(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_croc") + "/config/" + taskName;
}

std::string getTaskFilePathCroc(const std::string& taskName) {
  return getTaskFileFolderCroc(taskName) + "/task.info";
}

}  // namespace anymal
