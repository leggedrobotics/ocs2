//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_wheels_loopshaping/AnymalWheelsLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalWheelsLoopshapingInterface(
    const std::string& taskName) {
  std::string taskFolder = getTaskFileFolderAnymalWheelsLoopshaping(taskName);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalWheelsKinematics();
  auto kinAd = AnymalWheelsKinematicsAd();
  auto com = AnymalWheelsCom();
  auto comAd = AnymalWheelsComAd();

  auto quadrupedInterface =
      std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedInterface(kin, kinAd, com, comAd, taskFolder));

  return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
      new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), taskFolder));
}

std::string getTaskFileFolderAnymalWheelsLoopshaping(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_wheels_loopshaping") + "/config/" + taskName;
}

std::string getTaskFilePathAnymalWheelsLoopshaping(const std::string& taskName) {
  return getTaskFileFolderAnymalWheelsLoopshaping(taskName) + "/task.info";
}

}  // end of namespace anymal
