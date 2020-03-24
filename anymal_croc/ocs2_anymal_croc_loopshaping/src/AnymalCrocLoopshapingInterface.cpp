//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_croc_loopshaping/AnymalCrocLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>

#include <ocs2_anymal_croc_switched_model/core/AnymalCrocCom.h>
#include <ocs2_anymal_croc_switched_model/core/AnymalCrocKinematics.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalCrocLoopshapingInterface(const std::string& taskName) {
  std::string taskFolder = getTaskFileFolderCrocLoopshaping(taskName);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalCrocKinematics();
  auto kinAd = AnymalCrocKinematicsAd();
  auto com = AnymalCrocCom();
  auto comAd = AnymalCrocComAd();

  auto quadrupedInterface = std::unique_ptr<switched_model::QuadrupedPointfootInterface>(
      new switched_model::QuadrupedPointfootInterface(kin, kinAd, com, comAd, taskFolder));

  return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
      new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), taskFolder));
}

std::string getTaskFileFolderCrocLoopshaping(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_croc_loopshaping") + "/config/" + taskName;
}

std::string getTaskFilePathCrocLoopshaping(const std::string& taskName) {
  return getTaskFileFolderCrocLoopshaping(taskName) + "/task.info";
}

}  // end of namespace anymal
