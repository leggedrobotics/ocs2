//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_wheels_loopshaping/AnymalWheelsLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_wheels/AnymalWheelsInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalWheelsLoopshapingInterface(
    const std::string& taskFolder) {
  auto quadrupedInterface = getAnymalWheelsInterface(taskFolder);

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
