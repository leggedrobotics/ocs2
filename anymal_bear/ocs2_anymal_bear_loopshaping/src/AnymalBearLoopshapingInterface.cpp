//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>

#include <ocs2_anymal_bear/AnymalBearInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalBearLoopshapingInterface(
    const std::string& taskFolder) {
  auto quadrupedInterface = getAnymalBearInterface(taskFolder);

  return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
      new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), taskFolder));
}

std::string getTaskFileFolderBearLoopshaping(const std::string& taskName) {
  return ros::package::getPath("ocs2_anymal_bear_loopshaping") + "/config/" + taskName;
}

std::string getTaskFilePathBearLoopshaping(const std::string& taskName) {
  return getTaskFileFolderBearLoopshaping(taskName) + "/task.info";
}

}  // end of namespace anymal
