//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_croc_loopshaping/AnymalCrocLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_croc/AnymalCrocInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalCrocLoopshapingInterface(
    const std::string& taskFolder) {
  auto quadrupedInterface = getAnymalCrocInterface(taskFolder);

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
