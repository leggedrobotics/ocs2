//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_mpc/AnymalInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalLoopshapingInterface(AnymalModel model,
                                                                                                         const std::string& configFolder) {
  auto quadrupedInterface = getAnymalInterface(model, configFolder);

  // load loopshaping
  auto loopshapingDefinition = ocs2::loopshaping_property_tree::load(configFolder + "/loopshaping.info");
  loopshapingDefinition->costMatrix() = quadrupedInterface->nominalCostApproximation().dfduu;
  loopshapingDefinition->print();

  return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
      new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), std::move(loopshapingDefinition)));
}

std::string getConfigFolderLoopshaping(const std::string& configName) {
  return ros::package::getPath("ocs2_anymal_loopshaping_mpc") + "/config/" + configName;
}

std::string getTaskFilePathLoopshaping(const std::string& configName) {
  return getConfigFolderLoopshaping(configName) + "/task.info";
}

}  // end of namespace anymal
