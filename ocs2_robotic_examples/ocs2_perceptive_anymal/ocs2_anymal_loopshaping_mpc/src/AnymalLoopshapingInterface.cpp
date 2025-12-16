//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h"

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_anymal_mpc/AnymalInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalLoopshapingInterface(const std::string& urdf,
                                                                                                         const std::string& configFolder) {
  return getAnymalLoopshapingInterface(urdf, switched_model::loadQuadrupedSettings(configFolder + "/task.info"),
                                       frameDeclarationFromFile(configFolder + "/frame_declaration.info"),
                                       ocs2::loopshaping_property_tree::load(configFolder + "/loopshaping.info"));
}

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalLoopshapingInterface(
    const std::string& urdf, switched_model::QuadrupedInterface::Settings settings, const FrameDeclaration& frameDeclaration,
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition) {
  auto quadrupedInterface = getAnymalInterface(urdf, std::move(settings), frameDeclaration);
  loopshapingDefinition->costMatrix() = quadrupedInterface->nominalCostApproximation().dfduu;
  loopshapingDefinition->print();

  return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
      new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), std::move(loopshapingDefinition)));
}

std::string getConfigFolderLoopshaping(const std::string& configName) {
  return ament_index_cpp::get_package_share_directory("ocs2_anymal_loopshaping_mpc") + "/config/" + configName;
}

std::string getTaskFilePathLoopshaping(const std::string& configName) {
  return getConfigFolderLoopshaping(configName) + "/task.info";
}

}  // end of namespace anymal
