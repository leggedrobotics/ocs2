//
// Created by rgrandia on 13.02.20.
//

#pragma once

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalLoopshapingInterface(const std::string& urdf,
                                                                                                         const std::string& configFolder);

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalLoopshapingInterface(
    const std::string& urdf, switched_model::QuadrupedInterface::Settings settings, const FrameDeclaration& frameDeclaration,
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition);

std::string getConfigFolderLoopshaping(const std::string& configName);

std::string getTaskFilePathLoopshaping(const std::string& configName);

}  // end of namespace anymal
