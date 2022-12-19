//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/FrameDeclaration.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf, const std::string& taskFolder);

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(const std::string& urdf,
                                                                       switched_model::QuadrupedInterface::Settings settings,
                                                                       const FrameDeclaration& frameDeclaration);

std::string getConfigFolder(const std::string& configName);

std::string getTaskFilePath(const std::string& configName);

}  // end of namespace anymal
