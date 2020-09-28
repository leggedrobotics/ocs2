//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include <ocs2_anymal_models/AnymalModels.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalInterface(AnymalModel model, const std::string& taskFolder);

std::string getConfigFolder(const std::string& configName);

std::string getTaskFilePath(const std::string& configName);

}  // end of namespace anymal
