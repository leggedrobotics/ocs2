//
// Created by rgrandia on 13.02.20.
//

#pragma once

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalWheelsLoopshapingInterface(
    const std::string& taskFolder);

std::string getTaskFileFolderAnymalWheelsLoopshaping(const std::string& taskName);

std::string getTaskFilePathAnymalWheelsLoopshaping(const std::string& taskName);

}  // end of namespace anymal
