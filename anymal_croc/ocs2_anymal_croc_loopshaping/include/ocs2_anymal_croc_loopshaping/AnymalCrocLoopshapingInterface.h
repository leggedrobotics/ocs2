//
// Created by rgrandia on 13.02.20.
//

#pragma once

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalCrocLoopshapingInterface(const std::string& taskName);

std::string getTaskFileFolderCrocLoopshaping(const std::string& taskName);

std::string getTaskFilePathCrocLoopshaping(const std::string& taskName);

}  // end of namespace anymal
