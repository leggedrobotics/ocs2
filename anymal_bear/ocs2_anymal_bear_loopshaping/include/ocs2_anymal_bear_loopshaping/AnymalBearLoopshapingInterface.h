//
// Created by rgrandia on 13.02.20.
//

#pragma once

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalBearLoopshapingInterface(const std::string& taskName);

std::string getTaskFileFolderBearLoopshaping(const std::string& taskName);

std::string getTaskFilePathBearLoopshaping(const std::string& taskName);

}  // end of namespace anymal
