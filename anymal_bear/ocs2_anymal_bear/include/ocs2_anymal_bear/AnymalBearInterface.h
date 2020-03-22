//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedPointfootInterface> getAnymalBearInterface(const std::string& taskName);

std::string getTaskFileFolderBear(const std::string& taskName);

std::string getTaskFilePathBear(const std::string& taskName);

}  // end of namespace anymal