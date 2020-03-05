//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalCrocInterface(const std::string& taskName);

std::string getTaskFileFolderCroc(const std::string& taskName);

std::string getTaskFilePathCroc(const std::string& taskName);

}  // end of namespace anymal
