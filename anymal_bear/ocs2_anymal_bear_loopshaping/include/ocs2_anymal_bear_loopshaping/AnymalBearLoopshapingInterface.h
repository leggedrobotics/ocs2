//
// Created by rgrandia on 13.02.20.
//

#pragma once

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedLoopshapingInterface> getAnymalBearLoopshapingInterface(const std::string& taskName);

}  // end of namespace anymal
