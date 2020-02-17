//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface);

}