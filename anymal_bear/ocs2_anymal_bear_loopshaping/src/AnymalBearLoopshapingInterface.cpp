//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_bear_switched_model/core/AnymalBearCom.h>
#include <ocs2_anymal_bear_switched_model/core/AnymalBearKinematics.h>

namespace anymal {

std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getAnymalBearLoopshapingInterface(const std::string& taskName) {
  std::string taskFolder = ros::package::getPath("ocs2_anymal_bear_loopshaping") + "/config/" + taskName;
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalBearKinematics();
  auto kinAd = AnymalBearKinematicsAd();
  auto com = AnymalBearCom();
  auto comAd = AnymalBearComAd();

  auto quadrupedInterface =
      std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedInterface(kin, kinAd, com, comAd, taskFolder));

  return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
      new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), taskFolder));
}

}  // end of namespace anymal
