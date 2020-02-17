//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_croc_loopshaping/AnymalCrocLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_croc_switched_model/core/AnymalCrocCom.h>
#include <ocs2_anymal_croc_switched_model/core/AnymalCrocKinematics.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedLoopshapingInterface> getAnymalCrocLoopshapingInterface(const std::string& taskName) {
  std::string taskFolder = ros::package::getPath("ocs2_anymal_croc_loopshaping") + "/config/" + taskName;
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalCrocKinematics();
  auto kinAd = AnymalCrocKinematicsAd();
  auto com = AnymalCrocCom();
  auto comAd = AnymalCrocComAd();

  auto quadrupedInterface =
      std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedInterface(kin, kinAd, com, comAd, taskFolder));

  return std::unique_ptr<switched_model::QuadrupedLoopshapingInterface>(
      new switched_model::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), taskFolder));
}

}  // end of namespace anymal
