//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_wheels_switched_model/constraint/AnymalWheelsComKinoConstraintAd.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h>
#include <memory>

namespace anymal {
WheeledQuadrupedInterface::WheeledQuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                                     const com_model_t& comModel, const ad_com_model_t& adComModel,
                                                     const std::string& pathToConfigFolder)
    : Base(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  constraintsPtr_.reset(new wheels_constraint_t(
      adKinematicModel, adComModel, std::static_pointer_cast<Base::logic_rules_t>(Base::getLogicRulesPtr()), Base::modelSettings()));
}

std::unique_ptr<anymal::WheeledQuadrupedInterface> getAnymalWheelsInterface(const std::string& taskName) {
  std::string taskFolder = getTaskFileFolderWheels(taskName);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalWheelsKinematics();
  auto kinAd = AnymalWheelsKinematicsAd();
  auto com = AnymalWheelsCom();
  auto comAd = AnymalWheelsComAd();
  return std::unique_ptr<anymal::WheeledQuadrupedInterface>(new anymal::WheeledQuadrupedInterface(kin, kinAd, com, comAd, taskFolder));
}

std::string getTaskFileFolderWheels(const std::string& taskName) {
  std::string taskFolder = ros::package::getPath("ocs2_anymal_wheels") + "/config/" + taskName;
  return taskFolder;
}

std::string getTaskFilePathWheels(const std::string& taskName) {
  return getTaskFileFolderWheels(taskName) + "/task.info";
}

}  // namespace anymal
