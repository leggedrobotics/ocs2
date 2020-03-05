//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h>
#include <ocs2_anymal_wheels_switched_model/constraint/AnymalWheelsComKinoConstraintAd.h>

namespace switched_model
{
  WheeledQuadrupedInterface::WheeledQuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
      const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder
    ) : Base(kinematicModel, adKinematicModel, comModel,  adComModel, pathToConfigFolder)
    {
      constraintsPtr_.reset(new anymal::AnymalWheelsComKinoConstraintAd(adKinematicModel, adComModel, logicRulesPtr_, modelSettings_));
      costFunctionPtr_.reset(new cost_function_t(*comModelPtr_, logicRulesPtr_, Q_, R_, QFinal_));
      operatingPointsPtr_.reset(new operating_point_t(*comModelPtr_, logicRulesPtr_));
      timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings_));
    }


} /* switched_model */

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalWheelsInterface(const std::string& taskName) {
  std::string taskFolder = getTaskFileFolderWheels(taskName);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  AnymalWheelsKinematics kin{};
  AnymalWheelsKinematicsAd kinAd{};
  AnymalWheelsCom com{};
  AnymalWheelsComAd comAd{};
  return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::WheeledQuadrupedInterface(kin, kinAd, com, comAd, taskFolder));
}

std::string getTaskFileFolderWheels(const std::string& taskName) {
  std::string taskFolder = ros::package::getPath("ocs2_anymal_wheels") + "/config/" + taskName;
  return taskFolder;
}

std::string getTaskFilePathWheels(const std::string& taskName) {
  return getTaskFileFolderWheels(taskName) + "/task.info";
}

}  // namespace anymal
