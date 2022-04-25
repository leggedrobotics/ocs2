//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

namespace anymal {

enum class AnymalModel { Bear, Cerberus, Chimera, Camel, Wheels, WheelsChimera };

std::string toString(AnymalModel model);

AnymalModel stringToAnymalModel(const std::string& name);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(AnymalModel model);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(AnymalModel model);

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(AnymalModel model);

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(AnymalModel model);

}  // namespace anymal
