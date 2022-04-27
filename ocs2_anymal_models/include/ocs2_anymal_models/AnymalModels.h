//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

namespace anymal {

enum class AnymalModel { Cerberus, Chimera, Camel, Urdf };

std::string toString(AnymalModel model);

AnymalModel stringToAnymalModel(const std::string& name);

std::string getUrdfPath(AnymalModel model);
std::string getUrdfString(AnymalModel model);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(const std::string& urdf);

}  // namespace anymal
