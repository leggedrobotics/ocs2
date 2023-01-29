//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/InverseKinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

#include "ocs2_anymal_models/FrameDeclaration.h"

namespace anymal {

enum class AnymalModel { Camel };

std::string toString(AnymalModel model);

AnymalModel stringToAnymalModel(const std::string& name);

std::string getUrdfPath(AnymalModel model);
std::string getUrdfString(AnymalModel model);

std::unique_ptr<switched_model::InverseKinematicsModelBase> getAnymalInverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getAnymalKinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getAnymalKinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getAnymalComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getAnymalComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf);

}  // namespace anymal
