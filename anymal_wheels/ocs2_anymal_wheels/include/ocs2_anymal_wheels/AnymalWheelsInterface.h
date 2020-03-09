//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

namespace switched_model {

class WheeledQuadrupedInterface : public switched_model::QuadrupedInterface {
 public:
  using Base = switched_model::QuadrupedInterface;
  WheeledQuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                            const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder);

}; /* WheeledQuadrupedInterface */

}  // namespace switched_model

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalWheelsInterface(const std::string& taskName);

std::string getTaskFileFolderWheels(const std::string& taskName);

std::string getTaskFilePathWheels(const std::string& taskName);

}  // end of namespace anymal
