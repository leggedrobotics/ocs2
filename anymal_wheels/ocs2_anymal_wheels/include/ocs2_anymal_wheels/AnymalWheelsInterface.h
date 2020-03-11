//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>
#include <ocs2_anymal_wheels_switched_model/constraint/AnymalWheelsComKinoConstraintAd.h>

namespace anymal {

class WheeledQuadrupedInterface : public switched_model::QuadrupedInterface {
 public:
   using Base = switched_model::QuadrupedInterface;
   // using wheels_constraint_t = switched_model::AnymalWheelsComKinoConstraintAd;

   /**
    *
    * @param kinematicModel
    * @param comModel
    * @param pathToConfigFolder : Reads settings from the task.info in this folder
    */
   WheeledQuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
       const ad_com_model_t& adComModel, const std::string& pathToConfigFolder);

   /**
    * Destructor
    */
   ~WheeledQuadrupedInterface() override = default;

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

 private:
  std::unique_ptr<constraint_t> constraintsPtr_;

}; /* WheeledQuadrupedInterface */


std::unique_ptr<anymal::WheeledQuadrupedInterface> getAnymalWheelsInterface(const std::string& taskName);

std::string getTaskFileFolderWheels(const std::string& taskName);

std::string getTaskFilePathWheels(const std::string& taskName);

}  // end of namespace anymal
