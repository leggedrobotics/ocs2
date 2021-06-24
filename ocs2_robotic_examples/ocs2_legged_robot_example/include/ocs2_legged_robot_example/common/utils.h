#pragma once

#include <Eigen/Dense>
#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include "ocs2_legged_robot_example/common/definitions.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Count contact feet */
inline size_t numberOfClosedContacts(const contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline vector_t weightCompensatingInputs(const scalar_t& totalMass, const contact_flag_t& contactFlags
                                         /*, const vector3_t& baseOrientation*/) {
  const auto numStanceLegs = numberOfClosedContacts(contactFlags);

  vector_t inputs = vector_t::Zero(INPUT_DIM_);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = totalMass * 9.81;
    //  const matrix3_t b_R_o = rotationMatrixOriginToBase(baseOrientation);
    //  const vector3_t forceInBase = b_R_o * vector3_t{0.0, 0.0, totalWeight / numStanceLegs};
    const vector3_t forceInBase = vector3_t{0.0, 0.0, totalWeight / numStanceLegs};

    for (size_t i = 0; i < FOOT_CONTACTS_NUM_; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }

  return inputs;
}

}  // namespace legged_robot
}  // namespace ocs2
