/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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

  vector_t inputs = vector_t::Zero(centroidalModelInfo.inputDim);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = totalMass * 9.81;
    //  TODO
    //  const matrix3_t b_R_o = rotationMatrixOriginToBase(baseOrientation);
    //  const vector3_t forceInBase = b_R_o * vector3_t{0.0, 0.0, totalWeight / numStanceLegs};
    vector_t forceInBase(3);
    forceInBase << 0.0, 0.0, totalWeight / numStanceLegs;

    for (size_t i = 0; i < centroidalModelInfo.numThreeDofContacts; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }

  return inputs;
}

}  // namespace legged_robot
}  // namespace ocs2
