/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/core/ComModelBase.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

comkino_input_t weightCompensatingInputs(const ComModelBase<scalar_t>& comModel, const contact_flag_t& contactFlags,
                                         const vector3_t& baseOrientation) {
  return weightCompensatingInputs(comModel.totalMass(), contactFlags, baseOrientation);
}

comkino_input_t weightCompensatingInputs(scalar_t mass, const contact_flag_t& contactFlags, const vector3_t& baseOrientation) {
  const int numStanceLegs = numberOfClosedContacts(contactFlags);

  comkino_input_t inputs = comkino_input_t::Zero();
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = mass * 9.81;
    const vector3_t forceInWorld = vector3_t{0.0, 0.0, totalWeight / numStanceLegs};
    const vector3_t forceInBase = rotateVectorOriginToBase(forceInWorld, baseOrientation);

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }

  return inputs;
}

template class ComModelBase<scalar_t>;
template class ComModelBase<ocs2::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
