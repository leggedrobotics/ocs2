/*
 * AnymalWeightCompensationForces.h
 *
 *  Created on: Nov 24, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/misc/WeightCompensationForces.h>

namespace anymal {

class AnymalWeightCompensationForces final : public switched_model::WeightCompensationForces<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef switched_model::WeightCompensationForces<12> Base;

  AnymalWeightCompensationForces();

  AnymalWeightCompensationForces(const AnymalWeightCompensationForces& rhs);

  ~AnymalWeightCompensationForces() {}
};

}  // namespace anymal
