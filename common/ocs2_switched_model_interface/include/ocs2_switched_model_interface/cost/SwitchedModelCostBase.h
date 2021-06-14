/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_core/cost/CostFunctionBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/cost/MotionTrackingCost.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class SwitchedModelCostBase : public ocs2::CostFunctionBase {
 public:
  using com_model_t = ComModelBase<scalar_t>;
  using kinematic_model_t = KinematicsModelBase<ocs2::scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>;

  //! Constructor
  SwitchedModelCostBase(const MotionTrackingCost::Weights& trackingWeights, const SwitchedModelModeScheduleManager& modeScheduleManager,
                        const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                        bool recompile);

  //! Destructor
  ~SwitchedModelCostBase() override = default;

  //! clone SwitchedModelCostBase class.
  SwitchedModelCostBase* clone() const override;

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

  //! No final cost for events.
  scalar_t finalCost(scalar_t t, const vector_t& x) override { return 0.0; }
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override {
    return ScalarFunctionQuadraticApproximation::Zero(x.size(), 0);
  }

 protected:
  //! Copy constructor
  SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

 private:
  std::unique_ptr<MotionTrackingCost> trackingCostPtr_;
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
};

}  // end of namespace switched_model
