/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class SwitchedModelCostBase : public ocs2::QuadraticCostFunction {
 public:
  using com_model_t = ComModelBase<scalar_t>;

  //! Constructor
  SwitchedModelCostBase(const com_model_t& comModel, const SwitchedModelModeScheduleManager& modeScheduleManager, const state_matrix_t& Q,
                        const input_matrix_t& R);

  //! Destructor
  ~SwitchedModelCostBase() override = default;

  //! clone SwitchedModelCostBase class.
  SwitchedModelCostBase* clone() const override;

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

 protected:
  //! Copy constructor
  SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

 private:
  void inputFromContactFlags(const contact_flag_t& contactFlags, const state_vector_t& nominalState, vector_t& inputs);

  std::unique_ptr<com_model_t> comModelPtr_;
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
};

}  // end of namespace switched_model
